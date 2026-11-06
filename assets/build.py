"""Compiles a YAML map into engine-independent box primitives."""
import argparse
import json
import math
from pathlib import Path
import sys

from jsonschema import Draft202012Validator
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = ROOT / "spec" / "map.schema.yaml"

class UniqueKeyLoader(yaml.SafeLoader):
  def construct_mapping(self, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
      key = self.construct_object(key_node, deep=deep)
      if not isinstance(key, str):
        raise ValueError("YAML mapping keys must be strings")
      if key in mapping:
        raise ValueError(f"Duplicate YAML key: {key}")
      mapping[key] = self.construct_object(value_node, deep=deep)
    return mapping

def _check_finite(value):
  if isinstance(value, float) and not math.isfinite(value):
    raise ValueError("Map numbers must be finite")
  if isinstance(value, dict):
    for child in value.values():
      _check_finite(child)
  elif isinstance(value, list):
    for child in value:
      _check_finite(child)
def compile_map(source: Path) -> dict:
  """Reads and validates a map; returns REP-103 primitives or raises ValueError."""
  try:
    spec = yaml.load(source.read_text(encoding="utf-8"), Loader=UniqueKeyLoader)
  except yaml.YAMLError as error:
    raise ValueError(f"Invalid YAML: {error}") from error
  schema = yaml.safe_load(SCHEMA_PATH.read_text(encoding="utf-8"))
  _check_finite(spec)
  error = next(Draft202012Validator(schema).iter_errors(spec), None)
  if error:
    location = ".".join(str(part) for part in error.absolute_path) or "map"
    raise ValueError(f"{location}: {error.message}")
  from assets.map_geometry import expand
  primitives, annotations = expand(spec)
  result = {"version": 1, "id": spec["id"], "frame": "rep103", "primitives": primitives, "annotations": annotations}
  _check_finite(result)
  return result
def main() -> int:
  """Compiles a map to JSON, returning a nonzero status on invalid input."""
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("source", type=Path)
  parser.add_argument("--output", type=Path, help="Defaults to sim/generated/<map id>.json")
  args = parser.parse_args()
  try:
    result = compile_map(args.source)
    output = args.output or ROOT / "sim" / "generated" / f"{result['id']}.json"
    contents = json.dumps(result, indent=2, allow_nan=False) + "\n"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(contents, encoding="utf-8", newline="\n")
  except (OSError, ValueError) as error:
    print(f"Map compilation failed: {error}", file=sys.stderr)
    return 1
  print(f"Compiled {len(result['primitives'])} primitives to {output}")
  return 0

if __name__ == "__main__":
  raise SystemExit(main())
