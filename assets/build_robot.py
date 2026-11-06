"""Compiles a robot YAML spec into a validated REP-103 JSON configuration."""
import argparse
import json
from pathlib import Path
import sys

from jsonschema import Draft202012Validator
import yaml

from assets.build import ROOT, UniqueKeyLoader

SCHEMA_PATH = ROOT / "spec" / "robot.schema.yaml"

def compile_robot(source: Path) -> dict:
  """Returns a validated robot configuration; invalid specs raise ValueError."""
  try:
    spec = yaml.load(source.read_text(encoding="utf-8"), Loader=UniqueKeyLoader)
  except yaml.YAMLError as error:
    raise ValueError(f"Invalid YAML: {error}") from error
  schema = yaml.safe_load(SCHEMA_PATH.read_text(encoding="utf-8"))
  error = next(Draft202012Validator(schema).iter_errors(spec), None)
  if error:
    location = ".".join(str(part) for part in error.absolute_path) or "robot"
    raise ValueError(f"{location}: {error.message}")
  json.dumps(spec, allow_nan=False)
  if spec["body"]["z_max"] <= spec["body"]["z_min"]:
    raise ValueError("body.z_max must exceed body.z_min")
  for axis in ("linear", "angular"):
    if spec[f"min_{axis}_velocity"] >= spec[f"max_{axis}_velocity"]:
      raise ValueError(f"min_{axis}_velocity must be less than max_{axis}_velocity")
  lidar = spec["lidar"]
  if lidar["angle_max"] <= lidar["angle_min"]:
    raise ValueError("lidar.angle_max must exceed lidar.angle_min")
  if lidar["range_max"] <= lidar["range_min"]:
    raise ValueError("lidar.range_max must exceed lidar.range_min")
  for field in spec["provisional"]:
    value = spec
    for part in field.split("."):
      if not isinstance(value, dict) or part not in value:
        raise ValueError(f"Unknown provisional field: {field}")
      value = value[part]
  return {**spec, "frame": "rep103"}
def main() -> int:
  """Writes a compiled robot spec; failures leave existing output unchanged."""
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("source", type=Path)
  parser.add_argument("--output", type=Path, help="Defaults to sim/generated/robots/<id>.json")
  args = parser.parse_args()
  try:
    result = compile_robot(args.source)
    output = args.output or ROOT / "sim" / "generated" / "robots" / f"{result['id']}.json"
    contents = json.dumps(result, indent=2, allow_nan=False) + "\n"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(contents, encoding="utf-8", newline="\n")
  except (OSError, ValueError) as error:
    print(f"Robot compilation failed: {error}", file=sys.stderr)
    return 1
  print(f"Compiled robot {result['id']} to {output}")
  return 0

if __name__ == "__main__":
  raise SystemExit(main())
