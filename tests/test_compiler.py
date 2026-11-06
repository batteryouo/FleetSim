import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml
from jsonschema import Draft202012Validator

from assets.build import ROOT, SCHEMA_PATH, compile_map

SOURCE = ROOT / "spec" / "maps" / "test_room.yaml"

class CompilerTests(unittest.TestCase):
  def setUp(self):
    self.workspace = tempfile.TemporaryDirectory(prefix="cadence-compiler-")
    self.addCleanup(self.workspace.cleanup)
    self.source = Path(self.workspace.name) / "map.yaml"
    self.spec = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))
  def _compile(self, spec):
    self.source.write_text(yaml.safe_dump(spec), encoding="utf-8")
    return compile_map(self.source)
  def test_schema_is_valid(self):
    Draft202012Validator.check_schema(yaml.safe_load(SCHEMA_PATH.read_text(encoding="utf-8")))
  def test_compiles_metric_primitives_without_frame_conversion(self):
    result = compile_map(SOURCE)
    self.assertEqual(result["frame"], "rep103")
    self.assertEqual(result["primitives"][0], {
      "id": "north", "type": "box", "center": [1.0, 2.1],
      "size": [10.4, 0.2], "yaw": 0.0, "z_min": 0.0, "z_max": 2.5,
    })
    rotated = result["primitives"][-1]
    self.assertEqual(rotated["center"], [12.0, 4.0])
    self.assertEqual(rotated["size"], [4.0, 0.2])
    self.assertEqual(rotated["z_min"], 0.4)
    self.assertEqual(rotated["z_max"], 1.6)
    self.assertEqual(rotated["yaw"], self.spec["walls"][-1]["yaw"])
  def test_height_endpoints_survive_json_without_arithmetic(self):
    for z_min, z_max in [(0.4, 1.6), (0.1, 0.3), (-0.4, 0.4), (1000.1, 1000.2)]:
      with self.subTest(z_min=z_min, z_max=z_max):
        self.spec["walls"][-1].update(z_min=z_min, z_max=z_max)
        primitive = json.loads(json.dumps(self._compile(self.spec)))["primitives"][-1]
        self.assertEqual(primitive["z_min"], z_min)
        self.assertEqual(primitive["z_max"], z_max)
        self.assertEqual(len(primitive["center"]), 2)
        self.assertEqual(len(primitive["size"]), 2)
  def test_generated_map_matches_source(self):
    generated = ROOT / "sim" / "generated" / "test_room.json"
    self.assertEqual(json.loads(generated.read_text(encoding="utf-8")), compile_map(SOURCE))
  def test_output_is_deterministic_and_has_no_source_path(self):
    first = self._compile(self.spec)
    second = self._compile(self.spec)
    self.assertEqual(first, second)
    self.assertNotIn(str(self.source), json.dumps(first))
  def test_rejects_invalid_geometry(self):
    for field, value in [
      ("size", [0, 1]), ("size", [-1, 1]), ("size", [1]),
      ("center", [1, 2, 3]), ("center", [True, 0]),
      ("yaw", float("nan")), ("yaw", float("inf")),
      ("z_min", 2.5), ("z_min", 3),
    ]:
      with self.subTest(field=field, value=value):
        spec = copy.deepcopy(self.spec)
        spec["walls"][0][field] = value
        with self.assertRaises(ValueError):
          self._compile(spec)
  def test_rejects_invalid_contract(self):
    variants = []
    for field, value in [("version", 0), ("version", False), ("id", "../outside"), ("walls", [])]:
      spec = copy.deepcopy(self.spec)
      spec[field] = value
      variants.append(spec)
    unknown_field = copy.deepcopy(self.spec)
    unknown_field["walls"][0]["height"] = 3
    variants.append(unknown_field)
    duplicate_id = copy.deepcopy(self.spec)
    duplicate_id["walls"][1]["id"] = duplicate_id["walls"][0]["id"]
    variants.append(duplicate_id)
    missing_field = copy.deepcopy(self.spec)
    del missing_field["walls"][0]["yaw"]
    variants.extend([missing_field, None, [], "not a map"])
    for spec in variants:
      with self.subTest(spec=spec):
        with self.assertRaises(ValueError):
          self._compile(spec)
  def test_rejects_duplicate_yaml_keys(self):
    self.source.write_text("version: 0\nversion: 1\n", encoding="utf-8")
    with self.assertRaisesRegex(ValueError, "Duplicate YAML key"):
      compile_map(self.source)
  def test_invalid_cli_input_does_not_overwrite_output(self):
    self.source.write_text("version: 99\n", encoding="utf-8")
    output = Path(self.workspace.name) / "output.json"
    output.write_text("existing output\n", encoding="utf-8")
    result = subprocess.run(
      [sys.executable, "-m", "assets.build", str(self.source), "--output", str(output)],
      cwd=ROOT, capture_output=True, text=True, timeout=30,
    )
    self.assertEqual(result.returncode, 1)
    self.assertIn("Map compilation failed", result.stderr)
    self.assertEqual(output.read_text(encoding="utf-8"), "existing output\n")

if __name__ == "__main__":
  unittest.main()
