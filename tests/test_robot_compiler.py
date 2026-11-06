import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from jsonschema import Draft202012Validator
import yaml

from assets.build_robot import ROOT, SCHEMA_PATH, compile_robot

SOURCE = ROOT / "spec" / "robots" / "a100.yaml"

class RobotCompilerTests(unittest.TestCase):
  def setUp(self):
    self.workspace = tempfile.TemporaryDirectory(prefix="cadence-robot-")
    self.addCleanup(self.workspace.cleanup)
    self.source = Path(self.workspace.name) / "robot.yaml"
    self.spec = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))
  def _compile(self, spec):
    self.source.write_text(yaml.safe_dump(spec), encoding="utf-8")
    return compile_robot(self.source)
  def _cli(self, source, output):
    return subprocess.run(
      [sys.executable, "-m", "assets.build_robot", str(source), "--output", str(output)],
      cwd=ROOT, capture_output=True, text=True, timeout=30,
    )
  def test_schema_is_valid(self):
    Draft202012Validator.check_schema(yaml.safe_load(SCHEMA_PATH.read_text(encoding="utf-8")))
  def test_compiles_explicit_rep103_values_and_provisional_fields(self):
    self.assertEqual(compile_robot(SOURCE), {**self.spec, "frame": "rep103"})
    self.assertEqual(self.spec["footprint"], {"type": "circle", "radius": 0.27})
    self.assertEqual(self.spec["body"], {"z_min": 0.0, "z_max": 0.25})
    self.assertEqual(self.spec["provisional"], ["max_linear_acceleration", "max_angular_acceleration"])
  def test_generated_file_matches_source(self):
    generated = ROOT / "sim" / "generated" / "robots" / "a100.json"
    self.assertEqual(json.loads(generated.read_text(encoding="utf-8")), compile_robot(SOURCE))
  def test_cli_is_byte_deterministic_and_has_no_source_path(self):
    output = Path(self.workspace.name) / "robot.json"
    first = self._cli(SOURCE, output)
    self.assertEqual(first.returncode, 0, first.stdout + first.stderr)
    contents = output.read_bytes()
    second = self._cli(SOURCE, output)
    self.assertEqual(second.returncode, 0, second.stdout + second.stderr)
    self.assertEqual(output.read_bytes(), contents)
    self.assertNotIn(str(SOURCE), contents.decode("utf-8"))
  def test_rejects_every_missing_field(self):
    paths = [(key,) for key in self.spec]
    paths += [("footprint", "type"), ("footprint", "radius"), ("body", "z_min"), ("body", "z_max")]
    for path in paths:
      with self.subTest(path=path):
        spec = copy.deepcopy(self.spec)
        parent = spec if len(path) == 1 else spec[path[0]]
        del parent[path[-1]]
        with self.assertRaises(ValueError):
          self._compile(spec)
  def test_rejects_unknown_fields_at_every_level(self):
    for parent in [None, "footprint", "body"]:
      with self.subTest(parent=parent):
        spec = copy.deepcopy(self.spec)
        target = spec if parent is None else spec[parent]
        target["unknown"] = 1
        with self.assertRaises(ValueError):
          self._compile(spec)
  def test_rejects_invalid_types_and_contract(self):
    for field, value in [
      ("version", 1), ("version", False), ("id", "../outside"),
      ("footprint", {"type": "rect", "radius": 0.27}),
      ("body", []), ("min_linear_velocity", True), ("max_linear_velocity", "1.0"),
      ("provisional", ["unknown"]), ("provisional", ["body.z_min"]),
      ("provisional", [1]), ("provisional", ["body", "body"]),
    ]:
      with self.subTest(field=field, value=value):
        spec = copy.deepcopy(self.spec)
        spec[field] = value
        with self.assertRaises(ValueError):
          self._compile(spec)
    for value in [None, [], "not a robot"]:
      with self.subTest(value=value):
        with self.assertRaises(ValueError):
          self._compile(value)
  def test_rejects_invalid_geometry_limits_and_nonfinite_numbers(self):
    variants = []
    for radius in [0, -0.1, True]:
      spec = copy.deepcopy(self.spec)
      spec["footprint"]["radius"] = radius
      variants.append(spec)
    for z_max in [0.0, -0.1]:
      spec = copy.deepcopy(self.spec)
      spec["body"]["z_max"] = z_max
      variants.append(spec)
    for axis in ["linear", "angular"]:
      for value in [1.0, 2.0]:
        spec = copy.deepcopy(self.spec)
        spec[f"min_{axis}_velocity"] = value
        variants.append(spec)
      for value in [0.0, -1.0]:
        spec = copy.deepcopy(self.spec)
        spec[f"max_{axis}_acceleration"] = value
        variants.append(spec)
    numeric_paths = [(key,) for key in self.spec if key.startswith(("min_", "max_"))]
    numeric_paths += [("footprint", "radius"), ("body", "z_min"), ("body", "z_max")]
    for path in numeric_paths:
      for value in [float("nan"), float("inf"), -float("inf")]:
        spec = copy.deepcopy(self.spec)
        parent = spec if len(path) == 1 else spec[path[0]]
        parent[path[-1]] = value
        variants.append(spec)
    for spec in variants:
      with self.subTest(spec=spec):
        with self.assertRaises(ValueError):
          self._compile(spec)
  def test_rejects_duplicate_yaml_keys_at_every_level(self):
    for text in [
      "version: 0\nversion: 1\n",
      "footprint:\n  radius: 0.27\n  radius: 0.3\n",
      "body:\n  z_min: 0\n  z_min: 1\n",
    ]:
      with self.subTest(text=text):
        self.source.write_text(text, encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "Duplicate YAML key"):
          compile_robot(self.source)
  def test_invalid_cli_input_does_not_overwrite_output(self):
    self.source.write_text("version: 99\n", encoding="utf-8")
    output = Path(self.workspace.name) / "robot.json"
    output.write_text("existing output\n", encoding="utf-8")
    result = self._cli(self.source, output)
    self.assertEqual(result.returncode, 1)
    self.assertIn("Robot compilation failed", result.stderr)
    self.assertEqual(output.read_text(encoding="utf-8"), "existing output\n")

if __name__ == "__main__":
  unittest.main()
