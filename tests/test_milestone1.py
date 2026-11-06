import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from assets.build import ROOT, compile_map

class Milestone1Tests(unittest.TestCase):
  def test_compiled_map_has_expected_physics_ray_distances(self):
    godot = os.environ.get("GODOT_BIN") or shutil.which("godot") or shutil.which("godot4")
    self.assertTrue(godot, "Set GODOT_BIN to the Godot console executable or put godot on PATH")
    with tempfile.TemporaryDirectory(prefix="cadence-m1-") as workspace:
      output = Path(workspace) / "test_room.json"
      build = subprocess.run(
        [sys.executable, str(ROOT / "assets" / "build.py"),
         str(ROOT / "spec" / "maps" / "test_room.yaml"), "--output", str(output)],
        capture_output=True, text=True, timeout=30,
      )
      self.assertEqual(build.returncode, 0, build.stdout + build.stderr)
      result = subprocess.run(
        [godot, "--headless", "--path", str(ROOT / "sim"),
         "--log-file", str(Path(workspace) / "godot.log"),
         "--script", "res://tests/ray_distance_test.gd", "--", "--map", str(output)],
        capture_output=True, text=True, timeout=60,
      )
      transcript = result.stdout + result.stderr
      self.assertEqual(result.returncode, 0, transcript)
      self.assertNotIn("SCRIPT ERROR", transcript)
      self.assertNotIn("ERROR:", transcript)
      self.assertIn("MILESTONE_1_PASS: 13 rays", transcript)
  def test_loader_rejects_invalid_height_contract_without_partial_map(self):
    godot = os.environ.get("GODOT_BIN") or shutil.which("godot") or shutil.which("godot4")
    self.assertTrue(godot, "Set GODOT_BIN to the Godot console executable or put godot on PATH")
    compiled = compile_map(ROOT / "spec" / "maps" / "test_room.yaml")
    cases = [
      ("z_min", None), ("z_max", "2.5"), ("z_min", True),
      ("z_min", 2.5), ("z_min", 3),
      ("center", [1, 2, 3]), ("size", [1, 2, 3]),
    ]
    with tempfile.TemporaryDirectory(prefix="cadence-heights-") as workspace:
      paths = []
      for index, (field, value) in enumerate(cases + [("z_min", "missing"), ("z_max", "missing")]):
        data = copy.deepcopy(compiled)
        if value == "missing":
          del data["primitives"][1][field]
        else:
          data["primitives"][1][field] = value
        path = Path(workspace) / f"invalid_{index}.json"
        path.write_text(json.dumps(data), encoding="utf-8")
        paths.append(str(path))
      manifest = Path(workspace) / "manifest.json"
      manifest.write_text(json.dumps(paths), encoding="utf-8")
      result = subprocess.run(
        [godot, "--headless", "--path", str(ROOT / "sim"),
         "--log-file", str(Path(workspace) / "godot.log"),
         "--script", "res://tests/height_contract_test.gd", "--", "--manifest", str(manifest)],
        capture_output=True, text=True, timeout=60,
      )
      transcript = result.stdout + result.stderr
      self.assertEqual(result.returncode, 0, transcript)
      self.assertNotIn("SCRIPT ERROR", transcript)
      self.assertIn(f"HEIGHT_CONTRACT_PASS: {len(paths)} invalid maps rejected", transcript)

if __name__ == "__main__":
  unittest.main()
