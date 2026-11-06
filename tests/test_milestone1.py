import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from assets.build import ROOT

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

if __name__ == "__main__":
  unittest.main()
