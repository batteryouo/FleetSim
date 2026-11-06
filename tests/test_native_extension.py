import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from assets.build import ROOT

class NativeExtensionTests(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.godot = os.environ.get("GODOT_BIN") or shutil.which("godot") or shutil.which("godot4")
    if not cls.godot:
      raise RuntimeError("Set GODOT_BIN to the Godot console executable or put godot on PATH")
    suffix = {"win32": ".dll", "linux": ".so", "darwin": ".dylib"}[sys.platform]
    cls.library = ROOT / "sim" / "native" / "bin" / f"cadence_kinematics{suffix}"
    if not cls.library.is_file():
      raise RuntimeError("Build the C++ extension with CMake before running tests; see README.md")
  def _run_isolated(self, include_library, lidar=False):
    with tempfile.TemporaryDirectory(prefix="cadence-native-load-") as workspace:
      project = Path(workspace)
      (project / "project.godot").write_text('config_version=5\n[physics]\n3d/physics_engine="Jolt Physics"\n', encoding="utf-8")
      scripts = ["sensors/lidar_kernel.gd", "bridge/frames.gd", "tests/lidar_test.gd"] if lidar else ["robots/kinematics.gd", "tests/kinematics_test.gd"]
      for relative in scripts + ["native/cadence.gdextension"]:
        target = project / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / "sim" / relative, target)
      if include_library:
        target = project / "native" / "bin" / self.library.name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(self.library, target)
      cases = project / "cases.json"
      cases.write_text(json.dumps([{
        "label": "isolated_native", "states": [0, 0, 0], "velocities": [0, 0],
        "commands": [0.5, 0], "dt": 2, "steps": 1,
        "expected_states": [1, 0, 0], "expected_velocities": [0.5, 0], "tolerance": 1e-12,
      }]), encoding="utf-8")
      if lidar:
        robot = json.loads((ROOT / "sim/generated/robots/a100.json").read_text(encoding="utf-8"))
        cases.write_text(json.dumps({"cases": [{"label": "isolated_lidar", "robot": robot,
          "map": {"frame": "rep103", "primitives": []},
          "snapshots": [{"ids": ["r0"], "poses": [0, 0, 0], "expected": [["+inf"] * robot["lidar"]["beam_count"]]}]}]}), encoding="utf-8")
      arguments = ["res://tests/lidar_test.gd", "--", str(cases)] if lidar else [
        "res://tests/kinematics_test.gd", "--", "--robot", str(ROOT / "sim/generated/robots/a100.json"), "--cases", str(cases)]
      return subprocess.run(
        [self.godot, "--headless", "--path", str(project),
         "--log-file", str(project / "godot.log"), "--script", *arguments],
        capture_output=True, text=True, timeout=60,
      )
  def test_loads_native_library_without_editor_import_cache(self):
    result = self._run_isolated(True)
    transcript = result.stdout + result.stderr
    self.assertEqual(result.returncode, 0, transcript)
    self.assertNotIn("ERROR:", transcript)
    self.assertIn("NATIVE_KINEMATICS: CadenceKinematics", transcript)
    self.assertIn("MILESTONE_2_PASS: 1 cases", transcript)
  def test_missing_library_fails_without_script_fallback(self):
    result = self._run_isolated(False)
    transcript = result.stdout + result.stderr
    self.assertEqual(result.returncode, 1, transcript)
    self.assertNotIn("SCRIPT ERROR", transcript)
    self.assertIn("Build the extension with CMake", transcript)
    self.assertNotIn("MILESTONE_2_PASS", transcript)
  def test_lidar_loads_without_editor_import_cache(self):
    result = self._run_isolated(True, lidar=True)
    transcript = result.stdout + result.stderr
    self.assertEqual(result.returncode, 0, transcript)
    self.assertNotIn("ERROR:", transcript)
    self.assertIn("NATIVE_LIDAR: CadenceLidar", transcript)
    self.assertIn("MILESTONE_3_PASS: 1 cases", transcript)
  def test_lidar_missing_library_fails_without_script_fallback(self):
    result = self._run_isolated(False, lidar=True)
    transcript = result.stdout + result.stderr
    self.assertEqual(result.returncode, 1, transcript)
    self.assertNotIn("SCRIPT ERROR", transcript)
    self.assertIn("Build the extension with CMake", transcript)
    self.assertNotIn("MILESTONE_3_PASS", transcript)

if __name__ == "__main__":
  unittest.main()
