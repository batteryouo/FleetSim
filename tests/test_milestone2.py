import copy
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import yaml

from assets.build import ROOT

def normalized_yaw(yaw):
  wrapped = (yaw + math.pi) % (2 * math.pi) - math.pi
  return math.pi if wrapped == -math.pi else wrapped
def arc_pose(pose, v, w, duration):
  x, y, yaw = pose
  if w == 0:
    return [x + v * duration * math.cos(yaw), y + v * duration * math.sin(yaw), normalized_yaw(yaw)]
  end = yaw + w * duration
  return [x + v / w * (math.sin(end) - math.sin(yaw)),
          y - v / w * (math.cos(end) - math.cos(yaw)), normalized_yaw(end)]
def case(label, pose, previous, command, dt, steps, expected_pose, expected_velocity, tolerance=1e-12):
  return {"label": label, "states": pose, "velocities": previous, "commands": command,
          "dt": dt, "steps": steps, "expected_states": expected_pose,
          "expected_velocities": expected_velocity, "tolerance": tolerance}

class Milestone2Tests(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.godot = os.environ.get("GODOT_BIN") or shutil.which("godot") or shutil.which("godot4")
    if not cls.godot:
      raise RuntimeError("Set GODOT_BIN to the Godot console executable or put godot on PATH")
    cls.workspace = tempfile.TemporaryDirectory(prefix="cadence-m2-")
    cls.addClassCleanup(cls.workspace.cleanup)
    cls.directory = Path(cls.workspace.name)
    production = ROOT / "spec" / "robots" / "a100.yaml"
    high = yaml.safe_load(production.read_text(encoding="utf-8"))
    high.update(id="kinematics_test", max_linear_acceleration=1000000.0, max_angular_acceleration=1000000.0)
    high_source = cls.directory / "high_acceleration.yaml"
    high_source.write_text(yaml.safe_dump(high), encoding="utf-8")
    asymmetric = copy.deepcopy(high)
    asymmetric.update(id="asymmetric_test", min_linear_velocity=-0.4, max_linear_velocity=0.8,
                      min_angular_velocity=-0.7, max_angular_velocity=0.6)
    asymmetric_source = cls.directory / "asymmetric.yaml"
    asymmetric_source.write_text(yaml.safe_dump(asymmetric), encoding="utf-8")
    cls.robots = {}
    for label, source in [("production", production), ("high", high_source), ("asymmetric", asymmetric_source)]:
      output = cls.directory / f"{label}.json"
      result = subprocess.run(
        [sys.executable, "-m", "assets.build_robot", str(source), "--output", str(output)],
        cwd=ROOT, capture_output=True, text=True, timeout=30,
      )
      if result.returncode != 0:
        raise RuntimeError(result.stdout + result.stderr)
      cls.robots[label] = output
  def _run(self, cases, robot="high", expected_status=0):
    source = self.directory / f"{self._testMethodName}.json"
    source.write_text(json.dumps(cases, allow_nan=False), encoding="utf-8")
    result = subprocess.run(
      [self.godot, "--headless", "--path", str(ROOT / "sim"),
       "--log-file", str(source.with_suffix(".log")),
       "--script", "res://tests/kinematics_test.gd", "--", "--robot", str(self.robots[robot]), "--cases", str(source)],
      capture_output=True, text=True, timeout=60,
    )
    transcript = result.stdout + result.stderr
    self.assertEqual(result.returncode, expected_status, transcript)
    self.assertNotIn("SCRIPT ERROR", transcript)
    if expected_status == 0 and not any(item.get("invalid") for item in cases):
      self.assertNotIn("ERROR:", transcript)
    if expected_status == 0:
      self.assertIn(f"MILESTONE_2_PASS: {len(cases)} cases", transcript)
    else:
      self.assertNotIn("MILESTONE_2_PASS", transcript)
  def test_acceptance_one_meter_per_second_for_two_seconds(self):
    self._run([case("acceptance", [0, 0, 0], [0, 0], [1, 0], 0.1, 20, [2, 0, 0], [1, 0], 1e-6)])
  def test_straight_from_nonzero_pose_and_yaw(self):
    pose = [3, -4, math.pi / 3]
    expected = [3 + 1.5 * math.cos(math.pi / 3), -4 + 1.5 * math.sin(math.pi / 3), math.pi / 3]
    self._run([case("offset_straight", pose, [0, 0], [0.75, 0], 0.2, 10, expected, [0.75, 0])])
  def test_pure_rotation_and_yaw_wrap(self):
    self._run([
      case("positive_wrap", [1, 2, math.pi - 0.1], [0, 0], [0, 0.4], 0.1, 10,
           [1, 2, -math.pi + 0.3], [0, 0.4]),
      case("negative_wrap", [1, 2, -math.pi + 0.1], [0, 0], [0, -0.5], 0.2, 5,
           [1, 2, math.pi - 0.4], [0, -0.5]),
      case("minus_pi_endpoint", [1, 2, -math.pi], [0, 0], [0, 0], 1, 1, [1, 2, math.pi], [0, 0]),
    ])
  def test_constant_arc_is_independent_of_step_size(self):
    pose = [1.2, -0.7, 0.3]
    expected = arc_pose(pose, 0.8, 0.5, 2.0)
    self._run([case(f"arc_dt_{dt}", pose, [0, 0], [0.8, 0.5], dt, round(2 / dt), expected, [0.8, 0.5])
               for dt in [2.0, 1.0, 0.5, 0.2, 0.1, 0.05, 0.01]])
  def test_near_zero_turn_retains_lateral_motion(self):
    self._run([case(f"small_turn_{w}", [0, 0, 0], [0, 0], [0.7, w], 2, 1,
                   [1.4, 1.4 * w, 2 * w], [0.7, w], 1e-18) for w in [1e-12, -1e-12]])
  def test_velocity_clamping_including_reverse_and_negative_angular(self):
    self._run([case(f"clamp_{v}_{w}", [0, 0, 0], [0, 0], [5 * v, 5 * w], 0.25, 1,
                   arc_pose([0, 0, 0], v, w, 0.25), [v, w])
               for v, w in [(1, 1), (-1, -1), (1, -1), (-1, 1)]])
    self._run([case("asymmetric_clamp", [0, 0, 0], [0, 0], [-5, 5], 0.25, 1,
                    arc_pose([0, 0, 0], -0.4, 0.6, 0.25), [-0.4, 0.6])], "asymmetric")
  def test_acceleration_ramp_and_reversal_follow_discrete_rule(self):
    self._run([
      case("linear_ramp", [0, 0, 0], [0, 0], [1, 0], 0.1, 10, [0.8, 0, 0], [1, 0]),
      case("reverse_ramp", [0, 0, 0], [0, 0], [-1, 0], 0.1, 10, [-0.8, 0, 0], [-1, 0]),
      case("angular_ramp", [0, 0, 0], [0, 0], [0, 1], 0.025, 6, [0, 0, 0.1125], [0, 1]),
      case("negative_angular_ramp", [0, 0, 0], [0, 0], [0, -1], 0.025, 6, [0, 0, -0.1125], [0, -1]),
      case("reversal", [0, 0, 0], [1, 0], [-1, 0], 0.25, 4, [-0.25, 0, 0], [-1, 0]),
    ], "production")
  def test_velocity_limits_apply_after_acceleration(self):
    self._run([case("outside_previous_limits", [0, 0, 0], [2, -2], [5, -5], 0.1, 1,
                    arc_pose([0, 0, 0], 1, -1, 0.1), [1, -1])], "production")
  def test_batch_matches_closed_form_and_single_robot_bitwise(self):
    states, commands, expected = [], [], []
    for robot in range(32):
      pose = [robot * 0.25, -robot * 0.5, robot * 0.1]
      v, w = (robot % 7 - 3) * 0.1, (robot % 9 - 4) * 0.1
      states.extend(pose)
      commands.extend([v, w])
      expected.extend(arc_pose(pose, v, w, 0.7))
    batch = case("batch_32", states, [0] * 64, commands, 0.1, 7, expected, commands)
    batch["compare_single"] = True
    self._run([batch])
  def test_float64_state_preserves_small_motion_at_large_coordinates(self):
    self._run([case("float64_precision", [1e9, -1e9, 0], [0, 0], [0.5, 0], 0.25, 1,
                    [1e9 + 0.125, -1e9, 0], [0.5, 0], 1e-9)])
  def test_empty_batch(self):
    self._run([case("empty_batch", [], [], [], 0.1, 1, [], [])])
  def test_rejects_invalid_step_configuration(self):
    cases = [case("zero_dt", [0, 0, 0], [0, 0], [0, 0], 0, 1, [], []),
             case("negative_dt", [0, 0, 0], [0, 0], [0, 0], -0.1, 1, [], []),
             case("short_state", [0, 0], [0, 0], [0, 0], 0.1, 1, [], []),
             case("short_velocity", [0, 0, 0], [], [0, 0], 0.1, 1, [], [])]
    for item in cases:
      item["invalid"] = True
    self._run(cases)
  def test_failed_expectation_exits_nonzero(self):
    self._run([case("wrong_expectation", [0, 0, 0], [0, 0], [1, 0], 1, 1,
                    [2, 0, 0], [1, 0])], expected_status=1)

if __name__ == "__main__":
  unittest.main()
