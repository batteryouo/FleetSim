import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from assets.build import ROOT
from tools.m3_fixtures import compiled, poses, robot_spec, stress_map, wall

class Milestone3Tests(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.godot = os.environ.get("GODOT_BIN") or shutil.which("godot") or shutil.which("godot4")
    if not cls.godot:
      raise RuntimeError("Set GODOT_BIN to the Godot console executable or put godot on PATH")
  def setUp(self):
    self.workspace = tempfile.TemporaryDirectory(prefix="cadence-m3-")
    self.addCleanup(self.workspace.cleanup)
    self.directory = Path(self.workspace.name)
  def _case(self, label, walls, positions, expected, **lidar):
    robot = robot_spec()
    robot["lidar"].update(beam_count=3, angle_min=-math.pi / 2, angle_max=math.pi / 2)
    robot["lidar"].update(lidar)
    return {"label": label, "map": compiled(self.directory, walls), "robot": robot,
            "snapshots": [{"ids": [f"r{i}" for i in range(len(positions) // 3)],
                           "poses": positions, "expected": expected}]}
  def _run(self, cases, expected_status=0):
    source = self.directory / "cases.json"
    source.write_text(json.dumps({"cases": cases}, allow_nan=False), encoding="utf-8")
    result = subprocess.run([self.godot, "--headless", "--path", str(ROOT / "sim"),
                             "--log-file", str(self.directory / "godot.log"), "--script",
                             "res://tests/lidar_test.gd", "--", str(source)],
                            capture_output=True, text=True, timeout=120)
    transcript = result.stdout + result.stderr
    self.assertEqual(result.returncode, expected_status, transcript)
    self.assertNotIn("SCRIPT ERROR", transcript)
    if expected_status == 0:
      self.assertNotIn("ERROR:", transcript)
      self.assertIn(f"MILESTONE_3_PASS: {len(cases)} cases", transcript)
  def test_corridor_angle_order_count_rotation_and_mount(self):
    corridor = [wall("left", 0, 2.1, 60, 0.2), wall("right", 0, -2.1, 60, 0.2)]
    self._run([
      self._case("acceptance", corridor, [0, 0, 0], [[2, "+inf", 2]]),
      self._case("yaw", corridor, [0, 0, math.pi / 2], [["+inf", 1.8, "+inf"]]),
      self._case("asymmetric", corridor, [0, 0.5, 0], [[2.5, "+inf", 1.5]]),
      self._case("mount_xy_yaw", corridor, [1, 0.5, 0], [["+inf", 1.2, "+inf"]],
                 mount={"x": 0.7, "y": 0.3, "z": 0.215, "yaw": math.pi / 2}),
      self._case("five_angles", corridor, [0, 0, 0], [[2, math.sqrt(8), "+inf", math.sqrt(8), 2]], beam_count=5),
    ])
  def test_robot_radius_self_exclusion_and_immediate_snapshot_updates(self):
    case = self._case("robots", [], [0, 0, 0, 2, 0, math.pi],
                      [["+inf", 1.53, "+inf"], ["+inf", 1.53, "+inf"]])
    case["snapshots"].append({"ids": ["r0", "r1"], "poses": [0, 0, 0, 3, 0, math.pi],
                              "expected": [["+inf", 2.53, "+inf"], ["+inf", 2.53, "+inf"]]})
    case["snapshots"].append({"ids": ["r0"], "poses": [0, 0, 0], "expected": [["+inf"] * 3]})
    self._run([case])
  def test_height_filter_closed_endpoints_and_range_sentinels(self):
    cases = []
    for label, lo, hi, distance in [("above", .216, 1, "+inf"), ("below", 0, .214, "+inf"),
                                    ("z_min", .215, 1, 2.7), ("z_max", 0, .215, 2.7)]:
      cases.append(self._case(label, [wall("obstacle", 3, 0, .2, 1, lo, hi)],
                              [0, 0, 0], [["+inf", distance, "+inf"]]))
    for label, x, expected in [("near", .3, "-inf"), ("far", 17, "+inf"), ("max", 16.3, 16.0)]:
      cases.append(self._case(label, [wall("obstacle", x, 0, .2, 1)],
                              [0, 0, 0], [["+inf", expected, "+inf"]]))
    for label, lo, hi, expected in [("body_above", .216, 1, "+inf"), ("body_below", 0, .214, "+inf"),
                                    ("body_min", .215, 1, 1.53), ("body_max", 0, .215, 1.53)]:
      case = self._case(label, [], [0, 0, 0, 2, 0, math.pi],
                        [["+inf", expected, "+inf"], ["+inf", expected, "+inf"]])
      case["robot"]["body"] = {"z_min": lo, "z_max": hi}
      cases.append(case)
    self._run(cases)
  def test_stress_map_backend_equivalence(self):
    layout = stress_map(self.directory)
    self.assertGreaterEqual(len(layout["primitives"]), 300)
    cases = []
    for beams in [360, 400, 720]:
      for count in [1, 5, 20, 40]:
        robot = robot_spec()
        robot["lidar"]["beam_count"] = beams
        cases.append({"label": f"stress_{beams}_{count}", "map": layout, "robot": robot,
                      "snapshots": [{"ids": [f"r{i}" for i in range(count)], "poses": poses(count, phase)}
                                    for phase in [0, .03, -.02]]})
    self._run(cases)
  def test_failed_expectation_exits_nonzero(self):
    self._run([self._case("deliberately_wrong", [], [0, 0, 0], [[1, 1, 1]])], expected_status=1)
