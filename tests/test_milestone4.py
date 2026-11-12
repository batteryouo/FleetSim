import json
import math
import tempfile
import unittest

import numpy as np

from assets.build import ROOT
from env import protocol
from env.protocol import ProtocolError
from env.transport import ZmqTransport
from env.world import World, WorldProcess, find_godot
from tools.m3_fixtures import compiled, robot_spec, wall

WARD = json.loads((ROOT / "sim" / "generated" / "synthetic_ward.json").read_text(encoding="utf-8"))
WARD_POSES = {"r0": [1.0, 0.0, 0.0], "r1": [2.0, 4.0, 1.5], "r2": [6.0, 4.0, -2.0]}

class Recording:
  """Transport wrapper that keeps every raw reply."""
  def __init__(self, transport):
    self.transport = transport
    self.replies = []
  def request(self, payload):
    reply = self.transport.request(payload)
    self.replies.append(reply)
    return reply
  def close(self):
    self.transport.close()

def launch(record=False):
  process = WorldProcess(find_godot())
  transport = ZmqTransport(process.endpoint, timeout=60)
  return World(Recording(transport) if record else transport, process)
def policy(seed, steps, robots):
  rng = np.random.default_rng(seed)
  return [{robot: rng.uniform(-1.0, 1.0, 2).tolist() for robot in robots} for _ in range(steps)]
def rollout(world, seed, steps=40):
  trajectory = [world.reset(seed, WARD_POSES)]
  for actions in policy(seed, steps, sorted(WARD_POSES)):
    trajectory.append(world.step(actions))
  return trajectory
def substep(state, velocity, command, dt, spec):
  limited = []
  for axis, previous, target in zip(["linear", "angular"], velocity, command):
    change = spec[f"max_{axis}_acceleration"] * dt
    value = previous + min(max(target - previous, -change), change)
    limited.append(min(max(value, spec[f"min_{axis}_velocity"]), spec[f"max_{axis}_velocity"]))
  v, w = limited
  x, y, yaw = state
  half = w * dt / 2
  distance = v * dt * (math.sin(half) / half if half else 1.0)
  end = (yaw + w * dt + math.pi) % (2 * math.pi) - math.pi
  return [x + distance * math.cos(yaw + half), y + distance * math.sin(yaw + half),
          math.pi if end == -math.pi else end], limited
def corridor_range(origin, angle, others, spec):
  """Independent range for walls at y = +-2 and other robots' offset discs."""
  direction = (math.cos(angle), math.sin(angle))
  nearest = (math.copysign(2.0, direction[1]) - origin[1]) / direction[1] if abs(direction[1]) > 1e-12 else math.inf
  center = spec["footprint"]["center"]
  for x, y, yaw in others:
    cx = x + math.cos(yaw) * center["x"] - math.sin(yaw) * center["y"]
    cy = y + math.sin(yaw) * center["x"] + math.cos(yaw) * center["y"]
    dx, dy = origin[0] - cx, origin[1] - cy
    projection = dx * direction[0] + dy * direction[1]
    discriminant = projection ** 2 - (dx * dx + dy * dy - spec["footprint"]["radius"] ** 2)
    if discriminant >= 0 and -projection - math.sqrt(discriminant) >= 0:
      nearest = min(nearest, -projection - math.sqrt(discriminant))
  lidar = spec["lidar"]
  return -math.inf if nearest < lidar["range_min"] else (math.inf if nearest > lidar["range_max"] else nearest)

class Milestone4Tests(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    find_godot()
    cls.spec = robot_spec()
    cls.workspace = tempfile.TemporaryDirectory(prefix="cadence-m4-")
    cls.addClassCleanup(cls.workspace.cleanup)
    cls.corridor = compiled(cls.workspace.name, [wall("left", 0, 2.1, 60, 0.2), wall("right", 0, -2.1, 60, 0.2)])
  def _world(self, record=False):
    world = launch(record)
    self.addCleanup(world.close)
    return world
  def test_same_seed_reproduces_trajectories_across_processes(self):
    replies = {}
    for label, seed in [("first", 11), ("second", 11), ("other", 12)]:
      world = self._world(record=True)
      world.configure(WARD, self.spec, sorted(WARD_POSES))
      trajectory = rollout(world, seed)
      replies[label] = world._transport.replies[1:]
      self.assertEqual([item.step for item in trajectory], list(range(41)))
      self.assertTrue(all(item.seed == seed for item in trajectory))
      world.close()
      self.assertEqual(world.process.returncode, 0)
      for marker in ["ERROR", "WARNING"]:
        self.assertNotIn(marker, world.process.log())
    self.assertEqual(replies["first"], replies["second"])
    self.assertNotEqual(replies["first"][-1], replies["other"][-1])
    final = protocol.observation(protocol.decode(replies["first"][-1]))
    for robot, pose in WARD_POSES.items():
      self.assertGreater(math.dist(final.robots[robot].pose[:2], pose[:2]), 0.1)
      self.assertTrue(np.isfinite(final.robots[robot].scan.ranges).any())
  def test_reset_restarts_an_identical_episode_in_one_process(self):
    world = self._world(record=True)
    world.configure(WARD, self.spec, sorted(WARD_POSES))
    rollout(world, 5, steps=15)
    first = world._transport.replies[1:]
    rollout(world, 5, steps=15)
    self.assertEqual(world._transport.replies[1 + len(first):], first)
  def test_physics_substeps_follow_independent_kinematics(self):
    commands = [[1.0, 0.8]] * 4 + [[-0.6, -1.0]] * 3 + [[0.3, 0.0]] * 3
    finals = {}
    for dt, substeps in [(0.01, 10), (0.02, 5), (0.1, 1)]:
      world = self._world()
      reply = world.configure(self.corridor, self.spec, ["r0"], dt, substeps)
      self.assertEqual((reply["physics_dt"], reply["physics_steps_per_control"]), (dt, substeps))
      self.assertAlmostEqual(reply["control_period"], 0.1, places=12)
      result = world.reset(0, {"r0": [0.5, -0.3, 3.0]})
      state, velocity = [0.5, -0.3, 3.0], [0.0, 0.0]
      for index, command in enumerate(commands):
        result = world.step({"r0": command})
        for _ in range(substeps):
          state, velocity = substep(state, velocity, command, dt, self.spec)
        robot = result.robots["r0"]
        np.testing.assert_allclose(robot.pose, state, rtol=0, atol=1e-9)
        np.testing.assert_allclose(robot.velocity, velocity, rtol=0, atol=1e-12)
        self.assertEqual(result.step, index + 1)
        self.assertAlmostEqual(result.time, (index + 1) * dt * substeps, places=12)
      finals[substeps] = state
    self.assertGreater(math.dist(finals[10][:2], finals[1][:2]), 1e-3)
  def test_scan_follows_laserscan_semantics_and_sees_offset_robots(self):
    world = self._world()
    world.configure(self.corridor, self.spec, ["a", "b"])
    poses = {"a": [0.0, 0.0, 0.0], "b": [3.0, 0.1, math.pi]}
    result = world.reset(3, poses)
    lidar = self.spec["lidar"]
    for robot, other in [("a", "b"), ("b", "a")]:
      scan = result.robots[robot].scan
      self.assertEqual((scan.angle_min, scan.angle_max, scan.range_min, scan.range_max),
                       (lidar["angle_min"], lidar["angle_max"], lidar["range_min"], lidar["range_max"]))
      self.assertEqual(scan.angle_increment, (lidar["angle_max"] - lidar["angle_min"]) / (lidar["beam_count"] - 1))
      self.assertEqual(scan.ranges.dtype, np.dtype("<f8"))
      self.assertEqual(len(scan.ranges), lidar["beam_count"])
      x, y, yaw = poses[robot]
      mount = lidar["mount"]
      origin = (x + math.cos(yaw) * mount["x"] - math.sin(yaw) * mount["y"],
                y + math.sin(yaw) * mount["x"] + math.cos(yaw) * mount["y"])
      expected = [corridor_range(origin, yaw + mount["yaw"] + scan.angle_min + beam * scan.angle_increment,
                                 [poses[other]], self.spec) for beam in range(len(scan.ranges))]
      np.testing.assert_allclose(scan.ranges, expected, rtol=0, atol=1e-4)
      self.assertTrue(np.isposinf(scan.ranges).any())
      self.assertLess(scan.ranges[len(scan.ranges) // 2], 2.6)
  def test_rejected_requests_leave_the_world_unchanged(self):
    world = self._world(record=True)
    ids = ["r0", "r1"]
    poses = {"r0": [1.0, 0.0, 0.0], "r1": [2.0, 4.0, 1.0]}
    actions = {"r0": [0.4, 0.2], "r1": [-0.3, 0.5]}
    def configure(**changes):
      fields = dict(map=WARD, robot_specs={"a100": self.spec}, physics_dt=0.01, physics_steps_per_control=10,
                    robots=[{"id": robot, "spec": "a100"} for robot in ids])
      return protocol.request("configure", **{**fields, **changes})
    def rejected(code, message):
      with self.assertRaises(ProtocolError) as context:
        world.request(message)
      self.assertEqual(context.exception.code, code, context.exception.message)
    rejected("protocol_version", {**protocol.request("close"), "protocol": 2})
    rejected("protocol_version", {"type": "close"})
    rejected("unknown_type", protocol.request("teleport"))
    rejected("invalid_state", protocol.request("reset", seed=1, poses=poses))
    rejected("invalid_state", protocol.request("step", step=0, actions=actions))
    second = {**self.spec, "id": "other"}
    rejected("unsupported", configure(robot_specs={"a100": self.spec, "other": second},
                                      robots=[{"id": "r0", "spec": "a100"}, {"id": "r1", "spec": "other"}]))
    for changes in [dict(physics_dt=0.0), dict(physics_dt=-0.01), dict(physics_steps_per_control=0),
                    dict(physics_steps_per_control=1.5), dict(robots=[]), dict(extra=1),
                    dict(robots=[{"id": "r0", "spec": "a100"}] * 2), dict(robots=[{"id": "0bad", "spec": "a100"}]),
                    dict(robots=[{"id": "r0", "spec": "missing"}]), dict(map={**WARD, "version": 0}),
                    dict(robot_specs={"a100": {**self.spec, "version": 0}})]:
      with self.subTest(changes=changes):
        rejected("invalid_argument", configure(**changes))
    world.configure(WARD, self.spec, ids)
    rejected("invalid_state", protocol.request("step", step=0, actions=actions))
    for message in [protocol.request("reset", seed=1, poses={"r0": poses["r0"]}),
                    protocol.request("reset", seed=-1, poses=poses),
                    protocol.request("reset", seed=1.0, poses=poses),
                    protocol.request("reset", seed=1, poses={**poses, "r1": [2.0, math.nan, 0.0]}),
                    protocol.request("reset", seed=1, poses={**poses, "r1": [2.0, 4.0]})]:
      with self.subTest(message=message):
        rejected("invalid_argument", message)
    world.reset(1, poses)
    first = world._transport.replies[-1]
    rejected("step_mismatch", protocol.request("step", step=1, actions=actions))
    for bad in [{"r0": actions["r0"]}, {**actions, "r2": [0.0, 0.0]}, {**actions, "r1": ["fast", 0.0]},
                {**actions, "r1": [math.inf, 0.0]}]:
      with self.subTest(actions=bad):
        rejected("invalid_argument", protocol.request("step", step=0, actions=bad))
    rejected("invalid_argument", configure(physics_dt=0.0))
    with self.assertRaises(ProtocolError) as context:
      protocol.decode(world._transport.request(b"\xc1"))
    self.assertEqual(context.exception.code, "malformed")
    stepped = world.step(actions)
    after_errors = world._transport.replies[-1]
    self.assertEqual(stepped.step, 1)
    world.reset(1, poses)
    self.assertEqual(world._transport.replies[-1], first)
    world.step(actions)
    self.assertEqual(world._transport.replies[-1], after_errors)
  def test_world_exits_on_close_disconnect_or_no_client(self):
    world = launch()
    world.close()
    self.assertEqual(world.process.returncode, 0)
    process = WorldProcess(find_godot())
    transport = ZmqTransport(process.endpoint)
    with self.assertRaises(ProtocolError):
      protocol.decode(transport.request(protocol.encode(protocol.request("unknown"))))
    transport.close()
    self.assertEqual(process.stop(30), 1, process.log())
    self.assertIn("Client connection ended", process.log())
    process = WorldProcess(find_godot(), startup_timeout=1.0)
    self.assertEqual(process.stop(30), 1, process.log())
    self.assertIn("No client connected", process.log())

class ClientValidationTests(unittest.TestCase):
  class Fake:
    def __init__(self, replies):
      self.replies = list(replies)
      self.sent = []
    def request(self, payload):
      self.sent.append(payload)
      return protocol.encode(self.replies.pop(0))
    def close(self):
      pass
  def _world(self):
    world = World(self.Fake([{"protocol": 1, "type": "configure", "robots": ["r0"]}]))
    world.configure({"version": 1}, {"id": "a100"}, ["r0"])
    return world
  def test_client_validates_before_sending(self):
    world = self._world()
    with self.assertRaises(RuntimeError):
      world.step({"r0": [0, 0]})
    for seed in [-1, 2 ** 63, True, 1.0]:
      with self.subTest(seed=seed), self.assertRaises(ValueError):
        world.reset(seed, {"r0": [0, 0, 0]})
    for poses in [{}, {"r0": [0, 0]}, {"r0": [0, 0, math.nan]}, {"r0": [0, 0, 0], "r1": [0, 0, 0]}]:
      with self.subTest(poses=poses), self.assertRaises(ValueError):
        world.reset(1, poses)
    self.assertEqual(len(world._transport.sent), 1)
  def test_client_rejects_fractional_substeps(self):
    world = World(self.Fake([]))
    for steps in [1.9, 0, -1, True, "10"]:
      with self.subTest(steps=steps), self.assertRaises(ValueError):
        world.configure({"version": 1}, {"id": "a100"}, ["r0"], physics_steps_per_control=steps)
    self.assertEqual(world._transport.sent, [])
  def test_raw_requests_update_client_state(self):
    mismatch = {"protocol": 1, "type": "error", "code": "step_mismatch", "message": "x"}
    world = self._world()
    world._transport.replies = [{"protocol": 1, "type": "reset", "step": 0},
                                {"protocol": 1, "type": "step", "step": 1},
                                {"protocol": 1, "type": "reset", "step": 0}, mismatch,
                                {"protocol": 1, "type": "configure", "robots": ["r1"]}]
    for message in [protocol.request("reset", seed=1, poses={"r0": [0, 0, 0]}),
                    protocol.request("step", step=0, actions={"r0": [0, 0]}),
                    protocol.request("reset", seed=1, poses={"r0": [0, 0, 0]})]:
      world.request(message)
    with self.assertRaises(ProtocolError):
      world.step({"r0": [0, 0]})
    self.assertEqual(protocol.decode(world._transport.sent[-1])["step"], 0)
    world.request(protocol.request("configure"))
    with self.assertRaises(RuntimeError):
      world.step({"r1": [0, 0]})
    with self.assertRaises(ValueError):
      world.reset(1, {"r0": [0, 0, 0]})
  def test_error_and_foreign_replies_raise_protocol_error(self):
    with self.assertRaises(ProtocolError) as context:
      protocol.decode(protocol.encode({"protocol": 1, "type": "error", "code": "step_mismatch", "message": "x"}))
    self.assertEqual(context.exception.code, "step_mismatch")
    for payload in [protocol.encode({"protocol": 2, "type": "close"}), protocol.encode([1]), b"\xc1"]:
      with self.subTest(payload=payload), self.assertRaises(ProtocolError):
        protocol.decode(payload)

if __name__ == "__main__":
  unittest.main()
