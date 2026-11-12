"""World protocol version 1 encoding and reply types; see docs/protocol.md."""
from dataclasses import dataclass
import math

import msgpack
import numpy as np

PROTOCOL_VERSION = 1
MAX_SEED = 2 ** 63 - 1

class ProtocolError(RuntimeError):
  """An error reply from the World; `code` is the protocol error code."""
  def __init__(self, code: str, message: str):
    super().__init__(f"{code}: {message}")
    self.code = code
    self.message = message

@dataclass(frozen=True)
class Scan:
  """One lidar scan with ROS LaserScan semantics; `ranges` is a read-only float64 array."""
  angle_min: float
  angle_max: float
  angle_increment: float
  range_min: float
  range_max: float
  ranges: np.ndarray

@dataclass(frozen=True)
class RobotObservation:
  """Ground-truth REP-103 pose (x, y, yaw), applied velocity (v, w), and lidar scan."""
  pose: tuple
  velocity: tuple
  scan: Scan

@dataclass(frozen=True)
class Observation:
  """World state after a reset or step; `time` is `step` control periods in seconds."""
  seed: int
  step: int
  time: float
  robots: dict
  events: list

def encode(message: dict) -> bytes:
  return msgpack.packb(message, use_bin_type=True)
def decode(payload: bytes) -> dict:
  try:
    reply = msgpack.unpackb(payload, raw=False, strict_map_key=True)
  except (ValueError, msgpack.ExtraData, msgpack.FormatError, msgpack.StackError) as error:
    raise ProtocolError("malformed", f"Undecodable reply: {error}") from error
  if not isinstance(reply, dict) or reply.get("protocol") != PROTOCOL_VERSION:
    raise ProtocolError("protocol_version", f"Expected a protocol {PROTOCOL_VERSION} reply")
  if reply.get("type") == "error":
    raise ProtocolError(reply.get("code", "unknown"), reply.get("message", ""))
  return reply
def request(type_: str, **fields) -> dict:
  return {"protocol": PROTOCOL_VERSION, "type": type_, **fields}
def vector(values, width: int, label: str) -> list:
  """Validates a finite numeric sequence and returns it as float64 values."""
  values = [float(value) for value in values]
  if len(values) != width or not all(math.isfinite(value) for value in values):
    raise ValueError(f"{label} must contain {width} finite numbers")
  return values
def observation(reply: dict) -> Observation:
  robots = {}
  for robot_id, data in reply["robots"].items():
    scan = data["scan"]
    if len(scan["ranges"]) % 8:
      raise ProtocolError("malformed", "ranges must be little-endian float64 values")
    ranges = np.frombuffer(scan["ranges"], dtype="<f8")
    robots[robot_id] = RobotObservation(
      tuple(data["pose"]), tuple(data["velocity"]),
      Scan(scan["angle_min"], scan["angle_max"], scan["angle_increment"],
           scan["range_min"], scan["range_max"], ranges))
  return Observation(reply["seed"], reply["step"], reply["time"], robots, reply["events"])
