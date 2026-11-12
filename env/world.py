"""Lockstep World client and a launcher for a local headless World process."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

from env import protocol
from env.transport import ZmqTransport

SIM_DIR = Path(__file__).resolve().parent.parent / "sim"

def find_godot() -> str:
  """Returns GODOT_BIN, or godot / godot4 from PATH; raises RuntimeError if none exists."""
  godot = os.environ.get("GODOT_BIN") or shutil.which("godot") or shutil.which("godot4")
  if not godot:
    raise RuntimeError("Set GODOT_BIN to the Godot console executable or put godot on PATH")
  return godot
def load_json(path) -> dict:
  return json.loads(Path(path).read_text(encoding="utf-8"))

class WorldProcess:
  """A headless Godot World server on a free local port.

  The server writes its endpoint once it listens; startup failures raise RuntimeError with the
  server log. `stop` waits for a normal exit, kills the process after `timeout` seconds, keeps
  the log text available through `log()`, and removes the temporary directory.
  """
  def __init__(self, godot: str | None = None, startup_timeout: float = 60.0, bind: str = "127.0.0.1"):
    self._directory = tempfile.TemporaryDirectory(prefix="cadence-world-")
    directory = Path(self._directory.name)
    endpoint_file = directory / "endpoint"
    log_path = directory / "world.log"
    self._log = log_path.open("w", encoding="utf-8")
    self._log_text = None
    self._process = subprocess.Popen(
      [godot or find_godot(), "--headless", "--path", str(SIM_DIR), "--script", "res://transport/server.gd",
       "--", "--bind", bind, "--endpoint-file", str(endpoint_file),
       "--accept-timeout", str(startup_timeout)],
      stdout=self._log, stderr=subprocess.STDOUT)
    deadline = time.monotonic() + startup_timeout
    while not endpoint_file.exists():
      if self._process.poll() is not None or time.monotonic() > deadline:
        self.stop(0)
        raise RuntimeError(f"The World did not start:\n{self.log()}")
      time.sleep(0.02)
    self.endpoint = endpoint_file.read_text(encoding="utf-8")
  def log(self) -> str:
    """Returns the server's combined stdout and stderr so far."""
    if self._log_text is not None:
      return self._log_text
    self._log.flush()
    return Path(self._log.name).read_text(encoding="utf-8", errors="replace")
  def stop(self, timeout: float = 10.0) -> int:
    """Waits for exit, killing the process after `timeout` seconds; returns its exit code."""
    try:
      return self._process.wait(timeout)
    except subprocess.TimeoutExpired:
      self._process.kill()
      return self._process.wait()
    finally:
      if self._log_text is None:
        self._log_text = self.log()
        self._log.close()
        self._directory.cleanup()
  @property
  def returncode(self):
    return self._process.poll()

class World:
  """Lockstep client for one World over a request transport.

  `configure` loads a compiled map and one shared robot spec for named robots; `reset` starts an
  episode from explicit poses; `step` applies one (v, w) action per robot for one control period.
  Error replies raise ProtocolError and leave the World unchanged. Not thread-safe.
  """
  def __init__(self, transport, process: WorldProcess | None = None):
    self._transport = transport
    self._process = process
    self._robots = []
    self._step = None
  @classmethod
  def launch(cls, godot: str | None = None, timeout: float = 60.0) -> "World":
    """Starts a local World process and connects to it."""
    process = WorldProcess(godot, timeout)
    try:
      return cls(ZmqTransport(process.endpoint, timeout), process)
    except Exception:
      process.stop(0)
      raise
  @property
  def process(self) -> WorldProcess | None:
    return self._process
  def request(self, message: dict) -> dict:
    """Sends one raw protocol message and returns the decoded non-error reply.

    Successful configure, reset, and step replies update the client's robot ids and step index,
    so raw requests and the typed methods can be mixed.
    """
    reply = protocol.decode(self._transport.request(protocol.encode(message)))
    if reply["type"] == "configure":
      self._robots = list(reply["robots"])
      self._step = None
    elif reply["type"] in ("reset", "step"):
      self._step = reply["step"]
    return reply
  def configure(self, map_data: dict, robot_spec: dict, robot_ids, physics_dt: float = 0.01,
                physics_steps_per_control: int = 10) -> dict:
    """Configures the World; the defaults give a 10 Hz control period of ten 0.01 s steps."""
    steps = physics_steps_per_control
    if isinstance(steps, bool) or not isinstance(steps, int) or steps < 1:
      raise ValueError("physics_steps_per_control must be a positive integer")
    robots = [{"id": robot_id, "spec": robot_spec["id"]} for robot_id in robot_ids]
    reply = self.request(protocol.request(
      "configure", map=map_data, robot_specs={robot_spec["id"]: robot_spec}, robots=robots,
      physics_dt=float(physics_dt), physics_steps_per_control=steps))
    return reply
  def reset(self, seed: int, poses: dict) -> protocol.Observation:
    """Starts an episode with zero velocities; `poses` maps every robot id to (x, y, yaw)."""
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed <= protocol.MAX_SEED:
      raise ValueError("seed must be an integer in [0, 2**63 - 1]")
    message = protocol.request("reset", seed=seed, poses=self._per_robot(poses, 3, "pose"))
    return protocol.observation(self.request(message))
  def step(self, actions: dict) -> protocol.Observation:
    """Advances one control period; `actions` maps every robot id to (v, w)."""
    if self._step is None:
      raise RuntimeError("Reset the World before stepping")
    message = protocol.request("step", step=self._step, actions=self._per_robot(actions, 2, "action"))
    return protocol.observation(self.request(message))
  def close(self) -> None:
    """Asks the World to exit, closes the transport, and stops a launched process."""
    try:
      if self._process is None or self._process.returncode is None:
        self.request(protocol.request("close"))
    except (ConnectionError, TimeoutError, protocol.ProtocolError):
      pass
    finally:
      self._transport.close()
      if self._process is not None:
        self._process.stop()
  def __enter__(self):
    return self
  def __exit__(self, *exc):
    self.close()
  def _per_robot(self, values: dict, width: int, label: str) -> dict:
    if set(values) != set(self._robots):
      raise ValueError(f"Expected one {label} for each configured robot: {self._robots}")
    return {robot_id: protocol.vector(values[robot_id], width, f"{label} for {robot_id}")
            for robot_id in self._robots}
