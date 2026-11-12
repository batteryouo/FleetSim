# World protocol v1

The World (Godot) and its clients (Python) exchange MessagePack maps over a
ZeroMQ request-reply connection. The World logic in `sim/world/runtime.gd` sees
only decoded dictionaries; `sim/transport/` owns sockets and encoding. Moving a
client to another process or machine changes the endpoint, not the World.

## Transport

- ZeroMQ ZMTP 3 over TCP with the NULL mechanism (no authentication or
  encryption; participants are trusted). The World implements the protocol in
  GDScript (`sim/transport/zmtp.gd`) and accepts one REQ or DEALER peer. A
  DEALER peer must send the empty delimiter frame that REQ adds.
- `sim/transport/server.gd` binds `127.0.0.1` on a free port by default, writes
  `tcp://ADDRESS:PORT` to `--endpoint-file`, and serves one connection. It exits
  0 after `close`, and 1 if the connection drops or no client connects within
  `--accept-timeout` seconds. `--bind` and `--port` select another interface.
- Each request is answered by exactly one reply before the next request.
  This strict alternation is the lockstep: the World advances only inside a
  `step` request.

## Encoding

- Every message is one MessagePack map with string keys. Requests and replies
  carry `protocol` (integer, `1`) and `type` (string).
- Floats are float64. Integers are signed 64-bit; larger values are malformed.
- `ranges` is MessagePack bin: little-endian float64 values, one per beam.
  `+inf` and `-inf` keep their REP-117 meanings.
- Unknown request fields are rejected. Adding, removing, or reinterpreting a
  field requires a new protocol version; the World rejects other versions with
  `protocol_version`.
- All values use REP-103 frames and SI units.

## Requests

| type | Fields | Reply |
| --- | --- | --- |
| `configure` | `map`, `robot_specs`, `robots`, `physics_dt`, `physics_steps_per_control` | `robots`, `physics_dt`, `physics_steps_per_control`, `control_period` |
| `reset` | `seed`, `poses` | observation |
| `step` | `step`, `actions` | observation |
| `close` | none | none; the World then exits |

`configure` fields:

- `map`: a compiled version 1 map (the JSON object produced by `assets.build`).
- `robot_specs`: spec name to compiled version 1 robot spec.
- `robots`: ordered array of `{id, spec}`. IDs are unique identifiers matching
  `^[A-Za-z][A-Za-z0-9_]*$`. Version 1 requires every robot to use the same spec
  (`unsupported` otherwise).
- `physics_dt`: positive kinematics step in seconds.
- `physics_steps_per_control`: positive integer. One control period is
  `physics_dt * physics_steps_per_control`. The Python client defaults to 0.01 s
  and 10, a 10 Hz control rate.

A successful `configure` replaces any previous configuration and ends the
current episode. `reset` requires a configuration; `step` requires an episode.

`reset` fields: `seed`, an integer in `[0, 2^63 - 1]`, and `poses`, a map from
every configured robot ID to `[x, y, yaw]`. Velocities start at zero and the
step counter at 0. The World currently draws no random numbers; the seed is
recorded and returned so later randomness (noise from `(seed, robot_id, step)`,
pedestrians, spawns) has a defined source.

`step` fields: `step`, the World's current step index, and `actions`, a map from
every configured robot ID to `[v, w]` in m/s and rad/s. A different step index is
rejected with `step_mismatch`, so duplicated or reordered requests cannot
advance the World. Each action is held for all physics steps of the control
period and limited by the robot spec as described in `docs/robot-spec.md`.
The scan is computed once per control period, from the final poses.
The spec's `lidar.rate_hz` does not schedule scans.

## Observations

`reset` and `step` reply with:

- `seed`, `step`, and `time` (`step` control periods, in seconds).
- `robots`: map in configured order from robot ID to:
  - `pose`: `[x, y, yaw]` of the robot's reference frame (the wheel axle).
  - `velocity`: `[v, w]` applied during the last physics step.
  - `scan`: `angle_min`, `angle_max`, `angle_increment`, `range_min`,
    `range_max`, and `ranges` with LaserScan semantics.
- `events`: an array, always empty in version 1. Collision events arrive with
  Milestone 5.

Poses and scans are ground truth; there is no noise, collision response, or
robot-wall blocking yet. Robots can pass through walls and each other.

## Errors

A rejected request returns `{protocol, type: "error", code, message}` and leaves
the configuration, episode, and step counter unchanged.

| code | Meaning |
| --- | --- |
| `malformed` | The payload is not a decodable MessagePack map. |
| `protocol_version` | `protocol` is missing or not 1. |
| `unknown_type` | `type` is not a version 1 request. |
| `invalid_argument` | A field is missing, unknown, or invalid. |
| `invalid_state` | `reset` before `configure`, or `step` before `reset`. |
| `step_mismatch` | `step` differs from the World's step index. |
| `unsupported` | Valid but not supported in version 1, such as mixed robot specs. |
| `internal` | A World component failed; the World state is unchanged. |

## Determinism

With the same configuration, seed, initial poses, and actions, a World produces
bit-identical reply bytes, across processes on the same build and machine. The
reply has no wall-clock fields. Cross-platform bit identity is not promised;
see `docs/robot-spec.md`.

## Python client

`env.World` wraps the protocol:

```python
from env import World
from env.world import load_json

with World.launch() as world:  # GODOT_BIN or godot on PATH
  world.configure(load_json("sim/generated/synthetic_ward.json"),
                  load_json("sim/generated/robots/a100.json"), ["r0", "r1"])
  observation = world.reset(seed=7, poses={"r0": (1, 0, 0), "r1": (2, 4, 1.5)})
  observation = world.step({"r0": (0.5, 0.1), "r1": (0.3, -0.2)})
  ranges = observation.robots["r0"].scan.ranges  # read-only float64 array
```

`World(transport)` accepts any object with `request(bytes) -> bytes` and
`close()`, for example `env.transport.ZmqTransport(endpoint)` connected to a
World started elsewhere. Error replies raise `env.ProtocolError` with the code.
`World.request` sends a raw protocol map; successful `configure`, `reset`, and
`step` replies still update the client's robot IDs and step index.

## Not in version 1

There is one client per World, and robots of one spec. There are no partial
observations, map blocks, async stepping, pedestrians, collision events, or
noise. A multi-node World will need several peers, per-node robot subsets, and
static-map partitions. Those would be a new protocol version; version 1 replies
already carry no wall-clock or process-local data.
