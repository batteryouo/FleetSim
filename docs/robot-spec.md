# Robot spec v0

## Source contract and units

`spec/robot.schema.yaml` defines a single robot configuration. All fields are
explicit and required; no defaults are inserted. Units are meters, radians, and
seconds in REP-103: x forward, y left, z up. Unknown fields at every object level,
duplicate YAML keys, non-finite numbers, and unsupported versions are errors.

```yaml
version: 0
id: a100
footprint:
  type: circle
  radius: 0.27
body:
  z_min: 0.0
  z_max: 0.25
min_linear_velocity: -1.0
max_linear_velocity: 1.0
min_angular_velocity: -1.0
max_angular_velocity: 1.0
max_linear_acceleration: 2.0
max_angular_acceleration: 10.0
lidar:
  mount: {x: 0.2, y: 0.0, z: 0.215, yaw: 0.0}
  beam_count: 400
  angle_min: -1.738520663452148
  angle_max: 1.7385571002960205
  range_min: 0.15
  range_max: 16.0
  rate_hz: 12.0
provisional:
  - max_linear_acceleration
  - max_angular_acceleration
  - lidar.beam_count
  - lidar.angle_min
  - lidar.angle_max
  - lidar.range_min
  - lidar.range_max
  - lidar.rate_hz
```

IDs start with an ASCII letter and contain only ASCII letters, digits, and
underscores. The circle is centered on the wheel axis and has a positive radius.
`body` gives absolute height endpoints with `z_max > z_min`. Each minimum velocity
must be strictly below its maximum; negative linear velocity permits reverse motion.
Both acceleration limits must be positive.

`provisional` is an explicit list, possibly empty, of unique source field paths
whose values are placeholders. Every path must exist in the source; nested fields
use dot-separated keys, such as `lidar.rate_hz`. A provisional value remains active during simulation.
The marker neither supplies a default nor disables validation. Replace a placeholder
with a confirmed value and remove its name from this list when that value becomes
available. The list survives compilation unchanged.

## A100 value provenance

The local numeric references are `tmp/references/A100/xacro/A100/A100.urdf.xacro`
and `A100.gazebo.xacro` in the same directory. They are not runtime dependencies and
are never copied into tracked paths. `spec/robots/a100.yaml` is authoritative.

| Field | Value | Basis | Provisional |
| --- | --- | --- | --- |
| `footprint.radius` | 0.27 m | User-selected enclosing circle from URDF geometry: base cylinder radius 0.24 m; front box center x = 0.125 m and size 0.25 by 0.19 m give a corner radius of about 0.26744 m; caster center x = 0.2425 m plus radius 0.0175 m reaches 0.26 m. | No |
| `body.z_min`, `body.z_max` | 0.0, 0.25 m | Simplified body envelope: caster bottom is 0.0175 - 0.0175 = 0; the upper base cylinder reaches 0.2475 + 0.005 / 2 = 0.25 m. | No |
| Linear velocity limits | [-1.0, 1.0] m/s | Chosen simulation action range, including reverse; not inferred from the URDF or hardware controller. | No |
| Angular velocity limits | [-1.0, 1.0] rad/s | Chosen simulation action range; not inferred from the URDF or hardware controller. | No |
| `max_linear_acceleration` | 2.0 m/s² | Gazebo diff_drive `wheelAcceleration = 2`: both wheels accelerate together. | Yes |
| `max_angular_acceleration` | 10.0 rad/s² | Opposite wheel acceleration gives (2 - (-2)) / 0.4 = 10, using Gazebo `wheelSeparation = 0.4`. | Yes |
| `lidar.mount` | (0.2, 0, 0.215) m, yaw 0 rad | URDF `scan_joint` origin; no roll or pitch. | No |
| `lidar.beam_count` | 400 | Gazebo ray sensor `samples`. | Yes |
| `lidar.angle_min`, `lidar.angle_max` | -1.738520663452148, 1.7385571002960205 rad | Gazebo ray sensor angular limits. | Yes |
| `lidar.range_min`, `lidar.range_max` | 0.15, 16 m | Gazebo ray sensor range limits. | Yes |
| `lidar.rate_hz` | 12 Hz | Gazebo ray sensor `update_rate`. | Yes |

The acceleration values are simulator settings pending controller confirmation.
Linear and angular rate limits apply independently; this milestone does not model
coupled wheel acceleration constraints. Deployment may narrow actions on the inference
side; the simulator keeps the configured action range. All six scan parameters
are provisional because the physical lidar model is unknown. The mount comes from
the robot geometry. The control rate remains undecided; `rate_hz` describes the
sensor configuration and does not schedule scans or select an integration dt.

`lidar.mount` requires four finite numeric fields. `beam_count` is an integer at
least 2; `angle_max > angle_min`; `0 <= range_min < range_max`; `rate_hz > 0`.
Unknown nested fields, including a noise field, are rejected.

## Compiled contract

Run both map and robot compilers as modules (`python -m`) from the repository root:

```powershell
python -m assets.build_robot spec/robots/a100.yaml
python -m assets.build_robot spec/robots/a100.yaml --output tmp/a100.json
```

The sibling robot compiler reuses the map pipeline's strict YAML loader without
changing the map compiler. The default output is
`sim/generated/robots/<id>.json`, independent of the working directory; an explicit
output and the source path are relative to the calling directory. Run the module
from the repository root so the `assets` package is importable.

The JSON contains every validated source field unchanged, plus `frame: "rep103"`.
Height endpoints and provisional markers remain explicit. There are no timestamps,
source paths, engine coordinates, or inserted defaults. Identical source input
produces identical output bytes. Validation completes before output is written;
invalid input returns a nonzero status and does not overwrite existing output.
Committed generated files are regenerated only by the compiler.

## Batched kinematics

`sim/robots/kinematics.gd` exposes a stateless `advance(states, previous_velocities,
commands, dt, limits)` call through a thin adapter to the C++ `CadenceKinematics`
GDExtension. `native/src/kinematics.cpp` contains the engine-independent numerical
kernel; the C++ Godot adapter validates inputs and transfers float64 arrays. Build
the extension as described in `docs/native-build.md` before use. The script loads
it explicitly when needed, so editor import caches are not required. A missing or
unloadable extension logs an error and returns an empty dictionary; there is no
GDScript numerical fallback.

It creates no nodes or physics bodies. One shared robot
configuration supplies the motion limits for the batch; the compiled A100 JSON can
be passed directly as `limits`.

- `states`: `PackedFloat64Array` of length 3N, ordered `[x0, y0, yaw0, x1, y1, yaw1, ...]`.
- `previous_velocities`: `PackedFloat64Array` of length 2N, ordered `[v0, w0, v1, w1, ...]`.
- `commands`: `PackedFloat64Array` of length 2N in the same velocity order.
- `dt`: an explicitly supplied, finite positive duration in seconds, fixed for that
  call. The caller obtains it from configuration; the integrator selects no rate.
- Result: a dictionary with fresh `states` and `velocities` float64 arrays. Feed both
  into the next call. Inputs are not modified. N = 0 returns empty arrays; N = 1
  uses the same code path as larger batches.

Inputs must be finite and buffer lengths must agree. Motion limits must be finite,
with strictly ordered velocity bounds and positive accelerations. Invalid arguments
log an error and return an empty dictionary without advancing state.

For each axis, the step first applies the acceleration limit:

```text
v_rate = v_previous + clamp(v_command - v_previous, -a_linear * dt, a_linear * dt)
w_rate = w_previous + clamp(w_command - w_previous, -a_angular * dt, a_angular * dt)
v = clamp(v_rate, min_linear_velocity, max_linear_velocity)
w = clamp(w_rate, min_angular_velocity, max_angular_velocity)
```

The resulting velocities are held constant over the entire step. Acceleration is
a discrete command-rate rule, not continuous acceleration integrated within the
step. Velocity clamping follows rate limiting even if a supplied previous velocity
lies outside the configured range.

For yaw theta and turn `q = w * dt`, exact unicycle integration uses:

```text
h = q / 2
d = v * dt * sinc(h)
x_next = x + d * cos(theta + h)
y_next = y + d * sin(theta + h)
yaw_next = normalize(theta + q) to (-pi, pi]
```

Here `sinc(h) = sin(h) / h`, with its limit at zero equal to 1. For `abs(h) < 1e-4`,
the integrator evaluates `1 - h²/6 + h⁴/120 - h⁶/5040`. This preserves tiny nonzero
turns without subtracting nearly equal trigonometric values. The branch uses angular
displacement, so a small angular velocity held for a long dt is still integrated
along an arc. The truncation at this threshold is below float64 precision.

All integration and storage remain planar REP-103 float64: the kernel uses C++
`double`, and its Godot interface uses `PackedFloat64Array`. `Vector2`, `Vector3`,
and Godot `real_t` are not used for robot state. A future visual node must convert
through `sim/bridge/frames.gd` when it is positioned. Identical inputs in the same runtime produce bit-identical
outputs, including batch versus single-robot evaluation. No randomness or clock
input is used. Cross-platform, cross-compiler, and C++/GDScript math-library bit
identity are not specified; the independent acceptance expectations use numerical
tolerances. Native compilation disables fast-math and floating-point contraction.

## Milestone 2 acceptance

```powershell
python -m unittest discover -s tests -p test_robot_compiler.py -v
python -m unittest discover -s tests -p test_milestone2.py -v
```

Build the C++ extension first using the README commands. The unittest runner
compiles fresh robot JSON and starts headless Godot. It requires the registered
`CadenceKinematics` native class and the loaded extension before reporting success.
A temporary test spec raises both accelerations to 1,000,000, enough to reach the test commands
in one step. Driving at 1 m/s for 2 s from rest must end at (2, 0, 0) within 1e-6 m.
Production-spec tests separately verify the discrete acceleration ramps and reversal.

Other checks compare against independent Python closed-form expectations: offset
straight motion, pure rotation and yaw wrapping, arcs at seven dt values, tiny
positive and negative turns, reverse and asymmetric velocity limits, and float64
motion at large coordinates. A 32-robot batch is checked both analytically and
bit-for-bit against separate single-robot runs. Every valid step also verifies
input immutability and identical output bytes on repeated calls. Invalid dt and
buffer lengths fail without advancing state. Any failed check, script error,
timeout, or unavailable Godot executable fails unittest with a nonzero exit status.
Native boundary checks also reject non-finite buffers and invalid motion limits.
Separate isolated-project tests verify loading without editor caches and failure
when the native library is missing.

The test harness exchanges complete test fixtures with a headless process; it is
not a runtime Python bridge. Robot collision handling, noise, and dispatch remain
outside the implementation.

## Batched lidar queries (Milestone 3 stage 1)

`sim/sensors/lidar_kernel.gd` is a thin adapter to the native `CadenceLidar`
RefCounted service. Each instance owns an independent Jolt query space. Call
`configure(compiled_map, compiled_robot)`, then `scan_batch(snapshot)`. The kernel
requires Jolt Physics in project settings. Configuration accepts validated compiler
outputs; snapshots contain `ids` (unique string array), `poses`
(`PackedFloat64Array` with the same 3N layout as kinematics), and `robot_spec`
(the configuration used to create the space). An empty batch returns no scans.
Call `close()` to release server RIDs before dropping the kernel; the native
destructor also releases them if the caller drops the last reference. Reconfiguration
also releases the old space. Call from the physics-safe execution context of the
owning Godot runtime; no World node or shared process state is accessed.

The kernel updates every query-only robot body from the snapshot before issuing
any ray, including newly added, moved, and removed robots. Static query proxies
have collision mask zero: they do not integrate motion or resolve contacts.
Jolt uses 3D boxes and horizontal rays at the mount z. Other robots use cylinders
with their true footprint radius and body height interval. Each ray query
explicitly excludes its own body's RID. Jolt shape margins are zero so proxy
geometry matches the specified dimensions. The axis mapping is defined only in `sim/bridge/frames.gd`; its
`basis_to_godot()` supplies the conversion basis to the C++ helpers in
`sim/bridge/native_frames.hpp`. All frame conversions stay in that interface
layer; authoritative state stays float64 REP-103.
Engine query vectors use the standard engine's single precision, so range
comparisons use tolerances; storing results in float64 does not recover that precision.

The mount is rigidly composed with each planar robot pose:

```text
origin_x = robot_x + cos(yaw) * mount.x - sin(yaw) * mount.y
origin_y = robot_y + sin(yaw) * mount.x + cos(yaw) * mount.y
origin_z = mount.z
angle_increment = (angle_max - angle_min) / (beam_count - 1)
beam_angle(i) = angle_min + i * angle_increment, i = 0 .. beam_count - 1
world_angle(i) = robot_yaw + mount.yaw + beam_angle(i)
```

Each scan contains `robot_id`, `angle_min`, `angle_max`, `angle_increment`,
`range_min`, `range_max`, and `ranges: PackedFloat64Array`. Angles are relative
to the lidar frame, increasing counterclockwise around +z. There is no temporal
skew: all beams query one snapshot. A hit at either range endpoint is valid;
no hit within `range_max` is positive infinity, and a return strictly below
`range_min` is negative infinity, following
[REP-117](https://www.ros.org/reps/rep-0117.html). Metadata and angular conventions
follow [LaserScan](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html).
These are in-memory scan dictionaries, not ROS messages or a Python bridge.

Map primitives and robot bodies are visible exactly when
`z_min <= mount.z <= z_max`. Height endpoints are inclusive. No footprint
shrinking, scan noise, collision response, or wall-clock scheduling is applied.

Run `python -m unittest discover -s tests -p test_milestone3.py -v` for corridor,
pose composition, range/height boundary, robot movement, self-exclusion, and
stress-map checks against independent box/circle intersections (1e-4 m tolerance).
Repeated snapshots must return bit-identical range arrays. See
`docs/benchmark-m3.md` for the historical comparison that selected Jolt 3D;
README documents the current Jolt-only release benchmark.

The native implementation updates proxies, computes beam endpoints, calls Jolt,
and assembles scans. Jolt still owns spatial indexing and intersection; this is
not the stage 2 analytic ray kernel. Configuration accepts validated compiler
outputs and checks the numeric fields used by the native code before creating
geometry. Invalid snapshots return an empty array without changing proxies;
failed reconfiguration closes the previous space. Scanning after close fails.
Build the shared extension before use; there is no script fallback.

Tests require `CadenceLidar` registration and cover loading without editor caches,
a missing library, rotated walls, empty batches, invalid native inputs, repeated
configuration/close, independent instances, and destruction without explicit close.
C++ and GDScript scan results use the same geometric tolerances; cross-language
or cross-platform bit identity is not promised.
