# Project Brief: Cadence

## Purpose

Cadence is a lightweight multi-robot simulation environment for training and evaluating multi-robot navigation policies, and later multi-robot dispatch. It replaces Gazebo for these workloads because Gazebo proved too heavy when simulating several robots during training.

The project takes a game-development approach: scenes are generated from data, Godot runs the simulation, and Python hosts the agents and training code.

## Scope

- Current validation scale: 2 to 5 robots in a single compute environment (one machine). This is not a limit; code must not hard-code small-N assumptions.
- The design must stay flexible enough to move agent computation to separate processes or machines later, without rewriting the World.
- Primary initial scenario: indoor environments such as hospital wards (corridors, patient rooms, nurse stations, carts, moving staff). The project itself is scene-agnostic; nothing in the core may assume a hospital.

## Non-Goals

- High-fidelity contact physics between robots. Robot-robot collisions are events, not dynamics to be simulated.
- Distributed physics. The World is always simulated in one place.
- Protection against malicious or cheating clients. All participants are trusted.
- Photorealistic rendering, at least initially. Geometry only needs to be correct for collision and range sensing.

## Technology Stack

- **Blender** (Python `bpy`, run headless): generates visual assets from specs and exports glTF.
- **Godot 4** (run headless for training): World simulation, sensing, pedestrians, collision checks.
- **Python**: bridge client, environment wrappers, training code.
- **C++17 / GDExtension**: batched differential-drive kinematics and Jolt lidar queries with thin GDScript entry points.
- **CMake and pinned godot-cpp**: build the native extension against the pinned Godot API bindings.

Milestones 1 and 2 are validated in these environments, using standard Godot
`4.7.1.stable.official.a13da4feb`:

| Platform | Python | CMake | Compiler | glibc |
| --- | --- | --- | --- | --- |
| Windows x86_64 | 3.11.6 | 3.28.2 | MinGW GCC 13.2.0 | Not applicable |
| Arch Linux x86_64 | 3.11.16 | 4.4.4 | GCC 16.2.1 | 2.44 |

Both passed all 37 tests. The Arch validation used commit `6ca0d76`, including
native `.so` loading and dynamic dependency checks. See `docs/native-build.md`
for the full commit, commands, and target ABI requirements. macOS is unvalidated.
Python dependencies are pinned in `requirements.txt`. Blender is not needed yet
and remains unpinned.

Jolt 3D is the confirmed geometry and query backend, selected explicitly in
`sim/project.godot` after the Milestone 3 comparison. Robot state remains planar;
lidar queries and kinematics run in C++; Jolt still computes ray intersections.
This language migration is separate from the stage 2 analytic lidar option.

## Architecture

### World and Agent separation

- The **World** (Godot) owns the authoritative state (environment, robot poses, pedestrians), collision detection, shared resources, and the definition of sensing.
- Sensing computation is a read-only query that advances no state. It may run anywhere that holds a synchronized state snapshot, not only inside the World process. This is not distributed physics.
- An **Agent** (Python) owns a robot's decision making: policy inference, planning, and any internal state.
- World and Agents communicate only through serialized messages (observations, actions, timestamps, events). They never share memory.
- In the current single-compute mode, the World integrates robot kinematics. The protocol must not assume this, so that robot-state integration can move to the Agent side later.
- Moving Agents to other processes or machines must only require changing the transport, not the World logic.

### Stepping modes

- **Lockstep (default)**: the World advances one fixed step only after all Agents have submitted actions. Used for training and for all reported results.
- **Async (later)**: World and Agents run independently. Treated as a separate configuration that needs its own validation; never the default.

### Determinism and reproducibility

- In lockstep mode, the same seed and configuration must produce identical trajectories.
- All randomness (map variation, noise, pedestrian behavior, spawn positions) must come from seeded generators owned by the World or the environment wrapper.
- Per-robot sensor noise is derived from `(seed, robot_id, step)`, never from a sequential global generator, so results do not depend on evaluation order or on where sensing runs.

### Collision handling

- Collision detection is authoritative on the World side.
- Checks use swept volumes between consecutive poses, so fast motion or coarse steps cannot skip a collision.
- A collision is a terminal event for the robots involved. Post-collision data is truncated rather than re-simulated, so no rollback or re-simulation is required.

### Shared resources

- Doors, elevators, narrow passages, and similar resources are owned by the World. Agents request access; the World grants it. This keeps dispatch logic consistent regardless of where Agents run.

### Latency

- The simulation itself must not introduce uncontrolled latency into observations. In reality, robots sense each other instantly.
- Communication delay, when studied, is injected explicitly and configurably, so it is a controlled variable rather than an artifact.

## Conventions

### Coordinate frames

- All external interfaces (observations, actions, map specs, robot specs, logs) use ROS REP-103: right-handed, x forward, y left, z up; meters, radians, seconds.
- Blender is Z-up and Godot is Y-up with -Z forward. Conversion to and from Godot happens in exactly one place: the World's interface layer (`sim/bridge/`). No other code may convert frames.

### Single source of truth

- Map and robot specs in `spec/` are the only source of truth for geometry and robot parameters.
- Everything derived from them (glTF, generated scenes, collision shapes) is generated, never edited by hand.

## Environment

- Maps are described in YAML (corridors, rooms, doors, fixed furniture) under `spec/maps/`.
- Collision geometry is simplified (boxes and primitives) and generated for Godot from the spec.
- Compiled primitives carry a 2D footprint (`center [x, y]`, `size [sx, sy]`, `yaw`) plus explicit `z_min` and `z_max`, so the same compiled map can drive a 2D or 3D backend. See `docs/map-spec.md`.
- Visual geometry is generated by Blender scripts from the same spec and exported as glTF.
- Optional: import an existing occupancy grid (ROS `.pgm` + `.yaml`) and extrude walls, so the simulated map matches a real site.
- Map variants for domain randomization are generated programmatically from specs.

## Robot Model

- Kinematic differential drive: poses are integrated from velocity commands, not from wheel-ground rigid-body physics.
- Simple collision footprint (cylinder or box) defined in `spec/robots/`.
- Robot state is planar: (x, y, yaw).
- The footprint and lidar mounting pose in `spec/robots/` are taken from the robot's URDF. Velocity limits are the configured simulation action range; deployment may narrow it on the inference side. Acceleration limits come from the controller configuration, not the URDF. Values not yet confirmed are marked as provisional in the spec.
- Planar lidar implemented with direct physics-space ray queries (batched where possible), not one node per ray.
- Lidar interface: input is a state snapshot plus the robot pose; output follows ROS LaserScan semantics; it must not assume it runs inside the World process. Lidar rate is configurable and may be lower than the control rate.
- Configurable noise models for odometry drift and range noise.
- Optional: a converter from the real robot's URDF so sensor mounting and dimensions match the hardware.

## Pedestrians and Dynamic Obstacles

- Pedestrians and movable obstacles (carts, beds) are World-owned entities.
- Pedestrian motion uses Godot navigation with simple scripted behaviors (walk to a destination, pause, avoid robots).
- Their behavior is seeded and therefore reproducible.

## Python Interface

- The Python side exposes the environment through the PettingZoo Parallel API.
- The bridge client handles transport, serialization, and lockstep synchronization.

## Repository Layout

```
cadence/
├── AGENTS.md
├── CLAUDE.md
├── docs/
│   ├── coding-standards.md
│   ├── design-context.md
│   ├── map-spec.md
│   └── project-brief.md
├── spec/               # Single source of truth
│   ├── map.schema.yaml # Map source contract
│   ├── maps/
│   └── robots/
├── assets/             # Blender side
│   ├── scripts/        # bpy generation scripts
│   ├── sources/        # Hand-made .blend files (Git LFS)
│   └── build.py        # spec -> glTF, outputs into sim/generated/
├── native/             # C++17 sources, Godot adapter, and CMake build
├── sim/                # Godot project (project.godot lives here)
│   ├── world/
│   ├── robots/
│   ├── bridge/         # Interface layer; the only place frames are converted
│   ├── native/         # Tracked .gdextension descriptor; bin/ is ignored
│   ├── tests/          # Headless Godot test scripts
│   └── generated/      # Generated assets; never edit by hand
├── env/                # Python bridge client and PettingZoo wrapper
├── tools/              # Benchmark tooling; URDF and occupancy-grid converters
└── tests/
```

Rules:

- Blender source files must never be placed inside `sim/`, because Godot imports everything in its project directory.
- `sim/generated/` is committed so the simulation runs without Blender installed, but it is produced only by the build tools under `assets/`.
- `sim/.godot/` is ignored by Git. `.blend` files and large textures use Git LFS.

## Milestones

Each milestone is complete only when its acceptance test runs headless and passes.

1. **Map spec to World**: generate a Godot world from a YAML spec. Test: rays cast from known positions return the expected wall distances.
2. **Single robot kinematics**: one differential-drive robot. Test: driving at 1 m/s for 2 s ends within tolerance of the expected pose.
3. **Lidar**: planar lidar on the robot. Test: in a corridor center, left and right readings match half the corridor width within tolerance. Also delivers the throughput benchmark below.
4. **Bridge and lockstep**: Python client with `step()`. Test: two runs with the same seed produce identical trajectories.
5. **Multi-robot and collisions**: several robots with World-side swept collision checks. Test: a high-speed head-on approach is always detected as a collision.
6. **Pedestrians**: seeded pedestrian agents. Test: same seed reproduces identical pedestrian trajectories.
7. **PettingZoo wrapper**: environment usable by standard multi-agent training code. Test: the PettingZoo API compliance check passes.
8. **Visual assets (optional)**: Blender-generated glTF for visualization and camera-based work.

The World should support saving viewport screenshots in headless mode, so results can be inspected without opening the editor.

Milestones 2 and 3 come before the Ward 73 round. The Ward 73 round may be split into
(a) map spec v1 and its compiler, validated on a synthetic map, then (b) tracing
Ward 73 once a real dimension is measured for scale calibration.

### Milestone 3 throughput benchmark

The stage 1 comparison is complete and Jolt 3D is confirmed. The rules below
document that selection; `docs/benchmark-m3.md` preserves the measured evidence.
Current benchmark runs measure Jolt only.

- **Stress map**: generated in code with ward-like complexity (hundreds of wall rectangles, for example a long corridor lined with rooms), plus moving robots that the lidar must also hit.
- **Stage 1**: compare GodotPhysics2D and Jolt 3D query kernels in GDScript. Only the query kernel is implemented for both backends, not full features.
- **Stage 2**, only if neither backend meets the target in stage 1: analytic ray intersection with a spatial index, implemented in the C++ GDExtension.
- **Reporting**: steps/s, rays/s, and per-instance memory, as curves over robot count (1 to at least 20) and beam count (360, 400, 720; 400 matches the A100 reference scan). Each condition runs 5 times; report the median and the spread. Stage 1 steps/s covers kinematics plus the lidar query kernel; swept collision arrives in Milestone 5, so these numbers are a lower bound on per-step cost.
- **Build**: run headless with the same release build used for training, not the editor's debug binary.
- **Target**: with 5 robots, 360 beams, and a lidar scan every control step, median single-instance throughput is at least 200 steps/s. The report records the machine it was measured on. Total throughput is the per-instance rate times the number of instances across however many machines are used, so the target does not depend on a particular training machine.
- **Parallel scaling**: also report per-instance throughput as the number of concurrent instances on one machine grows toward its physical core count, to show how much instances slow each other down.
- **Decision rule**: a backend "meets the target" when it satisfies the target above.
  - Both meet the target: choose Jolt 3D if its median single-instance throughput is at least 70% of GodotPhysics2D's at both 5 and 20 robots (360 beams); otherwise choose 2D. The 20-robot check guards headroom that pedestrians, more robots, and more beams will consume later.
  - Exactly one meets the target: choose it.
  - Neither meets the target: go to stage 2. Lidar then no longer depends on engine queries, so the engine backend defaults to Jolt 3D.
  - A difference within the measured run-to-run spread counts as equal. If the comparison against the 70% threshold is a tie by this definition, lower per-instance memory decides, because memory limits how many parallel instances fit on one machine.

## Current Implementation

Milestone 1 provides rectangular walls in map spec v0, an engine-independent JSON
compiler, a minimal Godot loader, headless physics ray-distance acceptance tests,
and headless tests that the loader rejects malformed compiled maps.
Milestone 2 adds robot spec v0, its JSON compiler, and batched float64 differential-drive
kinematics implemented as a C++ GDExtension with a thin GDScript entry point. It has
configurable dt, explicit velocity and acceleration limits, and headless acceptance
tests. The integrator uses planar REP-103 state without robot nodes or physics bodies.
See `docs/native-build.md` for native build and loading requirements.
See `docs/map-spec.md` and `docs/robot-spec.md` for the contracts and `README.md` for
compile and test commands. Milestone 3 stage 1 provides snapshot-driven Jolt 3D
lidar queries through C++ GDExtension, with headless geometry, lifecycle, and repeatability tests and an exported-release
throughput and peak-memory benchmark. The 2D query runtime is removed following
confirmation of the selection in `docs/benchmark-m3.md`. Ward 73, stage 2, robot collision handling, noise, and agent
transport remain outside this implementation.

## Open Questions

These are undecided. Do not assume an answer; ask, or implement in a way that keeps the decision open.

- Training paradigm: reinforcement learning with custom observations, or a full ROS 2 / Nav2 stack in the loop?
- Sensors: lidar and odometry only, or also cameras?
- Robot platform: A100. Its URDF and Gazebo xacro files are local references under `tmp/`, not tracked. The lidar model is needed to confirm scan parameters (beam count, angle range, range limits). Controller configuration is needed to confirm the provisional acceleration limits.
- Map source: hand-written specs, an existing occupancy grid, or a floor plan?
- Blender version to pin; broader Python and Godot version compatibility to validate.
- Transport and serialization: TCP or ZeroMQ; JSON or MessagePack.
- Collision termination scope: only the robots involved, or the whole episode?
- Control rate and lidar rate.
