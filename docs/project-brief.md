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

Milestone 1 is validated with Python 3.11.6 and Godot 4.7.1 stable (standard build).
Python dependencies are pinned in `requirements.txt`. Blender is not needed yet and
remains unpinned.

Milestones 1 through 3 use 3D physics, selected explicitly in `sim/project.godot`
(currently GodotPhysics3D). This is provisional: the final geometry and query
backend is decided by the Milestone 3 benchmark.

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
├── sim/                # Godot project (project.godot lives here)
│   ├── world/
│   ├── robots/
│   ├── bridge/         # Interface layer; the only place frames are converted
│   ├── tests/          # Headless Godot test scripts
│   └── generated/      # Generated assets; never edit by hand
├── env/                # Python bridge client and PettingZoo wrapper
├── tools/              # URDF and occupancy-grid converters
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

- **Stress map**: generated in code with ward-like complexity (hundreds of wall rectangles, for example a long corridor lined with rooms), plus moving robots that the lidar must also hit.
- **Stage 1**: compare GodotPhysics2D and Jolt 3D query kernels in GDScript. Only the query kernel is implemented for both backends, not full features.
- **Stage 2**, only if stage 1 misses the target: a C++ GDExtension, either batched engine queries or analytic intersection with a spatial index.
- **Reporting**: steps/s, rays/s, and per-instance memory, as curves over robot count (1 to at least 20) and beam count (360, 720). Stage 1 steps/s covers the lidar query kernel only; swept collision arrives in Milestone 5, so these numbers are a lower bound on per-step cost.
- **Build**: run headless with the same release build used for training, not the editor's debug binary.
- **Target**: on the training machine, with 5 robots, 360 beams, and a lidar scan every control step, aggregate throughput across parallel World instances is at least 1,000 steps/s. The report states how many cores the World instances use and how many are reserved for the Python side (bridge, policy inference, trainer); aggregate throughput is measured at that instance count, not one instance per core. Per-instance curves are reported for scalability. A GDExtension may be needed if parallel instances cannot reach the target.
- **Decision rule**: TBD by the human, covering both outcomes: both backends meet the target, or neither does.

## Current Implementation

Milestone 1 provides rectangular walls in map spec v0, an engine-independent JSON
compiler, a minimal Godot loader, headless physics ray-distance acceptance tests,
and headless tests that the loader rejects malformed compiled maps.
Milestone 2 adds robot spec v0, its JSON compiler, and batched float64 differential-drive
kinematics with configurable dt, explicit velocity and acceleration limits, and
headless acceptance tests. The integrator uses planar REP-103 state without robot
nodes or physics bodies.
See `docs/map-spec.md` and `docs/robot-spec.md` for the contracts and `README.md` for
compile and test commands. Ward 73, lidar, robot collision handling, noise, and agent
transport are outside this implementation.

## Open Questions

These are undecided. Do not assume an answer; ask, or implement in a way that keeps the decision open.

- Training paradigm: reinforcement learning with custom observations, or a full ROS 2 / Nav2 stack in the loop?
- Sensors: lidar and odometry only, or also cameras?
- Geometry and query backend: Godot 2D physics, Jolt 3D, or a GDExtension. Robot state is planar either way. Decided by the Milestone 3 benchmark; the decision rule is still open.
- Robot platform: A100. Its URDF and Gazebo xacro files are local references under `tmp/`, not tracked. The lidar model is needed to confirm scan parameters (beam count, angle range, range limits). Controller configuration is needed to confirm the provisional acceleration limits.
- Map source: hand-written specs, an existing occupancy grid, or a floor plan?
- Blender version to pin; broader Python and Godot version compatibility to validate.
- Transport and serialization: TCP or ZeroMQ; JSON or MessagePack.
- Collision termination scope: only the robots involved, or the whole episode?
- Control rate and lidar rate.
