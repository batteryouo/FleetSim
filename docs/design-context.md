# Design Context

This document records why Cadence is designed the way it is. It complements `docs/project-brief.md`, which states what the project is. It was distilled from the planning discussion that produced the brief.

If this document and the brief disagree, the brief wins for current requirements; report the conflict so one of them can be fixed.

## Workflow and Roles

- **Planning**: done outside the repository (a separate Claude chat). Decisions flow into `docs/` as documents.
- **Worker**: Codex implements tasks. It follows `AGENTS.md`.
- **Reviewer**: Claude Code reviews Codex's changes. It follows `CLAUDE.md`.
- The human relays tasks, review findings, and decisions between them and has the final say.

## Origin

- The project targets multi-robot navigation research, and later multi-robot dispatch.
- Gazebo felt too heavy when training with several robots. Likely causes (camera/GPU sensors, GUI, per-robot ROS stacks, topic and tf volume) were never profiled; the decision to build a lightweight simulator was made anyway, using a game-development approach.
- The current validation scale is 2 to 5 robots on one machine; larger fleets are an explicit goal. The design must allow moving agent computation to other processes or machines later.

## Reasoning Behind Key Decisions

### Distributed simulation does not need distributed physics

Games need authoritative servers, prediction, and rollback because players interact physically every frame. In navigation research, robot-robot contact is an event (failure), not dynamics to simulate. Therefore:

- Only collision detection must be authoritative and central. Physics never needs to be distributed.
- Swept checks between consecutive poses prevent missed collisions from coarse steps.
- Error budget: at 0.5 to 1.2 m/s with up to 50 ms staleness, position error is at most about 5 cm, comparable to lidar noise. Synchronization precision is not a practical concern for slow indoor robots.

### Distributed sensing is not distributed physics

- Sensing (for example, a lidar scan) reads state and advances none. Any node holding a copy of the static map and a synchronized snapshot of dynamic state (poses, pedestrians, door states) can compute it, similar to game clients that replicate the world.
- Authority does not move: the World still owns state, collision detection, and shared resources. Only the computation of a read-only query is relocated.
- Two interface rules keep this option open at low cost: the lidar takes a state snapshot and a robot pose instead of reaching into World internals, and per-robot noise is derived from `(seed, robot_id, step)` so the result is identical wherever and in whatever order it is computed.

### Why collision is terminal and rollback is truncation

- Without detection, robots continue in physically impossible overlapping states. Observations become invalid and the policy may learn that passing through is acceptable.
- With collision as a terminal event, "rollback" degenerates into truncating the trajectory at the collision step and marking it terminal. No re-simulation and no determinism across clients are needed for this.
- Rollback becomes hard only if collisions are non-terminal (requires deterministic re-simulation), or if agents hold state that cannot be snapshotted (for example, ROS nodes such as AMCL or Nav2 costmaps).

### What research does care about

- **Reproducibility**: same seed, same result. This is why lockstep is the default.
- **Latency as a confounder**: real robots sense each other instantly through lidar. Simulator-induced delay must not leak into observations; communication delay, when studied, is injected explicitly.
- Tasks that would need precise central physics, and are out of scope for now: cooperative transport (for example, pushing a bed together), tight formations, physical interaction with carts or doors, and fast vehicles.
- Shared resources (doors, elevators, narrow passages) need consistent ownership, which is a cheap logic-level lock in the World, not a physics problem.

### No ROS dependency

- Policy portability depends on matching data semantics, not on sharing middleware.
- Deployment uses a thin on-robot adapter: subscribe to scan and odometry, build the observation, run the policy, publish velocity commands.
- Therefore observations and actions must match the semantics of ROS LaserScan, Odometry, and Twist: scan angle range, angular resolution, range limits, rates, noise, velocity and acceleration limits, control rate, and latency.
- Running Nav2 in the loop was rejected for now because it brings back the weight that motivated leaving Gazebo.

### Planar state with the selected Jolt 3D backend

- Decided: robot and pedestrian state is planar, (x, y, yaw). Lidar, differential drive, collision checks, and pedestrians are all planar problems.
- Confirmed: Jolt 3D for geometry and lidar queries. The measured Jolt/2D ratios were 92.94% at 5 robots and 95.95% at 20 robots, both above the 70% threshold; both backends met the 200 steps/s target at 5 robots. See `docs/benchmark-m3.md`.
- 2D backend advantages: likely cheaper broadphase and ray queries; height ranges can be resolved at compile time by assigning each primitive to the lidar layer and/or the body layer from its z interval.
- 3D backend advantages: obstacle height ranges are handled naturally by casting rays at the lidar height; cameras, if needed, render from the same scene without a mirrored copy.
- Either way, the compiled map carries explicit height ranges, so the map pipeline does not depend on the choice.
- The planar state model only breaks down for true 3D terrain (ramps, stairs), which is rare in indoor navigation.

### Greybox first, Blender later

- This mirrors game-industry level design: blockout and playtesting come first; the art pass comes after gameplay is settled; collision geometry is always a separate, simplified version of the visual model.
- Lidar and odometry gain nothing from visual meshes. Blender work is deferred until cameras or visualization need it.
- If Blender were the source of truth, collision would have to be derived from meshes, which is hard to reproduce, hard to randomize, and hard to verify automatically. The spec stays the single source of truth; Blender is only an output (visual assets) or, optionally, an input editor that exports specs.

### Map pipeline

- Maps are authored in YAML and compiled by Python into primitive-only JSON, because Godot has no built-in YAML parser and because high-level constructs (room rows, catalog references) should be expanded in one place.
- Every compiled primitive carries a 2D footprint plus explicit `z_min` and `z_max` copied verbatim from the source, so the same compiled map can drive a 2D or 3D backend. Heights are not reconstructed from a box center and size, because the floating-point round trip would shift boundaries that decide whether an obstacle intersects the lidar plane.
- A Godot editor plugin may later provide visual editing with round-trip load and export. Specs remain authoritative. Free rectangles were chosen over TileMapLayer because real walls and doors have arbitrary metric sizes.

### Obstacles and the lidar plane

- A 2D lidar only sees what intersects its scan plane. Every obstacle has a footprint and a height range (z_min, z_max).
- With a 2D backend, this becomes two collision layers: a lidar layer (objects intersecting the scan plane) and a body layer (objects intersecting the robot's height range). With a 3D backend, rays at the lidar height and a body shape spanning the robot's height range give the same result.
- Known hard cases from the reference site: wall handrails (about 0.7 to 0.9 m high, protruding about 0.08 to 0.10 m; usually invisible to the lidar but able to touch the body), IV poles (only thin wheeled legs at lidar height), carts and wheelchairs (hollow bases), and glass partitions (lidar may pass through or reflect; stored as `lidar_transparent`, effect not yet implemented).
- Movable obstacles are placed randomly within spawn regions for domain randomization.

### Language and performance

- World orchestration and test harnesses use GDScript for fast iteration. Batched kinematics is implemented as a C++17 GDExtension by preference, without a measured performance justification; its native build and loading infrastructure is available for later work.
- Isolate hot paths behind stable interfaces: lidar batch queries, swept collision checks, and batched kinematics integration.
- The main risk is call overhead (for example, 5 robots times 360 beams is 1800 engine calls per step), not GDScript arithmetic.
- The Milestone 3 benchmark selected Jolt engine queries through GDScript; stage 2 native lidar is not required by the measured target. The existing C++ kinematics implementation does not establish a lidar performance result.
- The target is single-instance throughput (200 steps/s at 5 robots and 360 beams), not a total for one machine. Training environments are independent and need no synchronization, so total throughput scales by running more instances, possibly across several machines. A per-instance number is portable across machines and deployment sizes; a per-machine total would depend on which machine trains. The value 200 comes from an earlier aggregate goal of 1,000 steps/s on a machine running about 5 instances after reserving cores for the Python side.
- The 2D/3D comparison is a ratio measured on one machine, so it is largely independent of which machine runs the benchmark.
- The M3 stage 1 release benchmark on the i5-12600K measured 356 steps/s for GodotPhysics2D and 331 steps/s for Jolt at 5 robots and 360 beams. Kinematics plus lidar cost about 1.6 and 1.7 microseconds per ray respectively; these are amortized full-step costs, not isolated engine-call timings. See `docs/benchmark-m3.md` for the measured configuration and spread.
- Analytic ray-rectangle intersection is only competitive with a spatial index and native code; in GDScript the loop itself would be too slow. It therefore belongs to the GDExtension stage, not to the first comparison.
- The decision rule favors 3D unless 2D is clearly faster: 3D handles obstacle height ranges naturally, needs no mirrored scene for cameras, keeps an outdoor option open, and keeps a single geometry representation. The 70% threshold, rather than a looser one, reflects that today's headroom will be consumed by pedestrians, larger fleets, and denser scans.
- If stage 2 is needed, the analytic lidar kernel depends only on the map and a state snapshot, not on Godot. This also suits distributed sensing: a node can compute scans without running the engine. The cost is writing intersection code for each new primitive type, which stays manageable because obstacles are simple primitives.
- Measurements use the release build used for training; the editor binary is a debug build and runs GDScript slower.
- Stage 1 compared only the query kernel on both backends, to avoid implementing every feature twice.
- C# was considered but adds a .NET dependency.

## Reference Site: Ward 73

- Target area: Ward 73 on the 7th floor of a hospital, based on a floor plan and site photos.
- The goal is the layout logic, not an exact replica: corridor topology, widths, room rows, door positions, nurse station area, and elevator/stair core.
- References live in `tmp/` (ignored by Git). They include a hospital floor plan and photos showing staff, so they must never be committed.
- The PDF is vector but flattened: about 25,000 lines and 19,000 curves, uniform line width, no layers. Automatic wall extraction is unreliable, so walls are traced or described parametrically.
- The printed scale (S=1/200) is invalid because the drawing was shrunk to A4. Scale is estimated from labeled room areas as a provisional value until a real dimension is measured.
- Repetitive patient-room rows use parametric `room_rows`; irregular areas use plain walls.
- Annotations (room IDs, nurse station, elevators) carry a separate reachable `nav_point` for later navigation goals and dispatch tasks.

## Current Status

- Milestone 1 is complete: map spec v0, the YAML-to-JSON compiler, the minimal Godot loader (now running on Jolt), headless ray-distance tests, and loader rejection tests.
- Milestone 2 is complete: robot spec v0 for the A100 (circle footprint radius 0.27 m), its compiler, and batched float64 differential-drive kinematics implemented in C++17 through GDExtension, with a thin GDScript entry point and headless tests. See `docs/robot-spec.md`.
- Windows x86_64 passed all 51 tests through M3 stage 1. Arch Linux x86_64 passed the 37-test M1/M2 suite at commit `6ca0d76`, including native `.so` loading and dynamic dependency checks; M3 is not yet validated on Linux. Environment versions and commands are recorded in `docs/native-build.md`. macOS remains unvalidated.
- Commits are local and not pushed.
- References are under `tmp/` (robot files under `tmp/references/A100/`); `docs/` and `.gitignore` are tracked; `tmp/` stays ignored.
- Milestone 3 stage 1 provides the confirmed Jolt 3D snapshot-driven lidar kernel, headless geometry/repeatability tests, and an exported-release benchmark. The 2D runtime is removed; `docs/benchmark-m3.md` preserves the historical comparison and decision rule. No stage 2, noise, collision response, or Python bridge is implemented.
- After Milestone 3: the Ward 73 round, possibly split into (a) map spec v1 on a synthetic map and (b) tracing Ward 73 after scale calibration. Its scope: walls, room_rows with explicit room IDs and per-room overrides, obstacles with height ranges, annotations with nav_point, spawn regions, scale calibration, obstacle catalog, overlay preview under `tmp/`, and a connectivity test (robot radius 0.3 m, every room door reachable from the nurse station).

## Open Questions

- Robot platform details: the A100 URDF and Gazebo xacro files are local references under `tmp/`, not tracked. Footprint and lidar mounting pose come from the URDF. Velocity limits are a chosen simulation action range ([-1, 1] m/s and rad/s, reverse allowed); deployment narrows actions on the inference side if the real robot needs it, so the simulator need not match the controller. Acceleration limits must come from controller parameters, not the URDF; the current values are provisional, derived from Gazebo plugin settings.
- Lidar model, needed for scan parameters (beam count, angle range, range limits).
- Whether cameras will be used.
- Policy network design (does not block the environment).
- Transport and serialization, control rate, and lidar rate.
- Whether collision terminates only the involved robots or the whole episode.
- A measured real dimension to replace the provisional map scale.
