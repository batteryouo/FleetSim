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
- Current scale is 2 to 5 robots on one machine. The design must allow moving agent computation to other processes or machines later.

## Reasoning Behind Key Decisions

### Distributed simulation does not need distributed physics

Games need authoritative servers, prediction, and rollback because players interact physically every frame. In navigation research, robot-robot contact is an event (failure), not dynamics to simulate. Therefore:

- Only collision detection must be authoritative and central. Physics never needs to be distributed.
- Swept checks between consecutive poses prevent missed collisions from coarse steps.
- Error budget: at 0.5 to 1.2 m/s with up to 50 ms staleness, position error is at most about 5 cm, comparable to lidar noise. Synchronization precision is not a practical concern for slow indoor robots.

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

### Planar state, 3D as a view

- Lidar, differential drive, collision checks, and pedestrians are all planar problems. Godot 2D physics is the main performance lever.
- Cameras are undecided. If needed, a 3D scene is rendered from the planar state; robots and pedestrians only have (x, y, yaw), so mirroring is straightforward.
- This only breaks down for true 3D terrain (ramps, stairs), which is rare in indoor navigation.

### Greybox first, Blender later

- This mirrors game-industry level design: blockout and playtesting come first; the art pass comes after gameplay is settled; collision geometry is always a separate, simplified version of the visual model.
- Lidar and odometry gain nothing from visual meshes. Blender work is deferred until cameras or visualization need it.
- If Blender were the source of truth, collision would have to be derived from meshes, which is hard to reproduce, hard to randomize, and hard to verify automatically. The spec stays the single source of truth; Blender is only an output (visual assets) or, optionally, an input editor that exports specs.

### Map pipeline

- Maps are authored in YAML and compiled by Python into primitive-only JSON, because Godot has no built-in YAML parser and because high-level constructs (room rows, catalog references) should be expanded in one place.
- Every primitive carries a height range so the same compiled map can drive the 2D world and a future 3D view.
- A Godot editor plugin may later provide visual editing with round-trip load and export. Specs remain authoritative. Free rectangles were chosen over TileMapLayer because real walls and doors have arbitrary metric sizes.

### Obstacles and the lidar plane

- A 2D lidar only sees what intersects its scan plane. Every obstacle has a footprint and a height range (z_min, z_max).
- Two collision layers: a lidar layer (objects intersecting the scan plane) and a body layer (objects intersecting the robot's height range).
- Known hard cases from the reference site: wall handrails (about 0.7 to 0.9 m high, protruding about 0.08 to 0.10 m; usually invisible to the lidar but able to touch the body), IV poles (only thin wheeled legs at lidar height), carts and wheelchairs (hollow bases), and glass partitions (lidar may pass through or reflect; stored as `lidar_transparent`, effect not yet implemented).
- Movable obstacles are placed randomly within spawn regions for domain randomization.

### Language and performance

- Start with GDScript everywhere: fast iteration, no compile step, and most code is not a bottleneck.
- Isolate hot paths behind stable interfaces: lidar batch queries, swept collision checks, and batched kinematics integration.
- The main risk is call overhead (for example, 5 robots times 360 beams is 1800 engine calls per step), not GDScript arithmetic.
- Add a throughput benchmark at Milestone 3. Only if it misses its target, rewrite the hot paths as a C++ GDExtension. Since walls are rectangles, analytic ray-segment intersection outside the physics engine is another option.
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

- Codex is implementing Milestone 1 only: scaffold, map spec v0, compiler, minimal Godot world loader, and a headless ray-distance test.
- Decisions sent to Codex for this round:
  - References are under `tmp/`, not `tmp/references/`.
  - `docs/` must be tracked; `.gitignore` must be tracked; `tmp/` stays ignored.
  - `sims/` is renamed to `sim/` if it holds nothing worth keeping.
  - Commit locally after tests pass; never push.
- Next round (approved, not started): Ward 73 map using map spec v1 (walls, room_rows with explicit room IDs and per-room overrides, obstacles with height ranges, annotations with nav_point, spawn regions), scale calibration, obstacle catalog, overlay preview under `tmp/`, and a connectivity test (robot radius 0.3 m, every room door reachable from the nurse station).

## Open Questions

- Robot platform details; a URDF exists but is not in the repository yet. Maximum velocity and acceleration must come from controller parameters, not the URDF.
- Lidar mounting height and specifications.
- Whether cameras will be used.
- Policy network design (does not block the environment).
- Transport and serialization, step rate, and sensor rates.
- Whether collision terminates only the involved robots or the whole episode.
- A measured real dimension to replace the provisional map scale.
