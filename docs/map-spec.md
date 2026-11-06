# Map spec v0

## Scope and units

`spec/map.schema.yaml` defines the source contract. Maps use meters and radians
in a right-handed REP-103 frame: x forward, y left, z up. A map contains an integer
`version: 0`, a map `id`, and a nonempty `walls` list. Unknown fields, missing
fields, duplicate YAML keys, non-finite numbers, and duplicate wall IDs are errors.
IDs start with an ASCII letter and contain only ASCII letters, digits, and underscores.

Each wall is a rectangular footprint extruded vertically:

```yaml
version: 0
id: example
walls:
  - id: wall_a
    center: [1.0, 2.0]
    size: [4.0, 0.2]
    yaw: 0.0
    z_min: 0.0
    z_max: 2.5
```

`center` is the footprint center `[x, y]`. `size` gives positive lengths along
the wall's local x and y axes before rotation. `yaw` is a counterclockwise rotation
about +z, viewed from above. `z_min` and `z_max` are absolute heights, with
`z_max > z_min`. Every field is explicit; there are no geometry defaults.

Room rows, obstacles, annotations, spawn regions, calibration, and random placement
are not part of v0. The engine does not contain hospital-specific assumptions.

## Compilation

```powershell
python assets/build.py spec/maps/test_room.yaml
python assets/build.py spec/maps/test_room.yaml --output tmp/test_room.json
```

The default output is `sim/generated/<map id>.json`, independent of the working
directory. An explicit output path is relative to the calling directory. Source
paths are relative to the calling directory. Compilation validates the complete
source before writing output, preserves source wall order, and produces deterministic
JSON without source paths or timestamps. Missing files or invalid input produce a
nonzero exit status. Derived geometry is never edited by hand.

The compiled contract contains `version: 0`, `id`, `frame: "rep103"`, and
`primitives`. Every primitive has `id`, `type: "box"`, `center: [x, y]`,
`size: [length_x, length_y]`, `yaw`, `z_min`, and `z_max`. The footprint is
two-dimensional. The height endpoints are copied directly from the source without
arithmetic and are the authoritative vertical bounds; consumers must not reconstruct
them from a derived center height or box height. All dimensions and poses remain
in REP-103; the compiler does not convert to Godot coordinates or embed reference files.

## World loading

`sim/world/map_loader.gd` validates the compiled version, frame, and primitives
before creating `StaticBody3D` walls with `BoxShape3D` collision geometry and matching
box meshes. It loads one map per instance. A malformed file fails without building
a partial map. Collision bodies explicitly use `MapLoader.WALL_LAYER` (layer 1),
which also supplies the ray test query mask. Primitive IDs must match the source
schema's `^[A-Za-z][A-Za-z0-9_]*$` pattern before being used as node names.
The main scene loads the committed
test map; tests explicitly pass the path of a freshly compiled map. The loader
requires two-component footprints, finite height endpoints, and `z_max > z_min`.
It derives the 3D box height and center only when constructing collision and visual
geometry. Those derived values do not replace the compiled height endpoints.

The minimal loader uses GodotPhysics3D to validate height intervals and rotated
geometry. The source and compiled contracts do not require this physics backend;
the eventual 2D/3D simulation choice remains open.

All REP-103/Godot conversions live in `sim/bridge/frames.gd`:
`(x, y, z) -> (-y, z, -x)`, dimensions `(sx, sy, sz) -> (sy, sz, sx)`, and
REP-103 yaw becomes rotation about Godot +Y with the same sign. This maps forward
to Godot -Z and left to -X. This file is a coordinate adapter, not an agent bridge.

## Milestone 1 acceptance

The test room has interior bounds x = [-4, 6] and y = [-2, 2], with walls spanning
z = [0, 2.5]. From `(0, 0, 1)`, the expected east, west, north, and south distances
are 6, 4, 2, and 2 meters. Additional checks use an offset origin, a diagonal ray,
and a separate wall rotated by pi/6 with a height interval of [0.4, 1.6]. Rays above
and below wall height intervals must miss. These are direct physics queries, not
a robot or lidar implementation.

`tests/test_milestone1.py` compiles the YAML through the CLI and launches Godot
headlessly. Queries execute during a physics callback after the bodies have entered
the physics world. Each hit must identify the correct wall and match its independent
expected distance within 0.001 m. Any unexpected miss, hit, engine script error,
timeout, or unavailable Godot binary fails the test.

A separate headless contract test rejects missing, nonnumeric, zero-width, or
reversed height intervals, three-component footprints, zero footprint dimensions,
duplicate or invalid primitive IDs, wrong frames, and invalid JSON. Each malformed
primitive follows a valid primitive to verify that rejection leaves zero child bodies.

Godot references: [headless command line](https://docs.godotengine.org/en/stable/tutorials/editor/command_line_tutorial.html)
and [physics ray queries](https://docs.godotengine.org/en/stable/tutorials/physics/ray-casting.html).
