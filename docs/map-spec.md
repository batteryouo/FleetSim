# Map spec v1

## Source and compiled contracts

Source YAML and compiled JSON both require integer version 1. Version 0 is
rejected; there is no compatibility mode. Units are meters and radians in REP-103.
IDs match `^[A-Za-z][A-Za-z0-9_]*$`. Unknown fields, missing required fields,
duplicate YAML keys, non-finite numbers, invalid heights, and duplicate output
IDs fail compilation before any output file is written.

The source requires `version`, `id`, and `walls` (possibly empty when room rows
or obstacles supply geometry). Optional blocks are `room_rows`, `doors`,
`annotations`, `catalog`, `obstacles`, `spawn_regions`, `validation`, and
`reference`. Omitted collection blocks are empty. Geometry fields have no defaults.
At least one compiled primitive is required. See `spec/map.schema.yaml` for the
machine-readable contract and `spec/maps/synthetic_ward.yaml` for a full example.

Compiled output contains only `version: 1`, `id`, `frame: rep103`, `primitives`,
and `annotations`. Each primitive is a box with `id`, `type: box`, two-component
`center`, positive two-component `size`, `yaw`, `z_min`, and `z_max`.
Height endpoints are copied without arithmetic and are authoritative.
Obstacle primitives additionally carry `kind` and `lidar_transparent`.
Wall primitives may additionally carry boolean `estimated`, copied from the
source wall and retained on every segment after door subtraction.
Reference paths, calibration, catalog references, validation settings, high-level
room rows, and source paths are never embedded. Output order is deterministic.

## Walls and subtractive doors

A wall has `id`, `center: [x, y]`, `size: [length, thickness]`, `yaw`, `z_min`,
and `z_max`. Sizes are positive, and `z_max > z_min`. Yaw rotates the local x/y
axes counterclockwise around +z. These geometric fields are unchanged from v0.
Optional boolean `estimated` records whether a wall is an estimated proxy.
`true` marks approximate geometry, including inferred thickness or heights;
`false` is an explicit author assertion that the wall is not an estimated proxy.
Omitting the field leaves that status unspecified and does not imply measurement.
The compiler preserves the flag without inserting a default, and the world loader
rejects non-boolean values. It is metadata and does not alter physics or sensing.

An explicit source door has `id`, `room`, `wall`, `offset`, `width`, and
`nav_point`. The owning `room` must name a point annotation. `offset` is a signed
distance along the wall's local x axis from its center; `width` is positive.
Doors cut through the entire wall thickness and height; no lintel or leaf is
created. For walls whose long dimension is local y, author the doorway wall with
length along local x and rotate its yaw accordingly.

For a wall of length L, each door occupies `[offset-width/2, offset+width/2]`.
The compiler sorts intervals, validates them, and emits the remaining segments
as `<wall>_segment0`, `<wall>_segment1`, etc. Uncut walls retain their ID.
Door intervals must be entirely within the wall and must not overlap.

The optional door field `minimum_end_distance` has a positive value in meters.
Its default is wall thickness, using the named
`DEFAULT_DOOR_END_MARGIN_FACTOR = 1.0` in `assets/map_geometry.py`. Each opening
must leave that distance at both original wall ends. A segment preceding an
opening, including between openings, must also meet that opening's minimum.
This prevents zero or near-zero stubs and defaults to a physically meaningful
one-thickness remnant. Door annotations are generated from the same interval
used for subtraction, so declared width and gap geometry share one source.

Example: a 10 m wall centered at zero with a 2 m door at offset 1 leaves
segments `[-5, 0]` and `[2, 5]`, centered at -2.5 and 3.5, of lengths 5 and 3.

## Room rows

Required fields: `id`, `origin`, `yaw`, `side` (`left` or `right`), `room_width`,
`depth`, `wall_thickness`, `z_min`, `z_max`, `door_width`, `door_offset`, and
`rooms`. The rooms list contains explicit `{id: ...}` entries, with optional
`overrides` for `room_width`, `depth`, `door_width`, and `door_offset` only.
Room count is the list length; IDs are never inferred from a numeric range.

The origin is the start of the row's corridor-facing wall centerline. Rooms
advance along local +x, with each width contributing to the next room's start.
Left rooms extend into local +y and right rooms into local -y. Width and depth
measure wall-centerline spacing and must exceed wall thickness. Yaw rotates the
entire row. Each room produces front, back, left, and right walls. Shared sides
may coincide; separate sides allow differing per-room depths. A single generated
door splits each front wall, giving five box primitives per room.

Room annotations use the explicit room ID and an interior center `nav_point`.
Generated doors use `<room>_door`, own that room, and use the actual gap center
for both `center` and `nav_point`. Each generated door is therefore a distinct
navigation target whose opening is checked for free space. Explicit doors keep
their authored nav_point. Overrides do not change IDs.
No furniture or bathroom interiors are inferred.

## Annotations and spawn regions

All annotations have `id` and an explicit `kind`:

| Kind | Other required fields | Meaning |
| --- | --- | --- |
| `point` | `label`, `point`, `nav_point` | Semantic location and a separately reachable navigation goal |
| `door` | `room`, `center`, `width`, `yaw`, `nav_point` | Compiler-generated doorway geometry and navigation goal |
| `zone` | `label`, `center`, `size`, `yaw` | Rectangular zone, with positive size; has no navigation point |

Labels are nonempty strings. Coordinates and yaw are finite. Source `annotations`
accept point and zone records; doors must be declared through source `doors` or
room rows so subtraction cannot be bypassed. `spawn_regions` is a list of zone
records, appended to compiled annotations. It defines placement areas only;
randomized placement and distributions are not implemented.

## Obstacles and catalog

An obstacle is a compound rectangular footprint with independently specified
height intervals for its parts. This representation retains empty space beneath
carts and between thin supports using the existing box primitive contract.
It does not approximate a hollow object with one solid bounding box.

Direct placement requires `id`, `center`, `yaw`, `kind` (`static` or `movable`),
`lidar_transparent` (boolean), and nonempty `parts`. Every part requires local
`center`, positive `size`, local `yaw`, `z_min`, `z_max`, and boolean `estimated`.
The estimate marker covers all numeric dimensions and local placement values in
that part. Heights are absolute, not shifted by placement. Part centers are
rotated and translated; yaws are added. Generated IDs are `<obstacle>_partN`.

Catalog placement uses `id`, `center`, `yaw`, and `catalog_id`. The source
`catalog` path is relative to the map YAML. The catalog requires `version: 1`
and `entries`; entries require `id`, `kind`, `lidar_transparent`, and `parts`
with the same part contract. Catalog IDs must be unique and referenced IDs must
exist. Catalog YAML uses the same duplicate-key and finite-number checks.

`spec/obstacles/catalog.yaml` includes estimated handrails, IV poles with low
wheeled-base proxies, hollow work carts, a counter, an extinguisher, a wheelchair,
and glass partitions. Every catalog part is marked estimated. Glass sets
`lidar_transparent: true`; the flag is stored only and has no sensor effect.
Movable obstacles are static snapshot geometry until placement/motion is added.

## Validation and effective clear width

Optional `validation` requires `robot` (robot YAML path relative to the map),
`start` (annotation ID), `targets` (nonempty ID list), and map `bounds` with
`center`, positive `size`, and `yaw`. Optional `clearance` is nonnegative and
defaults to `DEFAULT_CLEARANCE = 0.05` m in `tools/validate_map.py`.
This block stays in YAML and does not affect compilation.

The validator reads footprint radius and body height from the robot spec. A100
has radius 0.27 m and body interval [0, 0.25] m; these are not provisional.
The current robot schema and validator support circular footprints only. When
non-circular robot footprints are added, their validation radius must be the
circumscribed radius: half the diagonal for rectangles, or the maximum distance
from the robot origin to a polygon vertex. Those branches are not implemented yet.

Shapely is confined to tooling/tests and pinned to 2.1.2. Compile-path modules do
not import it. The validator converts every box intersecting the closed robot
height interval to a polygon, buffers each by radius plus clearance, unions
them, and subtracts the result from the supplied map bounds. Circular buffers
use 16 segments per quadrant and a circumscribed correction `1/cos(pi/64)` to
avoid underestimating the robot envelope; this is conservative by about 0.12%.

Every point/door nav_point must lie strictly in free space. Start and all target
nav_points must lie in the same free-space polygon. Zones are not collected as
navigation points and cannot serve as targets. A handrail entirely above the
robot body is omitted; lowering the same obstacle into the body interval blocks
motion. No robot dimensions are hard-coded into validation.

All unbuffered map primitives, including those above the robot body, must fit
inside the validation bounds. The start's connected free-space polygon must not
touch the bounds boundary: such contact indicates an open perimeter that could
allow false routes through the surrounding exterior. Disconnected exterior
free-space polygons may touch the boundary. Real open entrances require an
explicitly closed validation perimeter before this check can establish indoor
connectivity. The synthetic utility area includes enclosing walls for this reason.
Semantic `point` coordinates may differ from their reachable `nav_point`; only
the latter is used for navigation validation.

Effective door clear width is measured on UNBUFFERED geometry: intersect the
door's centerline span with height-relevant walls/obstacles, subtract those
intersections, and take the longest continuous remaining interval. A centered
0.2 m intrusion in a 1.2 m opening leaves two 0.5 m intervals: width is 0.5 m,
not their sum. An obstacle in front of the span does not reduce this metric.
The door passes the width diagnostic when width is at least twice radius plus
clearance, i.e. `2 * (radius + clearance)`; route connectivity is checked separately
and remains authoritative. A sufficient width alone does not establish a route.
Tests avoid exact threshold equality because zero-width buffered channels are
numerically unstable. Required negative fixtures are clearly below threshold.

## Reference alignment and preview

Optional `reference` requires `meters_per_unit` (positive), `unit` (`pixel` or
`pdf_point`), `origin`, `yaw`, `status` (`provisional`, `derived`, or `measured`),
and a nonempty `source_note`. `source` is an optional path relative to the map.
The alignment is `map_point = origin + R(yaw) * reference_point * meters_per_unit`.
Reference coordinates use x right/y up; raster coordinates are measured from the
bottom-left image edge. Geometry is always authored in meters and is NEVER scaled
by this block. Compilation never opens reference input, even when absent.

The PNG preview uses inverse alignment and optionally a PNG/JPEG background.
Missing or unsupported backgrounds (including PDF input) produce a clear notice
and a geometry-only preview. It draws boxes, door centers, semantic nav_points,
and spawn zones. Outputs must be under ignored tmp/ or OS temporary storage.
Matplotlib 3.8.2 is an optional preview dependency, separate from compilation.

## Commands and loading

```powershell
python -m assets.build spec/maps/synthetic_ward.yaml
python -m tools.validate_map spec/maps/synthetic_ward.yaml
python -m tools.preview_map spec/maps/synthetic_ward.yaml --output tmp/previews/synthetic_ward.png
python -m unittest discover -s tests
```

Run modules from the repository root. Default compile output is
`sim/generated/<id>.json`; `--output` overrides it. Missing/invalid input exits
nonzero without overwriting an existing output. Generated JSON is never hand-edited.

The Jolt world loader validates v1 primitive geometry and all three annotation
kinds, duplicate IDs, and door ownership before creating any nodes. Annotations
create no nodes; `get_annotations()` returns a deep copy. Physics boxes use
`WALL_LAYER = 1`. Height-to-box conversion happens only at construction. Coordinate
conversion stays in `sim/bridge/`: REP-103 `(x,y,z)` maps to `(-y,z,-x)`.
The original test-room geometry and M1 ray expectations are unchanged.

## Source-plan workflow

When a source plan preserves CAD layer markers, a tracked extraction tool can
filter layers and apply reference scale exactly once. Doors come from the door
layer. Manual corrections belong in an explicit overrides file. A scale update
reruns extraction instead of migrating metric coordinates by hand. Generated
site maps remain uncommitted for now. The tool requires a synthetic layered
fixture so its tests do not depend on untracked input. The extraction tool is
implemented in `tools/plan_extract.py`; see `docs/plan-extraction.md` for its
source-unit config, overrides, surface-proxy limitations, and PDF overlays.
Real-site overrides are currently local under tmp/ alongside generated maps.

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
