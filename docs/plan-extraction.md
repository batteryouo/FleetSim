# Layered source-plan extraction

`tools/plan_extract.py` reads a layered PDF and generates a metric map v1 YAML.
It is tooling, independent of the runtime and the map compiler. Install its
dependencies with `python -m pip install -r requirements-extraction.txt`.
That file pins PyMuPDF 2.0 and Matplotlib 3.8.2 in addition to the base dependencies.
The complete Python test suite requires these extraction dependencies.

## Inputs and coordinates

The config and overrides YAML contracts are in `spec/extraction.schema.yaml`.
Unknown keys, missing required fields, duplicate YAML keys, and non-finite values
are rejected. Input PDFs may remain local; tests use a small, independently
authored layered fixture and a mocked PDF adapter, with no real-site dependency.

All source coordinates are page coordinates: x right and y down, in PDF points.
`origin` identifies the page point mapped to map (0, 0). For source point (u, v),
origin (u0, v0), scale s, and counterclockwise map yaw q:

```text
map_point = R(q) * [(u - u0) * s, (v0 - v) * s]
```

Scale applies in extraction exactly once. Generated YAML is already in meters;
assets.build never applies scale. Changing scale means rerunning extraction.
Height ranges and clearance are already meters and are not scaled. Overlay
rendering applies the inverse transform to compare with the original PDF crop.

Required config fields:

| Field | Meaning |
| --- | --- |
| `id`, `page` | Map ID and zero-based page index |
| `meters_per_unit`, `origin`, `yaw` | Positive scale and source-to-map transform |
| `crop` | Ordered page rectangle `[xmin, ymin, xmax, ymax]` |
| `wall_layers`, `door_layers` | Exact nonempty layer name lists; missing requested layers fail |
| `line_width` | Positive minimum proxy width in source units |
| `curve_subdivisions` | Integer 2-256 for cubic curve tessellation |
| `door_tolerance` | Positive source-unit tolerance for swing/leaf matching |
| `z_min`, `z_max` | Default absolute proxy height endpoints in meters |
| `robot` | Robot YAML path, relative to config location |
| `clearance` | Nonnegative robot clearance in meters |

Optional `source_sha256` locks the extraction to a particular PDF. Source path
IDs use the PDF drawing sequence number; if a new export changes those IDs,
review the inventory and manual choices before updating the hash.

## Geometry and doors

The PDF adapter normalizes line, rectangle, quadrilateral, and cubic path items.
The selected wall-layer paths become narrow rectangular surface proxies, clipped
to the crop. Curves become a configurable number of short straight segments.
Widths are the maximum of the PDF stroke width and configured minimum, scaled
to meters. Separate contour faces can remain separate primitives. Filled areas
are represented by their contours, not reconstructed solid wall volumes.
This is a first-draft geometric proxy: it preserves drawn boundaries and gaps,
but it is not a wall-thickness reconstruction or a general CAD importer.

Door layers supply candidate openings rather than blocking geometry. A candidate
must have a quarter-circle cubic swing with matching endpoint radii and a nearby
radial leaf segment. Its hinge comes from the endpoint tangent normals. A missing
or ambiguous leaf is not guessed. Candidate IDs are `<path-id>_arcN`.

Overrides explicitly select candidates and assign owning-room annotations. For
an unusual leaf convention, `alternate: true` chooses the other endpoint of the
same source swing. Both choices still derive their width from the source radius.
Doors outside the crop fail. Each selected opening is passed through map v1's
subtractive-door compiler using a virtual wall; only two short jamb stubs remain.
The minimum stub requirement is half the proxy thickness to avoid subtraction
roundoff at exact endpoints. The door nav_point is its gap center.

The candidate inventory needs human review: curved furniture or unusual door
symbols can resemble a swing. Unselected candidates do not create annotations.
Connectivity validation does not establish that the source interpretation is
correct, so visually review the overlay and record interpretation assumptions.

## Overrides

Required fields are `annotations`, `doors`, `validation_bounds`, `start`, and
`targets`. Annotation entries provide `id`, `label`, semantic `point`, and
reachable `nav_point` in page coordinates. Door entries provide `id`, `room`,
and `candidate`, with optional boolean `alternate`. Validation bounds provide
source-unit `center` and positive `size` and rotate with the map transform.

Optional blocks:

- `remove_paths`: exact source path IDs to exclude; unknown IDs fail.
- `exclude_regions`: ordered page rectangles to remove adjacent areas. A line
  crossing an exclusion is split and the remaining pieces retained.
- `walls`: explicit repair or boundary segments with `id`, `start`, `end`, and
  positive `width` in page units. Optional `z_min`/`z_max` override metric heights.
- `notes`: English interpretation notes, kept in the authored overrides only.

Use explicit boundary segments to close the simulation scope at adjacent-area
connections. Label the adjacent connection with a point annotation. These caps
represent simulation limits, not claims that a physical wall exists. Validate
the bounded region using tools.validate_map; do not allow exterior bypasses.
Window/glazing layers may require separate selection to close the perimeter;
assuming a full-height proxy must be documented when sill heights are unknown.

Manual overrides are authored inputs, distinct from generated YAML. Real-site
inputs, config, overrides, inventory, map, compiled JSON, and overlay currently
remain under ignored tmp/. Only the general tool/schema and synthetic fixtures
are tracked. No real-site coordinates, layer names, or calibration numbers are
needed by the tracked tool or its tests.

## Commands and outputs

```powershell
python -m tools.plan_extract tmp/source.pdf --config tmp/plan.config.yaml --overrides tmp/plan.overrides.yaml --output tmp/plan.yaml --inventory tmp/plan.inventory.json --overlay tmp/plan-overlay.png
python -m assets.build tmp/plan.yaml --output tmp/plan.json
python -m tools.validate_map tmp/plan.yaml
```

All extractor outputs must be in tmp/ or OS temporary storage. Output paths must
be distinct from inputs and from one another. The output map is checked by the
map compiler before replacing its contents; invalid inputs preserve an existing
map. Its robot path is resolved to an absolute path for local validation, but
that path is stripped by assets.build. The compiled JSON contains no source PDF,
paths, scale, or extraction metadata.

`--inventory` writes source/retained path counts and door candidates. `--overlay`
renders the actual PDF crop with blue geometry, green doors, and red navigation
points. Missing PDFs, missing layers, stale path/candidate IDs, invalid page
indices, and mismatched source hashes fail with a diagnostic and nonzero exit.

Run `python -m unittest discover -s tests` with GODOT_BIN set for the complete
suite, including earlier milestones. Linux extraction/native-lidar validation
must be repeated on the target environment; the older Arch record covers M1/M2.
