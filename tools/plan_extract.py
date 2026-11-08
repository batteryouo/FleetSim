"""Extracts metric map v1 geometry from layered PDF paths and explicit source-unit overrides."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import tempfile

import yaml
from jsonschema import Draft202012Validator
from shapely.geometry import LineString, box

from assets.build import ROOT, UniqueKeyLoader, _check_finite, compile_map
from assets.map_geometry import point
from tools.benchmark_m3 import fixture_output
def read_yaml(path):
  value = yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueKeyLoader)
  _check_finite(value)
  return value
def validate_inputs(config, overrides):
  schema = yaml.safe_load((ROOT / "spec/extraction.schema.yaml").read_text(encoding="utf-8"))
  for name, value in [("config", config), ("overrides", overrides)]:
    _check_finite(value)
    error = next(
      Draft202012Validator({"$ref": f"#/$defs/{name}", "$defs": schema["$defs"]}).iter_errors(
        value
      ),
      None,
    )
    if error:
      raise ValueError(f"Invalid {name}: {error.message}")
  if config["crop"][0] >= config["crop"][2] or config["crop"][1] >= config["crop"][3]:
    raise ValueError("Crop bounds must be strictly ordered")
  for bounds in overrides.get("exclude_regions", []):
    if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
      raise ValueError("Excluded region bounds must be strictly ordered")
  if config["z_min"] >= config["z_max"]:
    raise ValueError("z_max must exceed z_min")
def pdf_paths(source, page_number):
  """Returns normalized paths in PDF page coordinates (x right, y down)."""
  if not source.is_file():
    raise ValueError(f"Source PDF does not exist: {source}")
  import pymupdf

  try:
    document = pymupdf.open(source)
  except RuntimeError as error:
    raise ValueError(f"Cannot read source PDF: {error}") from error
  with document:
    if page_number < 0 or page_number >= len(document):
      raise ValueError("PDF page index is out of range")
    page = document[page_number]
    paths = []
    for drawing in page.get_drawings():
      segments = []
      curves = []
      subpath_start = None
      last_end = None
      for item in drawing["items"]:
        if item[0] == "l":
          if last_end != list(item[1]):
            subpath_start = list(item[1])
          segments.append([list(item[1]), list(item[2])])
          last_end = list(item[2])
        elif item[0] == "re":
          r = item[1]
          corners = [list(r.tl), list(r.tr), list(r.br), list(r.bl)]
          segments.extend([[corners[i], corners[(i + 1) % 4]] for i in range(4)])
          subpath_start = last_end = corners[0]
        elif item[0] == "qu":
          q = item[1]
          corners = [list(q.ul), list(q.ur), list(q.lr), list(q.ll)]
          segments.extend([[corners[i], corners[(i + 1) % 4]] for i in range(4)])
          subpath_start = last_end = corners[0]
        elif item[0] == "c":
          if last_end != list(item[1]):
            subpath_start = list(item[1])
          curves.append([list(p) for p in item[1:]])
          last_end = list(item[4])
        else:
          raise ValueError(f"Unsupported PDF path item: {item[0]}")
      if (
        drawing.get("closePath", False)
        and subpath_start is not None
        and last_end != subpath_start
      ):
        segments.append([last_end, subpath_start])
      paths.append(
        dict(
          id=f"path{drawing['seqno']}",
          layer=drawing.get("layer", ""),
          width=drawing.get("width", 0),
          segments=segments,
          curves=curves,
          closed=False,
        )
      )
    return paths
def curve_points(curve, subdivisions):
  result = []
  for i in range(subdivisions + 1):
    t = i / subdivisions
    weights = [(1 - t) ** 3, 3 * (1 - t) ** 2 * t, 3 * (1 - t) * t * t, t**3]
    result.append([sum(weights[j] * curve[j][axis] for j in range(4)) for axis in range(2)])
  return result
def door_candidates(paths, layers, tolerance):
  """Finds quarter-circle swings and uses their leaf segment to resolve the closed opening."""
  selected = [p for p in paths if p["layer"] in layers]
  leaves = [segment for p in selected for segment in p["segments"]]
  result = []
  for path in selected:
    for index, curve in enumerate(path["curves"]):
      a, b, c, d = curve
      v = [b[i] - a[i] for i in range(2)]
      w = [d[i] - c[i] for i in range(2)]
      det = v[0] * w[1] - v[1] * w[0]
      if abs(det) < 1e-10:
        continue
      # Center is the intersection of the endpoint tangent normals.
      k = ((d[0] - a[0]) * w[0] + (d[1] - a[1]) * w[1]) / det
      center = [a[0] - v[1] * k, a[1] + v[0] * k]
      radius = math.dist(center, a)
      if radius <= tolerance or abs(math.dist(center, d) - radius) > tolerance:
        continue
      if (
        abs(sum((a[i] - center[i]) * (d[i] - center[i]) for i in range(2)))
        > tolerance * radius
      ):
        continue
      endpoints = []
      for end in [a, d]:
        match = min(
          (
            min(
              math.dist(s, center) + math.dist(e, end),
              math.dist(e, center) + math.dist(s, end),
            )
            for s, e in leaves
          ),
          default=math.inf,
        )
        endpoints.append(match)
      if min(endpoints) > 2 * tolerance or abs(endpoints[0] - endpoints[1]) < tolerance:
        continue
      closed_end = d if endpoints[0] < endpoints[1] else a
      result.append(
        dict(
          id=f"{path['id']}_arc{index}",
          start=center,
          end=closed_end,
          alternate_end=a if closed_end is d else d,
          width=radius,
        )
      )
  return result
def extract(paths, config, overrides):
  """Builds source YAML fields; config and override coordinates are unscaled PDF page units."""
  validate_inputs(config, overrides)
  layers = {p["layer"] for p in paths}
  missing = set(config["wall_layers"] + config["door_layers"]) - layers
  if missing:
    raise ValueError(f"Missing requested PDF layers: {sorted(missing)}")
  scale = config["meters_per_unit"]
  if scale <= 0 or config["curve_subdivisions"] < 2 or config["line_width"] <= 0:
    raise ValueError("Scale, line width, and curve resolution must be positive")
  origin = config["origin"]
  yaw = config["yaw"]
  transform = lambda p: point([0, 0], yaw, (p[0] - origin[0]) * scale, (origin[1] - p[1]) * scale)
  roi = box(*config["crop"])
  exclusions = [box(*r) for r in overrides.get("exclude_regions", [])]
  walls = []
  skipped = set(overrides.get("remove_paths", []))
  available = {p["id"] for p in paths}
  if skipped - available:
    raise ValueError(f"Unknown removed path IDs: {sorted(skipped-available)}")
  for path in paths:
    if path["layer"] not in config["wall_layers"] or path["id"] in skipped:
      continue
    segments = list(path["segments"])
    for curve in path["curves"]:
      samples = curve_points(curve, config["curve_subdivisions"])
      segments.extend(zip(samples, samples[1:]))
    if path["closed"] and segments and math.dist(segments[-1][1], segments[0][0]) > 1e-6:
      segments.append([segments[-1][1], segments[0][0]])
    for index, (a, b) in enumerate(segments):
      if math.dist(a, b) < 1e-8:
        continue
      clipped = LineString([a, b]).intersection(roi)
      if any(clipped.intersects(region) for region in exclusions):
        for region in exclusions:
          clipped = clipped.difference(region)
      pieces = (
        [clipped]
        if clipped.geom_type == "LineString"
        else list(getattr(clipped, "geoms", []))
      )
      for piece_index, piece in enumerate(pieces):
        if piece.geom_type != "LineString" or piece.is_empty or piece.length < 1e-8:
          continue
        start, end = [transform(list(p)) for p in [piece.coords[0], piece.coords[-1]]]
        width = max(path["width"], config["line_width"]) * scale
        walls.append(
          dict(
            id=f"{path['id']}_edge{index}_piece{piece_index}",
            center=[(start[i] + end[i]) / 2 for i in range(2)],
            size=[math.dist(start, end), width],
            yaw=math.atan2(end[1] - start[1], end[0] - start[0]),
            z_min=config["z_min"],
            z_max=config["z_max"],
          )
        )
  for wall in overrides.get("walls", []):
    a, b = [transform(p) for p in [wall["start"], wall["end"]]]
    walls.append(
      dict(
        id=wall["id"],
        center=[(a[i] + b[i]) / 2 for i in range(2)],
        size=[math.dist(a, b), wall["width"] * scale],
        yaw=math.atan2(b[1] - a[1], b[0] - a[0]),
        z_min=wall.get("z_min", config["z_min"]),
        z_max=wall.get("z_max", config["z_max"]),
      )
    )
  annotations = [
    dict(
      id=a["id"],
      kind="point",
      label=a["label"],
      point=transform(a["point"]),
      nav_point=transform(a["nav_point"]),
    )
    for a in overrides["annotations"]
  ]
  candidates = door_candidates(paths, config["door_layers"], config["door_tolerance"])
  lookup = {d["id"]: d for d in candidates}
  doors = []
  for door in overrides["doors"]:
    if door["candidate"] not in lookup:
      raise ValueError(f"Door candidate not found: {door['candidate']}")
    candidate = lookup[door["candidate"]]
    end = candidate["alternate_end"] if door.get("alternate", False) else candidate["end"]
    if not roi.covers(LineString([candidate["start"], end])):
      raise ValueError(f"Door candidate lies outside crop: {door['candidate']}")
    a, b = [transform(p) for p in [candidate["start"], end]]
    center = [(a[i] + b[i]) / 2 for i in range(2)]
    width = math.dist(a, b)
    thickness = config["line_width"] * scale
    wall_id = door["id"] + "_jambs"
    angle = math.atan2(b[1] - a[1], b[0] - a[0])
    # A source opening already has a gap. A virtual wall makes its door annotation
    # use the compiler's subtraction contract, retaining only two short jamb stubs.
    walls.append(
      dict(
        id=wall_id,
        center=center,
        size=[width + 2 * thickness, thickness],
        yaw=angle,
        z_min=config["z_min"],
        z_max=config["z_max"],
      )
    )
    doors.append(
      dict(
        id=door["id"],
        room=door["room"],
        wall=wall_id,
        offset=0,
        width=width,
        minimum_end_distance=thickness / 2,
        nav_point=center,
      )
    )
  spec = dict(version=1, id=config["id"], walls=walls, doors=doors, annotations=annotations)
  bounds = overrides["validation_bounds"]
  spec["validation"] = dict(
    robot=config["robot"],
    clearance=config["clearance"],
    start=overrides["start"],
    targets=overrides["targets"],
    bounds=dict(
      center=transform(bounds["center"]), size=[v * scale for v in bounds["size"]], yaw=yaw
    ),
  )
  return spec, dict(paths=len(paths), walls=len(walls), door_candidates=candidates)
def write_map(source, config_path, overrides_path, output):
  output = fixture_output(output)
  if output.suffix.lower() not in [".yaml", ".yml"]:
    raise ValueError("Generated map output must be YAML")
  if output in [path.resolve() for path in [source, config_path, overrides_path]]:
    raise ValueError("Generated output must not overwrite an input")
  config = read_yaml(config_path)
  overrides = read_yaml(overrides_path)
  validate_inputs(config, overrides)
  if (
    "source_sha256" in config
    and hashlib.sha256(source.read_bytes()).hexdigest() != config["source_sha256"]
  ):
    raise ValueError("Source PDF hash does not match the extraction config")
  paths = pdf_paths(source, config["page"])
  spec, report = extract(paths, config, overrides)
  # Validation paths must remain valid when output is relocated under tmp/.
  spec["validation"]["robot"] = str((config_path.parent / config["robot"]).resolve())
  output.parent.mkdir(parents=True, exist_ok=True)
  with tempfile.TemporaryDirectory(prefix="plan-check-", dir=output.parent) as workspace:
    temporary = Path(workspace) / "checking.yaml"
    contents = yaml.safe_dump(spec, sort_keys=False)
    temporary.write_text(contents, encoding="utf-8", newline="\n")
    compile_map(temporary)
    output.write_text(contents, encoding="utf-8", newline="\n")
  return report
def render_overlay(source, config, generated, output):
  """Renders the original PDF crop with extracted metric geometry transformed back to page units."""
  import pymupdf
  import matplotlib

  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  import numpy as np
  from matplotlib.patches import Polygon
  from tools.validate_map import rectangle

  compiled = compile_map(generated)
  with pymupdf.open(source) as document:
    crop = pymupdf.Rect(config["crop"])
    raster = document[config["page"]].get_pixmap(matrix=pymupdf.Matrix(3, 3), clip=crop)
    image = np.frombuffer(raster.samples, dtype="uint8").reshape(
      raster.height, raster.width, raster.n
    )
  fig, ax = plt.subplots(figsize=(18, 12))
  ax.imshow(image, extent=[crop.x0, crop.x1, crop.y1, crop.y0], alpha=0.5)
  def to_page(p):
    x, y = point([0, 0], -config["yaw"], p[0], p[1])
    return [
      x / config["meters_per_unit"] + config["origin"][0],
      config["origin"][1] - y / config["meters_per_unit"],
    ]

  for primitive in compiled["primitives"]:
    corners = [to_page(p) for p in rectangle(primitive).exterior.coords]
    ax.add_patch(Polygon(corners, color="royalblue", alpha=0.4))
  for a in compiled["annotations"]:
    p = to_page(a["center"] if a["kind"] == "door" else a["nav_point"])
    ax.plot(*p, "go" if a["kind"] == "door" else "ro", markersize=2)
    if a["kind"] == "point":
      ax.annotate(a["id"], p, fontsize=6)
  ax.set_xlim(crop.x0, crop.x1)
  ax.set_ylim(crop.y1, crop.y0)
  ax.set_aspect("equal")
  ax.set_xlabel("PDF page units; blue: geometry, green: doors, red: navigation points")
  output.parent.mkdir(parents=True, exist_ok=True)
  fig.savefig(output, dpi=160, bbox_inches="tight")
  plt.close(fig)
def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("source", type=Path)
  parser.add_argument("--config", type=Path, required=True)
  parser.add_argument("--overrides", type=Path, required=True)
  parser.add_argument("--output", type=Path, required=True)
  parser.add_argument("--inventory", type=Path)
  parser.add_argument("--overlay", type=Path)
  args = parser.parse_args()
  try:
    output = fixture_output(args.output)
    inventory = fixture_output(args.inventory) if args.inventory else None
    overlay = fixture_output(args.overlay) if args.overlay else None
    outputs = [p for p in [output, inventory, overlay] if p is not None]
    if len(set(outputs)) != len(outputs) or set(outputs).intersection(
      p.resolve() for p in [args.source, args.config, args.overrides]
    ):
      raise ValueError("Outputs must be distinct and must not overwrite inputs")
    if inventory and inventory.suffix.lower() != ".json":
      raise ValueError("Inventory output must be JSON")
    if overlay and overlay.suffix.lower() != ".png":
      raise ValueError("Overlay output must be PNG")
    report = write_map(args.source, args.config, args.overrides, output)
    if args.inventory:
      inventory.parent.mkdir(parents=True, exist_ok=True)
      inventory.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if overlay:
      render_overlay(args.source, read_yaml(args.config), output, overlay)
    print(
      f"Extracted {report['walls']} walls; {len(report['door_candidates'])} door candidates"
    )
    return 0
  except (OSError, ValueError, KeyError, ImportError) as error:
    print(f"Extraction failed: {error}")
    return 1

if __name__ == "__main__":
  raise SystemExit(main())
