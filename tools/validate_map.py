"""Validates robot configuration-space connectivity and doorway clear widths."""

import argparse
import math
from pathlib import Path
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import unary_union
from assets.build import compile_map, UniqueKeyLoader
from assets.build_robot import compile_robot
from assets.map_geometry import point
import yaml

DEFAULT_CLEARANCE = 0.05
def rectangle(item):
  x, y = item["size"]
  return Polygon(
    [
      point(item["center"], item["yaw"], u * x / 2, v * y / 2)
      for u, v in [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    ]
  )
def radius_for(footprint):
  # Rotating in place about the robot origin sweeps the offset disc out to this radius.
  if footprint["type"] == "circle":
    return footprint["radius"] + math.hypot(footprint["center"]["x"], footprint["center"]["y"])
  raise ValueError("Unsupported robot footprint")
def clear_width(door, occupied):
  line = LineString(
    [point(door["center"], door["yaw"], x, 0) for x in [-door["width"] / 2, door["width"] / 2]]
  )
  free = line.difference(occupied)
  if free.is_empty:
    return 0.0
  if free.geom_type == "LineString":
    return free.length
  return max((part.length for part in free.geoms if part.geom_type == "LineString"), default=0.0)
def validate_geometry(compiled, robot, config):
  radius = radius_for(robot["footprint"])
  clearance = config.get("clearance", DEFAULT_CLEARANCE)
  distance = radius + clearance
  body = robot["body"]
  bounds = rectangle(config["bounds"])
  geometry = [(p, rectangle(p)) for p in compiled["primitives"]]
  obstacles = [
    shape
    for p, shape in geometry
    if p["z_min"] <= body["z_max"] and p["z_max"] >= body["z_min"]
  ]
  occupied = unary_union(obstacles)
  # Circumscribed polygon buffers conservatively approximate the circular robot.
  buffered = unary_union(
    [p.buffer(distance / math.cos(math.pi / 64), quad_segs=16) for p in obstacles]
  )
  free = bounds.difference(buffered)
  points = {
    a["id"]: Point(a["nav_point"])
    for a in compiled["annotations"]
    if a["kind"] in ["point", "door"]
  }
  errors = [
    f"{p['id']}: geometry extends outside validation bounds"
    for p, shape in geometry
    if not bounds.covers(shape)
  ]
  for name, p in points.items():
    if not free.contains(p):
      errors.append(f"{name}: nav_point is not in free space")
  regions = [free] if free.geom_type == "Polygon" else list(getattr(free, "geoms", []))
  start = points.get(config["start"])
  region = next(
    (region for region in regions if start is not None and region.contains(start)), None
  )
  if region is None:
    errors.append("Start is missing or outside free space")
  elif region.intersects(bounds.boundary):
    errors.append(
      "Start free-space region reaches validation bounds; close the validation perimeter"
    )
  for target in config["targets"]:
    if target not in points or region is None or not region.contains(points[target]):
      errors.append(f"{target}: unreachable from start")
  widths = {
    a["id"]: clear_width(a, occupied) for a in compiled["annotations"] if a["kind"] == "door"
  }
  for name, width in widths.items():
    if width < 2 * distance:
      errors.append(f"{name}: clear width {width:.6f} below required {2*distance:.6f}")
  return dict(
    ok=not errors,
    errors=errors,
    clear_widths=widths,
    required_width=2 * distance,
    radius=radius,
    clearance=clearance,
  )
def validate(source):
  """Executes the source validation block using its referenced robot spec."""
  spec = yaml.load(source.read_text(encoding="utf-8"), Loader=UniqueKeyLoader)
  compiled = compile_map(source)
  if "validation" not in spec:
    raise ValueError("Map has no validation block")
  config = spec["validation"]
  robot = compile_robot(source.parent / config["robot"])
  return validate_geometry(compiled, robot, config)
def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("source", type=Path)
  args = parser.parse_args()
  try:
    result = validate(args.source)
    print(f"Required width: {result['required_width']:.3f} m")
    for name, width in result["clear_widths"].items():
      print(f"{name}: clear width {width:.3f} m")
    for error in result["errors"]:
      print(error)
    print("CONNECTIVITY_PASS" if result["ok"] else "CONNECTIVITY_FAIL")
    return 0 if result["ok"] else 1
  except (ValueError, OSError) as error:
    print(f"Validation failed: {error}")
    return 1

if __name__ == "__main__":
  raise SystemExit(main())
