"""Renders compiled map footprints and optional raster alignment to a local PNG."""

import argparse
import math
from pathlib import Path
import yaml
from assets.build import compile_map, UniqueKeyLoader
from assets.map_geometry import point
from tools.paths import fixture_output
def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("source", type=Path)
  parser.add_argument("--output", type=Path, required=True)
  args = parser.parse_args()
  output = fixture_output(args.output)
  import matplotlib

  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  from matplotlib.patches import Polygon

  compiled = compile_map(args.source)
  spec = yaml.load(args.source.read_text(encoding="utf-8"), Loader=UniqueKeyLoader)
  reference = spec.get("reference")
  fig, ax = plt.subplots(figsize=(12, 8))
  transform = lambda p: p
  if reference:
    scale = reference["meters_per_unit"]
    yaw = reference["yaw"]
    origin = reference["origin"]
    transform = lambda p: point(
      [0, 0], -yaw, (p[0] - origin[0]) / scale, (p[1] - origin[1]) / scale
    )
    source = args.source.parent / reference.get("source", "__missing__")
    if source.is_file() and source.suffix.lower() in [".png", ".jpg", ".jpeg"]:
      raster = plt.imread(source)
      ax.imshow(raster, extent=[0, raster.shape[1], 0, raster.shape[0]], origin="upper")
    else:
      print("Reference raster unavailable; drawing geometry only")
  for p in compiled["primitives"]:
    sx, sy = p["size"]
    corners = [
      transform(point(p["center"], p["yaw"], u * sx / 2, v * sy / 2))
      for u, v in [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    ]
    ax.add_patch(Polygon(corners, alpha=0.35, color="steelblue" if "kind" in p else "black"))
  for a in compiled["annotations"]:
    if a["kind"] in ["point", "door"]:
      x, y = transform(a["center"] if a["kind"] == "door" else a["nav_point"])
      ax.plot(x, y, "go" if a["kind"] == "door" else "ro", markersize=3)
      ax.annotate(a["id"], (x, y), fontsize=7)
    else:
      sx, sy = a["size"]
      corners = [
        transform(point(a["center"], a["yaw"], u * sx / 2, v * sy / 2))
        for u, v in [(-1, -1), (1, -1), (1, 1), (-1, 1)]
      ]
      ax.add_patch(Polygon(corners, fill=False, edgecolor="orange", linestyle="--"))
  ax.autoscale_view()
  ax.set_aspect("equal")
  ax.grid(alpha=0.2)
  ax.set_xlabel(reference["unit"] if reference else "meters")
  output.parent.mkdir(parents=True, exist_ok=True)
  fig.savefig(output, dpi=140)
  plt.close(fig)
  print(output)

if __name__ == "__main__":
  main()
