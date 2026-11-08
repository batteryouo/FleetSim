"""Expands compound rectangular obstacle footprints without geometry dependencies."""

from assets.map_geometry import point
def expand_obstacles(spec, catalog):
  entries = {entry["id"]: entry for entry in catalog.get("entries", [])}
  if len(entries) != len(catalog.get("entries", [])):
    raise ValueError("Duplicate catalog id")
  for entry in entries.values():
    if any(part["z_max"] <= part["z_min"] for part in entry["parts"]):
      raise ValueError("Catalog z_max must exceed z_min")
  result = []
  seen = set()
  for obstacle in spec.get("obstacles", []):
    if obstacle["id"] in seen:
      raise ValueError("Duplicate obstacle id")
    seen.add(obstacle["id"])
    if "catalog_id" in obstacle:
      if obstacle["catalog_id"] not in entries:
        raise ValueError("Unknown catalog id")
      shape = entries[obstacle["catalog_id"]]
    else:
      shape = obstacle
    for index, part in enumerate(shape["parts"]):
      if part["z_max"] <= part["z_min"]:
        raise ValueError("Obstacle z_max must exceed z_min")
      result.append(
        dict(
          id=f"{obstacle['id']}_part{index}",
          type="box",
          center=point(obstacle["center"], obstacle["yaw"], *part["center"]),
          size=part["size"],
          yaw=obstacle["yaw"] + part["yaw"],
          z_min=part["z_min"],
          z_max=part["z_max"],
          kind=shape["kind"],
          lidar_transparent=shape["lidar_transparent"],
        )
      )
  return result
