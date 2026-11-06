"""Generates deterministic benchmark fixtures through the map compiler."""
import json
from pathlib import Path

import yaml

from assets.build import compile_map
from assets.build_robot import compile_robot, ROOT

def wall(identifier, x, y, width, depth, z_min=0.0, z_max=2.5, yaw=0.0):
  return {"id": identifier, "center": [x, y], "size": [width, depth],
          "yaw": yaw, "z_min": z_min, "z_max": z_max}
def compiled(directory, walls, name="fixture"):
  if not walls:
    walls = [wall("outside_scan", 1000, 1000, 1, 1)]
  source = Path(directory) / f"{name}.yaml"
  source.write_text(yaml.safe_dump({"version": 1, "id": name, "walls": walls}), encoding="utf-8")
  result = compile_map(source)
  source.with_suffix(".json").write_text(json.dumps(result, indent=2), encoding="utf-8")
  return result
def stress_map(directory):
  walls = [wall("outer_n", 80, 6.1, 160.4, 0.2), wall("outer_s", 80, -6.1, 160.4, 0.2),
           wall("end_w", -0.1, 0, 0.2, 12), wall("end_e", 160.1, 0, 0.2, 12)]
  for side, sign in [("n", 1), ("s", -1)]:
    for index in range(41):
      walls.append(wall(f"partition_{side}_{index}", index * 4, sign * 4.1, 0.2, 4))
    for index in range(40):
      x = index * 4
      walls.extend([wall(f"front_a_{side}_{index}", x + 0.8, sign * 2.1, 1.6, 0.2),
                    wall(f"front_b_{side}_{index}", x + 3.3, sign * 2.1, 1.4, 0.2),
                    wall(f"off_plane_{side}_{index}", x + 2, sign * 0.8, 0.3, 0.3,
                         *([0.5, 1.0] if index % 2 else [0.0, 0.1]))])
  return compiled(directory, walls, "m3_stress")
def robot_spec():
  return compile_robot(ROOT / "spec/robots/a100.yaml")
def poses(count, phase=0.0):
  return [value for index in range(count) for value in
          (3.0 + 3.6 * index + phase, (-0.25 if index % 2 else 0.25), 0.0)]
