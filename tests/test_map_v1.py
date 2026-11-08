import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import yaml
from assets.build import ROOT, compile_map
def row_spec():
  return dict(
    version=1,
    id="ward",
    walls=[],
    room_rows=[
      dict(
        id="row",
        origin=[0, 2],
        yaw=0,
        side="left",
        room_width=4,
        depth=3,
        wall_thickness=0.2,
        z_min=0,
        z_max=2.5,
        door_width=1,
        door_offset=0,
        rooms=[
          dict(id="room_a"),
          dict(id="room_b", overrides=dict(room_width=5, door_offset=0.5)),
        ],
      )
    ],
  )

class MapV1Tests(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.path = Path(self.temp.name) / "source.yaml"
  def compile(self, spec):
    self.path.write_text(yaml.safe_dump(spec), encoding="utf-8")
    return compile_map(self.path)
  def test_rows_and_overrides_have_hand_computed_geometry(self):
    result = self.compile(row_spec())
    self.assertEqual(len(result["primitives"]), 10)
    parts = {p["id"]: p for p in result["primitives"]}
    self.assertEqual(parts["room_a_front_segment0"]["center"], [0.75, 2])
    self.assertEqual(parts["room_a_front_segment0"]["size"], [1.5, 0.2])
    self.assertEqual(parts["room_a_front_segment1"]["center"], [3.25, 2])
    self.assertEqual(parts["room_b_back"]["center"], [6.5, 5])
    doors = {a["id"]: a for a in result["annotations"] if a["kind"] == "door"}
    self.assertEqual(doors["room_b_door"]["center"], [7, 2])
    self.assertEqual(doors["room_a_door"]["width"], 1)
    rooms = {a["id"]: a for a in result["annotations"] if a["kind"] == "point"}
    for door in doors.values():
      self.assertEqual(door["nav_point"], door["center"])
      self.assertNotEqual(door["nav_point"], rooms[door["room"]]["nav_point"])
  def plain(self):
    return dict(
      version=1,
      id="plain",
      walls=[dict(id="wall", center=[0, 0], size=[10, 0.2], yaw=0, z_min=0, z_max=2)],
      annotations=[
        dict(id="room", kind="point", label="Room", point=[0, 2], nav_point=[0, 2])
      ],
      doors=[dict(id="door", room="room", wall="wall", offset=1, width=2, nav_point=[1, 1])],
    )
  def test_subtractive_door_and_invalid_openings(self):
    spec = self.plain()
    parts = self.compile(spec)["primitives"]
    self.assertEqual([p["center"] for p in parts], [[-2.5, 0], [3.5, 0]])
    self.assertEqual([p["size"][0] for p in parts], [5, 3])
    for offset in [5, 3.9]:
      bad = copy.deepcopy(spec)
      bad["doors"][0]["offset"] = offset
      with self.assertRaises(ValueError):
        self.compile(bad)
    bad = copy.deepcopy(spec)
    bad["doors"].append(dict(bad["doors"][0], id="other", offset=1.5))
    with self.assertRaises(ValueError):
      self.compile(bad)
  def test_recursive_required_unknown_and_nonfinite_fields(self):
    spec = row_spec()
    spec["annotations"] = self.plain()["annotations"]
    spec["walls"] = self.plain()["walls"]
    spec["doors"] = self.plain()["doors"]
    schema = yaml.safe_load((ROOT / "spec/map.schema.yaml").read_text())
    def walk(value, definition, path):
      if "$ref" in definition:
        definition = schema["$defs"][definition["$ref"].split("/")[-1]]
      if "oneOf" in definition:
        definition = next(
          schema["$defs"][d["$ref"].split("/")[-1]]
          for d in definition["oneOf"]
          if schema["$defs"][d["$ref"].split("/")[-1]]["properties"]["kind"]["const"]
          == value["kind"]
        )
      if isinstance(value, dict):
        for key in list(definition.get("required", [])) + ["unexpected"]:
          bad = copy.deepcopy(spec)
          target = bad
          for step in path:
            target = target[step]
          if key == "unexpected":
            target[key] = 1
          else:
            del target[key]
          with self.subTest(path=path, key=key), self.assertRaises(ValueError):
            self.compile(bad)
        for key, child in value.items():
          walk(child, definition["properties"][key], path + [key])
      elif isinstance(value, list):
        for index, child in enumerate(value):
          walk(child, definition["items"], path + [index])
      elif isinstance(value, (float, int)) and not isinstance(value, bool):
        for invalid in [float("nan"), float("inf")]:
          bad = copy.deepcopy(spec)
          target = bad
          for step in path[:-1]:
            target = target[step]
          target[path[-1]] = invalid
          with self.assertRaises(ValueError):
            self.compile(bad)

    walk(spec, schema, [])
  def test_compiled_determinism_and_references(self):
    spec = row_spec()
    a = json.dumps(self.compile(spec))
    b = json.dumps(self.compile(spec))
    self.assertEqual(a, b)
    self.assertNotIn(str(self.path), a)
    for change in ["height", "id", "room"]:
      bad = self.plain()
      if change == "height":
        bad["walls"][0]["z_max"] = 0
      elif change == "id":
        bad["annotations"][0]["id"] = "wall"
      else:
        bad["doors"][0]["room"] = "missing"
      with self.assertRaises(ValueError):
        self.compile(bad)
  def test_loader_annotations_acceptance_and_rejection(self):
    data = self.compile(self.plain())
    data["annotations"].append(
      dict(id="zone", kind="zone", label="Spawn", center=[2, 2], size=[1, 1], yaw=0)
    )
    valid = Path(self.temp.name) / "valid.json"
    valid.write_text(json.dumps(data))
    paths = []
    invalid = []
    for index, annotation in enumerate(data["annotations"]):
      for key, value in annotation.items():
        bad = copy.deepcopy(data)
        del bad["annotations"][index][key]
        invalid.append(bad)
        if isinstance(value, list):
          wrong = [None, [], [1], [1, 2, 3], ["x", 0], [True, 0], [1e999, 0]]
          if key == "size":
            wrong.extend([[0, 1], [1, -1]])
        elif isinstance(value, str):
          wrong = (
            [None, 7, "", "bad/id"] if key in ["id", "room", "kind"] else [None, 7, ""]
          )
        else:
          wrong = [None, "1", True, 1e999] + ([0, -1] if key == "width" else [])
        for replacement in wrong:
          bad = copy.deepcopy(data)
          bad["annotations"][index][key] = replacement
          invalid.append(bad)
      for replacement in [{"unexpected": 1}, {"id": "wall_segment0"}, {"id": "room"}]:
        bad = copy.deepcopy(data)
        bad["annotations"][index].update(replacement)
        if replacement == {"id": annotation["id"]}:
          continue
        invalid.append(bad)
      bad = copy.deepcopy(data)
      bad["annotations"].append(copy.deepcopy(annotation))
      invalid.append(bad)
    bad = copy.deepcopy(data)
    bad["annotations"][1]["room"] = "unknown_room"
    invalid.append(bad)
    for index, bad in enumerate(invalid):
      path = Path(self.temp.name) / f"bad{index}.json"
      path.write_text(json.dumps(bad))
      paths.append(str(path))
    manifest = Path(self.temp.name) / "manifest.json"
    manifest.write_text(json.dumps(paths))
    script = Path(self.temp.name) / "accept.gd"
    script.write_text(
      """extends SceneTree
const Loader = preload("res://world/map_loader.gd")
func _initialize():
  var world = Loader.new()
  var status = world.load_map(OS.get_cmdline_user_args()[0])
  var annotations = world.get_annotations()
  var good = status == OK and annotations.size() == 3 and world.get_child_count() == 2
  annotations.clear()
  good = good and world.get_annotations().size() == 3
  world.free()
  quit(0 if good else 1)
"""
    )
    godot = os.environ["GODOT_BIN"]
    for script_path, args in [
      (str(script), [str(valid)]),
      ("res://tests/height_contract_test.gd", ["--manifest", str(manifest)]),
    ]:
      result = subprocess.run(
        [
          godot,
          "--headless",
          "--path",
          str(ROOT / "sim"),
          "--log-file",
          str(Path(self.temp.name) / "log.txt"),
          "--script",
          script_path,
          "--",
          *args,
        ],
        capture_output=True,
        text=True,
        timeout=60,
      )
      self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
      self.assertNotIn("SCRIPT ERROR", result.stdout + result.stderr)
      if "height_contract" in script_path:
        self.assertIn(
          f"HEIGHT_CONTRACT_PASS: {len(paths)} invalid maps rejected", result.stdout
        )
