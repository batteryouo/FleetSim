extends Node3D

const Frames = preload("res://bridge/frames.gd")
@export var map_path: String = ""

func _ready() -> void:
  if not map_path.is_empty():
    var status := load_map(map_path)
    if status != OK:
      get_tree().quit(1)

func load_map(path: String) -> Error:
  if has_meta("map_loaded"):
    push_error("A map is already loaded")
    return ERR_ALREADY_IN_USE
  var file := FileAccess.open(path, FileAccess.READ)
  if file == null:
    push_error("Cannot open map: " + path)
    return ERR_FILE_CANT_OPEN
  var parser := JSON.new()
  if parser.parse(file.get_as_text()) != OK:
    push_error("Invalid map JSON: " + parser.get_error_message())
    return ERR_PARSE_ERROR
  var data = parser.data
  if not _valid_map(data):
    push_error("Expected a compiled v0 REP-103 map containing box primitives")
    return ERR_INVALID_DATA
  for primitive in data["primitives"]:
    var body := StaticBody3D.new()
    body.name = primitive["id"]
    var height := float(primitive["z_max"]) - float(primitive["z_min"])
    var center := [primitive["center"][0], primitive["center"][1], float(primitive["z_min"]) + height / 2.0]
    body.position = Frames.position_to_godot(center)
    body.rotation.y = Frames.yaw_to_godot(float(primitive["yaw"]))
    var shape := BoxShape3D.new()
    shape.size = Frames.size_to_godot([primitive["size"][0], primitive["size"][1], height])
    var collision := CollisionShape3D.new()
    collision.shape = shape
    body.add_child(collision)
    var visual := MeshInstance3D.new()
    var mesh := BoxMesh.new()
    mesh.size = shape.size
    visual.mesh = mesh
    body.add_child(visual)
    add_child(body)
  set_meta("map_loaded", true)
  return OK

func _valid_map(data: Variant) -> bool:
  if not data is Dictionary:
    return false
  if data.get("version") != 0 or data.get("frame") != "rep103":
    return false
  if not data.get("id") is String or not data.get("primitives") is Array:
    return false
  if data["primitives"].is_empty():
    return false
  var ids := {}
  for primitive in data["primitives"]:
    if not primitive is Dictionary or primitive.get("type") != "box":
      return false
    var id = primitive.get("id")
    if not id is String or id.is_empty() or ids.has(id):
      return false
    ids[id] = true
    if not _vector2_array(primitive.get("center")) or not _vector2_array(primitive.get("size")):
      return false
    for dimension in primitive["size"]:
      if dimension <= 0:
        return false
    if not _finite_number(primitive.get("yaw")):
      return false
    if not _finite_number(primitive.get("z_min")) or not _finite_number(primitive.get("z_max")):
      return false
    if primitive["z_max"] <= primitive["z_min"]:
      return false
  return true

func _vector2_array(value: Variant) -> bool:
  if not value is Array or value.size() != 2:
    return false
  for component in value:
    if not _finite_number(component):
      return false
  return true

func _finite_number(value: Variant) -> bool:
  return (value is float or value is int) and is_finite(float(value))
