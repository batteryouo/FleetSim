extends RefCounted

const Frames = preload("res://bridge/frames.gd")
const LIDAR_LAYER := 1
var backend: String
var _spec: Dictionary
var _space: RID
var _map_bodies: Array[RID] = []
var _robots := {}
var _shapes: Array = []
var _robot_shape: Shape2D
var _robot_shape_3d: Shape3D
var _cosines := PackedFloat64Array()
var _sines := PackedFloat64Array()

func configure(compiled_map: Dictionary, robot_spec: Dictionary, selected_backend: String) -> Error:
  close()
  if selected_backend not in ["2d", "jolt"] or compiled_map.get("frame") != "rep103":
    push_error("Lidar requires a compiled REP-103 map and backend 2d or jolt")
    return ERR_INVALID_PARAMETER
  if selected_backend == "jolt" and ProjectSettings.get_setting("physics/3d/physics_engine") != "Jolt Physics":
    push_error("The jolt lidar kernel requires Jolt Physics")
    return ERR_UNCONFIGURED
  backend = selected_backend
  _spec = robot_spec.duplicate(true)
  var lidar: Dictionary = _spec["lidar"]
  var increment: float = (lidar["angle_max"] - lidar["angle_min"]) / (lidar["beam_count"] - 1)
  for beam in range(int(lidar["beam_count"])):
    var angle: float = lidar["angle_min"] + beam * increment + lidar["mount"]["yaw"]
    _cosines.append(cos(angle))
    _sines.append(sin(angle))
  if backend == "2d":
    _space = PhysicsServer2D.space_create()
    PhysicsServer2D.space_set_active(_space, true)
    var circle := CircleShape2D.new()
    circle.radius = _spec["footprint"]["radius"]
    _robot_shape = circle
  else:
    _space = PhysicsServer3D.space_create()
    PhysicsServer3D.space_set_active(_space, true)
    var cylinder := CylinderShape3D.new()
    cylinder.radius = _spec["footprint"]["radius"]
    cylinder.height = _spec["body"]["z_max"] - _spec["body"]["z_min"]
    cylinder.margin = 0.0
    _robot_shape_3d = cylinder
  for primitive in compiled_map["primitives"]:
    if backend == "2d":
      if not _visible(primitive):
        continue
      var shape := RectangleShape2D.new()
      shape.size = Vector2(primitive["size"][0], primitive["size"][1])
      _shapes.append(shape)
      var transform := Transform2D(Frames.planar_yaw_to_godot(primitive["yaw"]),
        Frames.planar_point_to_godot(primitive["center"][0], primitive["center"][1]))
      _map_bodies.append(_body_2d(shape.get_rid(), transform, LIDAR_LAYER, false))
    else:
      var shape := BoxShape3D.new()
      var height: float = primitive["z_max"] - primitive["z_min"]
      shape.size = Frames.size_to_godot([primitive["size"][0], primitive["size"][1], height])
      shape.margin = 0.0
      _shapes.append(shape)
      var transform := Transform3D(Basis(Vector3.UP, Frames.yaw_to_godot(primitive["yaw"])),
        Frames.point_to_godot(primitive["center"][0], primitive["center"][1], primitive["z_min"] + height / 2.0))
      _map_bodies.append(_body_3d(shape.get_rid(), transform, false))
  return OK

func _visible(bounds: Dictionary) -> bool:
  var height: float = _spec["lidar"]["mount"]["z"]
  return bounds["z_min"] <= height and height <= bounds["z_max"]

func _body_2d(shape: RID, transform: Transform2D, layer: int, moving: bool) -> RID:
  var body := PhysicsServer2D.body_create()
  PhysicsServer2D.body_set_mode(body, PhysicsServer2D.BODY_MODE_KINEMATIC if moving else PhysicsServer2D.BODY_MODE_STATIC)
  PhysicsServer2D.body_set_collision_layer(body, layer)
  PhysicsServer2D.body_set_collision_mask(body, 0)
  PhysicsServer2D.body_add_shape(body, shape)
  PhysicsServer2D.body_set_state(body, PhysicsServer2D.BODY_STATE_TRANSFORM, transform)
  PhysicsServer2D.body_set_space(body, _space)
  return body

func _body_3d(shape: RID, transform: Transform3D, moving: bool) -> RID:
  var body := PhysicsServer3D.body_create()
  PhysicsServer3D.body_set_mode(body, PhysicsServer3D.BODY_MODE_KINEMATIC if moving else PhysicsServer3D.BODY_MODE_STATIC)
  PhysicsServer3D.body_set_collision_layer(body, LIDAR_LAYER)
  PhysicsServer3D.body_set_collision_mask(body, 0)
  PhysicsServer3D.body_add_shape(body, shape)
  PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_TRANSFORM, transform)
  PhysicsServer3D.body_set_space(body, _space)
  return body

func scan_batch(snapshot: Dictionary) -> Array[Dictionary]:
  var ids: Array = snapshot["ids"]
  var poses: PackedFloat64Array = snapshot["poses"]
  if not _space.is_valid() or snapshot["robot_spec"] != _spec or poses.size() != ids.size() * 3:
    push_error("Lidar snapshot must match the configured robot spec and contain three pose values per id")
    return []
  var seen := {}
  for id in ids:
    if not id is String or seen.has(id):
      push_error("Lidar robot ids must be unique strings")
      return []
    seen[id] = true
  for value in poses:
    if not is_finite(value):
      push_error("Lidar poses must be finite")
      return []
  for id in _robots.keys():
    if not seen.has(id):
      _free_body(_robots[id])
      _robots.erase(id)
  var body_height: float = _spec["body"]["z_max"] - _spec["body"]["z_min"]
  for index in range(ids.size()):
    var x := poses[index * 3]
    var y := poses[index * 3 + 1]
    if backend == "2d":
      var transform := Transform2D(0.0, Frames.planar_point_to_godot(x, y))
      if not _robots.has(ids[index]):
        _robots[ids[index]] = _body_2d(_robot_shape.get_rid(), transform, LIDAR_LAYER if _visible(_spec["body"]) else 0, true)
      else:
        # Kinematic targets wait for a physics tick; snapshots must be queryable now.
        # No physics step runs between the mode changes.
        PhysicsServer2D.body_set_mode(_robots[ids[index]], PhysicsServer2D.BODY_MODE_RIGID)
        PhysicsServer2D.body_set_state(_robots[ids[index]], PhysicsServer2D.BODY_STATE_TRANSFORM, transform)
        PhysicsServer2D.body_set_mode(_robots[ids[index]], PhysicsServer2D.BODY_MODE_KINEMATIC)
    else:
      var transform := Transform3D(Basis.IDENTITY, Frames.point_to_godot(x, y, _spec["body"]["z_min"] + body_height / 2.0))
      if not _robots.has(ids[index]):
        _robots[ids[index]] = _body_3d(_robot_shape_3d.get_rid(), transform, true)
      else:
        # Apply the snapshot immediately instead of queuing a kinematic target.
        PhysicsServer3D.body_set_mode(_robots[ids[index]], PhysicsServer3D.BODY_MODE_RIGID)
        PhysicsServer3D.body_set_state(_robots[ids[index]], PhysicsServer3D.BODY_STATE_TRANSFORM, transform)
        PhysicsServer3D.body_set_mode(_robots[ids[index]], PhysicsServer3D.BODY_MODE_KINEMATIC)
  var scans: Array[Dictionary] = []
  var lidar: Dictionary = _spec["lidar"]
  var mount: Dictionary = lidar["mount"]
  var max_range: float = lidar["range_max"]
  var direct_2d: PhysicsDirectSpaceState2D
  var direct_3d: PhysicsDirectSpaceState3D
  if backend == "2d":
    direct_2d = PhysicsServer2D.space_get_direct_state(_space)
  else:
    direct_3d = PhysicsServer3D.space_get_direct_state(_space)
  for index in range(ids.size()):
    var yaw := poses[index * 3 + 2]
    var c := cos(yaw)
    var s := sin(yaw)
    var x: float = poses[index * 3] + c * mount["x"] - s * mount["y"]
    var y: float = poses[index * 3 + 1] + s * mount["x"] + c * mount["y"]
    var ranges := PackedFloat64Array()
    ranges.resize(_cosines.size())
    var query_2d: PhysicsRayQueryParameters2D
    var query_3d: PhysicsRayQueryParameters3D
    var origin_2d: Vector2
    var origin_3d: Vector3
    if backend == "2d":
      origin_2d = Frames.planar_point_to_godot(x, y)
      query_2d = PhysicsRayQueryParameters2D.new()
      query_2d.from = origin_2d
      query_2d.exclude = [_robots[ids[index]]]
      query_2d.collision_mask = LIDAR_LAYER
      query_2d.hit_from_inside = true
    else:
      origin_3d = Frames.point_to_godot(x, y, mount["z"])
      query_3d = PhysicsRayQueryParameters3D.new()
      query_3d.from = origin_3d
      query_3d.exclude = [_robots[ids[index]]]
      query_3d.collision_mask = LIDAR_LAYER
      query_3d.hit_from_inside = true
    for beam in range(ranges.size()):
      var end_x := x + max_range * (c * _cosines[beam] - s * _sines[beam])
      var end_y := y + max_range * (s * _cosines[beam] + c * _sines[beam])
      var hit: Dictionary
      var distance := INF
      if backend == "2d":
        query_2d.to = Frames.planar_point_to_godot(end_x, end_y)
        hit = direct_2d.intersect_ray(query_2d)
        if not hit.is_empty():
          distance = origin_2d.distance_to(hit["position"])
      else:
        query_3d.to = Frames.point_to_godot(end_x, end_y, mount["z"])
        hit = direct_3d.intersect_ray(query_3d)
        if not hit.is_empty():
          distance = origin_3d.distance_to(hit["position"])
      ranges[beam] = -INF if distance < lidar["range_min"] else (INF if distance > max_range else distance)
    scans.append({"robot_id": ids[index], "angle_min": lidar["angle_min"], "angle_max": lidar["angle_max"],
      "angle_increment": (lidar["angle_max"] - lidar["angle_min"]) / (lidar["beam_count"] - 1),
      "range_min": lidar["range_min"], "range_max": max_range, "ranges": ranges})
  return scans

func _free_body(body: RID) -> void:
  if backend == "2d":
    PhysicsServer2D.free_rid(body)
  else:
    PhysicsServer3D.free_rid(body)

func close() -> void:
  for body in _map_bodies:
    _free_body(body)
  for body in _robots.values():
    _free_body(body)
  _map_bodies.clear()
  _robots.clear()
  _shapes.clear()
  _robot_shape = null
  _robot_shape_3d = null
  _cosines.clear()
  _sines.clear()
  if _space.is_valid():
    _free_body(_space)
    _space = RID()
