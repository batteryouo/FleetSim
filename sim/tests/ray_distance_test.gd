extends SceneTree

const MapLoader = preload("res://world/map_loader.gd")

class PhysicsChecks extends Node:
  const Frames = preload("res://bridge/frames.gd")
  var world: Node3D
  var frames := 0
  var failures := 0
  var checks := 0

  func _physics_process(_delta: float) -> void:
    frames += 1
    if frames < 2:
      return
    set_physics_process(false)
    _check_frames()
    _hit("east", [0, 0, 1], [1, 0, 0], 6.0, "east")
    _hit("west", [0, 0, 1], [-1, 0, 0], 4.0, "west")
    _hit("north", [0, 0, 1], [0, 1, 0], 2.0, "north")
    _hit("south", [0, 0, 1], [0, -1, 0], 2.0, "south")
    _hit("offset_east", [1.5, 0.5, 1], [1, 0, 0], 4.5, "east")
    _hit("offset_north", [1.5, 0.5, 1], [0, 1, 0], 1.5, "north")
    _hit("diagonal", [0, 0, 1], [1, 1, 0], 2.0 * sqrt(2.0), "north")
    _miss("above_room", [0, 0, 3], [1, 0, 0])
    _miss("below_room", [0, 0, -0.2], [1, 0, 0])
    var normal_y := sqrt(3.0) / 2.0
    _hit("rotated_front", [10.5, 4 + 3 * normal_y, 1], [0.5, -normal_y, 0], 2.9, "rotated")
    _hit("rotated_back", [13.5, 4 - 3 * normal_y, 1], [-0.5, normal_y, 0], 2.9, "rotated")
    _miss("below_rotated", [10.5, 4 + 3 * normal_y, 0.2], [0.5, -normal_y, 0])
    _miss("above_rotated", [10.5, 4 + 3 * normal_y, 1.8], [0.5, -normal_y, 0])
    if failures == 0:
      print("MILESTONE_1_PASS: %d rays" % checks)
    get_tree().quit(0 if failures == 0 else 1)

  func _check_frames() -> void:
    if Frames.position_to_godot([1, 0, 0]) != Vector3.FORWARD:
      _fail("REP-103 forward must map to Godot -Z")
    if Frames.position_to_godot([0, 1, 0]) != Vector3.LEFT:
      _fail("REP-103 left must map to Godot -X")
    if Frames.position_to_godot([0, 0, 1]) != Vector3.UP:
      _fail("REP-103 up must map to Godot +Y")
    if Frames.size_to_godot([2, 3, 4]) != Vector3(3, 4, 2):
      _fail("Box dimensions must follow the frame axis mapping")
    var forward := Basis(Vector3.UP, Frames.yaw_to_godot(PI / 2.0)) * Vector3.FORWARD
    if not forward.is_equal_approx(Vector3.LEFT):
      _fail("Positive REP-103 yaw must rotate forward toward left")

  func _ray(origin: Array, direction: Array) -> Dictionary:
    var start := Frames.position_to_godot(origin)
    var end := start + Frames.position_to_godot(direction).normalized() * 20.0
    var query := PhysicsRayQueryParameters3D.create(start, end, MapLoader.WALL_LAYER)
    return world.get_world_3d().direct_space_state.intersect_ray(query)

  func _hit(label: String, origin: Array, direction: Array, expected: float, wall: String) -> void:
    checks += 1
    var result := _ray(origin, direction)
    if result.is_empty():
      _fail(label + ": no hit")
      return
    var distance := Frames.position_to_godot(origin).distance_to(result["position"])
    if abs(distance - expected) > 0.001 or String(result["collider"].name) != wall:
      _fail("%s: expected %s at %.6f m, got %s at %.6f m" % [label, wall, expected, result["collider"].name, distance])
    else:
      print("PASS %s: %.6f m" % [label, distance])

  func _miss(label: String, origin: Array, direction: Array) -> void:
    checks += 1
    if not _ray(origin, direction).is_empty():
      _fail(label + ": expected no hit")
    else:
      print("PASS " + label + ": no hit")

  func _fail(message: String) -> void:
    failures += 1
    push_error(message)

func _initialize() -> void:
  var args := OS.get_cmdline_user_args()
  if args.size() != 2 or args[0] != "--map":
    push_error("Usage: -- --map <compiled-map.json>")
    quit(1)
    return
  var world := MapLoader.new()
  root.add_child(world)
  if world.load_map(args[1]) != OK:
    quit(1)
    return
  var runner := PhysicsChecks.new()
  runner.world = world
  root.add_child(runner)
