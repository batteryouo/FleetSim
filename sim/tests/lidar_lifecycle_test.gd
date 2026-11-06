extends SceneTree

const Lidar = preload("res://sensors/lidar_kernel.gd")
const Frames = preload("res://bridge/frames.gd")
var failures := 0

func _initialize() -> void:
  call_deferred("_run")

func _check(condition: bool, message: String) -> void:
  if not condition:
    failures += 1
    push_error(message)

func _run() -> void:
  var robot: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://generated/robots/a100.json"))
  robot["lidar"]["beam_count"] = 3
  robot["lidar"]["angle_min"] = -PI / 2
  robot["lidar"]["angle_max"] = PI / 2
  var layout := {"frame": "rep103", "primitives": [{"center": [3, 0], "size": [0.2, 10], "yaw": 0, "z_min": 0, "z_max": 2}]}
  var snapshot := {"ids": ["r0"], "poses": PackedFloat64Array([0, 0, 0]), "robot_spec": robot}
  var kernel := Lidar.new()
  _check(kernel.configure(layout, robot) == OK, "Initial configure failed")
  var native: RefCounted = ClassDB.instantiate("CadenceLidar")
  _check(native.call("configure", layout, robot, Frames.basis_to_godot()) == OK, "Native configure failed")
  await physics_frame
  var initial := kernel.scan_batch(snapshot)
  _check(initial.size() == 1 and abs(initial[0]["ranges"][1] - 2.7) < 0.00001, "Initial geometry mismatch")
  for invalid in [
    {},
    {"ids": ["r0", "r0"], "poses": PackedFloat64Array([0, 0, 0, 1, 0, 0]), "robot_spec": robot},
    {"ids": [4], "poses": PackedFloat64Array([0, 0, 0]), "robot_spec": robot},
    {"ids": ["r0"], "poses": PackedFloat64Array([0, 0]), "robot_spec": robot},
    {"ids": ["r0"], "poses": PackedFloat64Array([NAN, 0, 0]), "robot_spec": robot},
    {"ids": ["r0"], "poses": PackedFloat64Array([INF, 0, 0]), "robot_spec": robot},
    {"ids": ["r0"], "poses": [0, 0, 0], "robot_spec": robot},
    {"ids": ["r0"], "poses": PackedFloat64Array([0, 0, 0]), "robot_spec": {}}
  ]:
    _check(native.call("scan_batch", invalid).is_empty(), "Invalid snapshot was accepted")
    _check(native.call("scan_batch", snapshot)[0]["ranges"].to_byte_array() == initial[0]["ranges"].to_byte_array(), "Invalid snapshot changed state")
  var empty := {"ids": [], "poses": PackedFloat64Array(), "robot_spec": robot}
  _check(native.call("scan_batch", empty).is_empty(), "Empty batch returned scans")
  var broken := layout.duplicate(true)
  broken["primitives"].append({"center": [5, 0], "size": [0, 1], "yaw": 0, "z_min": 0, "z_max": 2})
  _check(native.call("configure", broken, robot, Frames.basis_to_godot()) != OK, "Malformed second primitive accepted")
  _check(native.call("scan_batch", snapshot).is_empty(), "Failed configure left a partial space")
  var bad_robot := robot.duplicate(true)
  bad_robot["lidar"]["beam_count"] = 1
  _check(native.call("configure", layout, bad_robot, Frames.basis_to_godot()) != OK, "Invalid scan parameters accepted")
  for iteration in range(16):
    _check(native.call("configure", layout, robot, Frames.basis_to_godot()) == OK, "Reconfigure failed")
    await physics_frame
    _check(native.call("scan_batch", snapshot).size() == 1, "Reconfigured scan failed")
    native.call("close")
    native.call("close")
    var temporary := Lidar.new()
    _check(temporary.configure(layout, robot) == OK, "Temporary configure failed")
    await physics_frame
    _check(temporary.scan_batch(snapshot).size() == 1, "Temporary scan failed")
    temporary = null
  _check(kernel.scan_batch(snapshot)[0]["ranges"].to_byte_array() == initial[0]["ranges"].to_byte_array(), "Independent instance changed")
  kernel.close()
  kernel.close()
  _check(kernel.scan_batch(snapshot).is_empty(), "Closed kernel returned scans")
  native = null
  if failures == 0:
    print("NATIVE_LIDAR_LIFECYCLE_PASS")
  quit(0 if failures == 0 else 1)
