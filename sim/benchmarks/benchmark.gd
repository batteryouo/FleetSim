extends Node

const Lidar = preload("res://sensors/lidar_kernel.gd")
const Kinematics = preload("res://robots/kinematics.gd")
var _started := false
var _config: Dictionary
var _kernel: RefCounted

func _ready() -> void:
  if OS.has_feature("editor") or OS.is_debug_build():
    push_error("Benchmark requires an exported release template")
    get_tree().quit(1)
    return
  _config = JSON.parse_string(FileAccess.get_file_as_string(OS.get_cmdline_user_args()[0]))
  if _config["backend"] != "jolt":
    push_error("Only the selected Jolt backend is available")
    get_tree().quit(1)
    return
  _kernel = Lidar.new()
  var layout: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(_config["map"]))
  if _kernel.configure(layout, _config["robot"]) != OK:
    get_tree().quit(1)

func _physics_process(_delta: float) -> void:
  if _started:
    return
  _started = true
  var count: int = _config["count"]
  var states := PackedFloat64Array(_config["poses"])
  var velocities := PackedFloat64Array()
  velocities.resize(count * 2)
  var ids: Array = []
  for index in range(count):
    ids.append("r%d" % index)
  var commands: Array[PackedFloat64Array] = []
  for phase in range(200):
    var command := PackedFloat64Array()
    for index in range(count):
      command.append(0.5 * sin(TAU * phase / 200.0 + index))
      command.append(0.0)
    commands.append(command)
  var snapshot := {"ids": ids, "poses": states, "robot_spec": _config["robot"]}
  var scans: Array[Dictionary] = []
  for step in range(int(_config["warmup_steps"])):
    var result := Kinematics.advance(states, velocities, commands[step % 200], _config["dt"], _config["robot"])
    if result.is_empty():
      get_tree().quit(1)
      return
    states = result["states"]
    velocities = result["velocities"]
    snapshot["poses"] = states
    scans = _kernel.scan_batch(snapshot)
  _write(_config["ready"], "ready")
  while not FileAccess.file_exists(_config["go"]):
    if Time.get_ticks_msec() > 120000:
      get_tree().quit(1)
      return
    OS.delay_msec(2)
  var steps := 0
  var elapsed_usec := 0
  var start := Time.get_ticks_usec()
  while Time.get_ticks_usec() - start < int(_config["seconds"] * 1000000):
    var command: PackedFloat64Array = commands[steps % 200]
    var before := Time.get_ticks_usec()
    var result := Kinematics.advance(states, velocities, command, _config["dt"], _config["robot"])
    states = result["states"]
    velocities = result["velocities"]
    snapshot["poses"] = states
    scans = _kernel.scan_batch(snapshot)
    elapsed_usec += Time.get_ticks_usec() - before
    steps += 1
  var output := {"backend": _config["backend"], "count": count, "beams": _config["robot"]["lidar"]["beam_count"],
    "steps": steps, "elapsed_seconds": elapsed_usec / 1000000.0,
    "wall_seconds": (Time.get_ticks_usec() - start) / 1000000.0,
    "release": not OS.is_debug_build(), "editor": OS.has_feature("editor"),
    "godot": Engine.get_version_info()["string"], "scans": scans.size(),
    "native": ClassDB.class_exists("CadenceKinematics"),
    "lidar_implementation": "cpp_jolt" if ClassDB.class_exists("CadenceLidar") else "unavailable", "checksum": 0.0}
  for scan in scans:
    for distance in scan["ranges"]:
      if is_finite(distance):
        output["checksum"] += distance
  _write(_config["output"], JSON.stringify(output, "  "))
  while not FileAccess.file_exists(_config["ack"]):
    if Time.get_ticks_usec() - start > 120000000:
      get_tree().quit(1)
      return
    OS.delay_msec(2)
  _kernel.close()
  get_tree().quit(0)

func _write(path: String, value: String) -> void:
  var file := FileAccess.open(path, FileAccess.WRITE)
  file.store_string(value)
  file.close()
