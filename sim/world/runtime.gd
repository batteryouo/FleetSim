extends RefCounted
## Lockstep World behind protocol version 1; see docs/protocol.md.
##
## `handle(request)` takes one decoded request dictionary and returns the reply dictionary. It
## owns no transport, clock or randomness. A rejected request returns an error reply and leaves
## the configuration and episode state unchanged. `closed` becomes true after a close request.

const PROTOCOL_VERSION := 1
const Kinematics = preload("res://robots/kinematics.gd")
const Lidar = preload("res://sensors/lidar_kernel.gd")
const MAP_VERSION := 1
const ROBOT_VERSION := 1
var closed := false
var _id_pattern := RegEx.create_from_string("^[A-Za-z][A-Za-z0-9_]*$")
var _lidar: RefCounted
var _spec := {}
var _ids: Array[String] = []
var _physics_dt := 0.0
var _substeps := 0
var _seed := -1
var _step := 0
var _states := PackedFloat64Array()
var _velocities := PackedFloat64Array()

func handle(request: Variant) -> Dictionary:
  if not request is Dictionary:
    return _error("malformed", "A request must be a map")
  if typeof(request.get("protocol")) != TYPE_INT or request["protocol"] != PROTOCOL_VERSION:
    return _error("protocol_version", "This World speaks protocol version %d" % PROTOCOL_VERSION)
  match request.get("type"):
    "configure":
      return await _configure(request)
    "reset":
      return _reset(request)
    "step":
      return _step_world(request)
    "close":
      var problem := _fields(request, [])
      if not problem.is_empty():
        return _error("invalid_argument", problem)
      close()
      closed = true
      return _reply("close")
  return _error("unknown_type", "Unknown request type: %s" % [request.get("type")])

func error_reply(code: String, message: String) -> Dictionary:
  return _error(code, message)

func close() -> void:
  if _lidar != null:
    _lidar.close()
    _lidar = null
  _spec = {}
  _ids.clear()
  _seed = -1

func _configure(request: Dictionary) -> Dictionary:
  var problem := _fields(request, ["map", "robot_specs", "robots", "physics_dt", "physics_steps_per_control"])
  if not problem.is_empty():
    return _error("invalid_argument", problem)
  var map = request["map"]
  var specs = request["robot_specs"]
  var robots = request["robots"]
  var dt = request["physics_dt"]
  var substeps = request["physics_steps_per_control"]
  if not map is Dictionary or map.get("version") != MAP_VERSION or map.get("frame") != "rep103":
    return _error("invalid_argument", "map must be a compiled version %d REP-103 map" % MAP_VERSION)
  if not specs is Dictionary or specs.is_empty():
    return _error("invalid_argument", "robot_specs must map spec names to compiled robot specs")
  for name in specs:
    var spec = specs[name]
    if not name is String or not spec is Dictionary or spec.get("version") != ROBOT_VERSION or spec.get("frame") != "rep103":
      return _error("invalid_argument", "Each robot spec must be a compiled version %d REP-103 spec" % ROBOT_VERSION)
  if not robots is Array or robots.is_empty():
    return _error("invalid_argument", "robots must be a non-empty array")
  var ids: Array[String] = []
  var spec_names := {}
  for robot in robots:
    if not robot is Dictionary or robot.size() != 2 or not robot.get("id") is String or not robot.get("spec") is String:
      return _error("invalid_argument", "Each robot must be {id, spec} with string values")
    if _id_pattern.search(robot["id"]) == null or robot["id"] in ids:
      return _error("invalid_argument", "Robot ids must be unique identifiers: %s" % robot["id"])
    if not specs.has(robot["spec"]):
      return _error("invalid_argument", "Unknown robot spec: %s" % robot["spec"])
    ids.append(robot["id"])
    spec_names[robot["spec"]] = true
  if spec_names.size() != 1:
    return _error("unsupported", "All robots must share one robot spec in protocol version %d" % PROTOCOL_VERSION)
  if not _finite_number(dt) or dt <= 0:
    return _error("invalid_argument", "physics_dt must be a finite positive number")
  if typeof(substeps) != TYPE_INT or substeps < 1:
    return _error("invalid_argument", "physics_steps_per_control must be a positive integer")
  var spec: Dictionary = specs[spec_names.keys()[0]]
  if Kinematics.advance(PackedFloat64Array(), PackedFloat64Array(), PackedFloat64Array(), float(dt), spec).is_empty():
    return _error("invalid_argument", "The robot spec's motion limits or physics_dt were rejected")
  var lidar := Lidar.new()
  if lidar.configure(map, spec) != OK:
    return _error("invalid_argument", "The lidar rejected the map or robot spec")
  await (Engine.get_main_loop() as SceneTree).physics_frame
  close()
  _lidar = lidar
  _spec = spec
  _ids = ids
  _physics_dt = float(dt)
  _substeps = substeps
  var reply := _reply("configure")
  reply["robots"] = ids.duplicate()
  reply["physics_dt"] = _physics_dt
  reply["physics_steps_per_control"] = _substeps
  reply["control_period"] = _control_period()
  return reply

func _reset(request: Dictionary) -> Dictionary:
  var problem := _fields(request, ["seed", "poses"])
  if not problem.is_empty():
    return _error("invalid_argument", problem)
  if _lidar == null:
    return _error("invalid_state", "Configure the World before reset")
  if typeof(request["seed"]) != TYPE_INT or request["seed"] < 0:
    return _error("invalid_argument", "seed must be a non-negative 64-bit integer")
  var states = _per_robot(request["poses"], 3)
  if states is String:
    return _error("invalid_argument", "poses: " + states)
  var velocities := PackedFloat64Array()
  velocities.resize(2 * _ids.size())
  return _commit("reset", request["seed"], 0, states, velocities)

func _step_world(request: Dictionary) -> Dictionary:
  var problem := _fields(request, ["step", "actions"])
  if not problem.is_empty():
    return _error("invalid_argument", problem)
  if _seed < 0:
    return _error("invalid_state", "Reset the World before stepping")
  if typeof(request["step"]) != TYPE_INT or request["step"] != _step:
    return _error("step_mismatch", "The World is at step %d" % _step)
  var commands = _per_robot(request["actions"], 2)
  if commands is String:
    return _error("invalid_argument", "actions: " + commands)
  var states := _states
  var velocities := _velocities
  for substep in range(_substeps):
    var result := Kinematics.advance(states, velocities, commands, _physics_dt, _spec)
    if result.is_empty():
      return _error("internal", "Kinematics rejected the step")
    states = result["states"]
    velocities = result["velocities"]
  return _commit("step", _seed, _step + 1, states, velocities)

## Scans the candidate state and adopts it only when the observation is complete.
func _commit(type: String, episode_seed: int, step: int, states: PackedFloat64Array, velocities: PackedFloat64Array) -> Dictionary:
  var scans: Array[Dictionary] = _lidar.scan_batch({"ids": _ids, "poses": states, "robot_spec": _spec})
  if scans.size() != _ids.size():
    return _error("internal", "Lidar did not return one scan per robot")
  _seed = episode_seed
  _step = step
  _states = states
  _velocities = velocities
  var robots := {}
  for index in range(_ids.size()):
    var scan := scans[index]
    robots[_ids[index]] = {
      "pose": [states[3 * index], states[3 * index + 1], states[3 * index + 2]],
      "velocity": [velocities[2 * index], velocities[2 * index + 1]],
      "scan": {"angle_min": scan["angle_min"], "angle_max": scan["angle_max"],
        "angle_increment": scan["angle_increment"], "range_min": scan["range_min"],
        "range_max": scan["range_max"], "ranges": scan["ranges"].to_byte_array()},
    }
  var reply := _reply(type)
  reply["seed"] = _seed
  reply["step"] = _step
  reply["time"] = _step * _control_period()
  reply["robots"] = robots
  reply["events"] = []
  return reply

## Returns a float64 buffer in configured robot order, or an error message.
func _per_robot(values: Variant, width: int) -> Variant:
  if not values is Dictionary or values.size() != _ids.size():
    return "expected exactly one entry for each of %s" % [_ids]
  var result := PackedFloat64Array()
  for id in _ids:
    var value = values.get(id)
    if not value is Array or value.size() != width:
      return "%s must be an array of %d numbers" % [id, width]
    for number in value:
      if not _finite_number(number):
        return "%s must contain finite numbers" % id
      result.append(float(number))
  return result

func _fields(request: Dictionary, required: Array) -> String:
  var allowed := required + ["protocol", "type"]
  for key in request:
    if not key in allowed:
      return "Unknown field: %s" % [key]
  for key in required:
    if not request.has(key):
      return "Missing field: %s" % key
  return ""

func _finite_number(value: Variant) -> bool:
  return typeof(value) == TYPE_INT or (typeof(value) == TYPE_FLOAT and is_finite(value))

func _control_period() -> float:
  return _physics_dt * _substeps

func _reply(type: String) -> Dictionary:
  return {"protocol": PROTOCOL_VERSION, "type": type}

func _error(code: String, message: String) -> Dictionary:
  var reply := _reply("error")
  reply["code"] = code
  reply["message"] = message
  return reply
