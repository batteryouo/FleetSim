extends SceneTree

const Kinematics = preload("res://robots/kinematics.gd")
var failures := 0

func _initialize() -> void:
  var args := OS.get_cmdline_user_args()
  if args.size() != 4 or args[0] != "--robot" or args[2] != "--cases":
    push_error("Usage: -- --robot <compiled-robot.json> --cases <test-cases.json>")
    quit(1)
    return
  var robot = _read_json(args[1])
  var cases = _read_json(args[3])
  if not robot is Dictionary or not cases is Array or cases.is_empty():
    push_error("Invalid kinematics test input")
    quit(1)
    return
  for test_case in cases:
    _run_case(test_case, robot)
  if not ClassDB.class_exists("CadenceKinematics") or not GDExtensionManager.is_extension_loaded(Kinematics.EXTENSION_PATH):
    _fail("Kinematics tests must execute the Cadence C++ GDExtension")
  else:
    print("NATIVE_KINEMATICS: CadenceKinematics")
  if failures == 0:
    print("MILESTONE_2_PASS: %d cases" % cases.size())
  quit(0 if failures == 0 else 1)

func _read_json(path: String) -> Variant:
  var file := FileAccess.open(path, FileAccess.READ)
  if file == null:
    return null
  return JSON.parse_string(file.get_as_text())

func _run_case(test_case: Dictionary, robot: Dictionary) -> void:
  var label: String = test_case["label"]
  var states := PackedFloat64Array(test_case["states"])
  var velocities := PackedFloat64Array(test_case["velocities"])
  var commands := PackedFloat64Array(test_case["commands"])
  var initial_states := states.duplicate()
  var initial_velocities := velocities.duplicate()
  var initial_commands := commands.duplicate()
  var dt: float = test_case["dt"]
  if test_case.get("nonfinite_dt", false):
    dt = NAN
  if test_case.has("nonfinite_buffer"):
    var value: float = NAN if test_case.get("nonfinite_value") == "nan" else INF
    match test_case["nonfinite_buffer"]:
      "states": states[0] = value
      "velocities": velocities[0] = value
      "commands": commands[0] = value
  if test_case.has("limit_overrides") or test_case.has("remove_limit") or test_case.has("nonfinite_limit"):
    robot = robot.duplicate(true)
    robot.merge(test_case.get("limit_overrides", {}), true)
    if test_case.has("remove_limit"):
      robot.erase(test_case["remove_limit"])
    if test_case.has("nonfinite_limit"):
      robot[test_case["nonfinite_limit"]] = NAN
  for step in range(int(test_case["steps"])):
    var before_states := states.to_byte_array()
    var before_velocities := velocities.to_byte_array()
    var before_commands := commands.to_byte_array()
    var result := Kinematics.advance(states, velocities, commands, dt, robot)
    if test_case.get("invalid", false):
      if not result.is_empty():
        _fail(label + ": invalid inputs were accepted")
      return
    if result.is_empty():
      _fail(label + ": valid inputs were rejected")
      return
    var repeat := Kinematics.advance(states, velocities, commands, dt, robot)
    if result["states"].to_byte_array() != repeat["states"].to_byte_array() or result["velocities"].to_byte_array() != repeat["velocities"].to_byte_array():
      _fail(label + ": repeated identical inputs changed output bits")
    if states.to_byte_array() != before_states or velocities.to_byte_array() != before_velocities or commands.to_byte_array() != before_commands:
      _fail(label + ": inputs were mutated")
    if typeof(result["states"]) != TYPE_PACKED_FLOAT64_ARRAY or typeof(result["velocities"]) != TYPE_PACKED_FLOAT64_ARRAY:
      _fail(label + ": output buffers must use float64 storage")
    states = result["states"]
    velocities = result["velocities"]
  _check_values(label + " states", states, test_case["expected_states"], float(test_case["tolerance"]))
  _check_values(label + " velocities", velocities, test_case["expected_velocities"], float(test_case["tolerance"]))
  if test_case.get("compare_single", false):
    _check_batch(label, initial_states, initial_velocities, initial_commands, dt, int(test_case["steps"]), robot, states, velocities)
  print("PASS " + label)

func _check_values(label: String, actual: PackedFloat64Array, expected: Array, tolerance: float) -> void:
  if actual.size() != expected.size():
    _fail(label + ": buffer length mismatch")
    return
  for index in range(actual.size()):
    if not is_finite(actual[index]) or absf(actual[index] - float(expected[index])) > tolerance:
      _fail("%s[%d]: expected %.17f, got %.17f" % [label, index, expected[index], actual[index]])

func _check_batch(label: String, initial_states: PackedFloat64Array, initial_velocities: PackedFloat64Array,
    commands: PackedFloat64Array, dt: float, steps: int, robot: Dictionary,
    batch_states: PackedFloat64Array, batch_velocities: PackedFloat64Array) -> void:
  var single_states := PackedFloat64Array()
  var single_velocities := PackedFloat64Array()
  for index in range(initial_velocities.size() / 2):
    var state := initial_states.slice(index * 3, index * 3 + 3)
    var velocity := initial_velocities.slice(index * 2, index * 2 + 2)
    var command := commands.slice(index * 2, index * 2 + 2)
    for step in range(steps):
      var result := Kinematics.advance(state, velocity, command, dt, robot)
      state = result["states"]
      velocity = result["velocities"]
    single_states.append_array(state)
    single_velocities.append_array(velocity)
  if single_states.to_byte_array() != batch_states.to_byte_array() or single_velocities.to_byte_array() != batch_velocities.to_byte_array():
    _fail(label + ": batch and single-robot outputs differ in bits")

func _fail(message: String) -> void:
  failures += 1
  push_error(message)
