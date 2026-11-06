extends RefCounted

static func advance(states: PackedFloat64Array, previous_velocities: PackedFloat64Array,
    commands: PackedFloat64Array, dt: float, limits: Dictionary) -> Dictionary:
  if not is_finite(dt) or dt <= 0.0 or states.size() % 3 != 0:
    push_error("Kinematics requires finite positive dt and three state components per robot")
    return {}
  var count := states.size() / 3
  if previous_velocities.size() != count * 2 or commands.size() != count * 2:
    push_error("Kinematics requires two velocities and two commands per robot")
    return {}
  if not _finite_buffer(states) or not _finite_buffer(previous_velocities) or not _finite_buffer(commands):
    push_error("Kinematics inputs must be finite")
    return {}
  if not _valid_limits(limits):
    push_error("Kinematics requires finite ordered velocity limits and positive accelerations")
    return {}
  var next_states := PackedFloat64Array()
  var velocities := PackedFloat64Array()
  next_states.resize(states.size())
  velocities.resize(previous_velocities.size())
  var linear_delta := float(limits["max_linear_acceleration"]) * dt
  var angular_delta := float(limits["max_angular_acceleration"]) * dt
  for robot in range(count):
    var pose_index := robot * 3
    var velocity_index := robot * 2
    var v := previous_velocities[velocity_index] + clampf(
      commands[velocity_index] - previous_velocities[velocity_index], -linear_delta, linear_delta)
    var w := previous_velocities[velocity_index + 1] + clampf(
      commands[velocity_index + 1] - previous_velocities[velocity_index + 1], -angular_delta, angular_delta)
    v = clampf(v, float(limits["min_linear_velocity"]), float(limits["max_linear_velocity"]))
    w = clampf(w, float(limits["min_angular_velocity"]), float(limits["max_angular_velocity"]))
    var half_turn := w * dt / 2.0
    var distance := v * dt * _sinc(half_turn)
    var heading := states[pose_index + 2] + half_turn
    next_states[pose_index] = states[pose_index] + distance * cos(heading)
    next_states[pose_index + 1] = states[pose_index + 1] + distance * sin(heading)
    next_states[pose_index + 2] = _normalize_yaw(states[pose_index + 2] + w * dt)
    velocities[velocity_index] = v
    velocities[velocity_index + 1] = w
  return {"states": next_states, "velocities": velocities}

static func _sinc(angle: float) -> float:
  if absf(angle) < 0.0001:
    var squared := angle * angle
    return 1.0 - squared / 6.0 + squared * squared / 120.0 - squared * squared * squared / 5040.0
  return sin(angle) / angle

static func _normalize_yaw(angle: float) -> float:
  var yaw := fmod(angle, TAU)
  if yaw <= -PI:
    yaw += TAU
  elif yaw > PI:
    yaw -= TAU
  return yaw

static func _finite_buffer(values: PackedFloat64Array) -> bool:
  for value in values:
    if not is_finite(value):
      return false
  return true

static func _valid_limits(limits: Dictionary) -> bool:
  for axis in ["linear", "angular"]:
    for field in ["min_" + axis + "_velocity", "max_" + axis + "_velocity", "max_" + axis + "_acceleration"]:
      var value = limits.get(field)
      if not (value is float or value is int) or not is_finite(float(value)):
        return false
    if limits["min_" + axis + "_velocity"] >= limits["max_" + axis + "_velocity"]:
      return false
    if limits["max_" + axis + "_acceleration"] <= 0.0:
      return false
  return true
