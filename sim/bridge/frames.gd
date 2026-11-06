extends RefCounted

static func position_to_godot(position: Array) -> Vector3:
  return Vector3(-float(position[1]), float(position[2]), -float(position[0]))

static func size_to_godot(size: Array) -> Vector3:
  return Vector3(float(size[1]), float(size[2]), float(size[0]))

static func yaw_to_godot(yaw: float) -> float:
  return yaw
