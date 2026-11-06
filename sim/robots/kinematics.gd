extends RefCounted

const EXTENSION_PATH := "res://native/cadence.gdextension"
static var _implementation: RefCounted

static func advance(states: PackedFloat64Array, previous_velocities: PackedFloat64Array,
    commands: PackedFloat64Array, dt: float, limits: Dictionary) -> Dictionary:
  if not is_instance_valid(_implementation):
    if not ClassDB.class_exists("CadenceKinematics"):
      var status := GDExtensionManager.load_extension(EXTENSION_PATH)
      if status != GDExtensionManager.LOAD_STATUS_OK and status != GDExtensionManager.LOAD_STATUS_ALREADY_LOADED:
        push_error("Cannot load Cadence native kinematics. Build the extension with CMake; see README.md")
        return {}
    if not ClassDB.class_exists("CadenceKinematics"):
      push_error("The loaded extension did not register CadenceKinematics")
      return {}
    _implementation = ClassDB.instantiate("CadenceKinematics")
  return _implementation.call("advance", states, previous_velocities, commands, dt, limits)
