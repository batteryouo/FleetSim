extends RefCounted

const Frames = preload("res://bridge/frames.gd")
const EXTENSION_PATH := "res://native/cadence.gdextension"
var _implementation: RefCounted

func configure(compiled_map: Dictionary, robot_spec: Dictionary) -> Error:
  close()
  if ProjectSettings.get_setting("physics/3d/physics_engine") != "Jolt Physics":
    push_error("The lidar kernel requires Jolt Physics")
    return ERR_UNCONFIGURED
  if not ClassDB.class_exists("CadenceLidar"):
    var status := GDExtensionManager.load_extension(EXTENSION_PATH)
    if status != GDExtensionManager.LOAD_STATUS_OK and status != GDExtensionManager.LOAD_STATUS_ALREADY_LOADED:
      push_error("Cannot load Cadence native lidar. Build the extension with CMake; see README.md")
      return ERR_UNAVAILABLE
  if not ClassDB.class_exists("CadenceLidar"):
    push_error("The loaded extension did not register CadenceLidar. Rebuild the extension with CMake")
    return ERR_UNAVAILABLE
  _implementation = ClassDB.instantiate("CadenceLidar")
  return _implementation.call("configure", compiled_map, robot_spec, Frames.basis_to_godot())

func scan_batch(snapshot: Dictionary) -> Array[Dictionary]:
  if not is_instance_valid(_implementation):
    push_error("Configure the native lidar kernel before scanning")
    return []
  return _implementation.call("scan_batch", snapshot)

func close() -> void:
  if is_instance_valid(_implementation):
    _implementation.call("close")
    _implementation = null
