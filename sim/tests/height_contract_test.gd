extends SceneTree

const MapLoader = preload("res://world/map_loader.gd")

func _initialize() -> void:
  var args := OS.get_cmdline_user_args()
  if args.size() != 2 or args[0] != "--manifest":
    push_error("Usage: -- --manifest <invalid-map-paths.json>")
    quit(1)
    return
  var file := FileAccess.open(args[1], FileAccess.READ)
  if file == null:
    push_error("Cannot open test manifest")
    quit(1)
    return
  var paths = JSON.parse_string(file.get_as_text())
  if not paths is Array or paths.is_empty():
    push_error("Test manifest must contain invalid map paths")
    quit(1)
    return
  var failures := 0
  for path in paths:
    var world := MapLoader.new()
    var status := world.load_map(path)
    if status == OK or world.get_child_count() != 0:
      failures += 1
      push_error("Invalid compiled map must fail without creating bodies: " + path)
    world.free()
  if failures == 0:
    print("HEIGHT_CONTRACT_PASS: %d invalid maps rejected" % paths.size())
  quit(0 if failures == 0 else 1)
