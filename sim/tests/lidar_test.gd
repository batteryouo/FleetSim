extends SceneTree

const Lidar = preload("res://sensors/lidar_kernel.gd")
var failures := 0

func _initialize() -> void:
  call_deferred("_run")

func _run() -> void:
  var args := OS.get_cmdline_user_args()
  var fixture: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(args[0]))
  for test_case in fixture["cases"]:
    var results := {}
    for backend in ["2d", "jolt"]:
      var kernel := Lidar.new()
      if kernel.configure(test_case["map"], test_case["robot"], backend) != OK:
        _fail("configure failed")
        continue
      await physics_frame
      var scans: Array[Dictionary] = []
      for snapshot in test_case["snapshots"]:
        scans = kernel.scan_batch({"ids": snapshot["ids"], "poses": PackedFloat64Array(snapshot["poses"]),
          "robot_spec": test_case["robot"]})
        if scans.size() != snapshot["ids"].size():
          _fail(test_case["label"] + ": missing scans")
          continue
        for index in range(scans.size()):
          var scan: Dictionary = scans[index]
          var lidar: Dictionary = test_case["robot"]["lidar"]
          if not scan["ranges"] is PackedFloat64Array or scan["ranges"].size() != lidar["beam_count"]:
            _fail("ranges must be float64 with beam_count entries")
          for key in ["angle_min", "angle_max", "range_min", "range_max"]:
            if scan[key] != lidar[key]:
              _fail("incorrect metadata " + key)
          if scan["robot_id"] != snapshot["ids"][index] or scan["angle_increment"] != (lidar["angle_max"] - lidar["angle_min"]) / (lidar["beam_count"] - 1):
            _fail("incorrect id or angular increment")
          if snapshot.has("expected"):
            for beam in range(scan["ranges"].size()):
              var expected = snapshot["expected"][index][beam]
              var actual: float = scan["ranges"][beam]
              if not _matches(actual, expected, 0.00001):
                _fail("%s %s robot %d beam %d: %s != %s" % [test_case["label"], backend, index, beam, actual, expected])
        if backend == "2d":
          results[str(snapshot)] = scans
        else:
          var other: Array = results[str(snapshot)]
          for index in range(scans.size()):
            for beam in range(scans[index]["ranges"].size()):
              if not _matches(scans[index]["ranges"][beam], other[index]["ranges"][beam], 0.0001):
                _fail("%s parity robot %d beam %d: %s != %s" % [test_case["label"], index, beam,
                  scans[index]["ranges"][beam], other[index]["ranges"][beam]])
                break
      kernel.close()
  if failures == 0:
    print("MILESTONE_3_PASS: %d cases" % fixture["cases"].size())
  quit(0 if failures == 0 else 1)

func _matches(actual: float, expected: Variant, tolerance: float) -> bool:
  if expected is String:
    return actual == (INF if expected == "+inf" else -INF)
  if is_inf(expected):
    return actual == expected
  return is_finite(actual) and abs(actual - expected) <= tolerance

func _fail(message: String) -> void:
  failures += 1
  if failures <= 20:
    push_error(message)
