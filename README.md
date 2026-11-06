# Cadence

Cadence is a lightweight multi-robot simulation project. Milestone 1 provides a
YAML map compiler, a minimal Godot world, and headless physics ray-distance checks.
Milestone 2 adds a robot spec compiler and batched float64 differential-drive
kinematics. Lidar, robot collision handling, noise, agent transport, and Ward 73
modeling are outside the current implementation.

## Setup

Validated tools: Python 3.11.6 and Godot 4.7.1 stable (standard build, not .NET).
Install Python dependencies from the repository root:

```powershell
python -m pip install -r requirements.txt
```

Use the Godot console executable on Windows. Set its actual path for the current
PowerShell session; no persistent environment changes are required:

```powershell
$env:GODOT_BIN = 'C:\path\to\Godot_v4.7.1-stable_win64_console.exe'
```

On other machines, set `GODOT_BIN` to the local executable or put `godot` / `godot4`
on `PATH`. Tests fail if Godot is unavailable; they do not skip the acceptance test.

## Compile and test

Run these commands from the repository root:

```powershell
python assets/build.py spec/maps/test_room.yaml
python -m assets.build_robot spec/robots/a100.yaml
python -m unittest discover -s tests -v
```

The compiler writes `sim/generated/test_room.json`. Generated maps are committed
so the Godot world can run without Python, but must only be regenerated through
`assets/build.py`. The test suite also compiles a fresh temporary map and loads it
into real Godot physics before checking 13 rays with a 1 mm distance tolerance.
The robot compiler writes `sim/generated/robots/a100.json`. Milestone 2 tests compile
fresh robot configurations and run headless motion checks, including the 1 m/s for
2 s acceptance case, acceleration ramps, and bit-identical batch/single execution.

To run only the robot compiler and Milestone 2 tests:

```powershell
python -m unittest discover -s tests -p test_robot_compiler.py -v
python -m unittest discover -s tests -p test_milestone2.py -v
```

To run only the headless ray check against the generated map:

```powershell
New-Item -ItemType Directory -Force tmp | Out-Null
& $env:GODOT_BIN --headless --path sim --log-file (Join-Path $PWD 'tmp/milestone1.log') --script res://tests/ray_distance_test.gd -- --map res://generated/test_room.json
```

Success prints `MILESTONE_1_PASS: 13 rays` and exits with status 0. To inspect the
world with a simple overhead camera:

```powershell
& $env:GODOT_BIN --path sim
```

The overhead scene displays the Milestone 1 map. Milestone 2 provides the headless
kinematics API without adding robot visual nodes.

See [map spec v0](docs/map-spec.md) and [robot spec v0](docs/robot-spec.md) for the
source and compiled contracts and
[project brief](docs/project-brief.md) for architecture and later milestones.
Local references and previews belong under ignored `tmp/`; never copy or commit
reference images, photos, or PDFs into tracked paths.
