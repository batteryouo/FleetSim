# Cadence

Cadence is a lightweight multi-robot simulation project. Milestone 1 provides a
YAML map compiler, a minimal Godot world, and headless physics ray-distance checks.
Milestone 2 adds a robot spec compiler and batched float64 differential-drive
kinematics implemented in a C++ GDExtension. Lidar, robot collision handling, noise,
agent transport, and Ward 73 modeling are outside the current implementation.

## Setup

Validated environments use standard Godot `4.7.1.stable.official.a13da4feb`:

| Platform | Python | CMake | Compiler | glibc |
| --- | --- | --- | --- | --- |
| Windows x86_64 | 3.11.6 | 3.28.2 | MinGW GCC 13.2.0 | Not applicable |
| Arch Linux x86_64 | 3.11.16 | 4.4.4 | GCC 16.2.1 | 2.44 |

Both environments passed all 37 tests; the Arch run validated commit `6ca0d76`.
See [Linux build and validation](docs/native-build.md#build-on-arch-linux) for
the exact Linux commands and [binary portability](docs/native-build.md#linux-binary-portability)
for target ABI requirements. macOS remains unvalidated.
Install Python dependencies from the repository root in a Python environment:

```powershell
python -m pip install -r requirements.txt
```

Build the native extension before running Godot or the test suite. CMake 3.25 or
newer and a C++17 compiler are required. On Windows, put `g++` and `mingw32-make`
on `PATH` for this generator; use the linked Linux commands on Arch:

```powershell
cmake -S native -B build/native -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build/native --parallel 4
```

The first configure downloads the pinned, checksum-verified godot-cpp dependency.
The DLL is written to ignored `sim/native/bin/cadence_kinematics.dll`; native
binaries and build caches are not committed. Rebuild after editing native source,
then restart Godot. See [native build](docs/native-build.md) for dependency,
precision, and offline-build details.

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
python -m assets.build spec/maps/test_room.yaml
python -m assets.build_robot spec/robots/a100.yaml
python -m unittest discover -s tests -v
```

Both compilers use module invocation (`python -m`) from the repository root.
The map compiler writes `sim/generated/test_room.json`. Generated maps are committed
so the Godot world can run without Python, but must only be regenerated through
`assets/build.py`. The test suite also compiles a fresh temporary map and loads it
into real Godot physics before checking 13 rays with a 1 mm distance tolerance.
The robot compiler writes `sim/generated/robots/a100.json`. Milestone 2 tests compile
fresh robot configurations and run headless motion checks, including the 1 m/s for
2 s acceptance case, acceleration ramps, and bit-identical batch/single execution.
The tests require the C++ extension and check that its native class is loaded;
there is no script fallback. Isolated loader tests cover a fresh project without
editor import caches and a missing DLL.

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
