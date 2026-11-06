# Native simulation kernels build

## Scope

`native/src/kinematics.cpp` implements batched differential-drive integration in
portable C++17 using `double` buffers. It has no Godot dependency, nodes, physics
bodies, randomness, or coordinate conversions. `godot_kinematics.cpp` validates
the public inputs and connects `PackedFloat64Array` to the kernel. The registered
`CadenceKinematics` class is one stateless `RefCounted` service for the entire batch.

The existing `sim/robots/kinematics.gd` entry point loads and caches that service.
Its `advance(...)` arguments, output dictionary, command order, dt handling, error
behavior, and input immutability follow `docs/robot-spec.md`. The script contains
no motion calculations or numerical fallback. Loading is explicit, so tests do
not require opening the editor to create `.godot/extension_list.cfg` first.

`native/src/godot_lidar.cpp` implements `CadenceLidar`, a stateful RefCounted
service per query space. It uses Godot PhysicsServer3D and Jolt for ray queries;
proxy updates, beam loops, and scan assembly execute in C++. The public GDScript
adapter preserves the lidar interface in `docs/robot-spec.md`. Explicit close
and native destruction release RIDs and shape references. The coordinate basis
is supplied by `sim/bridge/frames.gd` and applied by `sim/bridge/native_frames.hpp`;
the native code does not define another axis mapping. Both classes share the existing `cadence_kinematics` library name
and descriptor, so build commands and deployment paths stay unchanged.

## Dependencies and compatibility

- CMake 3.25 or newer.
- Python 3 for generating godot-cpp bindings.
- A C++17 compiler and a build tool supported by the selected CMake generator.
- Standard Godot 4.7.1.

### Validated environments

| Platform | Godot | Python | CMake | Compiler | glibc |
| --- | --- | --- | --- | --- | --- |
| Windows x86_64 | 4.7.1.stable.official.a13da4feb | 3.11.6 | 3.28.2 | MinGW GCC 13.2.0 | Not applicable |
| Arch Linux x86_64 | 4.7.1.stable.official.a13da4feb | 3.11.16 | 4.4.4 | GCC 16.2.1 | 2.44 |

The Arch validation used commit `6ca0d76ba0b2fe3b3a9dbc3b66e27f30cc969d7a`.
All 37 tests passed, including native `.so` loading without editor import caches,
missing-library rejection, independent motion checks, and batch/single-robot
bit identity. Dynamic dependency checks passed with no missing libraries;
`ldd` listed `libm.so.6`, `libc.so.6`, the ELF loader, and the Linux vDSO.
Windows also passed all 37 tests. These are functional validation results,
not throughput benchmark results. macOS remains unvalidated.

### Pinned bindings

CMake FetchContent downloads official godot-cpp revision
`e83fd0904c13356ed1d4c3d09f8bb9132bdc6b77` (`godot-4.5-stable`) and verifies
archive SHA-256 `579af30c5f62c1084edb28216788188227d80a583f0114463699ac4edd22140f`.
No third-party sources are vendored into tracked paths. The dependency revision
and hash live in `native/CMakeLists.txt` and must be updated together.

The extension targets the 4.5 API and declares minimum compatibility 4.5; the
project's tested runtime remains 4.7.1. Godot supports extensions built against
older minor APIs on newer engines. See the official
[godot-cpp compatibility policy](https://github.com/godotengine/godot-cpp#compatibility).

The bindings use `GODOTCPP_PRECISION=single` for the standard engine's `real_t`
ABI. Robot state still uses `double` and `PackedFloat64Array` throughout. A
double-precision custom engine requires matching binding settings and separate
validation. The binding profile includes RefCounted, the 3D physics query and shape classes,
and their dependencies. OS is also included because godot-cpp's internal verbose
logging requires it.

## Build on Windows

From the repository root, with CMake, Python, `g++`, and `mingw32-make` on `PATH`:

```powershell
cmake -S native -B build/native -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build/native --parallel 4
```

The first configure needs network access to download the dependency; subsequent
builds reuse the ignored build tree. Compiler selection is cached by CMake. Use
a separate build directory if changing compilers or generators.

For an offline build with an existing checkout of the pinned revision:

```powershell
cmake -S native -B build/native -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_SOURCE_DIR_GODOT_CPP=C:/path/to/godot-cpp
cmake --build build/native --parallel 4
```

The output is `sim/native/bin/cadence_kinematics.dll`. MinGW's C++ runtime is
linked statically through godot-cpp's build settings. The validated DLL depends
only on Windows system libraries, so MinGW runtime DLLs are not required beside it.
The `.gdextension` descriptor is tracked; binaries, dependency downloads, and
build caches are ignored and must be rebuilt after checkout. Map and robot JSON
remain committed generated data and are still written only by the asset compilers.

Restart Godot after rebuilding; hot reload is disabled. On Windows, a running
process that has loaded the DLL must be closed before replacing it.

## Build on Arch Linux

The following configure and build commands were used for the Arch validation.
Run them from the repository root with an active Python virtual environment
containing the dependencies in `requirements.txt`. CMake, Ninja, and GCC must be
available on `PATH`; `VIRTUAL_ENV` identifies the active environment.

```bash
cmake -S native -B build/native-linux -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python"

cmake --build build/native-linux --parallel 4
```

The output is ignored `sim/native/bin/cadence_kinematics.so`. Set `GODOT_BIN` to
the local Godot executable, then reproduce the acceptance and dependency checks:

```bash
"$GODOT_BIN" --headless --version
python -m unittest discover -s tests -v
ldd sim/native/bin/cadence_kinematics.so
git rev-parse HEAD
```

Use a separate build tree for each platform or toolchain. The dependency download
and checksum verification are the same as on Windows. CMake also configures macOS
library paths in the descriptor, but macOS build and execution are unvalidated.

## Linux binary portability

Build on each target machine or in a build environment targeting that machine's
ABI, then run the headless tests there. Treat the Arch-built `.so` as validated
for the recorded Arch environment. A binary built on a rolling-release distro
must not be assumed to load on older distributions, including Ubuntu LTS.

The binary's runtime requirements depend on the build environment, linked
libraries, and referenced symbol versions. The recorded glibc 2.44 identifies
the tested environment; it is not an established minimum runtime version.
The Arch artifact dynamically depends on glibc/libm. Its `ldd` output does not
include a dynamic `libstdc++` dependency: the pinned godot-cpp Linux build defaults
to static libgcc/libstdc++ linkage. Builds that disable that setting must also
satisfy the dynamic C++ runtime's symbol-version requirements. See the
[GCC C++ library ABI policy](https://gcc.gnu.org/onlinedocs/libstdc++/manual/abi.html)
and the pinned [godot-cpp Linux build settings](https://github.com/godotengine/godot-cpp/blob/e83fd0904c13356ed1d4c3d09f8bb9132bdc6b77/cmake/linux.cmake).

Static C++ runtime linkage does not establish compatibility with older glibc
environments. Keep native binaries out of Git and build on or for each target
environment. A future shared artifact must be built for an explicit compatibility
baseline and validated on its intended targets before reuse.

## Validation

Set `GODOT_BIN` as described in the README, then run:

```powershell
python -m unittest discover -s tests -v
```

The existing independent motion expectations exercise the C++ implementation.
The runner checks both native class registration and the extension manager's
loaded state, and prints `NATIVE_KINEMATICS: CadenceKinematics`. Repeated identical
inputs and batch versus scalar execution must match in bytes. Cross-compiler and
cross-platform bit identity is not promised. The kernel is compiled without
fast-math or floating-point contraction; motion tolerances are unchanged.

Isolated-project tests copy only the entry point, test harness, descriptor, and
compiled library into a temporary project without an editor cache. One test
requires successful native execution; another omits the library and requires a
nonzero exit with a build instruction. Invalid buffer lengths, non-finite values,
and invalid motion limits are also tested at the native boundary.

The C++ lidar is validated on Windows only; the recorded Arch run covers M1/M2,
not the expanded physics bindings. Rebuild and rerun the full suite on each target.
The lidar tests verify native registration, loading without caches, rejection of
a missing library, geometry, snapshot updates, and resource lifecycle. The Windows
build passes all 55 tests. See `docs/benchmark-native-lidar.md` for a focused release
comparison; the historical 2D/Jolt selection remains in `docs/benchmark-m3.md`.
