# Native kinematics build

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

## Dependencies and compatibility

- CMake 3.25 or newer; validated with 3.28.2.
- Python 3 for generating godot-cpp bindings; validated with 3.11.6.
- A C++17 compiler; Windows x86_64 is validated with MinGW GCC 13.2.0.
- Standard Godot 4.7.1, matching the existing acceptance environment.

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
validation. The binding profile keeps RefCounted and its dependencies; OS is
also included because godot-cpp's internal verbose logging requires it.

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

CMake also configures library paths for Linux and macOS in the descriptor, but
these platforms have not been validated. Choose a suitable local generator and
toolchain before building there; only Windows x86_64 is an acceptance platform.

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

This migration establishes a native build and integration path. It makes no
performance claim and does not decide the Milestone 3 lidar backend. The M3
benchmark and its acceptance criteria remain separate work.
