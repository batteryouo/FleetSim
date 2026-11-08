# Native Jolt lidar comparison

## Scope and method

This comparison measures moving the Jolt query orchestration from GDScript to
C++ GDExtension. Both versions use Jolt intersections, STATIC query proxies,
the same 326-wall map, float64 planar state, and the same native kinematics.
It does not implement stage 2 analytic ray intersection or a custom spatial index.

The baseline is the Jolt-only implementation at `b44e8f1`; the candidate is the
native lidar implementation. Both are exported release builds using standard
Godot 4.7.1.stable.official.a13da4feb on Windows 11 10.0.26300, an i5-12600K
(10 physical / 16 logical cores), and 31.75 GiB OS-visible RAM. The native
library uses MinGW GCC 13.2.0, CMake 3.28.2, the pinned godot-cpp bindings,
Release/template_release, and the project's strict floating-point options.

Each condition runs five times for two seconds after 50 warmup steps, alternating
baseline/candidate order. There is one instance, 360 beams per robot, and either
5 or 20 robots. The fixture uses dt=0.05 s and the same deterministic sinusoidal
commands as the M3 benchmark. Timing includes kinematics, proxy updates, and
all lidar queries. Values below are median [minimum, maximum]; they are not
confidence intervals. No affinity or scheduling priority is imposed.

This focused comparison does not repeat the full beam-count or parallel-scaling
matrix from `benchmark-m3.md`. Linux native lidar remains unvalidated.

## Results

| Robots | Implementation | Steps/s | Peak MiB |
| --- | --- | --- | --- |
| 5 | gdscript | 310.77 [284.06, 314.71] | 73.97 [73.84, 74.19] |
| 5 | cpp_jolt | 392.28 [376.69, 398.78] | 73.94 [73.81, 74.14] |
| 20 | gdscript | 80.91 [80.42, 81.72] | 74.07 [73.80, 74.11] |
| 20 | cpp_jolt | 97.96 [97.53, 98.52] | 73.89 [73.84, 74.02] |

At 5 robots, the candidate's median throughput is 1.262 times the baseline (26.2% higher).
At 20 robots, the candidate's median throughput is 1.211 times the baseline (21.1% higher).

The 200 steps/s qualification applies only to 5 robots; both versions pass.
These measurements describe this machine and workload, not a general C++ speedup.

## Reproduction

Build the baseline in a separate checkout at `b44e8f1`. In that checkout,
run:

```powershell
cmake -S native -B build/native -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DGODOTCPP_TARGET=template_release
cmake --build build/native --parallel 4
python -m tools.benchmark_m3 --release-template C:/path/to/windows_release_x86_64.exe --output tmp/benchmarks/baseline-export --smoke --seconds .3
```

In the current checkout, build and validate the candidate:

```powershell
cmake -S native -B build/native -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DGODOTCPP_TARGET=template_release
cmake --build build/native --parallel 4
python -m unittest discover -s tests -v
python -m tools.benchmark_m3 --release-template C:/path/to/windows_release_x86_64.exe --output tmp/benchmarks/candidate-export --smoke --seconds .3
python -m tools.compare_lidar --baseline C:/path/to/baseline-checkout/tmp/benchmarks/baseline-export/benchmark.exe --candidate tmp/benchmarks/candidate-export/benchmark.exe --output tmp/benchmarks/lidar-comparison
```

Set GODOT_BIN as in README. Keep each executable beside its original `.pck` and
native library. Use new output directories on repeated runs. Smoke mode prepares
the exports; the comparison tool performs the five-repeat measurements. It checks
release mode, scan counts, a positive finite-range checksum, and the candidate's
`cpp_jolt` implementation marker. Old baseline workers predate that marker.

The exact comparison invocation for the recorded run was:

```powershell
python -m tools.compare_lidar --baseline tmp/benchmarks/m3/m3-jolt-selected-smoke/benchmark.exe --candidate tmp/benchmarks/m3/m3-native-lidar-final/benchmark.exe --output tmp/benchmarks/m3/m3-native-lidar-final-comparison
```

Raw worker results, hashes, and peak memory samples are in ignored
`tmp/benchmarks/m3/m3-native-lidar-final-comparison/results.json`. Artifacts are not committed.

## Artifact fingerprints

| Implementation | Artifact | SHA-256 |
| --- | --- | --- |
| gdscript | benchmark.exe | `04baf75cc1d69dd93eb709533ecab4fd7770bb8a530645717017a06a9d9809fc` |
| gdscript | benchmark.pck | `5b0d6dbfe0ff30a3946174b97c70b604aa0a46e05b764b9f21b51719ac3b08eb` |
| gdscript | cadence_kinematics.dll | `00667d986dab19237ccb5bff310739f7193d51ac80b415e9e1b31d650bb03358` |
| cpp_jolt | benchmark.exe | `04baf75cc1d69dd93eb709533ecab4fd7770bb8a530645717017a06a9d9809fc` |
| cpp_jolt | benchmark.pck | `2faae9437c40c37d4c8c353bfcaebb023e2627d331d0da8388c9bee84822a32c` |
| cpp_jolt | cadence_kinematics.dll | `82b99cfcf9d28ff33b0a6b917ddad9cb00aa50288f0971f529520521838e08ef` |
