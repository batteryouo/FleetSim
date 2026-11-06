"""Renders archived two-backend M3 measurements as tables and plots."""
import argparse
import json
from pathlib import Path

from assets.build import ROOT
from tools.benchmark_m3 import BACKENDS, BEAMS, COUNTS, decide, parallel_counts

CHANGED_FILES = {
  "Contracts and compiler": ["spec/robot.schema.yaml", "spec/robots/a100.yaml", "assets/build_robot.py", "sim/generated/robots/a100.json"],
  "Query runtime": ["sim/project.godot", "sim/bridge/frames.gd", "sim/sensors/lidar_kernel.gd", "sim/sensors/lidar_kernel.gd.uid", "sim/native/cadence.gdextension.uid"],
  "Release benchmark": ["sim/export_presets.cfg", "sim/benchmarks/benchmark.gd", "sim/benchmarks/benchmark.gd.uid", "sim/benchmarks/benchmark.tscn", "tools/__init__.py", "tools/m3_fixtures.py", "tools/benchmark_m3.py", "tools/report_m3.py", "requirements.txt"],
  "Tests": ["tests/test_robot_compiler.py", "tests/test_milestone3.py", "tests/test_benchmark_m3.py", "sim/tests/lidar_test.gd", "sim/tests/lidar_test.gd.uid"],
  "Documentation": ["README.md", "docs/robot-spec.md", "docs/project-brief.md", "docs/design-context.md", "docs/benchmark-m3.md", "docs/figures/m3-count.png", "docs/figures/m3-beams.png", "docs/figures/m3-parallel.png"],
}

def spread(value, scale=1, digits=1):
  return f"{value['median'] / scale:.{digits}f} [{value['min'] / scale:.{digits}f}, {value['max'] / scale:.{digits}f}]"
def charts(data, directory):
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  directory.mkdir(parents=True, exist_ok=True)
  metrics = [("steps_per_second", 1, "Steps/s"), ("rays_per_second", 1e6, "Million rays/s"),
             ("peak_bytes", 1048576, "Peak per-instance MiB")]
  for horizontal in ["count", "beams"]:
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)
    for row, backend in enumerate(BACKENDS):
      for column, (metric, scale, title) in enumerate(metrics):
        ax = axes[row, column]
        for fixed in (BEAMS if horizontal == "count" else COUNTS):
          items = sorted([item for item in data["single"] if item["backend"] == backend and
                          item["beams" if horizontal == "count" else "count"] == fixed],
                         key=lambda item: item[horizontal])
          x = [item[horizontal] for item in items]
          ax.plot(x, [item[metric]["median"] / scale for item in items], marker="o",
                  label=f"{fixed} {'beams' if horizontal == 'count' else 'robots'}")
          ax.fill_between(x, [item[metric]["min"] / scale for item in items],
                          [item[metric]["max"] / scale for item in items], alpha=.15)
        ax.set_title(f"{backend}: {title}")
        ax.set_xlabel("Robots" if horizontal == "count" else "Beams per robot")
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    fig.suptitle("Release kinematics + lidar; median and min-max of five runs")
    fig.savefig(directory / f"m3-{horizontal}.png", dpi=130)
    plt.close(fig)
  fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
  for ax, (metric, scale, title) in zip(axes, [
    ("per_instance_steps", 1, "Mean per-instance steps/s"), ("total_steps", 1, "Aggregate steps/s"),
    ("per_instance_peak_bytes", 1048576, "Mean per-instance peak MiB")]):
    for backend in BACKENDS:
      items = [item for item in data["parallel"] if item["backend"] == backend]
      x = [item["instances"] for item in items]
      ax.plot(x, [item[metric]["median"] / scale for item in items], marker="o", label=backend)
      ax.fill_between(x, [item[metric]["min"] / scale for item in items],
                      [item[metric]["max"] / scale for item in items], alpha=.15)
    ax.set_title(title)
    ax.set_xlabel("Concurrent instances")
    ax.grid(alpha=.25)
    ax.legend()
  fig.suptitle("5 robots, 360 beams; median and min-max of five synchronized groups")
  fig.savefig(directory / "m3-parallel.png", dpi=130)
  plt.close(fig)
def render(source):
  data = json.loads(source.read_text(encoding="utf-8"))
  if data["smoke"] or len(data["single"]) != 36 or any(len(item["runs"]) != 5 for item in data["single"]):
    raise ValueError("An archived full two-backend, five-repeat benchmark is required")
  expected = {(backend, count, beams) for backend in BACKENDS for count in COUNTS for beams in BEAMS}
  if {(item["backend"], item["count"], item["beams"]) for item in data["single"]} != expected:
    raise ValueError("Incomplete single-instance conditions")
  expected_parallel = {(backend, count) for backend in BACKENDS for count in parallel_counts(data["machine"]["physical_cores"])}
  if {(item["backend"], item["instances"]) for item in data["parallel"]} != expected_parallel:
    raise ValueError("Incomplete parallel conditions")
  decision = decide(data["single"])
  if decision != data["decision"]:
    # JSON turns integer dictionary keys into strings.
    if json.loads(json.dumps(decision)) != data["decision"]:
      raise ValueError("Stored decision differs from measured results")
  figures = ROOT / "docs" / "figures"
  charts(data, figures)
  machine = data["machine"]
  lookup = {(item["backend"], item["count"], item["beams"]): item for item in data["single"]}
  lines = ["# Milestone 3 stage 1 benchmark", "", "## Outcome", "",
    f"The project-brief rule recommends **{decision['outcome']}**.",
    "The current runtime retains only the confirmed Jolt backend. No stage 2 is implemented.",
    "This report preserves historical two-backend measurements.", "",
    f"The recorded dataset used **{data.get('robot_body_mode', 'static').upper()} robot query bodies**.", "",
    "At 5 robots and 360 beams, the single-instance target is median >= 200 steps/s:", "",
    "| Backend | Steps/s median [min, max] | Target met | Peak MiB median [min, max] |",
    "| --- | --- | --- | --- |"]
  for backend in BACKENDS:
    item = lookup[backend, 5, 360]
    lines.append(f"| {backend} | {spread(item['steps_per_second'])} | {decision['meets'][backend]} | {spread(item['peak_bytes'], 1048576)} |")
  lines += ["", "The headroom comparison uses 360 beams:", "",
            "| Robots | 2D median steps/s | Jolt median steps/s | Jolt / 2D | 70% threshold tie |",
            "| --- | --- | --- | --- | --- |"]
  for count in [5, 20]:
    planar = lookup["2d", count, 360]["steps_per_second"]["median"]
    jolt = lookup["jolt", count, 360]["steps_per_second"]["median"]
    comparison = decision["ratios"].get(count, {})
    lines.append(f"| {count} | {planar:.2f} | {jolt:.2f} | {jolt / planar:.2%} | {comparison.get('threshold_tie', 'not needed')} |")
  lines += ["", "The 200 steps/s qualification uses the median exactly. Spread is the observed",
    "minimum-to-maximum interval of five runs. For the 70% comparison, overlapping",
    "Jolt and 0.7-times-2D intervals count as a tie; if needed, median peak memory at",
    "the 5-robot/360-beam target condition resolves it. This is an operational",
    "interpretation of the brief's spread rule, not a statistical confidence interval."]
  if "memory_tiebreak" in decision:
    lines += ["", f"Memory tiebreak (bytes): `{decision['memory_tiebreak']}`."]
  lines += ["", "## Machine and build", "", "| Item | Value |", "| --- | --- |",
    f"| CPU | {machine['cpu']} |",
    f"| Cores | {machine['physical_cores']} physical, {machine['logical_cores']} logical |",
    f"| OS-visible RAM | {machine['ram_bytes']} bytes ({machine['ram_bytes'] / 1073741824:.2f} GiB) |",
    f"| OS | {machine.get('os_caption', machine['os'])} |",
    f"| Platform API | {machine['os']} |", f"| Godot | {machine['godot']} |",
    f"| Compiler | {machine['compiler']} |", f"| CMake | {machine['cmake']} |",
    f"| Python | {machine['python']} |", f"| Baseline commit | `{machine['commit']}` plus this M3 change |",
    f"| Release template SHA-256 | `{machine['template_sha256']}` |",
    f"| Exported executable SHA-256 | `{machine['executable_sha256']}` |",
    f"| Exported resource pack SHA-256 | `{machine.get('pack_sha256', 'not recorded')}` |",
    f"| Native library SHA-256 | `{machine.get('native_sha256', 'not recorded')}` |", "",
    "The extension is built with CMAKE_BUILD_TYPE=Release and GODOTCPP_TARGET=template_release.",
    "Each worker asserts it is an exported, non-debug, non-editor runtime and that",
    "the C++ kinematics class is loaded. Both backends run under the same project",
    "configuration: GodotPhysics2D and Jolt Physics. No affinity or scheduling",
    "priority is imposed; this hybrid CPU's P/E cores and normal OS scheduling can",
    "affect parallel scaling. Linux M3 throughput has not been measured.", "",
    "## Workload and measurement", "",
    f"The deterministic compiler-generated stress map contains {data['wall_primitives']} rectangular walls:",
    "a 160 m corridor 4 m wide with 40 rooms per side, door openings, outer walls,",
    "partitions, end caps, and 80 boxes alternating above/below the scan plane.",
    "The full primitive input is identical for both kernels. 2D filters the 80",
    "off-plane walls at load time; Jolt keeps their 3D geometry. Neither backend",
    "uses analytic intersection code. Robots retain the A100's true 0.27 m radius",
    "and 0-0.25 m body height; every robot scans every step and can hit teammates.", "",
    f"Each run warms up for {data['warmup_steps']} steps, then measures for at least {data['seconds']} wall-clock seconds.",
    f"Benchmark-only dt = {data['dt']} s is supplied in each worker configuration;",
    "it does not select the project's control rate. Precomputed deterministic",
    "sinusoidal linear commands (200-step period, 0.5 m/s amplitude, distinct",
    "per-robot phases, zero angular velocity) keep motion bounded. The existing",
    "C++ integrator updates all poses and previous velocities on every step.", "",
    "Timed work includes the native kinematics call, state/snapshot updates,",
    "query-body synchronization, snapshot validation, all ray calls, and",
    "construction of scan arrays and metadata. Pose integration and sensing run",
    "inside one physics-safe callback; motion is kinematic and bodies have zero",
    "contact masks. Steps/s is steps divided by accumulated timed work; rays/s is",
    "steps/s times robot count times beam count. Timer resolution is one microsecond.",
    "This is a **lower bound on per-step cost**: export/load/warmup, command-table",
    "generation, outer loop/timer overhead, result checksums, file I/O, rendering,",
    "regular frame scheduling, collision handling, transport, policy inference,",
    "noise, and pedestrians are outside the timed interval. Raw results also",
    "record whole-loop wall rates to expose any timing-overhead difference.", "",
    "Peak per-instance memory is the process OS high-water resident set, not a",
    "Godot allocator counter: Windows peak_wset, Linux VmHWM. It includes startup,",
    "engine, extension, map, warmup, and measured work. Workers remain alive until",
    "the parent reads this counter; memory is not estimated from sparse samples.",
    "Five fresh processes run per single condition. Backend order alternates",
    "between repeats to reduce order bias. Results and fixtures stay under tmp/.", "",
    "Parallel groups warm up independently, then start from one file barrier.",
    "All group members must finish before peak memory is read. Each count has",
    "five fresh groups per backend. Per-instance rate is the group arithmetic",
    "mean; aggregate rate is the sum of member rates, followed by median/min/max",
    "across groups. These remain query-plus-kinematics capacity estimates, not",
    "end-to-end trainer throughput. No cores are reserved for a Python trainer",
    "in this saturation experiment. Every individual worker result is retained.", "",
    "Functional tests independently check geometry, metadata, mount composition,",
    "height/range boundaries, self exclusion, immediate robot movement/removal,",
    "and deliberately wrong expectations. Stress parity covers N=1,5,20,40, all",
    "three beam counts and three snapshots, with <=1e-4 m finite-range difference",
    "and identical infinity classes. M1/M2 tests also run with Jolt enabled.", "",
    (f"Implementation validation: all {data['validation']['unittest_count']} unittest tests passed."
     if data.get("validation", {}).get("unittest_passed") else
     "Correctness tests must be run separately; this benchmark does not replace unittest."),
    "The release benchmark asserts exported release execution and native kinematics",
    "loading in every worker.", "",
    "## Single-instance results", "", "Values are median [minimum, maximum] over five runs.", "",
    "| Backend | Robots | Beams | Steps/s | Million rays/s | Peak MiB |",
    "| --- | --- | --- | --- | --- | --- |"]
  for item in data["single"]:
    lines.append(f"| {item['backend']} | {item['count']} | {item['beams']} | {spread(item['steps_per_second'])} | {spread(item['rays_per_second'], 1e6, 3)} | {spread(item['peak_bytes'], 1048576)} |")
  lines += ["", "![Robot-count curves](figures/m3-count.png)", "", "![Beam-count curves](figures/m3-beams.png)", "",
    "## Parallel scaling", "", "Each instance has 5 robots and 360 beams. Values are median [min, max] across five groups.", "",
    "| Backend | Instances | Mean per-instance steps/s | Aggregate steps/s | Mean per-instance peak MiB |",
    "| --- | --- | --- | --- | --- |"]
  for item in data["parallel"]:
    lines.append(f"| {item['backend']} | {item['instances']} | {spread(item['per_instance_steps'])} | {spread(item['total_steps'])} | {spread(item['per_instance_peak_bytes'], 1048576)} |")
  lines += ["", "![Parallel curves](figures/m3-parallel.png)", "", "## Reproduction", "",
    "Use a separate checkout of `9d36f69` to rerun the historical two-backend",
    "comparison below. Current code runs Jolt only; see README for its commands.",
    "`tools.report_m3` accepts archived full two-backend datasets, not Jolt-only runs.", "",
    "From the repository root, with a matching Godot editor and official release",
    "export template installed (or an extracted template passed explicitly):", "", "```powershell",
    'cmake -S native -B build/native -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DGODOTCPP_TARGET=template_release',
    "cmake --build build/native --parallel 4", "python -m assets.build_robot spec/robots/a100.yaml",
    "python -m unittest discover -s tests -v",
    "python -m tools.benchmark_m3 --release-template tmp/tools/templates/windows_release_x86_64.exe --output tmp/m3-results-20261107",
    "python -m tools.report_m3 tmp/m3-results-20261107/results.json", "```", "",
    "Set GODOT_BIN to the Godot console/editor executable, as in README. The exact",
    "measured run used the commands above on Windows. The explicit template was",
    "extracted from the official 4.7.1-stable export-templates archive, not built",
    "from editor sources. The exporter stages sim/ under the output directory and",
    "selects the benchmark scene there; the tracked project's main scene is unchanged.",
    "Use a new output directory for a new run. The report renderer requires matplotlib",
    "(validated with 3.8.2); benchmark execution only needs requirements.txt.", "",
    "Linux: build using docs/native-build.md, set GODOT_BIN, and run the same module",
    "with --release-template pointing to linux_release.x86_64. The Linux preset and",
    "VmHWM measurement path are supplied but remain unvalidated for M3. Installed",
    "templates can be used by omitting --release-template. A smoke export can be",
    "checked with --smoke --seconds 0.3; smoke results are rejected by the report renderer.", "",
    f"Local raw results: `{source.relative_to(ROOT).as_posix() if source.is_relative_to(ROOT) else source.as_posix()}`. This includes every",
    "worker's config, log, output and OS peak bytes in aggregate JSON. Fixtures,",
    "staged exports, native binaries, and raw benchmark JSON are ignored by Git.", ""]
  lines += ["## Changed files", "",
    "The existing M3 target/decision-rule edits in the brief and design context",
    "are preserved and included. Current implementation/status descriptions are",
    "updated for stage 1. Godot-generated resource UIDs are tracked.", ""]
  for category, paths in CHANGED_FILES.items():
    lines.append(f"{category}:")
    lines.append("")
    lines.extend(f"- `{path}`" for path in paths)
    lines.append("")
  (ROOT / "docs" / "benchmark-m3.md").write_text("\n".join(lines), encoding="utf-8")

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("results", type=Path)
  render(parser.parse_args().results.resolve())
