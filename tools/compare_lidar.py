"""Compares exported GDScript and C++ Jolt lidar workers with alternating run order."""
import argparse
import hashlib
from pathlib import Path

from tools.benchmark_m3 import aggregate, run_group, write_json
from tools.paths import fixture_output
from tools.m3_fixtures import stress_map

def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--baseline", type=Path, required=True,
                      help="GDScript Jolt export from b44e8f1; a missing "
                           "lidar_implementation field means gdscript. "
                           "Exports marked cpp_jolt are rejected as baselines.")
  parser.add_argument("--candidate", type=Path, required=True)
  parser.add_argument("--output", type=Path, required=True)
  args = parser.parse_args()
  output = fixture_output(args.output)
  output.mkdir(parents=True, exist_ok=False)
  stress_map(output)
  executables = {"gdscript": args.baseline.resolve(), "cpp_jolt": args.candidate.resolve()}
  results = {"seconds": 2, "repeats": 5, "beams": 360, "single": [], "artifacts": {}}
  for name, executable in executables.items():
    files = [executable, executable.with_suffix(".pck")]
    files += sorted(executable.parent.glob("cadence_kinematics.*"))
    results["artifacts"][name] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}
  for count in [5, 20]:
    runs = {name: [] for name in executables}
    for repeat in range(5):
      order = list(executables) if repeat % 2 == 0 else list(reversed(executables))
      for name in order:
        runs[name] += run_group(executables[name], output, f"{name}-{count}-{repeat}",
                                "jolt", count, 360, 1, 2, implementation=name)
    for name in executables:
      item = aggregate("jolt", count, 360, runs[name])
      item["implementation"] = name
      results["single"].append(item)
      print(f"{name} N={count}: {item['steps_per_second']}", flush=True)
    write_json(output / "results.json", results)

if __name__ == "__main__":
  main()
