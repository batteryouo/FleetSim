"""Exports and measures the Milestone 3 release query kernels."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import statistics
import subprocess
import tempfile
import time

import psutil

from assets.build import ROOT
from tools.m3_fixtures import poses, robot_spec, stress_map

COUNTS = [1, 2, 5, 10, 20, 40]
BEAMS = [360, 400, 720]
BACKENDS = ["2d", "jolt"]

def command(args, **kwargs):
  result = subprocess.run([str(value) for value in args], cwd=ROOT, capture_output=True, text=True, **kwargs)
  if result.returncode:
    raise RuntimeError(result.stdout + result.stderr)
  return result.stdout.strip()
def write_json(path, value):
  Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
def parallel_counts(cores):
  values = []
  count = 1
  while count <= cores:
    values.append(count)
    count *= 2
  return sorted(set(values + [cores]))
def fixture_output(path):
  path = Path(path).resolve()
  if not any(path.is_relative_to(root.resolve()) for root in [ROOT / "tmp", Path(tempfile.gettempdir())]):
    raise ValueError("Benchmark fixtures and exports must stay under tmp/ or the OS temporary directory")
  return path
def summary(values):
  return {"median": statistics.median(values), "min": min(values), "max": max(values)}
def cpu_model():
  if os.name == "nt":
    import winreg
    with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0") as key:
      return winreg.QueryValueEx(key, "ProcessorNameString")[0].strip()
  for line in Path("/proc/cpuinfo").read_text().splitlines():
    if line.startswith("model name"):
      return line.split(":", 1)[1].strip()
  return platform.processor()
def os_caption():
  if os.name == "nt":
    return command(["powershell", "-NoProfile", "-Command",
                    "$os = Get-CimInstance Win32_OperatingSystem; $os.Caption + ' ' + $os.Version"])
  return platform.platform()
def peak_memory(process):
  if os.name == "nt":
    return psutil.Process(process.pid).memory_info().peak_wset
  status = Path(f"/proc/{process.pid}/status").read_text()
  return int(next(line for line in status.splitlines() if line.startswith("VmHWM:")).split()[1]) * 1024
def export_project(godot, output, release_template):
  stage = output / "project"
  shutil.copytree(ROOT / "sim", stage, ignore=shutil.ignore_patterns(".godot"), dirs_exist_ok=True)
  project = stage / "project.godot"
  project.write_text(project.read_text().replace('run/main_scene="res://world/world.tscn"',
                     'run/main_scene="res://benchmarks/benchmark.tscn"'), encoding="utf-8")
  presets = stage / "export_presets.cfg"
  if release_template:
    presets.write_text(presets.read_text().replace('custom_template/release=""',
                       'custom_template/release=' + json.dumps(str(release_template.resolve()).replace("\\", "/"))), encoding="utf-8")
  executable = output / ("benchmark.exe" if os.name == "nt" else "benchmark.x86_64")
  preset = "Windows Benchmark" if os.name == "nt" else "Linux Benchmark"
  transcript = command([godot, "--headless", "--path", stage, "--log-file", output / "export.log",
                        "--export-release", preset, executable], timeout=180)
  (output / "export.stdout.txt").write_text(transcript, encoding="utf-8")
  if "ERROR:" in transcript or "ERROR:" in (output / "export.log").read_text(encoding="utf-8"):
    raise RuntimeError(f"Godot reported export errors; inspect {output / 'export.log'}")
  if not executable.exists():
    raise RuntimeError("Release export did not produce an executable")
  return executable
def run_group(executable, output, label, backend, count, beams, instances, seconds):
  directory = output / label
  directory.mkdir()
  robot = robot_spec()
  robot["lidar"]["beam_count"] = beams
  go = directory / "go"
  workers = []
  try:
    for index in range(instances):
      prefix = directory / str(index)
      config = {"backend": backend, "count": count, "robot": robot, "poses": poses(count),
                "map": str(output / "m3_stress.json"), "dt": .05, "seconds": seconds, "warmup_steps": 50,
                **{key: str(prefix.with_suffix("." + key)) for key in ["ready", "output", "ack"]},
                "go": str(go)}
      source = prefix.with_suffix(".config.json")
      write_json(source, config)
      log = prefix.with_suffix(".stdout.txt").open("w", encoding="utf-8")
      process = subprocess.Popen([str(executable), "--headless", "--log-file", str(prefix.with_suffix(".godot.log")),
                                  "--", str(source)], stdout=log, stderr=subprocess.STDOUT, cwd=output)
      workers.append((process, log, config))
    deadline = time.monotonic() + 90
    def wait_for(key):
      while not all(Path(config[key]).exists() for _, _, config in workers):
        if time.monotonic() > deadline or any(process.poll() is not None for process, _, _ in workers):
          raise RuntimeError(f"Worker failed or timed out; inspect {directory}")
        time.sleep(.01)
    wait_for("ready")
    go.write_text("go", encoding="utf-8")
    wait_for("output")
    results = []
    for process, _, config in workers:
      result = json.loads(Path(config["output"]).read_text())
      result["peak_bytes"] = peak_memory(process)
      result["steps_per_second"] = result["steps"] / result["elapsed_seconds"]
      result["wall_steps_per_second"] = result["steps"] / result["wall_seconds"]
      result["rays_per_second"] = result["steps_per_second"] * count * beams
      if not result["release"] or result["editor"] or not result["native"] or result["scans"] != count or result["checksum"] <= 0:
        raise RuntimeError(f"Invalid benchmark workload: {result}")
      results.append(result)
      Path(config["ack"]).write_text("ack", encoding="utf-8")
    for process, log, _ in workers:
      if process.wait(timeout=30):
        raise RuntimeError(f"Worker exited unsuccessfully; inspect {directory}")
      log.close()
    return results
  finally:
    for process, log, _ in workers:
      if process.poll() is None:
        process.kill()
        process.wait()
      log.close()
def decide(single):
  lookup = {(item["backend"], item["count"], item["beams"]): item for item in single}
  meets = {backend: lookup[backend, 5, 360]["steps_per_second"]["median"] >= 200 for backend in BACKENDS}
  if not any(meets.values()):
    return {"outcome": "Stage 2 required; engine defaults to Jolt 3D", "meets": meets, "ratios": {}}
  if meets["2d"] != meets["jolt"]:
    return {"outcome": "Jolt 3D" if meets["jolt"] else "GodotPhysics2D", "meets": meets, "ratios": {}}
  ratios = {}
  tied = False
  for count in [5, 20]:
    jolt = lookup["jolt", count, 360]["steps_per_second"]
    planar = lookup["2d", count, 360]["steps_per_second"]
    ratio = jolt["median"] / planar["median"]
    overlaps = jolt["min"] <= .7 * planar["max"] and jolt["max"] >= .7 * planar["min"]
    ratios[count] = {"median": ratio, "threshold_tie": overlaps}
    if ratio < .7 and not overlaps:
      return {"outcome": "GodotPhysics2D", "meets": meets, "ratios": ratios}
    tied |= overlaps
  if tied:
    memory = {backend: lookup[backend, 5, 360]["peak_bytes"]["median"] for backend in BACKENDS}
    outcome = "Jolt 3D" if memory["jolt"] <= memory["2d"] else "GodotPhysics2D"
    return {"outcome": outcome, "meets": meets, "ratios": ratios, "memory_tiebreak": memory}
  return {"outcome": "Jolt 3D", "meets": meets, "ratios": ratios}
def aggregate(backend, count, beams, runs):
  return {"backend": backend, "count": count, "beams": beams, "runs": runs,
          **{key: summary([run[key] for run in runs]) for key in
             ["steps_per_second", "rays_per_second", "peak_bytes", "wall_steps_per_second"]}}
def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--godot", default=os.environ.get("GODOT_BIN") or shutil.which("godot") or shutil.which("godot4"))
  parser.add_argument("--release-template", type=Path)
  parser.add_argument("--output", type=Path, default=ROOT / "tmp" / ("m3-" + time.strftime("%Y%m%d-%H%M%S")))
  parser.add_argument("--seconds", type=float, default=2.0)
  parser.add_argument("--smoke", action="store_true", help="Export and run one condition per backend, not a decision benchmark")
  args = parser.parse_args()
  if not args.godot or args.seconds <= 0:
    parser.error("A Godot editor executable and positive duration are required")
  output = fixture_output(args.output)
  output.mkdir(parents=True, exist_ok=False)
  layout = stress_map(output)
  executable = export_project(args.godot, output, args.release_template)
  machine = {"cpu": cpu_model(), "physical_cores": psutil.cpu_count(logical=False),
             "logical_cores": psutil.cpu_count(), "ram_bytes": psutil.virtual_memory().total,
             "os": platform.platform(), "os_caption": os_caption(), "python": platform.python_version(),
             "godot": command([args.godot, "--headless", "--version"]),
             "compiler": command(["g++", "--version"]).splitlines()[0],
             "cmake": command(["cmake", "--version"]).splitlines()[0],
             "commit": command(["git", "rev-parse", "HEAD"]),
             "executable_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
             "pack_sha256": hashlib.sha256(executable.with_suffix(".pck").read_bytes()).hexdigest(),
             "native_sha256": hashlib.sha256((output / ("cadence_kinematics.dll" if os.name == "nt" else "cadence_kinematics.so")).read_bytes()).hexdigest(),
             "template_sha256": hashlib.sha256(args.release_template.read_bytes()).hexdigest() if args.release_template else "installed template"}
  results = {"machine": machine, "wall_primitives": len(layout["primitives"]), "seconds": args.seconds,
             "dt": .05, "warmup_steps": 50, "single": [], "parallel": [], "smoke": args.smoke}
  for beams in ([360] if args.smoke else BEAMS):
    for count in ([5] if args.smoke else COUNTS):
      paired = {backend: [] for backend in BACKENDS}
      for repeat in range(1 if args.smoke else 5):
        for backend in (BACKENDS if repeat % 2 == 0 else BACKENDS[::-1]):
          paired[backend] += run_group(executable, output, f"single-{backend}-{count}-{beams}-{repeat}",
                                        backend, count, beams, 1, args.seconds)
      for backend in BACKENDS:
        item = aggregate(backend, count, beams, paired[backend])
        results["single"].append(item)
        print(f"{backend} N={count} beams={beams}: {item['steps_per_second']}", flush=True)
      write_json(output / "results.json", results)
  if not args.smoke:
    for instances in parallel_counts(machine["physical_cores"]):
      paired = {backend: [] for backend in BACKENDS}
      for repeat in range(5):
        for backend in (BACKENDS if repeat % 2 == 0 else BACKENDS[::-1]):
          runs = run_group(executable, output, f"parallel-{backend}-{instances}-{repeat}",
                           backend, 5, 360, instances, args.seconds)
          paired[backend].append({"instances": runs,
            "per_instance_steps": statistics.mean(run["steps_per_second"] for run in runs),
            "total_steps": sum(run["steps_per_second"] for run in runs),
            "wall_total_steps": sum(run["wall_steps_per_second"] for run in runs),
            "per_instance_peak_bytes": statistics.mean(run["peak_bytes"] for run in runs)})
      for backend in BACKENDS:
        results["parallel"].append({"backend": backend, "instances": instances, "runs": paired[backend],
          **{key: summary([run[key] for run in paired[backend]]) for key in
             ["per_instance_steps", "total_steps", "wall_total_steps", "per_instance_peak_bytes"]}})
        print(f"parallel {backend} x{instances}: {results['parallel'][-1]['total_steps']}", flush=True)
      write_json(output / "results.json", results)
    results["decision"] = decide(results["single"])
  write_json(output / "results.json", results)
  print(f"Results: {output / 'results.json'}", flush=True)

if __name__ == "__main__":
  main()
