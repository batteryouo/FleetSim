import unittest

from assets.build import ROOT
from tools.benchmark_m3 import decide, fixture_output, parallel_counts, summary

def measurements(planar=400, jolt=350):
  return [{"backend": backend, "count": count, "beams": 360,
           "steps_per_second": summary([rate - 1, rate, rate + 1]),
           "peak_bytes": summary([memory] * 5)}
          for backend, rate, memory in [("2d", planar, 100), ("jolt", jolt, 110)]
          for count in [5, 20]]

class BenchmarkM3Tests(unittest.TestCase):
  def test_exactly_one_meets_target(self):
    self.assertEqual(decide(measurements(300, 190))["outcome"], "GodotPhysics2D")
    self.assertEqual(decide(measurements(190, 300))["outcome"], "Jolt 3D")
  def test_neither_meets_requires_stage2(self):
    self.assertIn("Stage 2 required", decide(measurements(190, 180))["outcome"])
  def test_both_meet_uses_both_headroom_checks(self):
    self.assertEqual(decide(measurements())["outcome"], "Jolt 3D")
    values = measurements()
    values[-1]["steps_per_second"] = summary([200, 201, 202])
    self.assertEqual(decide(values)["outcome"], "GodotPhysics2D")
  def test_threshold_overlap_uses_memory(self):
    values = measurements(400, 280)
    self.assertEqual(decide(values)["outcome"], "GodotPhysics2D")
    for item in values:
      if item["backend"] == "jolt":
        item["peak_bytes"] = summary([90] * 5)
    self.assertEqual(decide(values)["outcome"], "Jolt 3D")
  def test_target_includes_exactly_200(self):
    self.assertTrue(decide(measurements(200, 100))["meets"]["2d"])
  def test_parallel_counts_include_physical_core_endpoint(self):
    self.assertEqual(parallel_counts(10), [1, 2, 4, 8, 10])
    self.assertEqual(parallel_counts(8), [1, 2, 4, 8])
    self.assertEqual(parallel_counts(1), [1])
  def test_fixtures_cannot_be_written_to_tracked_paths(self):
    self.assertEqual(fixture_output(ROOT / "tmp" / "m3"), ROOT / "tmp" / "m3")
    with self.assertRaises(ValueError):
      fixture_output(ROOT / "spec" / "benchmark")
