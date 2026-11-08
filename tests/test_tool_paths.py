from pathlib import Path
import tempfile
import unittest

from tools import benchmark_m3, compare_lidar, plan_extract, preview_map
from tools.paths import ROOT, fixture_output

class ToolPathTests(unittest.TestCase):
  def test_tools_share_guard_and_legacy_import(self):
    for module in [benchmark_m3, compare_lidar, plan_extract, preview_map]:
      self.assertIs(module.fixture_output, fixture_output)
  def test_rejects_tracked_paths_traversal_and_tmp_sibling(self):
    for path in [
      ROOT / "spec/map.yaml",
      ROOT / "tmp/../spec/map.yaml",
      ROOT / "tmp-other/map.yaml",
    ]:
      with self.subTest(path=path), self.assertRaises(ValueError):
        fixture_output(path)
  def test_accepts_resolved_local_and_os_temporary_paths(self):
    for path in [
      ROOT / "tmp/artifacts/../map.png",
      Path(tempfile.gettempdir()) / "cadence-output.json",
    ]:
      self.assertEqual(fixture_output(path), path.resolve())
