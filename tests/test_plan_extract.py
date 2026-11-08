import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from unittest.mock import MagicMock

import yaml
from jsonschema import Draft202012Validator

from assets.build import ROOT, compile_map
from assets.build_robot import compile_robot
from tools.plan_extract import (
  curve_points,
  door_candidates,
  extract,
  read_yaml,
  write_map,
  pdf_paths,
  main,
)
from tools.validate_map import validate_geometry

class PlanExtractionTests(unittest.TestCase):
  def setUp(self):
    fixture = json.loads((ROOT / "tests/fixtures/layered_plan.json").read_text())
    self.paths = fixture["paths"]
    self.config = fixture["config"]
    self.overrides = fixture["overrides"]
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.directory = Path(self.temp.name)
  def compiled(self, spec):
    source = self.directory / "map.yaml"
    source.write_text(yaml.safe_dump(spec), encoding="utf-8")
    return compile_map(source)
  def test_schema_and_input_rejections(self):
    Draft202012Validator.check_schema(
      yaml.safe_load((ROOT / "spec/extraction.schema.yaml").read_text())
    )
    for key in self.config:
      bad = dict(self.config)
      del bad[key]
      with self.subTest(key=key), self.assertRaises(ValueError):
        extract(self.paths, bad, self.overrides)
    for key, value in [
      ("extra", 1),
      ("meters_per_unit", 0),
      ("yaw", float("nan")),
      ("crop", [2, 2, 1, 1]),
      ("z_max", 0),
      ("wall_layers", ["missing"]),
    ]:
      with self.subTest(key=key), self.assertRaises(ValueError):
        extract(self.paths, dict(self.config, **{key: value}), self.overrides)
    for change in [
      dict(remove_paths=["missing"]),
      dict(doors=[dict(id="d", room="right", candidate="missing")]),
      dict(extra=True),
    ]:
      with self.assertRaises(ValueError):
        extract(self.paths, self.config, dict(self.overrides, **change))
    source = self.directory / "duplicate.yaml"
    source.write_text("page: 0\npage: 1\n")
    with self.assertRaisesRegex(ValueError, "Duplicate YAML key"):
      read_yaml(source)
  def test_override_shape_validation_and_alternate_swing(self):
    for key in ["annotations", "doors", "start", "targets", "validation_bounds"]:
      invalid = copy.deepcopy(self.overrides)
      del invalid[key]
      with self.assertRaises(ValueError):
        extract(self.paths, self.config, invalid)
    for invalid in [
      dict(annotations=[dict(self.overrides["annotations"][0], extra=1)]),
      dict(doors=[dict(self.overrides["doors"][0], alternate="yes")]),
      dict(validation_bounds=dict(center=[0, 0], size=[0, 1])),
      dict(walls=[dict(id="repair", start=[0, 0], end=[1, 0], width=0)]),
    ]:
      with self.assertRaises(ValueError):
        extract(self.paths, self.config, dict(self.overrides, **invalid))
    alternate = dict(self.overrides, doors=[dict(self.overrides["doors"][0], alternate=True)])
    spec, _ = extract(self.paths, self.config, alternate)
    self.assertEqual(spec["doors"][0]["nav_point"], [2, 3])
  def test_layers_door_inference_and_hand_computed_coordinates(self):
    spec, report = extract(self.paths, self.config, self.overrides)
    self.assertEqual(report["walls"], 7)
    self.assertFalse(any("furniture" in p["id"] or "leaf" in p["id"] for p in spec["walls"]))
    self.assertEqual(spec["walls"][0]["center"], [2.5, 5])
    self.assertEqual(spec["walls"][0]["size"], [5, 0.1])
    self.assertEqual(spec["doors"][0]["nav_point"], [2.5, 2.5])
    self.assertEqual(spec["doors"][0]["width"], 1)
    compiled = self.compiled(spec)
    robot = compile_robot(ROOT / "spec/robots/a100.yaml")
    self.assertTrue(validate_geometry(compiled, robot, spec["validation"])["ok"])
    self.assertEqual(len(compiled["primitives"]), 8)
  def test_scale_rebuild_and_yaw_preserve_height_and_topology(self):
    first, _ = extract(self.paths, self.config, self.overrides)
    second, _ = extract(self.paths, dict(self.config, meters_per_unit=1), self.overrides)
    for a, b in zip(first["walls"], second["walls"]):
      self.assertEqual(b["center"], [v * 2 for v in a["center"]])
      self.assertEqual(b["size"], [v * 2 for v in a["size"]])
      self.assertEqual(b["z_max"], a["z_max"])
    turned, _ = extract(self.paths, dict(self.config, yaw=math.pi / 2), self.overrides)
    self.assertAlmostEqual(turned["walls"][0]["center"][0], -5)
    self.assertAlmostEqual(turned["walls"][0]["center"][1], 2.5)
    self.assertAlmostEqual(turned["walls"][0]["yaw"], math.pi / 2)
  def test_missing_leaf_and_nonquarter_curves_are_not_guessed(self):
    candidates = door_candidates(self.paths, ["OPENINGS"], 0.01)
    self.assertEqual(len(candidates), 1)
    self.assertEqual(candidates[0]["start"], [5, 4])
    self.assertEqual(candidates[0]["end"], [5, 6])
    self.assertEqual(candidates[0]["alternate_end"], [3, 4])
    self.assertEqual(
      door_candidates([p for p in self.paths if p["id"] != "leaf"], ["OPENINGS"], 0.01), []
    )
    bad = copy.deepcopy(self.paths)
    bad[4]["curves"][0][-1] = [7, 7]
    self.assertEqual(door_candidates(bad, ["OPENINGS"], 0.01), [])
  def test_crops_exclusions_and_explicit_repairs(self):
    curve = self.paths[4]["curves"][0]
    samples = curve_points(curve, 8)
    self.assertEqual(samples[0], curve[0])
    self.assertEqual(samples[-1], curve[-1])
    curved = dict(id="arch", layer="STRUCTURE", width=0.2, segments=[], curves=[curve])
    curved_spec, _ = extract(self.paths + [curved], self.config, self.overrides)
    self.assertEqual(len([p for p in curved_spec["walls"] if p["id"].startswith("arch_")]), 8)
    overrides = dict(
      self.overrides,
      remove_paths=["partition_a"],
      exclude_regions=[[7, -1, 8, 1]],
      walls=[dict(id="repair", start=[5, 0], end=[5, 4], width=0.2)],
    )
    spec, _ = extract(self.paths, self.config, overrides)
    self.assertFalse(any(p["id"].startswith("partition_a") for p in spec["walls"]))
    pieces = [p for p in spec["walls"] if p["id"].startswith("outline_edge0")]
    self.assertEqual(len(pieces), 2)
    self.assertAlmostEqual(sum(p["size"][0] for p in pieces), 4.5)
    self.assertEqual(spec["walls"][-2]["id"], "repair")
    with self.assertRaisesRegex(ValueError, "outside crop"):
      extract(self.paths, dict(self.config, crop=[-1, -1, 4, 11]), self.overrides)
  def test_determinism_no_reference_dependency_and_failure_keeps_output(self):
    config = self.directory / "config.yaml"
    overrides = self.directory / "overrides.yaml"
    output = self.directory / "result.yaml"
    config.write_text(yaml.safe_dump(self.config))
    overrides.write_text(yaml.safe_dump(self.overrides))
    with patch("tools.plan_extract.pdf_paths", return_value=self.paths):
      write_map(Path("not-used.pdf"), config, overrides, output)
      first = output.read_bytes()
      write_map(Path("not-used.pdf"), config, overrides, output)
      self.assertEqual(first, output.read_bytes())
      invalid = dict(
        self.overrides, doors=[dict(id="bad", room="right", candidate="missing")]
      )
      overrides.write_text(yaml.safe_dump(invalid))
      with self.assertRaises(ValueError):
        write_map(Path("not-used.pdf"), config, overrides, output)
      self.assertEqual(first, output.read_bytes())
    compiled = self.compiled(yaml.safe_load(first))
    self.assertNotIn(str(self.directory), json.dumps(compiled))
  def test_missing_source_and_protected_output_fail_cleanly(self):
    config = self.directory / "config.yaml"
    overrides = self.directory / "overrides.yaml"
    config.write_text(yaml.safe_dump(self.config))
    overrides.write_text(yaml.safe_dump(self.overrides))
    for output in [self.directory / "out.yaml", ROOT / "spec/maps/forbidden.yaml"]:
      run = subprocess.run(
        [
          sys.executable,
          "-m",
          "tools.plan_extract",
          str(self.directory / "missing.pdf"),
          "--config",
          str(config),
          "--overrides",
          str(overrides),
          "--output",
          str(output),
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
      )
      self.assertEqual(run.returncode, 1, run.stdout + run.stderr)
      self.assertIn("Extraction failed:", run.stdout, run.stdout + run.stderr)
      self.assertNotIn("Traceback", run.stderr)
      self.assertFalse(output.exists())
  def test_pdf_adapter_preserves_layers_and_closes_only_last_subpath(self):
    import pymupdf

    page = MagicMock()
    page.get_drawings.return_value = [
      dict(
        seqno=4,
        layer="STRUCTURE",
        width=0.2,
        closePath=True,
        items=[
          ("l", pymupdf.Point(0, 0), pymupdf.Point(1, 0)),
          ("l", pymupdf.Point(5, 5), pymupdf.Point(6, 5)),
        ],
      ),
      dict(
        seqno=5,
        layer="OPENINGS",
        width=0.1,
        items=[
          ("re", pymupdf.Rect(0, 0, 1, 2), 1),
          ("qu", pymupdf.Quad([(0, 0), (1, 0), (0, 2), (1, 2)])),
          ("c", *[pymupdf.Point(*p) for p in self.paths[4]["curves"][0]]),
        ],
      ),
    ]
    document = MagicMock()
    document.__enter__.return_value = document
    document.__len__.return_value = 1
    document.__getitem__.return_value = page
    source = self.directory / "dummy.pdf"
    source.touch()
    with patch("pymupdf.open", return_value=document):
      paths = pdf_paths(source, 0)
      self.assertEqual(paths[0]["layer"], "STRUCTURE")
      self.assertEqual(paths[0]["segments"][-1], [[6, 5], [5, 5]])
      self.assertNotIn("closed", paths[0])
      self.assertEqual(len(paths[1]["segments"]), 8)
      self.assertEqual(paths[1]["curves"], self.paths[4]["curves"])
      with self.assertRaisesRegex(ValueError, "out of range"):
        pdf_paths(source, 1)
  def test_bad_pdf_and_source_hash_fail_cleanly(self):
    source = self.directory / "bad.pdf"
    source.write_bytes(b"not a pdf")
    with self.assertRaisesRegex(ValueError, "Cannot read source PDF"):
      pdf_paths(source, 0)
    config = self.directory / "config.yaml"
    overrides = self.directory / "overrides.yaml"
    config.write_text(yaml.safe_dump(dict(self.config, source_sha256="0" * 64)))
    overrides.write_text(yaml.safe_dump(self.overrides))
    with self.assertRaisesRegex(ValueError, "hash"):
      write_map(source, config, overrides, self.directory / "out.yaml")
  def test_hairline_and_manual_proxies_are_marked_estimated(self):
    paths = copy.deepcopy(self.paths)
    for path in paths:
      path["width"] = 0
    overrides = dict(
      self.overrides,
      walls=[dict(id="repair", start=[0, 1], end=[0, 2], width=0.4, z_min=0.7, z_max=0.9)],
    )
    spec, _ = extract(paths, self.config, overrides)
    self.assertTrue(all(wall["estimated"] is True for wall in spec["walls"]))
    for wall in spec["walls"]:
      if wall["id"] != "repair":
        self.assertEqual(wall["size"][1], 0.1)
        self.assertEqual([wall["z_min"], wall["z_max"]], [0, 2.5])
    repair = next(wall for wall in spec["walls"] if wall["id"] == "repair")
    self.assertEqual([repair["z_min"], repair["z_max"]], [0.7, 0.9])
    self.assertTrue(all(p["estimated"] is True for p in self.compiled(spec)["primitives"]))
  def test_unexpected_missing_internal_key_is_not_hidden(self):
    argv = [
      "plan_extract",
      "missing.pdf",
      "--config",
      "config.yaml",
      "--overrides",
      "overrides.yaml",
      "--output",
      str(self.directory / "out.yaml"),
    ]
    with patch("sys.argv", argv), patch(
      "tools.plan_extract.write_map", side_effect=KeyError("seqno")
    ):
      with self.assertRaisesRegex(KeyError, "seqno"):
        main()
