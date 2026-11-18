"""Keep numerical work and recording formats usable without their live consumers."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from magtrack_src.position.locate import Locator
from tests.test_home_free_tracking import FRAME
from magtrack_src.position.measurements import CoilCalibrationMeasurement


class ModuleBoundaries(unittest.TestCase):
    def imported_project_modules(self, *names):
        program = (
            "import importlib, json, sys\n"
            f"for name in {names!r}: importlib.import_module(name)\n"
            "print(json.dumps([name for name in sys.modules if name.startswith('magtrack_src.')]))"
        )
        result = subprocess.run([sys.executable, "-c", program],
                                cwd=Path(__file__).resolve().parents[1],
                                check=True, capture_output=True, text=True)
        return json.loads(result.stdout)

    def test_calculations_import_without_live_tracking_or_io_layers(self):
        modules = self.imported_project_modules(
            "magtrack_src.position.coil_model", "magtrack_src.position.coil_fit",
            "magtrack_src.position.position_solver")
        for prefix in ("magtrack_src.hardware", "magtrack_src.web", "magtrack_src.recording",
                       "magtrack_src.position.locate"):
            self.assertFalse(any(name.startswith(prefix) for name in modules), prefix)

    def test_recording_reader_and_comparison_do_not_import_plotting(self):
        modules = self.imported_project_modules("magtrack_src.recording.schema",
                                                "magtrack_src.recording.compare")
        self.assertNotIn("magtrack_src.recording.log_plot", modules)
        self.assertNotIn("magtrack_src.recording.data_log", modules)

    def test_previous_public_imports_remain_available(self):
        from magtrack_src.position import coil_fit, coil_model, locate, position_solver
        from magtrack_src.recording import data_log, log_plot, schema
        for name, module in (("Coil", coil_model), ("mm_per_ut", coil_model),
                             ("least_squares", coil_fit), ("fit_coil", coil_fit),
                             ("solve", position_solver), ("check_points", position_solver),
                             ("left_out_check", position_solver)):
            self.assertIs(getattr(locate, name), getattr(module, name))
        self.assertIs(log_plot.SAMPLE_COLUMNS, schema.SAMPLE_COLUMNS)
        self.assertIs(log_plot.read_samples, schema.read_samples)
        self.assertIs(data_log.RAW_COLUMNS, schema.RAW_COLUMNS)

    def test_measurement_files_retain_field_meaning_and_precision(self):
        point = {"position": [4.123456789, -8.987654321, 1.246813579],
                 "coil_field_ut": [-12.876543219, 3.192837465, 42.918273645],
                 "spread_ut": 0.123456789}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "coil.json"
            path.write_text(json.dumps({"format_version": 2, "field_kind": "absolute_coil_field", "frame": FRAME, "coil": None,
                                        "points": [point]}), encoding="utf-8")
            locator = Locator(path)
            measurement = locator.points[0]
            self.assertIsInstance(measurement, CoilCalibrationMeasurement)
            self.assertEqual(measurement.position_cm, tuple(point["position"]))
            self.assertEqual(measurement.coil_field_ut, tuple(point["coil_field_ut"]))
            self.assertEqual(measurement.max_field_deviation_ut, point["spread_ut"])
            locator._save()
            self.assertEqual(json.loads(path.read_text())["points"], [point])
