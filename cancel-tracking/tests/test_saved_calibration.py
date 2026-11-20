"""Damaged saved data and interrupted writes must not become live calibration."""

import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from magtrack_src.calibration.store import CalibrationFile
from magtrack_src.persistence import temporary_json, write_json_atomic
from magtrack_src.position.coil_model import Coil
from magtrack_src.position.locate import Locator
from tests.test_home_free_tracking import FRAME


IDENTITY = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
DEFAULTS = (None, (0, 0, 0), (1, 1, 1))


class SavedCalibration(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.store = CalibrationFile(self.root / "sensor.json", self.root / "tumble.json",
                                     self.root / "mounting.json")
        self.path = self.root / "coil.json"

    def tearDown(self):
        self.directory.cleanup()

    def write(self, path, data):
        path.write_text(json.dumps(data), encoding="utf-8")

    def coil_data(self):
        return {"format_version": 2, "field_kind": "absolute_coil_field", "frame": dict(FRAME), "coil": Coil((0, 0, 0), (0, 23400, 0), 2.2).as_dict(),
                "points": [{"position": [2, -12, 1], "coil_field_ut": [1, 2, 3], "spread_ut": 0.2}],
                "checks": [[0.1, None]]}

    def assert_rejected_coil(self, data):
        self.write(self.path, data)
        original = self.path.read_bytes()
        locator = Locator(self.path)
        self.assertIsNone(locator.coil)
        self.assertEqual(locator.points, [])
        self.assertEqual(locator.checks, [])
        self.assertIsNone(locator.frame)
        self.assertIn("was not loaded", locator.message)
        self.assertEqual(self.path.read_bytes(), original)
        json.dumps(locator.status(), allow_nan=False)

    def test_missing_files_are_normal_first_run(self):
        self.assertEqual(self.store.load(), DEFAULTS)
        self.assertIsNone(self.store.load_error)
        self.assertIsNone(self.store.load_axes())
        self.assertIsNone(self.store.mounting_load_error)
        self.assertIsNone(Locator(self.path).message)
        self.assertIsNone(CalibrationFile(self.root / "x", self.root / "y").load_axes())

    def test_legacy_sensor_defaults_and_current_round_trip(self):
        self.write(self.store.path, {"offset": [1, 2, 3]})
        self.assertEqual(self.store.load(), ((1, 2, 3), (0, 0, 0), (1, 1, 1)))
        self.store.save((1.123456789, 2, 3), (0.1, 0.2, 0.3), (0.9, 1, 1.1))
        self.assertEqual(self.store.load(), ((1.123456789, 2, 3), (0.1, 0.2, 0.3), (0.9, 1, 1.1)))

    def test_sensor_vectors_and_gains_are_validated_as_one_record(self):
        good = {"offset": [1, 2, 3], "bias": [0, 0, 0], "gains": [1, 1, 1]}
        bad_vectors = (None, [], [1, 2], [1, 2, 3, 4], "123", {"x": 1},
                       [True, 2, 3], ["1", 2, 3], [math.nan, 2, 3], [math.inf, 2, 3],
                       [1e308, 2, 3], [10 ** 400, 2, 3])
        for field in good:
            for bad in bad_vectors:
                with self.subTest(field=field, bad=bad):
                    data = dict(good, **{field: bad})
                    self.write(self.store.path, data)
                    original = self.store.path.read_bytes()
                    self.assertEqual(self.store.load(), DEFAULTS)
                    self.assertIn("were not loaded", self.store.load_error)
                    self.assertEqual(self.store.path.read_bytes(), original)
        for gain in (0, -1):
            self.write(self.store.path, dict(good, gains=[gain, 1, 1]))
            self.assertEqual(self.store.load(), DEFAULTS)

    def test_invalid_json_encodings_and_top_level_types_do_not_crash_startup(self):
        for raw in (b"{", b"null", b"[]", b"42", b'"text"', b"{}", b"\xff"):
            with self.subTest(raw=raw):
                for path in (self.store.path, self.store.mounting_path, self.path):
                    path.write_bytes(raw)
                self.assertEqual(self.store.load(), DEFAULTS)
                self.assertIsNotNone(self.store.load_error)
                self.assertIsNone(self.store.load_axes())
                self.assertIsNotNone(self.store.mounting_load_error)
                self.assertIsNotNone(Locator(self.path).message)

    def test_unreadable_paths_report_load_errors(self):
        for path in (self.store.path, self.store.mounting_path, self.path):
            path.mkdir()
        self.assertEqual(self.store.load(), DEFAULTS)
        self.assertIsNotNone(self.store.load_error)
        self.assertIsNone(self.store.load_axes())
        self.assertIsNotNone(self.store.mounting_load_error)
        self.assertIsNotNone(Locator(self.path).message)

    def test_mounting_accepts_rotations_reflections_and_existing_saved_mapping(self):
        for axes in (IDENTITY, [[1, 0, 0], [0, 1, 0], [0, 0, -1]],
                     json.loads((Path(__file__).parents[1] / "data/calibration/mmc_mounting.json").read_text())["axes"]):
            with self.subTest(axes=axes):
                self.store.save_axes(axes)
                self.assertEqual(self.store.load_axes(), axes)
                self.assertIsNone(self.store.mounting_load_error)

    def test_mounting_rejects_bad_shape_scaling_and_shear(self):
        cases = (None, [], IDENTITY[:2], IDENTITY + [[0, 0, 1]], [[1, 0], [0, 1], [0, 0]],
                 [[2, 0, 0], [0, 1, 0], [0, 0, 1]], [[1, 0, 0], [1, 0, 0], [0, 0, 1]],
                 [[math.nan, 0, 0], [0, 1, 0], [0, 0, 1]],
                 [[1, 0, 0], [0.01, 1, 0], [0, 0, 1]])
        for axes in cases:
            with self.subTest(axes=axes):
                self.write(self.store.mounting_path, {"axes": axes})
                self.assertIsNone(self.store.load_axes())
                self.assertIn("was not loaded", self.store.mounting_load_error)

    def test_invalid_save_keeps_previous_sensor_and_mounting_files(self):
        self.store.save((1, 2, 3), (0, 0, 0), (1, 1, 1))
        self.store.save_axes(IDENTITY)
        for operation, path in ((lambda: self.store.save((math.nan, 0, 0), (0, 0, 0), (1, 1, 1)), self.store.path),
                                (lambda: self.store.save((0, 0, 0), (0, 0, 0), (0, 1, 1)), self.store.path),
                                (lambda: self.store.save_axes([[2, 0, 0], [0, 1, 0], [0, 0, 1]]), self.store.mounting_path)):
            before = path.read_bytes()
            with self.assertRaises(ValueError):
                operation()
            self.assertEqual(path.read_bytes(), before)

    def test_each_sensor_artifact_preserves_its_previous_version_on_replace_failure(self):
        self.store.save((1, 2, 3), (0, 0, 0), (1, 1, 1))
        self.store.save_axes(IDENTITY)
        self.store.log_tumble([(1, 2, 3)], {"passed": True})
        operations = ((lambda: self.store.save((4, 5, 6), (0, 0, 0), (1, 1, 1)), self.store.path),
                      (lambda: self.store.save_axes([[0, 1, 0], [1, 0, 0], [0, 0, 1]]), self.store.mounting_path),
                      (lambda: self.store.log_tumble([(4, 5, 6)], None), self.store.tumble_log_path))
        for operation, path in operations:
            with self.subTest(path=path.name):
                before = path.read_bytes()
                with patch.object(Path, "replace", side_effect=OSError("replacement denied")):
                    with self.assertRaises(OSError):
                        operation()
                self.assertEqual(path.read_bytes(), before)
                self.assertEqual(list(self.root.glob("*.tmp")), [])

    def test_current_coil_data_loads_without_rounding(self):
        data = self.coil_data()
        data["points"][0]["coil_field_ut"][0] = 1.123456789
        self.write(self.path, data)
        locator = Locator(self.path)
        self.assertIsNone(locator.message)
        self.assertEqual(locator.points[0].coil_field_ut[0], 1.123456789)
        self.assertEqual(locator.checks, [(0.1, None)])
        locator._save()
        self.assertEqual(json.loads(self.path.read_text()), data)

    def test_old_reference_relative_data_is_not_reinterpreted_or_overwritten(self):
        data = {"home": [0, -13, 1], "points": [{"position": [1, 2, 3], "error": [4, 5, 6]}]}
        self.write(self.path, data)
        before = self.path.read_bytes()
        locator = Locator(self.path)
        self.assertEqual(locator.points, [])
        self.assertTrue(locator.status()["legacy_calibration"])
        locator._save()
        self.assertEqual(self.path.read_bytes(), before)
        with self.assertRaises(ValueError):
            locator.capture_point((1, 2, 3))

    def test_invalid_measurements_reject_whole_coil_record(self):
        for field, values in (("position", (None, [1, 2], [1, 2, 3, 4], [True, 1, 2], [1e308, 1, 2])),
                              ("coil_field_ut", (None, [1, 2], [math.nan, 1, 2], [math.inf, 1, 2], "123")),
                              ("spread_ut", (-1, None, True, "0.1", math.inf))):
            for bad in values:
                with self.subTest(field=field, bad=bad):
                    data = self.coil_data()
                    data["points"][0][field] = bad
                    self.assert_rejected_coil(data)
        for bad in (None, {}, [None], [[]]):
            self.assert_rejected_coil(dict(self.coil_data(), points=bad))

    def test_invalid_model_and_frame_are_not_partly_installed(self):
        for field, values in (("center", (None, [1, 2], [0, 0, math.inf])),
                              ("s", ([0, 0, 0], [0, math.nan, 0], [0, 1e308, 0])),
                              ("half_length", (None, True, -1, 0, 0.1, 7, math.inf))):
            for bad in values:
                with self.subTest(field=field, bad=bad):
                    data = self.coil_data()
                    data["coil"][field] = bad
                    self.assert_rejected_coil(data)
        for bad in ([], False, 1):
            self.assert_rejected_coil(dict(self.coil_data(), coil=bad))
        for bad in (None, [], {}, {"id": "map"}):
            self.assert_rejected_coil(dict(self.coil_data(), frame=bad))
        data = self.coil_data()
        data["frame"] = dict(FRAME, ambient_field=[0, 0, 40])
        self.assert_rejected_coil(data)

    def test_invalid_check_distances_cannot_leak_into_status_json(self):
        for bad in (None, {}, [[1]], [[1, 2, 3]], [[-1, 2]], [[math.nan, 2]],
                    [[1, True]], [[1, "2"]], [[1, 2], [1, 2]]):
            with self.subTest(checks=bad):
                self.assert_rejected_coil(dict(self.coil_data(), checks=bad))

    def test_load_errors_clear_after_file_is_repaired(self):
        self.write(self.store.path, [])
        self.store.load()
        self.assertIsNotNone(self.store.load_error)
        self.store.save((1, 2, 3), (0, 0, 0), (1, 1, 1))
        self.store.load()
        self.assertIsNone(self.store.load_error)

    def test_invalid_snapshot_keeps_previous_coil_file(self):
        data = self.coil_data()
        self.write(self.path, data)
        locator = Locator(self.path)
        locator.coil.s = (0, math.nan, 0)
        locator._save()
        self.assertIn("Couldn't save", locator.message)
        self.assertEqual(json.loads(self.path.read_text()), data)
        self.assertEqual(list(self.root.glob("*.tmp")), [])


class AtomicJsonWrites(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.path = self.root / "calibration.json"
        self.path.write_text('{"old": true}', encoding="utf-8")

    def tearDown(self):
        self.directory.cleanup()

    def assert_preserved(self):
        self.assertEqual(json.loads(self.path.read_text()), {"old": True})
        self.assertEqual(list(self.root.glob("*.tmp")), [])

    def test_destination_stays_old_until_complete_file_is_replaced(self):
        with temporary_json(self.path, {"new": True}) as temporary:
            self.assertEqual(temporary.parent, self.path.parent)
            self.assertEqual(json.loads(self.path.read_text()), {"old": True})
            self.assertEqual(json.loads(temporary.read_text()), {"new": True})
            temporary.replace(self.path)
        self.assertEqual(json.loads(self.path.read_text()), {"new": True})
        self.assertFalse(temporary.exists())

    def test_abandoned_write_is_cleaned_up(self):
        with temporary_json(self.path, {"new": True}):
            pass
        self.assert_preserved()

    def test_failed_flush_or_replace_keeps_last_good_file_and_retry_succeeds(self):
        for target in ("magtrack_src.persistence.os.fsync", "pathlib.Path.replace"):
            with self.subTest(stage=target):
                with patch(target, side_effect=OSError("disk failure")):
                    with self.assertRaises(OSError):
                        write_json_atomic(self.path, {"new": True})
                self.assert_preserved()
        write_json_atomic(self.path, {"new": True})
        self.assertEqual(json.loads(self.path.read_text()), {"new": True})

    def test_partial_write_failure_keeps_last_good_file(self):
        import magtrack_src.persistence as persistence
        real_open = persistence.tempfile.NamedTemporaryFile
        def interrupted(*args, **kwargs):
            output = real_open(*args, **kwargs)
            def write_then_fail(value):
                output.file.write(value[:5])
                raise OSError("interrupted write")
            output.write = write_then_fail
            return output
        with patch.object(persistence.tempfile, "NamedTemporaryFile", side_effect=interrupted):
            with self.assertRaises(OSError):
                write_json_atomic(self.path, {"new": True})
        self.assert_preserved()

    def test_nonfinite_json_and_creation_errors_preserve_destination(self):
        for data in ({"field": math.nan}, {"field": math.inf}, {"field": -math.inf}):
            with self.assertRaises(ValueError):
                write_json_atomic(self.path, data)
            self.assert_preserved()
        with patch("magtrack_src.persistence.tempfile.NamedTemporaryFile", side_effect=OSError("cannot create")):
            with self.assertRaises(OSError):
                write_json_atomic(self.path, {"new": True})
        self.assert_preserved()
