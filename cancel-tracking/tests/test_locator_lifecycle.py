"""Controlled operation interleavings: no sleeps or sensor hardware required."""

import json
import math
import tempfile
import threading
import unittest
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

from magtrack_src.config import LOCATE_POINT_SAMPLES
from magtrack_src.position.coil_model import Coil
from magtrack_src.position.locate import Locator
from magtrack_src.position.measurements import CoilCalibrationMeasurement
from magtrack_src.persistence import temporary_json


class LocatorLifecycle(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / "coil.json"
        self.locator = Locator(self.path)
        self.locator.home = (0.0, -13.0, 0.0)
        self.model = Coil((0, 0, 0), (0, 23400, 0), 2.2)
        self.locator.coil = self.model
        self.locator._home_field = self.model.field(self.locator.home)
        self.locator.points = [CoilCalibrationMeasurement((i, -10, 1), (i, 2, 3), 0.1)
                               for i in range(4)]
        self.original_points = list(self.locator.points)
        self.threads, self.releases, self.errors = [], [], []
        self.thread_class = threading.Thread

    def tearDown(self):
        for release in self.releases:
            release.set()
        for thread in self.threads:
            thread.join(5)
        try:
            self.assertFalse(any(thread.is_alive() for thread in self.threads))
            self.assertEqual(self.errors, [])
        finally:
            self.directory.cleanup()

    def event(self):
        event = threading.Event()
        self.releases.append(event)
        return event

    def make_thread(self, *, target, args=(), daemon=True):
        def run():
            try:
                target(*args)
            except BaseException as error:
                self.errors.append(error)
        thread = self.thread_class(target=run, daemon=daemon)
        self.threads.append(thread)
        return thread

    def run_async(self, target):
        thread = self.make_thread(target=target)
        thread.start()
        return thread

    def begin_fit(self):
        with patch("magtrack_src.position.locate.threading.Thread", side_effect=self.make_thread):
            with self.locator._lock:
                self.locator._start_fit()
        return self.threads[-1]

    def test_clear_cancels_a_partly_collected_measurement(self):
        self.locator.capture_point((2, -10, 1))
        with patch("magtrack_src.position.locate.solve", return_value=((0, -13, 0), 0)):
            self.locator.update({"error": [1, 2, 3], "predicted": [10, 20, 30]})
            self.locator.clear_points()
            self.assertFalse(self.locator.status()["capturing"])
            for _ in range(LOCATE_POINT_SAMPLES + 1):
                self.locator.update({"error": [1, 2, 3], "predicted": [10, 20, 30]})
        self.assertEqual(self.locator.points, [])
        self.assertIsNone(self.locator.coil)
        self.assertEqual(json.loads(self.path.read_text())["points"], [])

    def test_clear_discards_a_running_fit_and_resets_fitting_status(self):
        entered, release = threading.Event(), self.event()
        def fitting(*_):
            entered.set()
            self.assertTrue(release.wait(5))
            return self.model
        with patch("magtrack_src.position.locate.fit_coil", side_effect=fitting), \
             patch("magtrack_src.position.locate.check_points", return_value=[0] * 4), \
             patch("magtrack_src.position.locate.left_out_check", return_value=[0] * 4):
            worker = self.begin_fit()
            self.assertTrue(entered.wait(5))
            self.locator.clear_points()
            self.assertFalse(self.locator.status()["fitting"])
            release.set()
            worker.join(5)
        self.assertIsNone(self.locator.coil)
        self.assertEqual(self.locator.checks, [])
        self.assertIsNone(json.loads(self.path.read_text())["coil"])

    def test_old_fit_cannot_replace_new_fit_even_when_input_values_repeat(self):
        entered, release = threading.Event(), self.event()
        new_model = Coil((0.1, 0, 0), (0, 25000, 0), 2.3)
        calls = []
        def fitting(*_):
            calls.append(threading.current_thread())
            if len(calls) == 1:
                entered.set()
                self.assertTrue(release.wait(5))
                return self.model
            return new_model
        with patch("magtrack_src.position.locate.fit_coil", side_effect=fitting), \
             patch("magtrack_src.position.locate.check_points", return_value=[0] * 4), \
             patch("magtrack_src.position.locate.left_out_check", return_value=[0] * 4):
            old_worker = self.begin_fit()
            self.assertTrue(entered.wait(5))
            self.locator.clear_points()
            with self.locator._lock:
                self.locator.points = list(self.original_points)
            new_worker = self.begin_fit()
            new_worker.join(5)
            self.assertIs(self.locator.coil, new_model)
            release.set()
            old_worker.join(5)
        self.assertIs(self.locator.coil, new_model)
        self.assertFalse(self.locator.status()["fitting"])
        self.assertEqual(json.loads(self.path.read_text())["coil"], new_model.as_dict())

    def test_solve_does_not_publish_after_clear(self):
        entered, release = threading.Event(), self.event()
        result = []
        def solving(*_):
            entered.set()
            self.assertTrue(release.wait(5))
            return (1, -12, 2), 0
        with patch("magtrack_src.position.locate.solve", side_effect=solving):
            worker = self.run_async(lambda: result.append(self.locator.update(
                {"error": [1, 2, 3], "predicted": [10, 20, 30]})))
            self.assertTrue(entered.wait(5))
            self.locator.clear_points()
            release.set()
            worker.join(5)
        self.assertIsNone(result[0]["position"])
        self.assertIn("changed", result[0]["why"])
        self.assertIsNone(self.locator._last)

    def test_old_fit_does_not_clear_the_new_workers_fitting_flag(self):
        old_entered, new_entered = threading.Event(), threading.Event()
        old_release, new_release = self.event(), self.event()
        calls = []
        def fitting(*_):
            calls.append(threading.current_thread())
            entered, release = ((old_entered, old_release) if len(calls) == 1
                                else (new_entered, new_release))
            entered.set()
            self.assertTrue(release.wait(5))
            return self.model
        with patch("magtrack_src.position.locate.fit_coil", side_effect=fitting), \
             patch("magtrack_src.position.locate.check_points", return_value=[0] * 4), \
             patch("magtrack_src.position.locate.left_out_check", return_value=[0] * 4):
            old_worker = self.begin_fit()
            self.assertTrue(old_entered.wait(5))
            new_worker = self.begin_fit()
            self.assertTrue(new_entered.wait(5))
            old_release.set()
            old_worker.join(5)
            self.assertTrue(self.locator.status()["fitting"])
            new_release.set()
            new_worker.join(5)
        self.assertFalse(self.locator.status()["fitting"])

    def test_solve_does_not_overwrite_a_new_reference_position(self):
        entered, release = threading.Event(), self.event()
        result = []
        def solving(*_):
            if threading.current_thread() is not threading.main_thread():
                entered.set()
                self.assertTrue(release.wait(5))
                return (1, -12, 2), 0
            return (2, -11, 3), 0
        with patch("magtrack_src.position.locate.solve", side_effect=solving):
            worker = self.run_async(lambda: result.append(self.locator.update(
                {"error": [1, 2, 3], "predicted": [10, 20, 30]})))
            self.assertTrue(entered.wait(5))
            current = self.locator.update({"error": [1, 2, 3], "predicted": [11, 20, 30]})
            release.set()
            worker.join(5)
        self.assertEqual(current["position"], [2, -11, 3])
        self.assertIsNone(result[0]["position"])
        self.assertEqual(self.locator._last, (2, -11, 3))

    def test_save_retries_if_clear_happens_after_its_snapshot(self):
        entered, cleared, release = threading.Event(), threading.Event(), self.event()
        real_replace, real_save = Path.replace, self.locator._save
        installed = []
        first = []
        @contextmanager
        def writing(path, *args, **kwargs):
            if not first:
                first.append(True)
                entered.set()
                self.assertTrue(release.wait(5))
            with temporary_json(path, *args, **kwargs) as tmp:
                yield tmp
        def replacing(path, target):
            installed.append(json.loads(path.read_text()))
            return real_replace(path, target)
        def saving():
            if self.locator.points == []:
                cleared.set()
            real_save()
        with patch("magtrack_src.position.locate.temporary_json", writing), patch.object(Path, "replace", replacing), \
             patch.object(self.locator, "_save", side_effect=saving):
            old_save = self.run_async(self.locator._save)
            self.assertTrue(entered.wait(5))
            clear = self.run_async(self.locator.clear_points)
            self.assertTrue(cleared.wait(5))
            release.set()
            old_save.join(5)
            clear.join(5)
        self.assertTrue(installed)
        self.assertTrue(all(snapshot["points"] == [] for snapshot in installed))
        self.assertEqual(json.loads(self.path.read_text())["points"], [])
        self.assertEqual(list(self.path.parent.glob("*.tmp")), [])

    def test_invalid_fields_cannot_enter_point_capture(self):
        self.locator.capture_point((2, -10, 1))
        with patch("magtrack_src.position.locate.solve") as solving:
            for field in ([math.nan, 1, 2], [math.inf, 1, 2], [1, 2], [1, 2, 3, 4], ["bad", 1, 2]):
                with self.subTest(field=field):
                    result = self.locator.update({"error": field})
                    self.assertIsNone(result["position"])
                    self.assertIn("invalid", result["why"])
            solving.assert_not_called()
        self.assertEqual(self.locator.status()["capture_done"], 0)

    def test_obsolete_save_error_does_not_replace_a_newer_status(self):
        entered, cleared, release = threading.Event(), threading.Event(), self.event()
        real_save = self.locator._save
        first = []
        @contextmanager
        def writing(path, *args, **kwargs):
            if not first:
                first.append(True)
                entered.set()
                self.assertTrue(release.wait(5))
                raise OSError("obsolete write failed")
            with temporary_json(path, *args, **kwargs) as tmp:
                yield tmp
        def saving():
            if self.locator.points == []:
                cleared.set()
            real_save()
        with patch("magtrack_src.position.locate.temporary_json", writing), \
             patch.object(self.locator, "_save", side_effect=saving):
            old_save = self.run_async(self.locator._save)
            self.assertTrue(entered.wait(5))
            clear = self.run_async(self.locator.clear_points)
            self.assertTrue(cleared.wait(5))
            with self.locator._lock:
                self.locator.message = "Newer operation status"
            release.set()
            old_save.join(5)
            clear.join(5)
        self.assertEqual(self.locator.status()["message"], "Newer operation status")
        self.assertEqual(json.loads(self.path.read_text())["points"], [])

    def test_nonfinite_sensitivity_is_not_published(self):
        with patch("magtrack_src.position.locate.solve", return_value=((0, -13, 0), 0)), \
             patch("magtrack_src.position.locate.mm_per_ut", return_value=math.nan):
            result = self.locator.update({"error": [1, 2, 3], "predicted": None})
        self.assertIsNone(result["position"])
        self.assertIn("invalid sensitivity", result["why"])
        self.assertIsNone(self.locator._last)
        json.dumps(result, allow_nan=False)

    def test_invalid_solver_results_never_replace_the_last_position(self):
        self.locator._last = (0, -13, 0)
        sample = {"error": [1, 2, 3], "predicted": None}
        for position, mismatch in (((math.nan, 0, 0), 0), ((0, -13, 0), math.nan),
                                   ((math.inf, 0, 0), 0), ((1e308, 0, 0), 0)):
            with self.subTest(position=position, mismatch=mismatch), \
                 patch("magtrack_src.position.locate.solve", return_value=(position, mismatch)):
                result = self.locator.update(sample)
                self.assertIsNone(result["position"])
                self.assertIn("invalid", result["why"])
                self.assertEqual(self.locator._last, (0, -13, 0))

