"""Calibration controls overlapping numerical work and disk writes."""

import json
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from magtrack_src.calibration.store import CalibrationFile
from magtrack_src.tracking.tracker import IDENTITY, Tracker


def fitted(center=2.0, passed=True):
    return {"center": (center, 3.0, 4.0), "gains": (1.1, 1.0, 0.9), "passed": passed}


class ObservedLock:
    """Signal an acquisition attempt before it can block on the real lock."""
    def __init__(self):
        self.lock = threading.Lock()
        self.attempt = None

    def __enter__(self):
        if self.attempt is not None:
            self.attempt.set()
        self.lock.acquire()

    def __exit__(self, *_):
        self.lock.release()


class TrackerLifecycle(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        root = Path(self.directory.name)
        self.store = CalibrationFile(root / "cal.json", root / "tumble.json", root / "mounting.json")
        self.tracker = Tracker(offset=(7, 8, 9), store=self.store, fit_mounting=True)
        self.threads, self.releases, self.errors = [], [], []
        self.mounting = patch("magtrack_src.tracking.tracker.fit_axes", return_value={"axes": IDENTITY})
        self.mounting.start()

    def tearDown(self):
        for release in self.releases:
            release.set()
        for thread in self.threads:
            thread.join(5)
        self.mounting.stop()
        try:
            self.assertFalse(any(thread.is_alive() for thread in self.threads))
            self.assertEqual(self.errors, [])
        finally:
            self.directory.cleanup()

    def event(self):
        release = threading.Event()
        self.releases.append(release)
        return release

    def run_async(self, operation):
        def run():
            try:
                operation()
            except BaseException as error:
                self.errors.append(error)
        thread = threading.Thread(target=run, daemon=True)
        self.threads.append(thread)
        thread.start()
        return thread

    def ready(self):
        self.tracker.tumble_start()
        # Numerical accuracy/coverage have separate simulation tests. These
        # fixtures control only the moment when computation or saving finishes.
        self.tracker._tumble = SimpleNamespace(ready=True, points=[(1, 2, 3)], quats=[(1, 0, 0, 0)])

    def assert_cleared(self):
        self.assertIsNone(self.tracker.offset)
        self.assertIsNone(self.tracker.tumble_result)
        self.assertEqual(self.tracker.gains, (1, 1, 1))
        self.assertFalse(self.store.path.exists())
        self.assertFalse(self.store.tumble_log_path.exists())

    def test_reset_discards_a_result_still_in_either_fitting_stage(self):
        for stage in ("fit_tumble", "fit_axes"):
            with self.subTest(stage=stage):
                entered, release = threading.Event(), self.event()
                def blocked(*_):
                    entered.set()
                    self.assertTrue(release.wait(5))
                    return fitted() if stage == "fit_tumble" else {"axes": IDENTITY}
                self.ready()
                with patch("magtrack_src.tracking.tracker.fit_tumble", return_value=fitted()), \
                     patch("magtrack_src.tracking.tracker." + stage, side_effect=blocked):
                    worker = self.run_async(self.tracker.tumble_finish)
                    self.assertTrue(entered.wait(5))
                    self.tracker.reset_calibration()
                    message = self.tracker.message
                    release.set()
                    worker.join(5)
                self.assert_cleared()
                self.assertEqual(self.tracker.message, message)

    def test_newer_tumble_result_survives_an_older_result(self):
        entered, release = threading.Event(), self.event()
        def fitting(*_):
            if not entered.is_set():
                entered.set()
                self.assertTrue(release.wait(5))
                return fitted(2)
            return fitted(9)
        self.ready()
        with patch("magtrack_src.tracking.tracker.fit_tumble", side_effect=fitting):
            worker = self.run_async(self.tracker.tumble_finish)
            self.assertTrue(entered.wait(5))
            self.ready()  # Even identical points belong to a different attempt.
            self.tracker.tumble_finish()
            expected = self.tracker.offset
            release.set()
            worker.join(5)
        self.assertAlmostEqual(expected[0], 9.9)
        self.assertEqual(self.tracker.offset, expected)
        self.assertEqual(self.store.load()[0], expected)
        self.assertEqual(json.loads(self.store.tumble_log_path.read_text())["result"]["center"][0], 9)

    def test_cancel_or_toggle_discards_a_running_calculation(self):
        for control in (self.tracker.tumble_cancel, lambda: self.tracker.set_calibration(False)):
            with self.subTest(control=control):
                entered, release = threading.Event(), self.event()
                def fitting(*_):
                    entered.set()
                    self.assertTrue(release.wait(5))
                    return fitted()
                self.ready()
                with patch("magtrack_src.tracking.tracker.fit_tumble", side_effect=fitting):
                    worker = self.run_async(self.tracker.tumble_finish)
                    self.assertTrue(entered.wait(5))
                    control()
                    release.set()
                    worker.join(5)
                self.assertEqual(self.tracker.offset, (7, 8, 9))
                self.assertFalse(self.store.path.exists())
                self.assertFalse(self.store.tumble_log_path.exists())

    def test_old_failure_cannot_replace_new_status_or_attempt_log(self):
        entered, release = threading.Event(), self.event()
        def failing(*_):
            entered.set()
            self.assertTrue(release.wait(5))
            raise ValueError("old attempt failed")
        self.ready()
        with patch("magtrack_src.tracking.tracker.fit_tumble", side_effect=failing):
            worker = self.run_async(self.tracker.tumble_finish)
            self.assertTrue(entered.wait(5))
            self.ready()
            release.set()
            worker.join(5)
        self.assertIsNone(self.tracker.message)
        self.assertIsNotNone(self.tracker._tumble)
        self.assertFalse(self.store.tumble_log_path.exists())

    def test_reset_waits_for_each_save_stage_then_clears_it(self):
        for stage in ("log_tumble", "save", "save_axes"):
            with self.subTest(stage=stage):
                entered, release = threading.Event(), self.event()
                observed = ObservedLock()
                self.tracker._calibration_commit_lock = observed
                original = getattr(self.store, stage)
                def writing(*args):
                    entered.set()
                    self.assertTrue(release.wait(5))
                    original(*args)
                self.ready()
                with patch("magtrack_src.tracking.tracker.fit_tumble", return_value=fitted()), \
                     patch.object(self.store, stage, side_effect=writing):
                    fit_worker = self.run_async(self.tracker.tumble_finish)
                    self.assertTrue(entered.wait(5))
                    # Disk I/O must not hold the sensor's state lock.
                    status_done = threading.Event()
                    def read_status():
                        self.tracker.status()
                        status_done.set()
                    self.run_async(read_status)
                    self.assertTrue(status_done.wait(5))
                    observed.attempt = threading.Event()
                    reset_done = threading.Event()
                    def reset():
                        self.tracker.reset_calibration()
                        reset_done.set()
                    reset_worker = self.run_async(reset)
                    self.assertTrue(observed.attempt.wait(5))
                    self.assertFalse(reset_done.is_set())
                    release.set()
                    fit_worker.join(5)
                    reset_worker.join(5)
                self.assertTrue(reset_done.is_set())
                self.assert_cleared()
                self.assertEqual(self.store.load_axes(), [list(row) for row in IDENTITY])

    def test_either_fit_failure_preserves_corrections_and_logs_failed_attempt(self):
        self.store.save((7, 8, 9), (0, 0, 0), (1, 1, 1))
        for stage in ("fit_tumble", "fit_axes"):
            with self.subTest(stage=stage):
                self.ready()
                with patch("magtrack_src.tracking.tracker.fit_tumble", return_value=fitted()), \
                     patch("magtrack_src.tracking.tracker." + stage, side_effect=ValueError("not enough data")):
                    self.tracker.tumble_finish()
                self.assertEqual(self.tracker.offset, (7, 8, 9))
                self.assertEqual(self.store.load()[0], (7, 8, 9))
                self.assertIn("not enough data", self.tracker.message)
                self.assertIsNone(json.loads(self.store.tumble_log_path.read_text())["result"])

    def test_completed_poor_quality_fit_is_still_applied_and_saved(self):
        self.ready()
        self.tracker._block = [(20, 30, 40)]
        with patch("magtrack_src.tracking.tracker.fit_tumble", return_value=fitted(passed=False)):
            self.tracker.tumble_finish()
        self.assertFalse(self.tracker.tumble_result["passed"])
        self.assertEqual(self.store.load()[0], self.tracker.offset)
        self.assertEqual(self.tracker._block, [])
        self.assertIsNotNone(self.tracker._capture)

    def test_disk_failures_report_that_applied_corrections_may_not_be_saved(self):
        for stage in ("log_tumble", "save", "save_axes"):
            with self.subTest(stage=stage):
                self.ready()
                with patch("magtrack_src.tracking.tracker.fit_tumble", return_value=fitted()), \
                     patch.object(self.store, stage, side_effect=OSError("disk unavailable")):
                    with self.assertRaises(OSError):
                        self.tracker.tumble_finish()
                self.assertIsNotNone(self.tracker.tumble_result)
                self.assertIn("applied, but could not save all", self.tracker.message)
                self.assertIn("disk unavailable", self.tracker.message)
                self.tracker.reset_calibration()  # Failure must release the commit lock.
                self.assert_cleared()

    def test_failed_attempt_log_error_retains_the_calculation_error(self):
        self.ready()
        with patch("magtrack_src.tracking.tracker.fit_tumble", side_effect=ValueError("bad field")), \
             patch.object(self.store, "log_tumble", side_effect=OSError("disk unavailable")):
            with self.assertRaises(OSError):
                self.tracker.tumble_finish()
        self.assertIn("bad field", self.tracker.message)
        self.assertIn("Couldn't save the failed attempt", self.tracker.message)
        self.assertEqual(self.tracker.offset, (7, 8, 9))
