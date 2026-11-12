import json
import math
import random
import tempfile
import unittest
from pathlib import Path

from magtrack_src.calibration.store import CalibrationFile
from magtrack_src.config import PASS_MIN_TURN_DEG, PASS_UT, TUMBLE_MAX_POINTS
from magtrack_src.geometry.vectors import norm, sub
from tests.sim.sim_board import EARTH, GYRO_BIAS, OFFSET, SimBoard
from magtrack_src.tracking.tracker import Tracker
from magtrack_src.calibration.tumble import SECTORS, Tumble, coverage, fit_ellipsoid, fit_tumble, spread_95

GAIN = (1.06, 0.97, 0.95)


def sphere_points(n, center=OFFSET, radii=(46.0, 46.0, 46.0), noise=0.0):
    points = []
    for _ in range(n):
        v = [random.gauss(0, 1) for _ in range(3)]
        m = math.sqrt(sum(c * c for c in v))
        points.append(tuple(center[i] + radii[i] * v[i] / m + random.gauss(0, noise) for i in range(3)))
    return points


class Fit(unittest.TestCase):
    def setUp(self):
        random.seed(6)

    def test_finds_centre_and_radii(self):
        center, radii = fit_ellipsoid(sphere_points(800, radii=(50.0, 44.0, 46.0)))
        self.assertLess(norm(sub(center, OFFSET)), 1e-6)
        self.assertLess(norm(sub(radii, (50.0, 44.0, 46.0))), 1e-6)

    def test_report_measures_the_asymmetry(self):
        r = fit_tumble(sphere_points(2000, radii=(50.0, 44.0, 46.0), noise=0.4))
        self.assertAlmostEqual(r["asymmetry_ut"], 6.0, delta=.2)
        self.assertAlmostEqual(r["field_ut"], 140 / 3, delta=.2)
        self.assertLess(r["spread_after_ut"], 1.0)
        self.assertGreater(r["spread_before_ut"], 2.5)
        self.assertTrue(r["passed"])
        json.dumps(r)                                   # goes straight to the page

    def test_fails_the_threshold_when_the_shape_is_rough(self):
        r = fit_tumble(sphere_points(2000, noise=6.0))
        self.assertGreater(r["spread_after_ut"], PASS_UT)
        self.assertFalse(r["passed"])

    def test_refuses_a_stretch_no_sensor_has(self):
        with self.assertRaises(ValueError):
            fit_tumble(sphere_points(2000, radii=(90.0, 30.0, 30.0)))

    def test_spread_of_a_perfect_sphere_is_zero(self):
        self.assertLess(spread_95(sphere_points(500), OFFSET, (1.0, 1.0, 1.0)), 1e-9)


class Coverage(unittest.TestCase):
    def test_one_pose_covers_almost_nothing(self):
        self.assertLessEqual(coverage([(50.0, 90.0, 10.0)] * 50), 2 / SECTORS)

    def test_a_flat_spin_is_not_enough(self):
        points = [(46 * math.cos(i * .1), 46 * math.sin(i * .1), 0.0) for i in range(100)]
        self.assertLess(coverage(points), .5)

    def test_every_direction_counts_as_full(self):
        random.seed(6)
        self.assertEqual(coverage(sphere_points(3000)), 1.0)

    def test_needs_both_readings_and_directions(self):
        t = Tumble()
        for p in sphere_points(100):
            t.add(p)
        self.assertFalse(t.ready)


class NotMoving(unittest.TestCase):
    """A still board must never pass: its noise looks like every direction."""
    def setUp(self):
        random.seed(8)

    @staticmethod
    def noisy_still_points(n=2000, wobble=0.0):
        lsb = 0.15
        return [tuple(round((c + random.gauss(0, .4) + random.gauss(0, wobble)) / lsb) * lsb
                      for c in (63.85, 85.26, 14.15)) for _ in range(n)]

    def test_noise_covers_nothing(self):
        self.assertEqual(coverage(self.noisy_still_points()), 0.0)

    def test_a_small_wobble_covers_nothing(self):
        self.assertEqual(coverage(self.noisy_still_points(wobble=3.0)), 0.0)

    def test_finish_stays_locked(self):
        t = Tumble()
        for p in self.noisy_still_points():
            t.add(p)
        self.assertFalse(t.ready)

    def test_the_fit_refuses_a_still_board(self):
        with self.assertRaises(ValueError):
            fit_tumble(self.noisy_still_points())

    def test_a_still_board_cannot_finish(self):
        tracker = Tracker(offset=OFFSET, bias=GYRO_BIAS)
        board = SimBoard(tracker)
        board.still(.5)
        tracker.tumble_start()
        board.still(30)
        status = tracker.status()
        self.assertEqual(status["tumble_coverage"], 0.0)
        self.assertFalse(status["tumble_ready"])
        with self.assertRaises(ValueError):
            tracker.tumble_finish()
        self.assertEqual(tracker.offset, OFFSET)        # nothing was overwritten


class Progress(unittest.TestCase):
    def test_it_never_goes_backwards(self):
        random.seed(9)
        t = Tumble()
        seen = []
        for p in sphere_points(1000, radii=(30.0, 30.0, 30.0)) + sphere_points(1000, center=(70.0, 90.0, 20.0)):
            t.add(p)
            seen.append(t.coverage)
        self.assertEqual(seen, sorted(seen))
        self.assertGreater(seen[-1], 0.5)


class DirectionMap(unittest.TestCase):
    def setUp(self):
        random.seed(11)

    def test_a_still_board_sweeps_nothing(self):
        t = Tumble()
        for p in NotMoving.noisy_still_points():
            t.add(p)
        self.assertEqual(t.swept, set())
        self.assertIsNone(t.now)
        self.assertLess(max(t.swing), 3)

    def test_every_direction_can_be_swept(self):
        t = Tumble()
        for p in sphere_points(3000):
            t.add(p)
        self.assertEqual(len(t.swept), SECTORS)
        self.assertNotIn((0, 0, 0), t.swept)
        self.assertAlmostEqual(norm(t.now), 1.0)

    def test_a_partial_turn_sweeps_nothing(self):
        t = Tumble()
        for p in sphere_points(3000, radii=(46.0, 46.0, 25.0)):      # Z never swings fully
            t.add(p)
        self.assertEqual(t.swept, set())
        self.assertFalse(t.counting)
        self.assertFalse(t.ready)

    def test_a_real_sensors_uneven_swings_still_count(self):
        t = Tumble()
        for p in sphere_points(3000, radii=(61.9, 64.6, 65.6)):      # the real board's numbers
            t.add(p)
        self.assertTrue(t.ready)

    def test_movement_shows_before_directions_count(self):
        t = Tumble()
        for p in sphere_points(1000, radii=(46.0, 46.0, 25.0)):
            t.add(p)
        self.assertFalse(t.counting)
        self.assertIsNotNone(t.now)

    def test_the_percent_is_exactly_the_dots(self):
        t = Tumble()
        for p in sphere_points(200):
            t.add(p)
            self.assertEqual(t.coverage, len(t.swept) / SECTORS)

    def test_swept_directions_stay_swept(self):
        t = Tumble()
        before = set()
        for p in sphere_points(1000, radii=(30.0, 30.0, 30.0)) + sphere_points(1000, center=(70.0, 90.0, 20.0)):
            t.add(p)
            self.assertTrue(before <= t.swept)
            before = set(t.swept)

    def test_the_page_gets_the_map(self):
        tracker = Tracker(bias=GYRO_BIAS)
        board = SimBoard(tracker)
        board.still(.5)
        tracker.tumble_start()
        board.tumble(10)
        status = tracker.status()
        json.dumps(status)
        self.assertGreater(len(status["tumble_swept"]), 3)
        self.assertEqual(len(status["tumble_now"]), 3)
        self.assertEqual(len(status["tumble_swing"]), 3)
        tracker.tumble_cancel()
        status = tracker.status()
        self.assertEqual((status["tumble_swept"], status["tumble_now"]), ([], None))


class LongTumble(unittest.TestCase):
    def test_a_long_still_start_cannot_block_it(self):
        random.seed(10)
        t = Tumble()
        for _ in range(TUMBLE_MAX_POINTS + 50):
            t.add(OFFSET)                           # sat still past the limit
        self.assertEqual(t.coverage, 0.0)
        for p in sphere_points(TUMBLE_MAX_POINTS * 2):
            t.add(p)                                # then a real tumble
        self.assertLessEqual(len(t.points), TUMBLE_MAX_POINTS)
        self.assertTrue(t.ready)


class WholeFlow(unittest.TestCase):
    def setUp(self):
        random.seed(7)

    def run_tumble(self, tracker, seconds=40):
        board = SimBoard(tracker, mag_gain=GAIN)
        board.still(.5)
        tracker.tumble_start()
        board.tumble(seconds)
        self.assertTrue(tracker.status()["tumble_ready"], tracker.status()["tumble_coverage"])
        tracker.tumble_finish()
        return board

    def test_tumble_then_turn_stays_within_the_limit(self):
        tracker = Tracker(bias=GYRO_BIAS)
        board = self.run_tumble(tracker)
        board.still(.5)
        r = tracker.status()["tumble_result"]
        self.assertTrue(r["passed"])
        self.assertLess(norm(sub(r["center"], OFFSET)), 1)
        expected = max(GAIN) * norm(EARTH) - min(GAIN) * norm(EARTH)
        self.assertLess(r["asymmetry_ut"], expected * 1.5)
        self.assertGreater(r["asymmetry_ut"], 1)
        board.samples.clear()
        for axis, deg in (((1, 0, 0), 170), ((0, 1, 0), 140), ((0, 0, 1), 200)):
            board.turn(axis, deg, rate=45)
            board.still(.5)
        self.assertLess(max(s["error_ut"] for s in board.samples), PASS_UT)
        self.assertGreater(max(s["turn"] for s in board.samples), PASS_MIN_TURN_DEG)

    def test_saved_and_reloaded(self):
        with tempfile.TemporaryDirectory() as d:
            store = CalibrationFile(Path(d) / "cal.json", Path(d) / "t.json")
            tracker = Tracker(bias=GYRO_BIAS, store=store)
            self.run_tumble(tracker)
            offset, bias, gains = store.load()
            self.assertEqual(offset, tracker.offset)
            self.assertEqual(gains, tracker.gains)
            log = json.loads(store.tumble_log_path.read_text())
            self.assertGreater(len(log["points"]), 500)
            self.assertTrue(log["result"]["passed"])

    def test_a_new_tumble_ignores_the_previous_one(self):
        tracker = Tracker(bias=GYRO_BIAS)
        board = self.run_tumble(tracker)
        board.mag_gain = (0.97, 1.0, 1.03)
        tracker.tumble_start()
        board.tumble(40)
        tracker.tumble_finish()
        radii = tracker.status()["tumble_result"]["radii"]
        self.assertLess(radii[0], radii[1])
        self.assertLess(radii[1], radii[2])

    def test_reset_starts_from_nothing_and_keeps_logs(self):
        with tempfile.TemporaryDirectory() as d:
            store = CalibrationFile(Path(d) / "cal.json", Path(d) / "t.json")
            recording = Path(d) / "logs" / "2026-10-30_23-41-20"
            recording.mkdir(parents=True)
            tracker = Tracker(bias=GYRO_BIAS, store=store)
            board = self.run_tumble(tracker)
            board.still(.5)
            tracker.reset_calibration()
            status = tracker.status()
            self.assertFalse(status["have_offset"])
            self.assertEqual(status["gains"], [1.0, 1.0, 1.0])
            self.assertEqual(status["bias"], [0.0, 0.0, 0.0])
            self.assertIsNone(status["tumble_result"])
            self.assertIsNone(tracker.earth)
            self.assertEqual(store.load(), (None, (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)))
            self.assertFalse(store.tumble_log_path.exists())
            self.assertTrue(recording.exists())
            board.still(.5)                                 # a later capture still works
            self.assertIsNone(status["capturing"] and tracker.status()["capturing"])
            self.assertIsNone(tracker.earth)                # nothing to cancel against yet

    def test_reset_stops_a_running_tumble(self):
        tracker = Tracker()
        tracker.tumble_start()
        tracker.reset_calibration()
        self.assertFalse(tracker.status()["tumbling"])

    def test_reset_with_nothing_saved_is_fine(self):
        with tempfile.TemporaryDirectory() as d:
            store = CalibrationFile(Path(d) / "c.json", Path(d) / "t.json")
            tracker = Tracker(store=store)
            tracker.reset_calibration()
            self.assertIn("cleared", tracker.message)

    def test_calibration_cant_be_switched_during_a_tumble(self):
        tracker = Tracker()
        tracker.tumble_start()
        with self.assertRaises(ValueError):
            tracker.set_calibration(False)

    def test_cancelling_changes_nothing(self):
        tracker = Tracker(offset=OFFSET, bias=GYRO_BIAS)
        board = SimBoard(tracker)
        board.still(.5)
        tracker.tumble_start()
        board.tumble(3)
        tracker.tumble_cancel()
        self.assertEqual(tracker.offset, OFFSET)
        self.assertEqual(tracker.gains, (1.0, 1.0, 1.0))
