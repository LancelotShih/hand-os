import json
import math
import random
import tempfile
import threading
import time
import unittest
from pathlib import Path

from magtrack_src.config import LOCATE_CENTER_SLACK_CM, LOCATE_POINT_SAMPLES
from magtrack_src.geometry.vectors import add, norm, sub
from magtrack_src.position.locate import Coil, Locator, check_points, fit_coil, mm_per_ut, solve
from magtrack_src.tracking.pair import Pair
from tests.sim.sim_board import AMBIENT_FIELD, GYRO_BIAS, OFFSET, SimBoard, axis_quat
from magtrack_src.tracking.tracker import Tracker

# Positions are cm from the coil's center as the user measured it.  The magnet
# itself sits a few mm off that, as a real one would.
STRENGTH = 23400.0        # about 47 uT at home, like the real coil at 1 A
HOME = (-0.6, -12.7, 1.3)                                         # the ICM on its tape mark
CENTER = (0.4, 0.3, -0.2)
END_ON = Coil(CENTER, (0.0, STRENGTH, 0.0), 2.2)      # one end points at home
BROADSIDE = Coil(CENTER, (STRENGTH, 0.0, 0.0), 2.2)   # its side faces home
FROM_HOME = [(0, 0, 5), (6, 0, 0), (-6, 0, 0), (0, 4, 0), (5, 5, 4), (-5, 6, 4), (0, -3, 3),
             (4, -2, 6), (-4, 2, -1), (3, 3, 2)]
SPOTS = [add(HOME, p) for p in FROM_HOME]


def errors(coil, spots, noise=0.0, seed=1):
    rng = random.Random(seed)
    home = coil.field(HOME)
    return [(p, tuple(v + rng.gauss(0, noise) for v in sub(coil.field(p), home))) for p in spots]


class Model(unittest.TestCase):
    def test_far_away_it_is_a_point_magnet(self):
        p = (40.0, 3.0, -2.0)
        d = sub(p, END_ON.center)
        r = norm(d)
        m = tuple(v * 2 * END_ON.half_length for v in END_ON.s)
        point = tuple((3 * d[i] * sum(a * b for a, b in zip(m, d)) / r ** 2 - m[i]) / r ** 3 for i in range(3))
        self.assertLess(norm(sub(END_ON.field(p), point)), 0.01 * norm(point))

    def test_the_field_is_the_same_at_the_mirror_point(self):
        p = (3.0, -2.0, 4.0)
        self.assertLess(norm(sub(END_ON.field(p), END_ON.field(END_ON.mirror(p)))), 1e-9)

    def test_broadside_home_cannot_be_pinned_down(self):
        self.assertLess(mm_per_ut(END_ON, HOME), 1.0)
        self.assertGreater(mm_per_ut(BROADSIDE, HOME), 10.0)     # and home is 1 cm off the middle


class Fit(unittest.TestCase):
    def test_finds_the_coil_from_ten_points(self):
        coil = fit_coil(errors(END_ON, SPOTS, noise=0.3), HOME)
        self.assertLess(norm(sub(coil.center, END_ON.center)), 0.3)
        self.assertGreater(abs(sum(a * b for a, b in zip(coil.axis, END_ON.axis))), 0.999)
        self.assertLess(max(check_points(coil, errors(END_ON, SPOTS, noise=0.3), HOME)), 2.0)

    def test_the_center_stays_near_zero(self):
        far = Coil((7.0, 0.0, 0.0), END_ON.s, END_ON.half_length)   # home measured 7 cm wrong
        coil = fit_coil(errors(far, SPOTS), HOME)
        self.assertLessEqual(norm(coil.center), LOCATE_CENTER_SLACK_CM + 1e-6)

    def test_refuses_too_few_points(self):
        with self.assertRaises(ValueError):
            fit_coil(errors(END_ON, SPOTS[:3]), HOME)

    def test_tracks_a_path_to_within_2_mm(self):
        """Inside the working range: 6-13 cm from the coil, within 40 degrees of its axis."""
        coil = fit_coil(errors(END_ON, SPOTS, noise=0.3), HOME)
        home, last, rng, worst = coil.field(HOME), HOME, random.Random(2), 0.0
        for k in range(200):
            p = add(HOME, (5 * math.sin(k / 30), 3 + 3 * math.cos(k / 25), 2 * math.sin(k / 17)))
            e = tuple(v + rng.gauss(0, 0.3) for v in sub(END_ON.field(p), END_ON.field(HOME)))
            last, miss = solve(coil, e, last, home)
            worst = max(worst, norm(sub(last, p)))
        self.assertLess(worst, 0.2)


def run_locator(locator):
    """Waits for a fit running on its own thread, and the save after it."""
    for _ in range(300):
        if not locator.status()["fitting"]:
            break
        time.sleep(0.02)
    for t in threading.enumerate():
        if t is not threading.current_thread() and t.daemon:
            t.join(2.0)


class OnTheBoard(unittest.TestCase):
    """The whole chain, simulated: coil always on, total field captured at home."""

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "coil.json"
        self.locator = Locator(self.path)
        self.pair = Pair(Tracker(OFFSET, GYRO_BIAS), locator=self.locator)
        self.locator.set_home(HOME)
        self.board = SimBoard(self.pair, coil=END_ON)
        self.board.pos = HOME
        self.board.still(1.5)                         # gyro bias, then the home capture

    def tearDown(self):
        run_locator(self.locator)  # finish background saves before removing their directory
        self.dir.cleanup()

    def capture(self, p):
        self.board.pos = p
        self.board.still(0.3)
        self.locator.capture_point(p)
        while self.locator.status()["capturing"]:
            self.board.still(0.1)
        run_locator(self.locator)

    def test_positions_after_calibrating(self):
        self.assertEqual(self.board.samples[-1]["locate"]["why"], "No coil model is available. Record fields at measured positions to calibrate the coil model.")
        for p in SPOTS[:8]:
            self.capture(p)
        status = self.locator.status()
        self.assertIsNotNone(status["coil"], status["message"])
        self.assertTrue(all(pt["left_out_mm"] is not None and pt["left_out_mm"] < 3 for pt in status["points"]))

        self.board.pos = add(HOME, (2.0, -1.0, 3.0))  # somewhere new, and turned
        self.board.turn((0, 0, 1), 40)
        self.board.turn((1, 0, 0), 25)
        self.board.still(0.5)
        got = self.board.samples[-1]["locate"]["position"]
        self.assertLess(norm(sub(got, self.board.pos)), 0.3)

        saved = json.loads(self.path.read_text())     # kept for the next launch
        self.assertEqual(len(saved["points"]), 8)
        again = Locator(self.path)
        self.assertEqual(again.status()["coil"], status["coil"])

        # the page rebuilds the coil from these to draw its field
        c = status["coil"]
        page_coil = Coil(c["center"], [v * c["strength"] for v in c["axis"]], c["half_length"])
        fitted = self.locator.coil
        self.assertLess(norm(sub(page_coil.field(HOME), fitted.field(HOME))), 0.5)
        self.assertLess(norm(sub(c["home_field"], fitted.field(HOME))), 0.01)
        self.assertEqual(status["home"], list(HOME))
        self.assertEqual(saved["home"], list(HOME))

    def test_coil_on_reference_includes_ambient_and_coil_at_home(self):
        expected_reference = add(AMBIENT_FIELD, END_ON.field(HOME))
        self.assertLess(norm(sub(self.pair.icm.reference_field, expected_reference)), 1.0)

    def test_a_moving_reading_is_not_kept(self):
        self.board.pos = add(HOME, (5.0, 0.0, 0.0))
        self.locator.capture_point(self.board.pos)
        for _ in range(LOCATE_POINT_SAMPLES):
            self.board.pos = add(self.board.pos, (-0.4, 0.0, 0.0))
            self.board.still(0.05)
        self.board.still(0.3)
        self.assertEqual(self.locator.status()["points"], [])
        self.assertIn("Hold the board still", self.locator.status()["message"])

    def test_points_can_be_removed_and_cleared(self):
        for p in SPOTS[:5]:
            self.capture(p)
        self.locator.remove_point(0)
        run_locator(self.locator)
        status = self.locator.status()
        self.assertEqual(len(status["points"]), 4)
        self.assertIsNotNone(status["coil"])
        self.locator.clear_points()
        self.assertIsNone(self.locator.status()["coil"])
        self.board.still(0.3)
        self.assertEqual(self.board.samples[-1]["locate"]["why"], "No coil model is available. Record fields at measured positions to calibrate the coil model.")

    def test_capturing_at_home_again_restarts_the_track_at_home(self):
        for p in SPOTS[:6]:
            self.capture(p)
        self.board.pos = HOME
        self.board.q = axis_quat((0, 0, 1), 0.0)
        self.board.still(0.3)
        self.pair.capture_reference_field()
        self.board.still(1.0)
        got = self.board.samples[-1]["locate"]["position"]
        self.assertLess(norm(sub(got, HOME)), 0.2)

    def test_home_measured_a_little_off_moves_the_coil_not_the_points(self):
        for p in SPOTS[:6]:
            self.capture(p)
        before = self.locator.status()
        self.locator.set_home(add(HOME, (1.0, 0.0, 0.0)))
        run_locator(self.locator)
        after = self.locator.status()
        self.assertEqual(after["points"][0]["position"], before["points"][0]["position"])
        self.assertGreater(after["coil"]["center_off_cm"], before["coil"]["center_off_cm"])


class Home(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "coil.json"

    def tearDown(self):
        for t in threading.enumerate():               # saves run on their own threads
            if t is not threading.current_thread() and t.daemon:
                t.join(2.0)
        self.dir.cleanup()

    def test_nothing_happens_until_home_is_set(self):
        locator = Locator(self.path)
        with self.assertRaises(ValueError):
            locator.capture_point((1, 2, 3))
        self.assertEqual(locator.update({"error": [0, 0, 0], "predicted": [1, 2, 3]})["why"], "Sensor-home coordinates are not saved. Enter and save them before tracking position.")
        with self.assertRaises(ValueError):
            locator.set_home((1, 0, 0))               # that's inside the coil
        locator.set_home(HOME)
        self.assertEqual(locator.status()["home"], list(HOME))

    def test_an_old_file_measured_from_home_is_moved_to_the_coil(self):
        old = Coil(sub(CENTER, HOME), END_ON.s, END_ON.half_length)     # as an old fit saw it
        pts = errors(END_ON, SPOTS[:8])
        self.path.write_text(json.dumps({
            "points": [{"position": list(p), "error": list(e), "spread_ut": 0.1} for p, e in
                       zip(FROM_HOME, (e for _, e in pts))],
            "coil": old.as_dict(), "checks": []}))
        locator = Locator(self.path)
        status = locator.status()
        self.assertTrue(status["points_from_home"])
        self.assertIsNone(status["coil"])
        suggested = status["suggested_home"]
        self.assertLess(norm(sub(suggested, sub(HOME, CENTER))), 0.1)

        locator.remove_point(7)                       # still the old form on disk
        self.assertNotIn("home", json.loads(self.path.read_text()))
        locator.set_home(HOME)
        run_locator(locator)
        status = locator.status()
        self.assertFalse(status["points_from_home"])
        self.assertLess(norm(sub(status["points"][0]["position"], SPOTS[0])), 0.01)
        self.assertLess(norm(sub(status["coil"]["center"], CENTER)), 0.1)
        self.assertEqual(json.loads(self.path.read_text())["home"], list(HOME))


if __name__ == "__main__":
    unittest.main()
