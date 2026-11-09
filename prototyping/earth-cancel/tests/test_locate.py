import json
import math
import random
import tempfile
import time
import unittest
from pathlib import Path

from config import LOCATE_POINT_SAMPLES
from geometry import norm, sub
from locate import HOME, Coil, Locator, check_points, fit_coil, mm_per_ut, solve
from pair import Pair
from tests.sim_board import GYRO_BIAS, OFFSET, SimBoard, axis_quat
from tracker import Tracker

STRENGTH = 23400.0        # about 47 uT at home, like the real coil at 1 A
END_ON = Coil((1.0, 13.0, -1.5), (0.0, STRENGTH, 0.0), 2.2)      # one end points at home
BROADSIDE = Coil((1.0, 13.0, -1.5), (STRENGTH, 0.0, 0.0), 2.2)   # its side faces home
SPOTS = [(0, 0, 5), (6, 0, 0), (-6, 0, 0), (0, 4, 0), (5, 5, 4), (-5, 6, 4), (0, -3, 3),
         (4, -2, 6), (-4, 2, -1), (3, 3, 2)]


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
        coil = fit_coil(errors(END_ON, SPOTS, noise=0.3))
        self.assertLess(norm(sub(coil.center, END_ON.center)), 0.3)
        self.assertGreater(abs(sum(a * b for a, b in zip(coil.axis, END_ON.axis))), 0.999)
        self.assertLess(max(check_points(coil, errors(END_ON, SPOTS, noise=0.3))), 2.0)

    def test_refuses_too_few_points(self):
        with self.assertRaises(ValueError):
            fit_coil(errors(END_ON, SPOTS[:3]))

    def test_tracks_a_path_to_within_2_mm(self):
        """Inside the working range: 6-13 cm from the coil, within 40 degrees of its axis."""
        coil = fit_coil(errors(END_ON, SPOTS, noise=0.3))
        home, last, rng, worst = coil.field(HOME), HOME, random.Random(2), 0.0
        for k in range(200):
            p = (5 * math.sin(k / 30), 3 + 3 * math.cos(k / 25), 2 * math.sin(k / 17))
            e = tuple(v + rng.gauss(0, 0.3) for v in sub(END_ON.field(p), END_ON.field(HOME)))
            last, miss = solve(coil, e, last, home)
            worst = max(worst, norm(sub(last, p)))
        self.assertLess(worst, 0.2)


def run_locator(locator):
    """Waits for a fit running on its own thread."""
    for _ in range(300):
        if not locator.status()["fitting"]:
            return
        time.sleep(0.02)


class OnTheBoard(unittest.TestCase):
    """The whole chain, simulated: coil always on, Earth captured at home."""

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "coil.json"
        self.locator = Locator(self.path)
        self.pair = Pair(Tracker(OFFSET, GYRO_BIAS), locator=self.locator)
        self.board = SimBoard(self.pair, coil=END_ON)
        self.board.still(1.5)                         # gyro bias, then the home capture

    def tearDown(self):
        self.dir.cleanup()

    def capture(self, p):
        self.board.pos = p
        self.board.still(0.3)
        self.locator.capture_point(p)
        while self.locator.status()["capturing"]:
            self.board.still(0.1)
        run_locator(self.locator)

    def test_positions_after_calibrating(self):
        self.assertEqual(self.board.samples[-1]["locate"]["why"], "not calibrated")
        for p in SPOTS[:8]:
            self.capture(p)
        status = self.locator.status()
        self.assertIsNotNone(status["coil"], status["message"])
        self.assertTrue(all(pt["left_out_mm"] is not None and pt["left_out_mm"] < 3 for pt in status["points"]))

        self.board.pos = (2.0, -1.0, 3.0)             # somewhere new, and turned
        self.board.turn((0, 0, 1), 40)
        self.board.turn((1, 0, 0), 25)
        self.board.still(0.5)
        got = self.board.samples[-1]["locate"]["position"]
        self.assertLess(norm(sub(got, self.board.pos)), 0.3)

        saved = json.loads(self.path.read_text())     # kept for the next launch
        self.assertEqual(len(saved["points"]), 8)
        again = Locator(self.path)
        self.assertEqual(again.status()["coil"], status["coil"])

    def test_a_moving_reading_is_not_kept(self):
        self.board.pos = (5.0, 0.0, 0.0)
        self.locator.capture_point(self.board.pos)
        for _ in range(LOCATE_POINT_SAMPLES):
            self.board.pos = (self.board.pos[0] - 0.4, 0.0, 0.0)
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
        self.assertEqual(self.board.samples[-1]["locate"]["why"], "not calibrated")

    def test_capturing_at_home_again_restarts_the_track_at_home(self):
        for p in SPOTS[:6]:
            self.capture(p)
        self.board.pos = HOME
        self.board.q = axis_quat((0, 0, 1), 0.0)
        self.board.still(0.3)
        self.pair.capture_earth()
        self.board.still(1.0)
        got = self.board.samples[-1]["locate"]["position"]
        self.assertLess(norm(got), 0.2)


if __name__ == "__main__":
    unittest.main()
