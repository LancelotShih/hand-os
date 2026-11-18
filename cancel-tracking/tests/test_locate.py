"""Absolute field fitting and the complete coil-off/coil-on recording workflow."""
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
from magtrack_src.position.coil_model import Coil, mm_per_ut
from magtrack_src.position.coil_fit import fit_coil
from magtrack_src.position.position_solver import check_points, solve
from magtrack_src.position.locate import Locator
from magtrack_src.tracking.pair import Pair
from magtrack_src.tracking.tracker import Tracker
from tests.sim.sim_board import AMBIENT_FIELD, GYRO_BIAS, OFFSET, SimBoard

CENTER = (0.4, 0.3, -0.2)
END_ON = Coil(CENTER, (0, 23400, 0), 2.2)
BROADSIDE = Coil(CENTER, (23400, 0, 0), 2.2)
BASE = (-0.6, -12.7, 1.3)
SPOTS = [add(BASE, p) for p in ((0, 0, 5), (6, 0, 0), (-6, 0, 0), (0, 4, 0),
         (5, 5, 4), (-5, 6, 4), (0, -3, 3), (4, -2, 6), (-4, 2, -1), (3, 3, 2))]


def fields(coil, spots, noise=0.0, seed=1):
    rng = random.Random(seed)
    return [(p, tuple(v + rng.gauss(0, noise) for v in coil.field(p))) for p in spots]


def run_locator(locator):
    deadline = time.monotonic() + 10
    while locator.status()["fitting"]:
        if time.monotonic() > deadline:
            raise AssertionError("coil fit did not finish")
        time.sleep(0.02)
    for worker in threading.enumerate():
        if worker is not threading.current_thread() and worker.daemon:
            worker.join(2)


class Model(unittest.TestCase):
    def test_far_away_it_is_a_point_magnet(self):
        p = (40, 3, -2)
        d, r = sub(p, END_ON.center), norm(sub(p, END_ON.center))
        moment = tuple(v * 2 * END_ON.half_length for v in END_ON.s)
        point = tuple((3 * d[i] * sum(a*b for a, b in zip(moment, d)) / r**2 - moment[i]) / r**3 for i in range(3))
        self.assertLess(norm(sub(END_ON.field(p), point)), 0.01 * norm(point))

    def test_the_field_is_the_same_at_the_mirror_point(self):
        p = (3, -2, 4)
        self.assertLess(norm(sub(END_ON.field(p), END_ON.field(END_ON.mirror(p)))), 1e-9)

    def test_the_middle_plane_is_weakly_determined(self):
        self.assertLess(mm_per_ut(END_ON, BASE), 1)
        self.assertGreater(mm_per_ut(BROADSIDE, BASE), 10)


class Fit(unittest.TestCase):
    def test_finds_polarity_and_coil_from_ten_absolute_fields(self):
        data = fields(END_ON, SPOTS, noise=0.3)
        coil = fit_coil(data)
        self.assertLess(norm(sub(coil.center, END_ON.center)), 0.3)
        self.assertGreater(sum(a*b for a, b in zip(coil.axis, END_ON.axis)), 0.999)
        self.assertLess(max(check_points(coil, data)), 2)

    def test_the_center_stays_near_zero(self):
        far = Coil((7, 0, 0), END_ON.s, END_ON.half_length)
        coil = fit_coil(fields(far, SPOTS))
        self.assertLessEqual(norm(coil.center), LOCATE_CENTER_SLACK_CM + 1e-6)

    def test_refuses_too_few_points(self):
        with self.assertRaises(ValueError):
            fit_coil(fields(END_ON, SPOTS[:3]))

    def test_tracks_a_path_to_within_two_mm_using_absolute_fields(self):
        coil = fit_coil(fields(END_ON, SPOTS, noise=0.3))
        last, rng, worst = BASE, random.Random(2), 0
        for k in range(200):
            p = add(BASE, (5*math.sin(k/30), 3+3*math.cos(k/25), 2*math.sin(k/17)))
            field = tuple(v + rng.gauss(0, 0.3) for v in END_ON.field(p))
            last, miss = solve(coil, field, last)
            worst = max(worst, norm(sub(last, p)))
        self.assertLess(worst, 0.2)


class OnTheBoard(unittest.TestCase):
    def setUp(self):
        random.seed(3)
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / "coil.json"
        self.locator = Locator(self.path)
        self.pair = Pair(Tracker(OFFSET, GYRO_BIAS, require_coil_off=True), locator=self.locator)
        self.board = SimBoard(self.pair)
        self.pair.capture_ambient(True)
        self.board.still(1.2)
        self.board.coil = END_ON
        self.pair.coil_power(True)

    def tearDown(self):
        run_locator(self.locator)
        self.directory.cleanup()

    def capture(self, p):
        self.board.pos = p
        self.board.still(0.3)
        self.pair.capture_point(p)
        while self.locator.status()["capturing"]:
            self.board.still(0.1)
        run_locator(self.locator)

    def test_positions_after_calibration_and_restart_file(self):
        for p in SPOTS[:8]:
            self.capture(p)
        self.assertIsNotNone(self.locator.coil, self.locator.message)
        self.board.pos = (2, -9, 3)
        self.board.turn((0, 0, 1), 40)
        self.board.turn((1, 0, 0), 25)
        self.board.still(0.2)
        choices = self.locator.status()["candidates"]
        self.assertTrue(choices)
        chosen = min(choices, key=lambda p: norm(sub(p, self.board.pos)))
        self.locator.choose_position(chosen, self.board.samples[-1]["reference_id"])
        self.board.still(0.1)
        self.assertLess(norm(sub(self.board.samples[-1]["locate"]["position"], self.board.pos)), 0.4)
        saved = json.loads(self.path.read_text())
        self.assertEqual(saved["format_version"], 2)
        self.assertNotIn("home", saved)
        self.assertIn("coil_field_ut", saved["points"][0])
        again = Locator(self.path)
        self.assertEqual(again.status()["coil"], self.locator.status()["coil"])

    def test_point_records_absolute_coil_field_without_a_starting_position(self):
        self.capture(SPOTS[0])
        point = self.locator.points[0]
        self.assertLess(norm(sub(point.coil_field_ut, END_ON.field(SPOTS[0]))), 1)

    def test_clear_cancels_recording_and_discards_model(self):
        self.pair.capture_point(SPOTS[0])
        self.locator.clear_points()
        self.board.pos = SPOTS[0]
        self.board.still(0.6)
        self.assertEqual(self.locator.points, [])
        self.assertFalse(self.locator.status()["capturing"])

    def test_no_field_recording_before_coil_on_confirmation(self):
        self.pair.coil_power(False)
        with self.assertRaises(ValueError):
            self.pair.capture_point(SPOTS[0])
