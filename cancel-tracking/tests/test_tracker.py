import random
import unittest

from magtrack_src.config import PASS_UT
from magtrack_src.geometry.vectors import norm, quat_conj, quat_rotate, sub
from tests.sim.sim_board import EARTH, GYRO_BIAS, OFFSET, SimBoard, axis_quat
from magtrack_src.tracking.tracker import Tracker

BIG_TURNS = (((1, 0, 0), 170), ((0, 1, 0), 140), ((0, 0, 1), 200), ((1, 1, 0), -160))


class Cancelling(unittest.TestCase):
    def setUp(self):
        random.seed(3)

    def worst_error(self, board, amp=0.0):
        board.samples.clear()
        for axis, deg in BIG_TURNS:
            board.turn(axis, deg, rate=45)
            board.shake(1.0, amp)
        return max(s["error_ut"] for s in board.samples)

    def test_hand_held(self):
        for amp in (5, 15, 30, 60):
            board = SimBoard(Tracker(offset=OFFSET, bias=GYRO_BIAS))
            board.shake(1.0, amp)
            self.assertLess(self.worst_error(board, amp), PASS_UT, f"+/-{amp} dps")

    def test_one_reading_with_a_saved_offset(self):
        board = SimBoard(Tracker(offset=OFFSET, bias=GYRO_BIAS), q=axis_quat((1, -1, 0), 50))
        board.still(.5)
        expected = quat_rotate(quat_conj(board.q), EARTH)
        self.assertLess(norm(sub(board.tracker.earth, expected)), 1)
        self.assertLess(self.worst_error(board), 2)

    def test_without_the_offset_it_fails(self):
        board = SimBoard(Tracker(offset=(0.0, 0.0, 0.0), bias=GYRO_BIAS))
        board.still(.5)
        self.assertGreater(self.worst_error(board), 100)

    def test_switching_calibration_off_shows_the_offset_and_back_again(self):
        tracker = Tracker(offset=OFFSET, bias=GYRO_BIAS)
        board = SimBoard(tracker)
        board.still(.5)
        self.assertLess(self.worst_error(board), 2)
        tracker.set_calibration(False)
        self.assertIsNone(tracker.earth)
        self.assertEqual(tracker.status()["capturing"], "earth")
        board.still(.5)
        self.assertFalse(tracker.status()["calibration_on"])
        self.assertEqual(tracker.offset, OFFSET)                # kept, just not applied
        self.assertGreater(self.worst_error(board), 100)
        self.assertFalse(board.samples[-1]["calibration_on"])
        tracker.set_calibration(True)
        board.still(.5)
        self.assertLess(self.worst_error(board), 2)

    def test_no_tumble_is_needed_to_get_an_earth_with_calibration_off(self):
        tracker = Tracker(bias=GYRO_BIAS, use_calibration=False)
        SimBoard(tracker).still(.5)
        self.assertIsNotNone(tracker.earth)


class Capturing(unittest.TestCase):
    def test_a_shake_does_not_delay_it(self):
        tracker = Tracker()
        board = SimBoard(tracker)
        board.shake(.4, 60)
        self.assertIsNone(tracker.status()["capturing"])

    def test_a_real_turn_restarts_it(self):
        tracker = Tracker(offset=(0.0, 0.0, 0.0))
        board = SimBoard(tracker, gyro_bias=(0, 0, 0))
        board.still(.05)
        board.turn((1, 0, 0), 80, rate=120)
        self.assertEqual(tracker.status()["capturing"], "earth")
        board.still(.4)
        self.assertIsNone(tracker.status()["capturing"])

class GyroBias(unittest.TestCase):
    def test_only_a_real_rest_counts(self):
        tracker = Tracker()
        board = SimBoard(tracker)
        board.shake(2, 15)
        self.assertEqual(tracker.bias, (0.0, 0.0, 0.0))
        board.still(1.1)
        self.assertLess(norm(sub(tracker.bias, GYRO_BIAS)), .03)
        bias = tracker.bias
        for _ in range(750):
            board.step((5.0, 0.0, 0.0))                 # a slow steady turn
        self.assertEqual(tracker.bias, bias)


class SensorLoss(unittest.TestCase):
    def test_captures_again_and_keeps_the_offset(self):
        tracker = Tracker(offset=OFFSET, bias=GYRO_BIAS)
        SimBoard(tracker).still(.5)
        tracker.sensor_lost()
        status = tracker.status()
        self.assertEqual(status["capturing"], "earth")
        self.assertIsNone(tracker.earth)
        self.assertTrue(status["have_offset"])
