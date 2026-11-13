import random
import unittest
from unittest.mock import patch

from magtrack_src.config import PASS_UT
from magtrack_src.geometry.vectors import add, norm, quat_conj, quat_rotate, sub
from tests.sim.sim_board import AMBIENT_FIELD, GYRO_BIAS, OFFSET, SimBoard, axis_quat
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
        expected = quat_rotate(quat_conj(board.q), AMBIENT_FIELD)
        self.assertLess(norm(sub(board.tracker.reference_field, expected)), 1)
        self.assertLess(self.worst_error(board), 2)

    def test_without_the_offset_it_fails(self):
        board = SimBoard(Tracker(offset=(0.0, 0.0, 0.0), bias=GYRO_BIAS))
        board.still(.5)
        self.assertGreater(self.worst_error(board), 100)

    def test_uniform_extra_background_is_cancelled_after_turning(self):
        ambient_field = add(AMBIENT_FIELD, (8.0, -3.0, 5.0))
        with patch("tests.sim.sim_board.AMBIENT_FIELD", ambient_field):
            tracker = Tracker(offset=OFFSET, bias=GYRO_BIAS)
            board = SimBoard(tracker)
            board.still(.5)
            self.assertLess(norm(sub(tracker.reference_field, ambient_field)), 1.0)
            self.assertLess(self.worst_error(board), 2.0)
            reference_field = [round(v, 2) for v in tracker.reference_field]
            # Existing clients/logs retain these historical output names.
            self.assertEqual(tracker.status()["earth"], reference_field)
            self.assertEqual(board.samples[-1]["predicted"], reference_field)
            tracker.capture_reference_field()
            self.assertEqual(tracker.status()["capturing"], "earth")

    def test_switching_calibration_off_shows_the_offset_and_back_again(self):
        tracker = Tracker(offset=OFFSET, bias=GYRO_BIAS)
        board = SimBoard(tracker)
        board.still(.5)
        self.assertLess(self.worst_error(board), 2)
        tracker.set_calibration(False)
        self.assertIsNone(tracker.reference_field)
        self.assertEqual(tracker.status()["capturing"], "earth")
        board.still(.5)
        self.assertFalse(tracker.status()["calibration_on"])
        self.assertEqual(tracker.offset, OFFSET)                # kept, just not applied
        self.assertGreater(self.worst_error(board), 100)
        self.assertFalse(board.samples[-1]["calibration_on"])
        tracker.set_calibration(True)
        board.still(.5)
        self.assertLess(self.worst_error(board), 2)

    def test_no_tumble_is_needed_to_capture_reference_with_calibration_off(self):
        tracker = Tracker(bias=GYRO_BIAS, use_calibration=False)
        SimBoard(tracker).still(.5)
        self.assertIsNotNone(tracker.reference_field)


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
    """The I2C link drops, the board may move with no readings, then it's back."""

    def setUp(self):
        random.seed(5)
        self.tracker = Tracker(offset=OFFSET, bias=GYRO_BIAS)
        self.board = SimBoard(self.tracker)
        self.board.still(1.0)                         # captured here: this is home
        self.reference_field = self.tracker.reference_field

    def gap(self, q):
        """Disconnected for 3 s, during which the board ends up at ``q``."""
        self.board.q = q
        self.board.t += 3.0
        self.tracker.sensor_lost()

    def first_error_after(self):
        self.board.samples.clear()
        self.board.still(0.1)
        return self.board.samples[0]["error_ut"]

    def test_keeps_the_capture(self):
        self.gap(self.board.q)
        status = self.tracker.status()
        self.assertIsNone(status["capturing"])
        self.assertEqual(self.tracker.reference_field, self.reference_field)
        self.assertTrue(status["have_offset"])
        self.assertIn("reconnected", self.tracker.message)
        self.assertLess(self.first_error_after(), PASS_UT)
        self.assertEqual(self.tracker.gaps, 0)

    def test_tilt_comes_fresh_from_the_accelerometer(self):
        self.gap(axis_quat((1, 0.3, 0), 35))           # tilted while disconnected
        self.assertLess(self.first_error_after(), PASS_UT)
        self.gap(axis_quat((0, 0, 1), 0))              # and put back flat
        self.assertLess(self.first_error_after(), PASS_UT)

    def test_a_turn_in_heading_needs_a_capture_at_home(self):
        self.gap(axis_quat((0, 0, 1), 40))             # turned flat on the table: can't be seen
        self.assertGreater(self.first_error_after(), PASS_UT)
        self.board.q = axis_quat((0, 0, 1), 0)          # back on home, squared up, and captured
        self.tracker.capture_reference_field()
        self.board.still(1.0)
        self.assertLess(self.board.samples[-1]["error_ut"], PASS_UT)

    def test_a_capture_cut_off_by_the_gap_starts_over(self):
        self.tracker.capture_reference_field()
        self.board.still(0.1)
        self.gap(self.board.q)
        self.assertEqual(self.tracker.status()["capturing"], "earth")
        self.board.still(1.0)
        self.assertIsNone(self.tracker.status()["capturing"])
