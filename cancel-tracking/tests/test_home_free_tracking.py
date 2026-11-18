"""Coil-off startup, saved hardware axes, absolute lookup and explicit ambiguity."""
import math
import random
import unittest

from magtrack_src.geometry.vectors import norm, quat_angle_deg, quat_conj, quat_mul, quat_rotate, sub
from magtrack_src.orientation.map_frame import align_vectors
from magtrack_src.position.coil_model import Coil
from magtrack_src.position.field_lookup import FieldLookup
from magtrack_src.position.locate import Locator
from magtrack_src.tracking.pair import Pair
from magtrack_src.tracking.tracker import Tracker
from tests.sim.sim_board import AMBIENT_FIELD, GYRO_BIAS, OFFSET, SimBoard, axis_quat

FRAME = {"id": "map", "gravity": [0, 0, 1], "ambient_field": [17, -4, -43], "correction_id": "corrections"}
MODEL = Coil((0, 0, 0), (0, 23400, 0), 2.2)


def reading(field, reference="ambient-1", **changes):
    sample = {"ambient_ready": True, "coil_on": True, "frame_id": FRAME["id"],
              "correction_id": FRAME["correction_id"], "reference_id": reference,
              "coil_field": field, "reconnect_warning": False}
    sample.update(changes)
    return sample


class StartingOrientation(unittest.TestCase):
    def test_live_orientation_is_available_before_reference_capture(self):
        tracker = Tracker(OFFSET, GYRO_BIAS, require_coil_off=True)
        board = SimBoard(tracker)
        board.still(0.2)
        self.assertFalse(board.samples)

        before = tracker.status()["orientation_quat"]
        board.turn((0, 0, 1), 90, rate=60)
        after = tracker.status()["orientation_quat"]

        self.assertIsNotNone(before)
        self.assertIsNotNone(after)
        change = quat_mul(quat_conj(before), after)
        self.assertAlmostEqual(quat_angle_deg(change), 90, delta=5)

        tracker.capture_reference_field(coil_off_confirmed=True)
        board.still(0.5)
        sample_quat = board.samples[-1]["quat"]
        status_quat = tracker.status()["orientation_quat"]
        self.assertLess(norm(sub(sample_quat, status_quat)), 1e-4)

    def test_independently_specified_quarter_turn(self):
        # Current board x points along saved y; current y points along saved -x.
        q = align_vectors((0, 0, 1), (-4, -17, -43), (0, 0, 1), (17, -4, -43))
        self.assertLess(norm(sub(quat_rotate(q, (1, 0, 0)), (0, 1, 0))), 1e-9)
        self.assertLess(norm(sub(quat_rotate(q, (0, 1, 0)), (-1, 0, 0))), 1e-9)

    def test_arbitrary_rotations_recover_the_saved_hardware_axes(self):
        for axis, angle in (((1, 0, 0), 180), ((0, 0, 1), 180), ((1, 2, -3), 123), ((-1, 2, 1), -75)):
            q = axis_quat(axis, angle)
            g = quat_rotate(quat_conj(q), (0, 0, 1))
            b = quat_rotate(quat_conj(q), AMBIENT_FIELD)
            recovered = align_vectors(g, b, (0, 0, 1), AMBIENT_FIELD)
            for basis in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
                self.assertLess(norm(sub(quat_rotate(recovered, basis), quat_rotate(q, basis))), 1e-8)

    def test_parallel_or_zero_fields_do_not_invent_an_orientation(self):
        for field in ((0, 0, 40), (0, 0, 0), (0.1, 0, 40)):
            with self.assertRaises(ValueError):
                align_vectors((0, 0, 1), field, (0, 0, 1), (17, -4, -43))


class AbsoluteLookup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.table = FieldLookup(MODEL)

    def locator(self):
        loc = Locator()
        loc.frame, loc.coil, loc._lookup = dict(FRAME), MODEL, self.table
        return loc

    def test_startup_requires_selection_and_history_is_warned(self):
        loc, p = self.locator(), (4, -11, 3)
        sample = reading(MODEL.field(p))
        first = loc.update(sample)
        self.assertIsNone(first["position"])
        self.assertGreaterEqual(len(first["candidates"]), 2)
        self.assertIn("mirrored positions", first["why"])
        self.assertIsNone(loc._last)
        loc.choose_position(p, sample["reference_id"])
        result = loc.update(sample)
        self.assertLess(norm(sub(result["position"], p)), 0.01)
        self.assertTrue(result["history_based"])
        self.assertIn("AMBIGUOUS", result["warning"])

    def test_no_match_does_not_publish_a_closest_position(self):
        loc = self.locator()
        loc._last = (4, -11, 3)
        loc._reference_id = "ambient-1"
        result = loc.update(reading((0, 0, 0)))
        self.assertIsNone(result["position"])
        self.assertIn("No position matches", result["why"])
        self.assertEqual(loc._last, (4, -11, 3))

    def test_new_capture_with_identical_values_still_requires_a_new_choice(self):
        loc, p = self.locator(), (4, -11, 3)
        sample = reading(MODEL.field(p))
        loc.update(sample)
        loc.choose_position(p, "ambient-1")
        self.assertIsNotNone(loc.update(sample)["position"])
        result = loc.update(reading(MODEL.field(p), reference="ambient-2"))
        self.assertIsNone(result["position"])
        with self.assertRaises(ValueError):
            loc.choose_position(p, "ambient-1")

    def test_fifteen_cm_is_not_a_tracking_cutoff(self):
        p = (10, -26, 5)
        matches = self.table.matches(MODEL.field(p), 0.01)
        self.assertTrue(any(norm(sub(p, match[0])) < 0.05 for match in matches))

    def test_middle_plane_exposes_more_than_two_matching_positions(self):
        matches = self.table.matches(MODEL.field((8, 0, 0)), 0.01)
        self.assertGreaterEqual(len(matches), 8)
        for p, miss in matches:
            self.assertLessEqual(miss, 0.01)

    def test_mismatched_sensor_corrections_are_blocked(self):
        result = self.locator().update(reading(MODEL.field((4, -11, 3)), correction_id="new-corrections"))
        self.assertIsNone(result["position"])
        self.assertIn("corrections changed", result["why"])

    def test_poles_have_opposite_field_polarity(self):
        loc = self.locator()
        poles = loc.status()["coil"]
        self.assertEqual(poles["north_pole"], [0, 2.2, 0])
        self.assertEqual(poles["south_pole"], [0, -2.2, 0])
        # Just beyond north the field leaves the pole; near south it enters it.
        self.assertGreater(MODEL.field((0, 3.2, 0))[1], 0)
        self.assertGreater(MODEL.field((0, -3.2, 0))[1], 0)


class CoilOffWorkflow(unittest.TestCase):
    def setUp(self):
        random.seed(31)

    def test_no_automatic_coil_on_reference_and_confirmation_is_required(self):
        tracker = Tracker(OFFSET, GYRO_BIAS, require_coil_off=True)
        board = SimBoard(tracker)
        board.still(0.5)
        self.assertIsNone(tracker.reference_field)
        self.assertFalse(tracker.status()["ambient_ready"])
        with self.assertRaises(ValueError):
            tracker.capture_reference_field()
        with self.assertRaises(ValueError):
            tracker.set_coil_power(True)

    def test_restart_at_arbitrary_pose_recovers_axes_and_absolute_coil_field(self):
        first = Tracker(OFFSET, GYRO_BIAS, require_coil_off=True)
        board = SimBoard(first)
        first.capture_reference_field(coil_off_confirmed=True)
        board.still(0.6)
        frame = first.map_frame()
        self.assertIsNotNone(frame)
        loc = Locator()
        loc.frame, loc.coil, loc._lookup = frame, MODEL, FieldLookup(MODEL)
        restarted = Tracker(OFFSET, GYRO_BIAS, require_coil_off=True, map_frame=frame)
        pair = Pair(restarted, locator=loc)
        moved = SimBoard(pair, q=axis_quat((1, 2, -1), 105))
        moved.pos = (5, -10, 3)  # Unknown startup location is never submitted.
        pair.capture_ambient(True)
        moved.still(0.6)
        moved.coil = MODEL
        pair.coil_power(True)
        moved.still(0.2)
        sample = moved.samples[-1]
        self.assertLess(norm(sub(sample["coil_field"], MODEL.field(moved.pos))), 1.5)
        choices = loc.status()["candidates"]
        self.assertTrue(choices)
        chosen = min(choices, key=lambda p: norm(sub(p, moved.pos)))
        loc.choose_position(chosen, sample["reference_id"])
        moved.still(0.1)
        self.assertLess(norm(sub(moved.samples[-1]["locate"]["position"], moved.pos)), 0.5)
        pair.sensor_lost()
        moved.still(0.1)
        self.assertTrue(moved.samples[-1]["locate"]["reconnect_warning"])
        pair.dismiss_reconnect_warning()
        self.assertFalse(pair.status()["reconnect_warning"])

    def test_changed_corrections_require_an_explicit_new_map(self):
        tracker = Tracker(OFFSET, GYRO_BIAS, require_coil_off=True)
        board = SimBoard(tracker)
        tracker.capture_reference_field(coil_off_confirmed=True)
        board.still(0.5)
        tracker.set_calibration(False)
        tracker.capture_reference_field(coil_off_confirmed=True)
        board.still(0.5)
        self.assertFalse(tracker.status()["ambient_ready"])
        self.assertIn("corrections changed", tracker.message)
        tracker.new_map()
        tracker.capture_reference_field(coil_off_confirmed=True)
        board.still(0.5)
        self.assertTrue(tracker.status()["ambient_ready"])
