import csv
import math
import random
import tempfile
import threading
import time
import unittest
from pathlib import Path

from axes_fit import (determinant, earth_seen, fit_axes, nearest_rotation, plain_mappings,
                      turned_deg)
from calibration import CalibrationFile
from compare import compare, noise_ut, report, still_stretches, verdict
from data_log import DataLog
from geometry import mat_vec, norm, quat_conj, quat_rotate
from mmc5983 import COUNTS_PER_UT, counts, field_ut
from mmc_loop import MmcReader
from pair import Pair
from tests.sim_board import EARTH, GYRO_BIAS, OFFSET, SimBoard
from tracker import Tracker

MMC_OFFSET = (31.0, -12.0, 44.0)


def about(axis, deg):
    """A rotation matrix about x, y or z."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    a, b = [(1, 2), (2, 0), (0, 1)][axis]
    m = [[1.0 if r == k else 0.0 for k in range(3)] for r in range(3)]
    m[a][a], m[a][b], m[b][a], m[b][b] = c, -s, s, c
    return m


def times(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)] for r in range(3)]


def transpose(m):
    return [list(row) for row in zip(*m)]


# chip -> board: a quarter turn, 2.5 degrees of solder tilt, and a mirrored Z
QUARTER_TILTED = times(about(0, 2.5), times(about(1, -1.5), about(2, 90)))
MIRRORED = times(QUARTER_TILTED, [[1, 0, 0], [0, 1, 0], [0, 0, -1]])


class FakeReader:
    """An MMC about 60 times a second, as ``MmcReader.take`` gives it."""
    connected, error = True, None

    def __init__(self, axes, noise=0.1):
        self.board = None
        self.axes = axes
        self.noise = noise
        self.offset = MMC_OFFSET
        self.calls = 0

    def take(self):
        self.calls += 1
        if self.calls % 8 or not self.connected:
            return None
        board = quat_rotate(quat_conj(self.board.q), EARTH)
        chip = mat_vec(transpose(self.axes), board)
        return (tuple(c + o + random.gauss(0, self.noise) for c, o in zip(chip, self.offset)),
                self.board.t - 0.003)


def pair_on_board(axes=QUARTER_TILTED, log=None):
    reader = FakeReader(axes)
    pair = Pair(Tracker(offset=OFFSET, bias=GYRO_BIAS),
                Tracker(bias=GYRO_BIAS, fit_mounting=True, block_samples=3), reader)
    board = SimBoard(pair, gyro_bias=GYRO_BIAS)
    reader.board = board
    return pair, board, reader


class Mounting(unittest.TestCase):
    def tumble(self, chip_to_board, n=1500, noise=0.1):
        board = SimBoard(Tracker(), gyro_bias=(0, 0, 0))
        points, quats = [], []
        random.seed(2)
        for i in range(n):
            u = i * .03
            for _ in range(10):
                board.step((90 * math.sin(.71 * u), 110 * math.sin(.53 * u + 1.3), 80 * math.sin(.37 * u)))
            b = quat_rotate(quat_conj(board.q), EARTH)
            points.append(tuple(v + random.gauss(0, noise)
                                for v in mat_vec(transpose(chip_to_board), b)))
            quats.append(board.q)
        return points, quats

    def test_there_are_48_plain_mappings_half_mirrored(self):
        maps = plain_mappings()
        self.assertEqual(len({str(m) for m in maps}), 48)
        self.assertEqual(sum(determinant(m) < 0 for m in maps), 24)

    def test_nearest_rotation_undoes_a_stretch(self):
        m = about(2, 33)
        self.assertLess(turned_deg(nearest_rotation([[2 * v for v in r] for r in m]), m), 1e-6)

    def test_finds_the_quarter_turn_and_the_tilt(self):
        points, quats = self.tumble(QUARTER_TILTED)
        fit = fit_axes(points, quats)
        self.assertLess(turned_deg(fit["axes"], QUARTER_TILTED), 0.3)
        self.assertAlmostEqual(fit["tilt_deg"], turned_deg(QUARTER_TILTED, about(2, 90)), delta=0.3)
        self.assertLess(fit["spread_ut"], 1.0)

    def test_a_mirrored_chip_is_found_as_its_backwards_twin(self):
        points, quats = self.tumble(MIRRORED)
        fit = fit_axes(points, quats)
        backwards = [[-v for v in row] for row in MIRRORED]
        self.assertGreater(determinant(fit["axes"]), 0)
        self.assertLess(turned_deg(fit["axes"], backwards), 0.3)
        self.assertLess(fit["spread_ut"], 1.0)

    def test_backwards_and_forwards_score_the_same(self):
        points, quats = self.tumble(QUARTER_TILTED)
        backwards = [[-v for v in row] for row in QUARTER_TILTED]
        self.assertAlmostEqual(norm(earth_seen(QUARTER_TILTED, points, quats)),
                               norm(earth_seen(backwards, points, quats)))

    def test_a_square_sensor_shows_almost_no_tilt(self):
        points, quats = self.tumble(about(2, 0))
        self.assertLess(fit_axes(points, quats)["tilt_deg"], 0.3)


class SideBySide(unittest.TestCase):
    def setUp(self):
        random.seed(5)

    def calibrate(self, axes=QUARTER_TILTED):
        pair, board, reader = pair_on_board(axes)
        board.still(.5)
        pair.tumble_start()
        board.tumble(40)
        self.assertTrue(pair.status()["finish_ready"])
        pair.tumble_finish()
        board.still(1.0)
        return pair, board, reader

    def test_both_track_earth_while_turning_at_any_mounting(self):
        for axes in (QUARTER_TILTED, MIRRORED):
            pair, board, _ = self.calibrate(axes)
            board.samples.clear()
            board.turn((1, 0, 0), 120)
            board.turn((0, 1, 0), 120)
            board.still(.2)
            done = [s for s in board.samples if s["mmc"] and s["mmc"]["error_deg"] is not None]
            self.assertGreater(len(done), 30)
            self.assertLess(max(s["mmc"]["error_deg"] for s in done), 3.0)
            self.assertLess(max(s["error_deg"] for s in done), 6.0)

    def test_both_report_in_the_same_shape(self):
        pair, board, _ = self.calibrate()
        board.samples.clear()
        board.turn((1, 0, 0), 60)
        status = pair.status()
        self.assertEqual(set(status["mmc"]) - set(status), {"connected", "error"})   # server adds these
        self.assertGreater(status["rate_hz"], 90)
        self.assertGreater(status["mmc"]["rate_hz"], 50)
        icm, mmc = board.samples[-1], board.samples[-1]["mmc"]
        self.assertEqual(set(mmc), set(icm) - {"mmc"})

    def test_mmc_fit_survives_a_restart(self):
        with tempfile.TemporaryDirectory() as d:
            store = CalibrationFile(Path(d) / "c.json", Path(d) / "t.json", Path(d) / "m.json")
            self.assertIsNone(store.load_axes())
            store.save_axes(QUARTER_TILTED)
            store.clear()                          # Reset calibration keeps the mounting
            self.assertEqual(store.load_axes(), QUARTER_TILTED)

    def test_only_the_mmc_gets_a_mounting_fit(self):
        pair, _, _ = self.calibrate()
        self.assertEqual(pair.icm.axes, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
        self.assertLess(turned_deg(pair.mmc.axes, QUARTER_TILTED), 1.0)
        self.assertIn("mounting", pair.icm.status()["tumble_result"])

    def test_finish_waits_for_the_mmc(self):
        pair, board, reader = pair_on_board()
        reader.take = lambda: None                 # the MMC has seen nothing
        board.still(.5)
        pair.tumble_start()
        board.tumble(30)
        self.assertTrue(pair.icm.status()["tumble_ready"])
        self.assertFalse(pair.status()["finish_ready"])
        self.assertTrue(pair.status()["tumble_ready"])
        with self.assertRaises(ValueError):
            pair.tumble_finish()

    def test_a_lost_mmc_does_not_hold_up_the_icm(self):
        pair, board, reader = pair_on_board()
        board.still(.5)
        pair.tumble_start()
        board.tumble(30)
        reader.connected = False
        self.assertTrue(pair.status()["finish_ready"])
        pair.tumble_finish()
        self.assertTrue(pair.icm.status()["have_offset"])
        self.assertFalse(pair.mmc.status()["tumbling"])
        self.assertFalse(pair.mmc.status()["have_offset"])

    def test_no_mmc_at_all(self):
        pair = Pair(Tracker(offset=OFFSET, bias=GYRO_BIAS))
        board = SimBoard(pair, gyro_bias=GYRO_BIAS)
        board.still(1.0)
        self.assertIsNone(pair.status()["mmc"])
        self.assertIsNone(board.samples[-1]["mmc"])

    def test_each_sensors_calibration_switches_alone(self):
        pair, _, _ = pair_on_board()
        pair.toggle_calibration("mmc")
        self.assertTrue(pair.status()["calibration_on"])
        self.assertFalse(pair.status()["mmc"]["calibration_on"])
        pair.toggle_calibration("icm")
        self.assertFalse(pair.status()["calibration_on"])
        pair.toggle_calibration("mmc")
        self.assertTrue(pair.status()["mmc"]["calibration_on"])

    def test_with_calibration_off_the_mmc_still_uses_its_mounting(self):
        pair, board, reader = pair_on_board()
        self.assertFalse(pair.status()["mmc"]["mounted"])
        reader.offset = (0.0, 0.0, 0.0)            # SET/RESET left nothing behind
        board.still(.5)
        pair.tumble_start()
        board.tumble(40)
        pair.tumble_finish()
        self.assertTrue(pair.status()["mmc"]["mounted"])
        pair.toggle_calibration("mmc")
        board.still(1.0)
        board.samples.clear()
        board.turn((1, 0, 0), 120)
        board.turn((0, 1, 0), 120)
        board.still(.2)
        done = [s["mmc"] for s in board.samples if s["mmc"] and s["mmc"]["error_deg"] is not None]
        self.assertGreater(len(done), 30)
        self.assertLess(max(s["error_deg"] for s in done), 3.0)
        self.assertFalse(done[-1]["calibration_on"])

    def test_a_missing_mmc_cant_be_switched(self):
        with self.assertRaises(ValueError):
            Pair(Tracker()).toggle_calibration("mmc")

    def test_reset_clears_both(self):
        pair, _, _ = self.calibrate()
        pair.reset_calibration()
        self.assertFalse(pair.status()["have_offset"])
        self.assertFalse(pair.mmc.status()["have_offset"])
        self.assertTrue(pair.status()["mmc"]["mounted"])
        self.assertLess(turned_deg(pair.mmc.axes, QUARTER_TILTED), 1.0)

    def test_with_the_mounting_known_the_mmc_cancels_earth_with_no_tumble(self):
        reader = FakeReader(QUARTER_TILTED)
        reader.offset = (0.0, 0.0, 0.0)            # SET/RESET left nothing behind
        pair = Pair(Tracker(offset=OFFSET, bias=GYRO_BIAS),
                    Tracker(bias=GYRO_BIAS, axes=QUARTER_TILTED, fit_mounting=True,
                            block_samples=3, use_calibration=False), reader)
        board = SimBoard(pair, gyro_bias=GYRO_BIAS)
        reader.board = board
        board.still(1.0)                           # started up, nobody clicked anything
        self.assertIsNotNone(pair.mmc.earth)
        self.assertFalse(pair.mmc.status()["have_offset"])
        board.samples.clear()
        board.turn((1, 0, 0), 150)
        board.turn((0, 1, 0), 150)
        board.still(.2)
        done = [s["mmc"] for s in board.samples if s["mmc"] and s["mmc"]["error_deg"] is not None]
        self.assertGreater(len(done), 30)
        self.assertLess(max(s["error_deg"] for s in done), 3.0)
        self.assertLess(max(s["error_ut"] for s in done), 3.0)

    def test_after_the_first_tumble_finish_needs_only_the_icm(self):
        pair, board, reader = pair_on_board(QUARTER_TILTED)
        pair.mmc.mounted = True
        pair.mmc.axes = QUARTER_TILTED
        reader.take = lambda: None                 # the MMC has seen nothing
        board.still(.5)
        pair.tumble_start()
        board.tumble(30)
        self.assertTrue(pair.status()["finish_ready"])
        pair.tumble_finish()
        self.assertFalse(pair.mmc.status()["tumbling"])
        self.assertEqual(pair.mmc.axes, QUARTER_TILTED)


class Driver(unittest.TestCase):
    def test_18_bit_counts(self):
        self.assertEqual(counts([0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00]), (131072,) * 3)
        self.assertEqual(counts([0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0b11011000]),
                         ((0x1234 << 2) | 3, (0x5678 << 2) | 1, (0x9ABC << 2) | 2))

    def test_the_pair_keeps_the_field_and_drops_the_drift(self):
        middle = 131072
        drift, field = 900, 8192                   # 8192 counts is half a gauss, 50 uT
        out = field_ut((middle + drift + field,) * 3, (middle + drift - field,) * 3)
        self.assertEqual(out, (field / COUNTS_PER_UT,) * 3)
        self.assertAlmostEqual(out[0], 50.0)


class Reader(unittest.TestCase):
    def test_reconnects_and_keeps_only_the_newest(self):
        stop = threading.Event()

        class Fake:
            opened = 0

            def __init__(self, bus, address):
                type(self).opened += 1
                if type(self).opened == 1:
                    raise OSError("not there")
                self.n = 0

            def read(self):
                self.n += 1
                if self.n == 50:
                    stop.set()
                time.sleep(0.001)
                return (float(self.n),) * 3, time.monotonic()

            def close(self):
                pass

        reader = MmcReader(1, 0x30, Fake)
        real_wait, stop.wait = stop.wait, lambda s: real_wait(0.01)
        reader.run(stop)
        self.assertEqual(Fake.opened, 2)
        self.assertTrue(reader.connected)
        self.assertEqual(reader.take()[0], (50.0,) * 3)
        self.assertIsNone(reader.take())


class Scoring(unittest.TestCase):
    def setUp(self):
        random.seed(6)

    def log_run(self, folder):
        """A real pipeline run: tumble, then log turning and resting."""
        pair, board, _ = SideBySide.calibrate(self)
        log = DataLog(folder)
        real = pair.update

        def update(accel, gyro, mag, now):
            sample = real(accel, gyro, mag, now)
            log.add(now, accel, gyro, mag, sample, pair.reading)
            return sample
        board.tracker = type("Wired", (), {"update": staticmethod(update)})()
        name = log.start(pair.status())
        board.turn((1, 0, 0), 150)
        board.still(2.0)
        board.turn((0, 1, 0), 120)
        board.still(2.0)
        log.stop()
        return Path(folder) / name

    def test_scores_a_real_run(self):
        with tempfile.TemporaryDirectory() as d:
            folder = self.log_run(d)
            r = compare(folder)
            text = report(r)
            with open(folder / "samples.csv", newline="") as f:
                header = next(csv.reader(f))
            self.assertIn("plot.html", [p.name for p in folder.iterdir()])
            self.assertIn("MMC direction error", (folder / "plot.html").read_text())
        self.assertIn("mmc_error_deg", header)
        self.assertGreater(r["turned_samples"], 30)
        self.assertGreater(r["still_s"], 2.0)
        # the simulated AK has 0.4 uT of noise, the simulated MMC 0.1 uT
        self.assertLess(r["mmc"]["noise"], r["icm"]["noise"])
        self.assertGreater(r["icm"]["rate"], 90)
        self.assertGreater(r["mmc"]["rate"], 50)
        self.assertLess(r["mmc"]["direction"][0], 2.0)
        self.assertIn("Direction error, typical", text)
        self.assertIn("Software calibration: ICM on, MMC on", text)

    def test_a_log_without_the_mmc_is_refused_plainly(self):
        with tempfile.TemporaryDirectory() as d:
            log = DataLog(d)
            name = log.start()
            log.stop()
            (Path(d) / name / "raw.csv").write_text("t_s,mx_ut\n0,1\n")
            with self.assertRaises(ValueError):
                compare(Path(d) / name)

    def test_noise_uses_equal_time_for_both(self):
        rows = [{"t_s": str(i * .001), "gx_dps": "0", "gy_dps": "0", "gz_dps": "0",
                 "a": str(random.gauss(0, 1) if i % 2 == 0 else "")} for i in range(4000)]
        for r in rows:
            r["a"] = r["a"] if r["a"] not in ("", "None") else ""
        still = still_stretches(rows)
        self.assertEqual(len(still), 1)
        self.assertAlmostEqual(noise_ut(still, ("a",)), 1 / math.sqrt(25), delta=0.03)

    def test_moving_is_not_still(self):
        rows = [{"t_s": str(i * .01), "gx_dps": "20", "gy_dps": "0", "gz_dps": "0"} for i in range(300)]
        self.assertEqual(still_stretches(rows), [])

    def test_verdicts(self):
        r = {"icm": {"direction": (2.0, 3, 4)}, "mmc": {"direction": (1.0, 2, 3)}}
        self.assertIn("MMC5983MA tracks", verdict(r))
        r["mmc"]["direction"] = (4.0, 5, 6)
        self.assertIn("ICM-20948 tracks", verdict(r))
        r["mmc"]["direction"] = (2.1, 5, 6)
        self.assertIn("Too close", verdict(r))
        r["mmc"]["direction"] = None
        self.assertIn("No verdict", verdict(r))
