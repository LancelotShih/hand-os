import csv
import json
import random
import re
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from magtrack_src.config import PASS_UT
from magtrack_src.recording.data_log import RAW_COLUMNS, DataLog
from magtrack_src.recording.log_plot import SAMPLE_COLUMNS, nice_top, read_samples, render, summarize, thin, write_plot
from tests.sim.sim_board import GYRO_BIAS, OFFSET, SimBoard
from magtrack_src.tracking.tracker import Tracker


class LoggedBoard(SimBoard):
    """A simulated board with the data log wired in, as ``sensor_loop`` does."""
    def __init__(self, tracker, log):
        super().__init__(tracker)
        self.log = log
        real = tracker.update

        def update(accel, gyro, mag, now):
            sample = real(accel, gyro, mag, now)
            log.add(now, accel, gyro, mag, sample)
            return sample
        tracker = self.tracker = type("Wired", (), {"update": staticmethod(update)})()


class Logging(unittest.TestCase):
    def setUp(self):
        random.seed(4)
        self.dir = tempfile.TemporaryDirectory()
        self.root = Path(self.dir.name)
        self.log = DataLog(self.root / "logs")

    def tearDown(self):
        self.dir.cleanup()

    def record(self):
        tracker = Tracker(offset=OFFSET, bias=GYRO_BIAS)
        board = LoggedBoard(tracker, self.log)
        board.still(.5)
        name = self.log.start(tracker.status())
        board.turn((1, 0, 0), 120, rate=60)
        board.still(.5)
        self.log.stop()
        return self.root / "logs" / name, board

    def test_name_is_the_start_time(self):
        name = self.log.start()
        self.log.stop()
        self.assertRegex(name, r"^\d{4}-\d\d-\d\d_\d\d-\d\d-\d\d$")

    def test_names_never_repeat(self):
        names = set()
        for _ in range(5):
            names.add(self.log.start())
            self.log.stop()
        self.assertEqual(len(names), 5)

    def test_one_log_at_a_time(self):
        self.log.start()
        with self.assertRaises(ValueError):
            self.log.start()
        self.log.stop()
        with self.assertRaises(ValueError):
            self.log.stop()

    def test_a_log_has_raw_data_processed_data_and_a_visual(self):
        folder, board = self.record()
        self.assertEqual(sorted(p.name for p in folder.iterdir()),
                         ["info.json", "plot.html", "raw.csv", "samples.csv"])
        with open(folder / "raw.csv", newline="") as f:
            raw = list(csv.DictReader(f))
        self.assertEqual(list(raw[0]), RAW_COLUMNS)
        self.assertGreater(len(raw), 1000)                  # every reading, 500 Hz here
        self.assertTrue(any(r["mx_ut"] for r in raw))
        self.assertTrue(any(not r["mx_ut"] for r in raw))   # no fresh magnetometer sample
        self.assertAlmostEqual(float(raw[0]["t_s"]), 0.0)
        c = read_samples(folder)
        self.assertEqual(c["t_s"], sorted(c["t_s"]))
        self.assertGreater(len(c["t_s"]), 30)
        self.assertLess(max(c["error_ut"]), PASS_UT)
        self.assertGreater(max(c["turn_deg"]), 100)
        self.assertEqual(json.loads((folder / "info.json").read_text())["have_offset"], True)

    def test_samples_before_zero_are_blank_not_zero(self):
        tracker = Tracker(bias=GYRO_BIAS)
        board = LoggedBoard(tracker, self.log)
        board.still(.5)
        name = self.log.start()
        board.still(.5)
        self.log.stop()
        c = read_samples(self.root / "logs" / name)
        self.assertGreater(len(c["t_s"]), 5)
        self.assertEqual(set(c["error_ut"]), {None})
        self.assertIn("no data", (self.root / "logs" / name / "plot.html").read_text())

    def test_an_empty_log_still_finishes(self):
        name = self.log.start()
        self.log.stop()
        self.assertIsNone(self.log.status()["error"])
        self.assertTrue((self.root / "logs" / name / "plot.html").exists())

    def test_nothing_is_recorded_when_not_logging(self):
        self.log.add(1.0, (0, 0, 1), (0, 0, 0), None, None)
        self.assertFalse((self.root / "logs").exists())

    def test_a_disk_problem_ends_the_log_quietly(self):
        self.log.start()
        self.log._raw_file.close()
        self.log.add(1.0, (0, 0, 1), (0, 0, 0), None, None)
        status = self.log.status()
        self.assertFalse(status["active"])
        self.assertIn("Recording stopped because a file could not be written", status["error"])


class Plot(unittest.TestCase):
    def test_thinning_keeps_the_spike(self):
        ts = [i * .01 for i in range(100000)]
        vs = [1.0] * 100000
        vs[54321] = 500.0
        vs[12345] = -300.0
        points = thin(ts, vs, 100)
        self.assertLessEqual(len(points), 200)
        self.assertIn((ts[54321], 500.0), points)
        self.assertIn((ts[12345], -300.0), points)
        self.assertEqual(points, sorted(points))

    def test_short_logs_are_not_thinned(self):
        self.assertEqual(len(thin([1.0, 2.0], [3.0, None])), 1)
        self.assertEqual(len(thin(list(range(100)), list(range(100)))), 100)

    def test_scale_covers_the_data(self):
        for v in (0.0, 4.0, 9.6, 10.0, 49.0, 4900.0, 9000.0):
            self.assertGreaterEqual(nice_top(v), v)

    def test_stats(self):
        c = {"t_s": [0.0, 1.0, 2.0, 3.0], "error_ut": [1.0, 3.0, 9.0, None],
             "turn_deg": [0.0, 40.0, 50.0, 60.0]}
        s = summarize(c)
        self.assertEqual((s["seconds"], s["samples"], s["worst"]), (3.0, 4, 9.0))
        self.assertAlmostEqual(s["mean"], 13 / 3)
        self.assertEqual(s["under"], 50.0)      # of the two turned samples with a error

    def test_the_plot_is_valid_and_has_every_trace(self):
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d) / "2026-01-01_00-00-00"
            folder.mkdir()
            with open(folder / "samples.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(SAMPLE_COLUMNS)
                for i in range(50):
                    w.writerow([i * .05, 1, 0, 0, 0, 0, 0, 0, i, 1, 2, 3, 1, 2, 3, 1, -2, 3, 3.7])
            write_plot(folder)
            text = (folder / "plot.html").read_text()
        svg = re.search(r"<svg.*</svg>", text, re.S).group(0)
        root = ET.fromstring(svg)
        titles = [e.text for e in root.iter("{http://www.w3.org/2000/svg}text")
                  if e.get("class") == "title"] or [e.text for e in root.iter("text")
                                                    if e.get("class") == "title"]
        self.assertEqual([t.strip() for t in titles],
                         ["field difference from the reference", "field difference along x",
                          "field difference along y", "field difference along z",
                          "Board rotation since reference measurement"])
        self.assertEqual(len(re.findall('class="trace"', svg)), 5)
        self.assertNotIn("{{", text)

    def test_older_logs_with_the_old_column_names_still_read(self):
        with tempfile.TemporaryDirectory() as d:
            header = [name.replace("error", "residual") for name in SAMPLE_COLUMNS]
            with open(Path(d) / "samples.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(header)
                w.writerow([0.1] * len(header))
            c = read_samples(d)
        self.assertEqual(c["error_ut"], [0.1])
        self.assertEqual(c["error_x_ut"], [0.1])

    def test_a_gap_breaks_the_line(self):
        c = {name: [] for name in SAMPLE_COLUMNS}
        for t in (0.0, 0.1, 0.2, 5.0, 5.1):
            for name in SAMPLE_COLUMNS:
                c[name].append(1.0)
            c["t_s"][-1] = t
        self.assertEqual(render("x", c).count('class="trace"'), 28)     # 11 plots, two sensors, 3 positions
