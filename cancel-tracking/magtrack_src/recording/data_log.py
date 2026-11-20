"""Records a run to its own folder, named by the time it started.

    logs/2026-10-30_23-15-02/
        raw.csv        every ICM step and consumed MMC readings, before correction
        samples.csv    what the view showed: orientation, field, error, position
        info.json      tracker/locator status at start, not a full event history
        plot.html      the visual, written when the log stops

Used by: earth_cancel.py, hardware/icm_loop.py, web/server.py.
Uses: recording/schema.py; recording/log_plot.py when a recording stops.
"""

import csv
import json
import threading
import time
from contextlib import ExitStack
from pathlib import Path

from magtrack_src.recording.log_plot import write_plot
from magtrack_src.recording.schema import RAW_COLUMNS, SAMPLE_COLUMNS, format_cells, sample_row

FLUSH_S = 1.0             # flush Python file buffers; not a power-loss durability guarantee


class DataLog:
    def __init__(self, root):
        self.root = Path(root)
        self._lock = threading.Lock()
        self._folder = None
        self._t0 = None
        self._started = 0.0
        self._flushed = 0.0
        self._rows = 0
        self._last = None
        self._error = None

    def _new_folder(self):
        """Time-based and unique, even for two logs in the same second."""
        name = stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        n = 1
        while True:
            try:
                (self.root / name).mkdir(parents=True)
                return self.root / name
            except FileExistsError:
                if not self.root.is_dir():
                    raise
                n += 1
                name = f"{stamp}_{n}"

    def start(self, info=None):
        with self._lock:
            if self._folder is not None:
                raise ValueError("A recording is already in progress.")
            # Keep new handles local until every startup write has succeeded.
            # Failed attempts may leave partial files, but never become active
            # or replace the last completed recording.
            try:
                folder = self._new_folder()
                with ExitStack() as opened:
                    (folder / "info.json").write_text(json.dumps(
                        {"started": time.strftime("%Y-%m-%dT%H:%M:%S%z"), **(info or {})}, indent=2))
                    raw_file = opened.enter_context(open(folder / "raw.csv", "w", newline=""))
                    samples_file = opened.enter_context(open(folder / "samples.csv", "w", newline=""))
                    raw, samples = csv.writer(raw_file), csv.writer(samples_file)
                    raw.writerow(RAW_COLUMNS)
                    samples.writerow(SAMPLE_COLUMNS)
                    raw_file.flush()
                    samples_file.flush()
                    started = time.monotonic()
                    self._raw_file, self._samples_file = raw_file, samples_file
                    self._raw, self._samples = raw, samples
                    self._folder, self._t0, self._rows, self._error = folder, None, 0, None
                    self._started = started
                    opened.pop_all()  # ownership transfers to the active recording
            except OSError as e:
                self._error = f"Recording could not start: {e}"
                raise
            return folder.name

    def add(self, now, accel, gyro, mag, sample, mmc=None):
        """One sensor reading (``mag`` may be ``None``) and its view sample, if
        any.  ``mmc`` is a fresh MMC ``(field in chip axes, age)``.  A disk problem
        ends the log, never the sensor loop."""
        with self._lock:
            if self._folder is None:
                return
            if self._t0 is None:
                self._t0 = self._flushed = now
            t = now - self._t0
            try:
                field, age = mmc or ([None] * 3, None)
                self._raw.writerow(format_cells([t, *accel, *gyro, *(mag or [None] * 3), *field, age]))
                if sample is not None:
                    self._samples.writerow(sample_row(t, sample))
                    self._rows += 1
                if now - self._flushed >= FLUSH_S:
                    self._flushed = now
                    self._raw_file.flush()
                    self._samples_file.flush()
            except (OSError, ValueError) as e:
                self._error = f"Recording stopped because a file could not be written: {e}"
                self._close()

    def _close(self):
        """Close the files; returns the finished folder."""
        folder, self._folder = self._folder, None
        self._last = folder.name
        for f in (self._raw_file, self._samples_file):
            try:
                f.close()
            except OSError as e:
                self._error = f"The recording may be incomplete because a file could not be closed: {e}"
        return folder

    def stop(self):
        with self._lock:
            if self._folder is None:
                raise ValueError("No recording is active.")
            folder = self._close()
        try:
            write_plot(folder)
        except (OSError, ValueError) as e:
            with self._lock:
                self._error = f"The recording was saved, but its plots could not be generated: {e}"
        return folder.name

    def status(self):
        with self._lock:
            active = self._folder is not None
            return {"active": active,
                    "name": self._folder.name if active else None,
                    "seconds": round(time.monotonic() - self._started, 1) if active else 0.0,
                    "rows": self._rows,
                    "last": self._last,
                    "error": self._error}
