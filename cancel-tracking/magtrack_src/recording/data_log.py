"""Records a run to its own folder, named by the time it started.

    logs/2026-10-30_23-15-02/
        raw.csv        every ICM step and consumed MMC readings, before correction
        samples.csv    what the view showed: orientation, field, error, position
        info.json      tracker/locator status at start, not a full event history
        plot.html      the visual, written when the log stops

Used by: earth_cancel.py, hardware/icm_loop.py, web/server.py.  Uses: recording/log_plot.py.
"""

import csv
import json
import threading
import time
from pathlib import Path

from magtrack_src.recording.log_plot import SAMPLE_COLUMNS, write_plot

RAW_COLUMNS = ["t_s", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
               "mx_ut", "my_ut", "mz_ut",
               "mmc_x_ut", "mmc_y_ut", "mmc_z_ut", "mmc_age_s"]
FLUSH_S = 1.0             # flush Python file buffers; not a power-loss durability guarantee


def _cells(values):
    return ["" if v is None else f"{v:.5f}" for v in values]


def _sample_row(t, s):
    m = s.get("mmc") or {}
    position = (s.get("locate") or {}).get("position")
    return _cells([t, *s["quat"], *s["euler"], s["turn"], *s["measured"],
                   *(s["predicted"] or [None] * 3), *(s["error"] or [None] * 3), s["error_ut"],
                   s.get("error_deg"), *(m.get("measured") or [None] * 3),
                   *(m.get("predicted") or [None] * 3), *(m.get("error") or [None] * 3),
                   m.get("error_ut"), m.get("error_deg"),
                   s.get("calibration_on"), m.get("calibration_on"),
                   *(position or [None] * 3)])


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
                n += 1
                name = f"{stamp}_{n}"

    def start(self, info=None):
        with self._lock:
            if self._folder is not None:
                raise ValueError("A recording is already in progress.")
            folder = self._new_folder()
            (folder / "info.json").write_text(json.dumps(
                {"started": time.strftime("%Y-%m-%dT%H:%M:%S%z"), **(info or {})}, indent=2))
            self._raw_file = open(folder / "raw.csv", "w", newline="")
            self._samples_file = open(folder / "samples.csv", "w", newline="")
            self._raw = csv.writer(self._raw_file)
            self._samples = csv.writer(self._samples_file)
            self._raw.writerow(RAW_COLUMNS)
            self._samples.writerow(SAMPLE_COLUMNS)
            self._folder, self._t0, self._rows, self._error = folder, None, 0, None
            self._started = time.monotonic()
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
                self._raw.writerow(_cells([t, *accel, *gyro, *(mag or [None] * 3), *field, age]))
                if sample is not None:
                    self._samples.writerow(_sample_row(t, sample))
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
