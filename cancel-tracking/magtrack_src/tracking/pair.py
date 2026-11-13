"""Runs the two magnetometers side by side.

Each gets its own Tracker, fed the same gyro and accelerometer inputs. Orientation
state is duplicated; magnetic capture times, sample ages and averaging blocks
are separate. Comparison therefore includes timing and calibration effects.

Exposes the pair control/status interface used by the server.
A ``Locator``, if given, turns the ICM's error into a position on every sample.

Used by: earth_cancel.py, hardware/icm_loop.py, web/server.py.  Uses: the Trackers, MmcReader and Locator it's given.
"""


class Pair:
    def __init__(self, icm, mmc=None, reader=None, locator=None):
        self.icm, self.mmc, self.reader, self.locator = icm, mmc, reader, locator
        self.reading = None                   # this step's fresh MMC reading, for the log
        self._last = None                     # newest MMC view sample

    @property
    def _mmc_on(self):
        return self.reader is not None and self.reader.connected

    def update(self, accel, gyro, mag, now):
        self.reading = field = t = None
        if self.mmc is not None:
            self.reading = self.reader.take()
            if self.reading is not None:
                field, t = self.reading
                self.reading = (field, now - t)
            s = self.mmc.update(accel, gyro, field, now, now - t if t is not None else 0.0)
            if s is not None:
                self._last = s
        sample = self.icm.update(accel, gyro, mag, now)
        if sample is not None:
            sample["mmc"] = self._last if self._mmc_on else None
            if self.locator is not None:
                sample["locate"] = self.locator.update(sample)
        return sample

    def capture_reference_field(self):
        self.icm.capture_reference_field()
        if self.mmc is not None:
            self.mmc.capture_reference_field()

    def toggle_calibration(self, side):
        tracker = {"icm": self.icm, "mmc": self.mmc}[side]
        if tracker is None:
            raise ValueError("The selected sensor is disabled.")
        tracker.set_calibration(not tracker.use_calibration)

    def tumble_start(self):
        self.icm.tumble_start()
        if self.mmc is not None:
            self.mmc.tumble_start()

    def tumble_cancel(self):
        self.icm.tumble_cancel()
        if self.mmc is not None:
            self.mmc.tumble_cancel()

    def _waiting_for_mmc(self, mmc_status):
        """Only a first tumble needs the MMC: it measures how the chip is mounted.
        A disconnected MMC mustn't hold up the ICM's calibration."""
        return self._mmc_on and not mmc_status["mounted"] and not mmc_status["tumble_ready"]

    def tumble_finish(self):
        mmc = self.mmc.status() if self.mmc is not None else None
        if mmc is not None and mmc["tumbling"] and self._waiting_for_mmc(mmc):
            raise ValueError("Keep rotating the board. The MMC still needs more field directions or magnetic readings to estimate its mounting alignment.")
        self.icm.tumble_finish()
        if mmc is not None and mmc["tumbling"]:
            if self._mmc_on and mmc["tumble_ready"]:
                self.mmc.tumble_finish()
            else:
                self.mmc.tumble_cancel()

    def reset_calibration(self):
        self.icm.reset_calibration()
        if self.mmc is not None:
            self.mmc.reset_calibration()

    def sensor_lost(self):
        self.icm.sensor_lost()
        if self.mmc is not None:
            self.mmc.sensor_lost()

    def status(self):
        """The ICM's status with ``mmc`` beside it in the same shape, and
        ``finish_ready`` for both together."""
        out = self.icm.status()
        out["locate"] = self.locator.status() if self.locator is not None else None
        if self.mmc is None:
            return {**out, "mmc": None, "finish_ready": out["tumble_ready"]}
        mmc = self.mmc.status()
        return {**out, "finish_ready": out["tumble_ready"] and not self._waiting_for_mmc(mmc),
                "mmc": {**mmc, "connected": self.reader.connected, "error": self.reader.error}}
