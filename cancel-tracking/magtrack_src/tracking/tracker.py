"""The core: turns sensor readings into orientation and error.

Knows nothing about I2C or the web.  ``update`` runs in the sensor loop; the
other methods are safe to call from the HTTP threads.
"""

import math
import threading
from collections import deque

from magtrack_src.tracking.earth_capture import Capture
from magtrack_src.calibration.mounting import fit_axes
from magtrack_src.config import BLOCK_SAMPLES, HISTORY_S, MAG_LATENCY_S, MAX_GAP_S, RATE_WINDOW_S
from magtrack_src.geometry.vectors import (angle_deg, mat_vec, norm, quat_angle_deg, quat_conj, quat_from_gravity,
                      quat_mul, quat_norm, quat_rotate, quat_to_euler, sub)
from magtrack_src.orientation.gyro_bias import GyroRest
from magtrack_src.orientation.madgwick import madgwick_step
from magtrack_src.calibration.tumble import Tumble, fit_tumble

IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
NO_OFFSET = (0.0, 0.0, 0.0)
NO_GAINS = (1.0, 1.0, 1.0)


class Tracker:
    def __init__(self, offset=None, bias=(0.0, 0.0, 0.0), store=None, gains=(1.0, 1.0, 1.0),
                 axes=None, fit_mounting=False, block_samples=BLOCK_SAMPLES,
                 use_calibration=True):
        self._lock = threading.Lock()
        self.axes = axes or IDENTITY              # sensor -> board; applied after the offset
        self._fit_mounting = fit_mounting         # True: a tumble replaces ``axes``
        self.mounted = axes is not None or not fit_mounting
        self.use_calibration = use_calibration    # False: readings as they came, only ``axes`` applied
        self._block_samples = block_samples
        self.gains = tuple(gains)                 # per-axis, applied to every reading first
        self.offset = offset                      # d, or None until a tumble
        self.earth = None                         # E, in the reference axes
        self.bias = tuple(bias)
        self.message = None
        self.gaps = 0
        self.rate_hz = 0.0                        # magnetometer readings per second
        self._rate_from = None
        self._rate_count = 0
        self._store = store                       # CalibrationFile, or None to save nothing
        self._q = None
        self._last_t = None
        self._q_ref = None
        self._t_ref = None
        self._hist = deque()
        self._tumble = None                       # Tumble while tumbling
        self.tumble_result = None
        self._capture = Capture()
        self._rest = GyroRest()
        self._block = []
        self._peak = 0.0
        self._peak_deg = 0.0
        self._turn = 0.0

    @property
    def _offset_now(self):
        return self.offset if self.use_calibration else NO_OFFSET

    @property
    def _gains_now(self):
        return self.gains if self.use_calibration else NO_GAINS

    # -- sensor loop ---------------------------------------------------------
    def update(self, accel, gyro, mag, now, mag_age=MAG_LATENCY_S):
        """One reading, accel and gyro in their own axes; ``mag`` is ``None`` if
        not fresh, and was measured ``mag_age`` seconds ago.  Returns a view sample
        every ``block_samples`` magnetometer samples, otherwise ``None``."""
        with self._lock:
            raw = mag
            if mag is not None:
                self._count_reading(now)
                mag = tuple(m * g for m, g in zip(mag, self._gains_now))
            if self._q is None:
                self._q = quat_from_gravity(accel)
            dt = 0.0 if self._last_t is None else now - self._last_t
            self._last_t = now
            if dt > MAX_GAP_S:
                self.gaps += 1
                dt = 0.0
            if dt > 0.0:
                g = [math.radians(gyro[i] - self.bias[i]) for i in range(3)]
                self._q = madgwick_step(self._q, *g, *accel, dt)
            self._hist.append((now, self._q))
            while self._hist[0][0] < now - HISTORY_S:
                self._hist.popleft()
            if raw is not None and self._tumble is not None:
                self._tumble.add(raw, self._q_at(now - mag_age))

            bias = self._rest.add(gyro, now)
            if bias is not None:
                self.bias = bias
            if self._capture is not None:
                self._capture_step(mag, now)
            if self._q_ref is None:
                return None

            q_rel = quat_norm(quat_mul(quat_conj(self._q_ref), self._q))
            self._turn = quat_angle_deg(q_rel)
            if mag is None:
                return None
            return self._sample(mag, now - mag_age, q_rel)

    def _count_reading(self, now):
        self._rate_count += 1
        if self._rate_from is None:
            self._rate_from, self._rate_count = now, 0
        elif now - self._rate_from >= RATE_WINDOW_S:
            self.rate_hz = self._rate_count / (now - self._rate_from)
            self._rate_from, self._rate_count = now, 0

    def _q_at(self, when):
        for t, q in reversed(self._hist):
            if t <= when:
                return q
        return self._hist[0][1]

    def _sample(self, mag, measured_at, q_rel):
        q_at_mag = quat_norm(quat_mul(quat_conj(self._q_ref), self._q_at(measured_at)))
        offset = self._offset_now
        u = sub(mag, offset) if offset is not None else mag
        self._block.append(quat_rotate(q_at_mag, mat_vec(self.axes, u)))
        if len(self._block) < self._block_samples:
            return None

        n = len(self._block)
        measured = tuple(sum(v[i] for v in self._block) / n for i in range(3))
        self._block = []
        yaw, pitch, roll = quat_to_euler(q_rel)
        out = {"quat": [round(v, 4) for v in q_rel],
               "euler": [round(yaw, 1), round(pitch, 1), round(roll, 1)],
               "turn": round(self._turn, 1),
               "measured": [round(v, 2) for v in measured],
               "measured_mag": round(norm(measured), 2),
               "predicted": None, "error": None, "error_ut": None, "error_deg": None,
               "calibration_on": self.use_calibration}
        if self.earth is not None and offset is not None:
            error = sub(measured, self.earth)
            size = norm(error)
            off_by = angle_deg(measured, self.earth)
            self._peak = max(self._peak, size)
            self._peak_deg = max(self._peak_deg, off_by)
            out.update(predicted=[round(v, 2) for v in self.earth],
                       predicted_mag=round(norm(self.earth), 2),
                       error=[round(v, 2) for v in error],
                       error_ut=round(size, 2), peak=round(self._peak, 2),
                       error_deg=round(off_by, 2), peak_deg=round(self._peak_deg, 2))
        return out

    def _capture_step(self, mag, now):
        done = self._capture.add(self._q, mag, now)
        if done is None:
            return
        self._capture = None
        u, q = done
        offset = self._offset_now
        self._set_reference(q, mat_vec(self.axes, sub(u, offset)) if offset is not None else None)

    def _set_reference(self, q, earth):
        self._q_ref = q
        self._t_ref = self._last_t
        self.earth = earth
        self._block = []
        self._peak = self._peak_deg = 0.0

    # -- control side --------------------------------------------------------
    def capture_earth(self):
        with self._lock:
            self.message = None
            self._capture = Capture()

    def set_calibration(self, on):
        """Use the tumble's midpoints and corrections, or the readings as they came.
        Earth was measured the other way, so it's measured again."""
        with self._lock:
            if self._tumble is not None:
                raise ValueError("finish or cancel the tumble first")
            self.use_calibration = on
            self.earth = None
            self._block = []
            self.message = None
            self._capture = Capture()

    def tumble_start(self):
        with self._lock:
            self._tumble = Tumble()
            self.tumble_result = None
            self.message = None

    def tumble_cancel(self):
        with self._lock:
            self._tumble = None

    def tumble_finish(self):
        """Fit outside the lock, so the sensor loop never waits for it.  Then Earth
        has to be measured again, in the new calibration."""
        with self._lock:
            t = self._tumble
            if t is None:
                raise ValueError("no tumble running")
            if not t.ready:
                raise ValueError("keep turning: not enough directions covered yet")
            self._tumble = None
        try:
            result = fit_tumble(t.points)
        except ValueError as e:
            if self._store is not None:
                self._store.log_tumble(t.points, None)
            with self._lock:
                self.message = f"Tumble failed: {e}"
            return
        offset = tuple(c * g for c, g in zip(result["center"], result["gains"]))
        centred = [tuple(m * g - o for m, g, o in zip(p, result["gains"], offset)) for p in t.points]
        result["mounting"] = fit_axes(centred, t.quats)
        with self._lock:
            self.gains = tuple(result["gains"])
            self.offset = offset
            if self._fit_mounting:
                self.axes = result["mounting"]["axes"]
                self.mounted = True
            self.earth = None
            self.tumble_result = result
            self.message = None
            self._capture = Capture()
            bias = self.bias
        if self._store is not None:
            self._store.log_tumble(t.points, result)
            self._store.save(offset, bias, result["gains"])
            if self._fit_mounting:
                self._store.save_axes(result["mounting"]["axes"])

    def reset_calibration(self):
        """Start again from nothing, as on a first run.  Stops any tumble."""
        with self._lock:
            self.gains = (1.0, 1.0, 1.0)
            self.offset = self.earth = None
            self.bias = (0.0, 0.0, 0.0)
            self._tumble = self.tumble_result = None
            self._block = []
            self._peak = self._peak_deg = 0.0
            self.message = "Calibration cleared. Rest the board for a second, then click Tumble."
            self._capture = Capture()
        if self._store is not None:
            try:
                self._store.clear()
            except OSError as e:
                with self._lock:
                    self.message = f"Couldn't delete the saved calibration: {e}"

    def sensor_lost(self):
        """The board may have moved unseen: orientation can't be trusted, so
        start over."""
        with self._lock:
            self._q = self._last_t = self._q_ref = self._t_ref = self.earth = None
            self._hist.clear()
            self._block = []
            self.message = "sensor reconnected: captured again from the current pose"
            self._capture = Capture()

    def _tumble_now(self):
        now = self._tumble.now if self._tumble else None
        return None if now is None else [round(v, 3) for v in now]

    def status(self):
        with self._lock:
            c = self._capture
            age = None
            if self._t_ref is not None and self._last_t is not None:
                age = round(self._last_t - self._t_ref, 1)
            return {
                "capturing": "earth" if c else None,
                "capture_s": round(self._last_t - c.start, 1)
                             if c and c.start is not None else 0.0,
                "tumbling": self._tumble is not None,
                "tumble_points": len(self._tumble.points) if self._tumble else 0,
                "tumble_coverage": round(self._tumble.coverage, 3) if self._tumble else 0.0,
                "tumble_ready": self._tumble.ready if self._tumble else False,
                "tumble_counting": self._tumble.counting if self._tumble else False,
                "tumble_swept": sorted(self._tumble.swept) if self._tumble else [],
                "tumble_now": self._tumble_now(),
                "tumble_swing": [round(v, 1) for v in self._tumble.swing] if self._tumble else [0.0] * 3,
                "tumble_result": self.tumble_result,
                "gains": [round(v, 4) for v in self.gains],
                "calibration_on": self.use_calibration,
                "mounted": self.mounted,
                "have_offset": self.offset is not None,
                "offset": None if self.offset is None else [round(v, 2) for v in self.offset],
                "earth": None if self.earth is None else [round(v, 2) for v in self.earth],
                "bias": [round(v, 3) for v in self.bias],
                "ref_age_s": age,
                "gaps": self.gaps,
                "rate_hz": round(self.rate_hz, 1),
                "message": self.message,
            }
