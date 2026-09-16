"""Orientation fusion for the ICM-20948.

A dependency-free Madgwick AHRS filter plus a small magnetometer
hard/soft-iron calibrator and a gyro-bias nuller.  No numpy -- everything
is plain floats so it runs anywhere ``smbus2`` does.

Why this exists
---------------
The accelerometer only sees gravity, so on its own it gives roll and pitch
but is completely blind to rotation about the vertical (yaw / heading).
The gyroscope sees yaw *rate*; integrating it drifts.  Only the
magnetometer gives an absolute, drift-free heading.  The Madgwick filter
fuses all three into a single quaternion describing the full 3-D
orientation of the board relative to an Earth frame.

Frames
------
Body frame  = the ICM-20948 accel/gyro axes as silk-screened on the board.
Earth frame = the filter's convention (Tait-Bryan ZYX): ``yaw`` is rotation
about body Z, ``pitch`` about Y, ``roll`` about X.

The AK09916 magnetometer die inside the package does **not** share the
accel/gyro axes.  Per the datasheet its axes map into the accel/gyro frame
as ``(x, y, z)_body = (y, x, -z)_mag``.  :meth:`MagCalibration.apply`
returns values already rotated into the body frame, so its output can be
fed straight to :meth:`Madgwick.update`.
"""

from __future__ import annotations

import json
import math
from pathlib import Path


# --- quaternion helpers (w, x, y, z) --------------------------------------

def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_norm(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return (w / n, x / n, y / n, z / n)


def quat_to_euler(q):
    """(w,x,y,z) -> (yaw, pitch, roll) in degrees, Tait-Bryan ZYX."""
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(yaw), math.degrees(pitch), math.degrees(roll))


# --- Madgwick AHRS ------------------------------------------------------

class Madgwick:
    """Madgwick's gradient-descent AHRS.

    Call :meth:`update` (9-DOF, needs a calibrated magnetometer) or
    :meth:`update_imu` (6-DOF, accel + gyro only) once per sample.  Gyro is
    in **rad/s**; accel and mag units are arbitrary (they are normalised
    internally).  ``dt`` is the sample interval in seconds.
    """

    def __init__(self, beta=0.1):
        self.beta = float(beta)
        self.q = (1.0, 0.0, 0.0, 0.0)

    def update_imu(self, gx, gy, gz, ax, ay, az, dt):
        q0, q1, q2, q3 = self.q
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            recip = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
            ax *= recip; ay *= recip; az *= recip

            _2q0 = 2.0 * q0; _2q1 = 2.0 * q1; _2q2 = 2.0 * q2; _2q3 = 2.0 * q3
            _4q0 = 4.0 * q0; _4q1 = 4.0 * q1; _4q2 = 4.0 * q2
            _8q1 = 8.0 * q1; _8q2 = 8.0 * q2
            q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
            s1 = (_4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1
                  + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az)
            s2 = (4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
                  + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az)
            s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
            snorm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            if snorm > 0.0:                       # skip if already aligned
                recip = 1.0 / snorm
                s0 *= recip; s1 *= recip; s2 *= recip; s3 *= recip
                qDot1 -= self.beta * s0
                qDot2 -= self.beta * s1
                qDot3 -= self.beta * s2
                qDot4 -= self.beta * s3

        q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt
        self.q = quat_norm((q0, q1, q2, q3))
        return self.q

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz, dt):
        if mx == 0.0 and my == 0.0 and mz == 0.0:
            return self.update_imu(gx, gy, gz, ax, ay, az, dt)

        q0, q1, q2, q3 = self.q
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            recip = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
            ax *= recip; ay *= recip; az *= recip
            recip = 1.0 / math.sqrt(mx * mx + my * my + mz * mz)
            mx *= recip; my *= recip; mz *= recip

            _2q0mx = 2.0 * q0 * mx
            _2q0my = 2.0 * q0 * my
            _2q0mz = 2.0 * q0 * mz
            _2q1mx = 2.0 * q1 * mx
            _2q0 = 2.0 * q0
            _2q1 = 2.0 * q1
            _2q2 = 2.0 * q2
            _2q3 = 2.0 * q3
            _2q0q2 = 2.0 * q0 * q2
            _2q2q3 = 2.0 * q2 * q3
            q0q0 = q0 * q0
            q0q1 = q0 * q1
            q0q2 = q0 * q2
            q0q3 = q0 * q3
            q1q1 = q1 * q1
            q1q2 = q1 * q2
            q1q3 = q1 * q3
            q2q2 = q2 * q2
            q2q3 = q2 * q3
            q3q3 = q3 * q3

            hx = (mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1
                  + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3)
            hy = (_2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2
                  - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3)
            _2bx = math.sqrt(hx * hx + hy * hy)
            _2bz = (-_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3
                    - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3)
            _4bx = 2.0 * _2bx
            _4bz = 2.0 * _2bz

            s0 = (-_2q2 * (2.0 * q1q3 - _2q0q2 - ax)
                  + _2q1 * (2.0 * q0q1 + _2q2q3 - ay)
                  - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                  + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                  + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz))
            s1 = (_2q3 * (2.0 * q1q3 - _2q0q2 - ax)
                  + _2q0 * (2.0 * q0q1 + _2q2q3 - ay)
                  - 4.0 * q1 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az)
                  + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                  + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                  + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz))
            s2 = (-_2q0 * (2.0 * q1q3 - _2q0q2 - ax)
                  + _2q3 * (2.0 * q0q1 + _2q2q3 - ay)
                  - 4.0 * q2 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az)
                  + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                  + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                  + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz))
            s3 = (_2q1 * (2.0 * q1q3 - _2q0q2 - ax)
                  + _2q2 * (2.0 * q0q1 + _2q2q3 - ay)
                  + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                  + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                  + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz))
            snorm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            if snorm > 0.0:                       # skip if already aligned
                recip = 1.0 / snorm
                s0 *= recip; s1 *= recip; s2 *= recip; s3 *= recip
                qDot1 -= self.beta * s0
                qDot2 -= self.beta * s1
                qDot3 -= self.beta * s2
                qDot4 -= self.beta * s3

        q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt
        self.q = quat_norm((q0, q1, q2, q3))
        return self.q


# --- magnetometer calibration ----------------------------------------------

class MagCalibration:
    """Hard-iron offset + first-order soft-iron scale for the AK09916, plus
    the axis remap into the accel/gyro body frame."""

    def __init__(self, offset=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0), remap=True):
        self.offset = tuple(float(v) for v in offset)
        self.scale = tuple(float(v) for v in scale)
        self.remap = bool(remap)

    def apply(self, m):
        """Raw AK09916 (mx,my,mz) in uT -> calibrated vector in the body frame."""
        cx = (m[0] - self.offset[0]) * self.scale[0]
        cy = (m[1] - self.offset[1]) * self.scale[1]
        cz = (m[2] - self.offset[2]) * self.scale[2]
        if self.remap:
            return (cy, cx, -cz)   # AK09916 axes -> ICM accel/gyro axes
        return (cx, cy, cz)

    def as_dict(self):
        return {"offset": list(self.offset), "scale": list(self.scale), "remap": self.remap}

    @classmethod
    def from_dict(cls, d):
        return cls(d.get("offset", (0, 0, 0)), d.get("scale", (1, 1, 1)), d.get("remap", True))

    def save(self, path):
        Path(path).write_text(json.dumps(self.as_dict(), indent=2))

    @classmethod
    def load(cls, path):
        p = Path(path)
        if not p.exists():
            return None
        try:
            return cls.from_dict(json.loads(p.read_text()))
        except (ValueError, OSError):
            return None


class MagCollector:
    """Accumulates raw magnetometer extremes during a calibration spin and
    turns them into a :class:`MagCalibration`."""

    MIN_SAMPLES = 200

    def __init__(self):
        self.n = 0
        self.lo = [math.inf, math.inf, math.inf]
        self.hi = [-math.inf, -math.inf, -math.inf]

    def add(self, m):
        self.n += 1
        for i in range(3):
            if m[i] < self.lo[i]:
                self.lo[i] = m[i]
            if m[i] > self.hi[i]:
                self.hi[i] = m[i]

    def span(self):
        if not self.n:
            return [0.0, 0.0, 0.0]
        return [self.hi[i] - self.lo[i] for i in range(3)]

    def result(self):
        span = self.span()
        if self.n < self.MIN_SAMPLES or min(span) < 5.0:
            raise ValueError(
                f"not enough magnetometer motion (n={self.n}, span={[round(s,1) for s in span]} uT); "
                "rotate the board through every orientation and try again")
        offset = [(self.hi[i] + self.lo[i]) / 2.0 for i in range(3)]
        radius = [s / 2.0 for s in span]
        avg = sum(radius) / 3.0
        scale = [avg / radius[i] if radius[i] > 1e-6 else 1.0 for i in range(3)]
        return MagCalibration(offset, scale, remap=True)


# --- the full fusion pipeline (thread-safe) ------------------------------

class OrientationTracker:
    """Wraps the filter, the magnetometer calibration, a gyro-bias nuller,
    and a settable reference pose.  Feed it raw sensor readings; read back
    the orientation relative to the reference.

    All public methods are safe to call from another thread (e.g. an HTTP
    handler) while :meth:`process` runs in the sensor loop.
    """

    RECAL_HOLD_S = 4.0   # keep the view steady this long after a mag calibration

    def __init__(self, beta=0.1, cal_path="imu_mag_cal.json"):
        import threading
        self._lock = threading.Lock()
        self.filter = Madgwick(beta=beta)
        self.cal_path = cal_path
        self.cal = MagCalibration.load(cal_path)
        self._collector = None
        self._q_ref = None
        # Finishing a magnetometer calibration flips the filter from 6-DOF to
        # 9-DOF; its yaw then slews onto magnetic north over a few seconds,
        # which otherwise drags the box back toward the reference ("snaps to
        # zero"). These freeze the *displayed* pose through that transient.
        self._recal_hold_qrel = None
        self._recal_hold_until = 0.0
        self._pending_recal_hold = False
        self._last_mag = None
        self._last_t = None
        # gyro bias
        self._gbias = [0.0, 0.0, 0.0]
        self._bias_accum = [0.0, 0.0, 0.0]
        self._bias_n = 0
        self._bias_target = 0
        self.null_gyro(80)          # auto-null on startup (assumes board is still)

    # -- sensor-loop side --------------------------------------------------
    def process(self, accel_g, gyro_dps, mag_ut, now):
        """One fusion step.  ``now`` is a monotonic timestamp in seconds.
        Returns a dict of derived values to attach to the outgoing sample."""
        with self._lock:
            dt = 0.0 if self._last_t is None else now - self._last_t
            self._last_t = now
            if dt <= 0.0 or dt > 0.2:
                dt = 0.0            # first sample, or a stall: don't integrate

            gx, gy, gz = gyro_dps
            if self._bias_n < self._bias_target:
                for i, v in enumerate((gx, gy, gz)):
                    self._bias_accum[i] += v
                self._bias_n += 1
                if self._bias_n == self._bias_target:
                    self._gbias = [a / self._bias_n for a in self._bias_accum]
            gx -= self._gbias[0]; gy -= self._gbias[1]; gz -= self._gbias[2]
            grx, gry, grz = (math.radians(v) for v in (gx, gy, gz))
            ax, ay, az = accel_g

            if mag_ut is not None:
                self._last_mag = mag_ut
                if self._collector is not None:
                    self._collector.add(mag_ut)

            mbody = None
            if self.cal is not None and self._last_mag is not None:
                mbody = self.cal.apply(self._last_mag)

            if dt > 0.0:
                if mbody is not None:
                    self.filter.update(grx, gry, grz, ax, ay, az,
                                       mbody[0], mbody[1], mbody[2], dt)
                else:
                    self.filter.update_imu(grx, gry, grz, ax, ay, az, dt)

            q_abs = self.filter.q

            # Just finished a magnetometer calibration: snapshot the pose the
            # user is currently looking at, then for a short window re-base the
            # reference every step so that relative pose stays put while the
            # filter re-aligns its absolute yaw to magnetic north. Without this
            # the box appears to drift back to a zeroed pose after calibration.
            if self._pending_recal_hold:
                self._pending_recal_hold = False
                self._recal_hold_qrel = None if self._q_ref is None else \
                    quat_norm(quat_mul(quat_conj(self._q_ref), q_abs))
                self._recal_hold_until = now + self.RECAL_HOLD_S
            if self._recal_hold_qrel is not None:
                if self._q_ref is not None and now < self._recal_hold_until:
                    self._q_ref = quat_norm(
                        quat_mul(q_abs, quat_conj(self._recal_hold_qrel)))
                else:
                    self._recal_hold_qrel = None

            q_rel = q_abs if self._q_ref is None else quat_norm(
                quat_mul(quat_conj(self._q_ref), q_abs))
            yaw, pitch, roll = quat_to_euler(q_rel)
            heading = quat_to_euler(q_abs)[0] % 360.0
            fix = "9dof" if mbody is not None else "6dof"
            nulling = self._bias_n < self._bias_target

        return {
            "quat": [round(v, 5) for v in q_rel],
            "euler": [round(yaw, 2), round(pitch, 2), round(roll, 2)],
            "heading": round(heading, 1),
            "fix": fix,
            "nulling": nulling,
        }

    # -- control side (other threads) -----------------------------------
    def set_reference(self):
        with self._lock:
            self._q_ref = self.filter.q
            self._recal_hold_qrel = None   # a manual re-zero wins over the hold

    def clear_reference(self):
        with self._lock:
            self._q_ref = None
            self._recal_hold_qrel = None

    def null_gyro(self, n=80):
        with self._lock:
            self._bias_accum = [0.0, 0.0, 0.0]
            self._bias_n = 0
            self._bias_target = int(n)

    def calibrate_start(self):
        with self._lock:
            self._collector = MagCollector()

    def calibrate_cancel(self):
        with self._lock:
            self._collector = None

    def calibrate_finish(self):
        with self._lock:
            if self._collector is None:
                raise ValueError("no calibration in progress")
            cal = self._collector.result()   # may raise ValueError
            self.cal = cal
            self._collector = None
            self._pending_recal_hold = True
        cal.save(self.cal_path)
        return cal

    def status(self):
        with self._lock:
            coll = self._collector
            return {
                "calibrating": coll is not None,
                "cal_samples": coll.n if coll else 0,
                "cal_span": [round(v, 1) for v in coll.span()] if coll else None,
                "have_cal": self.cal is not None,
                "have_ref": self._q_ref is not None,
                "nulling_gyro": self._bias_n < self._bias_target,
                "gyro_bias": [round(v, 3) for v in self._gbias],
            }
