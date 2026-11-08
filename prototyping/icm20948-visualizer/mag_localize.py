"""Single-dipole magnetic localization for the ICM-20948 magnetometer.

Given a calibrated 3-axis magnetometer reading (body frame) and the board's
absolute orientation quaternion from the AHRS filter, estimate where the IMU is
relative to one small DC electromagnet placed nearby.

Physics
-------
More than ~3x its own size away, an energised electromagnet is a magnetic
dipole with moment ``m`` along its axis.  Its field at the sensor, in the AHRS
world frame, is

    B(p) = (mu0 |m| / 4 pi) * (1 / r^3) * [ 3 (u . p_hat) p_hat - u ]

with ``u`` the unit axis of the magnet, ``p`` the vector from magnet to sensor,
``r = |p|``, ``p_hat = p / r``.  Writing ``k = mu0 |m| / 4 pi`` and
``A = k / r^3``:

    B_parallel  = A (3 cos^2(t) - 1)          ( t = angle between u and p_hat )
    |B|         = A sqrt(1 + 3 cos^2(t))

Measuring the whole vector ``B`` (three numbers) inverts this for ``p`` (three
numbers).  The map is exactly two-to-one -- ``B(p) == B(-p)`` -- so one reading
fixes ``r`` and the full direction of ``p`` up to a single front/back sign,
which the caller pins with ``side`` (the "Flip side" button in the UI).

Pipeline
--------
1. Hard/soft-iron calibration of the AK09916 lives in :mod:`imu_ahrs`
   (:class:`MagCalibration`); this module consumes its already-calibrated,
   body-frame output.
2. **Zero field** -- magnet OFF: average the calibrated reading rotated into the
   world frame.  Captures Earth's field plus static distortion, to subtract.
3. **Calibrate magnet** -- magnet ON, sensor held on the magnet axis at a known
   distance ``d``: the leftover ``B_em = B_world - field_zero`` points along the
   axis with magnitude ``2k / d^3``, giving ``k = |B_em| d^3 / 2`` and
   ``u = B_em / |B_em|``.
4. Every sample: ``B_em = B_world - field_zero``, solve the dipole equation for
   ``p``, express it in a magnet-centred frame, smooth it, and report it plus
   the offset from a caller-chosen start point.

No numpy: plain tuples of floats, stdlib only, same as the rest of the tool.
All lengths are centimetres, all fields microtesla; ``k`` is therefore in
uT*cm^3 and every formula above stays self-consistent.
"""

from __future__ import annotations

import json
import math
import threading
from pathlib import Path


# --- tuning ---------------------------------------------------------------

B_MIN_UT       = 1.0    # below this |B_em| there is no usable position fix
COIL_CAL_MIN_UT = 3.0   # refuse to calibrate the magnet on a weaker signal
MIN_CAL_CM     = 6.0    # accepted range for the calibration distance
MAX_CAL_CM     = 40.0
NEAR_FIELD_CM  = 8.0    # inside this the dipole model is not trustworthy
EMA_ALPHA      = 0.35   # position smoothing (higher = snappier, noisier)
JUMP_GATE_CM   = 40.0   # ignore single-frame jumps larger than this
FIELD_ZERO_SAMPLES = 40 # ~1.5 s at 30 Hz


# --- small vector helpers (tuples of 3 floats) --------------------------

def _add(a, b):   return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def _sub(a, b):   return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def _scale(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def _dot(a, b):   return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def _norm(a):     return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
def _dist(a, b):  return _norm(_sub(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _unit(a):
    n = _norm(a)
    return (a[0] / n, a[1] / n, a[2] / n) if n > 1e-12 else (0.0, 0.0, 0.0)


def quat_rotate(q, v):
    """Rotate body-frame vector ``v`` into the world frame by quaternion
    ``q = (w, x, y, z)`` (the AHRS convention: ``qRot(q, v_body) = v_world``)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def _signal(b_mag):
    if b_mag < B_MIN_UT:  return "none"
    if b_mag < 2.5:       return "weak"
    if b_mag < 6.0:       return "ok"
    return "good"


# --- the dipole inversion ---------------------------------------------------

def solve_position(B, k, u, side):
    """Invert the dipole field for the magnet -> sensor vector.

    ``B``    measured electromagnet field, world frame, uT
    ``k``    mu0 |m| / 4 pi, in uT*cm^3 (from calibration)
    ``u``    unit magnet axis, world frame
    ``side`` +1 / -1: which half-space (sign of ``p . u``) the sensor is in

    Returns ``(p_world_cm, r_cm, cos_theta)`` or ``None`` if the signal is
    too weak to trust.
    """
    b = _norm(B)
    if b < B_MIN_UT:
        return None

    beta = _dot(B, u) / b                 # signed cos of angle(B, axis)
    rho = beta * beta                     # in [0, 1]
    disc = math.sqrt(rho * (rho + 8.0))
    # 9 c^2 - (6 + 3 rho) c + (1 - rho) = 0, with c = cos^2(theta);
    # sign of (3c - 1) must match sign of beta, which selects the root.
    if beta >= 0.0:
        c2 = (2.0 + rho + disc) / 6.0
    else:
        c2 = (2.0 + rho - disc) / 6.0
    c2 = min(1.0, max(0.0, c2))

    A = b / math.sqrt(1.0 + 3.0 * c2)     # = k / r^3, always > 0
    r = (k / A) ** (1.0 / 3.0)

    cos_t = math.sqrt(c2)
    sin_t = math.sqrt(max(0.0, 1.0 - c2))

    # lateral direction: the part of B perpendicular to the axis. Undefined
    # on-axis (rotational symmetry) -- fall back to a pure axial position.
    B_perp = _sub(B, _scale(u, _dot(B, u)))
    s_hat = _unit(B_perp) if _norm(B_perp) > 1e-6 else (0.0, 0.0, 0.0)

    p_hat = _add(_scale(u, cos_t), _scale(s_hat, sin_t))   # the +side branch
    p = _scale(p_hat, r)
    if side < 0:
        p = _scale(p, -1.0)                                # B(-p) == B(p)
    return p, r, (cos_t if side > 0 else -cos_t)


# --- persisted magnet calibration ---------------------------------------

class CoilCal:
    """Everything ``solve_position`` needs plus a fixed magnet-centred frame:
    ``axis`` is +Z, ``x_axis`` / ``y_axis`` complete a right-handed basis so a
    solved world position can be reported as tidy (x, y, z) around the magnet."""

    def __init__(self, k, axis, x_axis, y_axis, distance_cm):
        self.k = float(k)
        self.axis = _unit(axis)
        self.x_axis = _unit(x_axis)
        self.y_axis = _unit(y_axis)
        self.distance_cm = float(distance_cm)

    def to_coil_frame(self, p_world):
        return (_dot(p_world, self.x_axis),
                _dot(p_world, self.y_axis),
                _dot(p_world, self.axis))

    def as_dict(self):
        return {"k": self.k, "axis": list(self.axis),
                "x_axis": list(self.x_axis), "y_axis": list(self.y_axis),
                "distance_cm": self.distance_cm}

    @classmethod
    def from_dict(cls, d):
        return cls(d["k"], d["axis"], d["x_axis"], d["y_axis"], d["distance_cm"])

    def save(self, path):
        Path(path).write_text(json.dumps(self.as_dict(), indent=2))

    @classmethod
    def load(cls, path):
        p = Path(path)
        if not p.exists():
            return None
        try:
            return cls.from_dict(json.loads(p.read_text()))
        except (ValueError, KeyError, OSError):
            return None


# --- the pipeline (thread-safe, mirrors OrientationTracker) ------------

class MagLocalizer:
    """Feed :meth:`process` one calibrated body-frame magnetometer vector and
    the board's absolute orientation quaternion per sample; read back the
    position relative to the electromagnet.  All other methods are safe to call
    from another thread (an HTTP handler) while :meth:`process` runs."""

    def __init__(self, cal_path="magpos_cal.json"):
        self._lock = threading.Lock()
        self.cal_path = str(cal_path)
        self.coil = CoilCal.load(self.cal_path)

        self._field_zero = None            # (bx, by, bz) world uT, magnet OFF
        self._fz_accum = [0.0, 0.0, 0.0]
        self._fz_n = 0
        self._fz_target = 0

        self._side = 1
        self._start = None                 # p0 in coil frame, cm
        self._p_smooth = None              # last smoothed world position, cm
        self._last_pos_coil = None
        self._last_b_em = None             # last world B_em vector, uT
        self._b_mag = None                 # last |B_em|, uT

    # -- sensor-loop side ------------------------------------------------
    def process(self, mbody, q_abs, now):
        out = {
            "pos": None, "delta": None, "range_cm": None, "lateral_cm": None,
            "b_em": None, "b_mag": None, "signal": "none",
            "near_field": False, "state": "need-mag-cal",
            "side": self._side, "have_start": self._start is not None,
        }
        with self._lock:
            if mbody is None or q_abs is None:
                return out

            mw = quat_rotate(q_abs, mbody)

            if self._fz_n < self._fz_target:
                for i in range(3):
                    self._fz_accum[i] += mw[i]
                self._fz_n += 1
                if self._fz_n == self._fz_target:
                    self._field_zero = tuple(a / self._fz_n for a in self._fz_accum)
                out["state"] = "zeroing-field"
                return out

            if self._field_zero is None:
                out["state"] = "need-field-zero"
                return out

            b_em = _sub(mw, self._field_zero)
            b_mag = _norm(b_em)
            self._last_b_em = b_em
            self._b_mag = b_mag
            out["b_em"] = [round(v, 3) for v in b_em]
            out["b_mag"] = round(b_mag, 3)
            out["signal"] = _signal(b_mag)

            if self.coil is None:
                out["state"] = "need-coil-cal"
                return out
            out["state"] = "ready"

            sol = solve_position(b_em, self.coil.k, self.coil.axis, self._side)
            if sol is None:
                out["signal"] = "none"
                return out
            p_world, _r, _cos_t = sol

            if self._p_smooth is None:
                self._p_smooth = p_world
            elif _dist(p_world, self._p_smooth) < JUMP_GATE_CM:
                a = EMA_ALPHA
                self._p_smooth = tuple(
                    a * p_world[i] + (1.0 - a) * self._p_smooth[i] for i in range(3))
            # else: single-frame glitch, hold the previous estimate

            p_coil = self.coil.to_coil_frame(self._p_smooth)
            self._last_pos_coil = p_coil
            out["pos"] = [round(v, 2) for v in p_coil]
            out["range_cm"] = round(_norm(p_coil), 2)
            out["lateral_cm"] = round(math.hypot(p_coil[0], p_coil[1]), 2)
            out["near_field"] = _norm(p_coil) < NEAR_FIELD_CM
            if self._start is not None:
                out["delta"] = [round(p_coil[i] - self._start[i], 2) for i in range(3)]
            return out

    # -- control side (other threads) ---------------------------------
    def zero_field_start(self, n=None):
        with self._lock:
            self._fz_accum = [0.0, 0.0, 0.0]
            self._fz_n = 0
            self._fz_target = int(n or FIELD_ZERO_SAMPLES)
            self._field_zero = None
            self._p_smooth = None

    def zero_field_cancel(self):
        with self._lock:
            self._fz_target = 0
            self._fz_n = 0

    def calibrate_coil(self, distance_cm):
        with self._lock:
            if self._field_zero is None:
                raise ValueError("zero the ambient field first (magnet OFF)")
            if self._last_b_em is None or self._b_mag is None or self._b_mag < COIL_CAL_MIN_UT:
                raise ValueError(
                    f"electromagnet field too weak ({self._b_mag or 0.0:.2f} uT) -- "
                    "switch it ON and hold the sensor on the axis, closer in")
            try:
                d = float(distance_cm)
            except (TypeError, ValueError):
                raise ValueError("distance must be a number of centimetres")
            if not (MIN_CAL_CM <= d <= MAX_CAL_CM):
                raise ValueError(f"distance must be {MIN_CAL_CM:g}..{MAX_CAL_CM:g} cm")

            b_em = self._last_b_em
            u = _unit(b_em)                                  # on-axis field // m
            k = _norm(b_em) * d ** 3 / 2.0                   # |B_em| = 2k/d^3
            ref = (1.0, 0.0, 0.0) if abs(u[2]) > 0.9 else (0.0, 0.0, 1.0)
            x_axis = _unit(_cross(ref, u))
            y_axis = _cross(u, x_axis)
            self.coil = CoilCal(k, u, x_axis, y_axis, d)
            self._p_smooth = None
            self._start = None
        self.coil.save(self.cal_path)
        return self.coil

    def set_start(self):
        with self._lock:
            if self._last_pos_coil is None:
                raise ValueError("no position fix yet -- get a signal first")
            self._start = self._last_pos_coil

    def clear_start(self):
        with self._lock:
            self._start = None

    def flip_side(self):
        with self._lock:
            self._side = -self._side
            self._p_smooth = None
            return self._side

    def status(self):
        with self._lock:
            return {
                "have_field_zero": self._field_zero is not None,
                "zeroing_field": self._fz_n < self._fz_target,
                "have_coil_cal": self.coil is not None,
                "coil_k": round(self.coil.k, 1) if self.coil else None,
                "coil_distance_cm": self.coil.distance_cm if self.coil else None,
                "side": self._side,
                "have_start": self._start is not None,
                "b_mag": round(self._b_mag, 2) if self._b_mag is not None else None,
            }
