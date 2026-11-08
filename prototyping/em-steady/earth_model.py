"""Always-on electromagnet localization: the math, no I/O.

The coil is never switched, so the magnetometer always sees

    b_body = R^T (E + B_em)                    (after hard/soft-iron correction)

where ``R`` rotates the board's axes into a fixed *reference frame*, ``E`` is
Earth's field in that frame (constant: same room, same place) and ``B_em`` the
electromagnet's field.  Knowing ``R`` and ``E``, the electromagnet's field is
simply

    B_em = R b_body - E

and is handed to :class:`em_localize.Localizer` for the dipole inversion.

Orientation (``R``)
-------------------
The magnetometer cannot help with orientation here: the coil's field would pull
any 9-DOF filter's "north" toward the magnet.  :class:`Attitude` therefore runs
Madgwick in 6-DOF mode (gyro + accelerometer).  Roll and pitch stay absolute
(gravity); yaw is integrated gyro and drifts with the residual gyro bias.  The
reference frame is the board's pose when :meth:`Attitude.set_reference` was
called -- the board sitting squared on a fixed mark on the jig.  Putting it back
on the mark and re-zeroing removes any accumulated yaw drift, and ``E`` (stored
in that same frame) stays valid across runs.

Earth's field (``E``)
---------------------
Nothing here relies on integrating the gyro through a long hand tumble: one
slip (a dropped I2C transfer, a stall) would silently ruin the whole fit.

1. With the coil off, the board is tumbled through every orientation.
   :func:`fit_iron` fits an axis-aligned ellipsoid to the raw vectors -> hard-iron
   offset and soft-iron scale.  This needs no orientation at all.
2. The board goes back on its mark, still, coil off.  The reference pose is
   re-taken there, so ``R`` is the identity and the corrected reading *is* ``E``.

Two gyro-free checks guard the fit: the corrected field must have the same length
in every orientation, and the same component along gravity.  The first is the
floor of how well Earth's field cancels.  :func:`gyro_consistency` also reports how
well the gyro-tracked orientation explains the tumble; it gates nothing, but it
shows whether orientation held up through fast motion.

Units: microtesla, seconds.  Stdlib only, like the rest of the tooling.
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
for _sib in ("em-localizer", "icm20948-visualizer"):
    _p = str(HERE.parent / _sib)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from em_localize import _dot, _norm, _scale, _solve, _sub, quat_rotate  # noqa: E402,F401
from imu_ahrs import Madgwick, MagCalibration, quat_conj, quat_mul, quat_norm  # noqa: E402


# --- tuning (adjust freely) -------------------------------------------------

AHRS_BETA = 0.05          # Madgwick gain: lower = less accel noise in tilt, slower settle
GYRO_NULL_SAMPLES = 200   # samples averaged for the startup / button gyro-bias null
AUTO_BIAS = True          # keep refining the gyro bias whenever the board sits still
STILL_WINDOW_S = 0.25     # gyro / accel are judged over blocks this long, not per sample
STILL_DPS = 0.5           # "still": each axis' block-mean bias-corrected rate is under this ...
STILL_G = 0.03            # ... and |accel| varies by less than this within the block ...
STILL_S = 1.0             # ... for this long
AUTO_BIAS_ALPHA = 0.1     # per-block weight of the still-time bias update
HISTORY_S = 0.25          # orientation history kept to time-align magnetometer samples
GAP_S = 0.2               # a pause this long loses orientation (no gyro was integrated)

EARTH_MIN_SAMPLES = 1000  # magnetometer samples needed for the iron fit (~10 s at 100 Hz)
EARTH_MIN_COVERAGE = 0.6  # share of the 26 gravity directions the board must visit
EARTH_MAX_RADIAL_UT = 5.0 # rms spread of the corrected field's length (near steel if above)
EARTH_MAX_VERT_UT = 8.0   # rms spread of the field along gravity (includes hand acceleration)
SOFT_IRON_MAX = 1.4       # reject ellipsoid scales outside [1/this, this]
EARTH_MARK_SAMPLES = 200  # samples averaged on the mark (~2 s at 100 Hz)
EARTH_MARK_MAX_UT = 3.0   # rms motion allowed while on the mark


# --- small helpers -------------------------------------------------------------

def quat_from_gravity(a):
    """Quaternion (body -> earth) that tilts ``a`` (accelerometer, pointing up
    when the board is flat) onto +z, with zero yaw."""
    n = _norm(a)
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    ax, ay, az = a[0] / n, a[1] / n, a[2] / n
    w = math.sqrt(max(0.0, (1.0 + az) / 2.0))
    if w < 1e-6:
        return (0.0, 1.0, 0.0, 0.0)          # upside down: 180 deg about x
    # vector part = (a x z) / (2 w)
    return quat_norm((w, ay / (2.0 * w), -ax / (2.0 * w), 0.0))


# --- orientation -----------------------------------------------------------------

class Attitude:
    """6-DOF orientation (gyro + accel), gyro-bias handling, and the reference
    pose.  :meth:`update` runs in the sensor loop; everything else is safe to
    call from other threads."""

    def __init__(self, beta=AHRS_BETA):
        self._lock = threading.Lock()
        self.filter = Madgwick(beta=beta)
        self._started = False
        self._last_t = None
        self._q_ref = None
        self._t_ref = None
        self._want_ref = False
        self._hist = deque()                # (t, q_abs)
        self._bias = [0.0, 0.0, 0.0]
        self._null_acc = [0.0, 0.0, 0.0]
        self._null_n = 0
        self._null_target = 0
        self._still_since = None
        self._blk = self._new_block()
        self._gaps = 0                      # orientation-losing pauses since the reference
        self.null_gyro()

    @staticmethod
    def _new_block():
        return {"t0": None, "n": 0, "g": [0.0, 0.0, 0.0], "lo": math.inf, "hi": -math.inf}

    def _refine_bias(self, accel_g, gyro_dps, now):
        """Judge stillness over short blocks: a single noisy sample must not
        restart the timer, and |accel| is compared to itself because the
        accelerometer is uncalibrated (it needn't read exactly 1 g)."""
        b = self._blk
        if b["t0"] is None:
            b["t0"] = now
        b["n"] += 1
        for i in range(3):
            b["g"][i] += gyro_dps[i]
        a = _norm(accel_g)
        b["lo"], b["hi"] = min(b["lo"], a), max(b["hi"], a)
        if now - b["t0"] < STILL_WINDOW_S:
            return
        mean = [v / b["n"] for v in b["g"]]
        still = (max(abs(mean[i] - self._bias[i]) for i in range(3)) < STILL_DPS
                 and b["hi"] - b["lo"] < STILL_G)
        self._blk = self._new_block()
        if not still:
            self._still_since = None
        elif self._still_since is None:
            self._still_since = now
        elif now - self._still_since >= STILL_S:
            for i in range(3):
                self._bias[i] += AUTO_BIAS_ALPHA * (mean[i] - self._bias[i])

    # -- sensor-loop side --------------------------------------------------
    def update(self, accel_g, gyro_dps, now):
        with self._lock:
            if not self._started:
                self.filter.q = quat_from_gravity(accel_g)
                self._started = True
            dt = 0.0 if self._last_t is None else now - self._last_t
            self._last_t = now
            if dt > GAP_S:
                self._gaps += 1
            if dt <= 0.0 or dt > GAP_S:
                dt = 0.0                        # first sample, or a stall

            if self._null_n < self._null_target:
                for i in range(3):
                    self._null_acc[i] += gyro_dps[i]
                self._null_n += 1
                if self._null_n == self._null_target:
                    self._bias = [v / self._null_n for v in self._null_acc]
                    self._still_since = None
            g = [gyro_dps[i] - self._bias[i] for i in range(3)]

            if AUTO_BIAS and self._null_n >= self._null_target:
                self._refine_bias(accel_g, gyro_dps, now)

            if dt > 0.0:
                self.filter.update_imu(*(math.radians(v) for v in g), *accel_g, dt)
            q = self.filter.q
            self._hist.append((now, q))
            while self._hist and self._hist[0][0] < now - HISTORY_S:
                self._hist.popleft()

            if self._want_ref and self._null_n >= self._null_target:
                self._q_ref, self._t_ref, self._want_ref = q, now, False
                self._gaps = 0
            return q

    def q_rel_at(self, t):
        """Orientation (board -> reference frame) at time ``t``, from the short
        history; ``None`` before a reference pose exists."""
        with self._lock:
            if self._q_ref is None or not self._hist:
                return None
            q = self._hist[0][1]
            for ts, qs in reversed(self._hist):
                if ts <= t:
                    q = qs
                    break
            return quat_norm(quat_mul(quat_conj(self._q_ref), q))

    # -- control side --------------------------------------------------------
    def set_reference(self):
        """Take the current pose as the reference (applied once the gyro null,
        if one is running, has finished)."""
        with self._lock:
            self._want_ref = True

    def null_gyro(self, n=GYRO_NULL_SAMPLES):
        with self._lock:
            self._null_acc = [0.0, 0.0, 0.0]
            self._null_n = 0
            self._null_target = int(n)
            self._blk = self._new_block()
            self._still_since = None

    def status(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            return {
                "have_ref": self._q_ref is not None,
                "ref_pending": self._want_ref,
                "ref_age_s": None if self._t_ref is None else round(now - self._t_ref, 1),
                "nulling_gyro": self._null_n < self._null_target,
                "orientation_gaps": self._gaps,
                "gyro_bias": [round(v, 3) for v in self._bias],
                "still": self._still_since is not None and now - self._still_since >= STILL_S,
            }


# --- Earth-field fit ---------------------------------------------------------------

def fit_ellipsoid(raws):
    """Axis-aligned ellipsoid through raw magnetometer vectors.  Returns
    ``(center, radii)`` or ``None`` if the fit is degenerate."""
    n = len(raws)
    mean = tuple(sum(r[i] for r in raws) / n for i in range(3))
    s0 = math.sqrt(sum(_dot(_sub(r, mean), _sub(r, mean)) for r in raws) / n) or 1.0
    AtA = [[0.0] * 6 for _ in range(6)]
    Atb = [0.0] * 6
    for r in raws:                              # a x^2 + b y^2 + c z^2 + d x + e y + f z = 1
        x, y, z = ((r[i] - mean[i]) / s0 for i in range(3))
        row = (x * x, y * y, z * z, x, y, z)
        for i in range(6):
            Atb[i] += row[i]
            for j in range(6):
                AtA[i][j] += row[i] * row[j]
    try:
        a, b, c, d, e, f = _solve(AtA, Atb)
    except ValueError:
        return None
    if min(a, b, c) <= 0.0:
        return None
    G = 1.0 + d * d / (4 * a) + e * e / (4 * b) + f * f / (4 * c)
    if G <= 0.0:
        return None
    center = (mean[0] - s0 * d / (2 * a), mean[1] - s0 * e / (2 * b), mean[2] - s0 * f / (2 * c))
    radii = tuple(s0 * math.sqrt(G / k) for k in (a, b, c))
    return center, radii


def gravity_bin(a):
    """Which of 26 directions around the board ``a`` falls in (cube faces,
    edges and corners).  The field and gravity are close together at most
    latitudes, so visiting them covers the field's sphere too."""
    n = _norm(a) or 1.0
    return tuple(0 if abs(v) / n < 0.38 else (1 if v > 0 else -1) for v in a)


def fit_iron(samples):
    """Hard/soft-iron fit from the tumble's ``(t, raw, accel, gyro, q)`` samples.
    Returns ``(cal, report)``.  Raises ValueError, with a human-readable reason,
    if the data cannot support a fit."""
    n = len(samples)
    if n < EARTH_MIN_SAMPLES:
        raise ValueError(f"only {n} samples; keep turning the board (need {EARTH_MIN_SAMPLES})")
    coverage = len({gravity_bin(s[2]) for s in samples}) / 26.0
    if coverage < EARTH_MIN_COVERAGE:
        raise ValueError(f"orientation coverage {coverage:.0%} is too low (need "
                         f"{EARTH_MIN_COVERAGE:.0%}); point every face and corner of the board up")
    raws = [s[1] for s in samples]
    ell = fit_ellipsoid(raws)
    if ell is None:
        raise ValueError("the data doesn't form an ellipsoid; turn the board through more orientations")
    center, radii = ell
    radius = sum(radii) / 3.0
    scale = tuple(radius / r for r in radii)
    if not all(1.0 / SOFT_IRON_MAX <= v <= SOFT_IRON_MAX for v in scale):
        raise ValueError(f"implausible soft-iron scales {[round(v, 2) for v in scale]}; "
                         "turn the board through more orientations, away from steel and the magnet")
    cal = MagCalibration(center, scale, remap=True)

    us = [cal.apply(r) for r in raws]
    radial = math.sqrt(sum((_norm(u) - radius) ** 2 for u in us) / n)
    if radial > EARTH_MAX_RADIAL_UT:
        raise ValueError(f"the field's length varies by {radial:.1f} uT rms; the field isn't uniform "
                         "here -- tumble in place, away from the magnet and steel")

    verts = [_dot(u, _scale(s[2], 1.0 / (_norm(s[2]) or 1.0))) for u, s in zip(us, samples)]
    vert = sum(verts) / n
    vert_rms = math.sqrt(sum((v - vert) ** 2 for v in verts) / n)
    if vert_rms > EARTH_MAX_VERT_UT:
        raise ValueError(f"the field along gravity varies by {vert_rms:.1f} uT rms; turn the board "
                         "more slowly (if it persists, the magnetometer axis map is wrong)")
    return cal, {
        "residual_ut": round(radial, 3), "vertical_rms_ut": round(vert_rms, 3),
        "vertical_ut": round(vert, 2), "radius_ut": round(radius, 2),
        "gyro_resid_ut": gyro_consistency(us, samples), "n": n, "coverage": round(coverage, 3),
    }


def gyro_consistency(us, samples):
    """Rms disagreement (uT) between corrected tumble samples rotated by the
    gyro-tracked orientation: about the noise if orientation held up, as large
    as the field if it was lost.  Informational; the fit doesn't use it."""
    rot = [quat_rotate(s[4], u) for u, s in zip(us, samples)]
    n = len(rot)
    mean = tuple(sum(v[i] for v in rot) / n for i in range(3))
    return round(math.sqrt(sum(_dot(_sub(v, mean), _sub(v, mean)) for v in rot) / n), 2)


class EarthField:
    """The magnetometer calibration plus Earth's field in the reference frame."""

    def __init__(self, cal, earth, report):
        self.cal = cal                          # MagCalibration (raw -> body)
        self.earth = tuple(earth)               # uT, reference frame
        self.report = dict(report)

    def subtract(self, raw, q_rel):
        """Electromagnet field in the reference frame from one raw sample."""
        return _sub(quat_rotate(q_rel, self.cal.apply(raw)), self.earth)

    def as_dict(self):
        e = self.earth
        return {
            **self.cal.as_dict(), **self.report,
            "method": "mark",
            "earth_ref_ut": [round(v, 3) for v in e],
            "earth_ut": round(_norm(e), 2),
            "dip_deg": round(math.degrees(math.atan2(-e[2], math.hypot(e[0], e[1]))), 1),
        }

    def save(self, path):
        Path(path).write_text(json.dumps(self.as_dict(), indent=2))

    @classmethod
    def load(cls, path):
        """``None`` if missing, unreadable, or from the old gyro-tracked fit."""
        p = Path(path)
        if not p.exists():
            return None
        try:
            d = json.loads(p.read_text())
            if d.get("method") != "mark":
                return None
            return cls(MagCalibration.from_dict(d), tuple(map(float, d["earth_ref_ut"])), d)
        except (ValueError, KeyError, TypeError, OSError):
            return None


class EarthCollector:
    """Accumulates tumble samples; tracks coverage so it can be shown live."""

    def __init__(self):
        self.samples = []
        self._bins = set()

    def add(self, t, raw, accel, gyro, q_rel):
        self.samples.append((t, tuple(raw), tuple(accel), tuple(gyro), tuple(q_rel)))
        self._bins.add(gravity_bin(accel))

    @property
    def n(self):
        return len(self.samples)

    def coverage(self):
        return len(self._bins) / 26.0

    def save_csv(self, path):
        """Keep the raw session: failed fits can be diagnosed offline."""
        with open(path, "w") as f:
            f.write("t,mx,my,mz,ax,ay,az,gx,gy,gz,qw,qx,qy,qz\n")
            for t, raw, accel, gyro, q in self.samples:
                f.write(",".join(f"{v:.5f}" for v in (t, *raw, *accel, *gyro, *q)) + "\n")

    def result(self):
        return fit_iron(self.samples)
