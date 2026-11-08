"""Pulsed-electromagnet localization: the math, no I/O.

The electromagnet is switched through a repeating pattern of states
(``+1`` = forward current, ``0`` = off, ``-1`` = reversed).  For every phase
the sensor loop hands :class:`PhaseDemod` the *average* magnetometer vector
measured while the coil sat in that state.  Over a short sliding window of
phases each axis is fitted as

    reading_k = E + D * (t_k - t_c) + s_k * B_em

``E`` is everything that does not follow the coil -- Earth's field, hard-iron
offset, nearby steel -- ``D`` is a linear drift (slow rotation, temperature),
and ``B_em`` is the electromagnet's field at ``+I``.  Because E and D are fitted
out every few hundred milliseconds, Earth's field is cancelled no matter how
large it is, and no orientation estimate is needed for the subtraction.

:class:`Localizer` then turns ``B_em`` into a position.  Far enough out (about 3x
the coil's size) the electromagnet is a magnetic dipole with moment vector
``m``:

    B(p) = ( 3 (m . p) p - m r^2 ) / r^5          r = |p|

``m`` (direction and strength together) is learned by holding the sensor at one
or more *known* positions -- a linear least-squares fit, see :func:`fit_moment`.
Given ``m``, a measured ``B`` inverts in closed form to two mirror-image
candidates ``p`` and ``-p`` (a dipole's field is identical at both); the one
closest to the previous position is kept, so the track stays continuous.

Units: centimetres and microtesla throughout, so ``m`` is in uT*cm^3.
Stdlib only, plain tuples of floats -- same as the rest of the tooling.
"""

from __future__ import annotations

import json
import math
import threading
from collections import deque
from pathlib import Path


# --- tuning (adjust freely) -------------------------------------------------

WINDOW_PHASES = 4        # phases per B_em estimate (>= 2; >= 3 enables drift fit)
FIT_DRIFT = True         # also fit a linear drift term inside each window
B_MIN_UT = 0.5           # below this |B_em| there is no usable position fix
SIGNAL_OK_UT = 1.5       # |B_em| at/above this is labelled "ok"
SIGNAL_GOOD_UT = 5.0     # |B_em| at/above this is labelled "good"
NEAR_FIELD_CM = 5.0      # inside this the point-dipole model is unreliable
EMA_ALPHA = 0.4          # position smoothing (1 = raw, smaller = smoother)
JITTER_WINDOW = 20       # estimates used for the B_em jitter (noise) readout
CAL_ESTIMATES = 20       # B_em estimates averaged per calibration point
CAL_SKIP = WINDOW_PHASES # estimates dropped first, so no pre-click phase leaks in
START_HINT = None        # (x, y, z) side to assume at start; None = last cal point


# --- small vector helpers (tuples of 3 floats) ----------------------------

def _add(a, b):   return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def _sub(a, b):   return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def _scale(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def _dot(a, b):   return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def _norm(a):     return math.sqrt(_dot(a, a))
def _dist(a, b):  return _norm(_sub(a, b))


def _unit(a):
    n = _norm(a)
    return (a[0] / n, a[1] / n, a[2] / n) if n > 1e-12 else (0.0, 0.0, 0.0)


def quat_rotate(q, v):
    """Rotate ``v`` by unit quaternion ``q = (w, x, y, z)`` (body -> world in
    the AHRS convention)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def _solve(A, b):
    """Solve the small dense system ``A x = b`` (Gauss-Jordan, partial pivot).
    ``A`` is a list of row lists, ``b`` a list.  Raises ValueError if singular."""
    n = len(b)
    M = [list(map(float, A[i])) + [float(b[i])] for i in range(n)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[piv][c]) < 1e-12:
            raise ValueError("singular system")
        M[c], M[piv] = M[piv], M[c]
        for r in range(n):
            if r != c:
                f = M[r][c] / M[c][c]
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    return [M[i][n] / M[i][i] for i in range(n)]


# --- the dipole model ------------------------------------------------------

def dipole_matrix(p):
    """3x3 matrix ``G(p)`` with ``B = G(p) m``:  G = (3 p_hat p_hat^T - I) / r^3."""
    r = _norm(p)
    u = _unit(p)
    r3 = r ** 3
    return [[(3.0 * u[i] * u[j] - (1.0 if i == j else 0.0)) / r3 for j in range(3)]
            for i in range(3)]


def dipole_field(m, p):
    """Field (uT) of moment ``m`` (uT*cm^3) at offset ``p`` (cm) from it."""
    G = dipole_matrix(p)
    return tuple(_dot(G[i], m) for i in range(3))


def solve_position(B, m):
    """Invert the dipole field: return the candidate ``p`` with ``p . m >= 0``
    (the other is ``-p``), or ``None`` if ``|B|`` is below :data:`B_MIN_UT`."""
    b = _norm(B)
    k = _norm(m)
    if b < B_MIN_UT or k <= 0.0:
        return None
    u = _unit(m)

    beta = _dot(B, u) / b                 # cos of angle(B, moment axis)
    rho = beta * beta
    disc = math.sqrt(rho * (rho + 8.0))
    # with c = cos^2(theta) between p and the axis:
    #   9 c^2 - (6 + 3 rho) c + (1 - rho) = 0,
    # and sign(3c - 1) must equal sign(beta), which picks the root.
    c2 = (2.0 + rho + disc) / 6.0 if beta >= 0.0 else (2.0 + rho - disc) / 6.0
    c2 = min(1.0, max(0.0, c2))

    A = b / math.sqrt(1.0 + 3.0 * c2)     # = k / r^3
    r = (k / A) ** (1.0 / 3.0)
    cos_t = math.sqrt(c2)
    sin_t = math.sqrt(max(0.0, 1.0 - c2))

    # lateral direction = part of B perpendicular to the axis (undefined
    # exactly on-axis, where the answer is purely axial anyway)
    B_perp = _sub(B, _scale(u, _dot(B, u)))
    s_hat = _unit(B_perp)
    p_hat = _add(_scale(u, cos_t), _scale(s_hat, sin_t))
    return _scale(p_hat, r)


def fit_moment(points):
    """Least-squares moment from known positions.

    ``points`` is a list of ``(p_cm, B_uT)`` pairs.  Since ``B_i = G(p_i) m`` is
    linear in ``m``, the fit is ``m = (sum G^T G)^-1 sum G^T B``.  One point is
    already enough; more points average out noise and placement error.

    Returns ``(m, report)`` where ``report`` has, per point, the field residual
    (uT) and how far the solver would place the sensor from where it really was
    (cm) -- the honest measure of how well the dipole model fits your setup.
    """
    if not points:
        raise ValueError("no calibration points")
    AtA = [[0.0] * 3 for _ in range(3)]
    Atb = [0.0] * 3
    for p, B in points:
        if _norm(p) < 1e-6:
            raise ValueError("a calibration point cannot be at the magnet itself")
        G = dipole_matrix(p)      # symmetric
        for i in range(3):
            for j in range(3):
                AtA[i][j] += sum(G[k][i] * G[k][j] for k in range(3))
            Atb[i] += sum(G[k][i] * B[k] for k in range(3))
    m = tuple(_solve(AtA, Atb))

    report = []
    for p, B in points:
        pred = dipole_field(m, p)
        sol = solve_position(B, m)
        if sol is None:
            pos_err = None
        else:
            pos_err = min(_dist(sol, p), _dist(_scale(sol, -1.0), p))
        report.append({
            "p": [round(v, 2) for v in p],
            "b": [round(v, 3) for v in B],
            "b_resid_ut": round(_dist(pred, B), 3),
            "pos_err_cm": None if pos_err is None else round(pos_err, 2),
        })
    return m, report


# --- phase demodulation ------------------------------------------------------

class PhaseDemod:
    """Sliding-window lock-in: per-phase averages in, ``B_em`` estimates out."""

    def __init__(self, window=WINDOW_PHASES, fit_drift=FIT_DRIFT):
        self.window = max(2, int(window))
        self.fit_drift = bool(fit_drift) and self.window >= 3
        self._phases = deque(maxlen=self.window)

    def reset(self):
        self._phases.clear()

    def push(self, state, t_mid, mean, n):
        """Add one finished phase.  ``mean`` is the average field vector over
        the phase's accepted samples (``n`` of them), ``t_mid`` their mean time.
        Returns ``{"b_em", "earth"}`` or ``None`` if the window cannot be solved
        yet (too few valid phases, or no on/off contrast in it)."""
        if n <= 0 or mean is None:
            self._phases.clear()          # a gap breaks the drift model
            return None
        self._phases.append((float(state), float(t_mid), tuple(mean)))
        if len(self._phases) < self.window:
            return None

        t_c = sum(ph[1] for ph in self._phases) / len(self._phases)
        rows = []
        for s, t, _ in self._phases:
            row = [1.0, s]
            if self.fit_drift:
                row.append(t - t_c)
            rows.append(row)
        k = len(rows[0])
        AtA = [[sum(r[i] * r[j] for r in rows) for j in range(k)] for i in range(k)]
        coef = []
        try:
            for axis in range(3):
                Atb = [sum(r[i] * ph[2][axis] for r, ph in zip(rows, self._phases))
                       for i in range(k)]
                coef.append(_solve(AtA, Atb))
        except ValueError:
            return None                    # e.g. a window with no "on" phase
        return {
            "b_em": tuple(c[1] for c in coef),
            "earth": tuple(c[0] for c in coef),
        }


# --- persisted calibration ---------------------------------------------------

class EMCalibration:
    """Known-position calibration points, and the moment fitted from them."""

    def __init__(self, points=None, frame="fixed"):
        self.points = list(points or [])      # [(p, B), ...]
        self.frame = frame
        self.m = None
        self.report = []
        if self.points:
            self.refit()

    def refit(self):
        if self.points:
            self.m, self.report = fit_moment(self.points)
        else:
            self.m, self.report = None, []

    def as_dict(self):
        return {
            "frame": self.frame,
            "moment": None if self.m is None else [round(v, 3) for v in self.m],
            "points": [{"p": list(p), "b": list(B)} for p, B in self.points],
        }

    def save(self, path):
        Path(path).write_text(json.dumps(self.as_dict(), indent=2))

    @classmethod
    def load(cls, path):
        p = Path(path)
        if not p.exists():
            return None
        try:
            d = json.loads(p.read_text())
            pts = [(tuple(map(float, q["p"])), tuple(map(float, q["b"])))
                   for q in d.get("points", [])]
            return cls(pts, d.get("frame", "fixed"))
        except (ValueError, KeyError, TypeError, OSError):
            return None


# --- the localizer (thread-safe) --------------------------------------------

def _signal(b):
    if b < B_MIN_UT:       return "none"
    if b < SIGNAL_OK_UT:   return "weak"
    if b < SIGNAL_GOOD_UT: return "ok"
    return "good"


class Localizer:
    """Feed :meth:`process` one ``B_em`` estimate (in the localization frame)
    at a time; read back the position.  Control methods are safe to call from
    other threads (HTTP handlers) while the sensor loop runs."""

    def __init__(self, cal_path="em_cal.json", frame="fixed"):
        self._lock = threading.Lock()
        self.cal_path = str(cal_path)
        self.frame = frame
        self.cal = EMCalibration.load(self.cal_path) or EMCalibration(frame=frame)
        self._p_smooth = None
        self._hint = None
        self._cap_p = None                # known position being captured
        self._cap = []
        self._cap_skip = 0
        self._jitter = deque(maxlen=JITTER_WINDOW)
        self._last_b = None
        self.last_error = None

    # -- sensor-loop side ------------------------------------------------
    def process(self, b_em):
        out = {"b_em": None, "b_mag": None, "signal": "none", "pos": None,
               "range_cm": None, "near_field": False, "jitter_ut": None,
               "state": "need-cal"}
        with self._lock:
            if b_em is None:
                out["state"] = "no-estimate"
                return out
            b = _norm(b_em)
            self._last_b = b_em
            self._jitter.append(b_em)
            out["b_em"] = [round(v, 3) for v in b_em]
            out["b_mag"] = round(b, 3)
            out["signal"] = _signal(b)
            out["jitter_ut"] = self._jitter_ut()

            if self._cap_p is not None:
                if self._cap_skip > 0:
                    self._cap_skip -= 1
                else:
                    self._cap.append(b_em)
                if len(self._cap) >= CAL_ESTIMATES:
                    self._finish_capture()
                out["state"] = "capturing"
                return out

            if self.cal.m is None:
                return out
            out["state"] = "ready"

            cand = solve_position(b_em, self.cal.m)
            if cand is None:
                return out
            cand_neg = _scale(cand, -1.0)
            ref = self._p_smooth if self._p_smooth is not None else self._start_ref()
            if ref is not None and _dist(cand_neg, ref) < _dist(cand, ref):
                cand = cand_neg

            if self._p_smooth is None:
                self._p_smooth = cand
            else:
                a = EMA_ALPHA
                self._p_smooth = tuple(a * cand[i] + (1.0 - a) * self._p_smooth[i]
                                       for i in range(3))
            p = self._p_smooth
            out["pos"] = [round(v, 2) for v in p]
            out["range_cm"] = round(_norm(p), 2)
            out["near_field"] = _norm(p) < NEAR_FIELD_CM
            return out

    def _start_ref(self):
        if self._hint is not None:
            return self._hint
        if START_HINT is not None:
            return tuple(START_HINT)
        if self.cal.points:
            return self.cal.points[-1][0]
        return None

    def _jitter_ut(self):
        n = len(self._jitter)
        if n < 3:
            return None
        mean = _scale(tuple(sum(v[i] for v in self._jitter) for i in range(3)), 1.0 / n)
        var = sum(_dot(_sub(v, mean), _sub(v, mean)) for v in self._jitter) / (n - 1)
        return round(math.sqrt(var), 3)

    def _finish_capture(self):
        n = len(self._cap)
        B = tuple(sum(v[i] for v in self._cap) / n for i in range(3))
        p = self._cap_p
        self._cap_p, self._cap = None, []
        self.cal.points.append((p, B))
        self.cal.frame = self.frame
        try:
            self.cal.refit()
            self.last_error = None
        except ValueError as e:
            self.cal.points.pop()
            self.cal.refit()
            self.last_error = f"calibration fit failed: {e}"
            return
        self._p_smooth = None
        self._hint = None
        self.cal.save(self.cal_path)

    # -- control side (other threads) -----------------------------------
    def capture_point(self, p):
        try:
            p = tuple(float(v) for v in p)
        except (TypeError, ValueError):
            raise ValueError("position must be three numbers (cm)")
        if len(p) != 3:
            raise ValueError("position must be three numbers (cm)")
        if _norm(p) < NEAR_FIELD_CM:
            raise ValueError(f"calibrate at least {NEAR_FIELD_CM:g} cm from the magnet")
        with self._lock:
            self._cap_p, self._cap, self._cap_skip = p, [], CAL_SKIP
            self.last_error = None

    def cancel_capture(self):
        with self._lock:
            self._cap_p, self._cap = None, []

    def undo_point(self):
        with self._lock:
            if self.cal.points:
                self.cal.points.pop()
                self.cal.refit()
                self.cal.save(self.cal_path)
            self._p_smooth = None

    def clear_points(self):
        with self._lock:
            self.cal = EMCalibration(frame=self.frame)
            self.cal.save(self.cal_path)
            self._p_smooth = None
            self._hint = None

    def flip(self):
        """Jump to the mirror-image solution (use if the track starts out
        reflected through the magnet)."""
        with self._lock:
            if self._p_smooth is not None:
                self._hint = _scale(self._p_smooth, -1.0)
            else:
                ref = self._start_ref()
                self._hint = _scale(ref, -1.0) if ref is not None else None
            self._p_smooth = None

    def reset_track(self):
        with self._lock:
            self._p_smooth = None
            self._jitter.clear()

    def status(self):
        with self._lock:
            m = self.cal.m
            return {
                "have_cal": m is not None,
                "cal_frame": self.cal.frame,
                "frame_mismatch": bool(self.cal.points) and self.cal.frame != self.frame,
                "moment": None if m is None else [round(v, 1) for v in m],
                "moment_k": None if m is None else round(_norm(m), 1),
                "cal_points": self.cal.report,
                "capturing": self._cap_p is not None,
                "capture_n": len(self._cap),
                "capture_target": CAL_ESTIMATES,
                "cal_error": self.last_error,
            }
