"""Turns the coil's field into a position.  Knows nothing about I2C or the web.

Positions are in cm from the coil's center, which is 0, in the home axes: the
ICM's own x, y, z while it sat at home.  *Home* is where the ICM sits on its tape
mark, given by the user as a position from the coil, e.g. (15, 0, 0).

With the coil always on, Earth is captured at home, so the capture also holds the
coil's field there.  The ICM's error is then

    error = B(p) - B(home)

The coil is two iron cores wound as one bundle, about 5 cm long, and the board
works within 15 cm of it.  That's too close to treat the coil as a point, so it's
modelled as a bar magnet: a north pole and a south pole, one at each end.

    B(p) = q (p - a) / |p - a|^3  -  q (p - b) / |p - b|^3
    a, b = center +- half_length * axis

``fit_coil`` finds axis, strength and half length from points captured at known
positions.  The model's center starts at 0 but may move up to
``LOCATE_CENTER_SLACK_CM``, to absorb a small mistake in measuring home and the
model's own roughness.  How far it moved is a check on the home measurement.
``solve`` then turns each error into a position, starting from the last one.  A
bar magnet's field is the same at ``p`` and at its mirror through the center, so
the track's continuity picks the side.

Units: cm and uT.  ``s`` is strength times axis, in uT*cm^2.

Used by: tracking/pair.py (Locator.update), web/server.py (point and home routes), earth_cancel.py.  Uses: geometry, config.
"""

import json
import math
import threading
from pathlib import Path

from magtrack_src.config import (LOCATE_CENTER_GRID_CM, LOCATE_CENTER_SLACK_CM, LOCATE_MAX_MISMATCH_UT,
                                 LOCATE_MIN_POINTS, LOCATE_POINT_SAMPLES, LOCATE_POINT_MAX_SPREAD_UT,
                                 LOCATE_RANGE_CM)
from magtrack_src.geometry.vectors import add, dot, norm, sub

ORIGIN = (0.0, 0.0, 0.0)  # the coil's center, as measured by the user
MIN_HALF_LENGTH_CM = 0.2
MAX_HALF_LENGTH_CM = 6.0
KEEP_CLEAR_CM = 3.0       # the coil's center can't be this close to a point or home: the coil is there


def _scale(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


class Coil:
    def __init__(self, center, s, half_length):
        self.center = tuple(center)
        self.s = tuple(s)
        self.half_length = half_length

    @classmethod
    def from_params(cls, x, slack=None):
        """``slack``: keep the center within this many cm of 0."""
        center = tuple(x[0:3])
        r = norm(center)
        if slack is not None and r > slack:
            center = _scale(center, slack / r)
        return cls(center, x[3:6], min(MAX_HALF_LENGTH_CM, max(MIN_HALF_LENGTH_CM, x[6])))

    def params(self):
        return [*self.center, *self.s, self.half_length]

    @property
    def strength(self):
        return norm(self.s)

    @property
    def axis(self):
        q = self.strength
        return (1.0, 0.0, 0.0) if q == 0.0 else _scale(self.s, 1.0 / q)

    def field(self, p):
        q = self.strength
        if q == 0.0:
            return (0.0, 0.0, 0.0)
        tip = _scale(self.s, self.half_length / q)
        out = (0.0, 0.0, 0.0)
        for pole, sign in ((add(self.center, tip), q), (sub(self.center, tip), -q)):
            d = sub(p, pole)
            r = norm(d)
            if r < 1e-6:
                return (math.inf, math.inf, math.inf)
            out = add(out, _scale(d, sign / r ** 3))
        return out

    def mirror(self, p):
        """The other position with the same field."""
        return sub(_scale(self.center, 2.0), p)

    def as_dict(self):
        return {"center": list(self.center), "s": list(self.s), "half_length": self.half_length}

    @classmethod
    def from_dict(cls, d):
        return cls([float(v) for v in d["center"]], [float(v) for v in d["s"]],
                   float(d["half_length"]))


# -- least squares ------------------------------------------------------------

def _solve_linear(a, b):
    """``a x = b`` by Gaussian elimination; ``None`` if ``a`` is singular."""
    n = len(b)
    m = [list(row) + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            return None
        m[col], m[piv] = m[piv], m[col]
        for r in range(col + 1, n):
            k = m[r][col] / m[col][col]
            if k:
                for c in range(col, n + 1):
                    m[r][c] -= k * m[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))) / m[r][r]
    return x


def _cost(r):
    return sum(v * v for v in r)


def least_squares(residuals, x0, iterations=100, steps=None):
    """Levenberg-Marquardt with a numeric Jacobian.  Returns ``(x, cost)``."""
    x = list(x0)
    r = residuals(x)
    cost = _cost(r)
    lam = 1e-3
    steps = steps or [1e-4 * max(1.0, abs(v)) for v in x]
    for _ in range(iterations):
        if not math.isfinite(cost):
            break
        cols = []
        for j, h in enumerate(steps):
            xj = list(x)
            xj[j] += h
            cols.append([(a - b) / h for a, b in zip(residuals(xj), r)])
        n = len(x)
        jtj = [[sum(a * b for a, b in zip(cols[i], cols[k])) for k in range(n)] for i in range(n)]
        jtr = [sum(a * b for a, b in zip(cols[i], r)) for i in range(n)]
        while True:
            a = [[jtj[i][k] + (lam * (jtj[i][i] + 1e-9) if i == k else 0.0) for k in range(n)]
                 for i in range(n)]
            dx = _solve_linear(a, [-v for v in jtr])
            if dx is not None:
                xn = [v + d for v, d in zip(x, dx)]
                rn = residuals(xn)
                cn = _cost(rn)
                if cn < cost:
                    break
            lam *= 4.0
            if lam > 1e10:
                return x, cost
        done = cost - cn < 1e-12 * (1.0 + cost)
        x, r, cost = xn, rn, cn
        lam = max(lam / 3.0, 1e-12)
        if done:
            break
    return x, cost


# -- fitting the coil -----------------------------------------------------------

def _dipole_matrix(d):
    """``B = M m`` for a point magnet of moment ``m`` seen from ``d``."""
    r2 = dot(d, d)
    r5 = r2 * r2 * math.sqrt(r2)
    r3 = r2 * math.sqrt(r2)
    return [[3.0 * d[i] * d[k] / r5 - (1.0 / r3 if i == k else 0.0) for k in range(3)]
            for i in range(3)]


def _coarse_search(points, home, slack_cm, grid_cm):
    """Best few coil centers on a grid within ``slack_cm`` of 0, treating the coil
    as a point: for a given center the field is linear in the magnet's moment, so
    each guess is quick."""
    clear = [home] + [p for p, _ in points]
    n = int(slack_cm // grid_cm)
    scored = []
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            for k in range(-n, n + 1):
                c = (i * grid_cm, j * grid_cm, k * grid_cm)
                if norm(c) > slack_cm + 1e-9 or any(norm(sub(c, p)) < KEEP_CLEAR_CM for p in clear):
                    continue
                at_home = _dipole_matrix(sub(home, c))
                rows, rhs = [], []
                for p, e in points:
                    m = _dipole_matrix(sub(p, c))
                    rows += [[m[a][b] - at_home[a][b] for b in range(3)] for a in range(3)]
                    rhs += list(e)
                ata = [[sum(row[a] * row[b] for row in rows) for b in range(3)] for a in range(3)]
                atb = [sum(row[a] * v for row, v in zip(rows, rhs)) for a in range(3)]
                moment = _solve_linear(ata, atb)
                if moment is None:
                    continue
                miss = sum((sum(row[b] * moment[b] for b in range(3)) - v) ** 2
                           for row, v in zip(rows, rhs))
                scored.append((miss, c, moment))
    scored.sort(key=lambda t: t[0])
    picked = []
    for miss, c, moment in scored:      # keep guesses apart, so they explore
        if all(norm(sub(c, other)) > 1.5 * grid_cm for _, other, _ in picked):
            picked.append((miss, c, moment))
        if len(picked) == 5:
            break
    return picked


def _residuals(points, home, slack_cm):
    def f(x):
        coil = Coil.from_params(x, slack_cm)
        at_home = coil.field(home)
        out = []
        for p, e in points:
            out.extend(sub(sub(coil.field(p), at_home), e))
        return out
    return f


def _refine(points, home, x0, slack_cm=LOCATE_CENTER_SLACK_CM, iterations=200):
    x, cost = least_squares(_residuals(points, home, slack_cm), x0, iterations,
                            steps=[1e-3] * 3 + [1e-3 * max(1.0, max(abs(v) for v in x0[3:6]))] * 3 + [1e-3])
    return Coil.from_params(x, slack_cm), cost


def fit_coil(points, home, slack_cm=LOCATE_CENTER_SLACK_CM, grid_cm=LOCATE_CENTER_GRID_CM):
    """The coil that best explains ``points``: ``[(position, error), ...]``,
    positions in cm from the coil's center, errors in uT, both in the home axes.
    ``home`` is where Earth was captured, in cm from the coil's center."""
    if len(points) < LOCATE_MIN_POINTS:
        raise ValueError(f"need at least {LOCATE_MIN_POINTS} points, have {len(points)}")
    best = None
    for _, c, moment in _coarse_search(points, home, slack_cm, grid_cm):
        half = 2.0
        coil, cost = _refine(points, home, [*c, *(v / (2.0 * half) for v in moment), half], slack_cm)
        if best is None or cost < best[1]:
            best = (coil, cost)
    if best is None:
        raise ValueError("couldn't place the coil: spread the points out more, and keep them 3 cm clear of it")
    return best[0]


# -- solving for position ------------------------------------------------------

def _seeds(coil, target):
    """Places to try when the last position doesn't lead anywhere: around the coil
    at the distance a point magnet of its strength would give this field."""
    moment = coil.strength * 2.0 * coil.half_length
    r = (moment / max(norm(target), 1e-3)) ** (1.0 / 3.0)
    dirs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    dirs += [(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    return [add(coil.center, _scale(d, r / norm(d))) for d in dirs]


def _settle(coil, target, start):
    x, cost = least_squares(lambda p: list(sub(coil.field(p), target)), start, 30,
                            steps=[1e-4] * 3)
    return tuple(x), math.sqrt(cost)


def solve(coil, error, start, home_field):
    """``(position, mismatch_ut)`` for one error vector, searching from ``start``.
    ``home_field`` is the coil's field at home.  ``mismatch_ut`` is how far the
    coil's field at the answer is from what was measured."""
    target = add(error, home_field)
    p, miss = _settle(coil, target, start)
    if miss > LOCATE_MAX_MISMATCH_UT or norm(sub(p, coil.center)) > 3 * LOCATE_RANGE_CM:
        tries = [_settle(coil, target, s) for s in _seeds(coil, target)]
        p, miss = min(tries, key=lambda t: t[1])
    other = coil.mirror(p)            # same field: keep the side nearer where we were
    if norm(sub(other, start)) < norm(sub(p, start)):
        p = other
    return p, miss


def mm_per_ut(coil, p):
    """How far off a position can be, in mm, for each uT of field error, in the
    worst direction.  The coil's field is the same all round its long axis, so
    beside the coil's middle this grows without limit: moving round the coil
    there barely changes the field."""
    j = []
    for k in range(3):
        d = [0.0, 0.0, 0.0]
        d[k] = 1e-3
        j.append(_scale(sub(coil.field(add(p, d)), coil.field(sub(p, d))), 1.0 / 2e-3))
    a = [[sum(j[k][r] * j[k][c] for k in range(3)) for c in range(3)] for r in range(3)]
    # smallest eigenvalue of the symmetric 3x3 a, in closed form
    off = a[0][1] ** 2 + a[0][2] ** 2 + a[1][2] ** 2
    mean = (a[0][0] + a[1][1] + a[2][2]) / 3.0
    spread = math.sqrt((sum((a[i][i] - mean) ** 2 for i in range(3)) + 2.0 * off) / 6.0)
    if spread == 0.0:
        smallest = mean
    else:
        b = [[(a[r][c] - (mean if r == c else 0.0)) / spread for c in range(3)] for r in range(3)]
        det = (b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1])
               - b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0])
               + b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0]))
        phi = math.acos(max(-1.0, min(1.0, det / 2.0))) / 3.0
        smallest = mean + 2.0 * spread * math.cos(phi + 2.0 * math.pi / 3.0)
    return 10.0 / math.sqrt(max(smallest, 1e-12))


def check_points(coil, points, home):
    """How far each point solves from where it was said to be, in mm."""
    at_home = coil.field(home)
    return [round(10.0 * norm(sub(solve(coil, e, p, at_home)[0], p)), 1) for p, e in points]


def left_out_check(coil, points, home):
    """The honest check: each point refitted without it, then solved.  ``None``
    for each when there are too few points to leave one out."""
    if len(points) <= LOCATE_MIN_POINTS:
        return [None] * len(points)
    out = []
    for i, (p, e) in enumerate(points):
        rest = points[:i] + points[i + 1:]
        refit, _ = _refine(rest, home, coil.params(), iterations=60)
        out.append(round(10.0 * norm(sub(solve(refit, e, p, refit.field(home))[0], p)), 1))
    return out


# -- live tracking ------------------------------------------------------------

def _vector(position, what):
    p = tuple(float(v) for v in position)
    if len(p) != 3 or not all(math.isfinite(v) for v in p):
        raise ValueError(f"enter {what}'s x, y and z in cm")
    return p


class Locator:
    """Fed the ICM's samples in the sensor loop; the other methods are for the
    HTTP threads.  Fitting runs on its own thread, so the loop never waits."""

    def __init__(self, path=None):
        self._lock = threading.Lock()
        self._save_lock = threading.Lock()   # one save at a time, so the newest state lands last
        self._path = Path(path) if path else None
        self.home = None              # where the ICM sits at home, cm from the coil's center
        self.points = []              # [(position, error, spread_ut)], cm from the coil's center
        self.coil = None
        self.checks = []              # per point: (mm off, mm off when left out)
        self.message = None
        self.fitting = False
        self._from_home = False       # points loaded from an old file, measured from home
        self._old_coil = None         # that file's coil, measured from home
        self._home_field = None
        self._capture = None          # (position, [errors])
        self._last = None
        self._earth = None
        self._load()

    # -- sensor loop -----------------------------------------------------------
    def update(self, sample):
        """Adds ``locate`` to an ICM view sample: position in cm from the coil's
        center, or ``None`` with a reason."""
        error = sample.get("error")
        with self._lock:
            if error is None:
                return {"position": None, "why": "no capture at sensor home yet"}
            if self.home is None:
                return {"position": None, "why": "sensor home not set"}
            if sample.get("predicted") != self._earth:      # captured again: back at home
                self._earth = sample.get("predicted")
                self._last = self.home
            if self._capture is not None:
                self._capture_step(error)
            coil, start, home, home_field = self.coil, self._last or self.home, self.home, self._home_field
        if coil is None:
            return {"position": None, "why": "not calibrated"}
        p, miss = solve(coil, error, start, home_field)
        if miss > LOCATE_MAX_MISMATCH_UT:
            return {"position": None, "why": f"no place fits this field (off by {miss:.1f} µT)"}
        with self._lock:
            self._last = p
        return {"position": [round(v, 2) for v in p],
                "from_coil_cm": round(norm(p), 1),
                "from_home_cm": round(norm(sub(p, home)), 1),
                "mismatch_ut": round(miss, 2),
                "mm_per_ut": round(min(mm_per_ut(coil, p), 999.0), 1)}

    def _capture_step(self, error):
        p, errors = self._capture
        errors.append(error)
        if len(errors) < LOCATE_POINT_SAMPLES:
            return
        self._capture = None
        n = len(errors)
        mean = tuple(sum(e[i] for e in errors) / n for i in range(3))
        spread = max(norm(sub(e, mean)) for e in errors)
        if spread > LOCATE_POINT_MAX_SPREAD_UT:
            self.message = (f"Point not kept: the reading moved {spread:.1f} µT while capturing. "
                            "Hold the board still and try again.")
            return
        self.points.append((p, mean, round(spread, 2)))
        self.message = None
        self._start_fit()

    # -- control side ----------------------------------------------------------
    def set_home(self, position):
        """Where the ICM sits on its home mark, in cm from the coil's center.
        Points already measured from the coil keep their positions; points from an
        old file, measured from home, are moved to be measured from the coil."""
        h = _vector(position, "the sensor home")
        if norm(h) < KEEP_CLEAR_CM:
            raise ValueError(f"the sensor home can't be within {KEEP_CLEAR_CM:.0f} cm of the coil's center")
        with self._lock:
            if self._capture is not None:
                raise ValueError("wait for the point capture to finish")
            if self._from_home:
                self.points = [(add(p, h), e, s) for p, e, s in self.points]
                self._from_home, self._old_coil = False, None
            self.home = h
            self._last = h
            self.message = None
            self._start_fit()

    def capture_point(self, position):
        p = _vector(position, "the point")
        with self._lock:
            if self.home is None:
                raise ValueError("set the sensor home first")
            if self._capture is not None:
                raise ValueError("already capturing a point")
            self._capture = (p, [])
            self.message = None

    def cancel_capture(self):
        with self._lock:
            self._capture = None

    def remove_point(self, index):
        with self._lock:
            if not 0 <= index < len(self.points):
                raise ValueError("no such point")
            del self.points[index]
            self._start_fit()

    def clear_points(self):
        with self._lock:
            self.points = []
            self.coil = None
            self._home_field = None
            self.checks = []
            self.message = None
            self._from_home, self._old_coil = False, None
        self._save()

    def _start_fit(self):
        """Called with the lock held."""
        points = [(p, e) for p, e, _ in self.points]
        if len(points) < LOCATE_MIN_POINTS or self.home is None:
            self.coil, self._home_field, self.checks, self.fitting = None, None, [], False
            self._save_later()
            return
        self.fitting = True
        threading.Thread(target=self._fit, args=(points, self.home), daemon=True).start()

    def _fit(self, points, home):
        try:
            coil = fit_coil(points, home)
            checks = list(zip(check_points(coil, points, home), left_out_check(coil, points, home)))
            message = None
        except ValueError as e:
            coil, checks, message = None, [], f"Fit failed: {e}"
        with self._lock:
            if [(p, e) for p, e, _ in self.points] != points or self.home != home:   # changed while fitting
                return
            self.coil, self.checks, self.message, self.fitting = coil, checks, message, False
            self._home_field = coil.field(home) if coil else None
            self._last = home
        self._save()

    def _save_later(self):
        threading.Thread(target=self._save, daemon=True).start()

    def _save(self):
        if self._path is None:
            return
        with self._save_lock:
            self._write()

    def _write(self):
        with self._lock:
            data = {"points": [{"position": list(p), "error": list(e), "spread_ut": s}
                               for p, e, s in self.points]}
            if self._from_home:       # still waiting for home: keep the old file's form
                data["coil"] = self._old_coil.as_dict() if self._old_coil else None
            else:
                data.update(home=list(self.home) if self.home else None,
                            coil=self.coil.as_dict() if self.coil else None, checks=self.checks)
        try:                          # written whole then swapped in, so a reader never sees half
            tmp = self._path.with_name(f"{self._path.name}.{threading.get_ident()}.tmp")
            tmp.write_text(json.dumps(data, indent=2))
            tmp.replace(self._path)
        except OSError as e:
            with self._lock:
                self.message = f"Couldn't save the coil calibration: {e}"

    def _load(self):
        if self._path is None:
            return
        try:
            d = json.loads(self._path.read_text())
            self.points = [(tuple(float(v) for v in q["position"]), tuple(float(v) for v in q["error"]),
                            float(q.get("spread_ut", 0.0))) for q in d["points"]]
            coil = Coil.from_dict(d["coil"]) if d.get("coil") else None
            if "home" not in d:       # an old file: positions measured from home, coil and all
                self._from_home = bool(self.points)
                self._old_coil = coil if self._from_home else None
                return
            self.home = tuple(float(v) for v in d["home"]) if d["home"] else None
            self.coil = coil
            self.checks = [tuple(c) for c in d.get("checks", [])]
            self._home_field = self.coil.field(self.home) if self.coil and self.home else None
        except (OSError, ValueError, KeyError, TypeError):
            self.home, self.points, self.coil, self.checks = None, [], None, []

    def status(self):
        with self._lock:
            c, home = self.coil, self.home
            checks = self.checks if len(self.checks) == len(self.points) else [(None, None)] * len(self.points)
            return {
                "home": None if home is None else [round(v, 2) for v in home],
                "points_from_home": self._from_home,
                "suggested_home": ([round(-v, 1) for v in self._old_coil.center]
                                   if self._old_coil else None),     # the old fit's coil, seen from home
                "points": [{"position": [round(v, 2) for v in p], "spread_ut": s, "off_mm": k[0],
                            "left_out_mm": k[1]} for (p, _, s), k in zip(self.points, checks)],
                "min_points": LOCATE_MIN_POINTS,
                "capturing": self._capture is not None,
                "capture_done": len(self._capture[1]) / LOCATE_POINT_SAMPLES if self._capture else 0.0,
                "fitting": self.fitting,
                "message": self.message,
                "range_cm": LOCATE_RANGE_CM,
                "center_slack_cm": LOCATE_CENTER_SLACK_CM,
                "coil": None if c is None or home is None else {
                    "center": [round(v, 2) for v in c.center],
                    "center_off_cm": round(norm(c.center), 1),
                    "axis": [round(v, 4) for v in c.axis],
                    "half_length": round(c.half_length, 2),
                    "strength": round(c.strength, 1),
                    "home_field": [round(v, 2) for v in c.field(home)],
                    "home_field_ut": round(norm(c.field(home)), 1),
                    "home_mm_per_ut": round(min(mm_per_ut(c, home), 999.0), 1)},
            }
