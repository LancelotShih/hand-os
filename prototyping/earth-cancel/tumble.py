"""Tumble calibration: finds the magnetometer's offset and axis asymmetry.

Earth's field has one strength, so as the board turns every reading should sit
on a sphere around zero.  The offset moves that sphere's centre and unequal axis
sensitivity stretches it into an ellipsoid.  Fitting the shape measures both,
from the magnetometer alone: no orientation is involved, so gyro error can't
affect it.
"""

import math

from config import (PASS_UT, TUMBLE_COVERAGE_RECOUNT, TUMBLE_MAX_FIELD_UT, TUMBLE_MAX_GAIN,
                    TUMBLE_MAX_POINTS, TUMBLE_MIN_COVERAGE, TUMBLE_MIN_POINTS,
                    TUMBLE_MIN_SWING_RATIO, TUMBLE_MIN_SWING_UT)
from geometry import dot, norm, sub

SECTORS = 26              # directions around a sphere, as in a 3x3x3 cube minus its centre
SECTOR_EDGE = 0.38        # a component this far along an axis counts as pointing that way


def midpoint_and_swing(points):
    """The middle of each direction's readings, and half the swing around it."""
    low = [min(p[i] for p in points) for i in range(3)]
    high = [max(p[i] for p in points) for i in range(3)]
    return (tuple((l + h) / 2.0 for l, h in zip(low, high)),
            tuple((h - l) / 2.0 for l, h in zip(low, high)))


def swung_enough(swing):
    """Has every direction really swung, and about equally?"""
    return min(swing) >= max(TUMBLE_MIN_SWING_UT, TUMBLE_MIN_SWING_RATIO * max(swing))


def sectors(points, mid):
    """Which of the sphere's directions the readings reach, seen from ``mid``.
    Each direction is a tuple of -1, 0 or 1 per axis."""
    seen = set()
    for p in points:
        v = sub(p, mid)
        n = norm(v)
        if n > 0.0:
            seen.add(tuple((c > SECTOR_EDGE * n) - (c < -SECTOR_EDGE * n) for c in v))
    return seen


def coverage(points):
    """Share of the sphere's directions the readings have reached, 0-1.

    Zero until every direction has really swung: measured from the middle of a
    still board's readings, noise alone points in every direction."""
    if not points:
        return 0.0
    mid, swing = midpoint_and_swing(points)
    return len(sectors(points, mid)) / SECTORS if swung_enough(swing) else 0.0


def solve_linear(A, b):
    """Gaussian elimination with partial pivoting.  Raises ValueError if singular."""
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-12:
            raise ValueError("the readings don't cover enough directions")
        M[c], M[p] = M[p], M[c]
        for r in range(n):
            if r != c:
                k = M[r][c] / M[c][c]
                M[r] = [a - k * e for a, e in zip(M[r], M[c])]
    return [M[i][n] / M[i][i] for i in range(n)]


def fit_ellipsoid(points):
    """Axis-aligned ellipsoid through ``points``.  Returns ``(center, radii)``.

    a x^2 + b y^2 + c z^2 + d x + e y + f z = 1, centred and scaled first so the
    numbers stay well conditioned."""
    n = len(points)
    mean = tuple(sum(p[i] for p in points) / n for i in range(3))
    size = math.sqrt(sum(dot(sub(p, mean), sub(p, mean)) for p in points) / n) or 1.0
    AtA = [[0.0] * 6 for _ in range(6)]
    Atb = [0.0] * 6
    for p in points:
        x, y, z = ((p[i] - mean[i]) / size for i in range(3))
        row = (x * x, y * y, z * z, x, y, z)
        for i in range(6):
            Atb[i] += row[i]
            for j in range(6):
                AtA[i][j] += row[i] * row[j]
    a, b, c, d, e, f = solve_linear(AtA, Atb)
    if min(a, b, c) <= 0.0:
        raise ValueError("the readings don't form a closed shape; turn through more directions")
    g = 1.0 + d * d / (4 * a) + e * e / (4 * b) + f * f / (4 * c)
    center = (mean[0] - size * d / (2 * a), mean[1] - size * e / (2 * b), mean[2] - size * f / (2 * c))
    return center, tuple(size * math.sqrt(g / k) for k in (a, b, c))


def spread_95(points, center, gains):
    """95% of readings lie within this many uT of a perfect sphere, after removing
    ``center`` and applying ``gains``."""
    sizes = [norm(tuple((p[i] - center[i]) * gains[i] for i in range(3))) for p in points]
    mean = sum(sizes) / len(sizes)
    return sorted(abs(s - mean) for s in sizes)[int(0.95 * (len(sizes) - 1))]


def fit_tumble(points):
    """Everything the tumble measured.  Raises ValueError if it can't be fitted."""
    center, radii = fit_ellipsoid(points)
    field = sum(radii) / 3.0
    if not TUMBLE_MIN_SWING_UT <= field <= TUMBLE_MAX_FIELD_UT:
        raise ValueError(f"a {field:.0f} uT field isn't Earth's; turn the board through every "
                         "direction, away from magnets")
    gains = tuple(field / r for r in radii)
    if not all(1.0 / TUMBLE_MAX_GAIN <= g <= TUMBLE_MAX_GAIN for g in gains):
        raise ValueError("the shape is too stretched to be the sensor; keep it away "
                         "from magnets and steel, and stay in one spot")
    after = spread_95(points, center, gains)
    return {"center": list(center), "radii": list(radii), "gains": list(gains),
            "field_ut": field,
            "asymmetry_ut": max(radii) - min(radii),
            "asymmetry_pct": 100.0 * (max(radii) - min(radii)) / field,
            "spread_before_ut": spread_95(points, center, (1.0, 1.0, 1.0)),
            "spread_after_ut": after,
            "points": len(points),
            "limit_ut": PASS_UT,
            "passed": after <= PASS_UT}


class Tumble:
    """Collects magnetometer readings while the board is turned."""

    def __init__(self):
        self.points = []
        self.quats = []                     # orientation at each reading
        self.swept = set()                  # directions reached; only ever grows
        self.swing = (0.0, 0.0, 0.0)
        self._mid = None                    # set once directions count
        self._rough_mid = None              # good enough to show movement before that
        self._since_recount = 0

    def add(self, mag, q=None):
        if len(self.points) >= TUMBLE_MAX_POINTS:
            self.points = self.points[::2]      # full: thin it evenly and keep going
            self.quats = self.quats[::2]
        self.points.append(mag)
        self.quats.append(q)
        self._since_recount += 1
        if self._since_recount >= TUMBLE_COVERAGE_RECOUNT:
            self._since_recount = 0
            self._recount()

    def _recount(self):
        mid, self.swing = midpoint_and_swing(self.points)
        if max(self.swing) >= TUMBLE_MIN_SWING_UT:
            self._rough_mid = mid
        if swung_enough(self.swing):
            self._mid = mid
            self.swept |= sectors(self.points, mid)

    @property
    def counting(self):
        return self._mid is not None

    @property
    def coverage(self):
        return len(self.swept) / SECTORS

    @property
    def now(self):
        """Where the field points now, as a unit vector; ``None`` until the board
        has moved enough to have a middle to measure from."""
        mid = self._mid or self._rough_mid
        if mid is None or not self.points:
            return None
        v = sub(self.points[-1], mid)
        n = norm(v)
        return tuple(c / n for c in v) if n > 0.0 else None

    @property
    def ready(self):
        return len(self.points) >= TUMBLE_MIN_POINTS and self.coverage >= TUMBLE_MIN_COVERAGE
