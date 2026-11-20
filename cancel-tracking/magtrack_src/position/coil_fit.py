"""Numerical coil fitting from known positions and absolute coil-field vectors.

No file, hardware, thread, orientation or live tracking state is owned here.
"""

import math

from magtrack_src.config import LOCATE_CENTER_GRID_CM, LOCATE_CENTER_SLACK_CM, LOCATE_MIN_POINTS
from magtrack_src.geometry.vectors import dot, norm, sub
from magtrack_src.position.coil_model import Coil

KEEP_CLEAR_CM = 3.0  # minimum calibration-point/home distance from the coil center

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


def _coarse_search(points, slack_cm, grid_cm):
    """Best few coil centers on a grid within ``slack_cm`` of 0, treating the coil
    as a point: for a given center the field is linear in the magnet's moment, so
    each guess is quick."""
    clear = [p for p, _ in points]
    n = int(slack_cm // grid_cm)
    scored = []
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            for k in range(-n, n + 1):
                c = (i * grid_cm, j * grid_cm, k * grid_cm)
                if norm(c) > slack_cm + 1e-9 or any(norm(sub(c, p)) < KEEP_CLEAR_CM for p in clear):
                    continue
                rows, rhs = [], []
                for p, e in points:
                    m = _dipole_matrix(sub(p, c))
                    rows += m
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


def _residuals(points, slack_cm):
    def f(x):
        coil = Coil.from_params(x, slack_cm)
        out = []
        for p, e in points:
            out.extend(sub(coil.field(p), e))
        return out
    return f


def refine_coil(points, x0, slack_cm=LOCATE_CENTER_SLACK_CM, iterations=200):
    x, cost = least_squares(_residuals(points, slack_cm), x0, iterations,
                            steps=[1e-3] * 3 + [1e-3 * max(1.0, max(abs(v) for v in x0[3:6]))] * 3 + [1e-3])
    return Coil.from_params(x, slack_cm), cost


def fit_coil(points, slack_cm=LOCATE_CENTER_SLACK_CM, grid_cm=LOCATE_CENTER_GRID_CM):
    """The coil that best explains ``points``: ``[(position, error), ...]``,
    positions in cm from the coil's center, coil fields in uT in saved map axes.
    These are ambient-subtracted fields in the saved hardware-defined map axes."""
    if len(points) < LOCATE_MIN_POINTS:
        raise ValueError(f"A coil model needs at least {LOCATE_MIN_POINTS} position-and-field measurements; {len(points)} are available.")
    best = None
    for _, c, moment in _coarse_search(points, slack_cm, grid_cm):
        half = 2.0
        coil, cost = refine_coil(points, [*c, *(v / (2.0 * half) for v in moment), half], slack_cm)
        if best is None or cost < best[1]:
            best = (coil, cost)
    if best is None:
        raise ValueError("Could not calculate a coil model. Spread the measured positions through the working area and keep the sensor at least 3 cm from the coil.")
    return best[0]


