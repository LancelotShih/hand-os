"""Finds how a magnetometer sits on the board, from a tumble.

A sensor on its own breakout isn't lined up with the gyro: it's turned by some
whole number of quarter turns, plus a small tilt from how it was soldered.  One
degree of tilt is about 1 uT of false error, so it has to be measured.

The coil-off ambient field is assumed fixed, so with the right ``axes`` every tumble reading,
turned by the orientation at that moment, lands on the same vector.  Uses only
the gyro and accelerometer, so one magnetometer's fit can't borrow from the other.

Turning can't tell ``axes`` from ``-axes``: a reading that's exactly backwards also
stays constant. Tracking is the same either way, because the reference is captured through
the same ``axes`` and only ever compared with itself, so the fit picks the one that
isn't mirrored. Its reference field can therefore point opposite to another sensor's.

Used by: tracking/tracker.py (after a tumble).  Uses: geometry.
"""

import itertools
import math

from magtrack_src.geometry.vectors import add, dot, mat_vec, norm, quat_conj, quat_rotate, sub

MAX_POINTS = 1500         # plenty for three angles, and quick in plain Python
ROUNDS = 40
POLAR_ROUNDS = 30


def plain_mappings():
    """Every way of pointing the chip's axes along the board's: 48 of them, half
    of them mirrored, because a datasheet's axes can be left-handed."""
    out = []
    for order in itertools.permutations(range(3)):
        for signs in itertools.product((1.0, -1.0), repeat=3):
            out.append([[signs[r] if order[r] == c else 0.0 for c in range(3)] for r in range(3)])
    return out


def determinant(m):
    return dot(m[0], (m[1][1] * m[2][2] - m[1][2] * m[2][1],
                      m[1][2] * m[2][0] - m[1][0] * m[2][2],
                      m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def inverse_transpose(m):
    d = determinant(m)
    if abs(d) < 1e-12:
        raise ValueError("Sensor alignment cannot be estimated from these readings. Rotate the board through more orientations.")
    return [[(m[(r + 1) % 3][(c + 1) % 3] * m[(r + 2) % 3][(c + 2) % 3]
              - m[(r + 1) % 3][(c + 2) % 3] * m[(r + 2) % 3][(c + 1) % 3]) / d
             for c in range(3)] for r in range(3)]


def nearest_rotation(m):
    """The closest pure rotation (or mirrored rotation) to ``m``."""
    size = max(abs(v) for row in m for v in row) or 1.0
    x = [[v / size for v in row] for row in m]
    for _ in range(POLAR_ROUNDS):
        inv = inverse_transpose(x)
        x = [[(a + b) / 2.0 for a, b in zip(ra, rb)] for ra, rb in zip(x, inv)]
    return x


def mean_ambient_field(axes, points, quats):
    """Estimate the tumble's ambient field in the orientation filter's fixed axes."""
    total = (0.0, 0.0, 0.0)
    for p, q in zip(points, quats):
        total = add(total, quat_rotate(q, mat_vec(axes, p)))
    return tuple(v / len(points) for v in total)


def improved(axes, points, quats):
    """One step: align rotated readings with their estimated fixed ambient field."""
    ambient_field = mean_ambient_field(axes, points, quats)
    m = [[0.0] * 3 for _ in range(3)]
    for p, q in zip(points, quats):
        b = quat_rotate(quat_conj(q), ambient_field)
        for r in range(3):
            for c in range(3):
                m[r][c] += b[r] * p[c]
    return nearest_rotation(m)


def turned_deg(a, b):
    """How far apart two rotations are."""
    trace = sum(a[r][c] * b[r][c] for r in range(3) for c in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))


def fit_axes(points, quats):
    """``points`` have their midpoint removed; ``quats`` are the orientation at each.

    Returns ``axes`` (chip -> board), the ``plain`` quarter-turn mapping it is
    nearest, the ``tilt_deg`` between them, and ``spread_ut``: 95% of readings land
    within this of the estimated ambient field. A sensor that needs no fit shows
    the method's own error here."""
    step = max(1, len(points) // MAX_POINTS)
    points, quats = points[::step], quats[::step]
    plain = max(plain_mappings(), key=lambda m: (round(norm(mean_ambient_field(m, points, quats)), 3),
                                                 determinant(m) > 0.0))
    axes = plain
    for _ in range(ROUNDS):
        axes = improved(axes, points, quats)
    ambient_field = mean_ambient_field(axes, points, quats)
    misses = sorted(norm(sub(quat_rotate(q, mat_vec(axes, p)), ambient_field))
                    for p, q in zip(points, quats))
    return {"axes": axes, "plain": plain,
            "tilt_deg": turned_deg(axes, plain),
            "spread_ut": misses[int(0.95 * (len(misses) - 1))]}
