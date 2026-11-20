"""Local absolute-field refinement and calibration consistency checks.

Live global matching is owned by field_lookup; history selection is in Locator.
"""

import math

from magtrack_src.config import LOCATE_MAX_MISMATCH_UT, LOCATE_MIN_POINTS, LOCATE_RANGE_CM
from magtrack_src.geometry.vectors import add, norm, sub
from magtrack_src.position.coil_fit import least_squares, refine_coil
from magtrack_src.position.coil_model import scale_vector

def _seeds(coil, target):
    """Places to try when the last position doesn't lead anywhere: around the coil
    at the distance a point magnet of its strength would give this field."""
    moment = coil.strength * 2.0 * coil.half_length
    r = (moment / max(norm(target), 1e-3)) ** (1.0 / 3.0)
    dirs = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    dirs += [(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    return [add(coil.center, scale_vector(d, r / norm(d))) for d in dirs]


def _settle(coil, target, start):
    x, cost = least_squares(lambda p: list(sub(coil.field(p), target)), start, 30,
                            steps=[1e-4] * 3)
    return tuple(x), math.sqrt(cost)


def solve(coil, target, start):
    """Local absolute-field check, choosing the nearer mirror to its known seed."""
    p, miss = _settle(coil, target, start)
    if miss > LOCATE_MAX_MISMATCH_UT or norm(sub(p, coil.center)) > 3 * LOCATE_RANGE_CM:
        tries = [_settle(coil, target, s) for s in _seeds(coil, target)]
        p, miss = min(tries, key=lambda t: t[1])
    other = coil.mirror(p)            # same field: keep the side nearer where we were
    if norm(sub(other, start)) < norm(sub(p, start)):
        p = other
    return p, miss


def check_points(coil, points):
    """How far each point solves from where it was said to be, in mm."""
    return [round(10.0 * norm(sub(solve(coil, e, p)[0], p)), 1) for p, e in points]


def left_out_check(coil, points):
    """Local leave-one-out consistency: refine from the all-points model without
    each point, then solve starting at its known position. This is not an
    independent global accuracy test. None when too few points remain."""
    if len(points) <= LOCATE_MIN_POINTS:
        return [None] * len(points)
    out = []
    for i, (p, e) in enumerate(points):
        rest = points[:i] + points[i + 1:]
        refit, _ = refine_coil(rest, coil.params(), iterations=60)
        out.append(round(10.0 * norm(sub(solve(refit, e, p)[0], p)), 1))
    return out


