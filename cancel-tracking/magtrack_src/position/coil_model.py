"""Finite two-pole coil field and local movement-per-field-change calculation.

Positions are cm; fields are microtesla. These functions do not read files,
start workers, select tracking history, or change hardware coordinate axes.
Physical accuracy of this model still needs independent measurements.
"""

import math

from magtrack_src.geometry.vectors import add, norm, sub
from magtrack_src.validation import finite_number, vector3

MIN_HALF_LENGTH_CM = 0.2
MAX_HALF_LENGTH_CM = 6.0


def scale_vector(a, k):
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
            center = scale_vector(center, slack / r)
        return cls(center, x[3:6], min(MAX_HALF_LENGTH_CM, max(MIN_HALF_LENGTH_CM, x[6])))

    def params(self):
        return [*self.center, *self.s, self.half_length]

    @property
    def strength(self):
        return norm(self.s)

    @property
    def axis(self):
        q = self.strength
        return (1.0, 0.0, 0.0) if q == 0.0 else scale_vector(self.s, 1.0 / q)

    def field(self, p):
        q = self.strength
        if q == 0.0:
            return (0.0, 0.0, 0.0)
        tip = scale_vector(self.s, self.half_length / q)
        out = (0.0, 0.0, 0.0)
        for pole, sign in ((add(self.center, tip), q), (sub(self.center, tip), -q)):
            d = sub(p, pole)
            r = norm(d)
            if r < 1e-6:
                return (math.inf, math.inf, math.inf)
            out = add(out, scale_vector(d, sign / r ** 3))
        return out

    def mirror(self, p):
        """The other position with the same field."""
        return sub(scale_vector(self.center, 2.0), p)

    def as_dict(self):
        return {"center": list(self.center), "s": list(self.s), "half_length": self.half_length}

    @classmethod
    def from_dict(cls, d):
        """Decode a saved model using the same length bounds as the fitter."""
        if not isinstance(d, dict):
            raise ValueError("saved coil model must be an object")
        center = vector3(d["center"], "coil center")
        strength_vector = vector3(d["s"], "coil strength vector")
        half_length = finite_number(d["half_length"], "coil half-length")
        if norm(strength_vector) <= 0.0:
            raise ValueError("coil strength must be greater than zero")
        if not MIN_HALF_LENGTH_CM <= half_length <= MAX_HALF_LENGTH_CM:
            raise ValueError(f"coil half-length must be between {MIN_HALF_LENGTH_CM} and {MAX_HALF_LENGTH_CM} cm")
        return cls(center, strength_vector, half_length)


def mm_per_ut(coil, p):
    """How far off a position can be, in mm, for each uT of field error, in the
    worst direction. In the plane through the coil's middle, the model's vector
    field is the same around each circle centered on its axis. Sensitivity grows
    without limit there; this does not detect the distant mirror solution."""
    j = []
    for k in range(3):
        d = [0.0, 0.0, 0.0]
        d[k] = 1e-3
        j.append(scale_vector(sub(coil.field(add(p, d)), coil.field(sub(p, d))), 1.0 / 2e-3))
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


