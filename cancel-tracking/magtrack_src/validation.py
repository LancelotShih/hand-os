"""Shape and numeric checks shared by saved calibration formats. No I/O."""

import math


def finite_number(value, name):
    """Require a JSON number, excluding booleans, strings and nonfinite values."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except OverflowError:
        raise ValueError(f"{name} is too large") from None
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def vector3(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain exactly three numbers")
    vector = tuple(finite_number(v, name) for v in value)
    if not math.isfinite(sum(v * v for v in vector)):
        raise ValueError(f"{name} is too large for vector calculations")
    return vector


def orthogonal_matrix3(value):
    """Require unit, perpendicular rows; allow rotations and reflections.

    The existing mounting fit can select either determinant sign. The 1e-5
    tolerance allows rounding without accepting meaningful scaling or shear.
    """
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("mounting axes must contain exactly three rows")
    rows = [vector3(row, "mounting row") for row in value]
    for i in range(3):
        for j in range(i, 3):
            product = sum(a * b for a, b in zip(rows[i], rows[j]))
            if not math.isclose(product, float(i == j), rel_tol=0.0, abs_tol=1e-5):
                raise ValueError("mounting axes must be unit length and perpendicular")
    return [list(row) for row in rows]
