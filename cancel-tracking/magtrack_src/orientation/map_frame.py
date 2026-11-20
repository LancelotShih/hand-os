"""Recover saved hardware-defined axes from gravity and a coil-off ambient field.

No compass names or east/north/up coordinate convention is imposed. The first
setup's board axes define the map. Two nonparallel physical vectors recover it.
"""

import math
from magtrack_src.geometry.vectors import (dot, norm, quat_between, quat_conj,
                                           quat_mul, quat_norm, quat_rotate, sub)
from magtrack_src.validation import vector3


def _unit(v):
    size = norm(v)
    if size < 1e-9:
        raise ValueError("A nonzero gravity and ambient magnetic field are needed to recover orientation.")
    return tuple(x / size for x in v)


def align_vectors(gravity, magnetic, saved_gravity, saved_magnetic):
    """Quaternion taking current board axes into saved map axes.

    Gravity determines tilt. The magnetic component perpendicular to gravity
    determines rotation about gravity. Reject near-parallel vectors rather
    than invent a heading (horizontal component below 5% of field magnitude).
    """
    g, b, target_g, target_b = [vector3(v, "orientation vector") for v in
                                (gravity, magnetic, saved_gravity, saved_magnetic)]
    g, target_g = _unit(g), _unit(target_g)
    def horizontal(v, up):
        h = sub(v, tuple(dot(v, up) * x for x in up))
        if norm(h) < 0.05 * norm(v):
            raise ValueError("The ambient field is too close to gravity to determine starting orientation.")
        return _unit(h)
    h, target_h = horizontal(b, g), horizontal(target_b, target_g)
    tilt = quat_between(g, target_g)
    turned_h = quat_rotate(tilt, h)
    # Both horizontal vectors share target_g, so this is a rotation about it.
    heading = quat_between(turned_h, target_h)
    if dot(turned_h, target_h) < -1.0 + 1e-8:
        heading = (0.0, *target_g)
    return quat_norm(quat_mul(heading, tilt))


def validate_frame(value):
    if not isinstance(value, dict) or not isinstance(value.get("id"), str) or not value["id"]:
        raise ValueError("saved map directions need an identifier")
    if not isinstance(value.get("correction_id"), str):
        raise ValueError("saved map directions need a sensor-correction identifier")
    gravity = vector3(value["gravity"], "saved gravity")
    ambient = vector3(value["ambient_field"], "saved ambient field")
    align_vectors(gravity, ambient, gravity, ambient)
    return {"id": value["id"], "gravity": gravity, "ambient_field": ambient,
            "correction_id": value["correction_id"]}
