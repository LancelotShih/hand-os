"""Vector and quaternion math.  Vectors are 3-tuples, quaternions (w, x, y, z).

Used by: orientation, calibration, tracking, position.  Uses: nothing.
"""

import math


def add(a, b):    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def sub(a, b):    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def dot(a, b):    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def norm(a):      return math.sqrt(dot(a, a))
def mat_vec(m, v): return (dot(m[0], v), dot(m[1], v), dot(m[2], v))


def angle_deg(a, b):
    """The angle between two vectors, 0-180 degrees; 0 if either is zero."""
    size = norm(a) * norm(b)
    return 0.0 if size == 0.0 else math.degrees(math.acos(max(-1.0, min(1.0, dot(a, b) / size))))


def quat_norm(q):
    n = math.sqrt(sum(v * v for v in q))
    return (1.0, 0.0, 0.0, 0.0) if n == 0.0 else tuple(v / n for v in q)


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_rotate(q, v):
    """``R v`` for the rotation ``R`` of unit quaternion ``q``."""
    w, x, y, z = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty,
            v[1] + w * ty + z * tx - x * tz,
            v[2] + w * tz + x * ty - y * tx)


def quat_angle_deg(q):
    """How far ``q`` turns, 0-180 degrees."""
    return math.degrees(2.0 * math.acos(min(1.0, abs(q[0]))))


def quat_between(a, b):
    """The smallest turn that takes direction ``a`` onto direction ``b``."""
    na, nb = norm(a), norm(b)
    if na < 1e-9 or nb < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    a, b = tuple(v / na for v in a), tuple(v / nb for v in b)
    c = dot(a, b)
    if c < -1.0 + 1e-9:                          # opposite: turn half way round any perpendicular
        axis = (0.0, -a[2], a[1]) if abs(a[0]) < 0.9 else (-a[1], a[0], 0.0)
        n = norm(axis)
        return (0.0, axis[0] / n, axis[1] / n, axis[2] / n)
    x = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
    return quat_norm((1.0 + c, *x))


def quat_from_gravity(a):
    """Quaternion (board -> level) that tilts the accelerometer's ``a`` onto +z."""
    n = norm(a)
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    ax, ay, az = a[0] / n, a[1] / n, a[2] / n
    w = math.sqrt(max(0.0, (1.0 + az) / 2.0))
    if w < 1e-6:
        return (0.0, 1.0, 0.0, 0.0)              # upside down
    return quat_norm((w, ay / (2.0 * w), -ax / (2.0 * w), 0.0))


def quat_to_euler(q):
    """(yaw, pitch, roll) in degrees, ZYX."""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (math.degrees(yaw), math.degrees(pitch), math.degrees(roll))
