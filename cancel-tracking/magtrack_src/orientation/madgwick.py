"""Madgwick orientation filter, gyro + accelerometer only.

Used by: tracking/tracker.py.  Uses: geometry, config.
"""

import math

from magtrack_src.config import AHRS_BETA
from magtrack_src.geometry.vectors import quat_norm


def madgwick_step(q, gx, gy, gz, ax, ay, az, dt, beta=AHRS_BETA):
    """One orientation update using three gyro and three accelerometer axes.  Gyro in rad/s, accel in any unit."""
    q0, q1, q2, q3 = q
    qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
    qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
    qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

    if not (ax == 0.0 and ay == 0.0 and az == 0.0):
        recip = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
        ax *= recip; ay *= recip; az *= recip

        _2q0 = 2.0 * q0; _2q1 = 2.0 * q1; _2q2 = 2.0 * q2; _2q3 = 2.0 * q3
        _4q0 = 4.0 * q0; _4q1 = 4.0 * q1; _4q2 = 4.0 * q2
        _8q1 = 8.0 * q1; _8q2 = 8.0 * q2
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
        s1 = (_4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1
              + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az)
        s2 = (4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
              + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az)
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
        snorm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        if snorm > 0.0:
            qDot1 -= beta * s0 / snorm
            qDot2 -= beta * s1 / snorm
            qDot3 -= beta * s2 / snorm
            qDot4 -= beta * s3 / snorm

    return quat_norm((q0 + qDot1 * dt, q1 + qDot2 * dt, q2 + qDot3 * dt, q3 + qDot4 * dt))
