"""A simulated board in a uniform field, stepped at 500 Hz.  No hardware.

With a ``coil``, the board also sees its field at ``pos`` (cm, in the world axes,
which are the board's own axes when ``q`` is the identity)."""

import math
import random

from magtrack_src.geometry.vectors import add, quat_conj, quat_mul, quat_norm, quat_rotate

AMBIENT_FIELD = (17.0, -4.0, -43.0)  # uniform, steady background in the fixed world axes
OFFSET = (-74.9, 64.0, 7.9)      # about 100 uT, in the board's axes
GYRO_BIAS = (0.4, -0.3, 0.25)
STEP_S = 0.002


def axis_quat(axis, deg):
    n = math.sqrt(sum(a * a for a in axis))
    s = math.sin(math.radians(deg) / 2) / n
    return (math.cos(math.radians(deg) / 2), axis[0] * s, axis[1] * s, axis[2] * s)


class SimBoard:
    def __init__(self, tracker, gyro_bias=GYRO_BIAS, q=(1.0, 0.0, 0.0, 0.0),
                 mag_gain=(1.0, 1.0, 1.0), coil=None):
        self.tracker = tracker
        self.coil = coil
        self.pos = (0.0, 0.0, 0.0)
        self.mag_gain = mag_gain
        self.gyro_bias = gyro_bias
        self.q = q
        self.t = 100.0
        self.samples = []
        self._k = 0

    def step(self, rate_dps=(0.0, 0.0, 0.0)):
        n = math.sqrt(sum(v * v for v in rate_dps))
        if n:
            self.q = quat_norm(quat_mul(self.q, axis_quat(rate_dps, n * STEP_S)))
        self.t += STEP_S
        self._k += 1
        accel = tuple(v + random.gauss(0, .003) for v in quat_rotate(quat_conj(self.q), (0, 0, 1)))
        gyro = tuple(rate_dps[i] + self.gyro_bias[i] + random.gauss(0, .05) for i in range(3))
        mag = None
        if self._k % 5 == 0:
            total_field = AMBIENT_FIELD if self.coil is None else add(AMBIENT_FIELD, self.coil.field(self.pos))
            field = quat_rotate(quat_conj(self.q), total_field)
            field = add(tuple(f * g for f, g in zip(field, self.mag_gain)), OFFSET)
            mag = tuple(v + random.gauss(0, 0.4) for v in field)
        sample = self.tracker.update(accel, gyro, mag, self.t)
        if sample is not None:
            self.samples.append(sample)

    def still(self, seconds):
        for _ in range(int(seconds / STEP_S)):
            self.step()

    def turn(self, axis, deg, rate=60.0):
        n = math.sqrt(sum(a * a for a in axis))
        sign = 1 if deg > 0 else -1
        for _ in range(int(abs(deg) / rate / STEP_S)):
            self.step(tuple(a / n * rate * sign for a in axis))

    def shake(self, seconds, amp):
        """Hand tremor on all three axes."""
        end = self.t + seconds
        while self.t < end:
            u = self.t
            self.step((amp * math.sin(2 * math.pi * 6.1 * u),
                       amp * math.sin(2 * math.pi * 8.3 * u + 1),
                       amp * math.sin(2 * math.pi * 4.7 * u + 2)))

    def tumble(self, seconds):
        """Turn through every direction, like a hand would."""
        end = self.t + seconds
        while self.t < end:
            u = self.t
            self.step((90 * math.sin(.71 * u), 110 * math.sin(.53 * u + 1.3), 80 * math.sin(.37 * u + 2.6)))
