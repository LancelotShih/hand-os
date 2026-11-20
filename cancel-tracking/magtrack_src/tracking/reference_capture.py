"""Capture a reference field and orientation at a roughly held pose.

This averages gain-scaled sensor readings. Tracker then applies the offset and
mounting corrections to store the total reference field. With the coil off,
that is the assumed uniform ambient field (Earth plus other uniform, steady
contributions). With the coil on, it also includes the coil field at capture.

Readings are averaged in sensor axes and paired with the middle sample's pose.
This approximates a held pose; permitted wobble does not generally cancel
exactly. Live tracking uses measurement-time rotations, but this capture does not.

Used by: tracking/tracker.py.  Uses: geometry, config.
"""

from magtrack_src.config import CAPTURE_MAX_TURN_DEG, CAPTURE_MIN_MAG, CAPTURE_S
from magtrack_src.geometry.vectors import add, quat_angle_deg, quat_conj, quat_mul


class ReferenceCapture:
    def __init__(self):
        self.start = None
        self._q_start = None
        self._mag = (0.0, 0.0, 0.0)
        self._qs = []

    def add(self, q, mag, now):
        """Return the average gain-scaled reading and pose once finished, else None."""
        if self.start is None:
            self.start, self._q_start = now, q
        if quat_angle_deg(quat_mul(quat_conj(self._q_start), q)) > CAPTURE_MAX_TURN_DEG:
            self.__init__()               # really turning: start over
            return None
        if mag is not None:
            self._mag = add(self._mag, mag)
            self._qs.append(q)
        n = len(self._qs)
        if now - self.start >= CAPTURE_S and n >= CAPTURE_MIN_MAG:
            return tuple(v / n for v in self._mag), self._qs[n // 2]
        return None
