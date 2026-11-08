"""One capture: a short average of the magnetometer at a roughly held pose.

The board needn't be still.  A wobble averages out over the window, and the
pose from the middle of the window matches the averaged reading.
"""

from config import CAPTURE_MAX_TURN_DEG, CAPTURE_MIN_MAG, CAPTURE_S
from geometry import add, quat_angle_deg, quat_conj, quat_mul


class Capture:
    def __init__(self):
        self.start = None
        self._q_start = None
        self._mag = (0.0, 0.0, 0.0)
        self._qs = []

    def add(self, q, mag, now):
        """One reading.  Returns ``(u, q)`` once finished, otherwise ``None``."""
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
