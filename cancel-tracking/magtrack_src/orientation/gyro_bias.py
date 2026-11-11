"""Measures the gyro's bias whenever the board happens to rest.

Used by: tracking/tracker.py.  Uses: geometry, config.
"""

from magtrack_src.config import REST_MAX_BIAS_DPS, REST_S, REST_SPREAD_DPS
from magtrack_src.geometry.vectors import add


class GyroRest:
    def __init__(self):
        self._window = None

    def add(self, gyro, now):
        """One reading.  Returns the bias when a full rest ends, otherwise ``None``."""
        w = self._window
        if w is None:
            w = self._window = {"start": now, "lo": list(gyro), "hi": list(gyro),
                                "sum": (0.0, 0.0, 0.0), "n": 0}
        w["lo"] = [min(a, b) for a, b in zip(w["lo"], gyro)]
        w["hi"] = [max(a, b) for a, b in zip(w["hi"], gyro)]
        if max(h - l for h, l in zip(w["hi"], w["lo"])) > REST_SPREAD_DPS:
            self._window = None
            return None
        w["sum"] = add(w["sum"], gyro)
        w["n"] += 1
        if now - w["start"] < REST_S:
            return None
        self._window = None
        mean = tuple(v / w["n"] for v in w["sum"])
        return mean if max(abs(v) for v in mean) < REST_MAX_BIAS_DPS else None
