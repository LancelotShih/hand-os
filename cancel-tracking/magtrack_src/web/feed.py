"""Hands samples from the sensor thread to the browser tabs.

Used by: hardware/icm_loop.py (push), web/server.py (since).  Uses: nothing.

final sample structure is:
{
    # From Tracker: orientation and magnetic-field data
    "quat": [1.0, 0.0, 0.0, 0.0],       # orientation quaternion (w, x, y, z)
    "euler": [0.0, 0.0, 0.0],           # yaw, pitch, roll in degrees
    "turn": 0.2,                         # degrees from the reference pose
    "measured": [20.1, -3.2, 41.5],     # magnetic field, µT
    "measured_mag": 46.2,
    "predicted": [20.0, -3.0, 41.4],    # captured reference field, µT
    "predicted_mag": 46.0,
    "error": [0.1, -0.2, 0.1],          # measured minus predicted, µT
    "error_ut": 0.25,
    "error_deg": 0.3,
    "peak": 0.7,
    "peak_deg": 1.1,
    "calibration_on": True,

    # From Pair: MMC result, or None if the MMC is off/unavailable
    "mmc": None,

    # From Pair + Locator: position result
    "locate": {
        "position": [12.3, -4.5, 8.0],  # x, y, z in cm
        "from_coil_cm": 15.3,
        "from_home_cm": 3.2,
        "mismatch_ut": 0.4,
        "mm_per_ut": 1.2,
    },

    # Added by Feed.push()
    "seq": 123,
}
"""

import threading
from collections import deque


class Feed:
    """Latest samples; each open stream advances its own sequence cursor.

    A new stream starts at zero and replays the retained buffer. Connection/error
    attributes are independent of the buffer lock.
    """

    def __init__(self):
        self._buf = deque(maxlen=200)
        self._lock = threading.Lock()
        self._seq = 0
        self.connected = False
        self.error = None

    def push(self, sample):
        with self._lock:
            self._seq += 1
            sample["seq"] = self._seq
            self._buf.append(sample)

    def since(self, seq):
        with self._lock:
            return [s for s in self._buf if s["seq"] > seq]
