"""Hands samples from the sensor thread to the browser tabs.

Used by: hardware/icm_loop.py (push), web/server.py (since).  Uses: nothing.
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
