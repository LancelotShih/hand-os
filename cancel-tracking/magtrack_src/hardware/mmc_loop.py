"""Reads the MMC5983MA forever on its own thread, reconnecting if it drops out.

Its own thread because a reading takes milliseconds of waiting, and the IMU loop
must not wait with it. The kernel serializes bus transactions; the threads still
share bandwidth. A single latest-unread slot can overwrite unconsumed readings.

Used by: earth_cancel.py, tracking/pair.py (take).  Uses: hardware/mmc5983.py.
"""

import sys
import threading

from magtrack_src.hardware.mmc5983 import MMC


class MmcReader:
    def __init__(self, bus, address, open_mmc=MMC):
        self.bus, self.address = bus, address
        self._open = open_mmc
        self._lock = threading.Lock()
        self._fresh = None
        self.connected = False
        self.error = "not started"

    def take(self):
        """The newest unread ``(field, t)``, or ``None``."""
        with self._lock:
            fresh, self._fresh = self._fresh, None
        return fresh

    def run(self, stop):
        while not stop.is_set():
            mmc = None
            try:
                mmc = self._open(self.bus, self.address)
                self.connected, self.error = True, None
                while not stop.is_set():
                    reading = mmc.read()
                    with self._lock:
                        self._fresh = reading
            except OSError as e:
                self.connected, self.error = False, str(e)
                print(f"MMC5983MA error ({e}); retrying in 2 s...", file=sys.stderr)
                stop.wait(2.0)
            finally:
                if mmc is not None:
                    mmc.close()
