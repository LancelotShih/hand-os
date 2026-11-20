"""Reads the IMU forever, reconnecting if it drops out.

``tracker`` is a ``Pair``, which collects the MMC5983MA's newest reading itself.

Used by: earth_cancel.py.  Uses: hardware/icm20948.py; calls the tracker (a Pair), DataLog and Feed it's given.
"""

import sys
import time

from magtrack_src.config import I2C_RETRY_DELAY_S
from magtrack_src.hardware.icm20948 import IMU


def sensor_loop(bus, address, tracker, feed, log, stop, open_imu=IMU):
    first = True
    while not stop.is_set():
        imu = None
        try:
            imu = open_imu(bus, address)
            feed.connected, feed.error = True, None
            if not first:
                tracker.sensor_lost()
            first = False
            while not stop.is_set():
                accel, gyro, mag = imu.read()
                now = time.monotonic()
                sample = tracker.update(accel, gyro, mag, now) #tracker is actually a Pair object
                log.add(now, accel, gyro, mag, sample, tracker.reading)
                if sample is not None:
                    sample["timestamp_ms"] = int(now * 1000)
                    feed.push(sample)
        except OSError as e:
            feed.connected, feed.error = False, str(e)
            print(f"IMU error ({e}); retrying in {I2C_RETRY_DELAY_S:g} s...", file=sys.stderr)
            stop.wait(I2C_RETRY_DELAY_S)
        finally:
            if imu is not None:
                imu.close()
