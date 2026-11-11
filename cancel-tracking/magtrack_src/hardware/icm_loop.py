"""Reads the IMU forever, reconnecting if it drops out.

``tracker`` is a ``Pair``, which collects the MMC5983MA's newest reading itself."""

import sys
import time

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
                sample = tracker.update(accel, gyro, mag, now)
                log.add(now, accel, gyro, mag, sample, tracker.reading)
                if sample is not None:
                    feed.push(sample)
        except OSError as e:
            feed.connected, feed.error = False, str(e)
            print(f"IMU error ({e}); retrying in 2 s...", file=sys.stderr)
            stop.wait(2.0)
        finally:
            if imu is not None:
                imu.close()
