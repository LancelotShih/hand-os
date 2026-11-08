#!/usr/bin/env python3
"""The main polling loop for an ICM-20948 -- open the bus, bring the chip up,
then print a sample line at a fixed rate until Ctrl-C.

This file is deliberately small and linear: it is the "how do I actually use
the driver" example.  Run it directly for a no-frills stream:

  ./icm20948_main.py                      # 10 Hz, scaled units, bus 1 @ 0x68
  ./icm20948_main.py -r 50 --mag          # 50 Hz, include the magnetometer
  ./icm20948_main.py --raw -n 20          # 20 raw-count samples, then stop

The full multi-command tool (buses / scan / whoami / read) lives in
:mod:`icm20948_cli`, which reuses :func:`poll_loop` and :func:`run_read` below.
"""

from __future__ import annotations

import argparse
import sys
import time

from icm20948_driver import ICM20948
from icm20948_i2c import open_bus
from icm20948_registers import ACCEL_FS, GYRO_FS


# --- formatting one sample into a printable line --------------------------

def format_scaled(reading, use_mag=False):
    ax, ay, az = reading["accel_g"]
    gx, gy, gz = reading["gyro_dps"]
    line = (f"acc(g)[{ax:+7.3f} {ay:+7.3f} {az:+7.3f}]  "
            f"gyro(dps)[{gx:+8.2f} {gy:+8.2f} {gz:+8.2f}]  "
            f"temp={reading['temp_c']:5.1f}C")
    if use_mag and reading.get("mag_ut"):
        mx, my, mz = reading["mag_ut"]
        line += f"  mag(uT)[{mx:+7.2f} {my:+7.2f} {mz:+7.2f}]"
    return line


def format_raw(accel, gyro, temp):
    return (f"acc[{accel[0]:6d} {accel[1]:6d} {accel[2]:6d}]  "
            f"gyr[{gyro[0]:6d} {gyro[1]:6d} {gyro[2]:6d}]  "
            f"temp={temp:6d}")


def _emit(line):
    print(line, flush=True)


# --- the loop -----------------------------------------------------------

def poll_loop(dev, *, rate=10.0, count=0, raw=False, use_mag=False, out=_emit):
    """Read `dev` at `rate` Hz, emitting one formatted line per sample.

    count=0 runs until KeyboardInterrupt; otherwise stops after `count` samples.
    `out` is the sink for each line (defaults to printing with a flush).
    """
    period = 1.0 / rate if rate > 0 else 0.0
    n = 0
    try:
        while count == 0 or n < count:
            t0 = time.monotonic()
            if raw:
                accel, gyro, temp = dev.read_raw()
                out(format_raw(accel, gyro, temp))
            else:
                out(format_scaled(dev.read(), use_mag))
            n += 1
            if period:
                dt = period - (time.monotonic() - t0)
                if dt > 0:
                    time.sleep(dt)
    except KeyboardInterrupt:
        print()


# --- shared "read" entry point (used here and by icm20948_cli) ----------

def add_read_arguments(p):
    """Attach the sample-stream options to an argparse parser/subparser."""
    p.add_argument("-n", "--count", type=int, default=0,
                   help="stop after N samples (0 = run until Ctrl-C)")
    p.add_argument("-r", "--rate", type=float, default=10.0,
                   help="samples per second (default: 10)")
    p.add_argument("--accel-range", type=int, default=4, choices=sorted(ACCEL_FS),
                   help="+/- g (default: 4)")
    p.add_argument("--gyro-range", type=int, default=500, choices=sorted(GYRO_FS),
                   help="+/- dps (default: 500)")
    p.add_argument("--mag", action="store_true",
                   help="also read the AK09916 magnetometer")
    p.add_argument("--raw", action="store_true",
                   help="print raw 16-bit counts instead of physical units")
    p.add_argument("--no-reset", action="store_true",
                   help="skip the power-on reset during init")


def run_read(args):
    """Open the bus, bring the chip up, and stream. Expects an argparse
    namespace with: bus, address, accel_range, gyro_range, no_reset, mag,
    raw, rate, count.  Returns a process exit code."""
    with open_bus(args.bus) as bus:
        dev = ICM20948(bus, address=args.address,
                       accel_range=args.accel_range, gyro_range=args.gyro_range)
        try:
            dev.begin(do_reset=not args.no_reset)
        except (OSError, RuntimeError) as e:
            print(f"Init failed: {e}")
            return 1
        if args.mag:
            try:
                dev.enable_magnetometer()
            except (OSError, RuntimeError) as e:
                print(f"Magnetometer init failed ({e}); continuing without it.")
        poll_loop(dev, rate=args.rate, count=args.count,
                  raw=args.raw, use_mag=args.mag)
    return 0


def build_parser():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-b", "--bus", type=int, default=1,
                   help="I2C bus number (default: 1, the RPi header bus)")
    p.add_argument("-a", "--address", type=lambda x: int(x, 0), default=0x68,
                   help="ICM-20948 address, 0x68 (AD0 low) or 0x69 (default: 0x68)")
    add_read_arguments(p)
    return p


def main(argv=None):
    return run_read(build_parser().parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
