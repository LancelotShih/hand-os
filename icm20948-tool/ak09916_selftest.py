#!/usr/bin/env python3
"""AK09916 magnetometer self-test (the mag die inside the ICM-20948).

The AK09916 has a built-in current coil that lays a *known* magnetic field
across its Hall sensors.  Self-test energises that coil, takes one measurement,
and you compare the result against fixed datasheet bounds.

What it actually tells you
--------------------------
* The Hall elements, analog front end, and ADC of the magnetometer are alive and
  producing sane numbers.  This is a real functional check -- unlike WHO_AM_I /
  WIA2, which only prove the I2C register interface answers.
* It is largely independent of the ambient field: the internal coil dominates,
  so a pass/fail still means something on a cluttered bench.  (A very strong
  external field can still push a reading out of range or set the overflow flag.)

What it does NOT tell you
-------------------------
* It is not a calibration.  A pass means "inside a wide window", not "accurate".
  Hard-iron / soft-iron distortion still needs a separate calibration step.
* The window is loose -- roughly +/-30 uT on X/Y and -150..-30 uT on Z -- so a
  marginal or slowly drifting part can still pass.

When it is useful
-----------------
* Bring-up: separate "the mag is wired and working" from "the mag is just
  reading noise / a stuck value".
* Field diagnostics: if mag readings look dead or frozen, run this to decide
  whether the sensor or the surrounding code is at fault.
* Production end-of-line: the datasheet explicitly intends this for a go/no-go
  test on finished products.

Datasheet sequence (AK09916 self-test):
  1. power-down mode            CNTL2 <- 0x00
  2. wait >= 100 us
  3. self-test mode             CNTL2 <- 0x10   (energises the internal coil)
  4. poll ST1.DRDY until set    (one measurement, ~8 ms)
  5. read HXL..HZH  (+ ST2 to release the latch; ST2.HOFL bit = overflow)
  6. part auto-returns to power-down mode
  7. pass if, in raw LSB:  -200 <= HX <= 200,  -200 <= HY <= 200,
                           -1000 <= HZ <= -200

Usage:
  ./ak09916_selftest.py                 # bus 1, ICM at 0x68
  ./ak09916_selftest.py -b 1 -a 0x69
  ./ak09916_selftest.py --restore       # re-enable 100 Hz continuous mode after
"""

from __future__ import annotations

import argparse
import sys
import time

from icm20948_driver import ICM20948
from icm20948_i2c import open_bus
from icm20948_registers import (
    AK_ADDRESS, AK_CNTL2, AK_HXL, AK_MODE_100HZ, AK_SENS_UT, AK_ST1,
)

# --- self-test additions to the AK09916 map --------------------------------
# Promote these into icm20948_registers.py if the self-test proves worth keeping.
AK_ST2             = 0x18   # bit 3 = HOFL (magnetic overflow); reading it frees the latch
AK_MODE_POWER_DOWN = 0x00
AK_MODE_SELF_TEST  = 0x10

# Datasheet pass window, in raw LSB.  Sensitivity is AK_SENS_UT = 0.15 uT/LSB.
SELF_TEST_BOUNDS = {
    "x": (-200, 200),
    "y": (-200, 200),
    "z": (-1000, -200),
}


def _s16_le(lo, hi):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


def run_self_test(bus, address=0x68, drdy_timeout=0.2, restore=False):
    """Run one AK09916 self-test cycle and return a result dict:

        {"raw": {x,y,z}, "ut": {x,y,z}, "overflow": bool,
         "checks": {axis: (ok, (lo, hi))}, "passed": bool}
    """
    with ICM20948(bus, address=address) as imu:
        imu.begin()
        imu.enable_magnetometer()          # bypass on, AK soft-reset, WIA2 verified
        b = imu._bus                       # same primary bus; AK09916 now visible at 0x0C

        # enable_magnetometer() left the mag in 100 Hz continuous mode.  Drain any
        # already-latched sample and stop continuous conversions, so the DRDY poll
        # below can only ever observe the single self-test measurement.
        b.read_i2c_block_data(AK_ADDRESS, AK_HXL, 8)   # read through ST2 clears DRDY
        b.write_byte_data(AK_ADDRESS, AK_CNTL2, AK_MODE_POWER_DOWN)
        time.sleep(0.001)                  # datasheet: wait >= 100 us in power-down
        b.write_byte_data(AK_ADDRESS, AK_CNTL2, AK_MODE_SELF_TEST)

        deadline = time.monotonic() + drdy_timeout
        while not (b.read_byte_data(AK_ADDRESS, AK_ST1) & 0x01):
            if time.monotonic() > deadline:
                raise RuntimeError("DRDY never asserted -- self-test measurement "
                                   "did not complete")
            time.sleep(0.001)

        d = b.read_i2c_block_data(AK_ADDRESS, AK_HXL, 8)   # HXL..ST2
        raw = {
            "x": _s16_le(d[0], d[1]),
            "y": _s16_le(d[2], d[3]),
            "z": _s16_le(d[4], d[5]),
        }
        overflow = bool(d[7] & 0x08)       # ST2.HOFL

        if restore:
            b.write_byte_data(AK_ADDRESS, AK_CNTL2, AK_MODE_100HZ)
        # else: the part is already back in power-down mode

    checks = {
        axis: (lo <= raw[axis] <= hi, (lo, hi))
        for axis, (lo, hi) in SELF_TEST_BOUNDS.items()
    }
    passed = not overflow and all(ok for ok, _ in checks.values())
    return {
        "raw": raw,
        "ut": {k: v * AK_SENS_UT for k, v in raw.items()},
        "overflow": overflow,
        "checks": checks,
        "passed": passed,
    }


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-b", "--bus", type=int, default=1,
                   help="I2C bus number (default: 1)")
    p.add_argument("-a", "--address", type=lambda x: int(x, 0), default=0x68,
                   help="ICM-20948 address, 0x68 or 0x69 (default: 0x68)")
    p.add_argument("--restore", action="store_true",
                   help="leave the magnetometer in 100 Hz continuous mode afterwards "
                        "(default: leave it powered down, where the self-test ends)")
    args = p.parse_args(argv)

    with open_bus(args.bus) as bus:
        try:
            res = run_self_test(bus, address=args.address, restore=args.restore)
        except OSError as e:
            print(f"I2C error on i2c-{args.bus} @ 0x{args.address:02X}: {e}")
            return 2
        except RuntimeError as e:
            print(f"Self-test could not run: {e}")
            return 2

    print(f"AK09916 self-test on i2c-{args.bus} (ICM @ 0x{args.address:02X})")
    print(f"  {'axis':<4} {'raw':>7} {'uT':>9}   {'bounds(raw)':>13}   result")
    for axis in ("x", "y", "z"):
        ok, (lo, hi) = res["checks"][axis]
        print(f"  {axis:<4} {res['raw'][axis]:>7d} {res['ut'][axis]:>9.2f}   "
              f"[{lo:>5d},{hi:>5d}]   {'pass' if ok else 'FAIL'}")
    if res["overflow"]:
        print("  ST2.HOFL set: magnetic overflow during the measurement")
    print(f"\n  => {'PASS' if res['passed'] else 'FAIL'}")
    return 0 if res["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
