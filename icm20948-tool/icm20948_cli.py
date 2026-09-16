#!/usr/bin/env python3
"""Command-line front end for the ICM-20948 tool.

This file is only argument parsing and dispatch -- every command delegates to
one of the other modules:

  buses    -> icm20948_i2c.list_buses
  scan     -> icm20948_i2c.scan_bus
  whoami   -> icm20948_driver.ICM20948.who_am_i
  read     -> icm20948_main.run_read  (the polling loop)

Examples:
  ./icm20948_cli.py buses
  ./icm20948_cli.py -b 1 scan
  ./icm20948_cli.py -b 1 whoami
  ./icm20948_cli.py -b 1 read -r 20 --mag
  ./icm20948_cli.py -b 1 -a 0x69 read --raw -n 5
"""

from __future__ import annotations

import argparse
import sys

from icm20948_driver import ICM20948
from icm20948_i2c import list_buses, open_bus, scan_bus
from icm20948_main import add_read_arguments, run_read
from icm20948_registers import WHO_AM_I_VALUE


def cmd_buses(_args):
    buses = list_buses()
    if not buses:
        print("No /dev/i2c-* buses. Enable I2C: sudo raspi-config nonint do_i2c 0 && sudo reboot")
        return 1
    for b in buses:
        print(f"/dev/i2c-{b}")
    return 0


def cmd_scan(args):
    found = scan_bus(args.bus)
    if not found:
        print(f"i2c-{args.bus}: no devices responded.")
        return 1
    print(f"i2c-{args.bus}: found {len(found)} device(s):")
    for addr in found:
        tag = ""
        if addr in (0x68, 0x69):
            tag = "  <- likely ICM-20948 (AD0 %s)" % ("low" if addr == 0x68 else "high")
        elif addr == 0x0C:
            tag = "  <- AK09916 magnetometer (ICM-20948 in bypass mode)"
        print(f"  0x{addr:02X}{tag}")
    return 0


def cmd_whoami(args):
    with open_bus(args.bus) as bus:
        dev = ICM20948(bus, address=args.address)
        try:
            who = dev.who_am_i()
        except OSError as e:
            print(f"No response at 0x{args.address:02X} on i2c-{args.bus} ({e}).")
            return 1
    ok = who == WHO_AM_I_VALUE
    print(f"i2c-{args.bus} @ 0x{args.address:02X}: WHO_AM_I = 0x{who:02X} "
          f"({'OK, ICM-20948 confirmed' if ok else 'unexpected'})")
    return 0 if ok else 1


def cmd_read(args):
    return run_read(args)


def build_parser():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-b", "--bus", type=int, default=1,
                   help="I2C bus number (default: 1, the RPi header bus)")
    p.add_argument("-a", "--address", type=lambda x: int(x, 0), default=0x68,
                   help="ICM-20948 address, 0x68 (AD0 low) or 0x69 (default: 0x68)")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("buses", help="list available /dev/i2c-* buses").set_defaults(func=cmd_buses)
    sub.add_parser("scan", help="probe the bus for devices").set_defaults(func=cmd_scan)
    sub.add_parser("whoami", help="read WHO_AM_I").set_defaults(func=cmd_whoami)

    rd = sub.add_parser("read", help="stream sensor data")
    add_read_arguments(rd)
    rd.set_defaults(func=cmd_read)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
