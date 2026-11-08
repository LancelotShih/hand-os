#!/usr/bin/env python3
"""Backwards-compatible facade for the ICM-20948 tool.

The implementation was split into focused modules; import from those directly
for new code:

  icm20948_registers.py  register map + full-scale / sensitivity tables
                         (the datasheet, as data -- no I/O)
  icm20948_i2c.py        /dev/i2c-* discovery and bus scanning
  icm20948_driver.py     the ICM20948 class: bring-up + measurements
  icm20948_main.py       the polling loop (poll_loop / run_read) and a
                         no-frills stream runner
  icm20948_cli.py        the full buses / scan / whoami / read CLI

This module just re-exports the common names and forwards `./icm20948.py ...`
to the CLI, so existing callers and scripts keep working.
"""

from icm20948_registers import *          # noqa: F401,F403  (register constants + scale tables)
from icm20948_driver import ICM20948      # noqa: F401
from icm20948_i2c import list_buses, open_bus, scan_bus  # noqa: F401
from icm20948_main import (               # noqa: F401
    format_raw, format_scaled, poll_loop, run_read,
)
from icm20948_cli import build_parser, main  # noqa: F401

if __name__ == "__main__":
    import sys
    sys.exit(main())
