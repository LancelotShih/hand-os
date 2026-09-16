"""I2C bus discovery and scanning helpers.

These know nothing about the ICM-20948 specifically -- they just find the
``/dev/i2c-*`` character devices the kernel exposes and probe them for any
chip that acknowledges its address, the same way ``i2cdetect`` does.
"""

from __future__ import annotations

import glob
import sys

try:
    from smbus2 import SMBus
except ImportError:
    sys.exit("smbus2 is not installed.  Install it with:\n"
             "  sudo apt install python3-smbus2      # Debian/RaspiOS package\n"
             "  # or:  pip install smbus2")


def list_buses():
    """Return the sorted bus numbers of every /dev/i2c-N device present."""
    out = []
    for path in glob.glob("/dev/i2c-*"):
        try:
            out.append(int(path.rsplit("-", 1)[1]))
        except ValueError:
            pass
    return sorted(out)


def open_bus(busno):
    """Open an SMBus, exiting with an actionable message on the common failures
    (I2C not enabled, wrong bus number, missing group membership)."""
    try:
        return SMBus(busno)
    except FileNotFoundError:
        avail = list_buses()
        if avail:
            sys.exit(f"/dev/i2c-{busno} not found. Available buses: {avail}")
        sys.exit(
            f"/dev/i2c-{busno} not found and no I2C buses are enabled.\n"
            "On a Raspberry Pi, enable the header I2C bus and reboot:\n"
            "  sudo raspi-config nonint do_i2c 0\n"
            "  sudo reboot")
    except PermissionError:
        sys.exit(
            f"Permission denied opening /dev/i2c-{busno}.\n"
            "Add your user to the 'i2c' group, then log out and back in:\n"
            "  sudo usermod -aG i2c \"$USER\"")


def scan_bus(busno):
    """Probe every 7-bit address on the bus and return the list that responded."""
    found = []
    with open_bus(busno) as bus:
        for addr in range(0x03, 0x78):
            try:
                # 0x30-0x37 and 0x50-0x5F: quick-write probe (matches i2cdetect);
                # elsewhere a byte read is the safer probe.
                if 0x30 <= addr <= 0x37 or 0x50 <= addr <= 0x5F:
                    bus.write_quick(addr)
                else:
                    bus.read_byte(addr)
                found.append(addr)
            except OSError:
                pass
    return found
