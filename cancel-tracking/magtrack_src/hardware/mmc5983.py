"""The MMC5983MA magnetometer, over I2C.  Needs smbus2.

Each reading is a SET/RESET pair: the field flips sign in the sensor between the
two and the sensor's own drift doesn't, so half their difference is the field
alone, whatever the temperature.

Used by: hardware/mmc_loop.py.  Uses: smbus2, config.
"""

import time

from magtrack_src.config import MMC_BANDWIDTH_HZ

XOUT0, STATUS, CTRL0, CTRL1, PRODUCT_ID = 0x00, 0x08, 0x09, 0x0A, 0x2F
EXPECTED_ID = 0x30
M_DONE = 0x01                                 # STATUS
TM_M, SET, RESET = 0x01, 0x08, 0x10           # CTRL0; these clear themselves
SW_RESET = 0x80                               # CTRL1
BANDWIDTH_CODES = {100: 0, 200: 1, 400: 2, 800: 3}
COUNTS_PER_UT = 163.84                        # 18-bit, 16384 counts per gauss
COIL_S = 0.001                                # settling after a SET or RESET
TIMEOUT_S = 0.1


def counts(b):
    """X, Y, Z from the 7 output bytes: 18 bits, the low two of each in byte 6."""
    return ((b[0] << 10) | (b[1] << 2) | (b[6] >> 6),
            (b[2] << 10) | (b[3] << 2) | ((b[6] >> 4) & 3),
            (b[4] << 10) | (b[5] << 2) | ((b[6] >> 2) & 3))


def field_ut(after_set, after_reset):
    """The field from one SET/RESET pair.  Its axes are the chip's, not the board's."""
    return tuple((a - b) / 2.0 / COUNTS_PER_UT for a, b in zip(after_set, after_reset))


class MMC:
    """``read()`` gives the field (uT, chip axes) and the time it was measured."""

    def __init__(self, bus, address):
        from smbus2 import SMBus
        self.address = address
        self.bus = SMBus(bus)
        try:
            self._begin()
        except Exception:
            self.bus.close()
            raise

    def _read(self, reg, length):
        # A combined transaction keeps the repeated START the datasheet asks for.
        from smbus2 import i2c_msg
        w, r = i2c_msg.write(self.address, [reg]), i2c_msg.read(self.address, length)
        self.bus.i2c_rdwr(w, r)
        return list(r)

    def _write(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def _begin(self):
        ident = self._read(PRODUCT_ID, 1)[0]
        if ident != EXPECTED_ID:
            raise OSError(f"MMC5983MA ID = 0x{ident:02X}, expected 0x{EXPECTED_ID:02X}")
        self._write(CTRL1, SW_RESET)
        time.sleep(0.015)
        self._write(CTRL1, BANDWIDTH_CODES[MMC_BANDWIDTH_HZ])

    def _measure(self):
        self._write(STATUS, M_DONE)               # writing a 1 clears it
        self._write(CTRL0, TM_M)
        deadline = time.monotonic() + TIMEOUT_S
        while not self._read(STATUS, 1)[0] & M_DONE:
            if time.monotonic() > deadline:
                raise OSError("MMC5983MA measurement timed out")
            time.sleep(0.0002)
        t = time.monotonic()
        return counts(self._read(XOUT0, 7)), t

    def read(self):
        self._write(CTRL0, SET)
        time.sleep(COIL_S)
        after_set, t_set = self._measure()
        self._write(CTRL0, RESET)
        time.sleep(COIL_S)
        after_reset, t_reset = self._measure()
        return field_ut(after_set, after_reset), (t_set + t_reset) / 2.0

    def close(self):
        self.bus.close()
