"""
TLV493D-A1B6 3D magnetic sensor driver.

TLV493D uses a register-less I2C protocol:
  - Read:  send device address + R, receive N bytes directly (no register prefix)
  - Write: send device address + W, send 4 config bytes directly

I2C address: 0x5E (ADDR pin = GND, default) or 0x1F (ADDR pin = VCC)

Read frame (10 bytes):
  Byte 0:  Bx[11:4]
  Byte 1:  By[11:4]
  Byte 2:  Bz[11:4]
  Byte 3:  Temp[11:4]
  Byte 4:  [7:4]=Bx[3:0], [3:0]=By[3:0]
  Byte 5:  [7:6]=ID (factory), [5:4]=reserved, [3:0]=Bz[3:0]
  Byte 6:  [7:4]=reserved, [3:0]=Temp[3:0]
  Byte 7:  factory config (MOD1 shadow – preserve bits [5:4])
  Byte 8:  factory config (MOD2 shadow – preserve bits [4:0])
  Byte 9:  factory config (reserved)

Write frame (4 bytes, NO register address prefix):
  Byte 0 (MOD1):
    [7]   FP    – frame parity; set so total 1-count across all 4 bytes is even
    [6:5] IICADR – I2C address select (00=0x5E, 01=0x1F, 10=0x5A, 11=0x44)
    [4]   INT   – INT pin enable (0=disabled)
    [3]   FAST  – fast-mode enable (0=off)
    [2:1] LP    – low-power mode (01 = Low Power 1, ~3 ms period)
    [0]   reserved
  Byte 1: preserve factory[7] bits [5:4], zero all others
  Byte 2 (MOD2):
    [7]   T_DIS – temperature disable (0=enabled)
    [6:5] DT,AM – default 0
    [4:1] LP period divider – 0
    [0]   PRD_M – 0
  Byte 3: preserve factory[8] bits [4:0], zero all others
"""

import smbus2
import time

TLV493D_ADDR_DEFAULT = 0x5E  # ADDR pin = GND (default)
TLV493D_ADDR_ALT     = 0x1F  # ADDR pin = VCC

_READ_LEN  = 10
_WRITE_LEN = 4


def _count_ones(data: list) -> int:
    return sum(bin(b).count('1') for b in data)


class TLV493D:
    def __init__(self, bus_num: int = 1, addr: int = TLV493D_ADDR_DEFAULT):
        self._bus  = smbus2.SMBus(bus_num)
        self._addr = addr
        self._init()

    # ── low-level raw I2C (no register address) ──────────────────────────────

    def _raw_read(self, n: int) -> bytes:
        msg = smbus2.i2c_msg.read(self._addr, n)
        self._bus.i2c_rdwr(msg)
        return bytes(msg)

    def _raw_write(self, data: list) -> None:
        msg = smbus2.i2c_msg.write(self._addr, data)
        self._bus.i2c_rdwr(msg)

    # ── initialisation ───────────────────────────────────────────────────────

    def _init(self) -> None:
        # Read 10 bytes to obtain factory-programmed values we must preserve
        factory = list(self._raw_read(_READ_LEN))

        # Build write frame
        # LP=01 → Low Power Mode 1 (~3 ms measurement period, ~333 Hz max)
        mod1 = 0x02                   # LP[2:1] = 01, FAST=0, INT=0, IICADR=00
        b1   = factory[7] & 0x18     # preserve factory bits [4:3]
        mod2 = 0x00                   # temperature enabled, defaults
        b3   = factory[8] & 0x1F     # preserve factory bits [4:0]

        frame = [mod1, b1, mod2, b3]

        # Even parity: set FP so the total number of 1-bits is even
        if _count_ones(frame) % 2 != 0:
            frame[0] |= 0x80

        self._raw_write(frame)
        time.sleep(0.005)

    # ── public API ───────────────────────────────────────────────────────────

    def read(self) -> dict:
        """
        Read all sensor data in one I2C transaction (10 bytes).

        Returns dict:
            mag  – {'raw': bytes(10), 'x': int, 'y': int, 'z': int}
            temp – {'raw': int}

        x/y/z are signed 12-bit values (range –2048 … +2047).
        temp is a raw unsigned 12-bit value.
        Temperature formula (approximate): T_degC = (raw - 340) / 11.79 + 25
        """
        d = list(self._raw_read(_READ_LEN))

        x    = (d[0] << 4) | (d[4] >> 4)
        y    = (d[1] << 4) | (d[4] & 0x0F)
        z    = (d[2] << 4) | (d[5] & 0x0F)
        temp = (d[3] << 4) | (d[6] & 0x0F)

        # Convert to signed 12-bit
        if x >= 2048: x -= 4096
        if y >= 2048: y -= 4096
        if z >= 2048: z -= 4096

        return {
            'mag':  {'raw': bytes(d), 'x': x, 'y': y, 'z': z},
            'temp': {'raw': temp},
        }

    def close(self) -> None:
        self._bus.close()
