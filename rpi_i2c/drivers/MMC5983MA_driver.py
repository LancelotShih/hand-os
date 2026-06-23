"""
MMC5983MA 3-axis magnetometer driver (18-bit resolution).

I2C address: 0x30 (fixed)
Product ID:  0x30  (register 0x2F)

Datasheet: https://www.memsic.com/magnetometer-sensor-15.html
"""

import smbus2
import time

MMC5983MA_ADDR = 0x30

# ── Registers ────────────────────────────────────────────────────────────────
REG_XOUT0   = 0x00  # X [17:10]
REG_XOUT1   = 0x01  # X [9:2]
REG_YOUT0   = 0x02  # Y [17:10]
REG_YOUT1   = 0x03  # Y [9:2]
REG_ZOUT0   = 0x04  # Z [17:10]
REG_ZOUT1   = 0x05  # Z [9:2]
REG_XYZOUT2 = 0x06  # X[1:0] in [7:6], Y[1:0] in [5:4], Z[1:0] in [3:2]
REG_TOUT    = 0x07  # Temperature (unsigned 8-bit, ~0.8°C/LSB, 0 = -75°C)
REG_STATUS  = 0x08  # [0]=Meas_M_Done, [1]=Meas_T_Done, [2]=OTP_Read_Done
REG_CTRL0   = 0x09
REG_CTRL1   = 0x0A
REG_CTRL2   = 0x0B
REG_CTRL3   = 0x0C  # [0]=ST_ENP, [1]=ST_ENM, [2]=SPI_3W
REG_PRODUCT = 0x2F  # Product ID: expected 0x30

# ── CTRL0 bits ───────────────────────────────────────────────────────────────
CTRL0_TM_M       = 0x01  # Take single magnetic measurement
CTRL0_TM_T       = 0x02  # Take single temperature measurement
CTRL0_DO_SET     = 0x08  # Perform SET (restore magnetisation)
CTRL0_DO_RESET   = 0x10  # Perform RESET (reverse magnetisation)
CTRL0_AUTO_SR_EN = 0x20  # Auto set/reset in continuous mode
CTRL0_CMM_FREQ_EN = 0x80 # Required to enable CMM (must also set CMM in CTRL2)

# ── CTRL1 bits ───────────────────────────────────────────────────────────────
# [1:0] BW – measurement bandwidth / duration
#   00 = 6.6 ms  (max 100 Hz)
#   01 = 3.5 ms  (max 200 Hz)
#   10 = 2.0 ms  (max 400 Hz)
#   11 = 1.2 ms  (max 800 Hz)
CTRL1_BW_6MS  = 0x00
CTRL1_SW_RST  = 0x80

# ── CTRL2 bits ───────────────────────────────────────────────────────────────
# [2:0] CMM_FREQ – ODR in continuous mode
#   000=1 Hz, 001=10 Hz, 010=20 Hz, 011=50 Hz, 100=100 Hz, 1xx=200 Hz
# [3] CMM_EN – enable continuous measurement mode
# [4] EN_PRD_SET – periodic set/reset enable
# [7:5] PRD_SET – how often to do periodic set (0=every 1, 1=every 25, ...)
CTRL2_CMM_100HZ = 0x04  # CMM_FREQ = 100
CTRL2_CMM_EN    = 0x08  # CMM_EN


class MMC5983MA:
    def __init__(self, bus_num: int = 1, addr: int = MMC5983MA_ADDR):
        self._bus  = smbus2.SMBus(bus_num)
        self._addr = addr
        self._init()

    # ── low-level helpers ────────────────────────────────────────────────────

    def _wr(self, reg: int, val: int) -> None:
        self._bus.write_byte_data(self._addr, reg, val)

    def _rd(self, reg: int) -> int:
        return self._bus.read_byte_data(self._addr, reg)

    def _rd_block(self, reg: int, n: int) -> bytes:
        return bytes(self._bus.read_i2c_block_data(self._addr, reg, n))

    # ── initialisation ───────────────────────────────────────────────────────

    def _init(self) -> None:
        pid = self._rd(REG_PRODUCT)
        if pid != 0x30:
            raise RuntimeError(
                f"MMC5983MA not found: Product ID=0x{pid:02X} (expected 0x30)"
            )

        # Software reset
        self._wr(REG_CTRL1, CTRL1_SW_RST)
        time.sleep(0.01)

        # Bandwidth: 6.6 ms (adequate for 100 Hz ODR)
        self._wr(REG_CTRL1, CTRL1_BW_6MS)

        # Perform initial SET to remove residual magnetisation
        self._wr(REG_CTRL0, CTRL0_DO_SET)
        time.sleep(0.001)

        # Enable auto set/reset + the CMM frequency enable bit
        self._wr(REG_CTRL0, CTRL0_AUTO_SR_EN | CTRL0_CMM_FREQ_EN)

        # 100 Hz continuous mode
        self._wr(REG_CTRL2, CTRL2_CMM_100HZ | CTRL2_CMM_EN)
        time.sleep(0.02)  # allow first measurement to complete

    # ── public API ───────────────────────────────────────────────────────────

    def read(self) -> dict:
        """
        Read 7 bytes of magnetic data (one I2C burst).

        Returns dict:
            mag  – {'raw': bytes(7), 'x': int, 'y': int, 'z': int}

        x/y/z are signed 18-bit values centred at zero.
        The raw 18-bit unsigned range is 0…262143; null-field output is ≈131072.
        Physical scale: 0.0625 µT / LSB (i.e. ±8191 µT full-scale).
        """
        d = self._rd_block(REG_XOUT0, 7)

        # Assemble 18-bit values: [17:10] from byte n, [9:2] from byte n+1,
        # [1:0] packed into byte 6 for all three axes.
        x = (d[0] << 10) | (d[1] << 2) | ((d[6] >> 6) & 0x03)
        y = (d[2] << 10) | (d[3] << 2) | ((d[6] >> 4) & 0x03)
        z = (d[4] << 10) | (d[5] << 2) | ((d[6] >> 2) & 0x03)

        # Shift to signed: subtract the null-field centre (2^17 = 131072)
        x -= 131072
        y -= 131072
        z -= 131072

        return {
            'mag': {'raw': bytes(d), 'x': x, 'y': y, 'z': z},
        }

    def read_temperature(self) -> dict:
        """
        Trigger a one-shot temperature measurement and return the result.
        Blocks ~10 ms while the measurement completes.

        Returns dict:
            temp – {'raw': int, 'celsius': float}
        Temperature formula: T_degC = (raw * 0.8) - 75
        """
        self._wr(REG_CTRL0, CTRL0_TM_T)
        deadline = time.time() + 0.1
        while time.time() < deadline:
            if self._rd(REG_STATUS) & 0x02:
                break
            time.sleep(0.002)

        raw = self._rd(REG_TOUT)
        return {'temp': {'raw': raw, 'celsius': raw * 0.8 - 75.0}}

    def close(self) -> None:
        self._bus.close()
