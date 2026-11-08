"""The ICM-20948 and its AK09916 magnetometer, over I2C.  Needs smbus2."""

import time

REG_BANK_SEL = 0x7F
PWR_MGMT_1, PWR_MGMT_2, USER_CTRL, INT_PIN_CFG = 0x06, 0x07, 0x03, 0x0F
WHO_AM_I, ACCEL_XOUT_H = 0x00, 0x2D
GYRO_CONFIG_1, ACCEL_CONFIG = 0x01, 0x14      # bank 2
AK_ADDRESS, AK_WIA2, AK_ST1, AK_HXL, AK_CNTL2, AK_CNTL3 = 0x0C, 0x01, 0x10, 0x11, 0x31, 0x32
AK_MODE_100HZ, AK_UT_PER_LSB = 0x08, 0.15
ACCEL_LSB_PER_G = 8192.0                      # +/- 4 g
GYRO_LSB_PER_DPS = 65.5                       # +/- 500 dps


def mag_to_body(m):
    """AK09916 axes -> ICM accel/gyro axes: X matches, Y and Z are reversed
    (ICM-20948 datasheet section 15, Figures 12 and 13).  Not the MPU-9250's
    (y, x, -z): that mismatch makes the three poses disagree by tens of uT."""
    return (m[0], -m[1], -m[2])


def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


class IMU:
    """``read()`` gives accel (g), gyro (dps) and magnetometer (uT, or ``None`` if
    there is no fresh sample), all in the accel/gyro axes."""

    def __init__(self, bus, address):
        from smbus2 import SMBus
        self.address = address
        self.bus = SMBus(bus)
        try:
            self._begin()
        except Exception:
            self.bus.close()
            raise

    def _bank(self, bank):
        self.bus.write_byte_data(self.address, REG_BANK_SEL, bank << 4)

    def _begin(self):
        w = self.bus.write_byte_data
        self._bank(0)
        who = self.bus.read_byte_data(self.address, WHO_AM_I)
        if who != 0xEA:
            raise OSError(f"WHO_AM_I = 0x{who:02X}, expected 0xEA")
        w(self.address, PWR_MGMT_1, 0x80)          # reset
        time.sleep(0.1)
        w(self.address, PWR_MGMT_1, 0x01)          # wake, auto clock
        time.sleep(0.01)
        w(self.address, PWR_MGMT_2, 0x00)          # all accel + gyro axes on
        time.sleep(0.01)
        self._bank(2)
        w(self.address, GYRO_CONFIG_1, 0x02 | 0x01)   # +/- 500 dps, low-pass on
        w(self.address, ACCEL_CONFIG, 0x02 | 0x01)    # +/- 4 g, low-pass on
        self._bank(0)
        w(self.address, USER_CTRL, 0x00)           # I2C bypass: the AK09916 joins this bus
        w(self.address, INT_PIN_CFG, 0x02)
        time.sleep(0.01)
        if self.bus.read_byte_data(AK_ADDRESS, AK_WIA2) != 0x09:
            raise OSError("no AK09916 magnetometer answering")
        w(AK_ADDRESS, AK_CNTL3, 0x01)              # soft reset
        time.sleep(0.01)
        w(AK_ADDRESS, AK_CNTL2, AK_MODE_100HZ)
        time.sleep(0.01)

    def read(self):
        d = self.bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 12)
        accel = tuple(_s16(d[i], d[i + 1]) / ACCEL_LSB_PER_G for i in (0, 2, 4))
        gyro = tuple(_s16(d[i], d[i + 1]) / GYRO_LSB_PER_DPS for i in (6, 8, 10))
        return accel, gyro, self._read_mag()

    def _read_mag(self):
        if not self.bus.read_byte_data(AK_ADDRESS, AK_ST1) & 0x01:
            return None                            # no fresh sample
        d = self.bus.read_i2c_block_data(AK_ADDRESS, AK_HXL, 8)   # HXL..ST2; ST2 unlatches
        if d[7] & 0x08:
            return None                            # overflow: the field is too strong
        return mag_to_body(tuple(_s16(d[i + 1], d[i]) * AK_UT_PER_LSB for i in (0, 2, 4)))

    def close(self):
        self.bus.close()
