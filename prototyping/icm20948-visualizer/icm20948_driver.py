"""The ICM-20948 driver class.

This is the only file that actually talks to the sensor.  It depends on the
register numbers in :mod:`icm20948_registers` and on ``smbus2`` for the I2C
transport; it does not import argparse, sockets, or anything else.

Lifecycle:

    with ICM20948(bus=1) as imu:   # or pass an already-open SMBus
        imu.begin()                # check WHO_AM_I, reset, wake, configure
        imu.enable_magnetometer()  # optional
        print(imu.read())          # scaled reading dict
"""

from __future__ import annotations

import sys
import time

try:
    from smbus2 import SMBus
except ImportError:
    sys.exit("smbus2 is not installed.  Install it with:\n"
             "  sudo apt install python3-smbus2      # Debian/RaspiOS package\n"
             "  # or:  pip install smbus2")

from icm20948_registers import (
    ACCEL_FS, ACCEL_SENS, GYRO_FS, GYRO_SENS,
    AK_ADDRESS, AK_CNTL2, AK_CNTL3, AK_HXL, AK_MODE_100HZ, AK_SENS_UT,
    AK_ST1, AK_WIA2, AK_WIA2_VALUE,
    B0_ACCEL_XOUT_H, B0_INT_PIN_CFG, B0_PWR_MGMT_1, B0_PWR_MGMT_2,
    B0_USER_CTRL, B0_WHO_AM_I,
    B2_ACCEL_CONFIG, B2_ACCEL_SMPLRT_DIV_1, B2_ACCEL_SMPLRT_DIV_2,
    B2_GYRO_CONFIG_1, B2_GYRO_SMPLRT_DIV,
    REG_BANK_SEL, TEMP_OFFSET_C, TEMP_SENS, WHO_AM_I_VALUE,
)


class ICM20948:
    def __init__(self, bus, address=0x68, accel_range=4, gyro_range=500):
        self._bus = SMBus(bus) if isinstance(bus, int) else bus
        self._owns_bus = isinstance(bus, int)
        self.address = address
        self.accel_range = accel_range
        self.gyro_range = gyro_range
        self._bank = None
        self.mag_enabled = False

    # -- context manager / cleanup --
    def close(self):
        if self._owns_bus:
            self._bus.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- low-level register access --
    def _set_bank(self, bank):
        if bank != self._bank:
            self._bus.write_byte_data(self.address, REG_BANK_SEL, bank << 4)
            self._bank = bank

    def _read(self, bank, reg):
        self._set_bank(bank)
        return self._bus.read_byte_data(self.address, reg)

    def _write(self, bank, reg, value):
        self._set_bank(bank)
        self._bus.write_byte_data(self.address, reg, value)

    def _read_block(self, bank, reg, length):
        self._set_bank(bank)
        return self._bus.read_i2c_block_data(self.address, reg, length)

    # -- identification / bring-up --
    def who_am_i(self):
        return self._read(0, B0_WHO_AM_I)

    def reset(self):
        self._write(0, B0_PWR_MGMT_1, 0x80)   # DEVICE_RESET
        time.sleep(0.1)
        self._bank = None                     # bank is 0 after reset, force re-select

    def wake(self):
        self._write(0, B0_PWR_MGMT_1, 0x01)   # CLKSEL=auto, clear SLEEP
        time.sleep(0.01)
        self._write(0, B0_PWR_MGMT_2, 0x00)   # enable all accel + gyro axes
        time.sleep(0.01)

    def configure(self):
        if self.accel_range not in ACCEL_FS:
            raise ValueError(f"accel_range must be one of {sorted(ACCEL_FS)}")
        if self.gyro_range not in GYRO_FS:
            raise ValueError(f"gyro_range must be one of {sorted(GYRO_FS)}")
        # gyro: full scale + DLPF enabled, no extra sample-rate division
        self._write(2, B2_GYRO_CONFIG_1, GYRO_FS[self.gyro_range] | 0x01)
        self._write(2, B2_GYRO_SMPLRT_DIV, 0x00)
        # accel: full scale + DLPF enabled, no extra sample-rate division
        self._write(2, B2_ACCEL_CONFIG, ACCEL_FS[self.accel_range] | 0x01)
        self._write(2, B2_ACCEL_SMPLRT_DIV_1, 0x00)
        self._write(2, B2_ACCEL_SMPLRT_DIV_2, 0x00)
        self._set_bank(0)

    def begin(self, do_reset=True):
        who = self.who_am_i()
        if who != WHO_AM_I_VALUE:
            raise RuntimeError(
                f"WHO_AM_I = 0x{who:02X}, expected 0x{WHO_AM_I_VALUE:02X} at "
                f"address 0x{self.address:02X} -- not an ICM-20948?")
        if do_reset:
            self.reset()
        self.wake()
        self.configure()

    # -- measurements --
    @staticmethod
    def _s16_be(hi, lo):
        v = (hi << 8) | lo
        return v - 65536 if v & 0x8000 else v

    def read_raw(self):
        d = self._read_block(0, B0_ACCEL_XOUT_H, 14)
        accel = (self._s16_be(d[0], d[1]), self._s16_be(d[2], d[3]), self._s16_be(d[4], d[5]))
        gyro  = (self._s16_be(d[6], d[7]), self._s16_be(d[8], d[9]), self._s16_be(d[10], d[11]))
        temp  = self._s16_be(d[12], d[13])
        return accel, gyro, temp

    def read(self):
        accel, gyro, temp = self.read_raw()
        a = ACCEL_SENS[self.accel_range]
        g = GYRO_SENS[self.gyro_range]
        out = {
            "accel_g":  tuple(v / a for v in accel),
            "gyro_dps": tuple(v / g for v in gyro),
            "temp_c":   temp / TEMP_SENS + TEMP_OFFSET_C,
        }
        if self.mag_enabled:
            out["mag_ut"] = self.read_mag()
        return out

    # -- AK09916 magnetometer (via I2C bypass) --
    def enable_magnetometer(self):
        self._write(0, B0_USER_CTRL, 0x00)      # disable ICM internal I2C master
        self._write(0, B0_INT_PIN_CFG, 0x02)    # BYPASS_EN=1: aux bus joins primary bus
        time.sleep(0.01)
        wia = self._bus.read_byte_data(AK_ADDRESS, AK_WIA2)
        if wia != AK_WIA2_VALUE:
            raise RuntimeError(
                f"AK09916 WIA2 = 0x{wia:02X}, expected 0x{AK_WIA2_VALUE:02X}")
        self._bus.write_byte_data(AK_ADDRESS, AK_CNTL3, 0x01)   # soft reset
        time.sleep(0.01)
        self._bus.write_byte_data(AK_ADDRESS, AK_CNTL2, AK_MODE_100HZ)
        time.sleep(0.01)
        self.mag_enabled = True

    def read_mag(self):
        st1 = self._bus.read_byte_data(AK_ADDRESS, AK_ST1)
        if not (st1 & 0x01):
            return None                          # DRDY low: no fresh sample
        d = self._bus.read_i2c_block_data(AK_ADDRESS, AK_HXL, 8)  # HXL..ST2
        # AK09916 sample output is little-endian
        def s16_le(lo, hi):
            v = (hi << 8) | lo
            return v - 65536 if v & 0x8000 else v
        mx = s16_le(d[0], d[1])
        my = s16_le(d[2], d[3])
        mz = s16_le(d[4], d[5])
        # d[7] is ST2; reading it (done above) releases the measurement latch
        return (mx * AK_SENS_UT, my * AK_SENS_UT, mz * AK_SENS_UT)
