import unittest

import imu
from imu import IMU, mag_to_body


class FakeBus:
    """Answers the two reads ``IMU.read`` makes."""
    def __init__(self, st1=0x01, st2=0x00):
        self.st1, self.st2 = st1, st2

    def read_byte_data(self, address, reg):
        return self.st1

    def read_i2c_block_data(self, address, reg, length):
        if address == imu.AK_ADDRESS:           # little-endian: x=100, y=-200, z=300 LSB
            return [0x64, 0x00, 0x38, 0xFF, 0x2C, 0x01, 0x00, self.st2]
        return [0x20, 0x00, 0, 0, 0, 0, 0x00, 0x83, 0, 0, 0, 0]     # big-endian


def fake_imu(**kw):
    s = IMU.__new__(IMU)
    s.address, s.bus = 0x68, FakeBus(**kw)
    return s


class Reading(unittest.TestCase):
    def test_magnetometer_axes_follow_the_datasheet(self):
        self.assertEqual(mag_to_body((1.0, 2.0, 3.0)), (1.0, -2.0, -3.0))

    def test_scaling_and_axes(self):
        accel, gyro, mag = fake_imu().read()
        self.assertEqual(accel[0], 1.0)                 # 0x2000 counts at +/- 4 g
        self.assertAlmostEqual(gyro[0], 131 / imu.GYRO_LSB_PER_DPS)
        self.assertEqual(mag, (100 * imu.AK_UT_PER_LSB, 200 * imu.AK_UT_PER_LSB, -300 * imu.AK_UT_PER_LSB))

    def test_no_fresh_sample(self):
        self.assertIsNone(fake_imu(st1=0).read()[2])

    def test_overflow_is_dropped(self):
        self.assertIsNone(fake_imu(st2=0x08).read()[2])
