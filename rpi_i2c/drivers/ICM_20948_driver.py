"""
ICM-20948 9-axis IMU driver (accelerometer, gyroscope, temperature).
Includes AK09916 internal magnetometer via the ICM-20948 I2C master.

I2C address: 0x68 (AD0=GND, default) or 0x69 (AD0=VCC)
WHO_AM_I:    0xEA
"""

import smbus2
import struct
import time

ICM20948_ADDR_LOW  = 0x68
ICM20948_ADDR_HIGH = 0x69

# REG_BANK_SEL exists at 0x7F in every bank; write (bank & 3) << 4 to select
REG_BANK_SEL = 0x7F

# ── Bank 0 ──────────────────────────────────────────────────────────────────
B0_WHO_AM_I              = 0x00  # Expected: 0xEA
B0_USER_CTRL             = 0x03
B0_PWR_MGMT_1            = 0x06
B0_PWR_MGMT_2            = 0x07
B0_ACCEL_XOUT_H          = 0x2D  # 6 bytes: XYZXYZ, big-endian
B0_GYRO_XOUT_H           = 0x33  # 6 bytes, big-endian
B0_TEMP_OUT_H            = 0x39  # 2 bytes, big-endian
B0_EXT_SLV_SENS_DATA_00  = 0x3B  # Auto-collected AK09916 data starts here

# ── Bank 2 ──────────────────────────────────────────────────────────────────
B2_GYRO_CONFIG_1  = 0x01  # [2:1]=GYRO_FS_SEL, [0]=GYRO_FCHOICE
B2_ACCEL_CONFIG   = 0x14  # [2:1]=ACCEL_FS_SEL, [0]=ACCEL_FCHOICE

# ── Bank 3 ──────────────────────────────────────────────────────────────────
B3_I2C_MST_CTRL    = 0x01
B3_I2C_SLV0_ADDR   = 0x03  # bit7=R/W̄, bits[6:0]=address
B3_I2C_SLV0_REG    = 0x04
B3_I2C_SLV0_CTRL   = 0x05  # bit7=EN, bits[3:0]=LENG
B3_I2C_SLV0_DO     = 0x06  # data-out for write transactions

# ── AK09916 (internal magnetometer, address 0x0C) ───────────────────────────
AK09916_ADDR  = 0x0C
AK09916_ST1   = 0x10  # Status 1: bit0=DRDY, bit1=DOR
AK09916_HXL   = 0x11  # X low (little-endian pairs: XL,XH,YL,YH,ZL,ZH)
AK09916_ST2   = 0x18  # Status 2: bit3=HOFL (overflow). Must read to unlock next.
AK09916_CNTL2 = 0x31  # Mode control
AK09916_CNTL3 = 0x32  # Soft reset (write 0x01)

AK09916_MODE_CONT4 = 0x08  # Continuous measurement mode 4 (100 Hz)


class ICM20948:
    def __init__(self, bus_num: int = 1, addr: int = ICM20948_ADDR_LOW):
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

    def _bank(self, b: int) -> None:
        self._wr(REG_BANK_SEL, (b & 0x03) << 4)

    # ── AK09916 access via I2C master ────────────────────────────────────────

    def _ak_write(self, reg: int, val: int) -> None:
        """Write one byte to AK09916 through the ICM-20948 I2C master."""
        self._bank(3)
        self._wr(B3_I2C_SLV0_ADDR, AK09916_ADDR)   # write direction (bit7=0)
        self._wr(B3_I2C_SLV0_REG,  reg)
        self._wr(B3_I2C_SLV0_DO,   val)
        self._wr(B3_I2C_SLV0_CTRL, 0x81)            # EN + length=1
        time.sleep(0.01)
        self._bank(0)

    # ── initialisation ───────────────────────────────────────────────────────

    def _init(self) -> None:
        self._bank(0)
        who = self._rd(B0_WHO_AM_I)
        if who != 0xEA:
            raise RuntimeError(
                f"ICM-20948 not found: WHO_AM_I=0x{who:02X} (expected 0xEA)"
            )

        # Soft reset; wait for bit to clear
        self._wr(B0_PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        # Wake, select auto-clock
        self._wr(B0_PWR_MGMT_1, 0x01)
        # Enable accel + gyro
        self._wr(B0_PWR_MGMT_2, 0x00)
        time.sleep(0.02)

        # Bank 2: ±2 g accel, ±250 dps gyro, DLPF on
        self._bank(2)
        self._wr(B2_ACCEL_CONFIG, 0x01)
        self._wr(B2_GYRO_CONFIG_1, 0x01)

        # Bank 0: enable I2C master controller
        self._bank(0)
        self._wr(B0_USER_CTRL, 0x20)

        # Bank 3: I2C master clock ~400 kHz
        self._bank(3)
        self._wr(B3_I2C_MST_CTRL, 0x17)

        # Soft-reset AK09916, then set continuous-100Hz mode
        self._ak_write(AK09916_CNTL3, 0x01)
        time.sleep(0.01)
        self._ak_write(AK09916_CNTL2, AK09916_MODE_CONT4)

        # Configure slave-0 to auto-read 9 bytes from AK09916 (ST1 … ST2)
        # on each sample cycle, depositing them into EXT_SLV_SENS_DATA_00+
        self._bank(3)
        self._wr(B3_I2C_SLV0_ADDR, 0x80 | AK09916_ADDR)  # read direction
        self._wr(B3_I2C_SLV0_REG,  AK09916_ST1)
        self._wr(B3_I2C_SLV0_CTRL, 0x89)                  # EN + length=9
        self._bank(0)

    # ── public API ───────────────────────────────────────────────────────────

    def read(self) -> dict:
        """
        Single I2C burst: reads accel, gyro, temp, and AK09916 mag in one call.

        Returns dict:
            accel   – {'raw': bytes(6), 'x': int, 'y': int, 'z': int}
            gyro    – {'raw': bytes(6), 'x': int, 'y': int, 'z': int}
            temp    – {'raw': bytes(2), 'value': int}
            mag     – {'raw': bytes(9), 'x': int, 'y': int, 'z': int,
                        'st1': int, 'st2': int}

        Accel/gyro/temp are big-endian signed 16-bit.
        Mag values are little-endian signed 16-bit (AK09916 native byte order).
        Temperature formula: T_degC = (T_raw / 333.87) + 21.0
        """
        self._bank(0)
        # 0x2D … 0x43  →  14 bytes (accel+gyro+temp) + 9 bytes (AK09916 via EXT)
        burst = self._rd_block(B0_ACCEL_XOUT_H, 23)

        accel_raw = burst[0:6]
        gyro_raw  = burst[6:12]
        temp_raw  = burst[12:14]
        mag_raw   = burst[14:23]  # ST1, HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2

        ax, ay, az = struct.unpack('>hhh', accel_raw)
        gx, gy, gz = struct.unpack('>hhh', gyro_raw)
        t,         = struct.unpack('>h',   temp_raw)

        st1 = mag_raw[0]
        mx, my, mz = struct.unpack('<hhh', mag_raw[1:7])
        st2 = mag_raw[8]

        return {
            'accel': {'raw': accel_raw, 'x': ax, 'y': ay, 'z': az},
            'gyro':  {'raw': gyro_raw,  'x': gx, 'y': gy, 'z': gz},
            'temp':  {'raw': temp_raw,  'value': t},
            'mag':   {'raw': mag_raw,   'x': mx, 'y': my, 'z': mz,
                      'st1': st1, 'st2': st2},
        }

    def close(self) -> None:
        self._bus.close()
