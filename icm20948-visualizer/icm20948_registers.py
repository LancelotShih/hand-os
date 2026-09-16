"""ICM-20948 register map and scale tables -- the datasheet, expressed as data.

Nothing in here does any I/O.  It is pure reference material that the driver
(:mod:`icm20948_driver`) reads from, split out so the numbers live in one place
and are easy to check against the TDK/InvenSense datasheet.

The ICM-20948 register space is *bank-selected*: you write the desired bank to
``REG_BANK_SEL`` and then the register numbers below mean whatever that bank
says they mean.  Constants here are grouped and prefixed by bank (``B0_``,
``B2_``) to make that explicit.
"""

from __future__ import annotations

# Bank selector -- present in every bank at the same address.
REG_BANK_SEL = 0x7F

# --- Bank 0: identity, power, data output -------------------------------------
B0_WHO_AM_I     = 0x00
B0_USER_CTRL    = 0x03
B0_PWR_MGMT_1   = 0x06
B0_PWR_MGMT_2   = 0x07
B0_INT_PIN_CFG  = 0x0F
B0_ACCEL_XOUT_H = 0x2D   # start of a 14-byte block: accel[6] gyro[6] temp[2]
B0_TEMP_OUT_H   = 0x39

# --- Bank 2: sample rate and full-scale configuration -----------------------
B2_GYRO_SMPLRT_DIV    = 0x00
B2_GYRO_CONFIG_1      = 0x01
B2_ACCEL_SMPLRT_DIV_1 = 0x10
B2_ACCEL_SMPLRT_DIV_2 = 0x11
B2_ACCEL_CONFIG       = 0x14

# Value the WHO_AM_I register returns for a genuine ICM-20948.
WHO_AM_I_VALUE = 0xEA

# --- AK09916 magnetometer --------------------------------------------------
# A separate die inside the ICM-20948 package.  Reached directly on the primary
# I2C bus once the ICM is put into bypass mode (see driver.enable_magnetometer).
AK_ADDRESS    = 0x0C
AK_WIA2       = 0x01
AK_WIA2_VALUE = 0x09
AK_ST1        = 0x10
AK_HXL        = 0x11
AK_CNTL2      = 0x31
AK_CNTL3      = 0x32
AK_MODE_100HZ = 0x08
AK_SENS_UT    = 0.15     # microtesla per LSB

# --- Full-scale select -> (config register bits[2:1], sensitivity LSB/unit) --
# Pick a range, write the bits into ACCEL_CONFIG / GYRO_CONFIG_1, then divide
# raw counts by the matching sensitivity to get g / dps.
ACCEL_FS   = {2: 0x00, 4: 0x02, 8: 0x04, 16: 0x06}
ACCEL_SENS = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
GYRO_FS    = {250: 0x00, 500: 0x02, 1000: 0x04, 2000: 0x06}
GYRO_SENS  = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}

# --- Temperature (datasheet formula: degC = raw / TEMP_SENS + TEMP_OFFSET_C) --
TEMP_SENS     = 333.87   # LSB per degC
TEMP_OFFSET_C = 21.0
