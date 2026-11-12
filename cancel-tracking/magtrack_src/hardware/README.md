# hardware

The only code that talks to the chips over I2C (needs `smbus2`).

| File | Public names | Job |
|---|---|---|
| `icm20948.py` | `IMU`, `mag_to_body` | ICM-20948 + AK09916 driver. `IMU(bus, address).read()` → `(accel g, gyro °/s, mag µT or None)`, all in the board (accel/gyro) axes. Opening it **resets the chip**. |
| `mmc5983.py` | `MMC`, `counts`, `field_ut` | MMC5983MA driver. `read()` → `(field µT in chip axes, monotonic time)`, from one SET/RESET pair, which removes the chip's own drift. |
| `icm_loop.py` | `sensor_loop` | Reads the ICM forever on its own thread, reconnecting after errors. Hands each reading to the tracker, the log and the feed. |
| `mmc_loop.py` | `MmcReader` | Reads the MMC forever on its own thread. `take()` gives the newest unread reading. |

- **Used by:** `earth_cancel.py` only. No other package imports this one.
- **Settings:** `I2C_*`, `MMC_I2C_ADDRESS`, `MMC_BANDWIDTH_HZ` in `config.py`.
- **Tests:** `tests/test_imu.py` (fake bus). The loops are exercised in `test_server.py` and `test_pair.py`.
- **Watch out:** the axis map in `mag_to_body` is `(x, -y, -z)`, from the datasheet. Don't change it to the MPU-9250's `(y, x, -z)`.
