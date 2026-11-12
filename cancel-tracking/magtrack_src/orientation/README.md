# orientation

How the board is turned, from the gyro and accelerometer only. The coil spoils the
magnetometer, so it's never used for orientation.

| File | Public names | Job |
|---|---|---|
| `madgwick.py` | `madgwick_step` | One 6-DOF Madgwick update. Gyro in rad/s, accel in any unit. Gain `AHRS_BETA`. |
| `gyro_bias.py` | `GyroRest` | Feeds on every gyro reading. When the board has rested for `REST_S`, returns the average as the new bias. |

- **Used by:** `tracking/tracker.py`, which owns the quaternion, the bias and the orientation history.
- **Settings:** `AHRS_BETA`, `REST_*` in `config.py`.
- **Tests:** `tests/test_tracker.py`.
- **Known limits:** heading only comes from the gyro, so it drifts. The bias update
  can learn a slow steady turn, and the accelerometer is uncalibrated and always
  trusted. See `docs/critique-2026-11-03.md` points 1-3.
