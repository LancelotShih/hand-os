# Orientation estimation

How the board is turned, from the gyro and accelerometer only. The coil adds a position-dependent magnetic field, so this implementation does
not use magnetometer readings to estimate orientation.

| File | Public names | Job |
|---|---|---|
| `madgwick.py` | `madgwick_step` | One Madgwick orientation update using the three gyroscope and three accelerometer axes. Gyro in rad/s, accel in any unit. Gain `AHRS_BETA`. |
| `gyro_bias.py` | `GyroRest` | Feeds on every gyro reading. Returns a mean after a sufficiently quiet, low-rate window of `REST_S`; quiet readings are treated as rest but cannot prove the board is still. |

- **Used by:** `tracking/tracker.py`, which owns the quaternion (four numbers representing rotation), gyroscope zero-rate estimate, and orientation history.
- **Settings:** `AHRS_BETA`, `REST_*` in `config.py`.
- **Tests:** `tests/test_tracker.py`.
- **Known limits:** heading (rotation around vertical) only comes from the gyro, so it can gradually drift away from the actual heading. The automatic gyroscope zero-rate update
  can learn a slow steady turn, and the accelerometer is uncalibrated and always
  trusted when nonzero. Bias windows can span missing readings. Re-capturing a
  field reference does not independently measure heading. See the current
  [review](../../docs/architecture-review-2026-11-12.md), R8.

The two magnetometers currently own separate copies of this state in `Tracker`.
Each tracker looks up orientation at its magnetic reading's estimated measurement
time, rather than rotating all readings using the latest quaternion.
