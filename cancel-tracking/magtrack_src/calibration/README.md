# calibration

Things measured once and saved to `data/calibration/`. The fits do no I/O.

| File | Public names | Job |
|---|---|---|
| `tumble.py` | `Tumble`, `fit_tumble`, `coverage`, `fit_ellipsoid`, `midpoint_and_swing`, `spread_95` | While the board is turned through every direction, collects raw magnetometer readings and tracks which of 26 directions are covered. `fit_tumble` fits an axis-aligned ellipsoid: each axis's midpoint and stretch (`gains`). Uses no orientation. |
| `mounting.py` | `fit_axes` | From the same tumble plus the orientation at each reading: the rotation (quarter turns + tilt) from a magnetometer's chip axes to the board's. Applied to the MMC, and shown only for the ICM. |
| `store.py` | `CalibrationFile` | Loads and saves offset, gyro bias and gains, the mounting, and the last tumble's raw readings. |

- **Used by:** `tracking/tracker.py` (runs the tumble and applies the results), and
  `earth_cancel.py` (loads saved files at startup).
- **Settings:** `TUMBLE_*`, `PASS_UT`, file paths in `config.py`.
- **Tests:** `tests/test_tumble.py`, plus `tests/test_pair.py` for mounting.
- **Saved files:** see the Calibrations table in `docs/architecture.md`.
