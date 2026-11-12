# position

The ICM's Earth-cancelled error in, position out. No I/O except `Locator`'s own JSON file.

`locate.py` contains:
- `Coil`: the coil modelled as a bar magnet (two poles), `field(p)` in µT at `p` in cm.
- `least_squares`: Levenberg-Marquardt with a numeric Jacobian.
- `fit_coil(points)`: fits the coil's center, axis, strength and length from captured
  `(position, error)` points. A coarse grid search first, then refinement.
- `solve(coil, error, start)`: the position whose field matches, picking the mirror
  side nearer the last position. `mm_per_ut`: local precision.
  `check_points` and `left_out_check`: accuracy reports.
- `Locator`: the live part. `update(sample)` runs in the sensor loop (called by
  `tracking/pair.py`). The point-capture methods are for HTTP. Fits run on their own
  thread, and points plus the coil are saved to `data/calibration/coil_cal.json`.

- **Units:** cm and µT. Positions are from the coil's center (0), in the home axes (the board's
  axes when Earth was captured on the home mark). `Locator.home` is the mark's position, set by the user.
- **Settings:** `LOCATE_*` in `config.py`.
- **Tests:** `tests/test_locate.py`.
- **Known limit:** the coil's field is symmetric about its long axis, so beside its
  middle a position can't be pinned down (`mm_per_ut` explodes). Home is currently
  broadside. 470 lines in one file: a candidate to split into model, fit and live tracking.
