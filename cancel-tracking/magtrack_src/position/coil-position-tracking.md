# Coil position tracking

Calculates a sensor position from the ICM's measured field minus its captured
reference field. `Locator` also reads and writes the saved coil-calibration JSON
file. Operating instructions and readout meanings are in the
[architecture](../../docs/architecture.md#position-setup).

`locate.py` contains:
- `Coil`: the coil modelled as a bar magnet (two poles), `field(p)` in µT at `p` in cm.
- `least_squares`: repeatedly adjusts parameters to reduce squared differences (Levenberg-Marquardt). It estimates how outputs change with small parameter changes, called a numeric Jacobian.
- `fit_coil(points, home)`: fits the coil's center, axis, strength and length from captured
  `(position, error)` points. A coarse grid search first, then refinement.
- `solve(coil, error, start, home_field)`: the position whose field matches, choosing
  the mirrored position closer to the previous estimate. `mm_per_ut`: local model sensitivity.
  `check_points` and `left_out_check`: local consistency reports, starting the position calculation at the
  entered coordinates; the latter starts refitting from the all-points model.
- `Locator`: the live part. `update(sample)` runs in the sensor loop (called by
  `tracking/pair.py`). The point-capture methods are for HTTP. Fits run on their own
  thread, and points plus the coil are saved to `data/calibration/coil_cal.json`.

- **Units:** cm and µT. Positions are from the coil's center (0), in the home axes (the board's
  axes when the reference field was captured on the home mark). `Locator.home` is the mark's position, set by the user.
- **Settings:** `LOCATE_*` in `config.py`.
- **Tests:** `tests/test_locate.py`.
- **Known limit:** the coil's field is symmetric about its long axis, so beside its
  middle a position can't be pinned down (`mm_per_ut` becomes very large). Home is currently
  broadside. A large movement-per-field-change value and the nominal working range do not stop position estimates from being published.

There is no explicit blind-spot rejection. `solve` chooses a matching estimate
using its starting point and the nearer mirror solution; `Locator.update` rejects
excessive field mismatch, not ambiguity. The output can therefore contain a
position even when several places match equally well. `mm_per_ut` describes local
sensitivity and does not identify the separate mirrored answer.

The current output has no history-assumption flag or dedicated blind-spot
warning. The page colors local sensitivity, but the solver still chooses a
position from history. Startup uses entered home coordinates.

`set_home` changes entered coordinates and refits; it does not take a reference
reading. Moving the physical mark changes the field used as the reference, unlike
correcting coordinates of the same mark. Existing point readings/model are not
automatically invalidated by reference or magnetometer-calibration changes.

`Locator` owns home/points/model, background fit/save workers, conversion of older files, and the previous position used to start the next position calculation. Fits run outside its state lock; the live solve
still runs on the acquisition thread. See the
[architecture](../../docs/architecture.md) for lifecycle details and the
[review](../../docs/architecture-review-2026-11-12.md) for revision/cancellation gaps.
