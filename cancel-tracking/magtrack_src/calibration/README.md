# calibration

Things measured once and saved to `data/calibration/`. The fits do no I/O.

| File | Public names | Job |
|---|---|---|
| `tumble.py` | `Tumble`, `fit_tumble`, `coverage`, `fit_ellipsoid`, `midpoint_and_swing`, `spread_95` | Each magnetometer axis's midpoint and stretch, from a tumble |
| `mounting.py` | `fit_axes` | Which way a magnetometer's axes point on the board, from the same tumble |
| `store.py` | `CalibrationFile` | Loads and saves both, plus the gyro bias and the last tumble's raw readings |

## The tumble (`tumble.py`)

Earth's field is the same strength in every direction, so as the board turns, a
perfect magnetometer's readings would all sit on a ball centred on zero. A real one's
ball is off-centre, and squashed a little along some axes. Fitting that shape gives
each axis's midpoint and how much to stretch it (`gains` in the code). This uses only
the magnetometer, never the gyro, so gyro error can't spoil it.

The fit only works if the board really was turned every way. Partial turns leave
most of the ball empty. So the tumble counts which of **26 directions** the field
has passed through: the 6 faces, 12 edges and 8 corners of a cube around the chip
(a 3×3×3 grid minus its centre). **Finish** unlocks at 20 of 26
(`TUMBLE_MIN_COVERAGE`). Counting only starts once every axis has swung at least
20 µT either way, because noise on a still board looks like it points everywhere.
The page's tumble ball shows the same 26 rings.

## Mounting (`mounting.py`)

**Mounting means which way a magnetometer's x, y and z point compared with the
gyro's x, y and z.** The tracker turns each magnetometer reading using the gyro's
idea of how the board turned. That only works if both sensors agree on what "x" means.
If the magnetometer is turned 90° from the gyro, the same turn moves its readings
the wrong way, and the error doesn't cancel.

- **ICM-20948:** its magnetometer is built into the same chip. The datasheet gives
  the exact mapping, `(x, −y, −z)`, and it's hard-coded in `hardware/icm20948.py`.
  The fit still runs on the ICM, but its result is only shown, as "Tilt on the
  board". The ICM's true tilt is about zero, so that number shows how accurate the
  method is.
- **MMC5983MA:** it's on its own breakout, standing on its side about 1 cm away.
  Its axes are turned by some number of quarter turns, plus a small tilt from how it
  was soldered. Each degree of tilt left in costs about 1 µT of error, so it's
  measured and applied.

How it's measured: Earth's field never moves. With the right mounting, every tumble
reading, turned back by the gyro's orientation at that moment, points the same way.
`fit_axes` tries all 48 quarter-turn layouts, keeps the one where the readings agree
best, then fine-tunes it to include the tilt.

- **Saved in:** `data/calibration/mmc_mounting.json`. **Reset calibration** keeps
  it, because it's a fact about how the hardware is built. Tumble again only if the
  MMC is remounted.
- **Backwards ambiguity:** turning can't tell a layout from its exact opposite, so
  the MMC's Earth can point the opposite way to the ICM's. Tracking and scores
  aren't affected.
- **Weak point:** unlike the tumble fit, this trusts the gyro through the whole
  tumble, so one slip spoils it (`docs/critique-2026-11-03.md`, point 6).
- **Only matters for the MMC.** If the MMC is dropped from the setup, nothing needs this.

## Links

- **Used by:** `tracking/tracker.py` (runs the tumble and applies the results), and
  `earth_cancel.py` (loads saved files at startup).
- **Settings:** `TUMBLE_*`, `PASS_UT`, file paths in `config.py`.
- **Tests:** `tests/test_tumble.py`, plus `tests/test_pair.py` for mounting.
- **Saved files:** see the Calibrations table in `docs/architecture.md`.
