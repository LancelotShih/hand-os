# Magnetometer calibration

Calculations for magnetometer correction and sensor mounting alignment. Results
are saved to `data/calibration/` through `store.py`; the fitting functions only
calculate values. Operating instructions and measurement definitions are in the
[architecture](../../docs/architecture.md#magnetometer-calibration).

| File | Public names | Job |
|---|---|---|
| `tumble.py` | `Tumble`, `fit_tumble`, `coverage`, `fit_ellipsoid`, `midpoint_and_swing`, `spread_95` | Each magnetometer axis's midpoint and stretch, from a tumble |
| `mounting.py` | `fit_axes` | Which way a magnetometer's axes point on the board, from the same tumble |
| `store.py` | `CalibrationFile` | Loads and saves both, plus the gyro bias and the last tumble's raw readings |

## Rotation calibration (`tumble.py`)

The ambient field is assumed to have the same magnitude and direction at every
position covered and to stay steady throughout the tumble. It includes Earth
plus any other uniform, steady background contribution. Turning the sensor
changes its measured components, not that physical field. With the coil off,
a perfect magnetometer's tumble readings would sit on a ball centred on zero. A real one's
ball is off-centre, and squashed a little along some axes. Fitting that shape gives
each axis's midpoint and how much to stretch it (`gains` in the code). This uses only
the magnetometer, never the gyro, so gyro error can't spoil it.

The calculation needs readings collected with the board turned through many
orientations. This rotation procedure is called a tumble in the code. Partial turns leave
most of the ball empty. So the tumble counts which of **26 directions** the field
has passed through: the 6 faces, 12 edges and 8 corners of a cube around the chip
(a 3×3×3 grid minus its centre). **Calculate and save corrections** unlocks at 20 of 26
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
  The fit still runs on the ICM, but its result is only shown, as "Estimated sensor mounting tilt". The implementation assumes the fixed axis map is sufficient; this
  displayed fit is a diagnostic, not an independent measurement of the actual mounting.
- **MMC5983MA:** it's on its own breakout, standing on its side about 1 cm away.
  Its axes are turned by some number of quarter turns, plus a small tilt from how it
  was soldered. Each degree of tilt left in costs about 1 µT of error, so it's
  measured and applied.

How it's measured: the ambient vector is assumed fixed throughout the tumble. With the right mounting, every tumble
reading, turned back by the gyro's orientation at that moment, points the same way.
`fit_axes` tries all 48 quarter-turn layouts, keeps the one where the readings agree
best, then fine-tunes it to include the tilt.

- **Saved in:** `data/calibration/mmc_mounting.json`. **Clear sensor calibration** keeps
  it, because it's a fact about how the hardware is built. Tumble again only if the
  MMC is remounted.
- **Backwards ambiguity:** turning can't tell a layout from its exact opposite, so
  the MMC's captured reference can point the opposite way to the ICM's. Tracking and scores
  aren't affected.
- **Weak point:** unlike the tumble fit, this trusts the gyro through the whole
  tumble, so orientation-estimation errors affect the result (`docs/critique-2026-11-03.md`, point 6).
- **Only applied to the MMC.** The current tumble still computes this diagnostic
  for the ICM when the MMC is disabled.

## Which results are applied and saved

The ellipsoid is axis-aligned: no cross-axis terms are fitted. Its three radii
are scaled to their own average, so absolute field scale is not independently
calibrated. `spread_after_ut` scores the same points used for fitting, relative
to their mean corrected strength. It does not establish direction or position
accuracy. `passed` is diagnostic; the tracker currently applies a fitted result
even when false.

`CalibrationFile` saves `offset = gains * raw_center`, for subtraction after
gain application. The saved gyroscope zero-rate estimate (`bias`) is the value at calibration
completion. Later automatic estimates are not written after every quiet interval. Sensor files are direct writes, with permissive
loaders; they are not versioned or fully validated. Locator persistence is a
separate implementation in `position/locate.py`.

The ambient field can include a uniform contribution other than Earth. Its
strength must still fit the configured tumble range (20-100 uT by default).
Those bounds are checks on plausible calibration data; passing them does not
identify the source of the field or verify uniformity across the working area.

## Links

- **Used by:** `tracking/tracker.py` (runs the tumble and applies the results), and
  `earth_cancel.py` (loads saved files at startup).
- **Settings:** `TUMBLE_*`, `PASS_UT`, file paths in `config.py`.
- **Tests:** `tests/test_tumble.py`, plus `tests/test_pair.py` for mounting.
- **Saved files:** see [architecture.md](../../docs/architecture.md#persistence-streaming-and-reproducibility).
