# Coil position tracking

Calculates a sensor position from the ICM's measured field minus its captured
reference field. `Locator` also reads and writes the saved coil-calibration JSON
file. Operating instructions and readout meanings are in the
[architecture](../../docs/architecture.md#position-setup).

| Module | Responsibility |
|---|---|
| `coil_model.py` | `Coil`: two effective poles, `field(p)` in µT at `p` in cm; `mm_per_ut`: local movement per unit of field change. No live state or I/O. |
| `coil_fit.py` | `least_squares`, `fit_coil`, `refine_coil`: numerical model fitting from current home-relative position/field pairs. A coarse search is followed by refinement. No live state or I/O. |
| `position_solver.py` | `solve`: current position calculation, including choosing the mirror nearer the starting position. `check_points` and `left_out_check`: existing local checks starting at entered coordinates. No live state or I/O. |
| `measurements.py` | `CoilCalibrationMeasurement`: named, tuple-compatible position, home-relative field difference and capture-variation fields, with explicit units. |
| `locate.py` | `Locator`: live state, point collection, fit/save workers, legacy-file conversion and persistence. Calls the numerical modules. `update` runs synchronously on the sensor loop. |

The previous public numerical imports from `locate.py` remain available for
compatibility. New callers should import the defining modules. Separating
lookup matching from history-based position selection is still planned work.

- **Units:** cm and µT. Positions are from the coil's center (0), in the home axes (the board's
  axes when the reference field was captured on the home mark). `Locator.home` is the mark's position, set by the user.
- **Settings:** `LOCATE_*` in `config.py`.
- **Tests:** `tests/test_locate.py`, `tests/test_locator_lifecycle.py`, and
  `tests/test_saved_calibration.py`.
- **Known limit:** the coil's field is symmetric about its long axis, so beside its
  middle a position can't be pinned down (`mm_per_ut` becomes very large). Home is currently
  broadside. A large movement-per-field-change value and the nominal working range do not stop position estimates from being published.

At startup, if the measured field matches multiple positions, the page asks the
user to choose the sensor's actual position. The model's field is symmetric
about the coil center, so mirrored positions can have the same reading. After
the choice, `Locator.update` uses the previous position to choose among matching
locations and marks ambiguous or locally weakly determined estimates with a
history-based warning. The sample includes other matching positions for the
page to mark as alternatives; the lookup can represent a continuous set of
solutions with sampled locations, so it is not necessarily exhaustive. This
does not reject estimates in a blind spot:
`mm_per_ut` describes local sensitivity and cannot identify the separate
mirrored answer.

`set_home` changes entered coordinates and refits; it does not take a reference
reading. Moving the physical mark changes the field used as the reference, unlike
correcting coordinates of the same mark. Existing point readings/model are not
automatically invalidated by reference or magnetometer-calibration changes.

`Locator` owns home/points/model, background fit/save workers, conversion of older files, and the previous position used to start the next position calculation. Fits run outside its state lock; the live solve
still runs on the acquisition thread. A local revision guards fit and solve
publication; Clear cancels collection and invalidates pending results. Saves
retry if their snapshot becomes obsolete before replacement. Reference changes
are still inferred from the rounded field vector, not explicit capture IDs.
Saved-file decoding validates local candidates before installing state;
`Coil.from_dict` checks model numbers and fitted length bounds. Shared
`persistence.temporary_json` writes the candidate file, while the locator keeps
ownership of the revision check and final replacement.
See the
[architecture](../../docs/architecture.md) for lifecycle details and the
[review](../../docs/architecture-review-2026-11-12.md) for revision/cancellation gaps.
