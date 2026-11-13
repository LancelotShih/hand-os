# Magnetic tracking

Estimates board orientation and calculates measured field minus captured reference field.
No hardware or web imports. Control operations can write through an injected
calibration store.

| File | Public names | Job |
|---|---|---|
| `tracker.py` | `Tracker` | One magnetometer's full set of processing steps: orientation (Madgwick filter and gyroscope zero-rate correction), reference capture, the tumble, and the view sample every `block_samples` readings. `update()` runs in the sensor loop; the other methods are for HTTP threads. |
| `reference_capture.py` | `ReferenceCapture` | Averages gain-scaled magnetometer readings for `CAPTURE_S` at a roughly held pose. Tracker applies offset and mounting corrections to store the reference field (including the coil if on). Restarts if the board turns more than `CAPTURE_MAX_TURN_DEG`. |
| `pair.py` | `Pair` | Runs the ICM and optional MMC Trackers on the same gyro/accel inputs. Adds the last MMC view and a synchronous locator result to ICM view samples. Provides the pair control/status interface used by the server. |

- **Used by:** `earth_cancel.py`, which builds a `Pair` and hands it to the ICM loop and the server.
- **Uses:** orientation, calibration, geometry. `Pair` is given its `Locator` and `MmcReader`; it doesn't import them.
- **Settings:** `BLOCK_SAMPLES`, `MMC_BLOCK_SAMPLES`, `MAG_LATENCY_S`, `HISTORY_S`,
  `MAX_GAP_S`, `CAPTURE_*`, `RATE_WINDOW_S` in `config.py`.
- **Output:** the view sample and status fields are listed in `docs/architecture.md`.
- **Tests:** `tests/test_tracker.py`, `tests/test_tumble.py`, `tests/test_pair.py`.
- **State ownership:** `Tracker` owns orientation, bias, capture, correction and
  sample blocks. `Pair` runs separate orientation filters for the two sensors.

Each tracker owns separate orientation, history, bias, capture, correction and sample
blocks. Matching IMU inputs are not a shared-state guarantee. Each magnetic reading uses a stored orientation from at or before its estimated
measurement time. The tracker does not estimate an intermediate orientation
between stored entries. Block sizes count readings,
not elapsed time. MMC views can repeat across ICM updates and do not say when the cached value was last updated.

`reference_field` is the captured corrected total field, including the coil at
home when on. The fixed ambient field means Earth plus any other uniform, steady
background contribution throughout the working area. Legacy sample `predicted`
and status `earth` names serialize this reference, not a separate Earth estimate.
`offset` is in scaled units: `gains * raw_center`, subtracted after applying gains.
A calibration switch leaves the mounting mapping enabled. Completing a tumble
applies the fit even if its quality flag is false. Capture averages in sensor
axes, so its allowance for modest motion is an approximation.

Reference, reset/reconnect semantics, optional sample fields and concurrency
limits are specified in [architecture.md](../../docs/architecture.md). See the
[review](../../docs/architecture-review-2026-11-12.md) before changing ownership.
