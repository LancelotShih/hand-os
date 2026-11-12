# tracking

The core: sensor readings in, orientation and Earth-cancelled error out. No I/O, no web.

| File | Public names | Job |
|---|---|---|
| `tracker.py` | `Tracker` | One magnetometer's whole pipeline: orientation (Madgwick + gyro bias), Earth capture, the tumble, and the view sample every `block_samples` readings. `update()` runs in the sensor loop; the other methods are for HTTP threads. |
| `earth_capture.py` | `Capture` | Averages the magnetometer for `CAPTURE_S` at a roughly held pose, giving Earth `E` and the reference pose. Restarts if the board turns more than `CAPTURE_MAX_TURN_DEG`. |
| `pair.py` | `Pair` | Runs the ICM's and MMC's Trackers on the same gyro/accel readings, so differences between them come from the sensors. Adds `mmc` and `locate` to each sample. Has the Tracker's control methods, so the server can't tell it from a single Tracker. |

- **Used by:** `earth_cancel.py`, which builds a `Pair` and hands it to the ICM loop and the server.
- **Uses:** orientation, calibration, geometry. `Pair` is given its `Locator` and `MmcReader`; it doesn't import them.
- **Settings:** `BLOCK_SAMPLES`, `MMC_BLOCK_SAMPLES`, `MAG_LATENCY_S`, `HISTORY_S`,
  `MAX_GAP_S`, `CAPTURE_*`, `RATE_WINDOW_S` in `config.py`.
- **Output:** the view sample and status fields are listed in `docs/architecture.md`.
- **Tests:** `tests/test_tracker.py`, `tests/test_tumble.py`, `tests/test_pair.py`.
- **Known shape:** `Tracker` does five jobs, and `Pair` runs two identical orientation
  filters. Splitting out a shared orientation object is the natural next refactor.
