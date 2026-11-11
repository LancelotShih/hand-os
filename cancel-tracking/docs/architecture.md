# Architecture

How the pieces fit together. Read `../CLAUDE.md` first for the map and the rules.

## The idea in one equation

Earth's field `E` is a fixed vector. Capture it once, in the board's pose at that
moment (the *reference* axes). After that the gyro and accelerometer say how the board
has turned (`R`, board → reference), so every magnetometer reading `m` can be turned
back into the reference axes and compared with `E`:

```
u        = axes · (gains ∘ m − offset)     # calibrated, in the board's axes
measured = R · u                          # in the reference axes
error    = measured − E                   # 0 if only Earth is present
```

- **Coil off:** `error` should stay near 0 at any orientation. That's the
  Earth-cancelling test, judged against `PASS_UT`.
- **Coil on, Earth captured at home:** `E` also holds the coil's field at home, so
  `error = B_coil(here) − B_coil(home)`. `position/locate.py` solves that for "here".

## One sample's path

```
hardware/icm_loop.py      imu.read() → accel (g), gyro (°/s), mag (µT, board axes, or None)
        │                 now = time.monotonic()
        ▼
tracking/pair.py          takes the MMC's newest reading from hardware/mmc_loop.py,
        │                 feeds the same accel + gyro to both Trackers
        ▼
tracking/tracker.py       per sensor:
                            gains → Madgwick step (gyro − bias) → orientation history
                            tumble running?   → calibration/tumble.py collects raw mag
                            board resting?    → orientation/gyro_bias.py updates the bias
                            capture running?  → tracking/earth_capture.py averages E
                            every BLOCK_SAMPLES mag readings (≈50 ms) → one view sample:
                              rotate each reading by the orientation *when it was measured*
                              (MAG_LATENCY_S for the ICM, the MMC's own timestamp),
                              average, subtract E
        ▼
tracking/pair.py          sample["mmc"] = MMC's latest view sample
                          sample["locate"] = position/locate.py Locator.update(sample)
        ▼
hardware/icm_loop.py      recording/data_log.py .add(raw readings + sample)  (if logging)
                          web/feed.py .push(sample)
        ▼
web/server.py /stream     Server-sent events: each new sample, plus "status" every 50 ms
        ▼
web/page/app.js           draws it
```

### The view sample (`Tracker._sample`)

`quat`, `euler` (yaw/pitch/roll °), `turn` (° from the capture), `measured`,
`measured_mag`, `predicted` (= E), `predicted_mag`, `error`, `error_ut`, `error_deg`,
`peak`, `peak_deg`, `calibration_on`. `Pair` adds `mmc` (the same shape, or `None`)
and `locate` (`position` in cm or `None` plus `why`, `from_coil_cm`, `from_home_cm`,
`mismatch_ut`, `mm_per_ut`). `Feed` adds `seq`.

### Status (`/state`, and the stream's `status` event)

`Pair.status()` gives the ICM tracker's status, plus `mmc` (the MMC tracker's status
plus `connected`/`error`) and `locate` (`Locator.status()`). `server.py` adds
`connected`, `error`, `pass_ut`, `pass_min_turn`, `tumble_min_coverage` and `log`.

## Frames

| Frame | Meaning | Where it's made |
|---|---|---|
| chip | A magnetometer's own axes | AK09916 registers; MMC5983MA registers |
| board | ICM-20948 accel/gyro axes, as silk-screened | `icm20948.mag_to_body` maps the AK09916 in. The MMC goes through `Tracker.axes`, which the tumble fits (`calibration/mounting.py`) |
| level | Gravity on +z, heading arbitrary | Madgwick's world frame (`quat_from_gravity` at start) |
| reference | The board's pose at the last Earth capture | `Tracker._q_ref`. `quat` in samples is board → reference |
| home | Reference axes captured on the home tape mark, origin at the mark | Positions, in cm |

## Calibrations

| What | How it's measured | Uses gyro? | Saved in `data/calibration/` |
|---|---|---|---|
| Magnetometer midpoints and axis stretch | Tumble: axis-aligned ellipsoid fit to raw readings (`calibration/tumble.py`) | No | `earth_cancel_offset.json` (ICM), `earth_cancel_mmc_offset.json` (MMC); raw tumble data in `*_tumble_log.json` |
| Sensor mounting (quarter turns + tilt) | Same tumble: the rotation that lands every reading on one Earth vector (`calibration/mounting.py`) | Yes | `mmc_mounting.json`, applied to the MMC only. Tracked in git, and kept by Reset calibration |
| Gyro bias | Any 1 s rest (`orientation/gyro_bias.py`) | n/a | Saved alongside the offset |
| Earth `E` and the reference pose | 0.3 s average at a held pose (`tracking/earth_capture.py`) | n/a | Not saved: taken at every start and every **Capture at home** |
| Coil model | Points at known positions, fitted as a two-pole bar magnet (`position/locate.py`) | n/a | `coil_cal.json`. Only valid while home stays on the same mark |

`calibration/store.py` (`CalibrationFile`) reads and writes the offset/mounting files.
`Locator` saves `coil_cal.json` itself.

## Threads and locks

| Thread | Runs | Touches |
|---|---|---|
| main | `ThreadingHTTPServer.serve_forever`; one extra thread per request, and `/stream` holds one per open tab | `Pair` control methods, `Feed.since`, `DataLog.start/stop` |
| ICM loop | `hardware/icm_loop.sensor_loop` | `Pair.update`, `DataLog.add`, `Feed.push` |
| MMC loop | `hardware/mmc_loop.MmcReader.run` | Its own `_fresh` slot, which `Pair` takes from |
| coil fit / save | Started by `Locator` | `Locator` state |

Every shared object guards its state with its own `threading.Lock`. Fits run outside
the lock (`Tracker.tumble_finish`, `Locator._fit`) so the sensor loop never waits on
them. If the ICM drops out, the loop reconnects and calls `Pair.sensor_lost()`, which
discards orientation and takes a new capture.

## Files written at runtime

| Path | Holds |
|---|---|
| `data/calibration/*.json` | See Calibrations above |
| `data/logs/<YYYY-MM-DD_HH-MM-SS>/raw.csv` | Every sensor reading as it came (ICM magnetometer in board axes, MMC in chip axes) |
| `.../samples.csv` | Each view sample (columns: `recording/log_plot.py` `SAMPLE_COLUMNS`) |
| `.../info.json` | The status when the log started, including calibration |
| `.../plot.html` | Self-contained plot, written when the log stops |

## Tests

`python3 -m unittest` from `cancel-tracking/`. `tests/sim/sim_board.py` steps a
simulated board at 500 Hz in a uniform field (optionally with a coil), with gyro bias
and a magnetometer offset. Its errors are idealized (see the critique, point 9), so a
wrong shared assumption won't show up in the tests.

| Suite | Covers |
|---|---|
| `test_imu.py` | ICM register parsing and the axis map, against a fake bus |
| `test_tracker.py` | Cancelling, capture, gyro bias, calibration switch |
| `test_tumble.py` | Ellipsoid fit, coverage, tumble flow, saving |
| `test_pair.py` | Mounting fit, side-by-side MMC, `compare` scoring |
| `test_locate.py` | Coil model, coil fit, live position |
| `test_server.py` | Routes, page files and ids, the stream, driven by the ICM loop with a fake IMU |
| `test_data_log.py` | Log folders, CSV contents, plot |
