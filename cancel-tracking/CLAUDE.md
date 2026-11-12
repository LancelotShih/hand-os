# cancel-tracking

Tracks an ICM-20948 in 3D around an always-on electromagnet, on a Raspberry Pi.
Earth's field is cancelled at every orientation using the gyro and accelerometer,
and whatever field remains is the coil's, which a coil model turns into a position.
An MMC5983MA runs beside the ICM's magnetometer (AK09916) so the two can be compared.
End goal: the ICM on a glove fingertip, the coil on the back of the palm.

Run with the system `python3` on the Pi (stdlib + `smbus2`, no numpy).

```bash
./earth_cancel.py                                        # web page on http://<pi>:8000
python3 -m unittest                                      # 119 tests, ~65 s, no hardware
python3 -m magtrack_src.recording.compare data/logs/<name>     # score a log
python3 -m magtrack_src.recording.log_plot data/logs/<name>    # redraw a log's plot.html
```
All commands run from `cancel-tracking/`.

## Map

Read the package README before changing a package. Each file's docstring says what it does.

| Path | Job |
|---|---|
| `earth_cancel.py` | Entry point: builds every object, starts the threads, serves the page |
| `magtrack_src/config.py` | Every setting and file path. Change numbers here, not in modules |
| `magtrack_src/hardware/` | The only code that touches I2C: the ICM-20948 and MMC5983MA drivers, plus their read loops |
| `magtrack_src/geometry/` | Vector and quaternion math on plain tuples |
| `magtrack_src/orientation/` | Madgwick filter (gyro + accel only), gyro bias measured during rests |
| `magtrack_src/calibration/` | Measured once and saved: magnetometer tumble fit, sensor mounting fit, the JSON store |
| `magtrack_src/tracking/` | The core. `Tracker`: readings in, orientation and Earth-cancelled error out. `Pair`: runs ICM and MMC side by side |
| `magtrack_src/position/` | ICM error in, position out: coil model, coil fit from known points, live solve |
| `magtrack_src/web/` | HTTP server, sample feed, and the browser page (`page/`) |
| `magtrack_src/recording/` | Logging a run to `data/logs/`, plotting it, scoring it |
| `tests/` | `unittest` suites; `tests/sim/sim_board.py` simulates the board |
| `data/` | Everything the program writes (gitignored except `mmc_mounting.json`) |
| `docs/architecture.md` | Frames, units, threads, one sample's path, saved files: read before cross-package changes |
| `docs/critique-2026-11-03.md` | Known weak points and open questions (paths in it predate the move) |

## Import rule

Packages only import downward:

```
web, recording            (top: consume samples and status)
position                  (consumes tracking's samples)
tracking                  (uses orientation + calibration)
orientation, calibration  (use geometry)
hardware, geometry        (bottom)
config                    (any package may read it)
```

`tracking`, `position`, `calibration` and `orientation` never import `hardware` or
`web`, which is why they're testable without the board. `earth_cancel.py` is the
only place that wires layers together. Imports are absolute: `from magtrack_src.tracking.tracker import Tracker`.

## Facts that are easy to get wrong

- **Magnetometer axis map.** The AK09916 reads in `(x, -y, -z)` of the accel/gyro
  axes (datasheet DS-000189 §15, Figs 12-13), applied in `hardware/icm20948.py`
  `mag_to_body`. The MPU-9250-style `(y, x, -z)` is wrong here. It caused a
  28 µT disagreement before it was fixed. Suspect this first in any fit that
  mixes magnetometer data with gyro or accelerometer data. The simulator shares
  whatever map the code assumes, so tests can't catch a wrong one.
- **Opening the ICM resets the chip.** `IMU()` soft-resets it. If the user's
  server is running (`pgrep -af earth_cancel`), don't open the ICM from another
  process. Reading `/state` and `/stream` over HTTP is safe. Don't stop their
  server without asking.
- **Units:** µT, cm, °/s (gyro, converted to rad/s only inside Madgwick), g, and
  seconds from `time.monotonic()`. Quaternions are `(w, x, y, z)`, vectors are 3-tuples.
- **Frames:** chip axes → board axes (accel/gyro) → reference axes (the board's
  pose at the last Earth capture). Positions are cm from the coil's center (0), in the
  board's axes as it sat at home. *Home* is a position the user enters, from the coil. See `docs/architecture.md`.
- **The coil is always on and not driven by the Pi.** Earth is captured at home
  with the coil on, so "error" = coil field here − coil field at home.
- **Heading can drift.** Only the gyro holds heading (the coil spoils the
  magnetometer for it). Roughly 1 µT of error per degree of orientation error.
  **Capture at home** resets it.
- **Threads:** the ICM loop, the MMC loop, HTTP request threads, and short-lived
  coil-fit and save threads. `Tracker`, `Locator`, `Feed`, `DataLog` and
  `MmcReader` each guard their state with their own lock. Never do slow work
  (fits, disk) while holding a lock the sensor loop needs.

## Hardware setup (from the user, 2026-11-08)

- Coil: two 20×50 mm iron cores wound as one bundle (40×20×50 mm), up to 1 A at 5.6 V.
  At 1 A, ~13 cm away: ~47 µT on the ICM.
- Home: tape mark ~13 cm from the coil, broadside to its long side. ICM flat, face up.
  MMC mounted vertically ~1 cm away on the same breadboard.
- Working range: within 15 cm of the coil. Target: ~5 mm accuracy, ≥10-20 fixes/s.
- One coil and one sensor can't pin down positions beside the coil's middle (its field
  is symmetric about its long axis). Home is broadside, so it sits at that weak spot.
  Turning the coil so an end points at the work area is the open recommendation.

Ask the user if any of this has changed. They are the source of truth for intent and
hardware. Judge approaches from the code, not from old logs or session transcripts.

## Where to add things

- **A setting:** `config.py`, with a comment saying what it does and its unit.
- **A new sensor:** a driver plus a read loop in `hardware/`, a `Tracker` for it, and
  wire it in `earth_cancel.py`. `Pair` currently assumes exactly ICM + MMC.
- **A new calibration:** the fit goes in `calibration/` (no I/O), saving and loading in
  `calibration/store.py`, and a button route in `web/server.py`.
- **A page card or script:** add it to `web/page/`, list it in `PAGE_FILES` in
  `web/server.py`, and link it from `index.html`. `tests/test_server.py` checks all three,
  and that every element id the scripts use exists.
- **A field in the live sample:** produce it in `tracking/` or `position/`. If it
  should be logged, add a column in `recording/log_plot.py` `SAMPLE_COLUMNS` and
  `recording/data_log.py` `_sample_row`.

## Conventions

- Stdlib only (plus `smbus2` in `hardware/`). Plain tuples, no numpy, so it runs on the Pi as is.
- Pure logic stays free of I/O and gets a test against `tests/sim/sim_board.py`.
- Text the user reads (page, `(i)` help in `web/page/info.js`, README) is plain
  language with short sentences. For calibration, say lowest, highest, midpoint and
  half the swing, not radius, offset or gain, and say "error", not "residual". Put
  definitions behind `(i)` buttons, not long paragraphs.
- Run `python3 -m unittest` before committing.
