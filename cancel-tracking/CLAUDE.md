# cancel-tracking

Tracks an ICM-20948 in 3D around an always-on electromagnet, on a Raspberry Pi.
Later magnetic readings are rotated using gyro/accelerometer orientation and
compared with a captured reference. A coil model turns that field difference
into an estimated position. Remaining error also includes orientation, sensor
and model errors; it is not an independently isolated coil measurement.
An MMC5983MA can run beside the ICM's magnetometer (AK09916) so the two can be compared,
but it's off by default (`MMC_ON` in `config.py`, `--mmc` for one run). Position uses only the ICM.
End goal: the ICM on a glove fingertip, the coil on the back of the palm.

Run with the system `python3` on the Pi (stdlib + `smbus2`, no numpy).

```bash
./earth_cancel.py                                        # web page on http://<pi>:8000 (add --mmc for the MMC)
python3 -m unittest                                      # hardware-free suite
python3 -m magtrack_src.recording.compare data/logs/<name>     # score a log made with --mmc
python3 -m magtrack_src.recording.log_plot data/logs/<name>    # redraw a log's plot.html
```
All commands run from `cancel-tracking/`.

## Map

Read the relevant topic guide linked from the architecture's
[component map](docs/architecture.md#components-and-dependencies) before changing
a package. Each file's docstring says what it does.

| Path | Job |
|---|---|
| `earth_cancel.py` | Entry point: builds every object, starts the threads, serves the page |
| `user-guide.md` | Link to the maintained architecture document; contains no separate instructions |
| `magtrack_src/config.py` | Operational settings and file paths; algorithm/display constants also exist in modules |
| `magtrack_src/hardware/` | The only code that touches I2C: the ICM-20948 and MMC5983MA drivers, plus their read loops |
| `magtrack_src/geometry/` | Vector and quaternion math on plain tuples |
| `magtrack_src/orientation/` | Madgwick filter (gyro + accel only), gyro bias measured during rests |
| `magtrack_src/calibration/` | Measured once and saved: magnetometer tumble fit, sensor mounting fit, the JSON store |
| `magtrack_src/tracking/` | The core. `Tracker`: readings in, orientation and the difference between measured and reference fields out. `Pair`: runs ICM and MMC side by side |
| `magtrack_src/position/` | ICM error in, position out: coil model, coil fit from known points, live solve |
| `magtrack_src/web/` | HTTP server, sample feed, and the browser page (`page/`) |
| `magtrack_src/recording/` | Logging a run to `data/logs/`, plotting it, scoring it |
| `tests/` | `unittest` suites; `tests/sim/sim_board.py` simulates the board |
| `data/` | Everything the program writes (gitignored except `mmc_mounting.json`) |
| `docs/architecture.md` | Maintained setup, operation, and current behavior: frames, units, ownership, lifecycle, timing and persistence. Read before cross-package changes |
| `docs/architecture-review-2026-11-12.md` | Confirmed edge cases, assumptions, and proposed module boundaries |
| `docs/next-steps.md` | Agreed future behavior, implementation status, and proposed next steps |
| `docs/critique-2026-11-03.md` | Historical critique; paths and some lifecycle behavior predate current code |

## Import rule

Core numerical packages avoid hardware/web imports. The intended dependency
direction is:

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

Runtime calls are broader than imports: the hardware loop invokes its injected
tracker/log/feed, and `Pair` invokes its reader and locator. `Tracker` performs
file I/O through an injected store during controls. `Locator` combines numerical
logic, live state, workers and persistence. Do not describe tracking as entirely
free of I/O or the sensor loop as nonblocking.

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
  pose at the last reference capture). Positions are cm from the coil's center (0), in the
  board's axes as it sat at home. *Home* is a position the user enters, from the coil. See `docs/architecture.md`.
- **The coil is always on and not driven by the Pi.** The reference field is captured at home
  with the coil on, so "error" = coil field here − coil field at home.
- **Heading can drift.** Only the gyro holds heading; the powered coil adds a position-dependent
  field that prevents treating the magnetometer as an ambient-field compass. At a 50 µT field, one degree can contribute about 0.87 µT
  of rotation error. **Measure reference field at sensor home** redefines the reference; it does
  not provide an independent heading measurement.
- **Threads:** the ICM loop, optional MMC loop, HTTP request threads, and coil-fit
  and save threads. State objects have individual locks, but cross-object controls
  are not atomic and some status fields are unlocked. Live position solving and
  log writes run synchronously on the sensor thread. Keep long fits outside state
  locks and guard against obsolete results before committing them.
- **Sensor home:** entered coordinates, physical mark, reference orientation,
  and captured field are distinct. Save sensor home coordinates changes the
  coordinates/refits; Measure reference field at sensor home measures the reference.
  Neither verifies physical position or orientation.
  Coil points have no automatic calibration/reference invalidation. See the review.

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
The user reconfirmed the fixed-coil setup, optional MMC, range and accuracy target
on 2026-11-12; individual mechanical dimensions remain the November 8 description.
The agreed future behavior is in `docs/next-steps.md`. It includes coil-off
startup without physical home and user selection of ambiguous initial positions;
these are not implemented. Large history/reconnect warnings are also planned. Keep
current behavior in architecture/package docs and future requirements in next steps.

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
- Use **orientation** consistently for 3D rotation. `ambient_field` means the assumed uniform,
  steady background (Earth plus other constant contributions). `reference_field`
  means the corrected total field captured at a pose, including the coil if on.
  Keep those concepts distinct. Legacy JSON/CSV keys `earth` and `predicted`
  are compatibility names for the captured reference, not an Earth-only estimate.
- Pure logic stays free of I/O and gets a test against `tests/sim/sim_board.py`.
- Maintain setup, operation, and current behavior in `docs/architecture.md` only;
  keep `user-guide.md` as a link. Put agreed future
  behavior and proposed work in `docs/next-steps.md`. Keep evidence and findings
  in the architecture review. Package guides describe local contracts rather
  than repeating detailed system rules.
- Prioritize human understanding over short labels in the page, help text,
  reports, and documentation. Name the quantity and what it compares. Use
  "angle between measured and reference fields" and "difference between measured
  and reference fields"; reserve "error" for a stated comparison against a model
  or independently known value. Define mathematical terms where they are used.
  Distinguish board rotation, field angles, sensor correction, reference
  measurement, and coil-model calibration. Do not call model sensitivity
  "precision" or imply that a small field mismatch proves position accuracy.
  Use fitted lowest/highest reading, midpoint, half of fitted reading range,
  and axis scale multiplier for calibration values. Help text must describe
  implemented behavior; do not promise planned warnings or controls.
- Run `python3 -m unittest` before committing.
