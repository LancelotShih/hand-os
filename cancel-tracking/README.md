# cancel-tracking

Cancels Earth's field at every angle on an ICM-20948, from one measurement, and
compares its magnetometer (AK09916) with an MMC5983MA doing the same job at the
same time.

Earth's field is a fixed vector `E`. Measure it once, and the gyro and
accelerometer then say how the board has turned, so the field the magnetometer
should read is known at any orientation:

```
predicted = Rᵀ E
error     = measured − predicted
```

The ICM's magnetometer also reads a fixed offset `d` that turns with the board and
can't be zeroed in hardware, so one reading can't tell `E` from `d`. A tumble
measures `d`, after which one reading per run is enough: `E = u − d`. The MMC
removes its own offset in hardware with SET/RESET, so it runs without `d` by default.

Each sensor's panel has a **Software calibration** switch. Off, that sensor's
readings are used as they came, with no midpoint or correction, so you can see
what it gives by itself. Switching measures Earth again, so hold the board roughly
still. A tumble's results are kept while the switch is off.

The MMC cancels Earth with no calibration: at startup it measures Earth once, and
the gyro and accelerometer then predict Earth at every angle. It needs one tumble
ever, to measure how the chip sits on the board. That goes in `data/calibration/mmc_mounting.json`,
which **Reset calibration** keeps, and later tumbles refresh it. If the sensor is
remounted, tumble again. Hold the board away from magnets at startup, and click
**Capture at home** to clear yaw drift.

The starting positions are `ICM_USE_CALIBRATION` and `MMC_USE_CALIBRATION` in
`magtrack_src/config.py`; the switches aren't saved between runs.

## Comparing the two magnetometers

Each magnetometer gets its own `Tracker`, and both are fed the same gyro and
accelerometer readings. They hold the same orientation, so error from turning is
shared and a difference between them is a difference between the sensors.

1. Click **Tumble** as below. **Finish** only waits for the MMC on the first tumble,
   which measures its mounting. Each sensor gets its own midpoints and axis
   correction, saved in its own file.
2. Click **Start log**, turn the board through large angles, rest it for a few
   seconds, then **Stop log**.
3. Score it:

```bash
python3 -m magtrack_src.recording.compare data/logs/<name>
```

The page shows the two sensors as identical panels, built from one template:
the same 3D view, numbers, tumble ball, result table and charts. The µT charts
share one scale and the 3D views share one camera. Orientation, gyro bias and
loop gaps are shared, so they appear once above the panels.

The headline is **direction error**: the angle between where a sensor says Earth
points and where it should point. The table also gives field error in µT, still
noise and reading rate. Noise is judged on 50 ms averages, so a faster sensor
gets credit for its speed and no more. Direction error near the capture proves
nothing, so only samples turned at least 30° count.

The MMC sits on its own breakout, so the tumble also measures how it's mounted:
its quarter turns and its tilt (`axes_fit.py`). This uses only the gyro and
accelerometer, so its calibration never leans on the AK09916. The ICM runs the
same fit but doesn't apply it, because its magnetometer is built in; its "Tilt on
the board" number shows how accurate the measurement is. Turning can't tell a
reading from an exactly backwards one, so the MMC's Earth may point opposite to
the ICM's. Tracking and every score are unaffected.

Both share the I2C bus, on separate threads. The bus runs at 100 kHz and the IMU
loop already fills it, so setting `dtparam=i2c_arm_baudrate=400000` in
`/boot/firmware/config.txt` and rebooting would give both more room.

`--no-mmc` runs the ICM alone. If the MMC drops out, the ICM carries on and the
reader keeps trying to reconnect.

## Use

```bash
cd cancel-tracking
./earth_cancel.py     # open http://<this-pi's-ip>:8000
```

1. Rest the board for a second. That measures the gyro bias.
2. Click **Tumble** and slowly turn the board through every direction for
   20–30 s, staying in one spot. The map shows which directions are done: turn
   the board to move the black ring onto hollow rings. **Finish** unlocks at 20
   of 26 directions. Earth is then measured again automatically, so hold roughly still.
3. Turn the board. The error should stay under 5 µT.

**Capture at home** retakes the reading and clears yaw drift. With the coil
off, keep magnets away. With it on, capture only on the home mark (see Position).

Tumble is the main calibration. It uses only the magnetometer, so gyro error
can't affect it, and it reports the offset and radius of each axis. A green
result means 95% of readings sit within 5 µT of a perfect sphere. It also fixes
unequal axis sensitivity.

The last calibration is saved and loads on the next launch. A new Tumble starts
from fresh readings and replaces it, nothing carries over. **Reset calibration**
erases it all, so the next run starts from nothing. Recorded logs and the MMC's
mounting are kept.

The limit is `PASS_UT` in `magtrack_src/config.py`. It sets every green and red on the page.

## Position

The **Position** card tracks the ICM in 3D around the coil, in cm from home.

The coil stays on the whole time. **Capture at home** measures Earth's field with
the board on its home mark, so the capture also holds the coil's field there. After
that the ICM's error is the coil's field where the board is now, minus its field at
home. `locate.py` models the coil as a bar magnet, with a pole at each end, and
solves for the position whose field matches.

1. Put the board on its home mark, squared to the tape, with the coil on. Start
   `./earth_cancel.py`, or click **Capture at home**. Home is wherever this
   happens, so always do it on the mark.
2. **Calibration points.** Put the board at a spot you've measured, flat and square
   as at home. Enter how far it is from home along the ICM's x, y and z (z is up),
   hold still and click **Capture point** (about 1 s). After 4 points the coil is
   found. 8–12 points, spread around and some raised, fit better. **Left out** is
   the honest accuracy: each point solved by a coil fitted without it.
3. Move the board. **Capture at home** clears drift and restarts the track at home.

**Point an end of the coil at the working area.** The coil's field is the same all
the way round its long axis. Beside the middle of the coil, moving round it barely
changes the reading, so a position there can't be pinned down. **Precision here**
shows how much the position can move per µT of field error. At 13 cm it's about
1 mm per µT within 50° of the coil's axis, 2.8 at 70°, and unusable at 90°. With
the coil broadside to home, home itself is at 90°.

The points and the fitted coil are saved in `data/calibration/coil_cal.json` and loaded on the next
launch. They hold only while home stays in the same spot: if the tape moves, click
**Clear points** and capture new ones. Tumble with the coil off or far away: it
needs the chip to see Earth alone.

## Logging

Click **Start log**, then **Stop log**. A link to the plot appears next to the
button. Each log gets its own folder named by its start time:

```
data/logs/2026-10-30_23-15-02/
    plot.html      the visual, opens on its own in any browser
    samples.csv    what the view showed: orientation, field and error for each sensor,
                   and the ICM's position in cm from home
    raw.csv        every sensor reading, offset not removed: the ICM's in the accel/gyro
                   axes, the MMC's in its own axes
    info.json      the calibration in use when the log started
```

Times are seconds from the start of the log. Blank cells mean no value yet: no
fresh magnetometer sample, or no Earth reference. Ctrl+C during a log still
finishes it. To redraw a plot: `python3 -m magtrack_src.recording.log_plot data/logs/<name>`.

## Code layout

Settings are all in `magtrack_src/config.py`. The code is split by job:

| Folder | Its job |
|---|---|
| `earth_cancel.py` | Starts everything and wires it together |
| `magtrack_src/hardware/` | Talks to the two chips |
| `magtrack_src/geometry/` | Vector and turn math |
| `magtrack_src/orientation/` | How the board is turned, from the gyro and accelerometer |
| `magtrack_src/calibration/` | Tumble, mounting, and saving them |
| `magtrack_src/tracking/` | Readings in, Earth cancelled, error out |
| `magtrack_src/position/` | The coil model and the position |
| `magtrack_src/web/` | The server and the page |
| `magtrack_src/recording/` | Logs, plots and scoring |
| `data/` | Everything the program saves: calibrations and logs |

Each folder has a short README. `docs/architecture.md` explains how a reading flows
through it all, and `CLAUDE.md` is the map for AI agents.

## Files it writes

| File | Holds |
|---|---|
| `data/calibration/earth_cancel_offset.json` | The offset, axis gains and gyro bias in use |
| `data/calibration/earth_cancel_tumble_log.json` | Readings and result of the last tumble |
| `data/calibration/earth_cancel_mmc_offset.json` | The same for the MMC |
| `data/calibration/mmc_mounting.json` | How the MMC sits on the board; Reset calibration keeps it |
| `data/calibration/earth_cancel_mmc_tumble_log.json` | The MMC's readings and result from the last tumble |
| `data/calibration/coil_cal.json` | Calibration points and the fitted coil; Clear points deletes them |

## Tests

```bash
python3 -m unittest     # from cancel-tracking/, no hardware needed
```

The tests simulate the board, so they can't catch a wrong assumption the
simulator shares. Axis directions come from the ICM-20948 datasheet, section 15.
