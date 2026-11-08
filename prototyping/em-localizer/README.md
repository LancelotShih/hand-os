# em-localizer

3-D position of an ICM-20948 relative to a **pulsed** electromagnet at the
origin, with a live browser view of a ±15 cm cube around the magnet.

The coil is switched on and off (via the L298N driver in `../em-driver`) many
times a second. Subtracting the coil-off readings from the coil-on readings
leaves only the electromagnet's field: Earth's field, hard-iron offsets and
nearby steel all cancel, however large they are, with no orientation estimate
needed for the subtraction. That field is then inverted through a calibrated
magnetic-dipole model into an (x, y, z) position.

## How it works

1. **Pulse.** The coil steps through `+I, off` (or `+I, off, −I, off` in
   bipolar mode), holding each state for `PHASE_S`.
2. **Average per phase.** Magnetometer samples from the first `SETTLE_S` after
   each switch are thrown away while the coil current settles; the rest are
   averaged into one vector per phase.
3. **Demodulate.** Over a sliding window of `WINDOW_PHASES` phases, each axis
   is fitted as `reading = E + D·t + state·B_em`. `E` is the background (Earth
   and everything else that doesn't follow the coil), `D` a linear drift (slow
   rotation, temperature), `B_em` the electromagnet's field. A new `B_em`
   arrives every phase.
4. **Locate.** A dipole's field is `B(p) = (3(m·p)p − m r²) / r⁵`. The moment
   vector `m` is learned from known positions (below), and given `m`, `B_em`
   inverts in closed form to two mirror-image answers, `p` and `−p`. The one
   nearer the previous position is kept, so the track stays continuous through
   the whole cube.

## Files

| File | What it holds |
|------|---------------|
| `em_locate.py` | Entry point. The sensor/coil loop, the `step` settling-time test, and the web server. Hardware and pulse constants at the top. |
| `em_localize.py` | The math, no I/O: phase demodulation, dipole model and inversion, moment fit, calibration storage, the tracking localizer. Algorithm constants at the top. |
| `em_view.html` | The live 3-D view (no external libraries, works offline). |
| `em_sim.py` | Simulated coil and sensor for `--sim`. |

Reused from sibling folders (imported, not copied): `Coil` and the pin numbers
from `../em-driver/em_toggle.py`; the ICM-20948 driver and Madgwick AHRS from
`../icm20948-visualizer`.

## Usage

Run with the system `python3` (it has `gpiozero`, `lgpio` and `smbus2`).

```bash
./em_locate.py step            # 1. measure the coil's settling time (hold the board still)
./em_locate.py                 # 2. run; open http://<this-pi's-ip>:8000
./em_locate.py --sim           # try everything with no hardware attached
```

**First-time setup:**

1. **Tune the timing.** Put the board ~10 cm from the magnet and run
   `./em_locate.py step`. It prints the field strength at that spot, the noise,
   how fast the field settles after a switch, and suggested `SETTLE_S` /
   `PHASE_S` values. An iron-cored coil can take noticeably longer than the
   0.03 s default.
2. **Calibrate.** Start `./em_locate.py`, open the page, click
   **Calibrate…**. Hold the board still with the chip at a known offset from the
   magnet's centre (in the board's own x/y/z, as silk-screened), enter it, click
   **Capture point** (~2 s). One point is enough; 3–6 spread around the cube
   are better. The table shows how far each point *solves* from where you said
   it was. That's the honest accuracy check for your setup. Saved to `em_cal.json`.
3. **Track.** Move the board around. If the track starts out mirrored through
   the magnet, click **Flip side** once.

### Orientation modes

The magnet's field is measured in the sensor's own axes, so the board's
orientation matters for *where* that field says it is:

- **`fixed`** (default): x/y/z are the board's silk-screened axes, and the
  board must keep the same orientation throughout: slide it, don't turn it.
  The page warns if it tilts more than `TILT_WARN_DEG`.
- **`ahrs`** (`-o ahrs`): the board may rotate. Each magnetometer sample is
  rotated into a reference frame using the Madgwick AHRS, which is fed only the
  clean coil-off samples, so heading stays absolute. The frame is the board's
  pose when the reference was set (automatically `AHRS_SETTLE_S` after start,
  or **Set reference pose**). Square the board to your jig at that moment.
  Needs a magnetometer calibration (**Magnetometer…** on the page; the pulsing
  keeps running and only coil-off samples are collected). Calibrations captured
  in one mode are not valid in the other.

## Constants

All adjustable values sit at the top of their file, with defaults:

| Constant | File | Default | Meaning |
|---|---|---|---|
| `IN1_PIN`, `IN2_PIN` | `em_locate.py` | from `em_toggle.py` | L298N GPIOs (BCM) |
| `DRIVE_MODE` | `em_locate.py` | `"onoff"` | or `"bipolar"`: 2× the signal and cancels leftover core magnetism |
| `PHASE_S` | `em_locate.py` | `0.10` | seconds per coil state |
| `SETTLE_S` | `em_locate.py` | `0.03` | samples skipped after each switch |
| `MAG_LATENCY_S` | `em_locate.py` | `0.012` | how old an AK09916 sample is when read |
| `ORIENTATION_MODE` | `em_locate.py` | `"fixed"` | or `"ahrs"` |
| `CUBE_HALF_CM` | `em_locate.py` | `15` | drawn working volume |
| `WINDOW_PHASES` | `em_localize.py` | `4` | phases per `B_em` estimate (latency vs noise) |
| `B_MIN_UT` | `em_localize.py` | `0.5` | no fix below this field |
| `NEAR_FIELD_CM` | `em_localize.py` | `5` | dipole model unreliable inside this |
| `EMA_ALPHA` | `em_localize.py` | `0.4` | position smoothing (1 = raw) |
| `CAL_ESTIMATES` | `em_localize.py` | `20` | estimates averaged per calibration point |
| `START_HINT` | `em_localize.py` | `None` | starting side; `None` = last calibration point |

Every command-line flag (`--phase`, `--settle`, `-m bipolar`, `-o ahrs`,
`--in1/--in2`, `--port`, …) overrides its constant for one run.

## Limits

- **Signal strength sets the range.** Position error grows roughly as
  `(r/3)·(noise/|B_em|)`, and `|B_em|` falls as 1/r³. The `step` command and the
  page's signal card show `|B_em|` and its jitter; aim for well above 1 µT at
  the far corners (~26 cm). More coil current, or `bipolar` mode, helps.
- **Near field.** Within ~3× the coil's size of the magnet, the point-dipole
  model is wrong. Positions inside `NEAR_FIELD_CM` are flagged.
- **Saturation.** The AK09916 clips at ±4900 µT, which a strong coil reaches a
  few cm from its pole.
- **Motion.** Each estimate spans `WINDOW_PHASES × PHASE_S` (0.4 s by default),
  so fast motion lags and blurs. The drift term absorbs slow rotation within a
  window; fast rotation in `ahrs` mode still leaves some error.
