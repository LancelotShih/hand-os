# icm20948 visualizer

3-D **position** tracking of an ICM-20948 IMU relative to a single nearby DC
electromagnet, with a live browser view.

The companion `../icm20948-tool` answers *which way is the board pointing*. This
project answers *where is the board*, by treating an energised electromagnet as
a fixed magnetic dipole and inverting its field at the IMU's magnetometer.

## How it works

More than ~3&times; its own size away, an energised electromagnet is a magnetic
dipole with moment **m** along its axis. Its field at the sensor is

```
B(p) = (k / r^3) * [ 3 (u . p_hat) p_hat - u ]      k = mu0 |m| / 4 pi
```

with **u** the unit magnet axis, **p** the magnet&rarr;sensor vector, `r = |p|`.
Measuring the whole vector **B** (three numbers) inverts this for **p** (three
numbers). The map is exactly two-to-one &mdash; `B(p) == B(-p)` &mdash; so one
reading fixes `r` and the full direction of **p** up to a single front/back
sign, pinned by the **Flip side** button.

Pipeline, per sample:

1. Madgwick AHRS (same filter as `../icm20948-tool`) gives the board's absolute
   orientation, used to rotate the magnetometer reading into a world frame.
2. `B_em = B_world - field_zero`, where `field_zero` was captured with the
   electromagnet **off** (Earth's field + static distortion).
3. `mag_localize.solve_position` inverts the dipole equation for **p**.
4. **p** is expressed in a magnet-centred frame, smoothed (EMA), and reported
   as a range, an along-axis / lateral split, and an offset from a chosen
   start point.

## File layout

| File | What it holds |
|------|---------------|
| `mag_localize.py` | The new part: `solve_position` (the dipole inversion) and `MagLocalizer` (field-zero capture, magnet calibration, per-sample solve, start-point offset, side flip). No I/O, stdlib only. |
| `position_web.py` | Local web server. A background thread owns the sensor, runs `OrientationTracker` + `MagLocalizer`, and streams samples over SSE. Serves `position_view.html` at `/`. |
| `position_view.html` | The live 3-D view: the magnet at the origin, the IMU moving around it, a trail, a start marker, signal bar, and the calibration buttons. |
| `imu_ahrs.py` | **Vendored** from `../icm20948-tool`, with one change: `OrientationTracker.process` also returns `quat_abs` and `mbody` so the localizer can reuse the fused orientation and the hard/soft-iron calibration. |
| `icm20948_driver.py`, `icm20948_registers.py` | **Vendored** unchanged from `../icm20948-tool`. |

Vendored rather than imported so this folder runs standalone; re-copy from
`../icm20948-tool` if the driver or filter there changes.

## Hardware

Same wiring as `../icm20948-tool` (I2C on bus 1 at `0x68`; see that README for
Raspberry Pi setup). Plus:

- **One DC electromagnet** placed where it will stay for the session. A small
  steel-cored lifting magnet (e.g. 24&nbsp;V, 2&nbsp;W) works; its useful range
  is roughly **8&ndash;25&nbsp;cm** before the field sinks into the AK09916's
  noise. A stronger / iron-cored coil extends that.
- No switch wiring is needed &mdash; you turn the magnet on and off by hand and
  click the matching button.

## Usage

```bash
./position_web.py -b 1                 # http://<this-pi's-ip>:8000
./position_web.py -b 1 --port 8080 -a 0x69
```

Then, from the page, once per session:

1. **Null gyro** &mdash; board still (also automatic at startup).
2. **Calibrate magnetometer&hellip;** &rarr; Start, tumble the board slowly
   through every orientation for ~20&ndash;30&nbsp;s, Finish. Do this **away
   from the electromagnet**. Writes `imu_mag_cal.json`.
3. Place the electromagnet, keep it **OFF**, hold the board still, click
   **Zero field (magnet OFF)**.
4. Switch the electromagnet **ON**. Hold the board on the magnet's axis at the
   distance shown in the **on-axis dist** box (default 12&nbsp;cm; pick
   10&ndash;20), click **Calibrate magnet**. Writes `magpos_cal.json`.
5. Move the board to wherever you want movement measured from and click
   **Set start**. The &Delta; rows now read relative to that point.

If the position looks mirrored through the magnet (moving the board away reads
as moving toward, or the along-axis sign is backwards), click **Flip side**
once &mdash; that resolves the front/back ambiguity for good until you
re-calibrate.

## Reading the numbers

- **range** &mdash; straight-line distance from the magnet, cm.
- **along axis** &mdash; component parallel to the magnet's axis (the physically
  meaningful coordinate; +/- is the two poles).
- **lateral** &mdash; distance from the axis line, cm.
- **x / y** &mdash; the lateral offset split into a fixed but *arbitrary* pair of
  perpendicular directions (a dipole is rotationally symmetric about its axis,
  so there is no natural "x"). Their *magnitude* (`lateral`) and their *change*
  are meaningful; the individual values are not an absolute compass bearing.
- **electromagnet signal** &mdash; `|B_em|` in µT. Below ~1&nbsp;µT there is no
  fix; ~2&ndash;6&nbsp;µT is usable; more is better. Move closer if it is weak.

Expected precision with a small 2&nbsp;W magnet: a couple of centimetres at
15&nbsp;cm, degrading fast past 25&nbsp;cm.

## Limits

- **Orientation drift.** Fusion runs 6-DOF while the magnet is energised (an
  energised coil nearby would wreck a magnetic heading). Roll/pitch stay
  absolute; yaw drifts slowly over minutes, which slowly rotates `field_zero`.
  Re-click **Zero field** (magnet off) if the fix drifts, or just re-**Set
  start**.
- **Steel in the magnet.** The lifting magnet's own pole pieces distort Earth's
  field even when off, and that distortion moves with the IMU relative to the
  magnet body &mdash; a bias that grows as you close in. Not corrected.
- **Residual magnetization.** After switching off, the core stays slightly
  magnetized, so "off" is not perfectly zero.
- **Near field.** Inside ~8&nbsp;cm (or less than ~3&times; the coil size) the
  dipole model is wrong; the view shows a warning there.
- **DC only.** No AC drive means no lock-in filtering; slow thermal / Earth
  drift leaks in over minutes. Keep sessions short or re-zero.

## As a library

```python
from mag_localize import MagLocalizer, solve_position

loc = MagLocalizer(cal_path="magpos_cal.json")
# feed it calibrated body-frame mag (from imu_ahrs) + the AHRS quaternion:
out = loc.process(mbody, quat_abs, time.monotonic())
# out["pos"] -> [x, y, z] cm in the magnet frame, or None until calibrated
```
