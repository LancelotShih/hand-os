# em-steady

3-D position of an ICM-20948 relative to an **always-on** electromagnet at the
origin, with a live browser view of a ±15 cm cube around the magnet.

`../em-localizer` pulses the coil and subtracts coil-off readings from coil-on
readings. Here the coil never switches. Instead, Earth's field is learned
once as a fixed vector, and every sample has it subtracted using the board's
orientation:

```
B_em = R(q) · b_measured − E_ref
```

`b_measured` is the hard/soft-iron-corrected magnetometer reading in the
board's axes, `R(q)` rotates it into a fixed **reference frame**, and `E_ref` is
Earth's field in that frame. `B_em` then goes through the same calibrated
dipole inversion as em-localizer.

## How it works

1. **Orientation without the magnetometer.** The coil's field is in every
   reading, so a 9-DOF filter would mistake the magnet for north. Orientation
   comes from the gyro and accelerometer only (Madgwick, 6-DOF). Roll and pitch
   are absolute (gravity). Yaw is integrated gyro, so it **drifts** slowly.
2. **Reference frame = the board on its mark.** Pick a fixed spot and
   orientation on the jig (the "mark"), squared to the axes you want positions
   in. The reference pose is taken automatically right after startup (start
   with the board on the mark, still), and again whenever you click **Re-zero on
   mark**. Re-zeroing clears accumulated yaw drift.
3. **Earth field ("normalize at all angles"), in two steps.** Neither step
   integrates the gyro through the tumble, because one slip there would ruin
   the fit.
   - *Tumble, coil off.* An axis-aligned ellipsoid is fitted to the raw
     magnetometer vectors: hard-iron offset and soft-iron scale, using no
     orientation at all.
   - *Still on the mark, coil off.* The gyro is re-nulled and the reference
     pose re-taken, so the rotation is the identity and the corrected reading
     simply **is** `E_ref`.

   Two gyro-free checks guard the fit: the corrected field must have the same
   length in every orientation (*Earth-fit floor*: the smallest `B_em` you can
   trust), and the same component along gravity. `E_ref` lives in the reference
   frame, so it stays valid across runs as long as the jig and mark don't move.
4. **Locate.** Every magnetometer sample becomes a `B_em`. Blocks of
   `BLOCK_SAMPLES` (10 Hz) are averaged and inverted through the dipole model
   `B(p) = (3(m·p)p − m r²)/r⁵`, with the moment `m` learned from known positions.

## Files

| File | What it holds |
|------|---------------|
| `steady_locate.py` | Entry point. Sensor loop (coil held on, orientation, Earth subtraction) and the web server. Hardware constants at the top. |
| `earth_model.py` | The math, no I/O: 6-DOF `Attitude` with gyro-bias nulling and still-time bias refinement, the hard/soft-iron fit and its checks, the mark capture's data type. Algorithm constants at the top. |
| `steady_view.html` | The live 3-D view (no external libraries, works offline). Shows the board's own axes at its position. |
| `steady_sim.py` | Simulated coil and a *rotating* sensor (drifting gyro bias, hard/soft iron) for `--sim`. |

Reused from sibling folders (imported, not copied): the dipole model,
inversion, moment fit and tracking `Localizer` from `../em-localizer/em_localize.py`;
`Coil` and the pin numbers from `../em-driver/em_toggle.py`; the ICM-20948
driver, `Madgwick` and `MagCalibration` from `../icm20948-visualizer`.

## Usage

Run with the system `python3` (it has `gpiozero`, `lgpio` and `smbus2`).

```bash
./steady_locate.py               # coil on via the L298N; open http://<this-pi's-ip>:8000
./steady_locate.py --no-coil     # EM powered separately -- never touches the GPIOs
./steady_locate.py --sim         # everything simulated, no hardware
```

**First-time setup:**

1. **Start on the mark.** Board squared on its mark, still, when you launch.
   The first ~1 s nulls the gyro and takes the reference pose.
2. **Earth field.** Click **Earth field…** → **Start**. The coil turns off.
   With `--no-coil`, unpower the EM for this whole step. Slowly tumble the board
   through every orientation (every face and corner up, and round in heading)
   in one spot, away from the magnet and steel, until coverage reads 90 %+
   (minimum 60 %), then click **Finish tumble**. Set the board **still on its
   mark** and click **Capture on mark** (~2 s). Saved to `earth_cal.json`; the
   raw tumble goes to `earth_session.csv`.
   Compare *tumble radius* with `|E|`: they should agree and match your local
   field (roughly 25–65 µT).
3. **Check the cancellation.** Click **Coil: on** to switch it off. `|B_em|`
   should now sit near the Earth-fit floor wherever the board is and however
   it's turned. Anything larger is orientation error or local field distortion.
   Turn the coil back on.
4. **Calibrate the electromagnet.** **Calibrate EM…**: hold the chip at a
   known offset from the magnet's centre, in **reference axes**, and click
   **Capture point**. The board may be turned any way. One point works; 3–6
   around the cube are better. The table shows how far each point solves from
   where you said it was. Saved to `em_cal.json`. Redo after a new Earth calibration.
5. **Track.** Move and turn the board freely. Re-zero on the mark now and then
   (the page shows time since the last re-zero; it also re-nulls the gyro, so
   the board must be still). If the page reports lost orientation, re-zero.
   If the track starts out mirrored through the magnet, click **Flip side** once.

**To refresh Earth's field later** (room changes, jig moved): open **Earth
field…** and click **Capture on mark** alone. The coil goes off for ~2 s.

### Reading the Earth result

| Symptom | Meaning |
|---|---|
| *gyro tracking* near the noise (~1 µT) to a few µT | orientation held up through the tumble |
| *gyro tracking* close to `\|E\|` | orientation was lost during fast motion; check `I2C error` count and wiring. Calibration isn't affected, but tracking will be: this is the first thing to fix |
| Finish refuses: length varies | tumbling too close to the magnet or steel |
| Finish refuses: field along gravity varies | turning too fast, or a wrong magnetometer axis map |
| Coil-off `\|B_em\|` grows steadily with time | yaw drift; re-zero, and let the board rest so the bias refines |

## Constants

| Constant | File | Default | Meaning |
|---|---|---|---|
| `IN1_PIN`, `IN2_PIN` | `steady_locate.py` | from `em_toggle.py` | L298N GPIOs (BCM) |
| `COIL_CONTROL` | `steady_locate.py` | `True` | `False` = EM powered separately (`--no-coil`) |
| `COIL_STATE` | `steady_locate.py` | `1` | coil direction while on |
| `COIL_SETTLE_S` | `steady_locate.py` | `0.2` | samples ignored after the coil switches |
| `MAG_LATENCY_S` | `steady_locate.py` | `0.012` | magnetometer sample age; orientation is looked up this far back |
| `BLOCK_SAMPLES` | `steady_locate.py` | `10` | samples averaged per position fix (100 Hz → 10 Hz) |
| `AUTO_REFERENCE` | `steady_locate.py` | `True` | take the reference pose right after the startup gyro null |
| `AHRS_BETA` | `earth_model.py` | `0.05` | Madgwick gain (accel trust for tilt) |
| `GYRO_NULL_SAMPLES` | `earth_model.py` | `200` | samples per gyro-bias null |
| `AUTO_BIAS`, `STILL_*` | `earth_model.py` | on, 0.5 °/s, 0.03 g, 1 s | refine gyro bias whenever the board rests |
| `EARTH_MIN_SAMPLES` | `earth_model.py` | `1000` | tumble samples needed |
| `EARTH_MIN_COVERAGE` | `earth_model.py` | `0.6` | share of 26 gravity directions visited |
| `EARTH_MAX_RADIAL_UT` | `earth_model.py` | `5` | allowed spread of the field's length |
| `EARTH_MAX_VERT_UT` | `earth_model.py` | `8` | allowed spread of the field along gravity |
| `EARTH_MARK_SAMPLES` | `earth_model.py` | `200` | samples averaged on the mark |

Position smoothing, signal thresholds and the near-field radius are
em-localizer's (`EMA_ALPHA`, `B_MIN_UT`, `NEAR_FIELD_CM`, … in `em_localize.py`).

## Limits

- **Yaw drift is the main error source.** An orientation error of δ leaves
  `|E|·δ` of Earth's field in `B_em`: about **0.8 µT per degree** at a typical
  45–50 µT. Position error is roughly `(r/3)·(field error/|B_em|)`, so drift
  hurts most where the magnet's field is weak (far corners). Re-zero on the
  mark often. Resting the board lets the gyro bias refine itself. The pulsed
  method in `../em-localizer` has no such limit, which is the trade-off for
  never switching the coil.
- **Tilt during acceleration.** The accelerometer can't tell gravity from
  motion, so fast moves briefly tilt the estimate. Settle before reading.
- **"Constant" Earth field.** Steel nearby, or an iron-core magnet, bends
  Earth's field differently at different places. That error moves with the
  board and can't be calibrated out by one vector. The coil-off check (setup
  step 3) shows how large it is. If the coil core holds remanent magnetism
  after being switched off, do the Earth tumble well away from it.
- **Near field / saturation.** As em-localizer: the dipole model is wrong
  within ~3× the coil size (`NEAR_FIELD_CM`), and the AK09916 clips at ±4900 µT.
