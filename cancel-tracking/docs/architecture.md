# Architecture

This is the maintained guide for setup, operation, and the current implementation,
including its limits. [CLAUDE.md](../CLAUDE.md) contains repository conventions.
Proposed changes and confirmed edge cases are in the
[architecture review](architecture-review-2026-11-12.md).
[Next steps](next-steps.md) records the agreed future behavior and proposed
implementation order. This document describes the current code only.

Start with [running the application](#running-the-application),
[magnetometer calibration](#magnetometer-calibration), or
[position setup](#position-setup). The [component map](#components-and-dependencies)
links to package-specific implementation guides.

## Scope and operating conditions

The current application is a Raspberry Pi bench experiment: a moving ICM-20948
near a fixed, continuously powered coil. Position uses the ICM's AK09916 only.
The gyroscope measures rotation rate, the accelerometer measures acceleration
including gravity, and the magnetometer measures the magnetic field.
An MMC5983MA can run beside it for comparison; it is off by default (`MMC_ON`,
`--mmc` / `--no-mmc`). The Pi does not control the coil current.

The confirmed working target is about 5 mm accuracy within 15 cm of the coil,
at 10-20 position updates per second. These are targets, not guarantees made
by the solver or the tests. Tracking relative to a moving palm-mounted coil
would require an additional way to account for the coil's changing pose.

The model assumes a fixed ambient field, stable coil output, rigid sensor
mounting, and repeatable reference captures at one physical mark and orientation.

## Running the application

Use the system `python3` on the Pi, with `smbus2` installed. All commands below
run from `cancel-tracking/`:

```bash
./earth_cancel.py          # ICM only
./earth_cancel.py --mmc    # include the optional MMC
```

Open `http://<this-pi's-ip>:8000`. Operational settings and file paths are in
`magtrack_src/config.py`; some algorithm bounds and display constants live in
their own modules.

Rest the board for about a second so the tracker can estimate what the gyroscope
reports when the board is not turning. This is its zero-rate estimate, called
`bias` in the code.
Saved calibration loads automatically. For new sensor corrections, follow
[magnetometer calibration](#magnetometer-calibration). For tracking, switch the
electromagnet off and measure the ambient field first. After that measurement,
switch the electromagnet on manually and confirm in the page to start tracking.
The application neither senses nor controls the physical coil power; its
`coil_on` status records the user's confirmation.

Each sensor panel has an **Apply magnetometer midpoint and scale corrections**
switch. The initial settings
are `ICM_USE_CALIBRATION` and `MMC_USE_CALIBRATION`; the switches are not saved
between runs. Their effect is defined in [field calculation](#field-calculation),
and reference replacement is described in [lifecycle](#lifecycle).

### Comparing the magnetometers

Run with `--mmc` and keep the coil off for this comparison: the sensors sit at
different positions and would otherwise see different coil fields.

1. Complete the [magnetometer rotation calibration](#magnetometer-calibration).
2. Click **Start recording**, turn the board through large angles, rest it for a few
   seconds, then click **Stop recording**.
3. Score the log:

```bash
python3 -m magtrack_src.recording.compare data/logs/<name>
```

The report compares the angle between measured and reference fields, the size
of their three-component difference, variation while the board appears still,
and reading rate. The median is the value with half the readings at or below it;
the 95% value has 95% at or below it. Only samples with at least
`PASS_MIN_TURN_DEG` (30 degrees) of board rotation from the reference orientation
count toward the field-comparison scores. Agreement near the reference
orientation proves little. The still-variation calculation uses raw readings
averaged into 50 ms intervals; it combines variation across all three axes.
It reports the square root of the sum of the three axes' pooled sample
variances, rather than an average per-axis value. For example, 1 uT of sample
variation on each axis gives about 1.73 uT combined variation. It includes any
field changes during the selected intervals, not only the sensor's own noise.

The two panels use the same template, chart scale, and 3D camera. Orientation,
gyroscope zero-rate estimates, and long pauses between updates are shown once
using the ICM tracker's values. Without
the MMC, only the ICM panel appears. The documented Pi setup uses a shared
100 kHz I2C bus; the application does not measure or configure that speed.

## Fixed ambient field and captured reference

**The ambient field is the background field throughout the working area.** It
includes Earth's field plus any other contribution that is uniform across that
area and steady during the run. The assumptions apply to every position covered:

- Its magnitude is the same at every position.
- Its direction is the same at every position, when expressed in fixed room axes.
- Both stay constant over time until the reference is measured again.

We call this vector `ambient_field`. Turning the board changes its x/y/z sensor
readings because the sensor axes turn; it does not change the physical ambient
field. The orientation estimate accounts for that change of axes.

A source being stationary does not make its field uniform. Nearby steel or a
magnet can produce a different field at each sensor position and break this
assumption. A magnetic offset attached to the board also turns with it; that is
handled by magnetometer calibration, not by the fixed ambient-field model.

The coil is treated separately because its field changes with position:

```text
total_field(position) = ambient_field + coil_field(position)
```

**The reference field is the corrected total reading saved during capture.**
`Tracker.reference_field` is a fixed stored vector in the reference axes until
another capture replaces it. What it contains depends on the coil:

```text
coil off: reference_field = ambient_field
coil on:  reference_field = ambient_field + coil_field(home)
```

With the coil off, capture can be anywhere in the assumed uniform ambient field;
the components are expressed in the axes defined by that capture. With the coil
on, current position tracking assumes capture on the entered home mark in its
reference orientation. It does not separately measure or store `ambient_field`
when the coil is on. Calling that combined reading Earth alone would be incorrect. Startup
that measures the ambient field with the coil off is in [next steps](next-steps.md).

## Field calculation

For one magnetometer reading `m`:

```text
corrected = axes * (gains elementwise-multiplied by m - offset)
measured  = R_at_measurement * corrected
error     = measured - reference_field
```

`error` is the code's historical name for the **difference between measured and
reference fields**. That difference is expected to change as the sensor moves
near the powered coil. A large value alone does not establish a measurement
mistake. The interface therefore calls it a field difference.

For example, `(10, 0, 0)` and `(0, 10, 0)` have the same strength, but point in
different directions. Their field difference has strength about 14.1 uT, and the
angle between them is 90 degrees. This is why the interface reports both values.

`axes` maps the magnetometer input axes into the ICM's accel/gyro axes.
The ICM driver already performs its fixed axis mapping; the MMC's mapping is
fitted. With software calibration off, gains are one and offset is zero;
the axis mapping is still applied.

The stored `offset` is the fitted raw center **multiplied by the gains**.
Consequently `gains * m - offset` equals `gains * (m - raw_center)`.
Do not substitute the raw center directly into the first expression.

`reference_field` is the captured constant vector described above:

- Coil off, uniform ambient field: `error` should be near zero at every covered
  position and orientation, subject to calibration and orientation error.
- Coil on, captured at sensor home: `error = B(here) - B(home)`, provided the
  background is uniform and the reference axes still match the setup.

`Locator` adds the fitted `B(home)` back to this difference and solves for a
position. Calibration, orientation error, drift, background changes, and model
error all contribute to the remaining field; it is not an isolated coil
measurement with independently verified accuracy.

### Field names in code and saved output

| Name | Meaning |
|---|---|
| `ambient_field` | Conceptual fixed background: Earth plus other uniform, steady contributions; `AMBIENT_FIELD` in the simulator |
| `reference_field` | Corrected reading stored by `Tracker` after `ReferenceCapture`; may include the coil at home |
| `mean_ambient_field` | Mounting fit's average rotated field, estimated from coil-off tumble readings |
| `coil_field` / `Coil.field(position)` | Position-dependent coil contribution |
| `error` | Measured field minus captured reference; includes measurement/model errors as well as the coil-field difference |

The readouts use the following meanings. These names also apply to saved plots
and the command-line comparison report.

| Displayed quantity | What it means | Code/output name |
|---|---|---|
| Difference between measured and reference fields | Length of the three-component difference; includes both strength and direction changes | `error_ut`; its components are `error` |
| Angle between measured and reference fields | Angle between the two magnetic vectors, in degrees | `error_deg` |
| Captured reference field strength | Strength of the last saved reference reading | `predicted_mag` |
| Board rotation since reference measurement | Angle between the board's reference and current orientations; not accumulated turning along its path | `turn` |
| Board orientation: yaw / pitch / roll | Current rotation relative to the reference orientation, expressed as rotations about z, y and x in that order | `euler`, with quaternion `quat` |
| Gyroscope zero-rate estimate | Apparent x/y/z rotation rates subtracted when estimating board orientation | `bias` |
| Fitted sensor midpoints | Estimated fixed magnetic shift on each sensor axis, before scale correction | `offset / gains` |
| Axis scale multiplier | Number multiplying each axis after its fitted midpoint is subtracted | `gains` |
| Estimated sensor mounting tilt | Fitted deviation from an axis-aligned mounting; not current board tilt | `mounting.tilt_deg` |
| Field-strength consistency of calibration readings | Difference from mean corrected strength that contains 95% of the calibration readings | `spread_after_ut` |
| Estimated movement per 1 uT field change | Movement that produces a 1 uT change locally, along the direction where the model's field changes least | `mm_per_ut` |
| Position difference using all measurements | Distance between entered and calculated coordinates using the model fitted to all measurements | `off_mm` |
| Position difference with this measurement excluded | Same distance after a local model refit that excludes this measurement | `left_out_mm` |

Both position checks start calculating position at its entered coordinates.
Their limits are explained under [position validity](#position-model-and-validity).
The field angle is undefined when either vector has zero strength; the current
`angle_deg` helper returns zero in that case. A displayed zero is then a fallback,
not evidence that the field directions agree.

Existing output names remain at the compatibility boundary: sample `predicted`
is `reference_field`, `predicted_mag` is its magnitude, and CSV `predicted_*_ut`
columns contain its components. Status JSON still uses `earth` for the same
reference vector and `capturing: "earth"` for reference capture in progress.
Comparison dictionaries also retain the key `direction` for field-angle scores.
These historical names do not imply a separate Earth-only measurement, predicted
field, or independently measured accuracy. JSON/CSV
formats, calibration filenames, and the `/capture` route are unchanged.

## Frames, origins, units, and time

A coordinate frame is a set of x, y and z directions and an origin. A vector's
three numbers only make sense with those directions specified. Position means
where the sensor is; orientation means how it is rotated. A pose includes both.
Heading is rotation around vertical; tilt describes leaning away from level.

| Name | Meaning | Implementation |
|---|---|---|
| Chip axes | Each magnetometer's register axes | AK09916 or MMC5983MA driver |
| Board axes | ICM accel/gyro axes | `icm20948.mag_to_body` for AK09916; `Tracker.axes` for MMC |
| Level axes | Initial accelerometer direction mapped to +z; heading arbitrary | `Tracker._q`, board to level |
| Reference axes | Board axes at the last completed reference capture | `conj(_q_ref) * _q`, board to reference |
| Position coordinates | Reference directions, with origin at the user-measured coil center | Positions and `Locator.home`, in cm |

**Sensor home is a position, not another rotation frame.** There are three
distinct pieces of state:

1. `Locator.home`: the entered coordinates of the physical sensor mark.
2. `Tracker._q_ref`: the orientation at the completed reference capture.
3. `Tracker.reference_field`: the field measured during that capture.

**Save sensor home coordinates** changes (1) and refits the coil. It does not take a magnetic
reading or change (2). **Measure reference field at sensor home** requests new (2) and (3). The
program cannot verify that the board is physically on the mark or squared to it.
Correcting coordinates of an unchanged mark differs from moving the mark:
moving it changes the field being subtracted and requires new point readings.

The coordinate origin remains the measured coil center, even if the fitted
model's `Coil.center` moves slightly away from zero. `from_coil_cm` is distance
from the measured origin; `mm_per_ut` uses the fitted model.

Vectors are 3-tuples internally and lists in JSON. Quaternions are `(w,x,y,z)`;
`quat_rotate(q,v)` maps a vector forward through that rotation. Euler output is
yaw/pitch/roll in degrees, ZYX. Microtesla (uT or µT) is the unit of magnetic field. Other units are cm, degrees/s,
g, and seconds. Gyro values become radians/s only for the orientation filter.
Position z is physically up only when the reference pose is flat and face up.

### Checking the coordinate directions

The directions come from the ICM chip's accelerometer/gyro axes, held fixed as
they were at capture. They are not the coil's axes or arbitrary bench directions.
With the board flat and face up, +z points out of the chip. The small x/y arrows
printed beside the chip normally mark +x and +y.

To check the physical board against the display, capture at home, then lift one
edge. The 3D board should tilt the same way; its black dot marks the +x edge.
Lift a side edge to identify +y. Position signs describe which side of the coil
the sensor occupies: if +x points toward the coil, a sensor home 10 cm away is
`(-10, 0, 0)`.

Axis-sign errors can produce mirrored motion or a misleadingly small fit error.
Check the physical chip axes and several directions of motion independently.
Include calibration points above and below the coil's height. After fitting,
raise the board and check that displayed z increases. If it decreases, correct
the entered coordinates against the chip axes; the display alone cannot
identify which sign was wrong.

### Measurement time

Acquisition uses `time.monotonic()`. The ICM timestamp is taken **after** its
register reads, not from a hardware timestamp. Its magnetic age is estimated
by `MAG_LATENCY_S`. The MMC timestamp is the midpoint of its two measurement
completion times. `Tracker._q_at` selects the last stored orientation at or
before that time, clamping to the oldest available one; it does not interpolate.

## Components and dependencies

`earth_cancel.py` constructs stores, trackers, the optional MMC reader,
`Locator`, `DataLog`, `Feed`, and the HTTP server. It starts the acquisition
threads and handles shutdown.

| Package | Owns | Boundary and exceptions |
|---|---|---|
| [hardware](../magtrack_src/hardware/sensor-acquisition.md) | Drivers, read loops, connection retry | The only production code importing `smbus2`; the ICM loop calls injected tracking, logging, and feed objects |
| [geometry](../magtrack_src/geometry/vector-and-quaternion-math.md) | Tuple vector/quaternion operations | No hardware, file, or web access |
| [orientation](../magtrack_src/orientation/orientation-estimation.md) | Madgwick step and gyro-rest estimator | No hardware, file, or web access; `Tracker` owns their state/history |
| [calibration](../magtrack_src/calibration/magnetometer-calibration.md) | Tumble and mounting fits, JSON store | Fits are pure calculations; `CalibrationFile` performs file I/O |
| [tracking](../magtrack_src/tracking/magnetic-tracking.md) | Per-sensor processing steps and `Pair` coordination | No hardware/web imports; `Tracker` calls its injected store during control operations |
| [position](../magtrack_src/position/coil-position-tracking.md) | Coil model, numerical fitting, live solve, point collection | `Locator` also owns threads and JSON persistence |
| [recording](../magtrack_src/recording/recording-and-analysis.md) | CSV writing, log plots, comparison | `schema.py` owns column definitions, row formatting and reading; `data_log` still requests plots on recording stop |
| [web](../magtrack_src/web/web-interface.md) | HTTP routes, feed buffer, browser scripts | Calls injected services; browser scripts share globals in HTML load order |
| Shared saved-data helpers | `validation.py`: finite numbers, three-component vectors and orthogonal mounting matrices; `persistence.py`: JSON reads and temporary-file replacement | Validation has no I/O; persistence does not own live state or operation ordering |

These links are the package guides. They describe local implementation details;
system behavior and operating instructions are maintained here. `tests/` contains
the hardware-free suites and simulator. Runtime output goes under `data/`, whose
files are listed under [persistence](#persistence-streaming-and-reproducibility).

Source imports and runtime calls are different boundaries: `Pair` does not
import hardware or position classes, but it calls its injected reader and
locator. The package diagram alone is not a guarantee of independent services.

Within `position`, `coil_model.py` owns the field and local movement-per-field-change
calculations, `coil_fit.py` owns numerical fitting, and `position_solver.py`
owns the current solve and point checks. Those modules have no live state,
worker threads or file access. `locate.py` coordinates those calculations with
measurement collection, live state, workers and persistence. The current solver
still selects the nearer mirror using its starting position; moving it to a
separate module has not yet separated matching from movement-history policy.

## One acquisition step

```text
IMU.read() -> accel, gyro, fresh mag or None -> timestamp
  Pair.update()
    consume newest unread MMC reading, if enabled
    update MMC Tracker with this accel/gyro step
    update ICM Tracker with the same accel/gyro step
    if ICM emits a view sample:
      attach last MMC view sample, if connected
      call Locator.update() synchronously
  DataLog.add() -> synchronous CSV writes, if logging
  Feed.push() -> attach sequence number, retain sample
  HTTP /stream -> deliver buffered samples and a status snapshot
  browser -> update readouts and draw canvases
```

Each `Tracker` performs its own orientation integration, gyroscope zero-rate estimation,
history lookup, tumble collection, reference capture, and magnetic averaging.
The two trackers normally receive matching initial bias and identical IMU
inputs, but orientation is duplicated state, not one shared object. Their
reference captures and magnetic measurement times are separate.

Magnetic readings are rotated individually and then averaged in fixed-count
blocks: five for the ICM, three for the MMC by default. About 50 ms is an
expected duration at nominal rates, not a fixed time window.

`MmcReader` has one latest-unread slot. A newer reading overwrites an unconsumed
one. `Pair` retains the last completed MMC view sample and attaches it to ICM
view samples until another arrives. Thus the two panel values are not
necessarily simultaneous, and `raw.csv` includes consumed MMC readings rather
than every conversion the MMC thread performed.

## Data contracts

Samples are dictionaries, without a central type or schema validator.

Internally, locator calibration entries use `CoilCalibrationMeasurement` from
`position/measurements.py`: entered `position_cm`, home-relative
`field_difference_ut`, and `max_field_deviation_ut` during capture. This named
tuple preserves numerical precision and the existing saved JSON keys. It does
not yet carry capture timestamps or calibration/reference identifiers; those
remain work in the plan. It must not be interpreted as an absolute coil-field
measurement for the future lookup table.

| Producer | Fields and availability |
|---|---|
| `Tracker._sample` | Always: `quat`, `euler`, `turn`, `measured`, `measured_mag`, `calibration_on`; `predicted`, `error`, `error_ut`, `error_deg` are null without a usable reference |
| Tracker with reference | Also: `predicted_mag`, `peak`, `peak_deg`; all magnetic vectors use that tracker's reference axes |
| `Pair.update` | `mmc`: latest cached MMC sample or null; `locate` when a locator is supplied |
| `Locator.update` success | `position`, `from_coil_cm`, `from_home_cm`, `mismatch_ut`, `mm_per_ut` |
| Locator without result | `position: null`, with a `why` string; success-only fields are absent |
| `Feed.push` | Adds `seq` to the sample after logging |

Sample quaternion/turn describe the current IMU step; the field is an average
of earlier magnetic measurements. Samples do not include acquisition times,
block intervals, reference identifiers, or MMC freshness identifiers.

`Pair.status()` returns the ICM status plus `finish_ready`, optional MMC status
(with connection/error), and optional locator status (home, points, fit state,
coil summary). HTTP adds ICM connection/error, thresholds, and log status.
These are separately locked snapshots, not one atomic snapshot of the system.
Tracker status also exposes `orientation_quat` at the status-update rate. Before
the first reference capture it is the live board-to-level estimate (heading is
arbitrary); afterward it is relative to the captured reference. This lets the
browser show board rotation even while no magnetic view samples are available.

## Magnetometer calibration

Rotation calibration, called a tumble in the code, corrects sensor readings
before the reference is measured and subtracted. It is separate from measuring
the ambient/reference field.

With the coil off or far away, click **Calibrate magnetometers** and slowly turn the rigid sensor
assembly through every direction near one spot, typically for 20-30 s. The same
ambient vector must be present throughout. Move the black ring onto the unfilled
directions on the coverage ball. **Calculate and save corrections** unlocks at 20 of 26 directions. With
the MMC enabled, it also waits for MMC coverage while the initial mounting fit
is still needed. Click **Calculate and save corrections** to run the fits, then hold roughly still for
the automatic reference capture.

The fit works as follows:

1. A perfect sensor would report different x/y/z components as it turns, but
   their combined magnitude would be constant. Those readings form a sphere.
2. The fit estimates the raw midpoint `raw_center` and three axis stretches.
   Subtracting the midpoint and applying relative `gains` makes the readings
   more spherical. The fit is axis-aligned; it does not correct cross-axis mixing.
3. The tracker stores `offset = gains * raw_center`, then applies
   `gains * reading - offset`. This is equivalent to `gains * (reading - raw_center)`.
4. The same tumble also fits magnetometer-to-board mounting using the orientation
   history. That is a separate calculation: only the MMC applies the fitted
   mounting; the ICM uses its fixed driver axis mapping. Its **Estimated sensor mounting tilt**
   result is a diagnostic, not an independent accuracy measurement.

A completed fit replaces the old correction. A green quality result means 95%
of the fitted readings are within `PASS_UT` (5 uT) of their mean corrected strength.
A red fitted result is still applied.

Afterward, with the coil off, turn the board and check that field error stays
under `PASS_UT`. This is an operating target, not independent accuracy validation.

The MMC also uses hardware SET/RESET to remove its own sensor offset and defaults
to software magnetic correction off. That assumes the hardware correction is
sufficient; fields from attached materials may still require software correction.
Its mounting fit remains applied even with the software switch off and must be
repeated after remounting. The fitted mapping can leave its reference pointing
opposite to the ICM's: that sign ambiguity does not affect per-sensor cancellation
or comparison scores.

The sphere fit uses only magnetic readings, not orientation. The mounting fit
does use orientation. A spherical result does not by itself establish correct
field direction, absolute field scale, a uniform environment, or position accuracy.
The fitted strength must also be within the configured 20-100 uT range; this is
a plausibility check, not proof that the captured field is Earth alone.
No calibration step verifies that the ambient-field assumptions hold throughout
the working area. Spatial or time changes in the ambient field remain errors
after subtraction and can be mistaken for changes in the coil field.

## Correction, calibration, and reference operations

These names refer to different operations in the current code:

| Operation | What it estimates | Collected and saved |
|---|---|---|
| Magnetometer correction | Reading midpoint and relative axis sensitivity | Tumble; correction saved per sensor |
| Mounting alignment | Magnetometer axes relative to gyro/accel axes | Same tumble; MMC mapping saved/applied, ICM fit displayed only |
| Gyroscope zero-rate estimate (`bias`) | Rotation reported during an assumed rest | Automatic quiet windows; saved only at tumble completion |
| Coil model calibration | Coil center, axis/strength and effective length | Known-position field differences; home, points and model saved |
| Reference field measurement | Field and pose used as the subtraction reference | Automatic startup or capture button; not saved across runs |

The last operation is a reference measurement, not a correction of the sensor.
When the coil is on, its field is included. Entering home coordinates is setup,
not a sensor calibration. There is no accelerometer correction calibration or
gyro scale calibration in this application. Orientation starts from gravity
with arbitrary heading; magnetometer heading initialization is not implemented.

## Lifecycle

| Operation | What actually changes |
|---|---|
| Startup | Loads saved sensor corrections/mounting and coil data; waits for the user to measure the ambient field with the physical coil off |
| Confirm coil state | The user switches the electromagnet manually and confirms its state in the page; `coil_on` is this software confirmation, not a sensor reading or hardware control |
| Gyro rest detected | Replaces that tracker's in-memory bias; it is written to disk only with a completed tumble |
| Calibrate magnetometers / Cancel calibration | Starts fresh collection / discards collection and any unfinished calculation; existing corrections keep running; start clears the previous displayed result |
| Calculate and save corrections | Fits outside locks, discards superseded results, otherwise applies corrections, optionally applies MMC mounting, saves files, starts another reference capture |
| Apply magnetometer midpoint and scale corrections | Keeps saved corrections but changes their use; clears the field reference and starts a new capture |
| Measure reference field at sensor home | Starts averaging; the previous reference remains usable until replacement completes; resets peaks and the sample block when complete |
| Clear sensor calibration | Invalidates unfinished tumble calculations, waits for any already-applied result to finish saving, then clears sensor corrections, bias and tumble result; keeps MMC mounting, coil points/model/home, orientation state, and recordings; starts another capture |
| Save sensor home coordinates | Validates entered coordinates, keeps current coil-coordinate points, starts refitting; legacy home-relative points are translated once |
| Record field at entered position | Averages `LOCATE_POINT_SAMPLES` view errors; rejects excessive variation, otherwise appends a point and refits |
| Delete coil calibration measurements | Cancels point capture, invalidates pending fits/position results, clears fitting status and removes points/model/checks; keeps entered home |

Capture requires at least `CAPTURE_S` (0.3 s) and `CAPTURE_MIN_MAG` (20) fresh
readings, restarting beyond 15 degrees from its starting pose. It averages in
sensor axes and picks the middle sample's orientation. This is an approximation
for a held pose, not motion-compensated averaging.

Coil points store differences against the reference that existed when they were
captured. There is no automatic invalidation when magnetic calibration,
reference orientation, physical home, coil output, or the environment changes.
`Locator` currently detects recapture by comparing `predicted` vectors rounded
to two decimals; this infers a reference change from its value, without an explicit capture identifier.

## Position setup

Use the [coordinate-direction checks](#checking-the-coordinate-directions)
before collecting points. Point an end of the coil toward the working area;
placing home broadside to the coil puts it near the weak middle plane described
under [position validity](#position-model-and-validity).

1. Measure the ICM home mark's coordinates in cm from the coil center, using the
   board's axes as it sits flat and face up at home. Enter them under Sensor home
   and click **Save sensor home coordinates**.
2. Put the board on that mark, squared to its reference directions, with the coil
   on. Click **Measure reference field at sensor home**. For later launches with saved calibration,
   put it there before starting the application.
3. Move the board to a measured position, flat and square as at home. Enter that
   position from the coil center, hold still, and click **Record field at entered position** (about
   one second at the expected rate). Collect 8-12 points spread around the area,
   including different heights. Fitting starts at four points.
4. Move toward the coil, sideways, and up. Check that the display follows each
   move. Test positions that were not used for fitting; **Position difference using all measurements** and **Position difference with this measurement excluded**
   alone do not establish accuracy, as explained under [validity](#position-model-and-validity).
5. To reset the reference after drift, return to the mark in its reference
   orientation and click **Measure reference field at sensor home**. This restarts tracking from
   home; it does not independently measure heading.

The card shows how far the fitted center moved from the entered origin. A large
shift can indicate coordinate errors or limitations of the coil model. Correcting
home coordinates with **Save sensor home coordinates** keeps the points and refits. Older
home-relative points are translated to coil coordinates on the first Set sensor
home; the initial input-box guess comes from the old model.

If the coil or physical home mark moves, set the new home, capture the reference
there in the correct orientation, click **Delete coil calibration measurements**, and collect new points.
Sensor-correction changes may also require new points; the program does not
enforce those dependencies. See [lifecycle](#lifecycle) for what each control keeps.

There is no startup wizard, enforced coil-off verification, initial heading
measurement, or candidate-position selection screen. Those belong to
[next steps](next-steps.md).

## Position model and validity

The coil is an effective two-pole bar magnet with fitted center, strength/axis,
and half-length. Fitting starts from a coarse grid of centers near zero and
refines the parameters with an iterative least-squares calculation (Levenberg-Marquardt), repeatedly reducing the squared differences between modelled and recorded fields. The center is limited by
`LOCATE_CENTER_SLACK_CM` (4 cm); half-length is clamped to bounds inside `coil_model.py`.
Four points is a minimum count, not proof of adequate geometry or a good fit.

The live solve starts at the previous position (initially home), tries other
starting guesses after a poor or distant result, and chooses the mirror solution closer to
the previous position. The model gives the same field at mirrored positions.
In the plane through the coil's middle perpendicular to its axis, an entire
circle has the same vector field, making position ambiguous there.

`mismatch_ut` measures agreement with the model; it is not position accuracy.
`mm_per_ut` estimates movement per unit of field change. It uses the model's
field Jacobian: a table of how small movements along x, y and z change the field.
It does not establish a measured position-accuracy range. Neither a large
movement-per-field-change value nor leaving the nominal 15 cm
range currently prevents publication of a position. The 15 cm value controls
the view and a rule for choosing alternative starting positions, not a hard tracking boundary. A weak or
incorrect field can therefore fit a distant or wrong position with low mismatch.
The current rejection test checks field mismatch, not independent accuracy.

**When several positions match the same field:** there is no explicit ambiguity rejection. If
several positions produce the same reading, the solver searches from the last
estimate and chooses the nearer mirror solution. It assumes continuous movement;
the magnetic reading does not establish which solution is correct. Around the
middle plane, the chosen answer can also depend on the search starting point.
A displayed position can therefore be ambiguous or wrong while its field
mismatch is very small. `mm_per_ut` does not detect the separate mirror solution.
There is no dedicated history/blind-spot warning or assumption flag in the
current output. The page colors Estimated movement per 1 uT field change green at 1.5 mm/uT or below and
red above 4 mm/uT. These display thresholds do not establish unique position,
reject a result, or identify the separate mirror solution. CSV position columns
do not record ambiguity, sensitivity, or history reliance.

For a vertical coil with north at the top, field above north and below south
points up; field around the middle plane points down. The axis is not itself a
region where nearby positions are hard to distinguish, although its opposite position produces the same field.
Mirror ambiguity also exists at off-axis positions away from the middle plane.

The checks on recorded measurements start calculating position at each entry's
entered coordinates. The check with one measurement excluded refines a model starting from the
model fitted to all measurements, then also starts the position calculation at
the excluded entry's coordinates. It is a local consistency check,
not independent acquisition or global tracking accuracy.

## Threads, locks, and failure handling

| Execution context | Work |
|---|---|
| Main | HTTP accept loop, shutdown orchestration |
| HTTP request threads | Controls, status/file reads, tumble fitting, log start/stop and plot generation; one long-lived thread per stream |
| ICM acquisition thread | Reads IMU, updates both trackers and live position, writes log rows, pushes feed samples |
| Optional MMC thread | Reads SET/RESET pairs, replaces its latest-unread slot |
| Locator worker threads | Coil fitting/checks and some saves; a new fit can be launched while an older one runs |

Trackers, locator, log, feed buffer, and MMC slot have individual locks.
Connection/error attributes and `Pair` coordination/caches do not share one
transaction lock. Locator saves also have a save lock. Sensor and coil stores
use the shared temporary-file replacement helper.

Each `Tracker` has a calibration revision. Reset, Cancel, starting another
tumble or toggling corrections invalidates an unfinished tumble calculation.
An obsolete result cannot change corrections, status or saved files. A separate
control lock orders applying/saving a result with these controls: if saving
has already started, Reset waits for it and then deletes the correction and
tumble-log files. The sensor state lock is released during disk I/O, so
acquisition and status reads do not wait on that control lock. Applying new
corrections also clears any partially averaged sample block.

A computation failure in either tumble or mounting fitting leaves existing
corrections in use, reports the failure and saves a failed-attempt log if
possible. A completed fit with a poor quality flag is still applied. A save
failure after application reports that the in-memory corrections were applied
but not all files were saved; the HTTP response reports the save error. Each
file is replaced only after its temporary version is written, flushed and
synchronized to disk. Failed writes preserve that file's previous complete
version. These replacements are not an atomic group: a mixture of old and new
files can remain after a failure. Sudden power loss still depends on filesystem
behavior; the containing directory is not explicitly synchronized. The control
lock protects one tracker instance, not separate programs sharing its files.

`Locator` advances a local calibration revision when measurements/home change
and when a fitted model is applied. A fit result is applied only if its input
revision is still current, even if a later operation recreated identical input
values. Clearing measurements cancels their capture and immediately resets the
fitting flag; obsolete workers cannot restore data or change a newer worker's
status. Running calculations are allowed to finish but their obsolete results
are discarded.

Position calculations also check the calibration revision and observed reference
generation before returning a result or updating the previous position. Invalid
magnetic vectors and nonfinite calculated positions, mismatch, distances or
sensitivity are rejected. This protects the live position boundary; it does not
validate every upstream sample or numerical fitting result. Saved calibration
validation is described under persistence below. Reference changes
are still detected from rounded field values as described under lifecycle,
so separate captures with identical values remain indistinguishable.

Saves take their snapshot after acquiring the save lock. If calibration changes
during the temporary-file write, the obsolete snapshot is discarded and the
save retries with current state. The revision check and final replacement occur
under the state lock. A failed obsolete write cannot overwrite a newer status
message. These locks protect one `Locator` instance, not multiple programs or
instances writing the same file.

Tumble fitting and coil fitting run outside their state locks. Live position
solving and logging still run on the acquisition thread; CSV writes/flushes
hold the log lock. Other Python threads can also compete for interpreter time.
There is no real-time or nonblocking acquisition guarantee. The locator guards
above do not provide an application-wide transaction: correction/model
compatibility, identifiers for saved setups and cross-object
coordination still have the limitations recorded in the review and next steps.

Drivers retry `OSError` after `I2C_RETRY_DELAY_S` (0.5 seconds). On ICM reopening, both trackers
keep their reference and previous orientation, clear their orientation histories
and sample blocks, and request a minimum tilt correction from the next accel
reading. Heading motion during the outage is unobservable. The last position estimate
is retained. If the board moved while disconnected, return it to home in the
reference orientation and click **Measure reference field at sensor home**. Connection messages
appear in the sensor status text;
there is no large history banner or dedicated dismissible reconnect warning. MMC-only reconnects
do not reset the MMC tracker. An ordinary IMU step longer than `MAX_GAP_S` is
counted and skipped for integration, without recovering the motion that was not measured.

Recording startup opens its CSV files locally, writes and flushes their
headers, then publishes the active recording state. If folder creation or any
startup write fails with `OSError`, opened CSV files are closed, recording stays
inactive, and the failure reason appears in status. The last completed recording
is preserved. Partial files can remain in the failed attempt's folder for
inspection; they are not presented as a completed recording. A retry creates a
new folder. The recording button displays rejected requests and network errors
and allows another attempt. The HTTP control handler returns ordinary error
JSON for `OSError` as well as `ValueError`.

Logged write errors end the active log. Other exceptions are not universally
converted into a failed/invalid sensor state. The HTTP server is intended for
the bench network: it binds all interfaces by default and has no authentication.

There is no session timeout. The page displays Time since reference measurement from `ref_age_s`,
computed using the latest IMU timestamp and the reference timestamp. It pauses
when readings stop, then includes the gap after readings resume. This is not
the age of a separately validated coil-off ambient-field or heading measurement.

## Logging

Click **Start recording**, then **Stop recording**. The page shows a link to the generated
plot. Each run gets a folder named by its start time, such as
`data/logs/2026-10-30_23-15-02/`. `plot.html` opens on its own in a browser.
Normal Ctrl+C shutdown also finishes an active log.

To redraw a saved plot:

```bash
python3 -m magtrack_src.recording.log_plot data/logs/<name>
```

File contents and limits are defined below. Blank CSV cells mean a value is not
available, such as a missing fresh magnetic reading or reference field.

## Persistence, streaming, and reproducibility

| Artifact | Contents and limits |
|---|---|
| `data/calibration/earth_cancel_offset.json` and `earth_cancel_mmc_offset.json` | Per-sensor offset in scaled units, gains, and bias at the last saved tumble |
| `data/calibration/earth_cancel_tumble_log.json` and `earth_cancel_mmc_tumble_log.json` | Per-sensor raw magnetic points and result from the last saved attempt; not the per-reading orientation/timestamp history |
| `data/calibration/mmc_mounting.json` | MMC axis mapping; tracked in git, retained by sensor reset |
| `data/calibration/coil_cal.json` | Entered home, entered positions, field differences, reading variation, fitted model/checks; old files without home need coordinate conversion through Save sensor home coordinates |
| `data/logs/<start>/raw.csv` | Every ICM loop reading in board axes without software offset removal; fresh AK values or blanks, consumed MMC values in chip axes and their ages |
| `samples.csv` | Rounded estimates at the ICM display-update rate and position coordinates; no position-quality/history flags; cached MMC values can repeat; columns and row formatting live in `recording/schema.py` |
| `info.json` | Tracker/locator status at log start, not a complete configuration or history of subsequent changes |
| `plot.html` | Self-contained plot generated at stop or by the plotting CLI; no position-ambiguity warning |

CSV time is monotonic seconds from the first logged reading; folder names and
metadata use wall time. Mid-run bias, reference, calibration, and model changes
are not fully recorded as events. Old position columns carry no origin version,
so historical home-relative logs need care when interpreted by current tools.

Saved sensor and coil calibration files are validated before any of their
values are installed. Vectors must have exactly three finite numeric components;
booleans, numeric strings, and values too large for the vector calculations are
rejected. Sensor axis scale multipliers must be positive. Mounting matrices must
have three unit, perpendicular rows within a `1e-5` dot-product tolerance;
rotations and reflections are accepted because the existing mounting fit
supports both. Coil strength must be nonzero, its half-length must be within
the fitter's 0.2-6 cm bounds, and the saved home must meet the existing 3 cm
minimum distance. The model field and sensitivity at home must be finite.
Measurement variation and check distances must be nonnegative; unavailable
check distances can be null. These structural checks do not prove physical
accuracy, environmental uniformity, or compatibility with current corrections.

Missing files are a normal first run. A malformed or unreadable file is left
untouched and ignored as a whole: sensor corrections return to their defaults,
mounting remains unknown, or coil tracking remains unconfigured. Startup exposes
the reason in the relevant sensor or position status message. Older sensor
files missing bias/gains retain zero/unit defaults; older coil files missing
home retain their explicit home-relative conversion workflow. File formats
have no schema version or shared setup identifier yet.

Sensor correction, mounting, last-tumble log and coil saves use complete-file
replacement. Serialization rejects nonfinite JSON numbers before replacement;
temporary files from failed or obsolete attempts are removed when possible.
This protects each previous file, not consistency across the whole file set or
against independent programs writing the same paths.

`Feed` retains 200 samples. `/stream` sends newer samples plus status roughly
every 50 ms, subject to scheduling and client speed. Each new connection starts
at sequence zero and receives retained history; there are no SSE IDs or
`Last-Event-ID` recovery. The browser plots samples using arrival time, not
acquisition time. Its field drawing reimplements the coil equation for display.

## Verification

From `cancel-tracking/`, run:

```bash
python3 -m unittest
```

Hardware is not required.
Suites cover parsing, tracking/capture, tumble fitting, mounting/comparison,
position fitting/paths, HTTP integration, logging, and plots.

The simulator steps idealized motion at 500 Hz. It includes noise, gyro bias,
axis gains, offsets and an optional coil made by the same `Coil` model used in
the solver. It omits hand translation acceleration, general cross-axis magnetic
distortion, realistic temperature/scale drift, background gradients, and real
bus/timestamp behavior. Many integration tests exercise sequential control
operations; passing them does not establish concurrent lifecycle correctness
or physical accuracy. See the review for gaps and proposed boundaries.
In particular, simulated axis directions cannot validate their own mapping;
the ICM-20948 axis map comes from its datasheet, section 15.
