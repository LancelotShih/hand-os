# Next steps

Agreed behavior from the November 12, 2026 interview, with the lookup-table
approach selected on November 17. This records remaining work and implementation
progress. See
[architecture.md](architecture.md) for current behavior and the
[source review](architecture-review-2026-11-12.md) for findings and evidence.

## Implementation progress: November 17

The first structural increment is implemented. The numerical coil model,
fitting and existing position solve/checks now live in separate modules;
`Locator` retains live coordination, workers and persistence. Calibration
measurements have named fields with explicit units. Shared recording columns,
CSV formatting and file reading now live outside the plot renderer. Current
ownership and record meanings are described in the architecture's
[component map](architecture.md#components-and-dependencies) and
[data contracts](architecture.md#data-contracts).

All 121 baseline tests passed before extraction. After extraction, all 125 tests
passed, including new import-boundary, compatibility and measurement-file
round-trip checks. Sixteen extracted numerical definitions match the original
syntax trees after two helper renames. Exact numerical replay also matched for
60 field/sensitivity/position cases, a noisy ten-point fit and both point-check
methods. That run still emitted the Windows stream-disconnect traceback without
failing its test; the subsequent reliability change below handles it.

This increment does not change hardware axis mappings, remove home, initialize
heading from magnetic readings, or add the lookup table. Setup identifiers,
operation coordination, shared orientation ownership and the remaining state
reliability fixes are still pending. The hardware-axis audit remains a
prerequisite to changing coordinate definitions and magnetic calculations.

The following reliability increment handles recording-start disk failures,
closes files opened by an unsuccessful attempt, preserves the last completed
recording, returns normal HTTP error JSON, and displays recording request
failures in the browser. Fault-injection tests cover folder creation, metadata,
both CSV opens, header writing and flushing, cleanup and retry. The combined
three-axis variation calculation is unchanged; its report label is explicit
and a new test checks the sum of the axis variances. All 130 tests passed after
these changes. Current behavior and retained partial attempt folders are
described under [failure handling](architecture.md#threads-locks-and-failure-handling).

The locator reliability increment adds local calibration revisions, rejects
obsolete fit/position results, cancels point capture on Clear and resets fitting
status. Serialized saves now also recheck their snapshot revision before file
replacement and retry with current state if necessary. Live position inputs
and calculated output values must be finite. Controlled event-based tests cover
Clear during capture/fit/solve, repeated input values across newer fits, new
references during a solve, and calibration changes during saving. Windows
stream aborts are handled during headers and body writes without swallowing
unrelated errors. Additional checks cover obsolete save errors and nonfinite
sensitivity/distances. All 143 tests passed; the prior disconnect traceback was
absent.

The sensor-calibration increment guards unfinished tumble and mounting fits
against Reset, Cancel, a newer tumble and correction toggles. Applying and saving
an accepted result is ordered with those controls, so an earlier save cannot
restore files after Reset. Fitting holds neither lock, and disk I/O does not
hold the sensor state lock. Failures in either numerical fit retain previous
corrections; completed poor-quality results are still applied. Disk failures
report when in-memory corrections were applied without all files being saved.
Controlled tests cover obsolete successes/failures, repeated input values,
Reset at each of the three save stages, and state access during slow saves.
All 152 tests passed after this increment.

The saved-data increment validates whole sensor correction, mounting and coil
records before installation. It rejects invalid shapes, nonfinite numbers,
nonpositive gains, invalid mounting matrices and out-of-bounds coil lengths.
Valid legacy defaults and home-relative files remain supported. Missing files
are normal; rejected files remain untouched and the startup status explains the
failure. Shared persistence helpers write and synchronize temporary files before
replacement, reject nonfinite JSON output, and remove abandoned temporary files.
The locator retains its revision check immediately before replacement.
Fault tests cover damaged input, failed partial writes, synchronization and
replacement failures, retries, legacy compatibility, and obsolete saves.
All 173 tests passed after this increment.
Schema/setup identifiers, atomic changes across several files, explicit capture
identifiers, validation of upstream readings/fitting results and coordination
across both sensors and the locator remain pending.

## Status at a glance

| Requirement | Current implementation |
|---|---|
| Start anywhere without sensor home | Not implemented; position tracking still requires home -> I don't like this idea because we shouldn't need to depend on this concept. |
| Generate a dense lookup table from a coil model fitted to measured positions | Not implemented; the current code solves directly against a fitted model using home-relative field differences |
| Measure the ambient field with coil off before tracking | Reference capture exists, but no enforced coil-off startup sequence -> we can build this into a electromagnet switch later...  |
| Establish starting heading while coil is off | Not implemented; initial orientation uses gravity with arbitrary heading -> gyro depends on initial condition and everything else is based off that... So we can use Earth's North as an absolute position instead of "home" |
| Reuse saved calibration or collect new readings | Saves load automatically; existing controls collect new data; no explicit startup choice |
| Ask the user to choose between matching starting positions | Not implemented; solver starts at home |
| Continue through ambiguity with a large warning | Estimates continue; the dedicated warning and recorded flags are not implemented |
| Resume after disconnect using the last estimate | Implemented; missed heading changes remain unknown |
| Keep an extra reconnect warning until dismissed | Not implemented; only connection/status messages exist |
| Report an error if no acceptable position matches (Don't just fix this behind the scenes using some magic) | Rejects field mismatch above 3 uT; finite-value validation still needs hardening |
| No hard 15 cm cutoff | Already matches intent; 15 cm is a nominal working target |
| No forced session timeout; display reference age | No timeout and a Time since reference measurement display exist; age is not yet tied to a verified coil-off measurement |
| Apply completed new magnetometer/coil fits, warning on poor quality | Replacement exists; tumble shows red quality results; overall coil-quality warnings remain to be defined |
| Update gyro zero automatically | Implemented; stillness detection can incorrectly learn slow turns as drift, but we can assume nobody will slow turn it like that |

## Agreed lookup-table approach: November 17

Use magnetic readings at independently measured sensor positions to fit a coil
model, then use that model to calculate a dense position-to-field table. The user
selected this approach rather than physically measuring every table entry.

1. Measure the ambient field with the coil off and establish sensor orientation.
   This does not require a particular sensor position under the uniform-background
   assumption. Sensor midpoint, scale and mounting corrections remain separate.
2. With the coil on, collect readings at known positions. Express the corrected
   readings and ambient field in the same coordinate directions, then subtract
   the ambient field. Store each position with its coil-field x/y/z components.
   Do not store a difference from a coil-on sensor-home reading.
3. Fit the coil model to these position-and-coil-field measurements. Adapt the
   fitting calculation to use the coil field itself, without a home-field term.
4. Calculate table entries throughout the working area. Each entry pairs a
   position with the model's coil-field vector. The table is derived from the
   fitted model; a denser table adds no independent physical measurements.
5. During tracking, calculate the coil field from each corrected reading using
   orientation and ambient subtraction, then find matching positions in the
   table within an allowed field tolerance. Retain all matching regions until
   the startup selection or movement-history policy resolves the choice.
6. If no acceptable match is found, report an error and no current position.
   Do not silently publish the nearest table entry as a successful match.

The coil center remains the position origin; the sensor does not need to visit
it. The user requires x/y/z definitions to follow the hardware manufacturer's
definitions. Do not assume x = east, y = north, or z = up for sensor readings.
This supersedes the earlier proposed compass-direction convention. The hardware
axis audit below is a prerequisite to choosing the table's coordinate basis.

The table retains the model's mirrored-position and middle-plane ambiguities.
Use the startup selection and visible movement-history warnings agreed below.
Choose grid spacing, interpolation, search indexing, and table coverage during
implementation. The nominal 15 cm range must not become an unannounced hard
tracking cutoff. Check accuracy at physical positions not used for fitting;
looking up values generated by the same model cannot validate that model.

Existing home-relative calibration data must not be silently treated as these
new coil-field measurements. Define an explicit migration or request new
measurements. Record which sensor correction and coil model a table belongs to
so an obsolete table is not reused after those inputs change.

## Hardware axis definitions: refactor requirement

The user flagged existing axis assumptions as a risk on November 17. Preserve
the manufacturer's meaning and positive direction of each native magnetometer
axis. Every conversion to another set of axes must have an explicit purpose,
source, and destination. Do not treat a hard-coded conversion as correct merely
because the simulator and tests use the same conversion. Equally, do not remove
a necessary sensor-alignment conversion merely because it is hard-coded.

The chip's axes turn when the sensor turns. Rotating a sensor without changing
its position changes the components of the field it measures. A single saved
position-to-field table therefore still needs consistent directions for field
comparison. A possible approach is to define the table's directions from the
hardware axes during map calibration, keep those directions fixed, and account
for later sensor rotation explicitly. This is an orientation reference, not a
required sensor-home position. The exact hardware basis and how its orientation
is recovered after restart remain to be specified.

Audit these existing boundaries before changing the tracking calculation:

| Boundary | Current code | Required verification |
|---|---|---|
| Native magnetometer axes to ICM accelerometer/gyroscope axes | [`mag_to_body`](../magtrack_src/hardware/icm20948.py) maps `(x, y, z)` to `(x, -y, -z)` | Compare both package-axis diagrams, pin-1 orientation, and the actual PCB mounting. This maps between hardware definitions; it does not assign compass directions. |
| Gyroscope/accelerometer axes to internal level directions | [`quat_from_gravity`](../magtrack_src/geometry/vectors.py) and initial orientation in [`Tracker`](../magtrack_src/tracking/tracker.py) | Keep this internal rotation distinct from native sensor axes. Initial heading is currently arbitrary. |
| Internal orientation to captured reference directions | `Tracker._q_ref`, `_sample`, and [`ReferenceCapture`](../magtrack_src/tracking/reference_capture.py) | Remove the dependency on a home capture while retaining correct orientation compensation at each magnetic reading's measurement time. |
| MMC axes to common calculation axes | [`fit_axes`](../magtrack_src/calibration/mounting.py) and loaded mounting correction | Verify mounting, handedness and polarity independently; self-consistent cancellation does not prove the native axes were interpreted correctly. |
| Field/table coordinates to displayed and entered positions | [`Locator`](../magtrack_src/position/locate.py), [`position_view.js`](../magtrack_src/web/page/position_view.js), and [`board_view.js`](../magtrack_src/web/page/board_view.js) | Ensure physical calibration measurements, table vectors, displayed axes and saved data all use the declared directions. Do not silently reinterpret old files. |

The manufacturer supplies separate accelerometer/gyroscope and magnetometer
axis diagrams in [ICM-20948 datasheet section 15, Figures 12 and 13](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000189-icm-20948-v1.5.pdf#page=83).
The current fake-bus test checks that the driver performs its specified mapping;
it does not independently establish that this mapping matches the assembled
board. Use physical orientation/rotation checks against the manufacturer and
PCB definitions as well as tests that do not derive their expected axes from
the transformation under test. This inspection has not verified the actual PCB
mounting or changed any numerical mapping.

## Agreed startup

1. Offer reuse of saved sensor/coil calibration or collection of new readings.
   A full tumble is optional when saved corrections remain suitable.
2. Require the coil to be off to measure the ambient field. Tumble also uses the
   coil-off condition. Reusing calibration does not replace this fresh reference
   measurement.
3. Use the accelerometer and calibrated coil-off magnetic readings to establish
   sensor orientation relative to the saved map's declared hardware-based
   directions. Using gravity and the ambient field for orientation does not
   redefine the manufacturer's x/y/z axes as up or north. The saved model and
   live readings must use consistent directions across restarts.
4. After the coil is turned on, find positions matching the measured coil field.
   Do not require a physical sensor home or an exact starting coordinate.
5. If multiple starting locations match, show the choices and ask the user which
   location the sensor occupies. Do not invent a previous position to choose one.

The current model gives matching mirror positions throughout space, not only
near the equator. A whole circle can match in the middle plane. The selection
interface must handle that distinction rather than always present two points.
Coil-off orientation alone does not resolve position ambiguity.

## Agreed tracking and recovery

- Track all around the coil. The approximate 15 cm working range and 5 mm
  accuracy target describe desired performance, not a hard radius cutoff.
- Continue estimates through ambiguous/weak regions using the previous position,
  with a large warning. Avoid weak regions during normal operation where practical.
- Keep the general history warning visible whenever that assumption is used.
  The current model requires it for all returned positions because mirror
  ambiguity persists even with good local sensitivity.
- After a disconnect, continue from the last estimate when readings return.
  Unobserved movement or heading changes can make that estimate wrong.
- Add a distinct large reconnect warning that remains until manually dismissed.
  Dismissal acknowledges the warning; it does not verify recovery or hide the
  separate history warning. Another disconnect must trigger it again.
- If no acceptable position matches, show an error and report no current
  position. Do not publish the closest poor fit as a successful result. Matching
  needs a noise tolerance rather than exact numerical equality.
- While readings are absent or rejected, distinguish any last-known position
  from a newly measured result.
- Set no forced session duration or mandatory reference-refresh interval. Show
  time since the last completed coil-off ambient-field/orientation measurement.

## Agreed correction and calibration policies

| Operation | Policy |
|---|---|
| Magnetometer tumble correction | A completed new fit overwrites the old correction; warn if quality is poor |
| Coil model from known-position readings | A completed new fit overwrites the old model; warn if quality is poor |
| Gyro zero/bias | Update automatically during tracking; preserve actual slow movement |
| Mounting alignment | Separate from the sphere fit; detailed acceptance criteria remain open |
| Ambient-field capture | A fresh reference measurement with coil off, not a sensor correction |

A calculation that cannot produce usable parameters has no new fit to install.
Poor quality and failed computation must remain separate states. Do not change
completed-fit replacement to silently retain the old fit when its quality is red.

## Proposed module boundaries

Keep the existing package layout. The main changes should separate the
responsibilities currently combined inside `Tracker` and `Locator`, rather
than replace the application with a new framework. This table describes the
target responsibilities; the progress section above identifies the completed
extractions. The remaining splits are still recommendations for implementation.

| Responsibility | What it should own | Why separate it |
|---|---|---|
| Hardware-axis conversions | Explicit transformations between named manufacturer-defined sensor axes and the declared common axes | Axis signs and permutations can be independently audited instead of being hidden inside field or display calculations. |
| Board orientation | Gyroscope zero-rate estimate, orientation updates, and timestamped orientation history for one physical board | Both magnetometers observe the same board motion. They can share orientation history while each uses its own magnetic measurement time. Sensor mounting corrections remain separate. |
| Per-sensor magnetic processing | Sensor midpoint/scale/mounting corrections, transformation at measurement time, and ambient subtraction | It can be tested with supplied readings and orientations, independently of HTTP, files, calibration controls and position selection. |
| Ambient measurement and calibration operations | Collection progress, calculated corrections, quality results, and explicit completion/cancellation | Measuring ambient field, correcting a sensor and fitting a coil are different operations. Their state must not be inferred from whether a vector happened to change. Numerical fitting functions remain separate from these controls. |
| Coil model and fitting | Calculating a field at a position and fitting model parameters from recorded position/field pairs | Extract these calculations from `position/locate.py` so they do not own live tracking, worker threads or files. The same model supplies table generation and validation. |
| Field lookup | Generating/indexing the derived table and returning all matching position regions, field mismatches and coverage status | Lookup must not quietly choose a mirrored position using history or declare no match after an incomplete search. It can be checked independently against the model. |
| Position tracking decisions | Startup selection, previous estimate, handling rejected readings, reconnect state, and reasons/warnings attached to each result | Keep the decision to use movement history visible and testable. Changing the search algorithm must not change this policy implicitly. |
| Saved files and recording format | Versioned loading/saving, numerical validation, column definitions, and reading old recordings | Live numerical objects should not parse files. Move shared recording definitions/readers out of `log_plot.py` so writing and comparison do not depend on plotting. |

Use small, named internal records, such as standard-library dataclasses, for
magnetic readings, ambient measurements, coil-calibration measurements and
position results. Record units, measurement time, axis-definition identifier,
and the applicable calibration/model identifiers. Use `measured_field`,
`ambient_field` and `coil_field` according to their actual meanings. Carry full
precision through these records; round and translate historical field names
only when producing JSON, CSV or display text.

Keep one explicit owner of the active setup and operation lifecycle. HTTP
handlers should submit operations to it instead of reaching through `Pair`
into `Locator`. Background calculations should receive a consistent set of
input values and return a result tagged with their input identifiers. Only
the setup owner can apply a still-current result. This owner coordinates the
smaller components; it should not absorb their numerical calculations or
file-format implementations.

The browser should display the position status and warning reasons returned by
tracking, rather than independently decide whether a position is trustworthy.
Its coil drawing currently repeats the model equation in JavaScript. Where
practical, supply field-drawing values from the shared model/table so future
model changes do not require maintaining two physics implementations. Keep
camera interaction and rendering in the browser.

Start by extracting the coil model/fitting and shared recording definitions
without changing their results. Introduce the internal records and explicit
setup identifiers next. Separate orientation and magnetic processing alongside
the hardware-axis audit, then add lookup and tracking-decision components as
the home-free behavior is implemented. Reuse existing tests and compare
representative replay results during structural changes. Avoid generic plugin
systems, a separate class for every small formula, or a wholesale browser
rewrite.

## Implementation plan: November 17

Implement the following stages in order, with a reviewable change and relevant
checks at each stage. This is a plan, not a record of completed features.
Keep the stationary-coil bench setup for this iteration. Coil switching can be
manual initially; automatic electromagnet control is later work. A manual
coil-off confirmation must not be presented as an independently verified coil
state.

### 1. Verify the hardware axes and define the map's directions

Complete the hardware-axis audit above. Trace native readings through sensor
alignment, orientation estimation, magnetic calculations, position coordinates,
and display. Check the chip's pin-1 orientation and PCB mounting against the
manufacturer's definitions. Do not change the existing sign flips without
evidence that they are wrong.

Decide which hardware axes define the common calculations and how the fixed
map directions are recorded. Define how coil-off magnetic readings and gravity
recover sensor orientation relative to those directions after a restart.
Preserve the manufacturer's sensor-axis names and signs; do not assume that
sensor x/y/z mean east/north/up. Keep the coil center as position zero.

**Completion check:** independently specified axis tests and a physical rotation
check agree. Turning the board at a fixed location is accounted for by
orientation, and recovering the saved directions does not require returning
the sensor to a particular position. If the available readings cannot establish
orientation, report that condition instead of inventing one. Resolve this
coordinate design before changing the field calculation.

### 2. Make state changes and saved data reliable

Start with focused regression tests for the reproduced failures in the
[source review](architecture-review-2026-11-12.md), especially R1-R5. Validate
incoming and saved numbers, vector lengths, and transformation shapes. Reject
nonfinite calculations before a position can be published.

Give ambient measurements, sensor corrections, map directions and coil models
explicit identifiers. A calculation started under older settings must not
restore cleared data or publish a position after those settings change. Clearing
measurements must cancel their pending recording and model-fitting work.

Separate the model's field calculation and fitting from live tracking, worker
threads and file storage. Keep full-precision values between calculation stages;
rounded browser values must not become calibration or lookup inputs. Avoid a
general package rewrite: extract the responsibilities needed by this change.

**Completion check:** reset/clear during a fit cannot restore obsolete results;
invalid numbers cannot become a successful position or invalid JSON. Existing
calculations remain consistent during these structural changes. The agreed
policy of applying a completed poor-quality fit with a warning is preserved.

### 3. Replace home capture with coil-off ambient measurement

Add an explicit startup sequence: choose saved or new sensor calibration,
confirm the coil is off, measure the ambient field, establish orientation, then
turn the coil on. Sensor rotation calibration remains a separate operation.

For each live magnetic reading, apply its sensor correction, account for its
orientation at measurement time, and subtract the ambient vector in the same
directions. The result is an estimate of the coil field at the current position.
It must not contain a subtraction of the coil field at a home position.
Keep orientation state/history separate from per-sensor magnetic corrections;
any consolidation of orientation ownership must preserve each sensor's timing.

**Completion check:** simulated coil-off readings cancel through turns, and
coil-on readings retain the coil-field estimate at different positions and
orientations. Restarting at a new position and orientation recovers the declared
map directions. An incomplete startup cannot appear as a valid position.

### 4. Fit the coil model without home-relative measurements

Collect a corrected coil-field estimate at each independently measured position,
in the declared map directions. Keep enough original readings, orientations,
timestamps and calibration identifiers to explain how each estimate was made.
Adapt the model fit to these position-and-field pairs, removing the home-field
term throughout fitting and point checks.

Version the saved format. Preserve old files, but do not treat their
home-relative field differences as new coil-field measurements. Reuse existing
data only through an explicit, justified conversion; otherwise request new
coil-calibration measurements. Sensor-correction or mounting changes must
identify affected measurements/models as needing review or rebuilding, without
silently mixing incompatible data. A fresh ambient measurement in the same
valid coordinate system need not discard an otherwise valid coil model.

**Completion check:** new models can be saved and reloaded without any home
coordinates. Tests use known coil-field vectors rather than home differences.
A failed calculation and a completed fit with poor quality remain distinct;
the latter replaces the old fit and produces the agreed warning.

### 5. Generate and search the dense lookup table

Generate position/field entries from the fitted model. Save the model identity,
axis definition, table spacing, coverage and generation settings with the table.
Treat the table as derived data that can be rebuilt from the saved model.

Implement a search that returns all matching position regions within the field
tolerance. Preserve mirrored answers and broad matching regions; one magnetic
vector must not be stored as a key with only one allowed position. Interpolate
or refine within candidate regions so output is not forced to grid points.
Check the final candidate's field agreement before publishing it.

Choose spacing and search structure by measuring their approximation error,
memory use and query time on the Pi. Prototype how coverage grows or is refined
outside the initial working area; do not turn 15 cm into a hard tracking limit.
If coverage is insufficient, report that explicitly. An incomplete search must
not be described as proof that no position matches, and it must not silently
return the nearest available entry.

**Completion check:** compare table results with direct model calculations at
positions both on and between grid points, including mirrored positions and
the middle plane. Test an out-of-tolerance field, insufficient coverage, and
positions beyond the nominal 15 cm range. This validates the lookup operation;
physical model accuracy is checked separately in stage 8.

### 6. Implement position selection, continuity and recovery

At startup, search without a presumed previous position. If distinct positions
or a broad region match, show the matching possibilities and ask the user to
identify the starting location. Do not quietly choose home or the closest
position to an invented starting point.

During tracking, use the previous estimate to choose between matches under the
agreed policy. Keep a large warning visible whenever movement history is used;
a small field mismatch or good local distinguishability must not hide it.
Show an error and no current position when no acceptable match is found.
Keep a last-known position visibly separate from a new measurement.

On reconnect, continue from the previous estimate with the separate warning
that stays until dismissed. Keep the general history warning independently
visible. Record a new disconnect as a new warning event.

**Completion check:** test ambiguous startup, broad matching regions, movement
through weak regions, rejected readings, and unobserved movement during a
disconnect. Every displayed position has an explicit basis: selected starting
location, current field match, or a match chosen using movement history.

### 7. Replace the home controls and record the new behavior

Replace the sensor-home workflow with the startup and known-position
measurement controls. Show ambient-measurement age, current calibration/model
status, lookup coverage, matching alternatives and the warnings above. Keep
labels tied to their actual calculations, following the terminology rules in
[CLAUDE.md](../CLAUDE.md#conventions).

Version any changed output contracts. Record the identifiers and state changes
needed to explain orientation, calibration, model/table replacement, rejected
positions, history-based choices and reconnect warnings. Include these states
in saved plots. Keep older logs readable without reinterpreting their origins
or historical field names. Update architecture and package documentation as
each behavior becomes implemented; retain this document for remaining work.

**Completion check:** browser and HTTP tests cover the complete startup and
tracking workflow. Recording/replay preserves position validity and warnings,
and the normal workflow contains no sensor-home requirement or hidden home
fallback. No forced session timeout or reference-refresh interval is added.

### 8. Validate on independent physical measurements and the Pi

Test at physical positions and along paths not used for fitting. Restart at
different positions and orientations; rotate in place; include several
heights, mirrored locations, weak regions, disconnects and changed magnetic
surroundings. Check the uniform ambient-field assumption across the working
area with the coil off. Determine field tolerance and quality-warning criteria
from real measurements rather than only the shared simulator/model.

Measure position errors, rejected/ambiguous results, orientation drift, table
approximation error, update rate, latency and memory use. Compare performance
with recording on and off. Keep lookup generation/fitting off the sensor-reading
path and optimize only measured bottlenecks.

**Completion check:** report measured performance against the approximately
5 mm accuracy and 10-20 updates/s targets in the nominal working area. Separate
physical model/sensor error from table approximation error. If the targets are
not met, report the shortfall and revise the model, measurements or implementation;
do not make the results appear better by publishing poor matches.

The first implementation deliverable is stage 1's axis/conversion report and
independent checks. The first behavior change follows only after the map's
coordinate definitions and restart-orientation method are explicit.

## Details still to decide

- Which hardware axis definition is the common basis and how the saved map's
  directions are recovered after restart, without requiring a home position.
- Table spacing and coverage, interpolation,
  and an efficient search that preserves distinct matching positions.
- Field-error tolerance and quality criteria for coil-model/mounting warnings.
- How the user selects a point when the matching set of positions is a broad region.
- How correction changes affect existing coil points/models: warn, invalidate,
  or recompute only when enough original measurement data is available.
- Exact controls and display wording for startup, stale data and refreshes.

These details must not be presented as implemented guarantees. The existing
simulation uses idealized errors and the same coil model as the solver; it is
not an independent demonstration of physical accuracy.
