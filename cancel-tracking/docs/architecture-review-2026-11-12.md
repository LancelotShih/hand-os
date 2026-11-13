# Architecture review: 2026-11-12

This is a source review of the current fixed-coil bench application. The user
confirmed: continuously powered stationary coil, moving ICM, optional MMC,
about 15 cm range, and a 5 mm accuracy target. The operating code is useful and
working; this review identifies limits and edge cases, not a failed experiment.

The [architecture](architecture.md) describes existing behavior. This document
records findings and proposed changes. The [November 3 critique](critique-2026-11-03.md)
is historical: its paths and some lifecycle recommendations predate current code.

## Evidence and scope

Reviewed all production Python modules, the browser scripts, package guides,
entry point, configuration, and test coverage. No physical sensor was opened,
no hardware settings were changed. The initial baseline retained documentation,
docstrings, comments and explanatory page text changes only. The warning feature
and its schema/test additions were removed from this diff on November 13;
future behavior is recorded separately in next steps.

On Windows/Python 3.13, `python -m unittest` passed all 119 tests in 100.730 s.
The intentional stream disconnect also emitted an uncaught
`ConnectionAbortedError`; the test still passed. Extra in-memory probes with
`unittest.mock` reproduced the lifecycle cases identified below. Timing on the
Pi, hardware axis accuracy, noise, and real position accuracy remain unmeasured.

Priority means suggested repair order: P1 is state validity/correctness; P2 is
validation, measurement integrity, or resilience; P3 is optimization after
measurement. These are not claims about how often an issue occurs in use.

### November 13 baseline diff verification

Removed the warning feature, extra sample/CSV fields, plot banner, display
threshold setting and six feature tests from the documentation baseline. These
remain planned work in next steps. The remaining Python changes are docstrings
and comments: executable ASTs match HEAD. Browser changes are explanatory copy
and comments; HTML structure, controls, CSV schema and tests match the original
behavior. All 119 original tests passed in 63.935 s. The existing Windows
aborted-stream traceback occurred again without failing the suite (R12).
Local documentation links, JavaScript syntax and diff whitespace checks passed.

### Later terminology changes

The subsequent naming pass uses orientation throughout and distinguishes the
uniform ambient field from the captured total reference field. Python names
and the reference-capture module were renamed; the field calculations and
control flow remain the same. Existing JSON/CSV field names remain compatible.
Two tests now cover a non-Earth background contribution and a coil-on reference.
The locator tests also wait for background saves before removing temporary files.
The 119-test documentation-only baseline above predates these naming changes.

After the naming pass, all 121 tests passed in 72.703 s. Across 30 production
Python files, executable ASTs match the pre-pass source after normalizing renamed
identifiers and two explanatory messages, and excluding docstrings. Local links,
browser-script syntax and diff whitespace checks passed. The existing Windows
aborted-stream traceback remains as described in R12.

| Findings | Main source locations |
|---|---|
| R1-R4: reference and lifecycle | [Tracker](../magtrack_src/tracking/tracker.py), [Pair](../magtrack_src/tracking/pair.py), [Locator](../magtrack_src/position/locate.py) |
| R5-R7: numerical validity and fit checks | [Calibration store](../magtrack_src/calibration/store.py), [tumble fit](../magtrack_src/calibration/tumble.py), [mounting fit](../magtrack_src/calibration/mounting.py), [position solver](../magtrack_src/position/locate.py) |
| R8: motion assumptions | [Gyro bias](../magtrack_src/orientation/gyro_bias.py), [Madgwick](../magtrack_src/orientation/madgwick.py), [capture](../magtrack_src/tracking/reference_capture.py) |
| R9-R11: acquisition and recording | [MMC reader](../magtrack_src/hardware/mmc_loop.py), [ICM loop](../magtrack_src/hardware/icm_loop.py), [logger](../magtrack_src/recording/data_log.py), [scoring](../magtrack_src/recording/compare.py), [plots/schema](../magtrack_src/recording/log_plot.py) |
| R12: presentation | [Server](../magtrack_src/web/server.py), [page controls](../magtrack_src/web/page/app.js), [position controls](../magtrack_src/web/page/position_view.js), [field renderer](../magtrack_src/web/page/field_view.js) |

## P1: reference and calibration state need explicit ownership

### R1. Recapture is detected from a rounded measurement

**Evidence:** `Locator.update` compares `sample["predicted"]` with `_reference_field`.
`Tracker._sample` rounds that vector to two decimals. A new capture with the
same rounded vector is not detected. Conversely, reference replacement after a
toggle or tumble is treated as proof that the board returned physically home.

**Reproduced:** set an existing locator's last position to `(12,0,0)`, home to
`(10,0,0)`, and send a new reference with the same rounded field. A stub solver
receives `(12,0,0)` as its starting position, rather than home.

**Consequence:** resets and coordinate-frame changes are inferred from numbers
that describe the measurement, not from the operation that changed its meaning.

**Proposed boundary:** a reference object with an incrementing identifier,
orientation, field, completion time, and capture reason. Notify the locator
explicitly. A capture identifier alone does not prove physical home; that is
still an operator condition that must be represented in the workflow.

### R2. Coil points can survive changes that invalidate their readings

**Evidence:** points store `B(p)-B(home)` only. They have no magnetic-calibration
or reference identifier. `Pair.reset_calibration`, toggles, and tumbles do not
invalidate the locator's points/model. `Locator.set_home` always retains current
coil-coordinate points and refits against the newly entered home.

**Consequence:** there is no distinction between correcting coordinates of the
same physical mark and moving the mark. Changing gains or mounting also changes
the meaning of existing field differences. A new fit can combine incompatible
measurements and still have a numerical solution.

**Proposed boundary:** associate points/models with a measurement setup revision.
Allow a coordinate correction of an unchanged mark separately from creating a
new physical home/reference. Define which changes require recapture or clearing
points. Repeated captures at the same physical pose should not automatically
discard an otherwise valid model.

### R3. Clearing points does not clear all related work

**Evidence:** `Locator.clear_points` does not clear `_capture` or `fitting`.
`_fit` discards a changed-points result without updating `fitting`.

**Reproduced:** start a point capture, feed one sample, clear points, then feed
20 more samples: a point appears again. Block a fit, clear points, then release
the fit: `fitting` remains true after the worker exits. The normal page disables
Clear during capture, but the HTTP route and service do not enforce that rule.

**Proposed change:** clear/cancel/reset should invalidate pending capture and
fit work through one revision counter and consistently reset the public state.
Test the service contract independently of disabled browser controls.

### R4. Locks protect access, but not the lifetime of a calculation

**Evidence:** `Tracker.tumble_finish` releases its lock to fit and commits later
without checking whether reset, a new tumble, or another control superseded it.
`Locator.update` snapshots a model, solves outside its lock, and updates `_last`
without verifying that the model/home/reference still match.

**Reproduced with event-controlled mocks:** a tumble paused in `fit_axes` restores
its offset after `reset_calibration` returns. A solve overlapping `clear_points`
still publishes a position from the cleared model. Coil fitting has a
points/home comparison guard, but it does not solve all these other races.

**Proposed boundary:** snapshot a revision before calculation and check it before
commit/publication. Serialize related control transitions. Do not solve this
by holding the sensor-loop lock through long fits.

## P2: numerical validity and acceptance

### R5. Nonfinite values can become apparently valid output

**Evidence:** JSON loaders convert numbers but generally do not validate vector
length, matrix shape, finiteness, orthogonality, or gain/length bounds. Valid JSON
with the wrong container type can also escape the loader's exception list.
`least_squares` returns its initial estimate for a nonfinite initial cost;
`miss > limit` does not reject NaN in `Locator.update`.

**Reproduced:** a locator with a valid coil and home accepts an error containing
NaN, returns home as a position, and emits `"mismatch_ut": NaN`. Python's default
JSON encoder permits that token; the browser's `JSON.parse` does not.

**Proposed change:** validate persisted and incoming structures once at their
boundaries, require finite solver inputs/results, and publish invalid-state
reasons rather than fabricated position success. Use strict JSON serialization
as an additional check, not the sole input validator.

### R6. Fit completion and fit quality are different states

**Evidence:** `fit_tumble` reports `passed=False` for excessive sphere spread,
but `Tracker.tumble_finish` still applies/saves that result. Mounting fitting is
unconditional, including for the ICM that will not apply it, and its exceptions
are outside the ellipsoid fit's error handler. There is no mounting-spread
acceptance gate. Four coil points merely enable fitting; no fit-quality or
conditioning gate establishes useful accuracy.

**Proposed change:** separate candidate creation, quality assessment, acceptance,
and persistence, and test failure of each stage independently. The user has now
confirmed that a completed magnetometer tumble should overwrite the previous
correction with a warning for poor quality. Applying a red but computable tumble
therefore agrees with intent; changing it to always retain the old correction
would not. The user also confirmed applying a completed new coil model with a
warning for poor quality; its overall warning criteria remain to be defined.
Mounting-fit acceptance remains a separate policy.

### R7. Existing accuracy checks are local and optimistic

**Evidence:** `check_points` starts the position solve at the known answer.
`left_out_check` excludes a point from refinement residuals, but initializes from
the model fitted to all points and starts localization at the held-out position.
The simulator generates coil fields using the same two-pole model that is fit.

**Consequence:** the earlier wording “Left out is the honest accuracy” overstated
this result (corrected in this documentation pass). It
does not test independent global model discovery, wrong-side recovery, a new
trajectory, or physical model mismatch. Low field mismatch can coexist with
large position error, especially near the middle plane.

**Proposed change:** retain these as local consistency checks; separately test
new physical positions and paths. For stricter cross-validation, refit from
training points alone and seed localization using the operational starting
condition, not the held-out coordinates.

### R8. Orientation and capture assume more than their names imply

**Evidence:** `GyroRest` accepts a quiet window with mean components below
3 degrees/s. It only observes gyro values, not physical rest. A steady
`(0,0,1)` degrees/s stream is learned as bias after one second (reproduced).
The rest window is not reset on sensor loss, and elapsed time across a gap can
complete it with very few samples. Madgwick normalizes and uses every nonzero
accel vector; it cannot distinguish gravity from hand acceleration here.

`ReferenceCapture` averages magnetic readings in sensor axes and picks a middle
orientation. Its previous claim that wobble averages out is not generally
true. Capture does not use the delayed orientation lookup used by live fields.

**Proposed change:** explicit startup rest, stronger bias-update eligibility,
gap-aware windows, and a calibrated accelerometer with motion-aware correction.
Average capture readings in a common frame with measurement-time orientations.
Keep the present approximation documented until measurements justify a change.

## P2: acquisition, comparison, and persistence

### R9. The two magnetometers are not synchronized observations

**Evidence:** MMC conversions can be overwritten in `MmcReader._fresh` before
consumption. `Pair._last` repeats old MMC view samples at the ICM's cadence.
It is not cleared by pair capture/reset/tumble controls. MMC-only reconnect
keeps its slot, cached view, tracker history, and any partial average block.
There is no sample ID/time in the cached view to identify freshness downstream.

`compare` scores those repeated values using ICM turn and row timing. Fixed
sample counts target approximately 50 ms at nominal rates, but actual block
durations vary. The raw-noise calculation does use 50 ms time bins, separately
from live block averaging. It still relies on a gyro threshold to identify rest.

**Consequence:** comparison reflects sensor, calibration, timing, geometry, and
pipeline differences. Same gyro inputs do not prove differences are solely due
to the sensor chips. The sensors are about 1 cm apart, which also matters in
the coil's nonuniform field.

**Proposed boundary:** timestamped readings and view blocks with intervals,
sensor sequence, and reference revision. Define cache invalidation, age limits,
and what a comparison counts. Use coil-off data for ambient-field cancellation scoring.

### R10. The acquisition path includes work that can delay acquisition

**Evidence:** live nonlinear position solving runs in `Pair.update` on the ICM
thread. `DataLog.add` formats/writes rows and flushes both files synchronously
under the log lock. Log start/stop also hold that lock for file work. Every point
change after the minimum count starts another fit thread; obsolete fits are
discarded only after calculation. Leave-one-out adds multiple refinements.

**Consequence:** releasing a fit's state lock does not establish a nonblocking
or real-time sensor loop. Pure Python fitting can compete for interpreter time.
There is no measured Pi latency budget or queue-overflow policy yet.

**Proposed change:** first record acquisition intervals and durations of solve,
logging and fit work on the Pi. Then consider one coalescing fit worker, cached
diagnostics, and a bounded writer queue with an explicit data-loss policy.
Preserve timestamp semantics when moving work between threads.

### R11. Saved state is not yet a complete experiment record

**Evidence:** `info.json` captures status only at start, without all algorithm
settings, code/schema version, and later control/bias/model events. Tumble
archives omit their quaternion history. `CalibrationFile` writes directly to
final paths, unlike the locator's serialized temporary-file replacement.
Calibration loaders can silently fall back to uncalibrated defaults.

Historical `samples.csv` positions used a home origin; current plots label the
same column names as coil-relative without origin metadata. Regenerating plots
and comparisons also uses today's thresholds from config, not recorded ones.

**Proposed boundary:** a versioned recording schema and calibration store, atomic
validated writes, configuration/revision metadata, and control events. Make
legacy origin handling explicit. File flushing is not a guarantee of durability
through power loss; the current one-second flush comment is too strong.

### R12. Presentation has its own state and error assumptions

**Evidence:** general `app.js` POSTs ignore the response, so some rejected
controls have no visible explanation. The position controls do read error
responses. Cached measurements remain displayed when acquisition stops.
Browser plots use arrival times and reconnect replays retained samples without
SSE IDs. On Windows, an aborted stream connection is not covered by the two
disconnect exceptions currently caught.

The browser duplicates the coil equation and reconstructs the model from rounded
status fields. It is a visualization, not a second independent measurement of
model accuracy. Script order and several math/threshold constants are shared
informally through globals and matching literals.

**Proposed change:** one control-request helper with consistent error display,
explicit stale/invalid sample state, and sample acquisition times. Add small
cross-language field fixtures if that duplicate equation is retained. Keep
visualization tuning separate from numerical acceptance requirements.

### R13. Position selection relies on history without a dedicated warning

**Evidence:** `solve` starts at the previous estimate and chooses the nearer
mirror solution. `Locator.update` rejects excessive field mismatch, not multiple
matching positions. The page colors local sensitivity but has no large ambiguity
warning; CSV position columns contain no history/sensitivity flags.

**Consequence:** an estimate can match the field well while its choice depends
on the starting guess and previous motion. Mirror ambiguity occurs away from the
equator too. A small sensitivity number does not eliminate the other solution.

**Agreed future behavior:** continue estimates using history with a large
warning, let the user select an ambiguous starting position, and retain a
separate reconnect warning until dismissed. These changes are in
[next steps](next-steps.md), not the baseline. The warning feature briefly added
in the working diff was removed on November 13 to keep this baseline focused on
documenting existing behavior.

## Physical and algorithmic assumptions to retain explicitly

| Assumption | Where it enters | What would invalidate it |
|---|---|---|
| Ambient field is uniform and stable | One captured `reference_field` vector | Translating near steel, changing nearby currents, moving magnetic objects |
| Coil pose/output stay fixed | Saved `Coil` and point differences | Moving/rotating the coil, current or core-state changes |
| Reference is taken on the same mark and orientation | Capture and position coordinates | Capturing elsewhere, changing the board's heading/tilt on the mark |
| Sensor offset/distortion is constant in its own axes | Tumble correction | Changing attached wiring/materials, temperature dependence, flexing assembly |
| Distortion is axis-aligned and approximately linear | Six-parameter ellipsoid | Cross-axis distortion, clipping, nonlinear response |
| Tumble data describe one constant-strength field | Ellipsoid and mounting fits | Translation through gradients, an attached source with changing relative pose |
| Acceleration mostly represents gravity | Initial orientation and Madgwick | Hand acceleration, vibration, accelerometer offset/scale errors |
| Quiet gyro means rest; gyro scale/timing are reliable | Bias learning and integration | Slow steady rotation, scale error, missing/late reads |
| A fixed magnetic delay is adequate | AK history lookup | Variable bus/filter latency or fast rotation |
| MMC SET/RESET removes enough offset for software correction to default off | `MMC_USE_CALIBRATION=False` | Board-fixed magnetic fields or remaining gain/distortion errors; hardware bridge-offset removal is not whole-assembly calibration |
| ICM magnetometer alignment is sufficiently described by the hard-coded axis map | ICM ignores fitted mounting | Actual package misalignment, wrong board-axis interpretation |
| A two-pole source approximates the actual coil in the working volume | Coil fit and solver | Core/geometry effects not absorbed by the model |
| Previous position identifies the correct solution branch | Mirror choice and solver seed | Startup elsewhere, large jumps, loss of tracking, crossing weak geometry |
| Small mismatch/sensitivity imply a useful local estimate | Position display | Model bias, global ambiguity, uncertain coil parameters |
| Enough point count implies usable geometry | Four-point fit trigger | Clustered, redundant, or poorly measured points |

Some limits are fundamental to this source model. In particular, mirrored
positions have identical field, and the middle plane has a continuous ambiguity.
Modularizing or improving an optimizer cannot create information absent from
the measurements. The nominal 15 cm range and `mm_per_ut` are not validity gates.

## Modularity: keep the structure, extract specific responsibilities

The package structure already separates drivers, math and presentation well.
The largest gains come from clarifying state ownership, not adding a generic
plugin framework or splitting every small function into a class.

| Proposed responsibility | Extract from | Purpose |
|---|---|---|
| `ReferenceCapture` / reference record | `Tracker`, inferred locator state | Explicit capture lifecycle, frame identity, completion event |
| `OrientationState` | Two trackers | One owner of gyro bias, orientation, history, and reconnect policy |
| Magnetic correction + timed block averaging | `Tracker` | Pure per-sensor transformations using measurement-time orientation |
| Calibration coordinator | Tracker control methods and store calls | Candidate/accept/save transition with revision guards |
| Coil model | `position/locate.py` | Pure field, mirror, Jacobian/sensitivity calculations |
| Fit/validation routines | `position/locate.py` | Numerical work without live state, threads, or file I/O |
| Locator service + store | Remaining `Locator` | Own home/points/model revision, worker lifecycle and persistence |
| Recording schema/readers | `recording/log_plot.py` | Writer and scorer no longer depend on plotting for their data contract |
| Control/request handling | Browser scripts | Consistent error handling and state transitions |

Do not share orientation by merely passing one quaternion to both sensors:
each reading needs its own timestamp lookup, and per-sensor capture frames must
remain explicit. Keep full-precision numerical results until serialization;
currently rounded view values feed localization and calibration-point capture.

## Suggested implementation sequence

1. Add focused regression tests for R1-R6, then fix revision handling, clear/reset
   semantics, nonfinite rejection, and candidate acceptance with current math.
2. Define reference/setup identity and point invalidation rules, including the
   distinction between correcting coordinates and moving the physical mark.
3. Extract pure coil/model/fit code and a recording schema without changing
   numerical behavior. Compare representative replay outputs before/after.
4. Consolidate orientation ownership while preserving sample-time alignment.
5. Measure Pi timing; optimize the slow work actually affecting acquisition.
6. Use independent physical validation before changing calibration/model math.

Additional performance candidates, not measured bottlenecks: tumble coverage
rescans accumulated points every 25 readings; history lookup scans backward;
mounting subsampling uses floor division and can retain almost twice its stated
1500-point target; the tumble canvas repaints pixels each animation frame.
Algorithm bounds and display constants also exist outside `config.py`, despite
the old claim that every setting lives there.
