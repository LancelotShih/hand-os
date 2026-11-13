# Next steps

Agreed behavior from the November 12, 2026 interview. This is the plan for future
work, not a description of features that already exist. See
[architecture.md](architecture.md) for current behavior and the
[source review](architecture-review-2026-11-12.md) for findings and evidence.

## Status at a glance

| Requirement | Current implementation |
|---|---|
| Start anywhere without sensor home | Not implemented; position tracking still requires home -> I don't like this idea because we shouldn't need to depend on this concept. |
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

## Agreed startup

1. Offer reuse of saved sensor/coil calibration or collection of new readings.
   A full tumble is optional when saved corrections remain suitable.
2. Require the coil to be off to measure the ambient field. Tumble also uses the
   coil-off condition. Reusing calibration does not replace this fresh reference
   measurement.
3. Establish up/down from the accelerometer and heading from the calibrated
   magnetometer while the coil is off. The saved coil model and live readings
   must use consistent coordinate directions across restarts.
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

## Suggested implementation order

This sequence is proposed for discussion. The documentation pass does not
implement these remaining changes.

1. **Harden current state handling.** Reject nonfinite values, validate saved
   structures, and stop obsolete fits/captures from undoing resets or clears.
   Add focused regression tests for the reproduced cases in the source review.
2. **Replace the home-dependent reference flow.** Separate the ambient field
   from coil field, initialize heading with coil off, and define stable coordinate
   directions for saved models. Version/migrate saved data explicitly; do not
   silently treat old home-relative field differences as absolute coil readings.
3. **Add starting-position selection and reconnect state.** Keep selected,
   estimated, stale and unavailable results distinct. Implement manual dismissal
   of the extra reconnect warning without clearing the general history warning.
4. **Add explicit warning and recording state.** Show the agreed large history
   warning and stronger weak-position warning. Record those states in logs and
   show them in exported plots. This changes output contracts and must be tested
   as feature work, rather than mixed into the documentation baseline.
5. **Separate responsibilities without changing the math.** Extract the pure coil
   model/fit routines from live locator state and persistence. Give orientation
   one clear owner while preserving measurement-time lookup per sensor. Move the
   recording schema/readers out of the plotting module.
6. **Verify quality and timing.** Define warning criteria, validate on independent
   physical positions/paths, and measure acquisition/solve/logging delays on the
   Pi before choosing performance optimizations.

## Details still to decide

- Field-error tolerance and quality criteria for coil-model/mounting warnings.
- How the user selects a point when the matching set of positions is a broad region.
- How correction changes affect existing coil points/models: warn, invalidate,
  or recompute only when enough original measurement data is available.
- Exact controls and display wording for startup, stale data and refreshes.

These details must not be presented as implemented guarantees. The existing
simulation uses idealized errors and the same coil model as the solver; it is
not an independent demonstration of physical accuracy.
