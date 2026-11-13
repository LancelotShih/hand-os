// The little (i) buttons: hover to read, click or tap to keep it open.
const INFO = {
  angleBetweenFields: "The angle between the measured magnetic field and the saved reference field, after expressing both in the same coordinate directions. At 0 degrees they point the same way; at 90 degrees they are perpendicular. This is not the board's rotation angle. Moving near the powered coil can change it. If either field has zero strength, its direction is undefined; the current calculation displays zero degrees as a fallback.",
  largestAngleBetweenFields: "The largest angle between the measured and reference fields since the most recent reference measurement.",
  readingsPerSecond: "How many fresh magnetic readings this sensor provides each second. The displayed field averages several readings. It updates more slowly than this count, and the two sensors can average different time intervals.",
  applyMagnetometerCorrections: "On: subtracts the magnetic midpoint and applies each axis's scale correction from the rotation calibration. Off: skips those software corrections but keeps the sensor-to-board alignment and any hardware correction. The saved corrections are retained. Changing this switch starts a new reference measurement; for position tracking, return the board to sensor home first.",
  estimatedSensorMountingTilt: "The estimated tilt of this magnetometer's axes from the nearest axis-aligned mounting on the board. It is calculated during rotation calibration using the gyroscope and accelerometer. The MMC applies the fitted alignment. The ICM displays this estimate for inspection and continues using its fixed axis mapping. This is not the board's current tilt.",
  differenceBetweenFields: "The size of the measured magnetic field minus the saved reference field, using all three x, y and z components. It reflects differences in both field strength and field direction. With the coil off, a small value is desirable. With the coil on, moving the sensor is expected to change this value.",
  magneticFieldView: "The blue arrow shows the saved reference field. The orange arrow shows the measured field, rotated into the same coordinate directions. The black line joins their tips and represents the difference between the two fields. The board shows the estimated sensor orientation.",
  fieldDifferenceCharts: "These plots show the difference between the measured and reference fields during the last 20 seconds. The first plot is the size of the whole difference; the other three are its signed x, y and z components. Both sensors use the same scale. Colors compare values with the configured field-comparison threshold. With the coil on, movement can cause expected field differences.",
  measuredFieldStrength: "The size of the current magnetic reading after the available sensor corrections that are enabled. This is the combined field from all sources, including the coil when it is on.",
  referenceFieldStrength: "The size of the field saved during the most recent reference measurement. With the coil off, it represents the ambient background. With the coil on at sensor home, it also includes the coil's field there. It is a saved measurement, not a prediction for the sensor's current position.",
  largestFieldDifference: "The largest difference between the measured and reference fields since the most recent reference measurement. This uses the whole three-component difference, not just the difference between their strengths.",
  boardRotationSinceReference: "How far the board's orientation has changed from its orientation during the reference measurement. This is the angle between those orientations, not the total amount it has turned along its path. The coil-off comparison scores use readings at least 30 degrees from the reference orientation by default.",
  boardOrientation: "The board's estimated orientation relative to its orientation during the reference measurement. Yaw is rotation about z, pitch about y, and roll about x, using that rotation order. These angles come from the gyroscope and accelerometer. They do not provide an independently measured compass heading.",
  timeSinceReferenceMeasurement: "Elapsed sensor time since the most recent completed reference measurement. It pauses while readings are absent and includes the gap when they resume. It does not measure time since calibration or guarantee that the reference is still valid.",
  gyroscopeZeroRate: "The x, y and z rotation rates the gyroscope appears to report when the board is still. The tracker estimates these rates from quiet readings and subtracts them during orientation tracking. A slow steady turn can be mistaken for rest.",
  sensorMidpoints: "The fitted x, y and z magnetic midpoints before scale correction, in microtesla. They describe the sensor's fixed magnetic shift. The tracker removes them as part of magnetometer correction. A dash means software correction is off or no midpoint is available.",
  pausesBetweenReadings: "The number of tracker updates separated by more than the allowed time, 0.2 seconds by default. Orientation integration skips that interval because the missed motion is unknown. Sensor reconnects are handled separately, so this is not a count of every disconnection.",
  measureReferenceField: "For position tracking, put the board on its sensor-home mark in the original orientation, with the coil on. This button averages the field there and saves it together with that orientation as the new reference. It includes the ambient background and coil field at home. During coil-off comparison, it instead saves the ambient field.",
  positionView: "Positions are measured in centimeters from the entered coil center, marked 0. The x, y and z directions are those of the board at the reference measurement. The copper block is the fitted coil model; N marks its north end. The black ring is sensor home, blue rings are calibration positions, and the green board is the estimated sensor position.",
  estimatedAndModelledCoilFields: "The orange arrow estimates the coil field from the current magnetic reading: subtract the saved reference, then add the model's coil field at home. This assumes the ambient field is unchanged throughout the working area. Purple arrows or lines show the fitted model's field. Arrow lengths are adjusted for visibility and are not proportional to field strength.",
  estimatedCoilFieldStrength: "The strength of the estimated coil field at the sensor. It is calculated by subtracting the saved reference from the measured field and adding the model's coil field at home. It depends on the model, sensor correction, orientation estimate, and the assumption of a uniform, steady ambient field.",
  sensorHomeCoordinates: "Sensor home is the physical mark where the board is placed for reference measurements. Enter its x, y and z coordinates in centimeters from the coil center. Save sensor home coordinates changes the entered position and refits the coil; it does not measure the field. If the physical mark moves, measure the reference there and replace the old coil-calibration measurements.",
  distanceFromSensorHome: "The straight-line distance from the estimated sensor position to the entered sensor-home coordinates, in centimeters.",
  distanceFromCoilCenter: "The straight-line distance from the estimated sensor position to the entered coil center at 0, in centimeters. This uses the entered origin, even if the fitted model places its center slightly away from it.",
  positionChangePerFieldChange: "The model estimates how many millimeters the sensor could move for a 1 microtesla field change, along the movement direction where the field changes least. A larger value means position is harder to distinguish locally. This is not measured position accuracy. Even a small value cannot distinguish separate positions with identical fields; the solver uses the previous estimate to choose.",
  coilCalibrationMeasurements: "Each entry pairs a magnetic reading with sensor coordinates that you measured independently. Hold the board flat and square as at sensor home, enter its coordinates from the coil center, then record the field. Four measurements allow fitting to start; use 8 to 12 spread through the working area and at different heights.",
  positionDifferenceUsingAllMeasurements: "The distance between this entry's entered coordinates and the position calculated from its magnetic reading, in millimeters. The coil model uses all recorded measurements, including this one. The position calculation also starts at the entered coordinates, so this is a local consistency check, not independently measured tracking accuracy.",
  positionDifferenceWithMeasurementExcluded: "The distance between the entered and calculated position when this measurement is excluded from a local refit of the coil model. The refit starts from the all-measurement model, and the position calculation starts at the entered coordinates. Available with at least five measurements. This checks local consistency; it does not test finding a new position from an unknown starting point.",
  calibrateMagnetometers: "With the coil off or far away, slowly rotate the rigid sensor assembly through every orientation near one spot. This measures each magnetic axis's midpoint and scale correction. It also estimates how the magnetometer is aligned with the gyroscope. This rotation procedure is called a tumble calibration in the code.",
  clearSensorCalibration: "Clears the magnetometers' software corrections, saved rotation-calibration results, and gyroscope zero-rate estimates. It then starts a new reference measurement. It keeps recordings, MMC mounting alignment, and the coil's sensor-home coordinates, recorded measurements, and model. Those coil measurements may need replacing after sensor corrections change.",
  calibrationDirectionCoverage: "The ball shows which directions the ambient field has pointed in the sensor's own axes during calibration. Green regions have been covered; gray ones still need readings. Rotate the board through all sides while staying near one spot with the coil off or far away. Direction counting starts only after all three axes have a sufficiently wide and balanced reading range.",
  recordMeasurements: "Saves sensor readings and displayed estimates in a new timestamped folder. Stop recording generates plots and a link to them. Some displayed values are averages, and MMC readings can repeat or be overwritten before recording. The saved files do not record every calibration change or movement assumption.",
  fittedLowestReading: "The lowest value predicted for this sensor axis by the calibration's fitted shape. It is not necessarily the smallest individual reading collected.",
  fittedHighestReading: "The highest value predicted for this sensor axis by the calibration's fitted shape. It is not necessarily the largest individual reading collected.",
  fittedMidpoint: "Halfway between the fitted lowest and highest readings for this axis. This is the axis's estimated magnetic shift. The correction subtracts it before multiplying by the axis's scale correction.",
  halfReadingRange: "Half the difference between the fitted highest and lowest readings for this axis. The axis scale corrections make these three half-ranges equal.",
  axisScaleMultiplier: "The multiplier applied to this axis after subtracting its fitted midpoint. A value above 1 increases its readings; a value below 1 reduces them. For example, 1.03 increases them by 3 percent.",
  calibrationStrengthVariation: "The displayed value bounds the difference from the mean corrected field strength for 95 percent of the readings used to calculate the calibration. It checks consistency of field strength, not independent field-angle or position accuracy. A completed calibration is still saved when its quality result exceeds the threshold.",
};

const infoTip = document.createElement("div");
infoTip.id = "infotip";
document.body.appendChild(infoTip);
let infoPinned = null;

function showInfo(button) {
  infoTip.textContent = INFO[button.dataset.info];
  infoTip.style.display = "block";
  const r = button.getBoundingClientRect();
  const left = Math.min(r.left, innerWidth - infoTip.offsetWidth - 8);
  const below = r.bottom + 6 + infoTip.offsetHeight < innerHeight;
  infoTip.style.left = Math.max(8, left) + "px";
  infoTip.style.top = (below ? r.bottom + 6 : r.top - infoTip.offsetHeight - 6) + "px";
}

function hideInfo() {
  infoTip.style.display = "none";
  infoPinned = null;
}

document.querySelectorAll(".info").forEach(button => {
  button.type = "button";
  button.textContent = "i";
  button.setAttribute("aria-label", INFO[button.dataset.info]);
  button.addEventListener("mouseenter", () => { if (!infoPinned) showInfo(button); });
  button.addEventListener("mouseleave", () => { if (!infoPinned) hideInfo(); });
  button.addEventListener("focus", () => { if (!infoPinned) showInfo(button); });
  button.addEventListener("blur", () => { if (infoPinned === button) hideInfo(); });
  button.addEventListener("click", e => {
    e.stopPropagation();
    if (infoPinned === button) { hideInfo(); return; }
    infoPinned = button;
    showInfo(button);
  });
});
document.addEventListener("click", hideInfo);
document.addEventListener("keydown", e => { if (e.key === "Escape") hideInfo(); });
window.addEventListener("scroll", hideInfo, {passive: true});
