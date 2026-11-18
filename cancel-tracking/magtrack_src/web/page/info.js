// The little (i) buttons: hover to read, click or tap to keep it open.
const INFO = {
  angleBetweenFields: "The angle between the measured magnetic field and the saved reference field, after expressing both in the same coordinate directions. At 0 degrees they point the same way; at 90 degrees they are perpendicular. This is not the board's rotation angle. Moving near the powered coil can change it. If either field has zero strength, its direction is undefined; the current calculation displays zero degrees as a fallback.",
  largestAngleBetweenFields: "The largest angle between the measured and reference fields since the most recent reference measurement.",
  readingsPerSecond: "How many fresh magnetic readings this sensor provides each second. The displayed field averages several readings. It updates more slowly than this count, and the two sensors can average different time intervals.",
  applyMagnetometerCorrections: "Enables or bypasses software midpoint and scale corrections. Sensor mounting remains applied. Changing corrections stops tracking; the saved map and its measurements must be replaced before using the changed corrections.",
  estimatedSensorMountingTilt: "The estimated tilt of this magnetometer's axes from the nearest axis-aligned mounting on the board. It is calculated during rotation calibration using the gyroscope and accelerometer. The MMC applies the fitted alignment. The ICM displays this estimate for inspection and continues using its fixed axis mapping. This is not the board's current tilt.",
  differenceBetweenFields: "The size of the measured magnetic field minus the saved reference field, using all three x, y and z components. It reflects differences in both field strength and field direction. With the coil off, a small value is desirable. With the coil on, moving the sensor is expected to change this value.",
  magneticFieldView: "The blue arrow shows the saved reference field. The orange arrow shows the measured field, rotated into the same coordinate directions. The black line joins their tips and represents the difference between the two fields. The board shows the estimated sensor orientation.",
  fieldDifferenceCharts: "These plots show the difference between the measured and reference fields during the last 20 seconds. The first plot is the size of the whole difference; the other three are its signed x, y and z components. Both sensors use the same scale. Colors compare values with the configured field-comparison threshold. With the coil on, movement can cause expected field differences.",
  measuredFieldStrength: "The size of the current magnetic reading after the available sensor corrections that are enabled. This is the combined field from all sources, including the coil when it is on.",
  referenceFieldStrength: "The ambient background measured with the electromagnet switched off. It includes Earth and other contributions assumed uniform in magnitude and direction across the working area. It is subtracted as a three-component vector after orientation correction.",
  largestFieldDifference: "The largest difference between the measured and reference fields since the most recent reference measurement. This uses the whole three-component difference, not just the difference between their strengths.",
  boardRotationSinceReference: "How far the board's orientation has changed from its orientation during the reference measurement. This is the angle between those orientations, not the total amount it has turned along its path. The coil-off comparison scores use readings at least 30 degrees from the reference orientation by default.",
  boardOrientation: "Orientation in the saved hardware-defined map axes. Gravity and the coil-off ambient field establish startup orientation. The gyro and accelerometer track it after the coil is on; heading can drift.",
  timeSinceReferenceMeasurement: "Elapsed sensor time since the most recent completed reference measurement. It pauses while readings are absent and includes the gap when they resume. It does not measure time since calibration or guarantee that the reference is still valid.",
  gyroscopeZeroRate: "The x, y and z rotation rates the gyroscope appears to report when the board is still. The tracker estimates these rates from quiet readings and subtracts them during orientation tracking. A slow steady turn can be mistaken for rest.",
  sensorMidpoints: "The fitted x, y and z magnetic midpoints before scale correction, in microtesla. They describe the sensor's fixed magnetic shift. The tracker removes them as part of magnetometer correction. A dash means software correction is off or no midpoint is available.",
  pausesBetweenReadings: "The number of tracker updates separated by more than the allowed time, 0.2 seconds by default. Orientation integration skips that interval because the missed motion is unknown. Sensor reconnects are handled separately, so this is not a count of every disconnection.",
  measureReferenceField: "Switch the electromagnet OFF and hold still. This captures the ambient field and establishes orientation without needing a known sensor position. At first setup, the board hardware axes define the saved map directions. Later starts recover those same directions from gravity and the ambient field.",
  positionView: "The origin is the entered electromagnet center. The optional XY grid is on the z=0 plane with 5 cm spacing. The red North and blue South ends show fitted magnetic polarity, including when field arrows are off. Display controls can swap or reverse axes without changing saved calibration coordinates. Matching choices appear when the magnetic reading cannot determine a unique starting position.",
  estimatedAndModelledCoilFields: "The measured coil field is the corrected, orientation-compensated total field minus the coil-off ambient vector. The fitted model generates the lookup table. A small field mismatch does not prove a unique or accurate position.",
  estimatedCoilFieldStrength: "Magnitude of the ambient-subtracted magnetic vector, in microtesla. It is available after the coil-off measurement and confirmation that the coil is on. Sensor, orientation and ambient-field errors remain in it.",
  distanceFromCoilCenter: "The straight-line distance from the estimated sensor position to the entered coil center at 0, in centimeters. This uses the entered origin, even if the fitted model places its center slightly away from it.",
  positionChangePerFieldChange: "The model estimates how many millimeters the sensor could move for a 1 microtesla field change, along the movement direction where the field changes least. A larger value means position is harder to distinguish locally. This is not measured position accuracy. Even a small value cannot distinguish separate positions with identical fields; the solver uses the previous estimate to choose.",
  coilCalibrationMeasurements: "Record absolute coil-field vectors at independently measured positions, in saved map axes and centimeters from the electromagnet center. At least four distributed positions are required. These measurements fit a model, which generates the lookup table. They do not independently validate that model.",
  positionDifferenceUsingAllMeasurements: "Local position difference using the fitted model, starting at the known calibration position. It checks local consistency, not global uniqueness or independent accuracy.",
  positionDifferenceWithMeasurementExcluded: "Refits without this measurement, then searches starting at its known position. This is a local leave-one-out check. It does not prove that the field has a unique position.",
  calibrateMagnetometers: "Switch the electromagnet OFF and rotate the board through all orientations. This fits sensor midpoint and relative axis-scale corrections. Completed poor-quality fits overwrite old corrections with a quality warning. A changed correction requires a new map and new coil measurements.",
  clearSensorCalibration: "Clears sensor midpoint and scale corrections and gyro zero rate, and stops tracking. Recordings and MMC mounting are kept. Recalibrate the sensors, start a new map, and record new coil measurements afterward.",
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
