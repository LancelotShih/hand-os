// The little (i) buttons: hover to read, click or tap to keep it open.
const INFO = {
  direction: "The angle between where this sensor says Earth's field points and where it should point. 0 is perfect.",
  worstdirection: "The biggest direction error since the last capture.",
  rate: "How many readings this sensor gives each second. Each side's charts and big number use averages about 50 ms long, so the two are judged over equal time.",
  usecal: "On: uses this sensor's tumble midpoints and corrections. Off: uses the readings as they came, so you can see what the sensor gives by itself. How the sensor sits on the board is still used, because that's needed to follow a turn. It's measured once and kept, even by Reset calibration. Changing this measures Earth again, so hold the board roughly still.",
  tilt: "How far the sensor sits from square with the gyro. The MMC is corrected for this, because it's on its own breakout. The ICM's magnetometer is built in, so it isn't, and its number shows how accurate this measurement is.",
  error: "How far the sensor's reading is from what we expected. Smaller is better, and 0 would be perfect.",
  view: "The blue arrow is the Earth field we expect. The orange arrow is what the sensor reads. The black line is the gap between them, which is the error.",
  chart: "The error over the last 20 seconds, split by direction. Both sensors share one scale, so their plots compare directly. Green means inside the limit, red means outside it.",
  measured: "The field strength the sensor reads right now, after corrections.",
  predicted: "The field strength we expect, from your last capture of Earth's field.",
  worst: "The biggest error seen since the last capture.",
  turned: "How far the board has turned since the last capture. The error only proves much after about 30 degrees.",
  facing: "Which way the board is pointing, compared with the last capture.",
  age: "Time since the last capture. Errors tend to grow with time, and Capture at sensor home resets them.",
  bias: "The gyro reads slightly wrong even when still. This amount is measured whenever the board rests, then removed.",
  midpointreadout: "The middle of each direction's readings as the board turns. It should be 0, so we subtract it from every reading.",
  gaps: "Times the sensor readings paused for too long. This should stay at 0.",
  capture: "Put the board on its tape mark (the sensor home), squared up, with the coil on, then click. It measures Earth's field (and the coil's, at the sensor home), and starts counting turns from here. Use it to clear drift. Always capture on the mark: the field there is what every position is worked out from.",
  posview: "Everything is measured from the coil's center, marked 0, along the ICM's x, y and z as it sits at the sensor home. The cube is 15 cm each way from 0. The copper block is the coil, drawn where the fit puts it, with N at its north end. The black ring is the sensor home. Blue rings are the sensor positions of your calibration points. The green board is the ICM, with its trail behind it and a dashed line down to the sensor home's height.",
  posfieldview: "The orange arrow on the board is the coil's field the ICM reads right now, with Earth taken out. The purple arrows or lines are the field the fitted coil makes everywhere in the cube. Arrows are longer where the field is stronger, but not in proportion: near the coil it's thousands of µT. Purple dots are where the fit puts the coil's two ends. Where the orange arrow doesn't match the purple ones around it, the fit or the reading is off.",
  poscoilfield: "How strong the coil's field is where the ICM is, from its reading with Earth taken out. It's the error plus the coil's field at the sensor home.",
  home: "The sensor home is where the ICM sits on its tape mark when you click Capture at sensor home. It's not the coil's position: the coil is always 0. Enter how far the ICM is from the coil's center along the ICM's own x, y and z (z is up). For example 15, 0, 0 means the sensor is 15 cm along its x from the coil. If you correct it later, your calibration points stay where they are and the coil is fitted again. The fit lets the coil's center move up to 4 cm from 0, and says how far it moved: more than about 1 cm means the sensor home or the points were measured wrong.",
  posfrom: "How far the ICM is from the sensor home, in a straight line.",
  poscoil: "How far the ICM is from 0, the coil's center, in a straight line.",
  posprecision: "How much the position can move for each 1 µT of field error, in the worst direction. Under 1.5 is good. Beside the middle of the coil, the field is the same all the way round the coil, so moving round it barely changes the reading and this number gets large.",
  calpoints: "Each point is the ICM's reading with the sensor held at a position you measured. Put the board there, flat and square like at the sensor home. Enter the sensor's position from the coil's center along the ICM's x, y and z (z is up), hold still and click Capture point. After 4 points the coil is found. 8 to 12 points, spread around and some raised, give a better fit.",
  offby: "How far this point solves from where you said it was, using the coil fitted to all points.",
  leftout: "The honest check: the coil is fitted again without this point, then this point is solved. This is close to the accuracy you'll get. Shown once there are 5 points.",
  tumble: "Turn the board in every direction. It measures how each sensor sits on the board, plus its midpoints and corrections. Do this once per setup.",
  reset: "Erases all calibration and starts as if this were the first run. Recorded logs and how the MMC is mounted are kept.",
  sweep: "Each sensor has to see Earth's field from every side. Each ball turns as your board does, and its black ring marks the side the field is coming through. Green patches are done, grey ones aren't, and the line is the path so far. Check every side by turning the board over. The MMC's ball uses the MMC's own axes, so its ring sits somewhere different from the ICM's. That's expected: it just needs to turn green too.",
  log: "Records everything to a new folder named with the time, with a graph and the raw numbers.",
  lowest: "The lowest this direction reads as the board turns. This comes from the fitted shape, so a single reading may not reach it.",
  highest: "The highest this direction reads as the board turns. This comes from the fitted shape, so a single reading may not reach it.",
  midpoint: "Halfway between lowest and highest. It should be 0, so we subtract it from every reading.",
  halfswing: "Half the distance from lowest to highest. All three should match, because they're all measuring the same field.",
  correction: "What we multiply this direction by so all three half swings match. 1.03 means it reads 3% low, so we turn it up 3%.",
  match: "If the sensor were perfect, the field strength would read the same however the board is turned. This shows how close it came.",
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
