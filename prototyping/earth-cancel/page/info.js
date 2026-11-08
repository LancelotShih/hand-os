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
  age: "Time since the last capture. Errors tend to grow with time, and Re-capture Earth resets them.",
  bias: "The gyro reads slightly wrong even when still. This amount is measured whenever the board rests, then removed.",
  midpointreadout: "The middle of each direction's readings as the board turns. It should be 0, so we subtract it from every reading.",
  gaps: "Times the sensor readings paused for too long. This should stay at 0.",
  capture: "Measures Earth's field where the board is now, and starts counting turns from here. Use it to clear drift.",
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
