// Live data from the server, the readouts and the buttons.  Every sensor readout
// goes through the same functions, once per side.
let latest = null, state = null;
const HIST_S = 20;

function post(path) { fetch(path, {method:"POST"}); }
$("capture").onclick = () => post("/capture");
$("tumble").onclick = () => post("/tumble/start");
$("tumblefinish").onclick = () => post("/tumble/finish");
$("tumblecancel").onclick = () => post("/tumble/cancel");
$("reset").onclick = () => {
  if (confirm("Erase all calibration and start over? Recorded logs and the MMC's mounting are kept.")) {
    post("/calibration/reset");
  }
};
SIDES.forEach(side => { side.part("usecal").onchange = () => post("/calibration/" + side.key + "/toggle"); });
$("log").onclick = () => post(state && state.log.active ? "/log/stop" : "/log/start");

// The server sends the ICM at the top level and the MMC, in the same shape, inside it.
const sampleOf = side => side.key === "icm" ? latest : latest && latest.mmc;
const stateOf = side => side.key === "icm" ? state : state && state.mmc;

function showSample(side) {
  const s = sampleOf(side), put = (name, text) => { side.part(name).textContent = text; };
  const big = side.part("big"), verdict = side.part("verdict");
  big.className = "big";
  verdict.className = "verdict";
  put("mm", s ? s.measured_mag.toFixed(1) + " µT" : "–");
  if (!s || s.error_ut === null) {
    big.textContent = "–";
    verdict.textContent = s ? "no Earth reference yet" : "no readings yet";
    ["deg", "peakdeg", "peak", "pm"].forEach(name => put(name, "–"));
    return;
  }
  big.textContent = s.error_deg.toFixed(1) + "° · " + s.error_ut.toFixed(1) + " µT";
  put("deg", s.error_deg.toFixed(1) + "°");
  put("peakdeg", s.peak_deg.toFixed(1) + "°");
  put("peak", s.peak.toFixed(1) + " µT");
  put("pm", s.predicted_mag.toFixed(1) + " µT");
  const lim = state ? state.pass_ut : 5, min = state ? state.pass_min_turn : 30;
  const ok = s.error_ut < lim, mark = ok ? " good" : " bad";
  big.className += mark;
  verdict.className += mark;
  verdict.textContent = !ok ? "over " + lim + " µT"
    : s.turn < min ? "within " + lim + " µT, turn the board to test it"
    : "within " + lim + " µT";
}

function sideStatus(s) {
  if (!s) return "Switched off (start with --mmc to use it)";
  if (!s.connected) return "Not connected" + (s.error ? ": " + s.error : "");
  if (s.capturing === "earth") return "Measuring Earth's field…";
  if (s.message) return s.message;
  if (!s.mounted) return "Mounting not measured: Tumble once, ever. Until then, errors while turning mean nothing.";
  if (!s.calibration_on) return "No calibration: cancelling Earth from the gyro and the first reading.";
  return s.have_offset ? "" : "Not calibrated yet: click Tumble.";
}

function showSideState(side) {
  const s = stateOf(side), put = (name, text) => { side.part(name).textContent = text; };
  put("status", sideStatus(s));
  put("rate", s && s.connected ? s.rate_hz.toFixed(0) + " readings a second" : "");
  put("ratenum", s ? s.rate_hz.toFixed(0) : "–");
  put("age", !s || s.ref_age_s === null ? "–" : s.ref_age_s.toFixed(0) + " s");
  // shown before the correction, so it matches the tumble table
  put("off", s && s.calibration_on && s.offset ? s.offset.map((v, i) => (v / s.gains[i]).toFixed(1)).join(", ") : "–");
  const usecal = side.part("usecal");
  usecal.disabled = !s || s.tumbling;
  if (s && usecal.checked !== s.calibration_on) {
    usecal.checked = s.calibration_on;
    side.hist.length = 0;               // the old error was measured the other way
  }
  const r = s && s.tumble_result;
  put("tilt", r ? r.mounting.tilt_deg.toFixed(1) + "°" : "–");
  showTumble(side, s);
  showResult(side, r, s && s.calibration_on);
}

function showTumble(side, s) {
  side.part("tumble").style.display = s && s.tumbling ? "block" : "none";
  if (!s || !s.tumbling) { side.trail.length = 0; return; }
  const need = state.tumble_min_coverage, bar = side.part("tumblebar");
  bar.style.width = Math.min(100, s.tumble_coverage / need * 100) + "%";
  bar.className = "tumblebar" + (s.tumble_ready ? " ok" : "");
  const swept = s.tumble_swept.length, total = 26, needed = Math.ceil(need * total);
  side.part("tumbleinfo").textContent = s.tumble_counting
    ? swept + " of " + total + " directions swept (need " + needed + "), " + s.tumble_points + " readings"
    : "Keep turning, flip it over fully. Half the swing so far: X " + s.tumble_swing[0].toFixed(0) + ", Y "
      + s.tumble_swing[1].toFixed(0) + ", Z " + s.tumble_swing[2].toFixed(0)
      + " µT. Dots start counting once all three are about equal.";
}

function showResult(side, r, applied) {
  const rows = side.part("resultrows");
  side.part("result").style.display = r ? "block" : "none";
  if (!r || rows.dataset.points === String(r.points) + r.field_ut + applied) return;
  rows.dataset.points = String(r.points) + r.field_ut + applied;
  const verdict = side.part("resultverdict");
  verdict.className = "resultverdict " + (r.passed ? "good" : "bad");
  verdict.textContent = (r.passed ? "Good: " : "Not good enough: ") + "readings match to within "
    + r.spread_after_ut.toFixed(1) + " µT (limit " + r.limit_ut + " µT)";
  side.part("resultnote").textContent = "The three directions disagreed by " + r.asymmetry_ut.toFixed(1)
    + " µT. Correcting that improved the match from " + r.spread_before_ut.toFixed(1) + " to "
    + r.spread_after_ut.toFixed(1) + " µT. Measured field strength: " + r.field_ut.toFixed(1) + " µT."
    + (applied ? "" : " Not applied: software calibration is off.");
  rows.textContent = "";
  ["X", "Y", "Z"].forEach((axis, i) => {
    const row = document.createElement("tr");
    const low = r.center[i] - r.radii[i], high = r.center[i] + r.radii[i];
    [axis, low.toFixed(1), high.toFixed(1), r.center[i].toFixed(1), r.radii[i].toFixed(1),
     r.gains[i].toFixed(3)].forEach(text => {
      const cell = document.createElement("td");
      cell.textContent = text;
      row.appendChild(cell);
    });
    rows.appendChild(row);
  });
}

function showShared() {
  const s = state, l = latest;
  $("capture").disabled = s.tumbling;
  $("tumble").disabled = s.tumbling;
  $("tumblecard").style.display = s.tumbling ? "block" : "none";
  $("tumblefinish").disabled = !s.finish_ready;
  $("bias").textContent = s.bias.map(v => v.toFixed(2)).join(", ");
  $("gaps").textContent = s.gaps;
  if (l) {
    $("turn").textContent = l.turn.toFixed(0) + "°";
    $("ypr").textContent = l.euler.map(v => v.toFixed(0) + "°").join(" / ");
  }
  showLog(s.log);
}

function showLog(l) {
  $("log").textContent = l.active ? "Stop log" : "Start log";
  $("log").className = l.active ? "primary" : "";
  $("logstat").textContent = l.error ? l.error
    : l.active ? "Logging " + l.name + " (" + l.seconds.toFixed(0) + " s)" : "";
  const link = !l.active && l.last;
  $("loglink").textContent = link ? "View " + l.last : "";
  if (link) $("loglink").href = "logs/" + l.last + "/plot.html";
  else $("loglink").removeAttribute("href");
}

const es = new EventSource("/stream");
es.onmessage = e => {
  latest = JSON.parse(e.data);
  positionSample(latest);
  SIDES.forEach(side => {
    const s = sampleOf(side);
    if (s && s.error_ut !== null) side.hist.push({t: performance.now() / 1000, size: s.error_ut, v: s.error});
    showSample(side);
  });
};
es.addEventListener("status", e => {
  state = JSON.parse(e.data);
  showShared();
  positionState(state);
  SIDES.forEach(showSideState);
  // a sensor that's switched off gets no panel; one panel lays its view and charts side by side
  SIDES.forEach(side => { side.panel.style.display = side.key !== "icm" && !stateOf(side) ? "none" : ""; });
  $("sides").classList.toggle("single", SIDES.filter(side => side.panel.style.display !== "none").length === 1);
});
(function frame() {
  const now = performance.now() / 1000;
  SIDES.forEach(side => {
    while (side.hist.length && side.hist[0].t < now - HIST_S) side.hist.shift();
    drawView(side.part("view"), sampleOf(side));
    const s = stateOf(side);
    if (s && s.tumbling) {
      // the shared orientation turns both balls, even if one sensor has no reading yet
      drawSweep(side.part("sweep"), s, latest ? latest.quat : [1, 0, 0, 0], side.trail);
    }
  });
  drawCharts(HIST_S, state ? state.pass_ut : 5);
  drawPosition();
  requestAnimationFrame(frame);
})();
