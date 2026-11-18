// Position, absolute coil field, matching choices and fitted poles in saved map axes.
const POS_VIEW_START = {yaw: -35, pitch: 25, zoom: 1};
const TRAIL_S = 4;
const COIL_LENGTH_CM = 5, COIL_WIDTH_CM = 4, COIL_HEIGHT_CM = 2;
const COIL_DRAWING_SCALE = 0.5;
const BOARD_HALF_SIZE_CM = 0.6, BOARD_HALF_THICKNESS_CM = 0.08;
const POSITION_GRID_SPACING_CM = 1, POSITION_GRID_LABEL_SPACING_CM = 5;
const posCamera = {...POS_VIEW_START};
const posTrail = [];
let posLatest = null, posState = null, posDrag = null;
let posOrientationQuat = null, positionGridVisible = false;

(function controls() {
  const view = $("posview");
  view.addEventListener("pointerdown", e => { posDrag = [e.clientX, e.clientY]; view.setPointerCapture(e.pointerId); });
  view.addEventListener("pointermove", e => {
    if (!posDrag) return;
    posCamera.yaw += (e.clientX - posDrag[0]) * 0.5;
    posCamera.pitch = Math.max(-89, Math.min(89, posCamera.pitch + (e.clientY - posDrag[1]) * 0.5));
    posDrag = [e.clientX, e.clientY];
  });
  view.addEventListener("pointerup", () => { posDrag = null; });
  view.addEventListener("pointercancel", () => { posDrag = null; });
  view.addEventListener("wheel", e => {
    e.preventDefault();
    posCamera.zoom = Math.max(0.4, Math.min(6, posCamera.zoom * Math.exp(-e.deltaY * 0.001)));
  }, {passive: false});
  const reset = () => Object.assign(posCamera, POS_VIEW_START);
  view.addEventListener("dblclick", reset);
  $("posreset").onclick = reset;
  const grid = $("posgridtoggle");
  grid.onchange = () => { positionGridVisible = grid.checked; };

  const send = (path, body) => fetch(path, {method: "POST", headers: {"Content-Type": "application/json"},
                                            body: JSON.stringify(body || {})})
    .then(r => r.json()).then(r => { if (!r.ok) $("calstatus").textContent = r.error; });
  const xyz = (ids, path) => {
    const v = ids.map(id => parseFloat($(id).value));
    if (v.some(isNaN)) { $("calstatus").textContent = "Enter all three sensor coordinates, x, y and z, in centimeters from the coil center."; return; }
    send(path, {x: v[0], y: v[1], z: v[2]});
  };
  $("calcapture").onclick = () => xyz(["calx", "caly", "calz"], "/locate/point");
  $("calcancel").onclick = () => send("/locate/cancel");
  $("calclear").onclick = () => {
    if (confirm("Delete all recorded coil-calibration measurements and the fitted coil model? Sensor corrections and saved map axes will be kept. Position estimates require a new coil model.")) send("/locate/clear");
  };
  $("calrows").addEventListener("click", e => {
    const i = e.target.dataset && e.target.dataset.remove;
    if (i !== undefined) send("/locate/remove", {index: Number(i)});
  });
})();

function showPositionChoices(candidates) {
  const box = $("positionchoices");
  while (box.children.length > candidates.length) box.lastChild.remove();
  candidates.forEach((p, i) => {
    let button = box.children[i];
    if (!button) {
      button = document.createElement("button"); box.appendChild(button);
      button.onclick = () => {
        const q = JSON.parse(button.dataset.position);
        post("/locate/choose", {x: q[0], y: q[1], z: q[2], reference_id: button.dataset.reference});
      };
    }
    button.textContent = "Choose position " + (i + 1) + ": " + displayVector(p).map(v => v.toFixed(1)).join(", ") + " cm";
    button.dataset.position = JSON.stringify(p);
    button.dataset.reference = posState && posState.reference_id;
  });
}

function positionSample(sample) {
  posLatest = sample;
  const loc = sample && sample.locate, now = performance.now() / 1000;
  const badge = $("positionambiguity");
  const ambiguityText = $("positionambiguitytext");
  const possible = loc && (loc.alternatives || loc.candidates) || [];
  badge.hidden = !(loc && loc.ambiguous);
  ambiguityText.textContent = loc && loc.ambiguous
    ? loc.position
      ? "AMBIGUOUS — prior position used (" + (possible.length + 1) + " candidates)"
      : "AMBIGUOUS — choose among " + possible.length + " candidates"
    : "";
  badge.title = (loc && (loc.warning || loc.why) || "")
    + " Red question marks show other sampled matching positions; the set may not be exhaustive.";
  if (loc && loc.position) posTrail.push({t: now, p: loc.position});
  while (posTrail.length && posTrail[0].t < now - TRAIL_S) posTrail.shift();
  const b = measuredCoilField(sample);
  $("poscoilfield").textContent = b ? Math.hypot(...b).toFixed(1) + " uT" : "-";
  $("historywarning").hidden = !(loc && loc.history_based);
  $("historywarning").textContent = loc && loc.warning || "";
  if (!loc || !loc.position) {
    $("posxyz").textContent = loc ? loc.why : "Waiting for a coil-off ambient measurement.";
    ["posfromcoil", "posprec"].forEach(id => { $(id).textContent = "-"; });
    $("posprec").className = "";
    return;
  }
  $("posxyz").textContent = displayVector(loc.position).map((v, k) => "xyz"[k] + " " + v.toFixed(1)).join("  ") + " cm (displayed axes)";
  $("posfromcoil").textContent = loc.from_coil_cm.toFixed(1) + " cm";
  $("posprec").textContent = loc.mm_per_ut >= 999 ? "999 mm per uT or more" : loc.mm_per_ut.toFixed(1) + " mm per uT";
  $("posprec").className = loc.mm_per_ut > 4 ? "bad" : "";
}

function positionState(state) {
  const s = state.locate;
  posOrientationQuat = state.orientation_quat;
  if (!s || !state.coil_on || !posState || posState.frame_id !== s.frame_id || posState.reference_id !== s.reference_id) {
    posLatest = null; posTrail.length = 0; $("historywarning").hidden = true;
  }
  posState = s;
  $("poscard").style.display = s ? "block" : "none";
  if (!s) return;
  const n = s.points.length;
  $("calcapture").disabled = s.capturing || !state.ambient_ready || !state.coil_on || s.legacy_calibration;
  $("calcancel").style.display = s.capturing ? "" : "none";
  $("calclear").disabled = (!n && !s.legacy_calibration) || s.capturing;
  $("calstatus").textContent = s.message || (s.capturing ? "Recording the field. Hold still: " + Math.round(s.capture_done * 100) + "%"
    : s.fitting ? "Fitting the coil model and building the field lookup table..."
    : n < s.min_points ? n + " measurements recorded; at least " + s.min_points + " are needed. Use different positions and heights." : "");
  const c = s.coil;
  $("coilsummary").textContent = c ? "Fitted coil center is " + c.center_off_cm.toFixed(1) + " cm from the entered origin. "
    + s.lookup_points + " model-generated lookup entries. North is red; south is blue."
    : "No fitted coil model is ready. Record fields at measured positions after the coil-off startup step.";
  showPositionChoices(state.coil_on ? s.candidates || [] : []);
  const key = JSON.stringify(s.points), rows = $("calrows");
  if (rows.dataset.key === key) return;
  rows.dataset.key = key; rows.textContent = "";
  s.points.forEach((pt, i) => {
    const row = document.createElement("tr"), mm = v => v === null || v === undefined ? "-" : v.toFixed(1);
    [i + 1, ...pt.position.map(v => v.toFixed(1)), mm(pt.off_mm), mm(pt.left_out_mm)].forEach(text => {
      const cell = document.createElement("td"); cell.textContent = text; row.appendChild(cell);
    });
    const cell = document.createElement("td"), del = document.createElement("button");
    del.textContent = "Remove"; del.className = "remove"; del.dataset.remove = i;
    del.setAttribute("aria-label", "Remove coil-calibration measurement " + (i + 1));
    cell.appendChild(del); row.appendChild(cell); rows.appendChild(row);
  });
}

function drawPosition() {
  const canvas = $("posview");
  if (!posState || canvas.offsetParent === null) return;
  const [g, w, h] = fit(canvas);
  const coil = posState.coil, range = posState.range_cm;
  const mid = [0, 0, 0];                               // the coil's center
  const u = Math.min(w, h) / (range * 3.4) * posCamera.zoom;
  const px = p => {
    const c = cam(displayVector(p), posCamera);
    return [w / 2 + c[0] * u, h / 2 - c[1] * u, c[2]];
  };
  const line = (a, b, col, wd, dash) => {
    g.strokeStyle = col; g.lineWidth = wd; g.setLineDash(dash || []);
    g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke(); g.setLineDash([]);
  };
  const label = (p, text, col) => { g.fillStyle = col; g.font = "12px system-ui"; g.fillText(text, p[0] + 5, p[1] - 5); };

  // the working cube around the coil
  const r = range, corners = [];
  for (const x of [-r, r]) for (const y of [-r, r]) for (const z of [-r, r]) corners.push([mid[0] + x, mid[1] + y, mid[2] + z]);
  [[0,1],[2,3],[4,5],[6,7],[0,2],[1,3],[4,6],[5,7],[0,4],[1,5],[2,6],[3,7]]
    .forEach(([a, b]) => line(px(corners[a]), px(corners[b]), css("--line"), 1));

  if (positionGridVisible) drawPositionGrid(g, px, r, line, label);

  if (coil) {
    drawField(g, px, coil, mid, range);
    drawCoil(g, px, coil);
  }

  // 0, the coil's center, and the axes
  const zero = px(mid);
  displayBasis().forEach((axis, k) => {
    const e = px(axis.map(v => v * 6));
    line(zero, e, css("--dim"), 1.5); label(e, "xyz"[k], css("--dim"));
  });
  g.fillStyle = css("--dim"); g.beginPath(); g.arc(zero[0], zero[1], 3, 0, 7); g.fill();
  label(zero, "coil center (0)", css("--dim"));

  posState.points.forEach((pt, i) => {
    const q = px(pt.position);
    g.strokeStyle = css("--pred"); g.lineWidth = 1.5;
    g.beginPath(); g.arc(q[0], q[1], 4, 0, 7); g.stroke();
    label(q, String(i + 1), css("--pred"));
  });

  for (let i = 1; i < posTrail.length; i++) {
    g.globalAlpha = i / posTrail.length;
    line(px(posTrail[i - 1].p), px(posTrail[i].p), css("--trace"), 2);
  }
  g.globalAlpha = 1;

  const loc = posLatest && posLatest.locate;
  if (loc && loc.ambiguous) {
    drawAlternativePositions(g, px, loc.position ? loc.alternatives : loc.candidates);
  }
  if (loc && loc.position) {
    const p = loc.position;
    line(px(p), px([p[0], p[1], 0]), css("--dim"), 1, [3, 3]);    // drop to the displayed map plane
    drawBoard(g, px, p, posOrientationQuat || posLatest.quat);
    const b = measuredCoilField(posLatest, coil);    // what the ICM reads from the coil
    if (b) {
      const size = Math.hypot(...b), len = fieldLength(size, 6);
      const tip = px(p.map((v, k) => v + b[k] / size * len));
      fieldArrow(g, px(p), tip, css("--meas"), 3, 10);
      label(tip, size.toFixed(0) + " µT", css("--meas"));
    }
  }
}

function drawAlternativePositions(g, px, positions) {
  for (const p of positions || []) {
    const point = px(p);
    g.globalAlpha = 0.65;
    g.strokeStyle = css("--bad"); g.lineWidth = 1.5; g.setLineDash([3, 3]);
    g.beginPath(); g.arc(point[0], point[1], 7, 0, 7); g.stroke();
    g.setLineDash([]);
    g.fillStyle = css("--bad"); g.font = "bold 10px system-ui"; g.fillText("?", point[0] - 3, point[1] + 4);
  }
  g.globalAlpha = 1;
}

function drawPositionGrid(g, px, range, line, label) {
  const limit = Math.floor(range / POSITION_GRID_SPACING_CM) * POSITION_GRID_SPACING_CM;
  for (let v = -limit; v <= limit; v += POSITION_GRID_SPACING_CM) {
    const major = v % POSITION_GRID_LABEL_SPACING_CM === 0;
    g.globalAlpha = v === 0 ? 0.9 : major ? 0.75 : 0.55;
    const color = major ? css("--dim") : css("--line");
    const width = v === 0 ? 1.5 : major ? 1.2 : 0.8;
    line(px([v, -limit, 0]), px([v, limit, 0]), color, width);
    line(px([-limit, v, 0]), px([limit, v, 0]), color, width);
  }
  g.globalAlpha = 0.8;
  for (let v = -limit; v <= limit; v += POSITION_GRID_LABEL_SPACING_CM) {
    label(px([v, -limit, 0]), String(v), css("--dim"));
    if (v !== -limit) label(px([-limit, v, 0]), String(v), css("--dim"));
  }
  label(px([limit, 0, 0]), "x (cm)", css("--dim"));
  label(px([0, limit, 0]), "y (cm)", css("--dim"));
  g.globalAlpha = 1;
}

function drawBox(g, px, center, axes, half, color) {
  const v = [];
  for (const a of [-1, 1]) for (const b of [-1, 1]) for (const c of [-1, 1]) {
    v.push(px([0, 1, 2].map(k => center[k] + a*half[0]*axes[0][k] + b*half[1]*axes[1][k] + c*half[2]*axes[2][k])));
  }
  const faces = [[0,1,3,2,.75],[4,5,7,6,.75],[0,1,5,4,.6],[2,3,7,6,.6],[0,2,6,4,.9],[1,3,7,5,.9]]
    .map(f => ({f, z: f.slice(0, 4).reduce((s, i) => s + v[i][2], 0) / 4}))
    .sort((a, b) => b.z - a.z);
  g.strokeStyle = css("--card"); g.lineWidth = 1;
  for (const {f} of faces) {
    g.globalAlpha = f[4]; g.fillStyle = color;
    g.beginPath(); f.slice(0, 4).forEach((i, k) => k ? g.lineTo(v[i][0], v[i][1]) : g.moveTo(v[i][0], v[i][1]));
    g.closePath(); g.fill(); g.globalAlpha = 1; g.stroke();
  }
  return v;
}

function drawCoil(g, px, coil) {
  const a = coil.axis;
  let side = [a[1], -a[0], 0];
  if (Math.hypot(...side) < 0.1) side = [1, 0, 0];
  side = side.map(v => v / Math.hypot(...side));
  const up = [a[1]*side[2]-a[2]*side[1], a[2]*side[0]-a[0]*side[2], a[0]*side[1]-a[1]*side[0]];
  const ends = [[1, "North", "#dc5454", -12], [-1, "South", "#4a88e4", 18]];
  const halfLength = COIL_LENGTH_CM * COIL_DRAWING_SCALE / 2;
  ends.forEach(([sign, name, color, labelOffset]) => {
    const center = a.map(v => sign * v * halfLength / 2);
    drawBox(g, px, center, [a, side, up], [
      halfLength / 2,
      COIL_WIDTH_CM * COIL_DRAWING_SCALE / 2,
      COIL_HEIGHT_CM * COIL_DRAWING_SCALE / 2
    ], color);
    const tip = px(a.map(v => sign * v * halfLength));
    g.fillStyle = color; g.beginPath(); g.arc(tip[0], tip[1], 2, 0, 7); g.fill();
    g.font = "bold 11px system-ui"; g.fillText(name, tip[0] + 5, tip[1] + labelOffset / 2);
  });
}

function drawBoard(g, px, p, q) {
  const axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]].map(e => rot(q || [1, 0, 0, 0], e));
  drawBox(g, px, p, axes, [BOARD_HALF_SIZE_CM, BOARD_HALF_SIZE_CM, BOARD_HALF_THICKNESS_CM], css("--board"));
  const nose = px([0, 1, 2].map(k => p[k] + axes[0][k] * BOARD_HALF_SIZE_CM + axes[2][k] * BOARD_HALF_THICKNESS_CM));
  g.fillStyle = css("--ink"); g.beginPath(); g.arc(nose[0], nose[1], 3, 0, 7); g.fill();
}
