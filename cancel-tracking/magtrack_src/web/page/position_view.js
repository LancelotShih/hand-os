// The Position card: where the ICM is, drawn in 3-D around the coil, plus the
// calibration points.  Positions are cm from home, in the ICM's own axes as it
// sat at home.  Drag to orbit, scroll to zoom, double-click to reset.
const POS_VIEW_START = {yaw: -35, pitch: 25, zoom: 1};
const TRAIL_S = 4;
const posCamera = {...POS_VIEW_START};
const posTrail = [];
let posLatest = null, posState = null, posDrag = null;

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

  const send = (path, body) => fetch(path, {method: "POST", headers: {"Content-Type": "application/json"},
                                            body: JSON.stringify(body || {})})
    .then(r => r.json()).then(r => { if (!r.ok) $("calstatus").textContent = r.error; });
  $("calcapture").onclick = () => {
    const v = ["calx", "caly", "calz"].map(id => parseFloat($(id).value));
    if (v.some(isNaN)) { $("calstatus").textContent = "Enter x, y and z in cm first."; return; }
    send("/locate/point", {x: v[0], y: v[1], z: v[2]});
  };
  $("calcancel").onclick = () => send("/locate/cancel");
  $("calclear").onclick = () => {
    if (confirm("Delete every calibration point? Positions stop until you capture new ones.")) send("/locate/clear");
  };
  $("calrows").addEventListener("click", e => {
    const i = e.target.dataset && e.target.dataset.remove;
    if (i !== undefined) send("/locate/remove", {index: Number(i)});
  });
})();

function positionSample(sample) {
  posLatest = sample;
  const loc = sample && sample.locate, now = performance.now() / 1000;
  if (loc && loc.position) posTrail.push({t: now, p: loc.position});
  while (posTrail.length && posTrail[0].t < now - TRAIL_S) posTrail.shift();
  const fmt = v => (v >= 0 ? "+" : "−") + Math.abs(v).toFixed(1);
  if (!loc || !loc.position) {
    $("posxyz").textContent = loc ? loc.why : "–";
    ["posfromhome", "posfromcoil", "posprec"].forEach(id => { $(id).textContent = "–"; });
    $("posprec").className = "";
    return;
  }
  const p = loc.position;
  $("posxyz").textContent = "x " + fmt(p[0]) + "  y " + fmt(p[1]) + "  z " + fmt(p[2]) + " cm";
  $("posfromhome").textContent = loc.from_home_cm.toFixed(1) + " cm";
  $("posfromcoil").textContent = loc.from_coil_cm.toFixed(1) + " cm";
  $("posprec").textContent = loc.mm_per_ut >= 999 ? "none here" : loc.mm_per_ut.toFixed(1) + " mm per µT";
  $("posprec").className = loc.mm_per_ut <= 1.5 ? "good" : loc.mm_per_ut > 4 ? "bad" : "";
}

function positionState(state) {
  const s = state.locate;
  posState = s;
  $("poscard").style.display = s ? "block" : "none";
  if (!s) return;
  const n = s.points.length;
  $("calcapture").disabled = s.capturing;
  $("calcancel").style.display = s.capturing ? "" : "none";
  $("calclear").disabled = !n || s.capturing;
  $("calstatus").textContent = s.message
    || (s.capturing ? "Hold still… " + Math.round(s.capture_done * 100) + "%"
      : s.fitting ? "Fitting the coil…"
      : n < s.min_points ? n + " of " + s.min_points + " points. Spread them around the space, some raised."
      : "");
  const c = s.coil;
  $("coilsummary").textContent = !c ? "Coil not found yet."
    : "Coil center " + Math.hypot(...c.center).toFixed(1) + " cm from home. Field at home "
      + c.home_field_ut.toFixed(0) + " µT. Precision at home " + (c.home_mm_per_ut >= 999 ? "none"
      : c.home_mm_per_ut.toFixed(1) + " mm per µT") + "."
      + (c.home_mm_per_ut > 4 ? " Home is beside the coil's middle, where positions are weak: turn the coil so one end points at home." : "");
  const key = JSON.stringify(s.points);
  const rows = $("calrows");
  if (rows.dataset.key === key) return;
  rows.dataset.key = key;
  rows.textContent = "";
  s.points.forEach((pt, i) => {
    const row = document.createElement("tr");
    const mm = v => v === null || v === undefined ? "–" : v.toFixed(1);
    [i + 1, ...pt.position.map(v => v.toFixed(1)), mm(pt.off_mm), mm(pt.left_out_mm)].forEach(text => {
      const cell = document.createElement("td");
      cell.textContent = text;
      row.appendChild(cell);
    });
    const cell = document.createElement("td"), del = document.createElement("button");
    del.textContent = "×";
    del.className = "remove";
    del.dataset.remove = i;
    del.setAttribute("aria-label", "Remove point " + (i + 1));
    cell.appendChild(del);
    row.appendChild(cell);
    rows.appendChild(row);
  });
}

function drawPosition() {
  const canvas = $("posview");
  if (!posState || canvas.offsetParent === null) return;
  const [g, w, h] = fit(canvas);
  const coil = posState.coil, range = posState.range_cm;
  const mid = coil ? coil.center : [0, 0, 0];
  const u = Math.min(w, h) / (range * 3.4) * posCamera.zoom;
  const px = p => {
    const c = cam([p[0] - mid[0], p[1] - mid[1], p[2] - mid[2]], posCamera);
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

  // home and its axes
  const home = px([0, 0, 0]);
  [[4,0,0,"x"],[0,4,0,"y"],[0,0,4,"z"]].forEach(([x, y, z, n]) => {
    const e = px([x, y, z]);
    line(home, e, css("--dim"), 1.5);
    label(e, n, css("--dim"));
  });
  g.fillStyle = css("--dim"); g.beginPath(); g.arc(home[0], home[1], 3, 0, 7); g.fill();
  label(home, "home", css("--dim"));

  if (coil) drawCoil(g, px, coil);

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
  if (loc && loc.position) {
    const p = loc.position;
    line(px(p), px([p[0], p[1], 0]), css("--dim"), 1, [3, 3]);    // drop to home's level
    drawBoard(g, px, p, posLatest.quat);
  }
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
  // two cores side by side: 5 cm along the axis, 4 cm across, 2 cm tall
  const a = coil.axis;
  let side = [a[1], -a[0], 0];
  if (Math.hypot(...side) < 0.1) side = [1, 0, 0];
  const n = Math.hypot(...side);
  side = side.map(v => v / n);
  const up = [a[1]*side[2] - a[2]*side[1], a[2]*side[0] - a[0]*side[2], a[0]*side[1] - a[1]*side[0]];
  drawBox(g, px, coil.center, [a, side, up], [2.5, 2, 1], css("--coil"));
  const tip = px(coil.center.map((c, k) => c + a[k] * 3.2));
  g.fillStyle = css("--ink"); g.font = "12px system-ui"; g.fillText("N", tip[0] - 4, tip[1] + 4);
}

function drawBoard(g, px, p, q) {
  const axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]].map(e => rot(q || [1, 0, 0, 0], e));
  drawBox(g, px, p, axes, [1.6, 1.1, 0.15], css("--board"));
  const nose = px([0, 1, 2].map(k => p[k] + axes[0][k] * 1.6 + axes[2][k] * 0.15));
  g.fillStyle = css("--ink"); g.beginPath(); g.arc(nose[0], nose[1], 3, 0, 7); g.fill();
}
