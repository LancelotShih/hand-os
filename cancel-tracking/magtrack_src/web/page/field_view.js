// The coil's field, worked out on the page from the fitted coil, for the Position
// card.  Same model as position/locate.py: a bar magnet with a pole at each end,
//   B(p) = q (p - a) / |p - a|^3  -  q (p - b) / |p - b|^3,   a, b = center ± half_length · axis
// in µT, with p in cm from the measured coil center, in the reference axes.
// Drawn as arrows on a grid
// through the cube, or as field lines from the north pole round to the south.
const FIELD_GRID = 7;                  // arrows along each edge of the cube
const FIELD_LINE_COUNT = 26;           // field lines leaving the north pole
const FIELD_STEP_CM = 0.25;            // field line drawing step
const FIELD_LOW_UT = 5, FIELD_HIGH_UT = 500;   // arrows grow from shortest to longest over this, on a log scale
let fieldMode = "arrows";
let fieldCache = {key: null};

(function fieldControls() {
  const pick = $("posfield");
  pick.value = fieldMode;
  pick.onchange = () => { fieldMode = pick.value; };
})();

function coilPoles(coil) {
  const h = coil.half_length;
  return [coil.center.map((c, k) => c + coil.axis[k] * h), coil.center.map((c, k) => c - coil.axis[k] * h)];
}

function coilField(coil, p) {
  const [n, s] = coilPoles(coil), q = coil.strength, out = [0, 0, 0];
  for (const [pole, sign] of [[n, q], [s, -q]]) {
    const d = [p[0] - pole[0], p[1] - pole[1], p[2] - pole[2]], r = Math.hypot(...d);
    if (r < 1e-6) return null;
    for (let k = 0; k < 3; k++) out[k] += sign * d[k] / (r * r * r);
  }
  return out;
}

// Arrow length in cm for a field of b µT, up to `longest` cm.  Logarithmic: the field
// is thousands of µT beside the poles and about 10 µT in the cube's far corners.
function fieldLength(b, longest) {
  const t = Math.log(b / FIELD_LOW_UT) / Math.log(FIELD_HIGH_UT / FIELD_LOW_UT);
  return longest * Math.max(0.15, Math.min(1, t));
}

function nearCoil(coil, p, clear) {
  // distance from p to the line between the poles: the model means nothing in there
  const [n, s] = coilPoles(coil), ab = s.map((v, k) => v - n[k]), ap = p.map((v, k) => v - n[k]);
  const t = Math.max(0, Math.min(1, ap.reduce((a, v, k) => a + v * ab[k], 0) / ab.reduce((a, v) => a + v * v, 0)));
  return Math.hypot(...ap.map((v, k) => v - ab[k] * t)) < clear;
}

function buildField(coil, mid, range) {
  const step = 2 * range / (FIELD_GRID - 1), arrows = [];
  for (let i = 0; i < FIELD_GRID; i++) for (let j = 0; j < FIELD_GRID; j++) for (let k = 0; k < FIELD_GRID; k++) {
    const p = [mid[0] - range + i * step, mid[1] - range + j * step, mid[2] - range + k * step];
    if (nearCoil(coil, p, 2)) continue;
    const b = coilField(coil, p);
    if (!b) continue;
    const size = Math.hypot(...b), half = fieldLength(size, step * 0.8) / 2;
    const dir = b.map(v => v / size);
    arrows.push({from: p.map((v, k) => v - dir[k] * half), to: p.map((v, k) => v + dir[k] * half),
                 strength: Math.min(1, Math.max(0, Math.log(size / FIELD_LOW_UT) / Math.log(FIELD_HIGH_UT / FIELD_LOW_UT)))});
  }

  // field lines: start just off the north pole, spread evenly round it, follow the field
  const [north, south] = coilPoles(coil), lines = [];
  const along = p => { const b = coilField(coil, p); if (!b) return null; const s = Math.hypot(...b); return b.map(v => v / s); };
  const inside = p => p.every((v, k) => Math.abs(v - mid[k]) <= range);
  for (let i = 0; i < FIELD_LINE_COUNT; i++) {
    const z = 1 - 2 * (i + 0.5) / FIELD_LINE_COUNT, r = Math.sqrt(1 - z * z), a = i * 2.39996;
    let p = [north[0] + 0.4 * r * Math.cos(a), north[1] + 0.4 * r * Math.sin(a), north[2] + 0.4 * z];
    const line = [p];
    for (let n = 0; n < 800 && inside(p); n++) {
      const d1 = along(p);
      if (!d1) break;
      const d2 = along(p.map((v, k) => v + d1[k] * FIELD_STEP_CM / 2));
      if (!d2) break;
      p = p.map((v, k) => v + d2[k] * FIELD_STEP_CM);
      line.push(p);
      if (Math.hypot(p[0] - south[0], p[1] - south[1], p[2] - south[2]) < 0.4) break;
    }
    if (line.length > 4) lines.push(line);
  }
  return {arrows, lines, poles: [north, south]};
}

function fieldFor(coil, mid, range) {
  const key = JSON.stringify([coil.center, coil.axis, coil.strength, coil.half_length, range]);
  if (fieldCache.key !== key) fieldCache = {key, ...buildField(coil, mid, range)};
  return fieldCache;
}

// A line with a head at `to`, drawn in screen space.
function fieldArrow(g, a, b, color, width, head) {
  const t = Math.atan2(b[1] - a[1], b[0] - a[0]), len = Math.hypot(b[0] - a[0], b[1] - a[1]);
  const hd = Math.min(head, len * 0.45);
  g.strokeStyle = color; g.fillStyle = color; g.lineWidth = width;
  g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
  g.beginPath(); g.moveTo(b[0], b[1]);
  g.lineTo(b[0] - hd * Math.cos(t - .45), b[1] - hd * Math.sin(t - .45));
  g.lineTo(b[0] - hd * Math.cos(t + .45), b[1] - hd * Math.sin(t + .45));
  g.closePath(); g.fill();
}

function drawField(g, px, coil, mid, range) {
  if (fieldMode === "off" || !coil.strength) return;
  const f = fieldFor(coil, mid, range), color = css("--field");
  if (fieldMode === "arrows") {
    for (const a of f.arrows) {
      g.globalAlpha = 0.25 + 0.6 * a.strength;
      fieldArrow(g, px(a.from), px(a.to), color, 1.2, 6);
    }
  } else {
    g.globalAlpha = 0.55;
    for (const line of f.lines) {
      const pts = line.map(px);
      g.strokeStyle = color; g.lineWidth = 1.2;
      g.beginPath(); pts.forEach((q, i) => i ? g.lineTo(q[0], q[1]) : g.moveTo(q[0], q[1])); g.stroke();
      const m = Math.floor(pts.length / 2);      // which way the field runs along this line
      fieldArrow(g, pts[m - 2], pts[m], color, 1.2, 7);
    }
  }
  g.globalAlpha = 1;
  f.poles.forEach(p => {                         // where the model puts the poles
    const q = px(p);
    g.fillStyle = color; g.beginPath(); g.arc(q[0], q[1], 3, 0, 7); g.fill();
  });
}

// The coil's field the ICM measures right now, in µT, in the home axes: the
// error is that field minus the coil's field at home.  Null before a coil fit.
function measuredCoilField(sample, coil) {
  if (!sample || !sample.error || !coil || !coil.home_field) return null;
  return sample.error.map((v, k) => v + coil.home_field[k]);
}
