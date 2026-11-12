// The tumble as a ball that turns with the board.  Whatever part of the ball the
// field passes through gets painted, so turning grey patches to the top paints them.
const BALL_PIXELS = 260;              // drawn at this size, then scaled to fit
const BALL_EDGE = 0.38;               // same direction boundaries as tumble.py
const TRAIL_POINTS = 500;
const ballCamera = {yaw: -35, pitch: 35};
let ballDragFrom = null;

const ballCanvas = document.createElement("canvas");
ballCanvas.width = ballCanvas.height = BALL_PIXELS;
const ballContext = ballCanvas.getContext("2d");
const ballImage = ballContext.createImageData(BALL_PIXELS, BALL_PIXELS);

SIDES.forEach(side => {
  const sweep = side.part("sweep");
  sweep.addEventListener("pointerdown", e => {
    ballDragFrom = [e.clientX, e.clientY];
    sweep.setPointerCapture(e.pointerId);
  });
  sweep.addEventListener("pointermove", e => {
    if (!ballDragFrom) return;
    ballCamera.yaw += (e.clientX - ballDragFrom[0]) * 0.5;
    ballCamera.pitch = Math.max(-89, Math.min(89, ballCamera.pitch + (e.clientY - ballDragFrom[1]) * 0.5));
    ballDragFrom = [e.clientX, e.clientY];
  });
  sweep.addEventListener("pointerup", () => { ballDragFrom = null; });
  sweep.addEventListener("pointercancel", () => { ballDragFrom = null; });
  sweep.addEventListener("dblclick", () => { ballCamera.yaw = -35; ballCamera.pitch = 35; });
});

function ballProject(p) {
  const a = ballCamera.yaw * Math.PI / 180, e = ballCamera.pitch * Math.PI / 180;
  const x = p[0]*Math.cos(a) - p[1]*Math.sin(a), y = p[0]*Math.sin(a) + p[1]*Math.cos(a);
  return [x, y*Math.sin(e) + p[2]*Math.cos(e), y*Math.cos(e) - p[2]*Math.sin(e)];
}

function hexColor(hex) {
  return [1, 3, 5].map(i => parseInt(hex.slice(i, i + 2), 16));
}

// Colours the ball one pixel at a time.  The top of the ball is the field's far
// end, so each pixel is scored by the opposite direction.
function paintBall(q, swept) {
  const done = hexColor(css("--good")), todo = hexColor(css("--patch"));
  const cells = new Uint8Array(27);
  for (const c of swept) cells[(c[0] + 1) * 9 + (c[1] + 1) * 3 + (c[2] + 1)] = 1;
  const a = ballCamera.yaw * Math.PI / 180, e = ballCamera.pitch * Math.PI / 180;
  const ca = Math.cos(a), sa = Math.sin(a), ce = Math.cos(e), se = Math.sin(e);
  const back = [q[0], -q[1], -q[2], -q[3]];
  const data = ballImage.data, half = BALL_PIXELS / 2;
  let i = 0;
  for (let row = 0; row < BALL_PIXELS; row++) {
    for (let col = 0; col < BALL_PIXELS; col++, i += 4) {
      const u = (col + 0.5 - half) / half, v = (half - row - 0.5) / half, rr = u * u + v * v;
      if (rr >= 1) { data[i + 3] = 0; continue; }
      const d = -Math.sqrt(1 - rr);
      const yc = v * se + d * ce, zw = v * ce - d * se;
      const p = rot(back, [u * ca + yc * sa, -u * sa + yc * ca, zw]);
      const cell = (-p[0] > BALL_EDGE ? 2 : -p[0] < -BALL_EDGE ? 0 : 1) * 9
                 + (-p[1] > BALL_EDGE ? 2 : -p[1] < -BALL_EDGE ? 0 : 1) * 3
                 + (-p[2] > BALL_EDGE ? 2 : -p[2] < -BALL_EDGE ? 0 : 1);
      const color = cells[cell] ? done : todo;
      const light = 0.62 + 0.38 * Math.max(0, 0.35 * v - 0.2 * u - 0.91 * d);
      data[i] = color[0] * light;
      data[i + 1] = color[1] * light;
      data[i + 2] = color[2] * light;
      data[i + 3] = 255;
    }
  }
  ballContext.putImageData(ballImage, 0, 0);
}

// ``trail`` is in the sensor's axes, so it turns with the ball.
function drawSweep(canvas, s, q, ballTrail) {
  const [g, w, h] = fit(canvas);
  const R = Math.min(w, h) / 2 - 34, cx = w / 2, cy = h / 2;
  const at = p => { const c = ballProject(rot(q, p)); return [cx + c[0]*R, cy - c[1]*R, c[2]]; };

  // only the ball's front shows, so turning the board is how you see the rest
  paintBall(q, s.tumble_swept);
  g.drawImage(ballCanvas, cx - R, cy - R, 2 * R, 2 * R);

  // the path swept so far
  if (s.tumble_now) {
    const here = s.tumble_now.map(c => -c);
    const last = ballTrail[ballTrail.length - 1];
    if (!last || here.some((c, i) => c !== last[i])) ballTrail.push(here);
    if (ballTrail.length > TRAIL_POINTS) ballTrail.shift();
  }
  g.strokeStyle = css("--ink");
  g.lineWidth = 1.5;
  g.lineCap = "round";
  ballTrail.slice(1).forEach((p, i) => {
    const a = at(ballTrail[i]), b = at(p);
    if (a[2] > 0 || b[2] > 0) return;
    g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
  });

  // the board, in the middle of the ball, turning as it does
  const corners = [[-.5, -.3, 0], [.5, -.3, 0], [.5, .3, 0], [-.5, .3, 0]].map(at);
  g.fillStyle = css("--board");
  g.globalAlpha = 0.9;
  g.beginPath();
  corners.forEach((c, i) => i ? g.lineTo(c[0], c[1]) : g.moveTo(c[0], c[1]));
  g.closePath(); g.fill();
  g.globalAlpha = 1;
  const nose = at([.5, 0, 0]);
  g.fillStyle = css("--ink");
  g.beginPath(); g.arc(nose[0], nose[1], 4, 0, 7); g.fill();

  // where the paint lands, with an arrow from above
  if (s.tumble_now) {
    const c = at(s.tumble_now.map(v => -v));
    const out = [cx + (c[0]-cx)*1.32, cy + (c[1]-cy)*1.32];
    g.strokeStyle = css("--ink");
    g.lineWidth = 3;
    g.beginPath(); g.arc(c[0], c[1], 9, 0, 7); g.stroke();
    if (Math.hypot(c[0]-cx, c[1]-cy) > R * 0.25) {
      g.lineWidth = 2;
      g.beginPath(); g.moveTo(out[0], out[1]); g.lineTo(cx + (c[0]-cx)*1.08, cy + (c[1]-cy)*1.08); g.stroke();
    }
  }
}
