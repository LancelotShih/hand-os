// The 3-D board and the two field arrows.  Drag to orbit, scroll to zoom,
// double-click to reset.
const VIEW_START = {yaw: -35, pitch: 25, zoom: 1, tips: false};
const ARROW_PER_UT = 2.1 / 60;        // scene units per uT
const TIPS_ZOOM = 6;                  // extra zoom while looking at the arrow tips
const SCALE_BARS = [50, 20, 10, 5, 2, 1];
const camera = {...VIEW_START};
let dragFrom = null;

function setTips(on) {
  camera.tips = on;
  $("tips").textContent = on ? "Show whole view" : "Zoom to tips";
}

function resetView() {
  Object.assign(camera, VIEW_START);
  setTips(false);
}

// one camera: turning either view turns both, so they always show the same angle
SIDES.forEach(side => {
  const view = side.part("view");
  view.addEventListener("pointerdown", e => {
    dragFrom = [e.clientX, e.clientY];
    view.setPointerCapture(e.pointerId);
  });
  view.addEventListener("pointermove", e => {
    if (!dragFrom) return;
    camera.yaw += (e.clientX - dragFrom[0]) * 0.5;
    camera.pitch = Math.max(-89, Math.min(89, camera.pitch + (e.clientY - dragFrom[1]) * 0.5));
    dragFrom = [e.clientX, e.clientY];
  });
  view.addEventListener("pointerup", () => { dragFrom = null; });
  view.addEventListener("pointercancel", () => { dragFrom = null; });
  view.addEventListener("wheel", e => {
    e.preventDefault();
    camera.zoom = Math.max(0.5, Math.min(8, camera.zoom * Math.exp(-e.deltaY * 0.001)));
  }, {passive: false});
  view.addEventListener("dblclick", resetView);
});
$("resetview").onclick = resetView;
$("tips").onclick = () => setTips(!camera.tips);

function rot(q, v) {
  const [w, x, y, z] = q;
  const t = [2*(y*v[2]-z*v[1]), 2*(z*v[0]-x*v[2]), 2*(x*v[1]-y*v[0])];
  return [v[0]+w*t[0]+y*t[2]-z*t[1], v[1]+w*t[1]+z*t[0]-x*t[2], v[2]+w*t[2]+x*t[1]-y*t[0]];
}

function cam(p, c = camera) {
  const a = c.yaw * Math.PI / 180, e = c.pitch * Math.PI / 180;
  const x = p[0]*Math.cos(a) - p[1]*Math.sin(a), y = p[0]*Math.sin(a) + p[1]*Math.cos(a);
  return [x, y*Math.sin(e) + p[2]*Math.cos(e), y*Math.cos(e) - p[2]*Math.sin(e)];
}

function drawView(canvas, sample) {
  const [g, w, h] = fit(canvas);
  const toScene = vec => vec.map(c => c * ARROW_PER_UT);
  const focus = camera.tips && sample && sample.predicted ? cam(toScene(sample.predicted)) : [0, 0];
  const u = Math.min(w, h) / 5.2 * camera.zoom * (focus[0] || focus[1] ? TIPS_ZOOM : 1);
  const cx = w/2, cy = h/2;
  const px = p => { const c = cam(p); return [cx + (c[0]-focus[0])*u, cy - (c[1]-focus[1])*u, c[2]]; };
  const line = (a, b, col, wd) => {
    g.strokeStyle = col; g.lineWidth = wd;
    g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
  };
  const dim = css("--line");
  [[1,0,0,"x"],[0,1,0,"y"],[0,0,1,"z"]].forEach(([x,y,z,n]) => {
    const e = px([x*2, y*2, z*2]);
    line(px([0,0,0]), e, dim, 1);
    g.fillStyle = css("--dim"); g.font = "12px system-ui"; g.fillText(n, e[0]+4, e[1]+4);
  });

  const q = sample ? sample.quat : [1,0,0,0];
  const hx = 1.3, hy = .8, hz = .08;
  const v = [[-hx,-hy,-hz],[hx,-hy,-hz],[hx,hy,-hz],[-hx,hy,-hz],
             [-hx,-hy,hz],[hx,-hy,hz],[hx,hy,hz],[-hx,hy,hz]].map(p => px(rot(q, p)));
  const faces = [[4,5,6,7,1],[0,1,2,3,.45],[0,1,5,4,.7],[2,3,7,6,.7],[1,2,6,5,.6],[0,3,7,4,.6]]
    .map(f => ({f, z: f.slice(0,4).reduce((s,i) => s + v[i][2], 0) / 4}))
    .sort((a, b) => b.z - a.z);
  g.strokeStyle = css("--card"); g.lineWidth = 1;
  for (const {f} of faces) {
    g.globalAlpha = f[4];
    g.fillStyle = css("--board");
    g.beginPath(); f.slice(0,4).forEach((i, k) => k ? g.lineTo(v[i][0], v[i][1]) : g.moveTo(v[i][0], v[i][1]));
    g.closePath(); g.fill(); g.globalAlpha = 1; g.stroke();
  }
  // a dot on the board's +x edge, so the turn reads even when the board is symmetric
  const nose = px(rot(q, [hx, 0, hz]));
  g.fillStyle = css("--ink"); g.beginPath(); g.arc(nose[0], nose[1], 3.5, 0, 7); g.fill();

  const arrow = (vec, col) => {
    if (!vec) return;
    const a = px([0,0,0]), b = px(toScene(vec));
    line(a, b, col, 3);
    const t = Math.atan2(b[1]-a[1], b[0]-a[0]);
    g.fillStyle = col; g.beginPath(); g.moveTo(b[0], b[1]);
    g.lineTo(b[0]-10*Math.cos(t-.4), b[1]-10*Math.sin(t-.4));
    g.lineTo(b[0]-10*Math.cos(t+.4), b[1]-10*Math.sin(t+.4));
    g.closePath(); g.fill();
  };
  if (sample) { arrow(sample.predicted, css("--pred")); arrow(sample.measured, css("--meas")); }
  if (sample && sample.predicted) {     // the difference itself, tip to tip
    line(px(toScene(sample.predicted)), px(toScene(sample.measured)), css("--ink"), 2);
  }

  const bar = SCALE_BARS.find(ut => ut * ARROW_PER_UT * u <= 110) || 1;
  const length = bar * ARROW_PER_UT * u;
  line([12, h - 12], [12 + length, h - 12], css("--dim"), 2);
  g.fillStyle = css("--dim"); g.font = "12px system-ui"; g.textAlign = "left";
  g.fillText(bar + " \u00b5T", 12, h - 20);
}
