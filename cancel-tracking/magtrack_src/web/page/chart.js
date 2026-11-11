// One plot per trace, per sensor, all on the same µT scale so they compare directly.
const TRACES = [
  ["trace-size", "|error|", s => s.size],
  ["trace-x", "x", s => s.v[0]],
  ["trace-y", "y", s => s.v[1]],
  ["trace-z", "z", s => s.v[2]],
];
const NICE_TOPS = [10, 20, 50, 100, 200, 500, 1000, 2000, 5000];
const PLOT_LEFT = 38, PLOT_TOP = 20, PLOT_BOTTOM = 4, AXIS_BOTTOM = 18;
let hoverFrac = null;              // pointer position across the plots, 0-1

SIDES.forEach(side => {
  side.part("traces").addEventListener("pointermove", e => {
    const r = side.part("trace-size").getBoundingClientRect();
    hoverFrac = Math.min(1, Math.max(0, (e.clientX - r.left - PLOT_LEFT) / (r.width - PLOT_LEFT)));
  });
  side.part("traces").addEventListener("pointerleave", () => { hoverFrac = null; });
});

function nearestSample(hist, t) {
  let best = null;
  for (const s of hist) if (best === null || Math.abs(s.t - t) < Math.abs(best.t - t)) best = s;
  return best;
}

function drawTrace(canvas, signed, label, get, hist, spanS, lim, top, now, hover, last) {
  const [g, w, h] = fit(canvas);
  const bottom = h - (last ? AXIS_BOTTOM : PLOT_BOTTOM);
  const low = signed ? -top : 0;
  const X = t => PLOT_LEFT + (t - now + spanS) / spanS * (w - PLOT_LEFT);
  const Y = v => PLOT_TOP + (top - v) / (top - low) * (bottom - PLOT_TOP);

  g.font = "11px system-ui";
  g.textBaseline = "middle";
  g.lineWidth = 1;
  const rule = (x1, y1, x2, y2) => { g.beginPath(); g.moveTo(x1, y1); g.lineTo(x2, y2); g.stroke(); };
  for (const v of signed ? [top, 0, -top] : [top, 0]) {
    g.strokeStyle = css("--line");
    rule(PLOT_LEFT, Y(v), w, Y(v));
    g.fillStyle = css("--dim");
    g.textAlign = "right";
    g.fillText(String(v), PLOT_LEFT - 6, Y(v));
  }
  for (let back = 0; back <= spanS; back += 5) {
    const x = X(now - back);
    g.strokeStyle = css("--line");
    rule(x, PLOT_TOP, x, bottom);
    if (last) {
      g.fillStyle = css("--dim");
      g.textAlign = back === 0 ? "right" : back === spanS ? "left" : "center";
      g.fillText(back === 0 ? "now" : "-" + back + " s", x, h - 7);
    }
  }
  g.strokeStyle = css("--dim");
  g.setLineDash([4, 4]);
  for (const v of signed ? [lim, -lim] : [lim]) rule(PLOT_LEFT, Y(v), w, Y(v));
  g.setLineDash([]);

  // green inside the limit, red outside
  const good = css("--good"), bad = css("--bad");
  g.lineWidth = 2;
  g.lineCap = "round";
  hist.slice(1).forEach((s, i) => {
    g.strokeStyle = Math.abs(get(s)) < lim ? good : bad;
    g.beginPath();
    g.moveTo(X(hist[i].t), Y(get(hist[i])));
    g.lineTo(X(s.t), Y(get(s)));
    g.stroke();
  });

  const shown = hover || hist[hist.length - 1];
  if (hover) {
    g.strokeStyle = css("--dim");
    g.lineWidth = 1;
    rule(X(hover.t), PLOT_TOP, X(hover.t), bottom);
    g.fillStyle = css("--trace");
    g.beginPath(); g.arc(X(hover.t), Y(get(hover)), 4, 0, 7); g.fill();
  }
  g.fillStyle = css("--ink");
  g.font = "600 13px system-ui";
  g.textAlign = "left";
  g.fillText(label, PLOT_LEFT, 9);
  g.textAlign = "right";
  g.font = "13px system-ui";
  g.fillText(shown ? get(shown).toFixed(1) + " µT" : "–", w, 9);
}

function drawCharts(spanS, lim) {
  const now = performance.now() / 1000;
  const peak = Math.max(0, ...SIDES.flatMap(side =>
    side.hist.map(s => Math.max(s.size, ...s.v.map(Math.abs)))));
  const top = NICE_TOPS.find(t => t >= peak * 1.05) || peak * 1.05;
  SIDES.forEach(side => {
    const hover = hoverFrac === null ? null : nearestSample(side.hist, now - spanS + hoverFrac * spanS);
    TRACES.forEach(([part, label, get], i) =>
      drawTrace(side.part(part), part !== "trace-size", label, get, side.hist, spanS, lim, top, now,
                hover, i === TRACES.length - 1));
  });
}
