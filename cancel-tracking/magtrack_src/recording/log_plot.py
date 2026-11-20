"""Draws a log's samples.csv as one self-contained plot.html.

Every trace gets its own plot.  The error plots share one scale.  Long logs
are thinned so the file stays small, keeping each stretch's highest and lowest
points so a spike can't disappear.

To redraw a log:  python3 -m magtrack_src.recording.log_plot data/logs/<name>

Used by: recording/data_log.py, recording/compare.py.  Uses: config.
"""

import html
import json
import math
import sys
from pathlib import Path

from magtrack_src.config import PASS_MIN_TURN_DEG, PASS_UT

# Compatibility exports: existing callers may still import these here.
from magtrack_src.recording.schema import SAMPLE_COLUMNS, read_samples

MAX_BUCKETS = 1500        # thinning: at most two points per bucket
MAX_GAP_S = 1.0           # a longer gap in the data breaks the line
NICE_TOPS = (10, 20, 50, 100, 200, 500, 1000, 2000, 5000)
POSITION_TOPS = (5, 10, 15, 20, 30, 50)   # cm

WIDTH, LEFT, RIGHT = 900, 52, 14
PLOT_H, HEADER_H, AXIS_H = 96, 26, 24
MAX_TRAJECTORY_SAMPLES = 12000


def thin(ts, vs, buckets=MAX_BUCKETS):
    """``(t, v)`` points with a stretch's lowest and highest kept, in time order."""
    points = [(t, v) for t, v in zip(ts, vs) if v is not None]
    if len(points) <= 2 * buckets:
        return points
    out = []
    for i in range(buckets):
        part = points[i * len(points) // buckets:(i + 1) * len(points) // buckets]
        low, high = min(part, key=lambda p: p[1]), max(part, key=lambda p: p[1])
        out.extend(sorted({low, high}))
    return out


def nice_top(value):
    return next((top for top in NICE_TOPS if top >= value * 1.05), value * 1.05)


def time_ticks(end):
    step = next((s for s in (1, 2, 5, 10, 30, 60, 120, 300, 600, 1800, 3600)
                 if end / s <= 8), 7200)
    return [i * step for i in range(int(end // step) + 1)]


def has_mmc(c):
    """Older logs, and runs with the MMC off, hold one magnetometer."""
    return any(v is not None for v in c.get("mmc_error_ut", ()))


def summarize(c, prefix=""):
    errors = [e for e in c[prefix + "error_ut"] if e is not None]
    turned = [e for e, t in zip(c[prefix + "error_ut"], c["turn_deg"])
              if e is not None and t >= PASS_MIN_TURN_DEG]
    return {"seconds": c["t_s"][-1] if c["t_s"] else 0.0,
            "samples": len(c["t_s"]),
            "mean": sum(errors) / len(errors) if errors else None,
            "worst": max(errors) if errors else None,
            "under": 100.0 * sum(e < PASS_UT for e in turned) / len(turned) if turned else None}


def _line_paths(points, x, y):
    """Polyline strings, broken wherever the data has a gap."""
    lines, current, last_t = [], [], None
    for t, v in points:
        if last_t is not None and t - last_t > MAX_GAP_S:
            lines.append(current)
            current = []
        current.append(f"{x(t):.1f},{y(v):.1f}")
        last_t = t
    lines.append(current)
    return [" ".join(line) for line in lines if line]


def _plot(title, unit, points, low, high, target, end, top_y, last):
    """SVG for one trace, its plot area starting at ``top_y``."""
    x = lambda t: LEFT + (t / end) * (WIDTH - LEFT - RIGHT)
    y = lambda v: top_y + PLOT_H * (high - v) / (high - low)
    parts = [f'<text class="title" x="{LEFT}" y="{top_y - 9}">{html.escape(title)}'
             f'<tspan class="axis"> {unit}</tspan></text>']
    for v in sorted({low, 0.0, high}):
        parts.append(f'<line class="grid" x1="{LEFT}" x2="{WIDTH - RIGHT}" y1="{y(v):.1f}" y2="{y(v):.1f}"/>')
        parts.append(f'<text class="axis" x="{LEFT - 7}" y="{y(v) + 4:.1f}" text-anchor="end">{v:g}</text>')
    for t in time_ticks(end):
        parts.append(f'<line class="grid" x1="{x(t):.1f}" x2="{x(t):.1f}" y1="{top_y}" y2="{top_y + PLOT_H}"/>')
        if last:
            parts.append(f'<text class="axis" x="{x(t):.1f}" y="{top_y + PLOT_H + 17}" '
                         f'text-anchor="middle">{t:g} s</text>')
    for v in target:
        parts.append(f'<line class="target" x1="{LEFT}" x2="{WIDTH - RIGHT}" y1="{y(v):.1f}" y2="{y(v):.1f}"/>')
    parts.extend(f'<polyline class="trace" points="{line}"/>' for line in _line_paths(points, x, y))
    if not points:
        parts.append(f'<text class="axis" x="{WIDTH / 2}" y="{top_y + PLOT_H / 2}" '
                     'text-anchor="middle">no data</text>')
    return "\n".join(parts)


def _traces(c):
    ts = c["t_s"]
    sides = (("ICM ", ""), ("MMC ", "mmc_")) if has_mmc(c) else (("", ""),)
    names = ("error_x_ut", "error_y_ut", "error_z_ut", "error_ut")
    top = nice_top(max((abs(v) for _, p in sides for k in names for v in c[p + k] if v is not None),
                       default=0.0))
    traces = []
    if has_mmc(c):                  # every plot in a pair shares one scale
        deg = nice_top(max((v for _, p in sides for v in c[p + "error_deg"] if v is not None),
                           default=0.0))
        traces += [(f"{label}angle between measured and reference fields", "°", thin(ts, c[p + "error_deg"]), 0.0, deg, ())
                   for label, p in sides]
    traces += [(f"{label}field difference from the reference", "µT", thin(ts, c[p + "error_ut"]), 0.0, top, (PASS_UT,))
               for label, p in sides]
    for axis in "xyz":
        traces += [(f"{label}field difference along {axis}", "µT",
                    thin(ts, c[f"{p}error_{axis}_ut"]), -top, top, ()) for label, p in sides]
    traces.append(("Board rotation since reference measurement", "°", thin(ts, c["turn_deg"]), 0.0, 180.0,
                   (PASS_MIN_TURN_DEG,)))
    if any(v is not None for v in c.get("pos_x_cm", ())):
        far = max(abs(v) for k in ("pos_x_cm", "pos_y_cm", "pos_z_cm") for v in c[k] if v is not None)
        span = next((top for top in POSITION_TOPS if top >= far), far)
        traces += [(f"Estimated position along {axis}, from the entered coil center", "cm", thin(ts, c[f"pos_{axis}_cm"]), -span, span, ())
                   for axis in "xyz"]
    return traces


def trajectory_data(c):
    """Time-ordered estimated poses for the interactive replay; missing fixes stay as gaps."""
    times = c["t_s"]
    stride = max(1, math.ceil(len(times) / MAX_TRAJECTORY_SAMPLES))
    indices = list(range(0, len(times), stride))
    if indices and indices[-1] != len(times) - 1:
        indices.append(len(times) - 1)

    def position_is_valid(i):
        return all(
            c[key][i] is not None and math.isfinite(c[key][i])
            for key in ("pos_x_cm", "pos_y_cm", "pos_z_cm")
        )

    def has_gap(start, end):
        return any(
            (not position_is_valid(i - 1) or times[i] - times[i - 1] > MAX_GAP_S)
            for i in range(start + 1, end + 1)
        )

    def vector(keys, i):
        values = [c[key][i] for key in keys]
        if any(value is None or not math.isfinite(value) for value in values):
            return None
        return [round(value, 5) for value in values]

    def flag(name, i):
        values = c.get(name, ())
        return bool(values[i]) if i < len(values) and values[i] is not None else False

    poses = []
    previous = 0
    for i in indices:
        poses.append({"t": round(times[i], 3),
                      "p": vector(("pos_x_cm", "pos_y_cm", "pos_z_cm"), i),
                      "q": vector(("qw", "qx", "qy", "qz"), i),
                      "history": flag("position_history_based", i),
                      "ambiguous": flag("position_ambiguous", i),
                      "break": bool(poses) and has_gap(previous, i)})
        previous = i
    return poses


def _tile(label, value):
    return f'<div class="tile"><div class="value">{value}</div><div class="label">{label}</div></div>'


def render(name, c):
    stats = summarize(c)
    traces = _traces(c)
    end = max(stats["seconds"], 1.0)
    rows = len(traces)
    height = rows * (PLOT_H + HEADER_H) + AXIS_H
    svg = [_plot(title, unit, points, low, high, target, end,
                 HEADER_H + i * (PLOT_H + HEADER_H), i == rows - 1)
           for i, (title, unit, points, low, high, target) in enumerate(traces)]
    fmt = lambda v, u: "–" if v is None else f"{v:.1f} {u}"
    tiles = "".join((_tile("Recording duration", f'{stats["seconds"]:.1f} s'),
                     _tile("Recorded view samples", f'{stats["samples"]:,}')))
    for label, prefix in (("ICM ", ""), ("MMC ", "mmc_")) if has_mmc(c) else (("", ""),):
        side = summarize(c, prefix)
        tiles += "".join((
            _tile(f"{label}mean field difference from the reference", fmt(side["mean"], "µT")),
            _tile(f"{label}largest field difference from the reference", fmt(side["worst"], "µT")),
            _tile(f"{label}samples below {PASS_UT:g} µT, after at least {PASS_MIN_TURN_DEG:g}° board rotation",
                  fmt(side["under"], "%"))))
    data = json.dumps([{"name": f"{t[0]} ({t[1]})", "points": [[round(p[0], 3), round(p[1], 2)] for p in t[2]]}
                       for t in traces], separators=(",", ":"))
    trajectory = json.dumps(trajectory_data(c), separators=(",", ":"))
    return (PAGE.replace("{{NAME}}", html.escape(name))
                .replace("{{TILES}}", tiles)
                .replace("{{HEIGHT}}", str(height))
                .replace("{{WIDTH}}", str(WIDTH))
                .replace("{{SVG}}", "\n".join(svg))
                .replace("{{DATA}}", data)
                .replace("{{TRAJECTORY}}", trajectory)
                .replace("{{SCALE}}", json.dumps({"left": LEFT, "width": WIDTH - LEFT - RIGHT,
                                                  "end": end})))


def write_plot(folder):
    folder = Path(folder)
    (folder / "plot.html").write_text(render(folder.name, read_samples(folder)))


PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Magnetic field tracking recording</title>
<style>
:root { --bg:#f6f6f4; --card:#fff; --ink:#1d1f21; --dim:#6b7075; --line:#e2e2de; --trace:#2f6fde; }
@media (prefers-color-scheme: dark) {
  :root { --bg:#141618; --card:#1e2124; --ink:#e8e9ea; --dim:#99a0a6; --line:#34383c; --trace:#6ea0f5; }
}
* { box-sizing:border-box; }
body { margin:0; padding:16px; background:var(--bg); color:var(--ink); font:15px/1.45 system-ui,sans-serif; }
main { max-width:960px; margin:0 auto; display:grid; gap:12px; }
h1 { font-size:18px; margin:0; }
.card { background:var(--card); border:1px solid var(--line); border-radius:8px; padding:12px 14px; position:relative; }
.tiles { display:grid; gap:12px; grid-template-columns:repeat(auto-fit,minmax(150px,1fr)); }
.value { font-size:24px; font-weight:600; }
.label { color:var(--dim); font-size:13px; }
svg { display:block; width:100%; height:auto; }
.title { fill:var(--ink); font-size:14px; font-weight:600; }
.axis { fill:var(--dim); font-size:12px; font-weight:400; font-variant-numeric:tabular-nums; }
.grid { stroke:var(--line); stroke-width:1; }
.target { stroke:var(--dim); stroke-width:1; stroke-dasharray:4 4; }
.trace { fill:none; stroke:var(--trace); stroke-width:2; stroke-linejoin:round; stroke-linecap:round; }
.trajectory-controls { display:flex; align-items:center; gap:10px; margin:8px 0; }
.trajectory-controls input { flex:1; min-width:80px; }
#trajectory { display:block; width:100%; aspect-ratio:4/3; touch-action:none; cursor:grab; }
#trajectory:active { cursor:grabbing; }
[hidden] { display:none !important; }
#cross { stroke:var(--dim); stroke-width:1; visibility:hidden; }
#tip { position:absolute; display:none; pointer-events:none; background:var(--card); border:1px solid var(--line);
       border-radius:6px; padding:6px 9px; font-size:13px; white-space:nowrap; font-variant-numeric:tabular-nums; }
#tip b { font-weight:600; }
a { color:var(--trace); }
</style>
</head>
<body>
<main>
  <h1>{{NAME}}</h1>
  <p>The field-difference plots show measured magnetic field minus captured reference field.
  Their overall size combines all three components; it is not just a difference in strengths.
  With the coil on, sensor movement is expected to change this reading.
  Board rotation is the change in board orientation since the reference measurement;
  the angle between fields compares the magnetic readings themselves.
  Dashed lines use the current comparison thresholds.</p>
  <div class="tiles">{{TILES}}</div>
  <div class="card" id="trajectory-card" hidden>
    <h2>Estimated 3D movement</h2>
    <p>The line traces recorded position estimates; the green square shows the recorded board orientation at the selected time.
    Red dashed sections were marked ambiguous and selected using the previous estimate. Gaps are left unconnected.</p>
    <div class="trajectory-controls">
      <button id="trajectory-play" type="button">Play</button>
      <input id="trajectory-slider" type="range" min="0" max="0" value="0" step="1" aria-label="Trajectory playback time">
      <output id="trajectory-time"></output>
    </div>
    <canvas id="trajectory" aria-label="Interactive 3D estimated position trajectory"></canvas>
    <p class="label">Drag to rotate; scroll to zoom. Coordinates and orientation are recorded estimates, not independent ground truth.</p>
  </div>
  <div class="card" id="plots">
    <svg id="svg" viewBox="0 0 {{WIDTH}} {{HEIGHT}}" role="img" aria-label="Differences from the magnetic reference field and board rotation over time">
{{SVG}}
<line id="cross" y1="0" y2="{{HEIGHT}}"/>
    </svg>
    <div id="tip"></div>
  </div>
  <div>Recording files: <a href="samples.csv">Displayed estimates (samples.csv)</a> &middot; <a href="raw.csv">Sensor readings (raw.csv)</a> &middot; <a href="info.json">Starting state (info.json)</a></div>
</main>
<script>
const traces = {{DATA}}, scale = {{SCALE}}, trajectory = {{TRAJECTORY}};
const svg = document.getElementById("svg"), cross = document.getElementById("cross"),
      tip = document.getElementById("tip"), box = document.getElementById("plots");

function nearest(points, t) {
  let best = null;
  for (const p of points) if (best === null || Math.abs(p[0] - t) < Math.abs(best[0] - t)) best = p;
  return best;
}

svg.addEventListener("pointermove", e => {
  const r = svg.getBoundingClientRect(), k = r.width / svg.viewBox.baseVal.width;
  const x = (e.clientX - r.left) / k;
  const t = Math.min(scale.end, Math.max(0, (x - scale.left) / scale.width * scale.end));
  const sx = scale.left + t / scale.end * scale.width;
  cross.setAttribute("x1", sx); cross.setAttribute("x2", sx);
  cross.style.visibility = "visible";
  tip.textContent = "";
  const head = document.createElement("b");
  head.textContent = t.toFixed(2) + " s";
  tip.appendChild(head);
  for (const tr of traces) {
    const p = nearest(tr.points, t);
    const row = document.createElement("div");
    row.textContent = tr.name + ": " + (p ? p[1].toFixed(1) : "–");
    tip.appendChild(row);
  }
  tip.style.display = "block";
  const b = box.getBoundingClientRect(), left = e.clientX - b.left + 14;
  tip.style.left = Math.min(left, b.width - tip.offsetWidth - 8) + "px";
  tip.style.top = (e.clientY - b.top + 14) + "px";
});
svg.addEventListener("pointerleave", () => { cross.style.visibility = "hidden"; tip.style.display = "none"; });

const trajectoryCard = document.getElementById("trajectory-card");
const trajectoryCanvas = document.getElementById("trajectory");
const trajectorySlider = document.getElementById("trajectory-slider");
const trajectoryPlay = document.getElementById("trajectory-play");
const trajectoryTime = document.getElementById("trajectory-time");
const validTrajectory = trajectory.some(row => row.p);
if (validTrajectory) {
  trajectoryCard.hidden = false;
  trajectorySlider.max = trajectory.length - 1;
  let trajectoryIndex = 0, trajectoryYaw = -35, trajectoryPitch = 25, trajectoryZoom = 1;
  let trajectoryDrag = null, trajectoryPlaying = false, playStartClock = 0, playStartTime = 0;
  let trajectoryExtent = 5;
  for (const row of trajectory) if (row.p) {
    trajectoryExtent = Math.max(trajectoryExtent, Math.hypot(...row.p));
  }
  trajectoryExtent = Math.ceil(trajectoryExtent / 5) * 5;

  function project(p, width, height, unit) {
    const yaw = trajectoryYaw * Math.PI / 180, pitch = trajectoryPitch * Math.PI / 180;
    const x = p[0] * Math.cos(yaw) - p[1] * Math.sin(yaw);
    const y = p[0] * Math.sin(yaw) + p[1] * Math.cos(yaw);
    return [width / 2 + x * unit,
            height / 2 - (y * Math.cos(pitch) - p[2] * Math.sin(pitch)) * unit];
  }

  function rotateByQuaternion(q, v) {
    if (!q) return v;
    const norm = Math.hypot(...q);
    if (!norm) return v;
    const [w, x, y, z] = q.map(value => value / norm);
    return [
      (1 - 2 * (y*y + z*z)) * v[0] + 2 * (x*y - z*w) * v[1] + 2 * (x*z + y*w) * v[2],
      2 * (x*y + z*w) * v[0] + (1 - 2 * (x*x + z*z)) * v[1] + 2 * (y*z - x*w) * v[2],
      2 * (x*z - y*w) * v[0] + 2 * (y*z + x*w) * v[1] + (1 - 2 * (x*x + y*y)) * v[2]
    ];
  }

  function drawTrajectory() {
    const bounds = trajectoryCanvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    const width = bounds.width, height = bounds.height;
    trajectoryCanvas.width = Math.round(width * dpr);
    trajectoryCanvas.height = Math.round(height * dpr);
    const ctx = trajectoryCanvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    const unit = Math.min(width, height) / (2 * trajectoryExtent * 1.3) * trajectoryZoom;
    const at = p => project(p, width, height, unit);
    const strokeLine = (a, b, color, alpha, lineWidth, dash) => {
      const start = at(a), end = at(b);
      ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.lineWidth = lineWidth;
      ctx.setLineDash(dash || []);
      ctx.beginPath(); ctx.moveTo(start[0], start[1]); ctx.lineTo(end[0], end[1]); ctx.stroke();
    };
    for (let v = -trajectoryExtent; v <= trajectoryExtent; v += 5) {
      const zero = v === 0;
      strokeLine([v, -trajectoryExtent, 0], [v, trajectoryExtent, 0],
                 zero ? "#6b7075" : "#b9bdc1", zero ? 0.8 : 0.5, zero ? 1.5 : 1);
      strokeLine([-trajectoryExtent, v, 0], [trajectoryExtent, v, 0],
                 zero ? "#6b7075" : "#b9bdc1", zero ? 0.8 : 0.5, zero ? 1.5 : 1);
      const xTick = at([v, -trajectoryExtent, 0]), yTick = at([-trajectoryExtent, v, 0]);
      ctx.globalAlpha = 0.9; ctx.fillStyle = "#6b7075"; ctx.font = "11px system-ui";
      ctx.fillText(String(v), xTick[0] + 3, xTick[1] - 3);
      if (v !== -trajectoryExtent) ctx.fillText(String(v), yTick[0] + 3, yTick[1] - 3);
    }
    strokeLine([0, 0, 0], [trajectoryExtent, 0, 0], "#6b7075", 0.9, 1.5);
    strokeLine([0, 0, 0], [0, trajectoryExtent, 0], "#6b7075", 0.9, 1.5);
    strokeLine([0, 0, 0], [0, 0, trajectoryExtent], "#6b7075", 0.9, 1.5);
    ctx.globalAlpha = 0.9; ctx.fillStyle = "#6b7075"; ctx.font = "12px system-ui";
    for (const [p, text] of [[[trajectoryExtent, 0, 0], "x (cm)"],
                             [[0, trajectoryExtent, 0], "y (cm)"],
                             [[0, 0, trajectoryExtent], "z (cm)"]]) {
      const point = at(p); ctx.fillText(text, point[0] + 4, point[1] - 4);
    }

    function drawPath(maxIndex, alpha) {
      let points = [], last = null, ambiguous = null;
      const flush = () => {
        if (points.length > 1) {
          ctx.globalAlpha = alpha;
          ctx.strokeStyle = ambiguous ? "#c62f2f" : "#2f6fde";
          ctx.lineWidth = 2; ctx.setLineDash(ambiguous ? [5, 4] : []);
          ctx.beginPath(); points.forEach((point, i) => i
            ? ctx.lineTo(point[0], point[1]) : ctx.moveTo(point[0], point[1]));
          ctx.stroke();
        }
        points = []; ambiguous = null;
      };
      for (let i = 0; i <= maxIndex; i++) {
        const row = trajectory[i];
        if (!row.p) { flush(); last = null; continue; }
        if (last && row.break) { flush(); last = null; }
        if (last) {
          const uncertain = !!(row.ambiguous || last.ambiguous);
          if (ambiguous !== null && uncertain !== ambiguous) flush();
          if (!points.length) points.push(at(last.p));
          points.push(at(row.p)); ambiguous = uncertain;
        }
        last = row;
      }
      flush();
    }
    drawPath(trajectory.length - 1, 0.2);
    drawPath(trajectoryIndex, 0.95);

    const current = trajectory[trajectoryIndex];
    if (current.p) {
      const corners = [[-0.6, -0.6, 0], [0.6, -0.6, 0], [0.6, 0.6, 0], [-0.6, 0.6, 0]]
        .map(offset => at(current.p.map((v, k) => v + rotateByQuaternion(current.q, offset)[k])));
      ctx.globalAlpha = 1; ctx.fillStyle = "#3d7f6e"; ctx.strokeStyle = "#1d1f21"; ctx.lineWidth = 1.5;
      ctx.beginPath(); corners.forEach((point, i) => i
        ? ctx.lineTo(point[0], point[1]) : ctx.moveTo(point[0], point[1]));
      ctx.closePath(); ctx.fill(); ctx.stroke();
      const nose = at(current.p.map((v, k) => v + rotateByQuaternion(current.q, [0.6, 0, 0])[k]));
      ctx.fillStyle = "#fff"; ctx.beginPath(); ctx.arc(nose[0], nose[1], 2, 0, Math.PI * 2); ctx.fill();
      if (current.ambiguous) {
        const center = at(current.p);
        ctx.strokeStyle = "#c62f2f"; ctx.lineWidth = 2; ctx.setLineDash([3, 3]);
        ctx.beginPath(); ctx.arc(center[0], center[1], 10, 0, Math.PI * 2); ctx.stroke();
      }
    }
    ctx.globalAlpha = 1; ctx.setLineDash([]);
    trajectoryTime.textContent = current.t.toFixed(2) + " s"
      + (current.p ? current.ambiguous ? " · ambiguous" : "" : " · no position estimate");
  }

  function selectTrajectoryIndex(index) {
    trajectoryIndex = Math.max(0, Math.min(trajectory.length - 1, index));
    trajectorySlider.value = trajectoryIndex;
    drawTrajectory();
  }
  trajectorySlider.addEventListener("input", () => {
    trajectoryPlaying = false; trajectoryPlay.textContent = "Play";
    selectTrajectoryIndex(Number(trajectorySlider.value));
  });
  trajectoryPlay.addEventListener("click", () => {
    if (trajectoryPlaying) {
      trajectoryPlaying = false; trajectoryPlay.textContent = "Play"; return;
    }
    if (trajectoryIndex === trajectory.length - 1) selectTrajectoryIndex(0);
    trajectoryPlaying = true; trajectoryPlay.textContent = "Pause";
    playStartClock = performance.now(); playStartTime = trajectory[trajectoryIndex].t;
    requestAnimationFrame(advanceTrajectory);
  });
  function advanceTrajectory(now) {
    if (!trajectoryPlaying) return;
    const targetTime = playStartTime + (now - playStartClock) / 1000;
    let next = trajectoryIndex;
    while (next + 1 < trajectory.length && trajectory[next + 1].t <= targetTime) next++;
    if (next !== trajectoryIndex) selectTrajectoryIndex(next);
    if (trajectoryIndex === trajectory.length - 1) {
      trajectoryPlaying = false; trajectoryPlay.textContent = "Play"; return;
    }
    requestAnimationFrame(advanceTrajectory);
  }
  trajectoryCanvas.addEventListener("pointerdown", e => {
    trajectoryDrag = [e.clientX, e.clientY]; trajectoryCanvas.setPointerCapture(e.pointerId);
  });
  trajectoryCanvas.addEventListener("pointermove", e => {
    if (!trajectoryDrag) return;
    trajectoryYaw += (e.clientX - trajectoryDrag[0]) * 0.5;
    trajectoryPitch = Math.max(-89, Math.min(89, trajectoryPitch + (e.clientY - trajectoryDrag[1]) * 0.5));
    trajectoryDrag = [e.clientX, e.clientY]; drawTrajectory();
  });
  trajectoryCanvas.addEventListener("pointerup", () => { trajectoryDrag = null; });
  trajectoryCanvas.addEventListener("pointercancel", () => { trajectoryDrag = null; });
  trajectoryCanvas.addEventListener("wheel", e => {
    e.preventDefault(); trajectoryZoom = Math.max(0.4, Math.min(6, trajectoryZoom * Math.exp(-e.deltaY * 0.001)));
    drawTrajectory();
  }, {passive: false});
  window.addEventListener("resize", drawTrajectory);
  selectTrajectoryIndex(0);
}
</script>
</body>
</html>
"""


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    write_plot(sys.argv[1])
    print(Path(sys.argv[1]) / "plot.html")
