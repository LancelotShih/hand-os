"""Draws a log's samples.csv as one self-contained plot.html.

Every trace gets its own plot.  The error plots share one scale.  Long logs
are thinned so the file stays small, keeping each stretch's highest and lowest
points so a spike can't disappear.

To redraw a log:  python3 -m magtrack_src.recording.log_plot data/logs/<name>
"""

import csv
import html
import json
import sys
from pathlib import Path

from magtrack_src.config import PASS_MIN_TURN_DEG, PASS_UT

SAMPLE_COLUMNS = ["t_s", "qw", "qx", "qy", "qz", "yaw_deg", "pitch_deg", "roll_deg", "turn_deg",
                  "measured_x_ut", "measured_y_ut", "measured_z_ut",
                  "predicted_x_ut", "predicted_y_ut", "predicted_z_ut",
                  "error_x_ut", "error_y_ut", "error_z_ut", "error_ut",
                  "error_deg",
                  "mmc_measured_x_ut", "mmc_measured_y_ut", "mmc_measured_z_ut",
                  "mmc_predicted_x_ut", "mmc_predicted_y_ut", "mmc_predicted_z_ut",
                  "mmc_error_x_ut", "mmc_error_y_ut", "mmc_error_z_ut",
                  "mmc_error_ut", "mmc_error_deg", "calibration_on", "mmc_calibration_on",
                  "pos_x_cm", "pos_y_cm", "pos_z_cm"]

MAX_BUCKETS = 1500        # thinning: at most two points per bucket
MAX_GAP_S = 1.0           # a longer gap in the data breaks the line
NICE_TOPS = (10, 20, 50, 100, 200, 500, 1000, 2000, 5000)
POSITION_TOPS = (5, 10, 15, 20, 30, 50)   # cm

WIDTH, LEFT, RIGHT = 900, 52, 14
PLOT_H, HEADER_H, AXIS_H = 96, 26, 24


def read_samples(folder):
    """Each column of samples.csv as a list, ``None`` where it's blank."""
    columns = {name: [] for name in SAMPLE_COLUMNS}
    with open(Path(folder) / "samples.csv", newline="") as f:
        for row in csv.DictReader(f):
            for name in SAMPLE_COLUMNS:
                value = row.get(name) or row.get(name.replace("error", "residual"))  # older logs
                columns[name].append(float(value) if value else None)
    return columns


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
        traces += [(f"{label}direction error", "°", thin(ts, c[p + "error_deg"]), 0.0, deg, ())
                   for label, p in sides]
    traces += [(f"{label}|error|", "µT", thin(ts, c[p + "error_ut"]), 0.0, top, (PASS_UT,))
               for label, p in sides]
    for axis in "xyz":
        traces += [(f"{label}error {axis}" if label else f"Error {axis}", "µT",
                    thin(ts, c[f"{p}error_{axis}_ut"]), -top, top, ()) for label, p in sides]
    traces.append(("Turned from capture", "°", thin(ts, c["turn_deg"]), 0.0, 180.0,
                   (PASS_MIN_TURN_DEG,)))
    if any(v is not None for v in c.get("pos_x_cm", ())):
        far = max(abs(v) for k in ("pos_x_cm", "pos_y_cm", "pos_z_cm") for v in c[k] if v is not None)
        span = next((top for top in POSITION_TOPS if top >= far), far)
        traces += [(f"Position {axis}, from home", "cm", thin(ts, c[f"pos_{axis}_cm"]), -span, span, ())
                   for axis in "xyz"]
    return traces


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
    tiles = "".join((_tile("Length", f'{stats["seconds"]:.1f} s'),
                     _tile("Samples", f'{stats["samples"]:,}')))
    for label, prefix in (("ICM ", ""), ("MMC ", "mmc_")) if has_mmc(c) else (("", ""),):
        side = summarize(c, prefix)
        tiles += "".join((
            _tile(f"{label}mean error", fmt(side["mean"], "µT")),
            _tile(f"{label}worst error", fmt(side["worst"], "µT")),
            _tile(f"{label}under {PASS_UT:g} µT once turned {PASS_MIN_TURN_DEG:g}°+",
                  fmt(side["under"], "%"))))
    data = json.dumps([{"name": f"{t[0]} ({t[1]})", "points": [[round(p[0], 3), round(p[1], 2)] for p in t[2]]}
                       for t in traces], separators=(",", ":"))
    return (PAGE.replace("{{NAME}}", html.escape(name))
                .replace("{{TILES}}", tiles)
                .replace("{{HEIGHT}}", str(height))
                .replace("{{WIDTH}}", str(WIDTH))
                .replace("{{SVG}}", "\n".join(svg))
                .replace("{{DATA}}", data)
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
<title>Earth Cancel Log</title>
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
  <div class="tiles">{{TILES}}</div>
  <div class="card" id="plots">
    <svg id="svg" viewBox="0 0 {{WIDTH}} {{HEIGHT}}" role="img" aria-label="Error and turn angle over time">
{{SVG}}
<line id="cross" y1="0" y2="{{HEIGHT}}"/>
    </svg>
    <div id="tip"></div>
  </div>
  <div>Raw data: <a href="samples.csv">samples.csv</a> &middot; <a href="raw.csv">raw.csv</a> &middot; <a href="info.json">info.json</a></div>
</main>
<script>
const traces = {{DATA}}, scale = {{SCALE}};
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
</script>
</body>
</html>
"""


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    write_plot(sys.argv[1])
    print(Path(sys.argv[1]) / "plot.html")
