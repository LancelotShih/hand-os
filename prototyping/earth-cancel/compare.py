"""Which magnetometer tracks Earth's direction better?  Scores one log.

    python3 compare.py logs/<name>

Log after a Tumble, with the board turning through large angles for a while and
resting for a few seconds at the end or in the middle.  Only stretches turned
at least PASS_MIN_TURN_DEG from the capture count: near the capture both are
right by construction.
"""

import csv
import math
import sys
from pathlib import Path

from config import PASS_MIN_TURN_DEG
from log_plot import read_samples

STILL_DPS = 3.0           # turning slower than this counts as still ...
STILL_MIN_S = 1.0         # ... once it has lasted this long
NOISE_WINDOW_S = 0.05     # noise is judged on averages this long, so rate is accounted for


def percentile(values, share):
    return sorted(values)[int(share * (len(values) - 1))]


def scores(values):
    """Typical, 95% and worst of a list, or ``None`` if it's empty."""
    return (percentile(values, 0.5), percentile(values, 0.95), max(values)) if values else None


def still_stretches(rows):
    """Lists of rows, each a stretch of at least STILL_MIN_S with the gyro quiet."""
    out, part = [], []
    for row in rows:
        if max(abs(float(row[k])) for k in ("gx_dps", "gy_dps", "gz_dps")) < STILL_DPS:
            part.append(row)
            continue
        if part and float(part[-1]["t_s"]) - float(part[0]["t_s"]) >= STILL_MIN_S:
            out.append(part)
        part = []
    if part and float(part[-1]["t_s"]) - float(part[0]["t_s"]) >= STILL_MIN_S:
        out.append(part)
    return out


def noise_ut(stretches, names):
    """How much a NOISE_WINDOW_S average wobbles on a still board: the size of the
    wobble across all three axes, in uT.  ``None`` with too few averages."""
    total, count = 0.0, 0
    for part in stretches:
        windows = {}
        for row in part:
            if row[names[0]]:
                n = int(float(row["t_s"]) / NOISE_WINDOW_S)
                windows.setdefault(n, []).append([float(row[k]) for k in names])
        means = [[sum(v) / len(r) for v in zip(*r)] for r in windows.values()]
        if len(means) < 3:
            continue
        centre = [sum(v) / len(means) for v in zip(*means)]
        total += sum((v - c) ** 2 for m in means for v, c in zip(m, centre))
        count += len(means) - 1
    return math.sqrt(total / count) if count else None


def rate_hz(rows, name):
    n = sum(1 for r in rows if r[name])
    return n / float(rows[-1]["t_s"]) if rows and float(rows[-1]["t_s"]) > 0.0 else 0.0


def calibration_state(column):
    seen = {v for v in column if v is not None}
    return "changed during log" if len(seen) > 1 else "on" if seen == {1.0} else "off" if seen else "–"


def compare(folder):
    c = read_samples(folder)
    with open(Path(folder) / "raw.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows or "mmc_x_ut" not in rows[0]:
        raise ValueError("this log has no MMC readings")
    turned = [i for i, t in enumerate(c["turn_deg"]) if t is not None and t >= PASS_MIN_TURN_DEG]

    def picked(name):
        return [c[name][i] for i in turned if c[name][i] is not None]

    still = still_stretches(rows)
    return {"calibration": {"icm": calibration_state(c["calibration_on"]),
                            "mmc": calibration_state(c["mmc_calibration_on"])},
            "seconds": c["t_s"][-1] if c["t_s"] else 0.0,
            "turned_samples": len(turned),
            "still_s": sum(float(p[-1]["t_s"]) - float(p[0]["t_s"]) for p in still),
            "icm": {"direction": scores(picked("error_deg")), "field": scores(picked("error_ut")),
                    "noise": noise_ut(still, ("mx_ut", "my_ut", "mz_ut")),
                    "rate": rate_hz(rows, "mx_ut")},
            "mmc": {"direction": scores(picked("mmc_error_deg")),
                    "field": scores(picked("mmc_error_ut")),
                    "noise": noise_ut(still, ("mmc_x_ut", "mmc_y_ut", "mmc_z_ut")),
                    "rate": rate_hz(rows, "mmc_x_ut")}}


def verdict(r):
    a, b = r["icm"]["direction"], r["mmc"]["direction"]
    if a is None or b is None:
        return "No verdict: turn the board further from the capture while logging."
    if min(a[0], b[0]) <= 0.0 or max(a[0], b[0]) / min(a[0], b[0]) < 1.2:
        return f"Too close to call on direction: typically {a[0]:.2f}° vs {b[0]:.2f}°."
    better, ratio = ("MMC5983MA", a[0] / b[0]) if b[0] < a[0] else ("ICM-20948", b[0] / a[0])
    return f"{better} tracks Earth's direction better: typical error {ratio:.1f} times smaller."


def report(r):
    def row(label, key, unit, at=None, places=2):
        cells = []
        for side in ("icm", "mmc"):
            v = r[side][key]
            v = v[at] if v is not None and at is not None else v
            cells.append("–" if v is None else f"{v:.{places}f} {unit}")
        return f"{label:<34}{cells[0]:>14}{cells[1]:>14}"

    return "\n".join([
        f'{r["seconds"]:.0f} s log, {r["turned_samples"]} samples turned '
        f'{PASS_MIN_TURN_DEG:g}°+, {r["still_s"]:.0f} s still',
        f'Software calibration: ICM {r["calibration"]["icm"]}, MMC {r["calibration"]["mmc"]}',
        "", f'{"":<34}{"ICM-20948":>14}{"MMC5983MA":>14}',
        row("Direction error, typical", "direction", "°", 0),
        row("Direction error, 95% under", "direction", "°", 1),
        row("Direction error, worst", "direction", "°", 2),
        row("Field error, typical", "field", "µT", 0),
        row("Field error, 95% under", "field", "µT", 1),
        row("Field error, worst", "field", "µT", 2),
        row(f"Still noise, {NOISE_WINDOW_S * 1000:.0f} ms average", "noise", "µT", None, 3),
        row("Readings per second", "rate", "Hz", None, 0),
        "", verdict(r)])


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    try:
        print(report(compare(sys.argv[1])))
    except (OSError, ValueError) as e:
        sys.exit(str(e))
