#!/usr/bin/env python3
"""Locate an ICM-20948 in 3-D relative to a pulsed electromagnet at the origin.

The coil is switched through a repeating pattern (on/off, or +I/off/-I/off)
via the L298N driver in ``../em-driver``.  Magnetometer samples are averaged
per phase -- skipping the first ``SETTLE_S`` after each switch while the coil
current settles -- and :mod:`em_localize` fits the phase averages into the
electromagnet's own field ``B_em`` with Earth's field (and any slow drift)
removed.  ``B_em`` is then inverted through a calibrated dipole model into a
position, streamed to a live 3-D view in the browser.

Commands:
  ./em_locate.py                 # run: http://<this-pi's-ip>:8000
  ./em_locate.py --sim           # same, with a simulated coil + sensor
  ./em_locate.py step            # measure coil settling time to tune SETTLE_S

Coordinate frame (ORIENTATION_MODE):
  "fixed"  x/y/z are the ICM-20948's own silk-screened axes; keep the board
           at the same orientation the whole time (slide it, don't turn it).
  "ahrs"   the board may rotate; x/y/z are its axes at the moment the
           reference pose was set (automatically at startup, or the
           "Set reference pose" button) -- square the board to your jig then.

Once per setup, from the page: hold the board at a known (x, y, z) from the
magnet's centre, type it in, click "Capture point".  One point works; 3-6
spread around the cube average out placement error.  Saved to em_cal.json.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import signal
import socket
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "em-driver"))
sys.path.insert(0, str(HERE.parent / "icm20948-visualizer"))

from em_toggle import IN1_PIN as DRIVER_IN1_PIN, IN2_PIN as DRIVER_IN2_PIN  # noqa: E402
from imu_ahrs import MagCalibration, OrientationTracker, quat_conj, quat_mul  # noqa: E402
import em_localize as eml  # noqa: E402


# --- configuration (adjust freely) -------------------------------------------

# wiring -- defaults come from ../em-driver/em_toggle.py; override here if needed
IN1_PIN = DRIVER_IN1_PIN       # BCM GPIO -> L298N IN1
IN2_PIN = DRIVER_IN2_PIN       # BCM GPIO -> L298N IN2
I2C_BUS = 1
I2C_ADDRESS = 0x68

# pulsing
DRIVE_MODE = "onoff"           # "onoff" (+I, off) or "bipolar" (+I, off, -I, off)
PHASE_S = 0.10                 # seconds the coil holds each state
SETTLE_S = 0.05                # ignore samples this long after a switch (run `step`)
MAG_LATENCY_S = 0.012          # AK09916 sample age when read (100 Hz ODR + conversion)

# orientation
ORIENTATION_MODE = "fixed"     # "fixed" or "ahrs" -- see the module docstring
AHRS_BETA = 0.1                # Madgwick gain
AHRS_SETTLE_S = 3.0            # ahrs: auto-set the reference pose this long after start
TILT_WARN_DEG = 3.0            # fixed: warn when the board tilts this far from start

# web view
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8000
CUBE_HALF_CM = 15.0            # drawn working volume: +/- this on every axis
RAW_CHART_S = 5.0              # seconds of raw magnetometer history in the page's chart

# files
CAL_FILE = HERE / "em_cal.json"
SIM_CAL_FILE = HERE / "em_cal_sim.json"   # --sim never touches the real calibrations
SIM_MAG_CAL_FILE = HERE / "imu_mag_cal_sim.json"
MAG_CAL_FILE = HERE / "imu_mag_cal.json"
MAG_CAL_FALLBACK = HERE.parent / "icm20948-visualizer" / "imu_mag_cal.json"

# `step` command
STEP_PHASE_S = 0.4             # long phases so the coil fully settles
STEP_CYCLES = 10
STEP_BIN_S = 0.005
STEP_TOLERANCE = 0.02          # "settled" = within 2 % of the final value
STEP_MIN_UT = 2.0              # below this on/off step there is no usable coil signal

COIL_ON_AT_START = True        # False: start with the coil held off (turn on from the page)

PATTERNS = {"onoff": (1, 0), "bipolar": (1, 0, -1, 0)}


# --- hardware --------------------------------------------------------------

def open_hardware(args, still=False):
    """Return ``(coil, imu)``, real or simulated (``still``: the simulated
    sensor does not move)."""
    if args.sim:
        from em_sim import SimCoil, SimIMU
        coil = SimCoil()
        imu = SimIMU(coil, still=still)
        imu.enable_magnetometer()
        return coil, imu
    from em_toggle import Coil
    from icm20948_driver import ICM20948
    coil = Coil(args.in1, args.in2)
    try:
        imu = ICM20948(args.bus, address=args.address)
        imu.begin(do_reset=True)
        imu.enable_magnetometer()
    except BaseException:
        coil.close()
        raise
    return coil, imu


def mag_calibration(tracker):
    """The tracker's hard/soft-iron calibration if there is one, otherwise just
    the AK09916 -> body axis remap."""
    return tracker.cal if tracker.cal is not None else MagCalibration()


def _angle_deg(a, b):
    na, nb = eml._norm(a), eml._norm(b)
    if na < 1e-9 or nb < 1e-9:
        return None
    c = max(-1.0, min(1.0, eml._dot(a, b) / (na * nb)))
    return math.degrees(math.acos(c))


# --- shared state between the sensor thread and the browser tabs -------------

class SampleBuffer:
    """Thread-safe ring buffer; each sample gets an increasing ``seq`` so every
    SSE client can resume independently."""

    def __init__(self, maxlen=600):
        self._buf = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._seq = 0
        self.connected = False
        self.error = None

    def push(self, sample):
        with self._lock:
            self._seq += 1
            sample["seq"] = self._seq
            self._buf.append(sample)

    def since(self, seq):
        with self._lock:
            return [s for s in self._buf if s["seq"] > seq]

    def status(self):
        with self._lock:
            return {"connected": self.connected, "error": self.error}


class Engine:
    """Owns the coil and the IMU: switches the coil phase by phase, averages
    the magnetometer per phase, and feeds :mod:`em_localize`."""

    def __init__(self, args, tracker, localizer, buf):
        self.args = args
        self.tracker = tracker
        self.localizer = localizer
        self.buf = buf
        self.pattern = PATTERNS[args.mode]
        self.demod = eml.PhaseDemod()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._q_ref = None            # ahrs: reference pose (absolute quaternion)
        self._want_ref = False
        self._g_ref = None            # fixed: gravity direction at start
        self._t_start = None
        self._fix = None
        self._coil_enabled = not args.coil_off

    # -- control side --------------------------------------------------
    def stop(self):
        self._stop.set()

    def set_reference(self):
        with self._lock:
            self._want_ref = True

    def set_coil_enabled(self, on):
        """Pulse the coil (True) or hold it off (False) from the next phase."""
        with self._lock:
            self._coil_enabled = bool(on)
        if not on:
            self.localizer.cancel_capture()

    def status(self):
        with self._lock:
            return {"have_ref": self._q_ref is not None, "ahrs_fix": self._fix,
                    "coil_enabled": self._coil_enabled}

    # -- sensor thread ---------------------------------------------------
    def run(self):
        coil = None
        try:
            while not self._stop.is_set():
                imu = None
                try:
                    coil, imu = open_hardware(self.args) if coil is None else (coil, self._reopen_imu())
                    self.buf.connected, self.buf.error = True, None
                    print(f"running: coil IN1=GPIO{self.args.in1} IN2=GPIO{self.args.in2}, "
                          f"{self.args.mode}, {self.args.phase * 1e3:.0f} ms/phase, "
                          f"{self.args.orientation} frame" + (" [SIM]" if self.args.sim else ""),
                          file=sys.stderr)
                    self._loop(coil, imu)
                except OSError as e:
                    self.buf.connected, self.buf.error = False, str(e)
                    print(f"IMU error ({e}); retrying in 2 s...", file=sys.stderr)
                    if coil is not None:
                        coil.off()
                    self.demod.reset()
                    self._stop.wait(2.0)
                finally:
                    if imu is not None:
                        imu.close()
        finally:
            if coil is not None:
                coil.close()
                print("coil off", file=sys.stderr)

    def _reopen_imu(self):
        from icm20948_driver import ICM20948
        imu = ICM20948(self.args.bus, address=self.args.address)
        imu.begin(do_reset=True)
        imu.enable_magnetometer()
        return imu

    def _loop(self, coil, imu):
        a = self.args
        ahrs = a.orientation == "ahrs"
        self._t_start = self._t_start or time.monotonic()
        t0 = time.monotonic()
        wall_offset = time.time() - time.monotonic()   # monotonic -> epoch, for the chart
        k = 0
        was_enabled = None
        while not self._stop.is_set():
            with self._lock:
                enabled = self._coil_enabled
            if enabled != was_enabled:
                self.demod.reset()            # never mix pulses across an on/off toggle
                was_enabled = enabled
            state = self.pattern[k % len(self.pattern)] if enabled else 0
            t_phase = t0 + k * a.phase
            if time.monotonic() - t_phase > a.phase:      # fell behind: resync
                t0 = time.monotonic() - k * a.phase
                t_phase = t0 + k * a.phase
            coil.set(state)
            t_sw = time.monotonic()
            t_end = t_phase + a.phase
            accept_after = t_sw + a.settle + MAG_LATENCY_S

            acc = [0.0, 0.0, 0.0]
            g_acc = [0.0, 0.0, 0.0]
            raw_log = []                  # every magnetometer read this phase, for the chart
            n = n_g = 0
            t_sum = 0.0
            res = {}
            while True:
                now = time.monotonic()
                if now >= t_end or self._stop.is_set():
                    break
                r = imu.read()
                raw = r.get("mag_ut")
                fresh = raw is not None and now >= accept_after
                # the AHRS only ever sees clean (coil-off) magnetometer samples
                res = self.tracker.process(r["accel_g"], r["gyro_dps"],
                                           raw if (fresh and state == 0) else None, now)
                for i in range(3):
                    g_acc[i] += r["accel_g"][i]
                n_g += 1
                if raw is None:
                    continue
                v = mag_calibration(self.tracker).apply(raw)
                raw_log.append([round(now + wall_offset, 3), round(v[0], 2), round(v[1], 2),
                                round(v[2], 2), 1 if fresh else 0])
                if not fresh:
                    continue
                if ahrs:
                    q_ref = self._update_ref(res, now)
                    if q_ref is None:
                        continue
                    q_rel = quat_mul(quat_conj(q_ref), tuple(res["quat_abs"]))
                    v = eml.quat_rotate(q_rel, v)
                for i in range(3):
                    acc[i] += v[i]
                n += 1
                t_sum += now

            if self._stop.is_set():
                break
            mean = tuple(x / n for x in acc) if n else None
            if enabled:
                est = self.demod.push(state, t_sum / n if n else 0.0, mean, n)
                out = self.localizer.process(est["b_em"] if est else None)
            else:
                est = None
                out = {"pos": None, "b_em": None, "b_mag": None, "signal": "none",
                       "jitter_ut": None, "near_field": False, "range_cm": None,
                       "state": "coil-disabled"}

            g_mean = tuple(x / max(1, n_g) for x in g_acc)
            if self._g_ref is None and n_g:
                self._g_ref = g_mean
            tilt = _angle_deg(g_mean, self._g_ref) if n_g else None
            with self._lock:
                self._fix = res.get("fix")

            sample = {
                "t": time.time(), "phase": k, "coil": state, "n_mag": n,
                "t_phase": round(t_sw + wall_offset, 3), "raw": raw_log,
                "earth_ut": round(eml._norm(est["earth"]), 2) if est else None,
                "euler": res.get("euler"), "tilt_deg": None if tilt is None else round(tilt, 1),
                **out,
            }
            if ahrs and self._q_ref is None:
                sample["state"] = "need-ref"
            truth = getattr(imu, "truth", None)
            if truth is not None:
                sample["truth"] = [round(v, 2) for v in truth]
            self.buf.push(sample)
            k += 1

    def _update_ref(self, res, now):
        with self._lock:
            auto = (self._q_ref is None and not res.get("nulling")
                    and now - self._t_start >= AHRS_SETTLE_S)
            if self._want_ref or auto:
                self._q_ref = tuple(res["quat_abs"])
                self._want_ref = False
                self._g_ref = None
                self.demod.reset()
                self.localizer.reset_track()
            return self._q_ref


# --- `step`: measure the coil's settling time --------------------------------

def run_step(args):
    """Toggle slowly and record how the field approaches its final value after
    each switch, to choose SETTLE_S (and PHASE_S)."""
    coil, imu = open_hardware(args, still=True)
    cal = (None if args.sim else MagCalibration.load(MAG_CAL_FILE)) or MagCalibration()
    phases = []                                        # (state, [(dt, vec), ...])
    print(f"step test: {args.cycles} cycles of off/on, {STEP_PHASE_S * 1e3:.0f} ms each "
          "-- hold the board still near the magnet", flush=True)
    try:
        for _ in range(args.cycles):
            for state in (0, 1):
                coil.set(state)
                t_sw = time.monotonic()
                samples = []
                while (now := time.monotonic()) < t_sw + STEP_PHASE_S:
                    raw = imu.read().get("mag_ut")
                    if raw is not None:
                        samples.append((now - t_sw, cal.apply(raw)))
                phases.append((state, samples))
    except KeyboardInterrupt:
        print()
    finally:
        coil.close()
        imu.close()

    def tail_mean(samples):
        tail = [v for dt, v in samples if dt >= STEP_PHASE_S / 2]
        if not tail:
            return None
        return tuple(sum(v[i] for v in tail) / len(tail) for i in range(3))

    bins = {"rise": {}, "fall": {}}
    deltas, noise = [], []
    for (s_prev, prev), (s_cur, cur) in zip(phases, phases[1:]):
        before, after = tail_mean(prev), tail_mean(cur)
        if before is None or after is None:
            continue
        d = eml._sub(after, before)
        dd = eml._dot(d, d)
        if dd < 1e-9:
            continue
        deltas.append(math.sqrt(dd))
        if s_cur == 0:
            tail = [v for dt, v in cur if dt >= STEP_PHASE_S / 2]
            noise += [eml._dist(v, after) for v in tail]
        edge = bins["rise" if s_cur == 1 else "fall"]
        for dt, v in cur:
            f = eml._dot(eml._sub(v, before), d) / dd
            edge.setdefault(int(dt / STEP_BIN_S), []).append(f)

    if not deltas:
        print("no usable data (is the magnetometer working? is the coil switching?)")
        return 1
    noise_rms = (sum(x * x for x in noise) / max(1, len(noise))) ** 0.5
    step = sum(deltas) / len(deltas)
    if step < max(STEP_MIN_UT, 3.0 * noise_rms):
        print(f"\nno coil field detected: on/off step {step:.2f} uT vs noise {noise_rms:.2f} uT rms\n"
              "check: magnet supply on, L298N wiring (IN1/IN2 on the right header pins,\n"
              "shared GND, ENA jumper), volts across OUT1-OUT2 while toggling, and that\n"
              "the board is within ~10 cm of the magnet")
        return 1

    def median(vals):
        v = sorted(vals)
        return v[len(v) // 2] if len(v) % 2 else 0.5 * (v[len(v) // 2 - 1] + v[len(v) // 2])

    def settle_time(edge):
        """Earliest bin after which every bin is within tolerance of 1.0.  The
        median across cycles ignores one bumped cycle; the tolerance widens to
        3 standard errors when the step is small next to the noise."""
        t = None
        for kk in sorted(edge, reverse=True):
            vals = edge[kk]
            tol = max(STEP_TOLERANCE, 3.0 * noise_rms / step / math.sqrt(len(vals)))
            if abs(1.0 - median(vals)) > tol:
                break
            t = kk * STEP_BIN_S
        return t

    print(f"\n|B_em| at this spot: {step:.2f} uT   (per-sample noise ~{noise_rms:.2f} uT rms)")
    print("\n  t after switch   rise (off->on)   fall (on->off)    [median fraction of final step]")
    last = int(min(STEP_PHASE_S, 0.12) / STEP_BIN_S)
    for kk in range(last + 1):
        cells = []
        for name in ("rise", "fall"):
            vals = bins[name].get(kk)
            cells.append(f"{median(vals):8.3f}" if vals else "       -")
        print(f"  {kk * STEP_BIN_S * 1e3:6.0f} ms        {cells[0]}         {cells[1]}")

    ts = [settle_time(bins["rise"]), settle_time(bins["fall"])]
    if None in ts:
        print("\nthe field never settled within tolerance -- try a longer STEP_PHASE_S")
        return 1
    t98 = max(ts)
    settle = max(0.0, t98 - MAG_LATENCY_S) + 0.005
    phase = settle + MAG_LATENCY_S + 4 / 100.0
    print(f"\nsettled to {STEP_TOLERANCE:.0%} by ~{t98 * 1e3:.0f} ms after a switch (read time)")
    print(f"suggested: SETTLE_S = {settle:.3f}   PHASE_S >= {phase:.3f}  (>= 4 samples per phase)")
    return 0


# --- HTTP ---------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):
    server_version = "EMLocate/1.0"

    def log_message(self, fmt, *args):
        pass

    def _state(self):
        s = self.server
        tr = s.tracker.status()
        return {
            **s.buf.status(), **s.localizer.status(), **s.engine.status(),
            "mag_cal": tr["have_cal"], "mag_calibrating": tr["calibrating"],
            "mag_cal_samples": tr["cal_samples"], "mag_cal_span": tr["cal_span"],
            "nulling_gyro": tr["nulling_gyro"],
        }

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(self.server.html_path.read_bytes(), "text/html; charset=utf-8")
        elif self.path == "/meta":
            self._json(self.server.meta)
        elif self.path == "/state":
            self._json(self._state())
        elif self.path == "/stream":
            self._stream()
        else:
            self.send_error(404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}") if n else {}
        except ValueError:
            body = {}
        s = self.server
        loc, tr = s.localizer, s.tracker
        actions = {
            "/cal/capture": lambda: loc.capture_point(body.get("p")),
            "/cal/cancel": loc.cancel_capture,
            "/cal/undo": loc.undo_point,
            "/cal/clear": loc.clear_points,
            "/flip": loc.flip,
            "/track/reset": loc.reset_track,
            "/ref/set": s.engine.set_reference,
            "/coil/on": lambda: s.engine.set_coil_enabled(True),
            "/coil/off": lambda: s.engine.set_coil_enabled(False),
            "/null-gyro": tr.null_gyro,
            "/magcal/start": tr.calibrate_start,
            "/magcal/cancel": tr.calibrate_cancel,
            "/magcal/finish": tr.calibrate_finish,
        }
        fn = actions.get(self.path)
        if fn is None:
            self.send_error(404)
            return
        try:
            result = fn()
        except ValueError as e:
            self._json({**self._state(), "ok": False, "error": str(e)})
            return
        payload = {"ok": True, **self._state()}
        if hasattr(result, "as_dict"):
            payload["result"] = result.as_dict()
        self._json(payload)

    def _send(self, data, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _json(self, obj):
        self._send(json.dumps(obj).encode(), "application/json")

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        last = 0
        try:
            while True:
                for smp in self.server.buf.since(last):
                    last = smp["seq"]
                    self.wfile.write(f"data: {json.dumps(smp)}\n\n".encode())
                self.wfile.write(f"event: status\ndata: {json.dumps(self._state())}\n\n".encode())
                self.wfile.flush()
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError):
            pass


def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def run_web(args):
    html_path = HERE / "em_view.html"
    if not html_path.exists():
        sys.exit(f"missing {html_path}")
    mag_cal_file = SIM_MAG_CAL_FILE if args.sim else MAG_CAL_FILE
    if not args.sim and not MAG_CAL_FILE.exists() and MAG_CAL_FALLBACK.exists():
        shutil.copy(MAG_CAL_FALLBACK, MAG_CAL_FILE)
        print(f"copied magnetometer calibration from {MAG_CAL_FALLBACK}", file=sys.stderr)

    tracker = OrientationTracker(beta=AHRS_BETA, cal_path=str(mag_cal_file))
    cal_file = SIM_CAL_FILE if args.sim else CAL_FILE
    localizer = eml.Localizer(cal_path=cal_file, frame=args.orientation)
    if localizer.status()["frame_mismatch"]:
        print(f"warning: {cal_file.name} was captured in the '{localizer.cal.frame}' frame, "
              f"now running '{args.orientation}' -- recalibrate", file=sys.stderr)
    buf = SampleBuffer()
    engine = Engine(args, tracker, localizer, buf)
    # bind first: if the port is taken we exit before the coil ever switches
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    server.buf, server.tracker, server.localizer, server.engine = buf, tracker, localizer, engine
    server.html_path = html_path
    server.meta = {
        "sim": args.sim, "mode": args.mode, "phase_s": args.phase, "settle_s": args.settle,
        "orientation": args.orientation, "cube_half_cm": CUBE_HALF_CM,
        "near_field_cm": eml.NEAR_FIELD_CM, "in1": args.in1, "in2": args.in2,
        "tilt_warn_deg": TILT_WARN_DEG, "raw_chart_s": RAW_CHART_S,
    }

    th = threading.Thread(target=engine.run, daemon=True)
    th.start()

    ip = local_ip() if args.host in ("0.0.0.0", "::") else args.host
    print(f"EM localizer:  http://{ip}:{args.port}   (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        engine.stop()
        th.join(timeout=3.0)
        server.server_close()
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", nargs="?", default="run", choices=["run", "step"])
    p.add_argument("--sim", action="store_true", help="simulated coil + sensor, no hardware")
    p.add_argument("--in1", type=int, default=IN1_PIN, help=f"BCM GPIO for IN1 (default {IN1_PIN})")
    p.add_argument("--in2", type=int, default=IN2_PIN, help=f"BCM GPIO for IN2 (default {IN2_PIN})")
    p.add_argument("-b", "--bus", type=int, default=I2C_BUS)
    p.add_argument("-a", "--address", type=lambda x: int(x, 0), default=I2C_ADDRESS)
    p.add_argument("-m", "--mode", choices=sorted(PATTERNS), default=DRIVE_MODE)
    p.add_argument("--phase", type=float, default=PHASE_S, help=f"s per phase (default {PHASE_S})")
    p.add_argument("--settle", type=float, default=SETTLE_S, help=f"s skipped after a switch (default {SETTLE_S})")
    p.add_argument("-o", "--orientation", choices=["fixed", "ahrs"], default=ORIENTATION_MODE)
    p.add_argument("--cycles", type=int, default=STEP_CYCLES, help="step: on/off cycles")
    p.add_argument("--coil-off", action="store_true", default=not COIL_ON_AT_START,
                   help="start with the coil held off (turn it on from the page)")
    p.add_argument("--host", default=HTTP_HOST)
    p.add_argument("--port", type=int, default=HTTP_PORT)
    args = p.parse_args(argv)

    if args.settle + MAG_LATENCY_S >= args.phase:
        p.error(f"--phase must exceed --settle + {MAG_LATENCY_S} s, or no sample is ever used")

    def _on_sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)

    return run_step(args) if args.command == "step" else run_web(args)


if __name__ == "__main__":
    sys.exit(main())
