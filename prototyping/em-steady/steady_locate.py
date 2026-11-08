#!/usr/bin/env python3
"""Locate an ICM-20948 in 3-D relative to an *always-on* electromagnet.

Unlike ``../em-localizer`` (which pulses the coil and subtracts coil-off from
coil-on), the coil here stays on.  Earth's field is learned once, as a fixed
vector in a reference frame, by tumbling the board with the coil off.  From
then on every magnetometer sample is rotated into that frame with a
gyro + accelerometer orientation estimate and Earth's field is subtracted:

    B_em = R(q) * b_measured - E_ref

``B_em`` goes through the same calibrated dipole inversion as em-localizer.

Commands:
  ./steady_locate.py               # run: http://<this-pi's-ip>:8000
  ./steady_locate.py --sim         # same, with a simulated coil + rotating sensor
  ./steady_locate.py --no-coil     # EM powered separately: never touch the GPIOs

Reference frame: the board's axes while it sits squared on a fixed *mark* on the
jig.  It is taken automatically at startup (so start with the board on the
mark, still), and again with "Re-zero on mark".  Yaw comes from the gyro alone
and drifts slowly; putting the board back on the mark and re-zeroing removes
the drift.  Earth's field, stored in this frame, stays valid across runs as
long as the jig doesn't move.

Once per setup, from the page:
  1. "Earth field...": Start (the coil turns off), tumble the board through
     every orientation for ~30 s, Finish, put it on the mark, Capture on mark.
     Saved to earth_cal.json; the raw tumble goes to earth_session.csv.
  2. "Calibrate EM...": hold the board at known (x, y, z) offsets from the
     magnet's centre, in the reference axes, and capture.  Saved to em_cal.json.
"""

from __future__ import annotations

import argparse
import json
import signal
import socket
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import earth_model as erm   # also puts ../em-localizer and ../icm20948-visualizer on sys.path
import em_localize as eml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "em-driver"))

from em_toggle import IN1_PIN as DRIVER_IN1_PIN, IN2_PIN as DRIVER_IN2_PIN  # noqa: E402
from imu_ahrs import quat_to_euler  # noqa: E402


# --- configuration (adjust freely) -------------------------------------------

# wiring -- defaults come from ../em-driver/em_toggle.py; override here if needed
IN1_PIN = DRIVER_IN1_PIN       # BCM GPIO -> L298N IN1
IN2_PIN = DRIVER_IN2_PIN       # BCM GPIO -> L298N IN2
I2C_BUS = 1
I2C_ADDRESS = 0x68

# coil
COIL_CONTROL = True            # False: the EM is powered separately (same as --no-coil)
COIL_STATE = 1                 # +1 forward or -1 reversed while "on"
COIL_SETTLE_S = 0.2            # after the coil switches, ignore samples this long

# sensing
MAG_LATENCY_S = 0.012          # AK09916 sample age when read (100 Hz ODR + conversion)
BLOCK_SAMPLES = 10             # magnetometer samples averaged per position update (10 Hz)
AUTO_REFERENCE = True          # take the reference pose once the startup gyro null ends

# web view
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8000
CUBE_HALF_CM = 15.0            # drawn working volume: +/- this on every axis
CHART_S = 5.0                  # seconds of B_em history in the page's chart

# files
EARTH_FILE = HERE / "earth_cal.json"
CAL_FILE = HERE / "em_cal.json"
SIM_EARTH_FILE = HERE / "earth_cal_sim.json"   # --sim never touches the real calibrations
SIM_CAL_FILE = HERE / "em_cal_sim.json"
SESSION_FILE = HERE / "earth_session.csv"      # raw samples of the latest tumble

COIL_ON_AT_START = True        # False: start with the coil held off (turn on from the page)


# --- hardware --------------------------------------------------------------

class NoCoil:
    """Stand-in when the electromagnet is powered independently of the Pi."""
    def set(self, state): pass
    def off(self): pass
    def close(self): pass


def open_coil(args):
    if args.sim:
        from steady_sim import SimCoil
        return SimCoil()
    if args.no_coil:
        return NoCoil()
    from em_toggle import Coil
    return Coil(args.in1, args.in2)


def open_imu(args, coil):
    if args.sim:
        from steady_sim import SimIMU
        imu = SimIMU(coil)
    else:
        from icm20948_driver import ICM20948
        imu = ICM20948(args.bus, address=args.address)
        imu.begin(do_reset=True)
    imu.enable_magnetometer()
    return imu


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
    """Owns the coil and the IMU: holds the coil on, tracks orientation, and
    turns every magnetometer sample into ``B_em`` for :mod:`em_localize`."""

    def __init__(self, args, attitude, earth, earth_path, session_path, localizer, buf):
        self.args = args
        self.att = attitude
        self.earth = earth                 # EarthField or None
        self.earth_path = earth_path
        self.session_path = session_path
        self.localizer = localizer
        self.buf = buf
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._coil_enabled = not args.coil_off
        self._phase = None                 # Earth calibration: "collect", "need-mark", "mark"
        self._collector = None             # EarthCollector while the tumble runs
        self._iron = None                  # (cal, report) from the tumble, awaiting the mark
        self._mark = []                    # reference-frame vectors captured on the mark
        self._earth_error = None
        self.imu_errors = 0
        if AUTO_REFERENCE:
            attitude.set_reference()

    # -- control side --------------------------------------------------
    def stop(self):
        self._stop.set()

    def set_coil_enabled(self, on):
        if self.args.no_coil:
            raise ValueError("the coil is not under this Pi's control (--no-coil)")
        with self._lock:
            self._coil_enabled = bool(on)
        if not on:
            self.localizer.cancel_capture()

    def set_reference(self):
        with self._lock:
            if self._phase is not None:
                raise ValueError("finish or cancel the Earth calibration first")
        self.att.null_gyro()               # the board is still on its mark: refresh the bias too
        self.att.set_reference()
        self.localizer.reset_track()

    def earth_start(self):
        if not self.att.status()["have_ref"]:
            raise ValueError("no reference pose yet -- put the board on its mark and re-zero")
        with self._lock:
            self._phase, self._collector = "collect", erm.EarthCollector()
            self._iron, self._mark, self._earth_error = None, [], None
        self.localizer.cancel_capture()

    def earth_cancel(self):
        with self._lock:
            self._phase, self._collector, self._iron = None, None, None
            self._mark = []

    def earth_finish(self):
        """End the tumble: fit hard/soft iron, then wait for the board on its mark."""
        with self._lock:
            coll = self._collector
            if self._phase != "collect":
                raise ValueError("no tumble in progress")
        coll.save_csv(self.session_path)
        try:
            iron = coll.result()               # may raise ValueError; collection continues
        except ValueError as e:
            with self._lock:
                self._earth_error = str(e)
            raise
        with self._lock:
            self._iron, self._collector, self._phase = iron, None, "need-mark"
            self._earth_error = None

    def earth_mark(self):
        """Measure Earth's field with the board still on its mark.  Also works on
        its own, with an existing calibration, to refresh ``E`` only."""
        with self._lock:
            if self._phase == "collect":
                raise ValueError("finish the tumble first")
            if self._iron is None and self.earth is None:
                raise ValueError("no iron calibration yet -- do the tumble first")
        self.localizer.cancel_capture()
        self.att.null_gyro()
        self.att.set_reference()           # before the phase: no sample may use the old reference
        with self._lock:
            self._phase, self._mark, self._earth_error = "mark", [], None

    def _mark_sample(self, raw, q):
        with self._lock:
            cal = self._iron[0] if self._iron else self.earth.cal
            self._mark.append(erm.quat_rotate(q, cal.apply(raw)))
            if len(self._mark) < erm.EARTH_MARK_SAMPLES:
                return
            vs = self._mark
            n = len(vs)
            E = tuple(sum(v[i] for v in vs) / n for i in range(3))
            motion = (sum(eml._dot(eml._sub(v, E), eml._sub(v, E)) for v in vs) / n) ** 0.5
            self._mark = []
            if motion > erm.EARTH_MARK_MAX_UT:
                self._phase = "need-mark"
                self._earth_error = (f"the field moved {motion:.1f} uT rms on the mark; "
                                     "keep the board still and capture again")
                return
            report = self._iron[1] if self._iron else self.earth.report
            self.earth = erm.EarthField(cal, E, {**report, "mark_rms_ut": round(motion, 3)})
            self._iron, self._phase = None, None
            self.earth.save(self.earth_path)
        self.localizer.reset_track()

    def status(self):
        with self._lock:
            coll = self._collector
            ef = self.earth
            return {
                "coil_enabled": self._coil_enabled,
                "coil_control": not self.args.no_coil,
                "have_earth": ef is not None,
                "earth": ef.as_dict() if ef is not None else None,
                "earth_phase": self._phase,
                "earth_n": coll.n if coll else 0,
                "earth_coverage": round(coll.coverage(), 3) if coll else None,
                "earth_min_n": erm.EARTH_MIN_SAMPLES,
                "earth_min_coverage": erm.EARTH_MIN_COVERAGE,
                "earth_mark_n": len(self._mark),
                "earth_mark_target": erm.EARTH_MARK_SAMPLES,
                "earth_iron": self._iron[1] if self._iron else None,
                "earth_error": self._earth_error,
                "imu_errors": self.imu_errors,
            }

    # -- sensor thread ---------------------------------------------------
    def run(self):
        coil = open_coil(self.args)
        try:
            while not self._stop.is_set():
                imu = None
                try:
                    imu = open_imu(self.args, coil)
                    self.buf.connected, self.buf.error = True, None
                    print("running: " + ("coil not controlled" if self.args.no_coil else
                                         f"coil IN1=GPIO{self.args.in1} IN2=GPIO{self.args.in2}")
                          + (" [SIM]" if self.args.sim else ""), file=sys.stderr)
                    self._loop(coil, imu)
                except OSError as e:
                    self.buf.connected, self.buf.error = False, str(e)
                    self.imu_errors += 1
                    print(f"IMU error ({e}); retrying in 2 s...", file=sys.stderr)
                    coil.off()
                    self._stop.wait(2.0)
                finally:
                    if imu is not None:
                        imu.close()
        finally:
            coil.close()
            if not self.args.no_coil:
                print("coil off", file=sys.stderr)

    def _loop(self, coil, imu):
        wall_offset = time.time() - time.monotonic()      # monotonic -> epoch, for the chart
        coil_state = None
        accept_after = 0.0
        block, chart = [], []
        tot = [0.0, 0.0, 0.0]
        sim_phase = None
        while not self._stop.is_set():
            with self._lock:
                coll = self._collector
                phase = self._phase
                enabled = self._coil_enabled and phase is None   # off throughout the Earth calibration
                ef = self.earth
            want = COIL_STATE if enabled else 0
            if want != coil_state:
                coil.set(want)
                coil_state = want
                accept_after = time.monotonic() + COIL_SETTLE_S + MAG_LATENCY_S
                block, tot = [], [0.0, 0.0, 0.0]
            if phase != sim_phase and hasattr(imu, "set_phase"):
                sim_phase = phase
                imu.set_phase(phase)                     # --sim: tumble, or return to the mark

            r = imu.read()
            now = time.monotonic()
            self.att.update(r["accel_g"], r["gyro_dps"], now)
            raw = r.get("mag_ut")
            if raw is None or now < accept_after:
                continue
            q = self.att.q_rel_at(now - MAG_LATENCY_S)
            if q is None:
                continue                                  # no reference pose yet

            if coll is not None and (coil_state == 0 or self.args.no_coil):
                coll.add(now, raw, r["accel_g"], r["gyro_dps"], q)
            if phase == "mark" and not self.att.status(now)["ref_pending"]:
                self._mark_sample(raw, q)
            if ef is None:
                self._push_idle(q, "earth-cal" if phase else "need-earth")
                continue

            b = ef.subtract(raw, q)
            block.append(b)
            chart.append([round(now + wall_offset, 3)] + [round(v, 2) for v in b])
            meas = erm.quat_rotate(q, ef.cal.apply(raw))
            for i in range(3):
                tot[i] += meas[i]
            if len(block) < BLOCK_SAMPLES:
                continue

            n = len(block)
            mean = tuple(sum(v[i] for v in block) / n for i in range(3))
            total = tuple(v / n for v in tot)
            block, tot = [], [0.0, 0.0, 0.0]
            if phase is not None:
                out = {"pos": None, "b_em": [round(v, 3) for v in mean],
                       "b_mag": round(eml._norm(mean), 3), "signal": "none",
                       "state": "earth-cal"}
            elif coil_state == 0 and not self.args.no_coil:
                # coil held off: B_em should read ~0 -- a live check of the Earth cancellation
                out = {"pos": None, "b_em": [round(v, 3) for v in mean],
                       "b_mag": round(eml._norm(mean), 3), "signal": "none",
                       "state": "coil-disabled"}
            else:
                out = self.localizer.process(mean)
            self._push(q, out, total, chart, ef, imu, coil_state)
            chart = []

    def _push(self, q, out, total, chart, ef, imu, coil_state):
        yaw, pitch, roll = quat_to_euler(q)
        sample = {
            "t": time.time(), "coil": coil_state, "raw": chart,
            "quat": [round(v, 4) for v in q],
            "euler": [round(yaw, 1), round(pitch, 1), round(roll, 1)],
            "total_ut": round(eml._norm(total), 2),
            "earth_ut": round(eml._norm(ef.earth), 2),
            **out,
        }
        truth = getattr(imu, "truth", None)
        if truth is not None:
            sample["truth"] = [round(v, 2) for v in truth]
        self.buf.push(sample)

    _last_idle = 0.0

    def _push_idle(self, q, state):
        """Without an Earth calibration there is no B_em; still stream the pose
        (10 Hz) so the page shows the board is alive."""
        now = time.monotonic()
        if now - self._last_idle < 0.1:
            return
        self._last_idle = now
        yaw, pitch, roll = quat_to_euler(q)
        self.buf.push({"t": time.time(), "state": state, "pos": None, "b_em": None,
                       "b_mag": None, "signal": "none", "raw": [],
                       "quat": [round(v, 4) for v in q],
                       "euler": [round(yaw, 1), round(pitch, 1), round(roll, 1)]})


# --- HTTP ---------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):
    server_version = "EMSteady/1.0"

    def log_message(self, fmt, *args):
        pass

    def _state(self):
        s = self.server
        return {**s.buf.status(), **s.localizer.status(), **s.engine.status(),
                **s.attitude.status()}

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
        loc, eng = s.localizer, s.engine
        actions = {
            "/cal/capture": lambda: loc.capture_point(body.get("p")),
            "/cal/cancel": loc.cancel_capture,
            "/cal/undo": loc.undo_point,
            "/cal/clear": loc.clear_points,
            "/flip": loc.flip,
            "/track/reset": loc.reset_track,
            "/ref/set": eng.set_reference,
            "/null-gyro": s.attitude.null_gyro,
            "/coil/on": lambda: eng.set_coil_enabled(True),
            "/coil/off": lambda: eng.set_coil_enabled(False),
            "/earth/start": eng.earth_start,
            "/earth/cancel": eng.earth_cancel,
            "/earth/finish": eng.earth_finish,
            "/earth/mark": eng.earth_mark,
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
    html_path = HERE / "steady_view.html"
    if not html_path.exists():
        sys.exit(f"missing {html_path}")
    earth_path = SIM_EARTH_FILE if args.sim else EARTH_FILE
    cal_path = SIM_CAL_FILE if args.sim else CAL_FILE

    attitude = erm.Attitude()
    earth = erm.EarthField.load(earth_path)
    localizer = eml.Localizer(cal_path=cal_path, frame="reference")
    if localizer.status()["frame_mismatch"]:
        print(f"warning: {cal_path.name} was captured in the '{localizer.cal.frame}' frame -- recalibrate",
              file=sys.stderr)
    buf = SampleBuffer()
    if earth is None and earth_path.exists():
        print(f"ignoring {earth_path.name}: redo the Earth calibration", file=sys.stderr)
    session_path = SESSION_FILE.with_name("earth_session_sim.csv") if args.sim else SESSION_FILE
    engine = Engine(args, attitude, earth, earth_path, session_path, localizer, buf)
    # bind first: if the port is taken we exit before the coil ever switches
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    server.buf, server.localizer, server.engine, server.attitude = buf, localizer, engine, attitude
    server.html_path = html_path
    server.meta = {
        "sim": args.sim, "no_coil": args.no_coil, "cube_half_cm": CUBE_HALF_CM,
        "near_field_cm": eml.NEAR_FIELD_CM, "in1": args.in1, "in2": args.in2,
        "chart_s": CHART_S, "block_samples": BLOCK_SAMPLES,
    }

    th = threading.Thread(target=engine.run, daemon=True)
    th.start()

    ip = local_ip() if args.host in ("0.0.0.0", "::") else args.host
    print(f"EM steady localizer:  http://{ip}:{args.port}   (Ctrl+C to stop)")
    print("keep the board still on its mark for the first second (gyro null + reference pose)")
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
    p.add_argument("--sim", action="store_true", help="simulated coil + rotating sensor, no hardware")
    p.add_argument("--no-coil", action="store_true", default=not COIL_CONTROL,
                   help="the EM is powered separately; never drive the L298N GPIOs")
    p.add_argument("--in1", type=int, default=IN1_PIN, help=f"BCM GPIO for IN1 (default {IN1_PIN})")
    p.add_argument("--in2", type=int, default=IN2_PIN, help=f"BCM GPIO for IN2 (default {IN2_PIN})")
    p.add_argument("-b", "--bus", type=int, default=I2C_BUS)
    p.add_argument("-a", "--address", type=lambda x: int(x, 0), default=I2C_ADDRESS)
    p.add_argument("--coil-off", action="store_true", default=not COIL_ON_AT_START,
                   help="start with the coil held off (turn it on from the page)")
    p.add_argument("--host", default=HTTP_HOST)
    p.add_argument("--port", type=int, default=HTTP_PORT)
    args = p.parse_args(argv)
    if args.sim:
        args.no_coil = False

    def _on_sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)

    return run_web(args)


if __name__ == "__main__":
    sys.exit(main())
