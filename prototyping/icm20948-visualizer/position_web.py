#!/usr/bin/env python3
"""Serve a live 3-D view of where an ICM-20948 IMU is, relative to one nearby
DC electromagnet.

A background thread owns the sensor: it runs the same Madgwick AHRS fusion as
``../icm20948-tool`` for orientation, then feeds the calibrated magnetometer
vector and that orientation into :mod:`mag_localize`, which subtracts the
ambient field and inverts the dipole equation for the IMU's position around the
magnet.  Any browser on the network watches it update live at ``/``.

One-time, from the page:
  1. Null gyro (also automatic at startup) -- board still.
  2. Calibrate magnetometer -- slow tumble through every orientation, *away*
     from the electromagnet.  Writes imu_mag_cal.json next to this script.
  3. Zero field -- electromagnet OFF, board still.
  4. Calibrate magnet -- electromagnet ON, board held on its axis at the set
     distance.  Writes magpos_cal.json.
  5. Set start -- wherever you want the movement readout measured from.

Example:
  ./position_web.py -b 1                 # http://<this-pi's-ip>:8000
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from imu_ahrs import OrientationTracker
from mag_localize import MagLocalizer
from icm20948_driver import ICM20948
from icm20948_registers import ACCEL_FS, GYRO_FS

try:
    from smbus2 import SMBus
except ImportError:
    sys.exit("smbus2 is not installed:  pip install smbus2")


# --- shared state between the reader thread and any number of browser tabs --

class SampleBuffer:
    """Thread-safe ring buffer of recent samples, each tagged with an
    increasing sequence number so each SSE client can resume independently."""

    def __init__(self, maxlen=2000):
        self._buf = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._seq = 0
        self.connected = False
        self.error = None
        self.mag_present = False

    def push(self, sample: dict):
        with self._lock:
            self._seq += 1
            sample["seq"] = self._seq
            self._buf.append(sample)

    def since(self, seq: int):
        with self._lock:
            return [s for s in self._buf if s["seq"] > seq]

    def status(self):
        with self._lock:
            return {"connected": self.connected, "error": self.error,
                    "mag_present": self.mag_present}


def reader_thread(buf: SampleBuffer, tracker: OrientationTracker,
                  localizer: MagLocalizer, args):
    """Owns the sensor; reconnects on I2C errors instead of dying."""
    period = 1.0 / args.rate if args.rate > 0 else 0.0
    while True:
        bus = None
        try:
            bus = SMBus(args.bus)
            dev = ICM20948(bus, address=args.address,
                           accel_range=args.accel_range, gyro_range=args.gyro_range)
            dev.begin(do_reset=True)
            try:
                dev.enable_magnetometer()
            except (OSError, RuntimeError) as e:
                print(f"magnetometer init failed: {e}", file=sys.stderr)
            buf.mag_present = dev.mag_enabled

            buf.connected = True
            buf.error = None
            print(f"IMU connected on i2c-{args.bus} @ 0x{args.address:02X}", file=sys.stderr)

            while True:
                t0 = time.monotonic()
                r = dev.read()
                ax, ay, az = r["accel_g"]
                gx, gy, gz = r["gyro_dps"]
                sample = {
                    "t": time.time(),
                    "accel": [ax, ay, az],
                    "gyro": [gx, gy, gz],
                    "temp": r["temp_c"],
                }
                mag = None
                if dev.mag_enabled:
                    m = r.get("mag_ut")
                    if m is not None:
                        mag = list(m)
                        sample["mag"] = mag

                res = tracker.process(r["accel_g"], r["gyro_dps"], mag, t0)
                sample.update(res)
                sample.update(localizer.process(res.get("mbody"),
                                                res.get("quat_abs"), t0))
                buf.push(sample)

                if period:
                    dt = period - (time.monotonic() - t0)
                    if dt > 0:
                        time.sleep(dt)
        except (OSError, RuntimeError) as e:
            buf.connected = False
            buf.error = str(e)
            print(f"IMU read error ({e}); retrying in 2s...", file=sys.stderr)
            time.sleep(2.0)
        finally:
            if bus is not None:
                try:
                    bus.close()
                except Exception:
                    pass


# --- HTTP server --------------------------------------------------------

class Handler(BaseHTTPRequestHandler):
    server_version = "ICM20948Visualizer/1.0"

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_file(self.server.html_path, "text/html; charset=utf-8")
        elif self.path == "/meta":
            self._serve_json(self.server.meta)
        elif self.path == "/state":
            self._serve_json({**self.server.tracker.status(),
                              **self.server.localizer.status()})
        elif self.path == "/stream":
            self._serve_stream()
        else:
            self.send_error(404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0) or 0)
        body = self.rfile.read(length) if length else b""
        tr: OrientationTracker = self.server.tracker
        loc: MagLocalizer = self.server.localizer

        if self.path == "/coil/calibrate":
            try:
                d = json.loads(body or b"{}").get("distance_cm")
            except ValueError:
                d = None
            fn = lambda: loc.calibrate_coil(d)
        else:
            actions = {
                "/null-gyro": tr.null_gyro,
                "/calibrate/start": tr.calibrate_start,
                "/calibrate/cancel": tr.calibrate_cancel,
                "/calibrate/finish": tr.calibrate_finish,
                "/field-zero/start": loc.zero_field_start,
                "/field-zero/cancel": loc.zero_field_cancel,
                "/start/set": loc.set_start,
                "/start/clear": loc.clear_start,
                "/side/flip": loc.flip_side,
            }
            fn = actions.get(self.path)
            if fn is None:
                self.send_error(404)
                return

        state = lambda: {**tr.status(), **loc.status()}
        try:
            result = fn()
        except ValueError as e:
            self._serve_json({"ok": False, "error": str(e), **state()})
            return
        payload = {"ok": True, **state()}
        if hasattr(result, "as_dict"):
            payload["cal"] = result.as_dict()
        self._serve_json(payload)

    def _serve_file(self, path: Path, content_type: str):
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_json(self, obj):
        data = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        buf: SampleBuffer = self.server.buf
        last_seq = 0
        try:
            while True:
                for s in buf.since(last_seq):
                    last_seq = s["seq"]
                    self.wfile.write(f"data: {json.dumps(s)}\n\n".encode())
                status = json.dumps({**buf.status(), **self.server.tracker.status(),
                                     **self.server.localizer.status()})
                self.wfile.write(f"event: status\ndata: {status}\n\n".encode())
                self.wfile.flush()
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError):
            pass


def local_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-b", "--bus", type=int, default=1, help="I2C bus number (default: 1)")
    p.add_argument("-a", "--address", type=lambda x: int(x, 0), default=0x68,
                   help="ICM-20948 address (default: 0x68)")
    p.add_argument("--host", default="0.0.0.0", help="bind address (default: 0.0.0.0)")
    p.add_argument("--port", type=int, default=8000, help="HTTP port (default: 8000)")
    p.add_argument("-r", "--rate", type=float, default=30.0, help="samples per second (default: 30)")
    p.add_argument("--accel-range", type=int, default=4, choices=sorted(ACCEL_FS))
    p.add_argument("--gyro-range", type=int, default=500, choices=sorted(GYRO_FS))
    p.add_argument("--beta", type=float, default=0.1, help="Madgwick filter gain (default: 0.1)")
    args = p.parse_args(argv)

    html_path = Path(__file__).with_name("position_view.html")
    if not html_path.exists():
        sys.exit(f"missing {html_path} (expected next to position_web.py)")

    tracker = OrientationTracker(
        beta=args.beta, cal_path=str(Path(__file__).with_name("imu_mag_cal.json")))
    localizer = MagLocalizer(cal_path=str(Path(__file__).with_name("magpos_cal.json")))

    buf = SampleBuffer()
    threading.Thread(target=reader_thread,
                     args=(buf, tracker, localizer, args), daemon=True).start()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    server.buf = buf
    server.tracker = tracker
    server.localizer = localizer
    server.html_path = html_path
    server.meta = {
        "bus": args.bus, "address": args.address, "rate": args.rate,
        "accel_range": args.accel_range, "gyro_range": args.gyro_range,
        "beta": args.beta,
    }

    ip = local_ip() if args.host in ("0.0.0.0", "::") else args.host
    print(f"ICM-20948 position view:  http://{ip}:{args.port}   (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        server.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
