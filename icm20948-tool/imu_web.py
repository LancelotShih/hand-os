#!/usr/bin/env python3
"""Serve a live browser dashboard for an ICM-20948 IMU over I2C.

Runs a small local web server: a background thread streams accel / gyro /
temp (and optionally magnetometer) samples from the sensor, and any browser
on the network can watch them update live at /.

Example:
  ./imu_web.py -b 1                 # http://<this-pi's-ip>:8000
  ./imu_web.py -b 1 --mag -r 40
  ./imu_web.py -b 1 --port 8080 -a 0x69
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

from icm20948_driver import ICM20948
from icm20948_registers import ACCEL_FS, GYRO_FS

try:
    from smbus2 import SMBus
except ImportError:
    sys.exit("smbus2 is not installed:  pip install smbus2")


# --- shared state between the reader thread and any number of browser tabs --

class SampleBuffer:
    """Thread-safe ring buffer of recent samples, each tagged with an
    increasing sequence number so each SSE client can resume from wherever
    it last read, independent of every other client."""

    def __init__(self, maxlen=2000):
        self._buf = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._seq = 0
        self.connected = False
        self.error = None

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
            return {"connected": self.connected, "error": self.error}


def reader_thread(buf: SampleBuffer, args):
    """Owns the sensor. Reconnects on I2C errors (e.g. the board gets
    unplugged) instead of dying, so the dashboard can show 'disconnected'
    and recover automatically once the sensor comes back."""
    period = 1.0 / args.rate if args.rate > 0 else 0.0
    while True:
        bus = None
        try:
            bus = SMBus(args.bus)
            dev = ICM20948(bus, address=args.address,
                            accel_range=args.accel_range, gyro_range=args.gyro_range)
            dev.begin(do_reset=True)
            if args.mag:
                try:
                    dev.enable_magnetometer()
                except (OSError, RuntimeError) as e:
                    print(f"magnetometer init failed, continuing without it: {e}", file=sys.stderr)

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
                if dev.mag_enabled:
                    m = r.get("mag_ut")
                    if m is not None:
                        sample["mag"] = list(m)
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
    server_version = "ICM20948Dashboard/1.0"

    def log_message(self, fmt, *args):
        pass  # the reader thread already logs connection state; keep stdout quiet

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_file(self.server.html_path, "text/html; charset=utf-8")
        elif self.path == "/meta":
            self._serve_json(self.server.meta)
        elif self.path == "/stream":
            self._serve_stream()
        else:
            self.send_error(404)

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
                status = json.dumps(buf.status())
                self.wfile.write(f"event: status\ndata: {status}\n\n".encode())
                self.wfile.flush()
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError):
            pass


def local_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))  # no packet actually sent; just picks the outbound iface
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
    p.add_argument("--host", default="0.0.0.0", help="bind address (default: 0.0.0.0, all interfaces)")
    p.add_argument("--port", type=int, default=8000, help="HTTP port (default: 8000)")
    p.add_argument("-r", "--rate", type=float, default=30.0, help="samples per second (default: 30)")
    p.add_argument("--accel-range", type=int, default=4, choices=sorted(ACCEL_FS))
    p.add_argument("--gyro-range", type=int, default=500, choices=sorted(GYRO_FS))
    p.add_argument("--mag", action="store_true", help="also stream the AK09916 magnetometer")
    args = p.parse_args(argv)

    html_path = Path(__file__).with_name("imu_dashboard.html")
    if not html_path.exists():
        sys.exit(f"missing {html_path} (expected next to imu_web.py)")

    buf = SampleBuffer()
    threading.Thread(target=reader_thread, args=(buf, args), daemon=True).start()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    server.buf = buf
    server.html_path = html_path
    server.meta = {
        "bus": args.bus, "address": args.address, "rate": args.rate,
        "accel_range": args.accel_range, "gyro_range": args.gyro_range,
        "mag": args.mag,
    }

    ip = local_ip() if args.host in ("0.0.0.0", "::") else args.host
    print(f"ICM-20948 dashboard: http://{ip}:{args.port}  (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        server.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
