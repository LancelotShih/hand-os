"""The web side: serves the page, streams samples, takes button clicks.

Used by: earth_cancel.py.  Uses: config, and the Pair, Feed and DataLog it's given; serves web/page/.
"""

import json
import re
import socket
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from magtrack_src.config import PAGE_DIR, PASS_MIN_TURN_DEG, PASS_UT, TUMBLE_MIN_COVERAGE

PAGE_FILES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
    "/draw.js": ("draw.js", "text/javascript; charset=utf-8"),
    "/sides.js": ("sides.js", "text/javascript; charset=utf-8"),
    "/board_view.js": ("board_view.js", "text/javascript; charset=utf-8"),
    "/sweep_view.js": ("sweep_view.js", "text/javascript; charset=utf-8"),
    "/chart.js": ("chart.js", "text/javascript; charset=utf-8"),
    "/field_view.js": ("field_view.js", "text/javascript; charset=utf-8"),
    "/position_view.js": ("position_view.js", "text/javascript; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/info.js": ("info.js", "text/javascript; charset=utf-8"),
}
LOG_FILES = {
    "plot.html": "text/html; charset=utf-8",
    "samples.csv": "text/csv; charset=utf-8",
    "raw.csv": "text/csv; charset=utf-8",
    "info.json": "application/json",
}
LOG_PATH = re.compile(r"/logs/([0-9_-]+)/([a-z.]+)")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _state(self):
        s = self.server
        return {**s.tracker.status(), "connected": s.feed.connected, "error": s.feed.error,
                "pass_ut": PASS_UT, "pass_min_turn": PASS_MIN_TURN_DEG,
                "tumble_min_coverage": TUMBLE_MIN_COVERAGE, "log": s.log.status()}

    def do_GET(self):
        if self.path in PAGE_FILES:
            name, ctype = PAGE_FILES[self.path]
            self._send((PAGE_DIR / name).read_bytes(), ctype)
        elif self.path == "/state":
            self._send(json.dumps(self._state()).encode(), "application/json")
        elif self.path == "/stream":
            self._stream()
        elif LOG_PATH.fullmatch(self.path):
            self._log_file(*LOG_PATH.fullmatch(self.path).groups())
        else:
            self.send_error(404)

    def _log_file(self, name, filename):
        path = self.server.log.root / name / filename
        if filename in LOG_FILES and path.is_file():
            self._send(path.read_bytes(), LOG_FILES[filename])
        else:
            self.send_error(404)

    def _body(self):
        """The JSON a button sent, or ``{}``."""
        size = int(self.headers.get("Content-Length") or 0)
        if not size:
            return {}
        try:
            body = json.loads(self.rfile.read(size))
        except ValueError:
            raise ValueError("The submitted values could not be read. Check the entries and try again.")
        return body if isinstance(body, dict) else {}

    def _locate_routes(self):
        loc = getattr(self.server.tracker, "locator", None)
        if loc is None:
            return {}
        xyz = lambda b: [b.get("x"), b.get("y"), b.get("z")] if all(k in b for k in "xyz") else []
        return {"/locate/home": lambda b: loc.set_home(xyz(b)),
                "/locate/point": lambda b: loc.capture_point(xyz(b)),
                "/locate/cancel": lambda b: loc.cancel_capture(),
                "/locate/remove": lambda b: loc.remove_point(int(b.get("index", -1))),
                "/locate/clear": lambda b: loc.clear_points()}

    def do_POST(self):
        t, log = self.server.tracker, self.server.log
        with_body = self._locate_routes().get(self.path)
        if with_body is not None:
            try:
                with_body(self._body())
                body = {"ok": True}
            except (ValueError, TypeError) as e:
                body = {"ok": False, "error": str(e)}
            self._send(json.dumps(body).encode(), "application/json")
            return
        fn = {"/capture": t.capture_reference_field,
              "/calibration/icm/toggle": lambda: t.toggle_calibration("icm"),
              "/calibration/mmc/toggle": lambda: t.toggle_calibration("mmc"),
              "/tumble/start": t.tumble_start, "/tumble/finish": t.tumble_finish,
              "/tumble/cancel": t.tumble_cancel, "/calibration/reset": t.reset_calibration,
              "/log/start": lambda: log.start(t.status()), "/log/stop": log.stop}.get(self.path)
        if fn is None:
            self.send_error(404)
            return
        try:
            fn()
            body = {"ok": True}
        except ValueError as e:
            body = {"ok": False, "error": str(e)}
        self._send(json.dumps(body).encode(), "application/json")

    def _send(self, data, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        last = 0
        try:
            while True:
                for s in self.server.feed.since(last):
                    last = s["seq"]
                    self.wfile.write(f"data: {json.dumps(s)}\n\n".encode())
                self.wfile.write(f"event: status\ndata: {json.dumps(self._state())}\n\n".encode())
                self.wfile.flush()
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            pass


def make_server(host, port, tracker, feed, log):
    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    server.tracker, server.feed, server.log = tracker, feed, log
    return server


def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()
