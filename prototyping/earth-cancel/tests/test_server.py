import json
import re
import socket
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

from data_log import DataLog
from feed import Feed
from pair import Pair
from sensor_loop import sensor_loop
from server import PAGE_FILES, make_server
from tracker import Tracker


class Server(unittest.TestCase):
    def setUp(self):
        self.tracker, self.feed = Tracker(), Feed()
        self.dir = tempfile.TemporaryDirectory()
        self.log = DataLog(self.dir.name)
        self.server = make_server("127.0.0.1", 0, Pair(self.tracker), self.feed, self.log)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.dir.cleanup()

    def get(self, path):
        return urllib.request.urlopen(f"http://127.0.0.1:{self.port}{path}", timeout=3).read()

    def post(self, path):
        req = urllib.request.Request(f"http://127.0.0.1:{self.port}{path}", method="POST", data=b"")
        return json.load(urllib.request.urlopen(req, timeout=3))

    def test_every_page_file_is_served_and_linked(self):
        for path in PAGE_FILES:
            self.assertTrue(self.get(path))
        html = self.get("/").decode()
        for name in re.findall(r'(?:src|href)="([^"]+)"', html):
            self.assertIn("/" + name, PAGE_FILES)

    def test_every_info_button_has_text(self):
        html = self.get("/").decode()
        info = self.get("/info.js").decode()
        keys = re.findall(r'data-info="(\w+)"', html)
        self.assertGreater(len(keys), 15)
        for key in keys:
            self.assertRegex(info, rf"\b{key}: \"")
        self.assertEqual(len(keys), len(set(keys)))

    def test_every_element_the_page_scripts_use_exists(self):
        html = self.get("/").decode()
        scripts = "".join(self.get(path).decode() for path in PAGE_FILES if path.endswith(".js"))
        ids = set(re.findall(r'\bid="([\w-]+)"', html))
        classes = {c for group in re.findall(r'class="([^"]+)"', html) for c in group.split()}
        for name in re.findall(r'\$\("([\w-]+)"\)', scripts):
            self.assertIn(name, ids)
        for name in re.findall(r'(?:part|put)\("([\w-]+)"', scripts):
            self.assertIn(name, classes)
        for name in re.findall(r'\["(trace-[\w-]+)"', scripts):
            self.assertIn(name, classes)

    def test_page_scripts_have_balanced_brackets(self):
        for path in PAGE_FILES:
            if not path.endswith(".js"):
                continue
            text = re.sub(r'"(?:[^"\\\n]|\\.)*"|`[^`]*`|//.*', '""', self.get(path).decode())
            for opening, closing in ("()", "[]", "{}"):
                self.assertEqual(text.count(opening), text.count(closing), (path, opening))

    def test_both_sensors_get_the_same_panel(self):
        html = self.get("/").decode()
        self.assertEqual(html.count('<template id="side">'), 1)
        sides = self.get("/sides.js").decode()
        self.assertIn('icm: "ICM-20948"', sides)
        self.assertIn('mmc: "MMC5983MA"', sides)

    def test_only_page_files_are_served(self):
        for path in ("/config.py", "/../config.py", "/page/app.js"):
            with self.assertRaises(urllib.error.HTTPError):
                self.get(path)

    def test_state(self):
        state = json.loads(self.get("/state"))
        self.assertEqual(state["capturing"], "earth")
        self.assertEqual(state["pass_ut"], 5.0)

    def test_zero_is_gone(self):
        with self.assertRaises(urllib.error.HTTPError):
            self.post("/zero/start")

    def test_calibration_switch(self):
        self.assertTrue(json.loads(self.get("/state"))["calibration_on"])
        self.assertTrue(self.post("/calibration/icm/toggle")["ok"])
        self.assertFalse(json.loads(self.get("/state"))["calibration_on"])
        self.assertFalse(self.post("/calibration/mmc/toggle")["ok"])    # no MMC here

    def test_tumble_buttons(self):
        self.assertFalse(self.post("/tumble/finish")["ok"])
        self.assertTrue(self.post("/tumble/start")["ok"])
        self.assertTrue(json.loads(self.get("/state"))["tumbling"])
        self.assertFalse(self.post("/calibration/icm/toggle")["ok"])
        self.assertFalse(self.post("/tumble/finish")["ok"])      # nothing collected yet
        self.assertTrue(self.post("/tumble/cancel")["ok"])
        self.assertFalse(json.loads(self.get("/state"))["tumbling"])

    def test_reset_button(self):
        self.tracker.offset = (1.0, 2.0, 3.0)
        self.assertTrue(self.post("/calibration/reset")["ok"])
        self.assertFalse(json.loads(self.get("/state"))["have_offset"])

    def test_log_buttons_and_files(self):
        self.assertTrue(self.post("/log/start")["ok"])
        self.assertFalse(self.post("/log/start")["ok"])
        name = json.loads(self.get("/state"))["log"]["name"]
        self.assertTrue(self.post("/log/stop")["ok"])
        self.assertFalse(self.post("/log/stop")["ok"])
        self.assertEqual(json.loads(self.get("/state"))["log"]["last"], name)
        for filename in ("plot.html", "raw.csv", "samples.csv", "info.json"):
            self.assertTrue(self.get(f"/logs/{name}/{filename}"))
        self.assertIn("offset", json.loads(self.get(f"/logs/{name}/info.json")))

    def test_only_log_files_are_served(self):
        self.post("/log/start")
        name = json.loads(self.get("/state"))["log"]["name"]
        self.post("/log/stop")
        for path in (f"/logs/{name}/secret.txt", f"/logs/{name}/../config.py", "/logs/../config.py",
                     f"/logs/{name}/missing.csv", "/logs/none/plot.html"):
            with self.assertRaises(urllib.error.HTTPError):
                self.get(path)

    def test_stream(self):
        self.feed.push({"r": 1.5})
        s = socket.create_connection(("127.0.0.1", self.port), timeout=3)
        s.sendall(b"GET /stream HTTP/1.1\r\nHost: x\r\n\r\n")
        data = b""
        while b'"r": 1.5' not in data or b"event: status" not in data:
            data += s.recv(4096)
        s.close()


class FakeIMU:
    """Fails once, then gives a few readings and stops the loop."""
    opened = 0

    def __init__(self, bus, address):
        type(self).opened += 1
        self.left = 3

    def read(self):
        if type(self).opened == 1:
            raise OSError("bus error")
        self.left -= 1
        if self.left == 0:
            self.stop.set()
        return (0.0, 0.0, 1.0), (0.0, 0.0, 0.0), None

    def close(self):
        pass


class Loop(unittest.TestCase):
    def test_reconnects_and_tells_the_tracker(self):
        stop = threading.Event()
        FakeIMU.stop = stop
        FakeIMU.opened = 0
        tracker, feed = Tracker(), Feed()
        with tempfile.TemporaryDirectory() as d:
            th = threading.Thread(target=sensor_loop,
                                  args=(1, 0x68, Pair(tracker), feed, DataLog(d), stop, FakeIMU))
            th.start()
            th.join(5)
        self.assertFalse(th.is_alive())
        self.assertTrue(feed.connected)
        self.assertEqual(FakeIMU.opened, 2)
        self.assertIn("reconnected", tracker.message)
