import asyncio
import io
import socket
import threading
import time
import unittest
from contextlib import redirect_stdout

from websockets.asyncio.server import serve
from websockets.sync.client import connect

from magtrack_src.transmission.inference_server_transmission import (
    DUMMY_THUMB_POSITION_MM,
    PACKET,
    packet_for_sample,
    pack_packet,
    transmission_loop,
)
from magtrack_src.transmission.inference_receiver import decode_frame, receive_stream
from magtrack_src.web.feed import Feed


class Packet(unittest.TestCase):
    def test_wire_format_matches_specification_vector(self):
        packet = pack_packet(
            123456,
            (10.5, -3.25, 22.0),
            (35.75, 8.0, 21.5),
            (0.5, 0.5, 0.5, 0.5),
        )
        expected = bytes.fromhex(
            "40 e2 01 00 00 00 28 41 00 00 50 c0 00 00 b0 41 "
            "00 00 0f 42 00 00 00 41 00 00 ac 41 00 00 00 3f "
            "00 00 00 3f 00 00 00 3f 00 00 00 3f"
        )
        self.assertEqual(len(packet), 44)
        self.assertEqual(packet, expected)

    def test_current_locator_position_maps_to_index_and_thumb_is_placeholder(self):
        sample = {
            "calibration_on": True,
            "predicted": [1.0, 2.0, 3.0],
            "quat": [0.5, 0.5, 0.5, 0.5],
            "locate": {"position": [1.05, -0.325, 2.2]},
        }

        packet = packet_for_sample(sample, 10)
        values = PACKET.unpack(packet)

        self.assertEqual(values[0], 10)
        self.assertEqual(values[1:4], DUMMY_THUMB_POSITION_MM)
        for actual, expected in zip(values[4:7], (10.5, -3.25, 22.0)):
            self.assertAlmostEqual(actual, expected)

    def test_skips_samples_before_calibration_or_without_a_position(self):
        sample = {"calibration_on": True, "predicted": None, "quat": [1, 0, 0, 0]}
        self.assertIsNone(packet_for_sample(sample, 10))

        sample["predicted"] = [1.0, 2.0, 3.0]
        sample["locate"] = {"position": None}
        self.assertIsNone(packet_for_sample(sample, 10))

    def test_rejects_nonfinite_values_and_nonunit_quaternions(self):
        with self.assertRaises(ValueError):
            pack_packet(1, (0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
        with self.assertRaises(ValueError):
            pack_packet(1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (2.0, 0.0, 0.0, 0.0))

    def test_receiver_decodes_multiple_packets_in_one_frame(self):
        frame = b"".join((
            pack_packet(0, (0, 0, 0), (1, 2, 3), (1, 0, 0, 0)),
            pack_packet(50, (0, 0, 0), (4, 5, 6), (1, 0, 0, 0)),
        ))
        readings = decode_frame(frame)
        self.assertEqual([reading[0] for reading in readings], [0, 50])
        self.assertEqual(readings[0][4:7], (1.0, 2.0, 3.0))

    def test_receiver_rejects_incomplete_frame(self):
        with self.assertRaises(ValueError):
            decode_frame(b"\x00" * 43)

    def test_receiver_connects_and_prints_packets(self):
        frame = b"".join((
            pack_packet(0, (0, 0, 0), (1, 2, 3), (1, 0, 0, 0)),
            pack_packet(50, (0, 0, 0), (4, 5, 6), (1, 0, 0, 0)),
        ))

        async def send_frame(connection):
            await connection.send(frame)
            await connection.close()

        async def run():
            async with serve(send_frame, "127.0.0.1", 0) as server:
                port = server.sockets[0].getsockname()[1]
                output = io.StringIO()
                with redirect_stdout(output):
                    await receive_stream(f"ws://127.0.0.1:{port}")
                return output.getvalue()

        output = asyncio.run(run())
        self.assertIn("t=       0 ms", output)
        self.assertIn("t=      50 ms", output)
        self.assertIn("index=(   1.00,    2.00,    3.00) mm", output)


class ProducerServer(unittest.TestCase):
    def connect_with_retry(self, uri):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            context = None
            try:
                context = connect(uri, open_timeout=0.2)
                return context, context.__enter__()
            except OSError:
                time.sleep(0.02)
        self.fail("WebSocket producer did not accept a client")

    def test_accepts_client_and_sends_a_binary_packet_after_calibration(self):
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]

        feed, stop = Feed(), threading.Event()
        server_thread = threading.Thread(
            target=transmission_loop,
            args=(feed, stop, "127.0.0.1", port),
            daemon=True,
        )
        server_thread.start()
        client_context = None
        client = None
        try:
            uri = f"ws://127.0.0.1:{port}"
            client_context, client = self.connect_with_retry(uri)

            now_ms = int(time.monotonic() * 1000)
            feed.push({
                "timestamp_ms": now_ms,
                "calibration_on": True,
                "predicted": None,
                "quat": [0.5, 0.5, 0.5, 0.5],
                "locate": {"position": None},
            })
            feed.push({
                "timestamp_ms": now_ms + 50,
                "calibration_on": True,
                "predicted": [1.0, 2.0, 3.0],
                "quat": [0.5, 0.5, 0.5, 0.5],
                "locate": {"position": [1.05, -0.325, 2.2]},
            })

            packet = client.recv(timeout=2)
            self.assertIsInstance(packet, bytes)
            values = PACKET.unpack(packet)
            self.assertEqual(values[0], 50)
            self.assertEqual(values[1:4], DUMMY_THUMB_POSITION_MM)
            for actual, expected in zip(values[4:7], (10.5, -3.25, 22.0)):
                self.assertAlmostEqual(actual, expected)

            client_context.__exit__(None, None, None)
            client_context = None
            client = None
            time.sleep(0.05)
            client_context, client = self.connect_with_retry(uri)
            feed.push({
                "timestamp_ms": now_ms + 100,
                "calibration_on": True,
                "predicted": [1.0, 2.0, 3.0],
                "quat": [0.5, 0.5, 0.5, 0.5],
                "locate": {"position": [1.05, -0.325, 2.2]},
            })
            next_packet = client.recv(timeout=2)
            self.assertEqual(PACKET.unpack(next_packet)[0], 100)
        finally:
            if client_context is not None:
                client_context.__exit__(None, None, None)
            stop.set()
            server_thread.join(timeout=3)
        self.assertFalse(server_thread.is_alive())


if __name__ == "__main__":
    unittest.main()
