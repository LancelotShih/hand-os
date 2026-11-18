"""Serve calibrated position and orientation packets to the laptop collector.

The Pi listens as the WebSocket server. The current Tracker feed produces about
20 samples per second; the interface specification's 100 Hz target is deferred.
"""

import asyncio
import math
import struct
import sys
import time

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

INFERENCE_WS_HOST = "0.0.0.0"
INFERENCE_WS_PORT = 8765
MAX_BACKLOG_MS = 500
POLL_INTERVAL_S = 0.01

PACKET = struct.Struct("<I10f")
UINT32_MAX = (1 << 32) - 1
MM_PER_CM = 10.0

# Temporary placeholder until a thumb position source is available.
DUMMY_THUMB_POSITION_MM = (0.0, 0.0, 0.0)


def pack_packet(timestamp_ms, thumb_mm, index_mm, quat):
    """Pack one validated sample into the 44-byte wire format."""
    if not 0 <= timestamp_ms <= UINT32_MAX:
        raise ValueError("timestamp_ms is outside the uint32 range")
    if len(thumb_mm) != 3 or len(index_mm) != 3 or len(quat) != 4:
        raise ValueError("expected three thumb, three index, and four quaternion values")

    values = tuple(float(v) for v in (*thumb_mm, *index_mm, *quat))
    if not all(math.isfinite(v) for v in values):
        raise ValueError("packet values must all be finite")

    q = values[6:10]
    q_length = math.sqrt(sum(v * v for v in q))
    if q_length == 0.0 or abs(q_length - 1.0) > 0.02:
        raise ValueError("quaternion must have unit length within 0.02")
    q = tuple(v / q_length for v in q)

    return PACKET.pack(timestamp_ms, *values[:6], *q)


def packet_for_sample(sample, timestamp_ms):
    """Return a packet when the sample is calibrated and has a valid position."""
    if sample.get("calibration_on") is not True or sample.get("predicted") is None:
        return None

    locate = sample.get("locate")
    position_cm = locate.get("position") if isinstance(locate, dict) else None
    quat = sample.get("quat")
    if not isinstance(position_cm, (list, tuple)) or not isinstance(quat, (list, tuple)):
        return None
    if len(position_cm) != 3 or len(quat) != 4:
        return None

    # Interim mapping: the current Locator is sent as the index position.
    # Confirm the physical sensor assignment before collecting real glove data.
    index_mm = tuple(float(v) * MM_PER_CM for v in position_cm)
    return pack_packet(timestamp_ms, DUMMY_THUMB_POSITION_MM, index_mm, quat)


async def _serve_feed(feed, stop, host, port):
    active_client = False
    timestamp_origin_ms = None
    last_sent_timestamp_ms = None

    async def send_samples(connection, last_seq):
        nonlocal timestamp_origin_ms, last_sent_timestamp_ms
        while not stop.is_set():
            samples = feed.since(last_seq)
            if not samples:
                await asyncio.sleep(POLL_INTERVAL_S)
                continue

            for sample in samples:
                last_seq = sample["seq"]
                sample_time_ms = sample.get("timestamp_ms")
                if not isinstance(sample_time_ms, int) or isinstance(sample_time_ms, bool):
                    continue
                if sample_time_ms < int(time.monotonic() * 1000) - MAX_BACKLOG_MS:
                    continue

                if timestamp_origin_ms is None:
                    timestamp_origin_ms = sample_time_ms
                timestamp_ms = sample_time_ms - timestamp_origin_ms
                if timestamp_ms > UINT32_MAX:
                    print("Inference stream timestamp exceeded uint32 range", file=sys.stderr)
                    await connection.close(code=1011, reason="timestamp range exceeded")
                    return
                if last_sent_timestamp_ms is not None and timestamp_ms <= last_sent_timestamp_ms:
                    continue

                try:
                    packet = packet_for_sample(sample, timestamp_ms)
                except (TypeError, ValueError) as error:
                    print(f"Skipping invalid inference sample: {error}", file=sys.stderr)
                    continue
                if packet is None:
                    continue

                try:
                    await connection.send(packet)
                except ConnectionClosed:
                    return
                last_sent_timestamp_ms = timestamp_ms

    async def handle_client(connection):
        nonlocal active_client
        if active_client:
            await connection.close(code=1013, reason="only one client is supported")
            return

        active_client = True
        print(f"Inference WebSocket client connected: {connection.remote_address}")
        buffered = feed.since(0)
        last_seq = buffered[-1]["seq"] if buffered else 0
        sender = asyncio.create_task(send_samples(connection, last_seq))
        closed = asyncio.create_task(connection.wait_closed())
        try:
            done, _ = await asyncio.wait((sender, closed), return_when=asyncio.FIRST_COMPLETED)
            if sender in done:
                await sender
                await connection.close()
            else:
                await closed
        finally:
            for task in (sender, closed):
                if not task.done():
                    task.cancel()
            await asyncio.gather(sender, closed, return_exceptions=True)
            active_client = False
            print("Inference WebSocket client disconnected")

    async with serve(handle_client, host, port, ping_interval=20, ping_timeout=20):
        print(f"Inference WebSocket server listening on {host}:{port}")
        while not stop.is_set():
            await asyncio.sleep(0.05)


def transmission_loop(feed, stop, host=INFERENCE_WS_HOST, port=INFERENCE_WS_PORT):
    """Run the Pi's WebSocket server until ``stop`` is set."""
    try:
        asyncio.run(_serve_feed(feed, stop, host, port))
    except OSError as error:
        print(f"Could not start inference WebSocket server: {error}", file=sys.stderr)
