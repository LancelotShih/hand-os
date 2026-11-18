"""Minimal laptop client for viewing the Pi's binary inference stream.

Run from the ``cancel-tracking`` directory on the laptop:

    python3 -m pip install -r requirements.txt
    python3 -m magtrack_src.transmission.inference_receiver ws://<pi-ip>:8765
    python3 -m magtrack_src.transmission.inference_receiver ws://10.0.0.123:8765


Replace ``<pi-ip>`` with the Raspberry Pi's local network address. The Pi's
``earth_cancel.py`` must be running, and calibration must be complete before
position packets are sent.
"""

import argparse
import asyncio
import math
import struct
import sys

from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed

PACKET = struct.Struct("<I10f")


def decode_frame(frame):
    """Decode one binary WebSocket frame into packet tuples."""
    if not isinstance(frame, bytes):
        raise ValueError("expected a binary WebSocket frame")
    if not frame or len(frame) % PACKET.size:
        raise ValueError(f"frame length must be a nonzero multiple of {PACKET.size}")

    readings = []
    for offset in range(0, len(frame), PACKET.size):
        values = PACKET.unpack_from(frame, offset)
        timestamp_ms = values[0]
        numbers = values[1:]
        if not all(math.isfinite(value) for value in numbers):
            raise ValueError("packet contains a non-finite value")
        quaternion = numbers[6:10]
        length = math.sqrt(sum(value * value for value in quaternion))
        if abs(length - 1.0) > 0.02:
            raise ValueError("packet quaternion is not unit length")
        readings.append((timestamp_ms, *numbers))
    return readings


async def receive_stream(url):
    """Connect to the Pi and print every valid reading until disconnected."""
    previous_timestamp_ms = None
    async with connect(url) as connection:
        print(f"Connected to {url}", flush=True)
        try:
            async for frame in connection:
                try:
                    readings = decode_frame(frame)
                    for reading in readings:
                        timestamp_ms = reading[0]
                        if (previous_timestamp_ms is not None
                                and timestamp_ms <= previous_timestamp_ms):
                            raise ValueError("timestamp is not strictly increasing")
                        previous_timestamp_ms = timestamp_ms
                        _, thumb_x, thumb_y, thumb_z, index_x, index_y, index_z, qw, qx, qy, qz = reading
                        print(
                            f"t={timestamp_ms:>8} ms  "
                            f"thumb=({thumb_x:7.2f}, {thumb_y:7.2f}, {thumb_z:7.2f}) mm  "
                            f"index=({index_x:7.2f}, {index_y:7.2f}, {index_z:7.2f}) mm  "
                            f"quat=({qw:.4f}, {qx:.4f}, {qy:.4f}, {qz:.4f})",
                            flush=True,
                        )
                except ValueError as error:
                    print(f"Discarding invalid frame: {error}", file=sys.stderr, flush=True)
        except ConnectionClosed as error:
            print(f"Connection closed: {error}", file=sys.stderr, flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("url", help="Pi WebSocket URL, for example ws://10.0.0.123:8765")
    args = parser.parse_args(argv)
    try:
        asyncio.run(receive_stream(args.url))
    except (OSError, ConnectionClosed) as error:
        print(f"Could not receive from {args.url}: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
