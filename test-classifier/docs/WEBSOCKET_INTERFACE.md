# WebSocket interface between the glove firmware (Raspberry Pi) and the data-collection / recognition software (laptop)

## 1. Roles

| | Device | Role |
|---|---|---|
| Glove firmware | Raspberry Pi | **WebSocket server**: listens on a port, streams sensor readings to whoever connects |
| Software | Laptop | **WebSocket client**: connects to `ws://10.0.0.123:8765` and only listens |

Data flows one way (Pi -> laptop). Sensor calibration is handled through the existing HTTP server. This connection only receives the final, calibrated sensor stream.
The laptop initiates the connection, so only the **Pi** needs an open port.

## 2. Connection

- URL: `ws://10.0.0.123:8765`.
  Plain `ws://` is enough (same local network); `wss://` is not needed for now.
- The Pi and the laptop must be on the same network.
- The WebSocket server may accept connections before calibration has completed, but **must not transmit any sensor data until calibration is complete**. When calibration completes, start streaming to the already-connected client right away (the client does not reconnect).
- If the stream has to pause later for any reason (for example a recalibration), just resume afterwards. The gap in `timestamp_ms` is detected and handled.
- **Stop** (and free resources) when the client disconnects. A new client must be able to connect again afterwards without restarting the Pi software.
- One client at a time is enough.
- Standard WebSocket ping/pong keepalives are fine and handled by the library on the software side.

## 3. Frame format

Each WebSocket **message (frame)** contains one or more readings, oldest first.

Binary frames: a frame is **N whole packets back to back**, `N * 44` bytes, N >= 1. No header, no checksum, no padding. A frame whose length is not a multiple of 44 is discarded entirely.

Batching (N > 1 per frame) is optional. There is no size limit so N = 1 works fine at 100 Hz; batching 2-5 readings per frame can reduce overhead on a busy Wi-Fi link.

### Binary packet layout (little-endian)

| Bytes | Type | Field | Notes |
|---|---|---|---|
| 0-3 | uint32 | `timestamp_ms` | Time the sample was **taken** (not sent), in ms, from a monotonic clock |
| 4-7 | float32 | `thumb_x` | Position relative to the magnet on the back of the hand |
| 8-11 | float32 | `thumb_y` | |
| 12-15 | float32 | `thumb_z` | |
| 16-19 | float32 | `index_x` | |
| 20-23 | float32 | `index_y` | |
| 24-27 | float32 | `index_z` | |
| 28-31 | float32 | `quat_w` | Hand orientation, **w first** (w, x, y, z) |
| 32-35 | float32 | `quat_x` | |
| 36-39 | float32 | `quat_y` | |
| 40-43 | float32 | `quat_z` | |

Python equivalent: `struct.pack("<I10f", timestamp_ms, thumb_x, thumb_y, thumb_z, index_x, index_y, index_z, qw, qx, qy, qz)`

### Rules for the values
- Every number must be **finite** (no NaN / infinity). If a sensor has no valid reading, **skip that sample**.
- The quaternion must be **unit length** (ex. |q| = 1 +/- 0.02).
- Position units are millimetres. Orientation is the back-of-hand IMU's quaternion in the Hamilton convention, w first (w, x, y, z); its reference frame must stay the same for the whole session.

### Test vector for the binary format (check your packing against this)

`timestamp_ms = 123456`, thumb = (10.5, -3.25, 22.0), index = (35.75, 8.0, 21.5), quaternion (w,x,y,z) = (0.5, 0.5, 0.5, 0.5)

Expected 44 bytes (hex):
```
40 e2 01 00 00 00 28 41 00 00 50 c0 00 00 b0 41 00 00 0f 42 00 00 00 41 00 00 ac 41 00 00 00 3f 00 00 00 3f 00 00 00 3f 00 00 00 3f
```

## 4. Timing

- Nominal rate **100 Hz** (one reading every 10 ms); a millisecond or two of jitter is fine.
- `timestamp_ms` must be **strictly increasing** and **must not restart during a connection**.
- Never repeat, reorder or zero-fill readings.
- **The real rate is measured from the timestamps** (so network jitter does not distort it); more than 10% off 100 Hz is flagged.
- **Do not buffer without limit.** TCP delivers everything in order, so if Wi-Fi stalls and the Pi queues data, it arrives later as a burst and the live view lags behind the hand. If the client falls more than about half a second behind, **drop the oldest readings** instead; the timestamp gap tells us it happened.
- Send immediately (no waiting to fill a batch beyond a few readings) and keep TCP_NODELAY on (the `websockets` library does this by default).

## 5. Network tips for the Pi

- Prefer **Ethernet or 5 GHz Wi-Fi**.
- Turn **Wi-Fi power saving off** on the Pi, it causes latency spikes: `sudo iw dev wlan0 set power_save off` (make it permanent in your network config).
- A guest or "client isolation" Wi-Fi network blocks device-to-device traffic: use the normal network.
- If the Pi has a firewall (`ufw`, nftables), allow inbound TCP on the chosen port.

## 6. What the software does

1. Connects, then treats every frame as described above, splitting batches into single readings.
2. While connected but receiving nothing (for example calibration is still running), it shows "waiting for data" and does not allow recording. It starts working as soon as readings arrive. If the stream goes silent later, it shows "no data" and any recording in progress fails.
3. Checks every reading: format, finite values, quaternion length, timestamp increasing.
4. Estimates lost readings from timestamp gaps (a gap over 15 ms means readings are missing) and shows `valid / invalid / missing` counts and the measured Hz.
5. A recording is kept only if **100 consecutive readings are all valid and none is missing**; otherwise it is thrown away and the user repeats the attempt. So a clean stream directly means more usable data.
6. If the connection closes, the software shows an error and the user reconnects (no silent reconnect).

## 7. Acceptance checklist

- [ ] The test vector above packs to exactly the expected bytes.
- [ ] From the laptop, the collector window connects to `ws://10.0.0.123:8765` and shows "connected".
- [ ] Connect while calibration is still running: no sensor data is sent and the window shows "waiting for data"; when calibration completes, readings start flowing without reconnecting.
- [ ] Live display: rate **100 +/- 2 Hz**, `invalid 0`, `missing 0` over a minute of holding and moving the hand.
- [ ] Stop and restart the laptop software (and/or unplug the network briefly): the Pi software keeps running and accepts a new connection.
