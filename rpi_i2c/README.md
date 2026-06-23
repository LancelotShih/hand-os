# rpi_i2c — I2C Sensor Monitor

Streams raw byte data and decoded values from onboard magnetometers and an IMU over I2C to the terminal. Intended for testing sensor wiring and signal quality before integrating into the broader hand-os firmware.

## Sensors

| Driver | Sensor | Type | Default I2C Address |
|---|---|---|---|
| `ICM_20948_driver.py` | ICM-20948 | 9-axis IMU (accel, gyro, temp) + internal AK09916 magnetometer | `0x68` |
| `MMC5983MA_driver.py` | MMC5983MA | 3-axis 18-bit magnetometer | `0x30` |
| `TLV493D_driver.py` | TLV493D-A1B6 | 3D magnetic sensor | `0x5E` |

## Hardware Setup

Connect sensors to the RPi I2C-1 bus (default):

| RPi Pin | Signal |
|---|---|
| Pin 3 (GPIO 2) | SDA |
| Pin 5 (GPIO 3) | SCL |
| Pin 1 or 17 | 3.3 V |
| Pin 6, 9, 14, 20, 25, 30, 34, or 39 | GND |

Enable I2C on the RPi if not already on:

```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

Verify sensors are visible on the bus:

```bash
sudo apt install i2c-tools
i2cdetect -y 1
```

## Installation

Enable the I2C interface (see above), then install Python dependencies inside a virtual environment:

```bash
cd rpi_i2c
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Or use the provided script which handles venv creation automatically on first run:

```bash
chmod +x run.sh
./run.sh
```

## Running

```bash
# Using the run script (recommended)
./run.sh

# Manually, with venv activated
source .venv/bin/activate
python main.py
# or
.venv/bin/python main.py

# Specify a different I2C bus (e.g. bus 4 on RPi 5)
./run.sh 4
python main.py 4
```

The monitor clears the terminal and refreshes at ~20 Hz. Press `Ctrl+C` to stop.

Each sensor is initialised independently — if one is not wired up or not found, the others continue streaming without error.

## Output Format

For each sensor, the display shows:

- **raw** — the exact bytes returned over I2C, in hex
- **val** — the decoded integer values with units and scale notes

Example:

```
── ICM-20948  IMU + AK09916 Magnetometer ──
  Accel   raw │ 01 2C FF D4 3F A8
  Accel   val │ X=  +300  Y=   -44  Z=+16296  (±2 g, 1 LSB = 0.061 mg)
  Gyro    raw │ 00 12 FF F1 00 08
  Gyro    val │ X=   +18  Y=   -15  Z=    +8  (±250 dps, 1 LSB = 0.0076 dps)
  ...
```

## File Structure

```
rpi_i2c/
├── drivers/
│   ├── __init__.py
│   ├── ICM_20948_driver.py
│   ├── MMC5983MA_driver.py
│   └── TLV493D_driver.py
├── main.py
├── requirements.txt
├── run.sh
└── README.md
```

## RPi 5 Notes

RPi 5 uses the same I2C-1 bus on the 40-pin header as RPi 3B+. If you have sensors on a secondary bus, pass the bus number as an argument: `./run.sh 4`.
