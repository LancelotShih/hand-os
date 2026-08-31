# icm20948 tool

A small Python tool to detect and read a **TDK/InvenSense ICM-20948**
9-axis IMU over I2C on a Raspberry Pi.

## File layout

| File | What it holds |
|------|---------------|
| `icm20948_registers.py` | Register addresses, bank numbers, WHO_AM_I value, full-scale + sensitivity tables. The datasheet as data; no I/O. |
| `icm20948_i2c.py` | `/dev/i2c-*` discovery and bus scanning (`list_buses`, `open_bus`, `scan_bus`). Not chip-specific. |
| `icm20948_driver.py` | The `ICM20948` class only: bank switching, bring-up, measurements, magnetometer bypass. |
| `icm20948_main.py` | The polling loop (`poll_loop`, `run_read`) and line formatters, plus a no-frills stream runner you can execute directly. |
| `icm20948_cli.py` | The full `buses` / `scan` / `whoami` / `read` command-line tool; pure argument parsing and dispatch. |
| `icm20948.py` | Backwards-compatible facade: re-exports the names above and forwards `./icm20948.py …` to the CLI. |
| `ak09916_selftest.py` | Standalone AK09916 magnetometer self-test: energises the mag die's internal coil and checks the reading against datasheet bounds. |
| `imu_web.py` + `imu_dashboard.html` | Optional live browser dashboard built on the driver. |

Dependency direction: `registers` <- `driver` / `i2c` <- `main` <- `cli`.

## One-time setup (Raspberry Pi 5)

### Hardware setup
On the ICM20948 board, aside from the obvious 3.3V, GND, and SDA/SCL connections, you need to also tie AD0 to GND to set I2C to address `0b1101000` (`0x68`) and nCS pin to 3.3V to maintain I2C mode

### Software setup

The 40-pin header I2C bus is disabled by default. Enable it and reboot:

```bash
sudo raspi-config nonint do_i2c 0     # enables i2c_arm + loads i2c-dev
sudo apt install -y i2c-tools python3-smbus2
sudo reboot
```

After reboot `/dev/i2c-1` exists. Your user is already in the `i2c` group, so
no `sudo` is needed to run the tool. Sanity check:

```bash
i2cdetect -y 1        # ICM-20948 answers at 0x68 (or 0x69 if AD0 is high)
```

Wiring: SDA -> GPIO2 (pin 3), SCL -> GPIO3 (pin 5), VCC -> 3V3, GND -> GND.

## Usage

If you would like to run the full tool along with the web interface, you can run any of the following example commands:

```bash
./imu_web.py -b 1                 # http://<this-pi's-ip>:8000
./imu_web.py -b 1 --mag -r 40
./imu_web.py -b 1 --port 8080 -a 0x69
```

Full command-line tool (`icm20948_cli.py`, or the `icm20948.py` facade):

```bash
./icm20948_cli.py buses                     # list /dev/i2c-* buses
./icm20948_cli.py -b 1 scan                 # probe the bus, flags 0x68/0x69/0x0C
./icm20948_cli.py -b 1 whoami               # read WHO_AM_I, expect 0xEA
./icm20948_cli.py -b 1 read                 # stream accel(g) / gyro(dps) / temp(C) at 10 Hz
./icm20948_cli.py -b 1 read -r 50 --mag     # 50 Hz, include AK09916 magnetometer (uT)
./icm20948_cli.py -b 1 read --raw -n 20     # 20 samples of raw 16-bit counts
./icm20948_cli.py -b 1 -a 0x69 whoami       # use the AD0-high address
```

Just the polling loop, no subcommands (`icm20948_main.py`):

```bash
./icm20948_main.py                          # 10 Hz scaled stream, bus 1 @ 0x68
./icm20948_main.py -r 50 --mag              # 50 Hz, include the magnetometer
./icm20948_main.py --raw -n 20              # 20 raw-count samples, then stop
```

`read` / `icm20948_main.py` options: `--accel-range {2,4,8,16}` g,
`--gyro-range {250,500,1000,2000}` dps, `-n/--count`, `-r/--rate`, `--mag`,
`--raw`, `--no-reset`.

Magnetometer self-test (`ak09916_selftest.py`):

```bash
./ak09916_selftest.py                 # bus 1, ICM @ 0x68; prints per-axis pass/fail
./ak09916_selftest.py -b 1 -a 0x69
./ak09916_selftest.py --restore       # re-enable 100 Hz continuous mode afterwards
```

Exit code is `0` on PASS, `1` on FAIL, `2` if the test could not run. The internal
coil field is Z-dominant, so a healthy part reads ~0 on X/Y and roughly -60 uT on
Z (datasheet window: X/Y within +/-200 LSB, Z within -1000..-200 LSB). A pass is a
functional check only -- it is **not** a hard/soft-iron calibration.

## As a library

```python
from icm20948_driver import ICM20948

with ICM20948(bus=1, address=0x68, accel_range=4, gyro_range=500) as imu:
    imu.begin()                 # verifies WHO_AM_I, resets, wakes, configures
    imu.enable_magnetometer()   # optional
    print(imu.read())           # {'accel_g':(...), 'gyro_dps':(...), 'temp_c':..., 'mag_ut':(...)}
```

## Notes

- Magnetometer support uses I2C **bypass mode** (AK09916 exposed at `0x0C` on the
  same bus); it does not use the ICM's internal I2C master / DMP.
- Temperature uses the datasheet formula `degC = raw/333.87 + 21`.
- Only `smbus2` is required (pure Python, no build step).
