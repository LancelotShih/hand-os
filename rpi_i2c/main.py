"""
I2C sensor monitor – streams raw byte data and decoded values to the terminal.

Sensors:
  ICM-20948  (0x68/0x69) – 9-axis IMU + internal AK09916 magnetometer
  MMC5983MA  (0x30)      – 3-axis 18-bit magnetometer
  TLV493D    (0x5E/0x1F) – 3D magnetic sensor

Usage:
  python main.py [bus_number]

  bus_number defaults to 1 (I2C-1 on RPi 3B+ and RPi 5).
  Run `ls /dev/i2c-*` to list available buses.

Dependencies:
  pip install smbus2
"""

import sys
import time

try:
    import smbus2  # noqa: F401 – checked early so the error is friendly
except ImportError:
    print("smbus2 not installed.  Run:  pip install smbus2")
    sys.exit(1)

from drivers.ICM_20948_driver import ICM20948, ICM20948_ADDR_LOW
from drivers.MMC5983MA_driver import MMC5983MA, MMC5983MA_ADDR
from drivers.TLV493D_driver   import TLV493D,   TLV493D_ADDR_DEFAULT

# ── ANSI helpers ─────────────────────────────────────────────────────────────
RESET  = "\033[0m"
BOLD   = "\033[1m"
DIM    = "\033[2m"
CYAN   = "\033[96m"
YELLOW = "\033[93m"
RED    = "\033[91m"
GREEN  = "\033[92m"

CLEAR_SCREEN  = "\033[2J\033[H"
ERASE_TO_EOL  = "\033[K"


def _clr() -> None:
    print(CLEAR_SCREEN, end="", flush=True)


def _hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def _bar(label: str, width: int = 56) -> str:
    side = (width - len(label) - 2) // 2
    return f"{'─' * side} {label} {'─' * (width - side - len(label) - 2)}"


# ── per-sensor display ───────────────────────────────────────────────────────

def _print_icm20948(data: dict) -> None:
    a = data['accel']
    g = data['gyro']
    t = data['temp']
    m = data['mag']

    print(f"{BOLD}{CYAN}{_bar('ICM-20948  IMU + AK09916 Magnetometer')}{RESET}")
    print(f"  Accel   raw │ {DIM}{_hex(a['raw'])}{RESET}")
    print(f"  Accel   val │ X={a['x']:+7d}  Y={a['y']:+7d}  Z={a['z']:+7d}  (±2 g, 1 LSB = 0.061 mg)")
    print(f"  Gyro    raw │ {DIM}{_hex(g['raw'])}{RESET}")
    print(f"  Gyro    val │ X={g['x']:+7d}  Y={g['y']:+7d}  Z={g['z']:+7d}  (±250 dps, 1 LSB = 0.0076 dps)")
    print(f"  Temp    raw │ {DIM}{_hex(t['raw'])}{RESET}")
    t_c = t['value'] / 333.87 + 21.0
    print(f"  Temp    val │ raw={t['value']:+7d}  → {t_c:+.2f} °C")
    print(f"  AK09916 raw │ {DIM}{_hex(m['raw'])}{RESET}")
    print(f"  AK09916 val │ X={m['x']:+7d}  Y={m['y']:+7d}  Z={m['z']:+7d}  "
          f"ST1=0x{m['st1']:02X}  ST2=0x{m['st2']:02X}")


def _print_mmc5983ma(data: dict) -> None:
    m = data['mag']
    print(f"{BOLD}{CYAN}{_bar('MMC5983MA  18-bit Magnetometer')}{RESET}")
    print(f"  Mag     raw │ {DIM}{_hex(m['raw'])}{RESET}")
    print(f"  Mag     val │ X={m['x']:+9d}  Y={m['y']:+9d}  Z={m['z']:+9d}  "
          f"(1 LSB ≈ 0.0625 µT)")


def _print_tlv493d(data: dict) -> None:
    m = data['mag']
    t = data['temp']
    print(f"{BOLD}{CYAN}{_bar('TLV493D  3D Magnetic Sensor')}{RESET}")
    print(f"  Mag     raw │ {DIM}{_hex(m['raw'])}{RESET}")
    print(f"  Mag     val │ X={m['x']:+6d}  Y={m['y']:+6d}  Z={m['z']:+6d}  (1 LSB ≈ 0.098 mT)")
    t_c = (t['raw'] - 340) / 11.79 + 25.0
    print(f"  Temp    val │ raw={t['raw']:4d}  → {t_c:+.1f} °C  (approximate)")


# ── sensor initialisation ─────────────────────────────────────────────────────

def _try_init(name, cls, kwargs):
    try:
        sensor = cls(**kwargs)
        print(f"  {GREEN}✓{RESET} {name}")
        return sensor
    except Exception as exc:
        print(f"  {RED}✗{RESET} {name}: {exc}")
        return None


def init_sensors(bus_num: int) -> dict:
    print(f"\n{BOLD}Initialising sensors on I2C bus {bus_num} …{RESET}\n")
    sensors = {}

    entries = [
        ("ICM-20948",  ICM20948,  {"bus_num": bus_num, "addr": ICM20948_ADDR_LOW}),
        ("MMC5983MA",  MMC5983MA, {"bus_num": bus_num, "addr": MMC5983MA_ADDR}),
        ("TLV493D",    TLV493D,   {"bus_num": bus_num, "addr": TLV493D_ADDR_DEFAULT}),
    ]

    for name, cls, kwargs in entries:
        obj = _try_init(name, cls, kwargs)
        if obj is not None:
            sensors[name] = obj

    return sensors


# ── main loop ────────────────────────────────────────────────────────────────

def main() -> None:
    bus_num = 1
    if len(sys.argv) > 1:
        try:
            bus_num = int(sys.argv[1])
        except ValueError:
            print(f"Usage: python main.py [bus_number]")
            sys.exit(1)

    sensors = init_sensors(bus_num)
    if not sensors:
        import glob
        available = sorted(glob.glob("/dev/i2c-*"))
        print(f"\n{RED}No sensors initialised.{RESET}")
        if not available:
            print("No I2C buses found on this machine.")
            print("This script must run on an RPi with I2C enabled.")
            print("  sudo raspi-config  →  Interface Options → I2C → Enable")
        else:
            print(f"Available I2C buses: {', '.join(available)}")
            if f"/dev/i2c-{bus_num}" not in available:
                others = [b.replace("/dev/i2c-", "") for b in available]
                print(f"Bus {bus_num} not found. Try: ./run.sh {others[0]}")
            else:
                print("Bus exists but no sensors responded. Check wiring and addresses.")
                print("  i2cdetect -y 1   (or the relevant bus number)")
        sys.exit(1)

    names  = list(sensors.keys())
    active = list(sensors.values())

    print(f"\n{BOLD}Streaming …  Ctrl+C to stop.{RESET}\n")
    time.sleep(0.5)

    read_fns = {
        "ICM-20948": _print_icm20948,
        "MMC5983MA": _print_mmc5983ma,
        "TLV493D":   _print_tlv493d,
    }

    frame = 0
    t_start = time.time()

    try:
        while True:
            _clr()
            elapsed = time.time() - t_start
            print(
                f"{BOLD}I2C Sensor Monitor{RESET}  "
                f"bus={bus_num}  "
                f"frame={frame}  "
                f"t={elapsed:7.2f}s  "
                f"{time.strftime('%H:%M:%S')}"
            )
            print("─" * 60)

            for name, sensor in zip(names, active):
                try:
                    data = sensor.read()
                    read_fns[name](data)
                except Exception as exc:
                    print(f"{RED}  {name} read error: {exc}{RESET}")
                print()

            frame += 1
            time.sleep(0.05)   # ~20 Hz refresh; lower to taste

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        for s in active:
            try:
                s.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
