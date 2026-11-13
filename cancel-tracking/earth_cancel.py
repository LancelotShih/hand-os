#!/usr/bin/env python3
"""Subtract a captured reference field at every angle, and track the
ICM-20948 around the coil.  With --mmc, an MMC5983MA runs beside it doing the
same job, so the two magnetometers can be compared.

Run with the system python3 (it has smbus2):
  ./earth_cancel.py                # open http://<this-pi's-ip>:8000
  ./earth_cancel.py --mmc          # with the MMC5983MA as well

First run: rest the board for a second, click "Tumble", follow the steps.
Position: start with the board on its home mark and the coil on, then add
calibration points on the page.
After a log made with --mmc: python3 -m magtrack_src.recording.compare data/logs/<name>
See docs/architecture.md for setup, operation, and how it works.
"""

import argparse
import signal
import sys
import threading

from magtrack_src import config
from magtrack_src.calibration.store import CalibrationFile
from magtrack_src.recording.data_log import DataLog
from magtrack_src.web.feed import Feed
from magtrack_src.position.locate import Locator
from magtrack_src.hardware.mmc_loop import MmcReader
from magtrack_src.tracking.pair import Pair
from magtrack_src.hardware.icm_loop import sensor_loop
from magtrack_src.web.server import local_ip, make_server
from magtrack_src.tracking.tracker import Tracker


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-b", "--bus", type=int, default=config.I2C_BUS)
    p.add_argument("-a", "--address", type=lambda x: int(x, 0), default=config.I2C_ADDRESS)
    p.add_argument("--mmc-address", type=lambda x: int(x, 0), default=config.MMC_I2C_ADDRESS)
    p.add_argument("--mmc", action=argparse.BooleanOptionalAction, default=config.MMC_ON,
                   help=f"run the MMC5983MA beside the ICM-20948 (default: {'on' if config.MMC_ON else 'off'}, "
                        "set by MMC_ON in magtrack_src/config.py)")
    p.add_argument("--host", default=config.HTTP_HOST)
    p.add_argument("--port", type=int, default=config.HTTP_PORT)
    args = p.parse_args(argv)

    def _on_sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)

    store = CalibrationFile(config.CALIBRATION_FILE, config.TUMBLE_LOG_FILE)
    offset, bias, gains = store.load()
    locator = Locator(config.COIL_FILE)
    tracker = Pair(Tracker(offset, bias, store, gains, use_calibration=config.ICM_USE_CALIBRATION),
                   locator=locator)
    if args.mmc:
        mmc_store = CalibrationFile(config.MMC_CALIBRATION_FILE, config.MMC_TUMBLE_LOG_FILE,
                                    config.MMC_MOUNTING_FILE)
        mmc_offset, _, mmc_gains = mmc_store.load()
        tracker = Pair(tracker.icm,
                       Tracker(mmc_offset, bias, mmc_store, mmc_gains, axes=mmc_store.load_axes(),
                               fit_mounting=True, block_samples=config.MMC_BLOCK_SAMPLES,
                               use_calibration=config.MMC_USE_CALIBRATION),
                       MmcReader(args.bus, args.mmc_address), locator)
    feed = Feed()
    log = DataLog(config.LOG_DIR)
    stop = threading.Event()
    server = make_server(args.host, args.port, tracker, feed, log)
    th = threading.Thread(target=sensor_loop, daemon=True,
                          args=(args.bus, args.address, tracker, feed, log, stop))
    th.start()
    mmc_th = None
    if tracker.reader is not None:
        mmc_th = threading.Thread(target=tracker.reader.run, args=(stop,), daemon=True)
        mmc_th.start()

    ip = local_ip() if args.host in ("0.0.0.0", "::") else args.host
    print(f"Magnetic field and position tracking:  http://{ip}:{args.port}   (Ctrl+C to stop)")
    print("Rest the board for about a second to estimate the gyroscope readings when it is not turning.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        stop.set()
        th.join(timeout=3.0)
        if mmc_th is not None:
            mmc_th.join(timeout=3.0)
        if log.status()["active"]:              # Ctrl+C mid-log still gives a complete log
            log.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
