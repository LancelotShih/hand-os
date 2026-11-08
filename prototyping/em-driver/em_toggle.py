#!/usr/bin/env python3
"""Toggle a DC electromagnet on and off through an L298N H-bridge.

First bring-up step for pulsed electromagnet localization: prove the Pi can
switch the coil on, off, and (optionally) reversed, on a steady clock.

L298N channel A truth table (ENA jumper left ON, i.e. channel always enabled):

    IN1  IN2   coil
     1    0    +I   (forward)
     0    1    -I   (reversed)
     0    0    off  (brake -- zero current, zero field)

Wiring (BCM GPIO numbers, not physical pin numbers):
    Pi GPIO <IN1_PIN> -> L298N IN1
    Pi GPIO <IN2_PIN> -> L298N IN2
    Pi GND            -> L298N GND   (shared with the magnet supply's GND)
    L298N OUT1/OUT2   -> magnet leads

Examples:
    ./em_toggle.py                         # on/off, 0.5 s each, until Ctrl+C
    ./em_toggle.py --period 0.05           # 10 Hz on/off (50 ms per phase)
    ./em_toggle.py --mode bipolar          # +I, off, -I, off, ...
    ./em_toggle.py --cycles 20             # stop after 20 full cycles
    ./em_toggle.py on --seconds 5          # hold ON for 5 s, then off
    ./em_toggle.py off                     # force the coil off and exit

The coil is always driven OFF on exit (normal end, Ctrl+C, or SIGTERM).
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

try:
    from gpiozero import DigitalOutputDevice
except ImportError:
    sys.exit("gpiozero is not installed:  sudo apt install python3-gpiozero python3-lgpio")


# --- wiring: update these to match your harness (BCM GPIO numbers) ----------

# IN1_PIN = 17    # -> L298N IN1  (physical pin 11)
# IN2_PIN = 27    # -> L298N IN2  (physical pin 13)
IN1_PIN = 14    # -> L298N IN1  (physical pin 8)
IN2_PIN = 15    # -> L298N IN2  (physical pin 10)


# --- coil driver ----------------------------------------------------------

class Coil:
    """One L298N channel driving one electromagnet. States: +1, 0, -1."""

    def __init__(self, in1_pin=IN1_PIN, in2_pin=IN2_PIN):
        # initial_value=False: both inputs low before anything else -> coil off
        self._in1 = DigitalOutputDevice(in1_pin, initial_value=False)
        self._in2 = DigitalOutputDevice(in2_pin, initial_value=False)
        self.state = 0

    def set(self, state: int):
        # drop both lines first so the bridge never sees an overlapping
        # IN1=IN2=1 transient when reversing
        self._in1.off()
        self._in2.off()
        if state > 0:
            self._in1.on()
        elif state < 0:
            self._in2.on()
        self.state = (state > 0) - (state < 0)

    def forward(self): self.set(+1)
    def reverse(self): self.set(-1)
    def off(self):     self.set(0)

    def close(self):
        self.off()
        # released pins fall back to input; GPIO17/27 default to pull-down on
        # the Pi, so the L298N inputs stay low (coil off) after exit
        self._in1.close()
        self._in2.close()


# --- run modes ------------------------------------------------------------

PATTERNS = {
    "onoff":   (+1, 0),
    "bipolar": (+1, 0, -1, 0),
}
LABEL = {+1: "+I ", 0: "off", -1: "-I "}


def run_toggle(coil: Coil, mode: str, period: float, cycles: int, quiet: bool):
    """Step through the mode's pattern, ``period`` seconds per phase, on an
    absolute clock so timing does not drift with print/GPIO latency."""
    pattern = PATTERNS[mode]
    t_start = time.monotonic()
    next_t = t_start
    step = 0
    total = cycles * len(pattern) if cycles > 0 else None
    worst_late = 0.0

    while total is None or step < total:
        state = pattern[step % len(pattern)]
        coil.set(state)
        now = time.monotonic()
        worst_late = max(worst_late, now - next_t)
        if not quiet:
            cycle = step // len(pattern) + 1
            print(f"t={now - t_start:8.3f}s  cycle {cycle:5d}  coil {LABEL[state]}",
                  flush=True)
        step += 1
        next_t = t_start + step * period
        dt = next_t - time.monotonic()
        if dt > 0:
            time.sleep(dt)

    print(f"done: {step} phases, worst switch lateness {worst_late * 1e3:.2f} ms")


def run_hold(coil: Coil, state: int, seconds: float):
    coil.set(state)
    print(f"coil {LABEL[state].strip()}" +
          (f" for {seconds:g} s" if seconds > 0 else "  (Ctrl+C to stop)"), flush=True)
    if seconds > 0:
        time.sleep(seconds)
    else:
        while True:
            time.sleep(1.0)


# --- CLI --------------------------------------------------------------------

def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("action", nargs="?", default="toggle",
                   choices=["toggle", "on", "reverse", "off"],
                   help="toggle (default), hold on / reverse, or force off")
    p.add_argument("--in1", type=int, default=IN1_PIN,
                   help=f"BCM GPIO for L298N IN1 (default: {IN1_PIN})")
    p.add_argument("--in2", type=int, default=IN2_PIN,
                   help=f"BCM GPIO for L298N IN2 (default: {IN2_PIN})")
    p.add_argument("-m", "--mode", choices=sorted(PATTERNS), default="onoff",
                   help="toggle pattern (default: onoff)")
    p.add_argument("-p", "--period", type=float, default=0.5,
                   help="seconds per phase (default: 0.5)")
    p.add_argument("-n", "--cycles", type=int, default=0,
                   help="full cycles to run, 0 = until Ctrl+C (default: 0)")
    p.add_argument("-s", "--seconds", type=float, default=0.0,
                   help="for on/reverse: hold time, 0 = until Ctrl+C (default: 0)")
    p.add_argument("-q", "--quiet", action="store_true",
                   help="don't print every switch (use for short periods)")
    args = p.parse_args(argv)

    if args.period <= 0:
        p.error("--period must be > 0")
    if args.in1 == args.in2:
        p.error("--in1 and --in2 must be different pins")

    # SIGTERM (e.g. `kill`, systemd stop) -> same clean path as Ctrl+C
    def _on_sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)

    coil = Coil(args.in1, args.in2)
    print(f"L298N: IN1=GPIO{args.in1}  IN2=GPIO{args.in2}", flush=True)
    try:
        if args.action == "toggle":
            run_toggle(coil, args.mode, args.period, args.cycles, args.quiet)
        elif args.action == "on":
            run_hold(coil, +1, args.seconds)
        elif args.action == "reverse":
            run_hold(coil, -1, args.seconds)
        # "off": nothing to do, close() below drives the coil off
    except KeyboardInterrupt:
        print()
    finally:
        coil.close()
        print("coil off")
    return 0


if __name__ == "__main__":
    sys.exit(main())
