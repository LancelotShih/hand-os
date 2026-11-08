"""Stand-ins for the coil and the ICM-20948 so the localizer runs with no
hardware (``./em_locate.py --sim``).

The simulated sensor hops between random still points in the +/-15 cm cube at
a fixed orientation, holding at each like a hand placing it, and sees a constant
Earth field (with a slow drift), the dipole field of the
electromagnet scaled by the coil's exponentially settling current, AK09916-like
noise, quantisation, and 100 Hz data-ready timing.  Readings come back in the
raw AK09916 axes with a hard-iron offset, exactly like the real part, so the
whole calibration path is exercised.
"""

from __future__ import annotations

import math
import random
import time

from em_localize import dipole_field


# --- simulated world (adjust freely) ----------------------------------------

SIM_MOMENT = (0.0, 0.0, 2.0e5)        # uT*cm^3: ~11 uT at the cube corners
SIM_EARTH_BODY = (18.0, -6.0, -42.0)  # uT, body frame
SIM_EARTH_DRIFT = (0.02, -0.01, 0.015)  # uT/s
SIM_HARD_IRON = (54.0, -94.4, -29.2)  # uT, raw AK frame
SIM_NOISE_UT = 0.4                    # rms per axis per sample
SIM_COIL_TAU_S = 0.006                # coil current time constant
SIM_MAG_ODR_HZ = 100.0
SIM_LSB_UT = 0.15
SIM_FULL_SCALE_UT = 4912.0
SIM_HALF_CM = 13.0                    # waypoints are drawn inside +/- this
SIM_KEEP_OUT_CM = 6.0                 # ... and at least this far from the magnet
SIM_HOLD_S = 4.0                      # sensor sits still at each waypoint this long
SIM_MOVE_S = 3.0                      # ... then glides to the next one in this long
SIM_SEED = 7


class SimCoil:
    def __init__(self, *_args, **_kw):
        self.state = 0
        self._t_switch = time.monotonic()
        self._i_from = 0.0

    def current(self, t):
        """Coil current (in units of full +I) at time ``t``."""
        a = math.exp(-max(0.0, t - self._t_switch) / SIM_COIL_TAU_S)
        return self.state + (self._i_from - self.state) * a

    def set(self, state):
        now = time.monotonic()
        self._i_from = self.current(now)
        self._t_switch = now
        self.state = (state > 0) - (state < 0)

    def off(self):
        self.set(0)

    def close(self):
        self.off()


_rng = random.Random(SIM_SEED)
_waypoints = []


def _waypoint(i):
    while len(_waypoints) <= i:
        while True:
            p = tuple(_rng.uniform(-SIM_HALF_CM, SIM_HALF_CM) for _ in range(3))
            if math.sqrt(sum(v * v for v in p)) >= SIM_KEEP_OUT_CM + 2.0:
                _waypoints.append(p)
                break
    return _waypoints[i]


def sim_position(t):
    """Hold at a waypoint, glide to the next (smoothstep), repeat -- like a
    hand placing the sensor.  Glide points that would pass inside the keep-out
    sphere around the magnet are pushed radially out onto it."""
    cycle = SIM_HOLD_S + SIM_MOVE_S
    i, f = int(t // cycle), t % cycle
    a = _waypoint(i)
    if f < SIM_HOLD_S:
        return a
    b = _waypoint(i + 1)
    u = (f - SIM_HOLD_S) / SIM_MOVE_S
    u = u * u * (3.0 - 2.0 * u)
    p = tuple(a[k] + (b[k] - a[k]) * u for k in range(3))
    r = math.sqrt(sum(v * v for v in p))
    if r < SIM_KEEP_OUT_CM:
        p = tuple(v * SIM_KEEP_OUT_CM / max(r, 1e-6) for v in p)
    return p


class SimIMU:
    def __init__(self, coil: SimCoil, still=False):
        self.coil = coil
        self.still = still               # hold at the first waypoint (for `step`)
        self.mag_enabled = False
        self._t0 = time.monotonic()
        self._next_sample = self._t0
        self.truth = None

    def begin(self, do_reset=True):
        pass

    def enable_magnetometer(self):
        self.mag_enabled = True

    def close(self):
        pass

    def _field_raw(self, t):
        p = sim_position(0.0 if self.still else t - self._t0)
        self.truth = p
        e = tuple(SIM_EARTH_BODY[i] + SIM_EARTH_DRIFT[i] * (t - self._t0) for i in range(3))
        b_em = dipole_field(SIM_MOMENT, p)
        i = self.coil.current(t)
        body = tuple(e[k] + i * b_em[k] + random.gauss(0.0, SIM_NOISE_UT) for k in range(3))
        # body = (raw_y, raw_x, -raw_z)  =>  raw = (body_y, body_x, -body_z)
        raw = (body[1], body[0], -body[2])
        raw = tuple(raw[k] + SIM_HARD_IRON[k] for k in range(3))
        return tuple(max(-SIM_FULL_SCALE_UT, min(SIM_FULL_SCALE_UT,
                         round(v / SIM_LSB_UT) * SIM_LSB_UT)) for v in raw)

    def read(self):
        time.sleep(0.0015)                       # ~ one real I2C transaction
        now = time.monotonic()
        out = {"accel_g": (0.0, 0.0, 1.0), "gyro_dps": (0.0, 0.0, 0.0), "temp_c": 25.0}
        if self.mag_enabled:
            mag = None
            if now >= self._next_sample:
                # the sample was measured at its ODR tick, not when we read it
                mag = self._field_raw(self._next_sample)
                period = 1.0 / SIM_MAG_ODR_HZ
                self._next_sample += period * max(1, math.floor((now - self._next_sample) / period) + 1)
            out["mag_ut"] = mag
        return out
