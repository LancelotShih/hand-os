"""Stand-ins for the coil and a *rotating* ICM-20948 so the steady localizer
runs with no hardware (``./steady_locate.py --sim``).

Positions follow the same hold-and-glide waypoints as ``../em-localizer``'s
simulator, but the board also turns: it sits still in the reference pose for
``SIM_START_S`` (gyro null + reference), then slews to a random tilt and
heading at every waypoint.  During an Earth calibration the engine calls
:meth:`SimIMU.set_phase`: the board tumbles through every orientation, then
returns to its mark.

Everything the real part gets wrong is in here: the gyro has a bias that
slowly random-walks (so yaw drifts after the startup null), and the
magnetometer has a hard-iron offset, soft-iron gains, noise, quantisation,
the AK09916 axis order and 100 Hz sample timing.
"""

from __future__ import annotations

import math
import random
import time

from earth_model import quat_rotate   # also puts ../em-localizer and ../icm20948-visualizer on sys.path
from em_localize import dipole_field
from em_sim import SIM_HOLD_S, SIM_MOVE_S, SIM_MOMENT, SimCoil, sim_position  # noqa: F401  (SimCoil re-exported)
from imu_ahrs import quat_conj, quat_mul, quat_norm


# --- simulated world (adjust freely) ----------------------------------------

SIM_EARTH_REF = (17.0, -4.0, -43.0)        # uT, in the reference (start-pose) frame
SIM_HARD_IRON = (54.0, -94.4, -29.2)       # uT, raw AK09916 axes
SIM_SOFT_IRON = (1.04, 0.95, 1.01)         # per-axis gain, raw AK09916 axes
SIM_NOISE_UT = 0.4                         # rms per axis per sample
SIM_MAG_ODR_HZ = 100.0
SIM_LSB_UT = 0.15
SIM_FULL_SCALE_UT = 4912.0

SIM_GYRO_BIAS_DPS = (0.40, -0.30, 0.25)    # constant part (removed by the startup null)
SIM_GYRO_WALK_DPS = 0.003                  # bias random walk, dps per sqrt(s)
SIM_GYRO_NOISE_DPS = 0.05
SIM_ACCEL_NOISE_G = 0.003

SIM_START_S = 3.0                          # still in the reference pose at startup
SIM_MAX_TILT_DEG = 45.0                    # waypoint poses: tilt up to this ...
SIM_MAX_YAW_DEG = 90.0                     # ... and heading up to this
SIM_SLEW_GAIN = 2.0                        # 1/s, how briskly the board turns to a pose
SIM_SLEW_MAX_DPS = 90.0
SIM_TUMBLE_DPS = (55.0, 70.0, 45.0)        # tumble rate amplitude per axis
SIM_STEP_S = 0.002                         # integration step
SIM_SEED = 11


def _quat_axis_angle(axis, angle):
    n = math.sqrt(sum(a * a for a in axis)) or 1.0
    s = math.sin(angle / 2.0) / n
    return (math.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)


def _rotvec(q):
    """Rotation vector (rad) of unit quaternion ``q``, shortest way round."""
    w, x, y, z = q
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    s = math.sqrt(x * x + y * y + z * z)
    if s < 1e-9:
        return (0.0, 0.0, 0.0)
    ang = 2.0 * math.atan2(s, w)
    return (x / s * ang, y / s * ang, z / s * ang)


_rng = random.Random(SIM_SEED)
_poses = [(1.0, 0.0, 0.0, 0.0)]


def _pose(i):
    """Target orientation (board -> reference) at waypoint ``i``."""
    while len(_poses) <= i:
        yaw = _quat_axis_angle((0, 0, 1), math.radians(_rng.uniform(-SIM_MAX_YAW_DEG, SIM_MAX_YAW_DEG)))
        a = _rng.uniform(0, 2 * math.pi)
        tilt = _quat_axis_angle((math.cos(a), math.sin(a), 0.0),
                                math.radians(_rng.uniform(0, SIM_MAX_TILT_DEG)))
        _poses.append(quat_norm(quat_mul(yaw, tilt)))
    return _poses[i]


class SimIMU:
    def __init__(self, coil: SimCoil):
        self.coil = coil
        self.mag_enabled = False
        self._t0 = time.monotonic()
        self._t = self._t0                       # simulation time integrated up to
        self._q = (1.0, 0.0, 0.0, 0.0)           # true orientation, board -> reference
        self._w = (0.0, 0.0, 0.0)                # true body rate, rad/s
        self._walk = [0.0, 0.0, 0.0]
        self._tumble_t = None
        self._on_mark = False
        self._next_tick = self._t0
        self._mag = None
        self.truth = None

    def begin(self, do_reset=True):
        pass

    def enable_magnetometer(self):
        self.mag_enabled = True

    def close(self):
        pass

    def set_phase(self, phase):
        """Earth calibration: ``"collect"`` tumbles, the mark phases return the
        board to its reference pose, anything else resumes the waypoints."""
        self._tumble_t = self._t if phase == "collect" else None
        self._on_mark = phase in ("need-mark", "mark")

    # -- the simulated motion ------------------------------------------------
    def _target_rate(self, t):
        if self._tumble_t is not None:
            u = t - self._tumble_t
            ramp = min(1.0, u / 1.5)
            return tuple(ramp * math.radians(a) * math.sin(f * u + ph) for a, f, ph in
                         zip(SIM_TUMBLE_DPS, (0.71, 0.53, 0.37), (0.0, 1.3, 2.6)))
        tm = t - self._t0 - SIM_START_S
        if tm < 0.0 or self._on_mark:
            target = _pose(0)
        else:
            cycle = SIM_HOLD_S + SIM_MOVE_S
            i, f = int(tm // cycle), tm % cycle
            target = _pose(i + 1) if f >= SIM_HOLD_S else _pose(i)
        err = _rotvec(quat_mul(quat_conj(self._q), target))   # body frame
        w = tuple(SIM_SLEW_GAIN * e for e in err)
        n = math.sqrt(sum(v * v for v in w))
        cap = math.radians(SIM_SLEW_MAX_DPS)
        return tuple(v * cap / n for v in w) if n > cap else w

    def _advance(self, t_to):
        while self._t < t_to:
            dt = min(SIM_STEP_S, t_to - self._t)
            self._w = self._target_rate(self._t)
            ang = math.sqrt(sum(v * v for v in self._w)) * dt
            if ang > 0.0:
                self._q = quat_norm(quat_mul(self._q, _quat_axis_angle(self._w, ang)))
            for i in range(3):
                self._walk[i] += random.gauss(0.0, SIM_GYRO_WALK_DPS * math.sqrt(dt))
            self._t += dt

    def _position(self, t):
        return sim_position(max(0.0, t - self._t0 - SIM_START_S))

    def _field_raw(self, t):
        p = self._position(t)
        self.truth = p
        b_ref = tuple(SIM_EARTH_REF[k] + self.coil.current(t) * v
                      for k, v in enumerate(dipole_field(SIM_MOMENT, p)))
        body = quat_rotate(quat_conj(self._q), b_ref)
        body = tuple(v + random.gauss(0.0, SIM_NOISE_UT) for v in body)
        ak = (body[1], body[0], -body[2])           # body = (ak_y, ak_x, -ak_z)
        raw = tuple(SIM_SOFT_IRON[k] * ak[k] + SIM_HARD_IRON[k] for k in range(3))
        return tuple(max(-SIM_FULL_SCALE_UT, min(SIM_FULL_SCALE_UT,
                         round(v / SIM_LSB_UT) * SIM_LSB_UT)) for v in raw)

    def read(self):
        time.sleep(0.0015)                          # ~ one real I2C transaction
        now = time.monotonic()
        period = 1.0 / SIM_MAG_ODR_HZ
        fresh = False
        while self._next_tick <= now:               # the AK latches one sample per tick
            self._advance(self._next_tick)
            self._mag = self._field_raw(self._next_tick)
            self._next_tick += period
            fresh = True
        self._advance(now)
        g = quat_rotate(quat_conj(self._q), (0.0, 0.0, 1.0))
        accel = tuple(v + random.gauss(0.0, SIM_ACCEL_NOISE_G) for v in g)
        gyro = tuple(math.degrees(self._w[i]) + SIM_GYRO_BIAS_DPS[i] + self._walk[i]
                     + random.gauss(0.0, SIM_GYRO_NOISE_DPS) for i in range(3))
        out = {"accel_g": accel, "gyro_dps": gyro, "temp_c": 25.0}
        if self.mag_enabled:
            out["mag_ut"] = self._mag if fresh else None
        return out

    @property
    def truth_quat(self):
        return self._q
