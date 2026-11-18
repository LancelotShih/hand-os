"""Live position tracking, measurement collection, workers, and saved coil data.

Numerical field calculations live in coil_model, fitting in coil_fit, and the
current position solve/checks in position_solver. This coordinator retains the
existing sensor-home workflow; see docs/architecture.md for its limits.
"""

import math
import threading
from pathlib import Path

from magtrack_src.config import (LOCATE_CENTER_SLACK_CM, LOCATE_MAX_MISMATCH_UT, LOCATE_MIN_POINTS,
                                LOCATE_POINT_SAMPLES, LOCATE_POINT_MAX_SPREAD_UT,
                                LOCATE_RANGE_CM)
from magtrack_src.geometry.vectors import add, norm, sub
from magtrack_src.persistence import read_json_object, temporary_json
from magtrack_src.position.coil_model import Coil, mm_per_ut
from magtrack_src.position.coil_fit import KEEP_CLEAR_CM, fit_coil
from magtrack_src.position.position_solver import check_points, left_out_check, solve
from magtrack_src.position.measurements import CoilCalibrationMeasurement
from magtrack_src.validation import finite_number, vector3

# Keep the former numerical import available for callers of the old module.
from magtrack_src.position.coil_fit import least_squares as least_squares

def _vector(position, what):
    p = tuple(float(v) for v in position)
    if len(p) != 3 or not all(math.isfinite(v) for v in p):
        raise ValueError(f"enter {what}'s x, y and z in cm")
    return p


class Locator:
    """Fed the ICM's samples in the sensor loop; the other methods are for the
    HTTP threads. Fitting runs on worker threads outside the state lock. Live
    solving is synchronous on the sensor loop; shared locks and Python execution
    time can still delay acquisition."""

    def __init__(self, path=None):
        self._lock = threading.Lock()
        self._save_lock = threading.Lock()   # serialize snapshot, temporary write and replacement
        self._revision = 0                  # calibration changes and applied model results
        self._reference_generation = 0      # reference changes observed in incoming samples
        self._path = Path(path) if path else None
        self.home = None              # where the ICM sits at home, cm from the coil's center
        self.points = []              # CoilCalibrationMeasurement entries
        self.coil = None
        self.checks = []              # per point: (mm off, mm off when left out)
        self.message = None
        self.fitting = False
        self._from_home = False       # points loaded from an old file, measured from home
        self._old_coil = None         # that file's coil, measured from home
        self._home_field = None
        self._capture = None          # (position, [errors])
        self._last = None
        self._reference_field = None
        self._load()

    # -- sensor loop -----------------------------------------------------------
    def update(self, sample):
        """Adds ``locate`` to an ICM view sample: position in cm from the coil's
        center, or ``None`` with a reason."""
        error = sample.get("error")
        if error is not None:
            try:
                error = tuple(float(value) for value in error)
                if len(error) != 3 or not all(math.isfinite(value) for value in error):
                    raise ValueError
            except (TypeError, ValueError, OverflowError):
                return {"position": None, "why": "Magnetic reading is invalid. Three finite field components are required."}
        with self._lock:
            if error is None:
                return {"position": None, "why": "No corrected reference field is available. Check sensor calibration and measure the reference field at sensor home."}
            if self.home is None:
                return {"position": None, "why": "Sensor-home coordinates are not saved. Enter and save them before tracking position."}
            # "predicted" is the legacy sample key for the captured reference_field.
            if sample.get("predicted") != self._reference_field:      # captured again: back at home
                reference = sample.get("predicted")
                self._reference_field = list(reference) if reference is not None else None
                self._reference_generation += 1
                self._last = self.home
            if self._capture is not None:
                self._capture_step(error)
            coil, start, home, home_field = self.coil, self._last or self.home, self.home, self._home_field
            revision, reference_generation = self._revision, self._reference_generation
        if coil is None:
            return {"position": None, "why": "No coil model is available. Record fields at measured positions to calibrate the coil model."}
        p, miss = solve(coil, error, start, home_field)
        with self._lock:
            if revision != self._revision or reference_generation != self._reference_generation:
                return {"position": None, "why": "Calibration or reference changed during the position calculation. Waiting for a new reading."}
            if len(p) != 3 or not all(math.isfinite(value) for value in p) or not math.isfinite(miss):
                return {"position": None, "why": "The position calculation produced invalid numbers. Waiting for a valid reading."}
            if miss > LOCATE_MAX_MISMATCH_UT:
                return {"position": None, "why": f"No position matches the measured field within the allowed tolerance. The best field match found differs by {miss:.1f} µT."}
        from_coil_cm, from_home_cm = norm(p), norm(sub(p, home))
        if not math.isfinite(from_coil_cm) or not math.isfinite(from_home_cm):
            return {"position": None, "why": "The position calculation produced invalid distances. Waiting for a valid reading."}
        sensitivity = mm_per_ut(coil, p)
        if not math.isfinite(sensitivity):
            return {"position": None, "why": "The position calculation produced invalid sensitivity. Waiting for a valid reading."}
        result = {"position": [round(v, 2) for v in p],
                  "from_coil_cm": round(from_coil_cm, 1),
                  "from_home_cm": round(from_home_cm, 1),
                  "mismatch_ut": round(miss, 2),
                  "mm_per_ut": round(min(sensitivity, 999.0), 1)}
        with self._lock:
            if revision != self._revision or reference_generation != self._reference_generation:
                return {"position": None, "why": "Calibration or reference changed during the position calculation. Waiting for a new reading."}
            self._last = p
            return result

    def _capture_step(self, error):
        p, errors = self._capture
        errors.append(error)
        if len(errors) < LOCATE_POINT_SAMPLES:
            return
        self._capture = None
        n = len(errors)
        mean = tuple(sum(e[i] for e in errors) / n for i in range(3))
        spread = max(norm(sub(e, mean)) for e in errors)
        if spread > LOCATE_POINT_MAX_SPREAD_UT:
            self.message = (f"Measurement not saved: the magnetic field varied by {spread:.1f} µT during recording. "
                            "Hold the board still and try again.")
            return
        self.points.append(CoilCalibrationMeasurement(p, mean, round(spread, 2)))
        self.message = None
        self._start_fit()

    # -- control side ----------------------------------------------------------
    def set_home(self, position):
        """Where the ICM sits on its home mark, in cm from the coil's center.
        Points already measured from the coil keep their positions; points from an
        old file, measured from home, are moved to be measured from the coil."""
        h = _vector(position, "the sensor home")
        if norm(h) < KEEP_CLEAR_CM:
            raise ValueError(f"the sensor home can't be within {KEEP_CLEAR_CM:.0f} cm of the coil's center")
        with self._lock:
            if self._capture is not None:
                raise ValueError("Wait for the current field recording to finish, or cancel it.")
            if self._from_home:
                self.points = [CoilCalibrationMeasurement(add(p, h), e, s) for p, e, s in self.points]
                self._from_home, self._old_coil = False, None
            self.home = h
            self._last = h
            self.message = None
            self._start_fit()

    def capture_point(self, position):
        p = _vector(position, "the point")
        with self._lock:
            if self.home is None:
                raise ValueError("Enter and save the sensor-home coordinates first.")
            if self._capture is not None:
                raise ValueError("A field recording is already in progress. Wait for it to finish or cancel it.")
            self._capture = (p, [])
            self.message = None

    def cancel_capture(self):
        with self._lock:
            self._capture = None

    def remove_point(self, index):
        with self._lock:
            if not 0 <= index < len(self.points):
                raise ValueError("The selected coil-calibration measurement no longer exists.")
            del self.points[index]
            self._start_fit()

    def clear_points(self):
        with self._lock:
            self._revision += 1
            self._capture = None
            self.fitting = False
            self._last = None
            self.points = []
            self.coil = None
            self._home_field = None
            self.checks = []
            self.message = None
            self._from_home, self._old_coil = False, None
        self._save()

    def _start_fit(self):
        """Called with the lock held."""
        self._revision += 1
        points = [(point.position_cm, point.field_difference_ut) for point in self.points]
        if len(points) < LOCATE_MIN_POINTS or self.home is None:
            self.coil, self._home_field, self.checks, self.fitting = None, None, [], False
            self._save_later()
            return
        self.fitting = True
        threading.Thread(target=self._fit, args=(points, self.home, self._revision), daemon=True).start()

    def _fit(self, points, home, revision):
        try:
            coil = fit_coil(points, home)
            checks = list(zip(check_points(coil, points, home), left_out_check(coil, points, home)))
            message = None
        except ValueError as e:
            coil, checks, message = None, [], f"Could not calculate the coil model: {e}"
        with self._lock:
            if revision != self._revision:
                return
            self._revision += 1
            self.coil, self.checks, self.message, self.fitting = coil, checks, message, False
            self._home_field = coil.field(home) if coil else None
            self._last = home
        self._save()

    def _save_later(self):
        threading.Thread(target=self._save, daemon=True).start()

    def _save(self):
        if self._path is None:
            return
        with self._save_lock:
            while not self._write():
                pass  # state changed during I/O: take a fresh snapshot under this save lock

    def _write(self):
        """Try to save one revision. False requests a retry with current state."""
        with self._lock:
            revision = self._revision
            data = {"points": [{"position": list(point.position_cm),
                                "error": list(point.field_difference_ut),
                                "spread_ut": point.max_field_deviation_ut}
                               for point in self.points]}
            if self._from_home:       # still waiting for home: keep the old file's form
                data["coil"] = self._old_coil.as_dict() if self._old_coil else None
            else:
                data.update(home=list(self.home) if self.home else None,
                            coil=self.coil.as_dict() if self.coil else None, checks=self.checks)
        try:
            with temporary_json(self._path, data) as tmp:
                with self._lock:
                    if revision == self._revision:
                        tmp.replace(self._path)
                        return True
            return False
        except (OSError, ValueError) as e:
            with self._lock:
                if revision == self._revision:
                    self.message = f"Couldn't save the coil calibration: {e}"
                    return True
            return False

    def _load(self):
        if self._path is None:
            return
        try:
            d = read_json_object(self._path)
            if not isinstance(d["points"], list):
                raise ValueError("coil measurements must be a list")
            points = []
            for q in d["points"]:
                if not isinstance(q, dict):
                    raise ValueError("each coil measurement must be an object")
                spread = finite_number(q.get("spread_ut", 0.0), "field variation")
                if spread < 0.0:
                    raise ValueError("field variation cannot be negative")
                points.append(CoilCalibrationMeasurement(vector3(q["position"], "measurement position"),
                                                         vector3(q["error"], "field difference"), spread))
            coil = Coil.from_dict(d["coil"]) if d.get("coil") is not None else None
            raw_checks = d.get("checks", [])
            if not isinstance(raw_checks, list):
                raise ValueError("coil checks must be a list")
            checks = []
            for pair in raw_checks:
                if not isinstance(pair, (list, tuple)) or len(pair) != 2:
                    raise ValueError("each coil check must contain two distances")
                distances = tuple(None if v is None else finite_number(v, "check distance") for v in pair)
                if any(v is not None and v < 0.0 for v in distances):
                    raise ValueError("check distances cannot be negative")
                checks.append(distances)
            if checks and len(checks) != len(points):
                raise ValueError("coil checks must correspond to the saved measurements")
            if "home" not in d:       # an old file: positions measured from home, coil and all
                self.points = points
                self._from_home = bool(points)
                self._old_coil = coil if self._from_home else None
                return
            home = vector3(d["home"], "sensor home") if d["home"] is not None else None
            if home is not None and norm(home) < KEEP_CLEAR_CM:
                raise ValueError(f"sensor home must be at least {KEEP_CLEAR_CM:g} cm from the coil center")
            home_field = coil.field(home) if coil is not None and home is not None else None
            if home_field is not None:
                vector3(home_field, "model field at sensor home")
                finite_number(mm_per_ut(coil, home), "model sensitivity at sensor home")
            # Install only after the entire file has passed validation.
            self.home, self.points, self.coil, self.checks = home, points, coil, checks
            self._home_field = home_field
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
            self.message = f"Saved coil calibration was not loaded ({self._path.name}): {error}"

    def status(self):
        with self._lock:
            c, home = self.coil, self.home
            checks = self.checks if len(self.checks) == len(self.points) else [(None, None)] * len(self.points)
            return {
                "home": None if home is None else [round(v, 2) for v in home],
                "points_from_home": self._from_home,
                "suggested_home": ([round(-v, 1) for v in self._old_coil.center]
                                   if self._old_coil else None),     # the old fit's coil, seen from home
                "points": [{"position": [round(v, 2) for v in point.position_cm],
                            "spread_ut": point.max_field_deviation_ut, "off_mm": k[0],
                            "left_out_mm": k[1]} for point, k in zip(self.points, checks)],
                "min_points": LOCATE_MIN_POINTS,
                "capturing": self._capture is not None,
                "capture_done": len(self._capture[1]) / LOCATE_POINT_SAMPLES if self._capture else 0.0,
                "fitting": self.fitting,
                "message": self.message,
                "range_cm": LOCATE_RANGE_CM,
                "center_slack_cm": LOCATE_CENTER_SLACK_CM,
                "coil": None if c is None or home is None else {
                    "center": [round(v, 2) for v in c.center],
                    "center_off_cm": round(norm(c.center), 1),
                    "axis": [round(v, 4) for v in c.axis],
                    "half_length": round(c.half_length, 2),
                    "strength": round(c.strength, 1),
                    "home_field": [round(v, 2) for v in c.field(home)],
                    "home_field_ut": round(norm(c.field(home)), 1),
                    "home_mm_per_ut": round(min(mm_per_ut(c, home), 999.0), 1)},
            }
