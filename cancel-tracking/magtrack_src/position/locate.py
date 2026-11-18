"""Absolute coil-field lookup, calibration collection, ambiguity policy and storage.

No sensor starting position is assumed. Map directions are saved separately
from position. Older reference-relative files are never reinterpreted.
"""

import threading
from pathlib import Path

from magtrack_src.config import (LOCATE_CENTER_SLACK_CM, LOCATE_MAX_MISMATCH_UT, LOCATE_MIN_POINTS,
                                LOCATE_POINT_SAMPLES, LOCATE_POINT_MAX_SPREAD_UT, LOCATE_RANGE_CM)
from magtrack_src.geometry.vectors import norm, sub
from magtrack_src.orientation.map_frame import validate_frame
from magtrack_src.persistence import read_json_object, temporary_json
from magtrack_src.position.coil_model import Coil, mm_per_ut
from magtrack_src.position.coil_fit import fit_coil, least_squares
from magtrack_src.position.field_lookup import FieldLookup
from magtrack_src.position.position_solver import check_points, left_out_check, solve
from magtrack_src.position.measurements import CoilCalibrationMeasurement
from magtrack_src.validation import finite_number, vector3


class Locator:
    def __init__(self, path=None):
        self._lock = threading.Lock()
        self._save_lock = threading.Lock()
        self._revision = 0
        self._reference_id = None
        self._selection_revision = 0
        self._seen_references = set()
        self._path = Path(path) if path else None
        self.frame = None
        self.points, self.checks = [], []
        self.coil, self._lookup = None, None
        self.message = None
        self.fitting = False
        self._legacy = False
        self._capture = None
        self._last = None
        self._choices = []
        self._load()

    def configure_frame(self, frame):
        frame = validate_frame(frame)
        with self._lock:
            if self.frame and self.frame["id"] == frame["id"]:
                return
            if self.frame and self.points:
                raise ValueError("Start a new map before changing its saved axes.")
            self.frame = frame
            self._revision += 1
            self._last, self._choices = None, []
            save = not self._legacy
        if save:
            self._save_later()

    def update(self, sample):
        invalid = lambda text: {"position": None, "why": text, "history_based": False,
                                "reconnect_warning": bool(sample.get("reconnect_warning"))}
        if not sample.get("ambient_ready"):
            return invalid("Measure the ambient field with the electromagnet switched off.")
        if not sample.get("coil_on"):
            return invalid("Switch the electromagnet on, then confirm Coil is on to start tracking.")
        try:
            field = vector3(sample.get("coil_field"), "coil field")
        except ValueError:
            return invalid("Magnetic reading is invalid. Three finite coil-field components are required.")
        with self._lock:
            if not self.frame or sample.get("frame_id") != self.frame["id"]:
                return invalid("The measurement and coil model use different map axes. Measure the ambient field again.")
            if sample.get("correction_id") != self.frame["correction_id"]:
                return invalid("Sensor corrections changed. Start a new map and record new coil measurements.")
            if not sample.get("reference_id"):
                return invalid("The ambient measurement has no identifier. Measure it again.")
            if sample["reference_id"] != self._reference_id:
                if sample["reference_id"] in self._seen_references:
                    return invalid("This reading belongs to an obsolete ambient measurement.")
                if self._reference_id is not None and self._capture is not None:
                    self._capture = None
                    self.message = "Field recording cancelled because the ambient measurement changed."
                self._reference_id = sample["reference_id"]
                self._seen_references.add(self._reference_id)
                self._selection_revision += 1
                self._last, self._choices = None, []
            if self._capture is not None:
                self._capture_step(field)
            lookup, coil, previous = self._lookup, self.coil, self._last
            revision, reference_id = self._revision, self._reference_id
            selection_revision = self._selection_revision
        if lookup is None or coil is None:
            return invalid("No coil model is ready. Record fields at measured positions, or wait for the model and lookup table.")
        try:
            matches = lookup.matches(field, LOCATE_MAX_MISMATCH_UT, previous)
            matches = [(vector3(p, "position"), finite_number(miss, "field mismatch")) for p, miss in matches
                       if 0 <= miss <= LOCATE_MAX_MISMATCH_UT]
        except (ValueError, OverflowError, ZeroDivisionError):
            return invalid("The position calculation produced invalid numbers. Waiting for a valid reading.")
        with self._lock:
            if (revision != self._revision or reference_id != self._reference_id or
                selection_revision != self._selection_revision):
                return invalid("Calibration or ambient measurement changed during lookup. Waiting for a new reading.")
            if not matches:
                self._choices = []
                return invalid("No position matches the measured coil field within the allowed tolerance.")
            if previous is None and len(matches) > 1:
                self._choices = [p for p, _ in matches]
                result = invalid("Several positions match this field. Hold still and choose the matching starting position.")
                result["candidates"] = [list(p) for p in self._choices]
                return result
            p, miss = min(matches, key=lambda match: norm(sub(match[0], previous))) if previous else matches[0]
        try:
            sensitivity = finite_number(mm_per_ut(coil, p), "position sensitivity")
            distance = finite_number(norm(p), "distance from coil")
        except (ValueError, OverflowError):
            return invalid("The position calculation produced invalid sensitivity or distance.")
        history = len(matches) > 1 or sensitivity > 4.0
        result = {"position": [round(v, 2) for v in p], "from_coil_cm": round(distance, 1),
                  "mismatch_ut": round(miss, 2), "mm_per_ut": round(min(sensitivity, 999.0), 1),
                  "history_based": history, "reconnect_warning": bool(sample.get("reconnect_warning")),
                  "warning": ("AMBIGUOUS FIELD: using the previous position to choose among matching locations. "
                              "Motion within a blind spot cannot be recovered from this field alone." if history else None)}
        with self._lock:
            if (revision != self._revision or reference_id != self._reference_id or
                selection_revision != self._selection_revision):
                return invalid("Calibration or ambient measurement changed during lookup. Waiting for a new reading.")
            self._last, self._choices = p, []
        return result

    def choose_position(self, position, reference_id):
        p = vector3(position, "selected position")
        with self._lock:
            if reference_id != self._reference_id or not self._choices:
                raise ValueError("The starting choices changed. Hold still and select a current matching position.")
            nearest = min(self._choices, key=lambda q: norm(sub(p, q)))
            if norm(sub(p, nearest)) > 1.0:
                raise ValueError("That position no longer matches the current reading. Hold still and choose again.")
            self._last, self._choices = nearest, []
            self._selection_revision += 1

    def forget_position(self):
        with self._lock:
            self._selection_revision += 1
            self._last, self._choices, self._capture = None, [], None

    def _capture_step(self, field):
        p, fields = self._capture
        fields.append(field)
        if len(fields) < LOCATE_POINT_SAMPLES:
            return
        self._capture = None
        mean = tuple(sum(v[i] for v in fields) / len(fields) for i in range(3))
        spread = max(norm(sub(v, mean)) for v in fields)
        if spread > LOCATE_POINT_MAX_SPREAD_UT:
            self.message = f"Measurement not saved: the field varied by {spread:.1f} microtesla. Hold still and try again."
            return
        self.points.append(CoilCalibrationMeasurement(p, mean, spread))
        self.message = None
        self._start_fit()

    def capture_point(self, position):
        p = vector3(position, "measurement position")
        with self._lock:
            if self._legacy:
                raise ValueError("The saved measurements use an older reference-relative format. Delete them before recording new measurements.")
            if self.frame is None:
                raise ValueError("Measure the coil-off ambient field before recording coil measurements.")
            if self._capture is not None:
                raise ValueError("A field recording is already in progress.")
            self._capture = (p, [])
            self.message = None

    def cancel_capture(self):
        with self._lock:
            self._capture = None

    def remove_point(self, index):
        with self._lock:
            if not 0 <= index < len(self.points):
                raise ValueError("The selected measurement no longer exists.")
            del self.points[index]
            self._start_fit()

    def clear_points(self, new_frame=False):
        with self._lock:
            self._revision += 1
            self._selection_revision += 1
            self._capture, self._last, self.coil, self._lookup = None, None, None, None
            self.points, self.checks, self._choices = [], [], []
            self.fitting, self._legacy = False, False
            self.message = None
            if new_frame:
                self.frame = None
            self._reference_id = None
            self._seen_references.clear()
        self._save()

    def _start_fit(self):
        self._revision += 1
        self.coil, self._lookup, self._last, self._choices = None, None, None, []
        self.checks = []
        points = [(point.position_cm, point.coil_field_ut) for point in self.points]
        if len(points) < LOCATE_MIN_POINTS:
            self.fitting = False
            self._save_later()
            return
        self.fitting = True
        threading.Thread(target=self._fit, args=(points, self._revision), daemon=True).start()

    def _fit(self, points, revision):
        try:
            coil = fit_coil(points)
            Coil.from_dict(coil.as_dict())  # validate before installing a numerical result
            checks = list(zip(check_points(coil, points), left_out_check(coil, points)))
            for pair in checks:
                for value in pair:
                    if value is not None and finite_number(value, "position check") < 0:
                        raise ValueError("position checks cannot be negative")
            lookup = FieldLookup(coil, LOCATE_RANGE_CM)
            message = None
        except (ValueError, OverflowError, ZeroDivisionError) as error:
            coil, lookup, checks, message = None, None, [], f"Could not calculate the coil model: {error}"
        with self._lock:
            if revision != self._revision:
                return
            self._revision += 1
            self.coil, self._lookup, self.checks, self.message = coil, lookup, checks, message
            self.fitting = False
        self._save()

    def _save_later(self):
        threading.Thread(target=self._save, daemon=True).start()

    def _save(self):
        if self._path is None:
            return
        with self._save_lock:
            while not self._write():
                pass

    def _write(self):
        with self._lock:
            if self._legacy:
                return True
            revision = self._revision
            data = {"format_version": 2, "field_kind": "absolute_coil_field", "frame": self.frame,
                    "points": [{"position": p.position_cm, "coil_field_ut": p.coil_field_ut,
                                "spread_ut": p.max_field_deviation_ut} for p in self.points],
                    "coil": self.coil.as_dict() if self.coil else None, "checks": self.checks}
        try:
            with temporary_json(self._path, data) as tmp:
                with self._lock:
                    if revision == self._revision:
                        tmp.replace(self._path)
                        return True
            return False
        except (OSError, ValueError) as error:
            with self._lock:
                if revision == self._revision:
                    self.message = f"Couldn't save the coil calibration: {error}"
                    return True
            return False

    def _load(self):
        if self._path is None:
            return
        try:
            data = read_json_object(self._path)
            if data.get("format_version") != 2 or data.get("field_kind") != "absolute_coil_field":
                self._legacy = True
                raise ValueError("Older reference-relative measurements cannot be used as absolute coil fields. Record new coil calibration; the original file was kept.")
            frame = validate_frame(data["frame"]) if data.get("frame") is not None else None
            if not isinstance(data["points"], list):
                raise ValueError("coil measurements must be a list")
            points = []
            for entry in data["points"]:
                if not isinstance(entry, dict):
                    raise ValueError("each coil measurement must be an object")
                spread = finite_number(entry.get("spread_ut", 0), "field variation")
                if spread < 0:
                    raise ValueError("field variation cannot be negative")
                points.append(CoilCalibrationMeasurement(vector3(entry["position"], "position"),
                              vector3(entry["coil_field_ut"], "coil field"), spread))
            coil = Coil.from_dict(data["coil"]) if data.get("coil") is not None else None
            checks = data.get("checks", [])
            if not isinstance(checks, list):
                raise ValueError("checks must be a list")
            parsed = []
            for pair in checks:
                if not isinstance(pair, list) or len(pair) != 2:
                    raise ValueError("each check must contain two distances")
                pair = tuple(None if v is None else finite_number(v, "check distance") for v in pair)
                if any(v is not None and v < 0 for v in pair):
                    raise ValueError("check distances cannot be negative")
                parsed.append(pair)
            if parsed and len(parsed) != len(points):
                raise ValueError("checks must correspond to measurements")
            if (points or coil) and frame is None:
                raise ValueError("saved coil data needs its map axes")
            lookup = FieldLookup(coil, LOCATE_RANGE_CM) if coil else None
            self.frame, self.points, self.coil, self.checks, self._lookup = frame, points, coil, parsed, lookup
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
            self.message = f"Saved coil calibration was not loaded ({self._path.name}): {error}"

    def status(self):
        with self._lock:
            checks = self.checks if len(self.checks) == len(self.points) else [(None, None)] * len(self.points)
            coil = self.coil
            return {"frame_id": self.frame["id"] if self.frame else None, "reference_id": self._reference_id,
                    "legacy_calibration": self._legacy,
                    "points": [{"position": list(p.position_cm), "spread_ut": p.max_field_deviation_ut,
                                "off_mm": c[0], "left_out_mm": c[1]} for p, c in zip(self.points, checks)],
                    "min_points": LOCATE_MIN_POINTS, "capturing": self._capture is not None,
                    "capture_done": len(self._capture[1]) / LOCATE_POINT_SAMPLES if self._capture else 0,
                    "fitting": self.fitting, "message": self.message, "range_cm": LOCATE_RANGE_CM,
                    "center_slack_cm": LOCATE_CENTER_SLACK_CM,
                    "candidates": [list(p) for p in self._choices],
                    "lookup_points": self._lookup.size if self._lookup else 0,
                    "coil": None if coil is None else {"center": list(coil.center), "axis": list(coil.axis),
                        "half_length": coil.half_length, "strength": coil.strength,
                        "north_pole": [c + coil.half_length * v for c, v in zip(coil.center, coil.axis)],
                        "south_pole": [c - coil.half_length * v for c, v in zip(coil.center, coil.axis)],
                        "center_off_cm": norm(coil.center)}}
