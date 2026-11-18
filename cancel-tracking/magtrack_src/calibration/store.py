"""What the calibrations save to disk.

Used by: earth_cancel.py (load), tracking/tracker.py (save).
Uses: shared validation and JSON persistence helpers.
"""

from pathlib import Path

from magtrack_src.persistence import read_json_object, write_json_atomic
from magtrack_src.validation import orthogonal_matrix3, vector3


class CalibrationFile:
    def __init__(self, path, tumble_log_path, mounting_path=None):
        self.path = Path(path)
        self.mounting_path = Path(mounting_path) if mounting_path else None
        self.tumble_log_path = Path(tumble_log_path)
        self.load_error = None
        self.mounting_load_error = None

    def load(self):
        """Validated ``(offset, bias, gains)``; defaults if absent or invalid.

        Missing bias/gains in older files retain their historical defaults.
        Rejected files stay untouched and set ``load_error`` for startup status.
        """
        self.load_error = None
        try:
            d = read_json_object(self.path)
            return self._corrections(d["offset"], d.get("bias", (0.0, 0.0, 0.0)),
                                     d.get("gains", (1.0, 1.0, 1.0)))
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.load_error = f"Saved sensor corrections were not loaded ({self.path.name}): {error}"
        return None, (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)

    @staticmethod
    def _corrections(offset, bias, gains):
        offset, bias, gains = (vector3(offset, "offset"), vector3(bias, "gyroscope zero rate"),
                               vector3(gains, "axis scale multipliers"))
        if any(g <= 0.0 for g in gains):
            raise ValueError("axis scale multipliers must be positive")
        return offset, bias, gains

    def load_axes(self):
        """How the sensor sits on the board, or ``None`` if it was never fitted."""
        self.mounting_load_error = None
        if self.mounting_path is None:
            return None
        try:
            return orthogonal_matrix3(read_json_object(self.mounting_path)["axes"])
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.mounting_load_error = f"Saved sensor mounting was not loaded ({self.mounting_path.name}): {error}"
        return None

    def save_axes(self, axes):
        if self.mounting_path is None:
            raise ValueError("No sensor mounting file is configured.")
        write_json_atomic(self.mounting_path, {"axes": orthogonal_matrix3(axes)})

    def save(self, offset, bias, gains):
        offset, bias, gains = self._corrections(offset, bias, gains)
        write_json_atomic(self.path, {"offset": offset, "bias": bias, "gains": gains})

    def clear(self):
        """Delete sensor corrections and the saved tumble result; keep recordings and mounting."""
        for path in (self.path, self.tumble_log_path):
            path.unlink(missing_ok=True)

    def log_tumble(self, points, result):
        """Replace the last attempt log; ``result`` is ``None`` if it failed."""
        write_json_atomic(self.tumble_log_path,
                          {"result": result, "points": [vector3(p, "tumble reading") for p in points]})
