"""What the calibrations save to disk."""

import json
from pathlib import Path


class CalibrationFile:
    def __init__(self, path, tumble_log_path, mounting_path=None):
        self.path = Path(path)
        self.mounting_path = Path(mounting_path) if mounting_path else None
        self.tumble_log_path = Path(tumble_log_path)

    def load(self):
        """``(offset, bias, gains)`` from an earlier calibration."""
        try:
            d = json.loads(self.path.read_text())
            return (tuple(float(v) for v in d["offset"][:3]),
                    tuple(float(v) for v in d.get("bias", (0.0, 0.0, 0.0))[:3]),
                    tuple(float(v) for v in d.get("gains", (1.0, 1.0, 1.0))[:3]))
        except (OSError, ValueError, KeyError, TypeError):
            return None, (0.0, 0.0, 0.0), (1.0, 1.0, 1.0)

    def load_axes(self):
        """How the sensor sits on the board, or ``None`` if it was never fitted."""
        try:
            axes = json.loads(self.mounting_path.read_text())["axes"]
            return [[float(v) for v in row[:3]] for row in axes[:3]]
        except (OSError, ValueError, KeyError, TypeError, AttributeError):
            return None

    def save_axes(self, axes):
        self.mounting_path.write_text(json.dumps({"axes": axes}, indent=2))

    def save(self, offset, bias, gains):
        self.path.write_text(json.dumps(
            {"offset": list(offset), "bias": list(bias), "gains": list(gains)}, indent=2))

    def clear(self):
        """Forget every calibration.  Recorded logs and the mounting are kept."""
        for path in (self.path, self.tumble_log_path):
            path.unlink(missing_ok=True)

    def log_tumble(self, points, result):
        """Keep the raw readings of every attempt; ``result`` is ``None`` if it failed."""
        self.tumble_log_path.write_text(json.dumps(
            {"result": result, "points": [list(p) for p in points]}))
