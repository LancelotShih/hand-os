"""Named records for the calibration measurements the current locator uses.

These are home-relative field differences, not absolute coil-field estimates.
Units are explicit. Tuple compatibility preserves existing numerical interfaces
and saved formats while the internal boundaries are refactored.
"""

from typing import NamedTuple, Tuple


class CoilCalibrationMeasurement(NamedTuple):
    """Position from the entered coil center and field difference in reference axes.

    max_field_deviation_ut is the largest vector distance from the capture mean.
    Reference/calibration identifiers and timestamps are not yet collected by
    the existing point-capture workflow; this record does not invent them.
    """

    position_cm: Tuple[float, float, float]
    field_difference_ut: Tuple[float, float, float]
    max_field_deviation_ut: float
