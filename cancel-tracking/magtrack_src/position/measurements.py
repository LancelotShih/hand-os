"""Physical calibration measurements, in saved map axes. cm and microtesla."""
from typing import NamedTuple


class CoilCalibrationMeasurement(NamedTuple):
    position_cm: tuple
    coil_field_ut: tuple
    max_field_deviation_ut: float
