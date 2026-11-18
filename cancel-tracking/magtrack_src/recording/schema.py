"""CSV column definitions, row formatting, and reading current/older recordings.

Numerical tracking does not depend on this file format. Writers, plots, and
comparison tools share it without making the plot renderer own their contract.
Existing column names and legacy residual-column support are preserved.
"""

import csv
from pathlib import Path

RAW_COLUMNS = ["t_s", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
               "mx_ut", "my_ut", "mz_ut",
               "mmc_x_ut", "mmc_y_ut", "mmc_z_ut", "mmc_age_s"]
SAMPLE_COLUMNS = ["t_s", "qw", "qx", "qy", "qz", "yaw_deg", "pitch_deg", "roll_deg", "turn_deg",
                  "measured_x_ut", "measured_y_ut", "measured_z_ut",
                  "predicted_x_ut", "predicted_y_ut", "predicted_z_ut",
                  "error_x_ut", "error_y_ut", "error_z_ut", "error_ut",
                  "error_deg",
                  "mmc_measured_x_ut", "mmc_measured_y_ut", "mmc_measured_z_ut",
                  "mmc_predicted_x_ut", "mmc_predicted_y_ut", "mmc_predicted_z_ut",
                  "mmc_error_x_ut", "mmc_error_y_ut", "mmc_error_z_ut",
                  "mmc_error_ut", "mmc_error_deg", "calibration_on", "mmc_calibration_on",
                  "pos_x_cm", "pos_y_cm", "pos_z_cm", "position_history_based", "reconnect_warning",
                  "ambient_ready", "coil_on", "frame_id", "reference_id", "correction_id"]

IDENTIFIER_COLUMNS = {"frame_id", "reference_id", "correction_id"}


def read_samples(folder):
    """Each column of samples.csv as a list, ``None`` where it's blank."""
    columns = {name: [] for name in SAMPLE_COLUMNS}
    with open(Path(folder) / "samples.csv", newline="") as f:
        for row in csv.DictReader(f):
            for name in SAMPLE_COLUMNS:
                value = row.get(name) or row.get(name.replace("error", "residual"))  # older logs
                columns[name].append((value if name in IDENTIFIER_COLUMNS else float(value)) if value else None)
    return columns


def format_cells(values):
    """CSV cells: missing values are blank; numbers retain the existing five decimals."""
    return ["" if v is None else v if isinstance(v, str) else f"{v:.5f}" for v in values]


def sample_row(t, s):
    """Serialize one view sample in SAMPLE_COLUMNS order using the existing keys."""
    m = s.get("mmc") or {}
    position = (s.get("locate") or {}).get("position")
    locate = s.get("locate") or {}
    return format_cells([t, *s["quat"], *s["euler"], s["turn"], *s["measured"],
                   *(s["predicted"] or [None] * 3), *(s["error"] or [None] * 3), s["error_ut"],
                   s.get("error_deg"), *(m.get("measured") or [None] * 3),
                   *(m.get("predicted") or [None] * 3), *(m.get("error") or [None] * 3),
                   m.get("error_ut"), m.get("error_deg"),
                   s.get("calibration_on"), m.get("calibration_on"),
                   *(position or [None] * 3), locate.get("history_based"), s.get("reconnect_warning"),
                   s.get("ambient_ready"), s.get("coil_on"),
                   s.get("frame_id"), s.get("reference_id"), s.get("correction_id")])


