# Recording and analysis

Records a run, draws it, scores it.

| File | Public names | Job |
|---|---|---|
| `data_log.py` | `DataLog`, `RAW_COLUMNS` | One folder per log in `data/logs/<start time>/`: `raw.csv` (every ICM loop reading and consumed MMC readings), `samples.csv` (one row per ICM display update), `info.json` (status at start). Handled write errors end the active log. |
| `log_plot.py` | `write_plot`, `read_samples`, `SAMPLE_COLUMNS` | Draws a log as a self-contained `plot.html`. CLI: `python3 -m magtrack_src.recording.log_plot data/logs/<name>`. |
| `compare.py` | `compare`, `report` | Scores a log: angles and three-component differences between measured and captured reference fields (only stretches turned ≥ `PASS_MIN_TURN_DEG`). CLI: `python3 -m magtrack_src.recording.compare data/logs/<name>`. |

- **Used by:** `earth_cancel.py` (the `DataLog`), `hardware/icm_loop.py` (`DataLog.add`),
  `web/server.py` (start, stop, serving log files).
- **Adding a logged field:** a column in `SAMPLE_COLUMNS` and the matching value in `data_log._sample_row`.
- **Tests:** `tests/test_data_log.py`, `tests/test_pair.py` (compare).

Writes and periodic flushes are synchronous under the log lock on the sensor
thread. A flush interval does not guarantee durability after power loss. MMC
values in `samples.csv` can repeat because the pair attaches its latest view;
MMC conversions overwritten before consumption are absent from `raw.csv`.

`info.json` is not a full configuration snapshot or event history. Later bias,
reference, calibration and model changes are not completely reconstructible.
Raw timestamps use ICM acquisition time; `mmc_age_s` offsets the consumed MMC
reading. Sample fields are rounded before logging.

Position columns contain coordinates only. Local sensitivity, ambiguity and
reliance on history are not recorded, and exported plots have no dedicated
position-ambiguity warning.

`compare` requires MMC readings and is intended for coil-off motion with pauses.
It selects readings using the ICM board-rotation angle, counts cached MMC values,
and estimates variation while apparently still from raw readings averaged into
50 ms intervals. The scores include timing, averaging, and orientation-estimation
effects as well as magnetometer behavior.
Plots/scoring use current config thresholds. Historical position columns have
no origin metadata, so older home-relative logs need separate interpretation.

The writer/scorer import schema/readers from `log_plot`; the column definitions and reading helpers is
currently owned by the plotting module.
