# Recording and analysis

Records a run, draws it, scores it.

| File | Public names | Job |
|---|---|---|
| `data_log.py` | `DataLog`, `RAW_COLUMNS` | One folder per log in `data/logs/<start time>/`: `raw.csv` (every ICM loop reading and consumed MMC readings), `samples.csv` (one row per ICM display update), `info.json` (status at start). Handled write errors end the active log. |
| `schema.py` | `RAW_COLUMNS`, `SAMPLE_COLUMNS`, `read_samples`, `format_cells`, `sample_row` | Shared CSV format, five-decimal cell formatting, view-row serialization and reading current/older recordings. |
| `log_plot.py` | `write_plot` | Draws a log as a self-contained `plot.html`. CLI: `python3 -m magtrack_src.recording.log_plot data/logs/<name>`. Re-exports `read_samples` and `SAMPLE_COLUMNS` for older callers. |
| `compare.py` | `compare`, `report` | Scores a log: angles and three-component differences between measured and captured reference fields (only stretches turned ≥ `PASS_MIN_TURN_DEG`). CLI: `python3 -m magtrack_src.recording.compare data/logs/<name>`. |

- **Used by:** `earth_cancel.py` (the `DataLog`), `hardware/icm_loop.py` (`DataLog.add`),
  `web/server.py` (start, stop, serving log files).
- **Adding a logged field:** a column in `schema.SAMPLE_COLUMNS` and the matching value in `schema.sample_row`.
- **Tests:** `tests/test_data_log.py`, `tests/test_pair.py` (compare).

Writes and periodic flushes are synchronous under the log lock on the sensor
thread. A flush interval does not guarantee durability after power loss. MMC
values in `samples.csv` can repeat because the pair attaches its latest view;
MMC conversions overwritten before consumption are absent from `raw.csv`.

`start` transfers file ownership to the active recording only after metadata,
CSV headers and header flushes succeed. An `OSError` closes local CSV handles,
sets the status error, and propagates to the caller. Partial attempt folders
are retained; retry uses a new folder. See the architecture's
[failure handling](../../docs/architecture.md#threads-locks-and-failure-handling)
for the operating behavior.

`info.json` is not a full configuration snapshot or event history. Later bias,
reference, calibration and model changes are not completely reconstructible.
Raw timestamps use ICM acquisition time; `mmc_age_s` offsets the consumed MMC
reading. Sample fields are rounded before logging.

Position samples include XYZ, quaternion orientation, and history/ambiguity
flags. The generated `plot.html` embeds an interactive 3D playback with a
time slider and a trace of estimated positions. Red dashed path sections mark
ambiguous estimates selected using the previous position; missing estimates
and long gaps break the trace. The path is an estimate, not ground truth, and
the lookup's sampled alternatives are not saved.

`compare` requires MMC readings and is intended for coil-off motion with pauses.
It selects readings using the ICM board-rotation angle, counts cached MMC values,
and estimates variation while apparently still from raw readings averaged into
50 ms intervals. The scores include timing, averaging, and orientation-estimation
effects as well as magnetometer behavior.
The reported still variation combines all supplied axes; it is not divided
by the number of axes. The three-axis regression test checks this definition.
Plots/scoring use current config thresholds. Historical position columns have
no origin metadata, so older home-relative logs need separate interpretation.

The writer, scorer and plot renderer import the shared format from `schema`.
Importing the schema or scorer does not load the plot renderer. `DataLog.stop`
still requests plot generation; separating that operation from recording is
not part of this extraction. JSON/CSV keys and legacy-column support are retained.
