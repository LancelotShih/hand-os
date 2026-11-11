# recording

Records a run, draws it, scores it.

| File | Public names | Job |
|---|---|---|
| `data_log.py` | `DataLog`, `RAW_COLUMNS` | One folder per log in `data/logs/<start time>/`: `raw.csv` (every reading), `samples.csv` (every view sample), `info.json` (status at start). A disk error ends the log, never the sensor loop. |
| `log_plot.py` | `write_plot`, `read_samples`, `SAMPLE_COLUMNS` | Draws a log as a self-contained `plot.html`. CLI: `python3 -m magtrack_src.recording.log_plot data/logs/<name>`. |
| `compare.py` | `compare`, `report` | Scores a log: which magnetometer tracked Earth's direction better (only stretches turned ≥ `PASS_MIN_TURN_DEG`). CLI: `python3 -m magtrack_src.recording.compare data/logs/<name>`. |

- **Used by:** `earth_cancel.py` (the `DataLog`), `hardware/icm_loop.py` (`DataLog.add`),
  `web/server.py` (start, stop, serving log files).
- **Adding a logged field:** a column in `SAMPLE_COLUMNS` and the matching value in `data_log._sample_row`.
- **Tests:** `tests/test_data_log.py`, `tests/test_pair.py` (compare).
