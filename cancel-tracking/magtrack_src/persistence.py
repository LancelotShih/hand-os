"""Strict JSON reads and complete-file replacement for calibration stores.

The caller owns operation ordering. Replacement is atomic per file, not across
several files or writers. Sudden power loss still depends on the filesystem.
"""

import json
import os
import tempfile
from contextlib import contextmanager
from pathlib import Path


def _invalid_constant(value):
    raise ValueError(f"{value} is not a finite JSON number")


def read_json_object(path):
    value = json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=_invalid_constant)
    if not isinstance(value, dict):
        raise ValueError("saved calibration must be a JSON object")
    return value


@contextmanager
def temporary_json(path, data):
    """Write/flush beside the destination; the caller chooses when to replace.

    This lets a live owner recheck its revision before replacement. Temporary
    files are removed on failed writes, stale revisions and failed replacement.
    """
    path = Path(path)
    encoded = json.dumps(data, indent=2, allow_nan=False)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                         prefix=path.name + ".", suffix=".tmp",
                                         dir=path.parent, delete=False) as output:
            temporary_path = Path(output.name)
            output.write(encoded)
            output.flush()
            os.fsync(output.fileno())
        yield temporary_path
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def write_json_atomic(path, data):
    with temporary_json(path, data) as temporary_path:
        temporary_path.replace(path)
