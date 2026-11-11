# web

Serves the page, streams samples, takes button clicks. Standard library `http.server`.

| File | Public names | Job |
|---|---|---|
| `server.py` | `make_server`, `local_ip`, `PAGE_FILES` | GET: page files, `/state`, `/stream` (server-sent events: samples, plus a `status` event every 50 ms), `/logs/<name>/<file>`. POST: `/capture`, `/tumble/{start,finish,cancel}`, `/calibration/{icm,mmc}/toggle`, `/calibration/reset`, `/log/{start,stop}`, `/locate/{point,cancel,remove,clear}`. |
| `feed.py` | `Feed` | Thread-safe buffer of the last 200 samples. Each tab resumes from its own `seq`. |
| `page/` | | The browser view, plain scripts with no build step. `draw.js` (helpers) → `sides.js` (one panel per sensor, from a `<template>`) → `board_view.js`, `sweep_view.js` (tumble ball), `chart.js`, `position_view.js` → `app.js` (stream, readouts, buttons) → `info.js` (`(i)` help text). Load order is set in `index.html`. |

- **Used by:** `earth_cancel.py`.
- **Talks to:** whatever object it's given as `tracker` (a `Pair`), `Feed` and `DataLog`. It imports none of them.
- **Adding a page file:** add it to `PAGE_FILES` and link it in `index.html`.
  `tests/test_server.py` checks both, and that every element id the scripts use exists.
- **Tests:** `tests/test_server.py`.
