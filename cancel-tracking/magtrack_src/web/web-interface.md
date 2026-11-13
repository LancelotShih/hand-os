# Web interface

Serves the page, streams samples, takes button clicks. Standard library `http.server`.

| File | Public names | Job |
|---|---|---|
| `server.py` | `make_server`, `local_ip`, `PAGE_FILES` | GET: page files, `/state`, `/stream` (server-sent events: samples, plus a `status` event every 50 ms), `/logs/<name>/<file>`. POST: `/capture`, `/tumble/{start,finish,cancel}`, `/calibration/{icm,mmc}/toggle`, `/calibration/reset`, `/log/{start,stop}`, `/locate/{home,point,cancel,remove,clear}`. |
| `feed.py` | `Feed` | Locked buffer of the last 200 samples. Each open stream tracks its own `seq`; a new connection starts at zero and replays retained samples. |
| `page/` | | The browser view, plain scripts with no build step. `draw.js` (helpers) → `sides.js` (one panel per sensor, from a `<template>`) → `board_view.js`, `sweep_view.js` (tumble ball), `chart.js`, `field_view.js` (the coil's field), `position_view.js` → `app.js` (stream, measurement readouts, buttons) → `info.js` (`(i)` help text). Load order is set in `index.html`. |

- **Used by:** `earth_cancel.py`.
- **Talks to:** whatever object it's given as `tracker` (a `Pair`), `Feed` and `DataLog`. It imports none of them.
- **Adding a page file:** add it to `PAGE_FILES` and link it in `index.html`.
  `tests/test_server.py` checks both, and that every element id the scripts use exists.
- **Tests:** `tests/test_server.py`.

Status is assembled from separate snapshots. Feed connection fields are not
covered by the sample-buffer lock. Stream status is sent approximately every
50 ms, not on a real-time schedule. SSE IDs and `Last-Event-ID` recovery are not
implemented, and the browser charts use arrival time. No authentication is
implemented; the default bind address exposes controls on the bench network.

Browser scripts share globals and depend on HTML load order. The field renderer
reimplements the backend coil equation from rounded status values; its arrows
are an illustration rather than a separate accuracy check. Position POSTs
display rejected-request messages; the general controls currently ignore them.

The Position card colors the estimated movement per unit of field change but has no dedicated history or
blind-spot warning. Reconnection is reported in sensor status text; there is no
separate warning with a dismissal control.
