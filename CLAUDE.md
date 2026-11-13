# hand-os

Tooling for hand-os, on a Raspberry Pi.

| Folder | What it is |
|---|---|
| `cancel-tracking/` | Tracks an ICM-20948 in 3D around an always-on electromagnet using reference-subtracted magnetic readings. See `cancel-tracking/CLAUDE.md` before working there. |

Setup, operation, current behavior, and package-guide links are maintained in
`cancel-tracking/docs/architecture.md`.
Agreed future behavior and proposed work are in `cancel-tracking/docs/next-steps.md`;
those requirements must not be described as already implemented.

Older experiments (`icm20948-tool`, `icm20948-visualizer`, `em-driver`, `em-localizer`,
`em-steady`) were removed in commit `0c52c77`. They are only in git history. Their
magnetometer axis map is wrong, so don't revive their math without checking it.
