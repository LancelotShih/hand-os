# Vector and quaternion math

Vector and quaternion math on plain tuples. Standard library only.

`vectors.py`: `add`, `sub`, `dot`, `norm`, `mat_vec`, `angle_deg`, `quat_norm`,
`quat_conj`, `quat_mul`, `quat_rotate` (`R v`), `quat_angle_deg`,
`quat_between`, `quat_from_gravity`, `quat_to_euler` (yaw, pitch, roll in degrees, ZYX).

Vectors store x, y and z components as 3-tuples. Quaternions represent rotation
using four numbers, `(w, x, y, z)`. A quaternion maps the
board to the reference/level frame in tracking; the helpers themselves have no
an indication of which coordinate axes a value uses. Callers must supply compatible frames and unit quaternions for
`quat_rotate`. `angle_deg` returns zero when either vector is zero, which is a
fallback convention rather than a defined field direction.

- **Used by:** orientation, calibration, tracking, position, and tests.
