# geometry

Vector and quaternion math on plain tuples. No dependencies.

`vectors.py`: `add`, `sub`, `dot`, `norm`, `mat_vec`, `angle_deg`, `quat_norm`,
`quat_conj`, `quat_mul`, `quat_rotate` (`R v`), `quat_angle_deg`,
`quat_from_gravity`, `quat_to_euler` (yaw, pitch, roll in degrees, ZYX).

Vectors are 3-tuples and quaternions are `(w, x, y, z)`. A quaternion maps the
board to the reference/level frame.

- **Used by:** orientation, calibration, tracking, position, and tests.
