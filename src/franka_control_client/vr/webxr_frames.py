"""Frame conversion from the WebXR reference space to the Franka base frame.

WebXR is right-handed, +Y up, -Z forward (the direction the operator faced when the
reference space was created or last recentered). The Franka base frame is +X forward,
+Y left, +Z up. With the operator standing behind the robot facing base +X:

    x_b = -z_xr,  y_b = -x_xr,  z_b = y_xr

``yaw_deg`` is the heading of the operator's forward direction in the base frame,
counter-clockwise (seen from above) from +X: 90 means the operator faces base +Y.

Rotations are mapped as matrices (M R Mᵀ), never by permuting quaternion components.
Quaternions are scipy order (x, y, z, w).
"""

from typing import Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

# rows: base axes expressed in WebXR coordinates
_XR_TO_BASE_FACING_X = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
)


def xr_to_base_matrix(yaw_deg: float = 0.0, heading_xr_rad: float = 0.0) -> np.ndarray:
    """Rotation matrix M with ``v_base = M @ v_xr``.

    ``heading_xr_rad`` is the operator's forward direction in the WebXR frame (see
    ``heading_from_quat``), 0 meaning WebXR -Z. It is undone first, so the operator's
    forward maps to ``yaw_deg`` however the WebXR reference space happens to be oriented.
    """
    yaw = R.from_euler("z", yaw_deg, degrees=True).as_matrix()
    unheading = R.from_euler("y", -heading_xr_rad).as_matrix()
    return yaw @ _XR_TO_BASE_FACING_X @ unheading


def heading_from_quat(rot: Sequence[float], min_horizontal: float = 0.3) -> Optional[float]:
    """Heading (rad, CCW about WebXR +Y, 0 = -Z) of a pose's forward (-Z) axis.

    Returns None if the forward axis is too close to vertical (looking straight up or down)
    for the heading to be meaningful.
    """
    f = R.from_quat(rot).apply([0.0, 0.0, -1.0])
    if float(np.hypot(f[0], f[2])) < min_horizontal:
        return None
    # rotating -Z by phi about +Y gives (-sin phi, 0, -cos phi)
    return float(np.arctan2(-f[0], -f[2]))


def position_delta(m: np.ndarray, pos: Sequence[float], base_pos: Sequence[float]) -> np.ndarray:
    """Controller displacement since ``base_pos``, in the base frame."""
    return m @ (np.asarray(pos, dtype=float) - np.asarray(base_pos, dtype=float))


def rotation_delta(m: np.ndarray, rot: Sequence[float], base_rot: Sequence[float]) -> R:
    """Controller rotation since ``base_rot`` (world-frame delta), in the base frame."""
    delta_xr = R.from_quat(rot) * R.from_quat(base_rot).inv()
    return R.from_matrix(m @ delta_xr.as_matrix() @ m.T)


def clamp_position(pos: np.ndarray, lower: Sequence[float], upper: Sequence[float]) -> np.ndarray:
    return np.clip(pos, np.asarray(lower, dtype=float), np.asarray(upper, dtype=float))


def in_bounds(pos: Sequence[float], lower: Sequence[float], upper: Sequence[float]) -> bool:
    p = np.asarray(pos, dtype=float)
    return bool(np.all(p >= np.asarray(lower)) and np.all(p <= np.asarray(upper)))


def limit_step(
    prev_pos: np.ndarray,
    prev_rot: R,
    target_pos: np.ndarray,
    target_rot: R,
    max_pos_step: float,
    max_rot_step_rad: float,
) -> Tuple[np.ndarray, R]:
    """Move from ``prev`` toward ``target`` by at most the given translation / rotation."""
    d = target_pos - prev_pos
    dist = float(np.linalg.norm(d))
    pos = target_pos if dist <= max_pos_step else prev_pos + d * (max_pos_step / dist)

    rel = target_rot * prev_rot.inv()
    rotvec = rel.as_rotvec()
    angle = float(np.linalg.norm(rotvec))
    if angle <= max_rot_step_rad:
        rot = target_rot
    else:
        rot = R.from_rotvec(rotvec * (max_rot_step_rad / angle)) * prev_rot
    return pos, rot
