from typing import Sequence

import numpy as np


FINGER_MAX_OPEN_WIDTH_M = 0.08
# Robotiq fingers move along +/-Y in robotiq_base. The base is mounted 90 deg
# around the EE Z axis, so the separation axis in the action EE frame is -X.
FINGER_SEPARATION_AXIS_EE = np.array([-1.0, 0.0, 0.0], dtype=np.float64)


def build_finger_waypoint_positions(
    action_chunk: np.ndarray,
    *,
    max_open_width: float = FINGER_MAX_OPEN_WIDTH_M,
    separation_axis_ee: Sequence[float] = FINGER_SEPARATION_AXIS_EE,
) -> tuple[np.ndarray, np.ndarray]:
    """Return left/right finger positions for [pos, quat_xyzw, gripper] actions."""
    actions = np.asarray(action_chunk, dtype=np.float64)
    if actions.size == 0:
        empty = np.empty((0, 3), dtype=np.float64)
        return empty, empty
    if actions.ndim != 2 or actions.shape[1] < 8:
        raise ValueError("action_chunk must have shape (N, >=8)")
    if not np.all(np.isfinite(actions[:, :8])):
        raise ValueError("action_chunk must contain finite numeric values")
    if max_open_width < 0.0:
        raise ValueError("max_open_width must be non-negative")

    ee_positions = actions[:, :3]
    ee_quats_xyzw = actions[:, 3:7]
    rotation_matrices = _quat_xyzw_to_rotation_matrices(ee_quats_xyzw)

    gripper_commands = np.clip(actions[:, 7], 0.0, 1.0)
    finger_widths = (1.0 - gripper_commands) * float(max_open_width)
    axis = _normalized_vector(separation_axis_ee)
    local_offsets = np.outer(finger_widths * 0.5, axis)
    world_offsets = np.einsum("nij,nj->ni", rotation_matrices, local_offsets)

    return ee_positions + world_offsets, ee_positions - world_offsets


def align_gripper_trajectory_start(
    action_chunk: np.ndarray, current_gripper_command: float
) -> np.ndarray:
    """Shift visual gripper commands so the first waypoint matches live state."""
    actions = np.array(action_chunk, dtype=np.float64, copy=True)
    if actions.size == 0:
        return actions.reshape(0, 8) if actions.ndim == 1 else actions
    if actions.ndim != 2 or actions.shape[1] < 8:
        raise ValueError("action_chunk must have shape (N, >=8)")
    current = float(np.clip(current_gripper_command, 0.0, 1.0))
    blend_to_policy = np.linspace(1.0, 0.0, actions.shape[0], dtype=np.float64)
    actions[:, 7] = np.clip(
        actions[:, 7] + (current - actions[0, 7]) * blend_to_policy,
        0.0,
        1.0,
    )
    return actions


def _quat_xyzw_to_rotation_matrices(quaternions: np.ndarray) -> np.ndarray:
    quats = np.asarray(quaternions, dtype=np.float64)
    if quats.ndim != 2 or quats.shape[1] != 4:
        raise ValueError("quaternions must have shape (N, 4)")

    norms = np.linalg.norm(quats, axis=1)
    if not np.all(np.isfinite(norms)) or np.any(norms <= 1e-9):
        raise ValueError("quaternions must be finite and non-zero")

    quats = quats / norms[:, None]
    x, y, z, w = quats.T

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    rotations = np.empty((quats.shape[0], 3, 3), dtype=np.float64)
    rotations[:, 0, 0] = 1.0 - 2.0 * (yy + zz)
    rotations[:, 0, 1] = 2.0 * (xy - wz)
    rotations[:, 0, 2] = 2.0 * (xz + wy)
    rotations[:, 1, 0] = 2.0 * (xy + wz)
    rotations[:, 1, 1] = 1.0 - 2.0 * (xx + zz)
    rotations[:, 1, 2] = 2.0 * (yz - wx)
    rotations[:, 2, 0] = 2.0 * (xz - wy)
    rotations[:, 2, 1] = 2.0 * (yz + wx)
    rotations[:, 2, 2] = 1.0 - 2.0 * (xx + yy)
    return rotations


def _normalized_vector(vector: Sequence[float]) -> np.ndarray:
    arr = np.asarray(vector, dtype=np.float64).reshape(-1)
    if arr.size != 3:
        raise ValueError("separation_axis_ee must have exactly 3 values")
    norm = np.linalg.norm(arr)
    if not np.isfinite(norm) or norm <= 1e-9:
        raise ValueError("separation_axis_ee must be finite and non-zero")
    return arr / norm
