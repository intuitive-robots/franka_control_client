"""End-effector pose representation shared by dataset conversion, training and inference.

Pose vector layout (10): [x, y, z, r6d_0..r6d_5, gripper]
    r6d = first two columns of the rotation matrix (Zhou et al. 2019), column-major:
          [R00, R10, R20, R01, R11, R21]

Chunk-relative actions: the positions of an action chunk predicted at step t are expressed relative to
the EE position observed at t (the start of the chunk). Orientation (R6D) and gripper stay absolute.
"""

import numpy as np
import torch
from scipy.spatial.transform import Rotation

POS = slice(0, 3)
R6D = slice(3, 9)
GRIPPER = 9
POSE_DIM = 10
POSE_NAMES = ["x", "y", "z", "r6d_0", "r6d_1", "r6d_2", "r6d_3", "r6d_4", "r6d_5", "gripper"]


def quat_xyzw_to_r6d(quat: np.ndarray) -> np.ndarray:
    """(..., 4) xyzw quaternion -> (..., 6) R6D."""
    shape = quat.shape[:-1]
    mat = Rotation.from_quat(quat.reshape(-1, 4)).as_matrix()  # (N, 3, 3)
    r6d = np.concatenate([mat[:, :, 0], mat[:, :, 1]], axis=-1)
    return r6d.reshape(*shape, 6).astype(np.float32)


def r6d_to_matrix(r6d: np.ndarray) -> np.ndarray:
    """(..., 6) R6D -> (..., 3, 3) rotation matrix (Gram-Schmidt)."""
    a1, a2 = r6d[..., :3], r6d[..., 3:6]
    b1 = a1 / np.linalg.norm(a1, axis=-1, keepdims=True)
    a2 = a2 - (b1 * a2).sum(-1, keepdims=True) * b1
    b2 = a2 / np.linalg.norm(a2, axis=-1, keepdims=True)
    b3 = np.cross(b1, b2)
    return np.stack([b1, b2, b3], axis=-1)


def r6d_to_quat_xyzw(r6d: np.ndarray) -> np.ndarray:
    """(..., 6) R6D -> (..., 4) xyzw quaternion."""
    shape = r6d.shape[:-1]
    mat = r6d_to_matrix(r6d.reshape(-1, 6))
    return Rotation.from_matrix(mat).as_quat().reshape(*shape, 4).astype(np.float32)


def ee_pose_to_vec(pos: np.ndarray, quat_xyzw: np.ndarray, gripper: np.ndarray) -> np.ndarray:
    """pos (..., 3), quat (..., 4), gripper (...,) or (..., 1) -> (..., 10)."""
    gripper = gripper.reshape(*pos.shape[:-1], 1)
    return np.concatenate([pos, quat_xyzw_to_r6d(quat_xyzw), gripper], axis=-1).astype(np.float32)


def to_chunk_relative(action: torch.Tensor, state: torch.Tensor) -> torch.Tensor:
    """action (B, T, 10) absolute, state (B, 10) at chunk start -> positions relative to state."""
    action = action.clone()
    action[..., POS] -= state[..., None, POS]
    return action


def from_chunk_relative(action: torch.Tensor, state: torch.Tensor) -> torch.Tensor:
    """Inverse of to_chunk_relative: (B, T, 10) or (B, 10) relative -> absolute."""
    action = action.clone()
    s = state[..., POS] if action.ndim == state.ndim else state[..., None, POS]
    action[..., POS] += s
    return action
