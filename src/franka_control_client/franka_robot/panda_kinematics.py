"""Minimal Franka Panda kinematics (FK + damped least squares IK).

Only needed to derive joint configurations for small Cartesian offsets
(e.g. "same pose, but 20 cm higher"), since the arm interface only exposes
joint position moves.

Uses the modified (Craig) DH parameters published by Franka Emika. The
returned frame is the flange (link8). Any fixed tool offset (Robotiq
gripper) does not matter for pure Cartesian offsets as long as the
orientation is kept constant, because the tool transform is then simply
carried along.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

# (a, d, alpha) per joint, modified DH, plus the fixed flange transform.
_DH = [
    (0.0, 0.333, 0.0),
    (0.0, 0.0, -np.pi / 2),
    (0.0, 0.316, np.pi / 2),
    (0.0825, 0.0, np.pi / 2),
    (-0.0825, 0.384, -np.pi / 2),
    (0.0, 0.0, np.pi / 2),
    (0.088, 0.0, np.pi / 2),
]
_FLANGE_D = 0.107

JOINT_LIMITS_LOWER = np.array(
    [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
)
JOINT_LIMITS_UPPER = np.array(
    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
)


def _dh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def forward_kinematics(joint_positions: Sequence[float]) -> np.ndarray:
    """Return the 4x4 flange pose for the given 7 joint angles."""
    q = np.asarray(joint_positions, dtype=np.float64).reshape(-1)
    if q.size != 7:
        raise ValueError(f"Expected 7 joint angles, got {q.size}")
    pose = np.eye(4)
    for (a, d, alpha), theta in zip(_DH, q):
        pose = pose @ _dh_transform(a, d, alpha, theta)
    pose = pose @ _dh_transform(0.0, _FLANGE_D, 0.0, 0.0)
    return pose


def jacobian(joint_positions: Sequence[float]) -> np.ndarray:
    """Return the 6x7 geometric Jacobian of the flange, in base frame."""
    q = np.asarray(joint_positions, dtype=np.float64).reshape(-1)
    pose = np.eye(4)
    origins = []
    axes = []
    for (a, d, alpha), theta in zip(_DH, q):
        # In modified DH the joint rotates about z of the frame reached after
        # the fixed Trans_x(a) * Rot_x(alpha) part of the link transform.
        pose = pose @ _dh_transform(a, 0.0, alpha, 0.0)
        axes.append(pose[:3, 2].copy())
        origins.append(pose[:3, 3].copy())
        pose = pose @ _dh_transform(0.0, d, 0.0, theta)
    pose = pose @ _dh_transform(0.0, _FLANGE_D, 0.0, 0.0)
    p_ee = pose[:3, 3]

    jac = np.zeros((6, 7))
    for i in range(7):
        jac[:3, i] = np.cross(axes[i], p_ee - origins[i])
        jac[3:, i] = axes[i]
    return jac


def _rotation_error(r_target: np.ndarray, r_current: np.ndarray) -> np.ndarray:
    r_err = r_target @ r_current.T
    angle = np.arccos(np.clip((np.trace(r_err) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array(
        [
            r_err[2, 1] - r_err[1, 2],
            r_err[0, 2] - r_err[2, 0],
            r_err[1, 0] - r_err[0, 1],
        ]
    ) / (2.0 * np.sin(angle))
    return axis * angle


def inverse_kinematics(
    target_pose: np.ndarray,
    initial_joint_positions: Sequence[float],
    max_iterations: int = 300,
    position_tolerance: float = 1e-4,
    rotation_tolerance: float = 1e-3,
    damping: float = 0.05,
    step_scale: float = 0.5,
) -> np.ndarray:
    """Damped least squares IK, seeded with ``initial_joint_positions``.

    Raises RuntimeError if it does not converge inside the joint limits.
    """
    q = np.asarray(initial_joint_positions, dtype=np.float64).reshape(-1).copy()
    r_target = target_pose[:3, :3]
    p_target = target_pose[:3, 3]

    for _ in range(max_iterations):
        pose = forward_kinematics(q)
        p_err = p_target - pose[:3, 3]
        r_err = _rotation_error(r_target, pose[:3, :3])
        if (
            np.linalg.norm(p_err) < position_tolerance
            and np.linalg.norm(r_err) < rotation_tolerance
        ):
            return np.clip(q, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)

        err = np.concatenate([p_err, r_err])
        jac = jacobian(q)
        dq = jac.T @ np.linalg.solve(
            jac @ jac.T + (damping**2) * np.eye(6), err
        )
        q = np.clip(
            q + step_scale * dq, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER
        )

    raise RuntimeError(
        "Inverse kinematics did not converge "
        f"(position error {np.linalg.norm(p_err):.4f} m, "
        f"rotation error {np.linalg.norm(r_err):.4f} rad)"
    )


def joint_position_with_cartesian_offset(
    joint_positions: Sequence[float],
    offset: Sequence[float],
) -> np.ndarray:
    """Joint angles for the same orientation, shifted by ``offset`` (x, y, z) in base frame."""
    pose = forward_kinematics(joint_positions)
    target = pose.copy()
    target[:3, 3] = pose[:3, 3] + np.asarray(offset, dtype=np.float64).reshape(3)
    return inverse_kinematics(target, joint_positions)
