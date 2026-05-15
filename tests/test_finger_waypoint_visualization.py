import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from franka_control_client.policy_inference.finger_waypoints import (
    align_gripper_trajectory_start,
    build_finger_waypoint_positions,
)


def test_finger_waypoints_are_8cm_apart_when_open():
    actions = np.array(
        [[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0, 0.0]],
        dtype=np.float64,
    )

    left, right = build_finger_waypoint_positions(actions)

    np.testing.assert_allclose(left[0], [0.06, 0.2, 0.3])
    np.testing.assert_allclose(right[0], [0.14, 0.2, 0.3])
    assert np.linalg.norm(left[0] - right[0]) == pytest.approx(0.08)


def test_finger_waypoints_collapse_when_closed():
    actions = np.array(
        [[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0, 1.0]],
        dtype=np.float64,
    )

    left, right = build_finger_waypoint_positions(actions)

    np.testing.assert_allclose(left[0], actions[0, :3])
    np.testing.assert_allclose(right[0], actions[0, :3])


def test_finger_waypoints_follow_end_effector_rotation():
    sin_half = np.sqrt(0.5)
    cos_half = np.sqrt(0.5)
    actions = np.array(
        [[0.0, 0.0, 0.0, 0.0, 0.0, sin_half, cos_half, 0.0]],
        dtype=np.float64,
    )

    left, right = build_finger_waypoint_positions(actions)

    np.testing.assert_allclose(left[0], [0.0, -0.04, 0.0], atol=1e-7)
    np.testing.assert_allclose(right[0], [0.0, 0.04, 0.0], atol=1e-7)


def test_visual_gripper_trajectory_starts_from_current_closed_state():
    actions = np.array(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.5],
            [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )

    visual_actions = align_gripper_trajectory_start(
        actions, current_gripper_command=1.0
    )
    left, right = build_finger_waypoint_positions(visual_actions)

    np.testing.assert_allclose(left[0], right[0])
    assert visual_actions[0, 7] == pytest.approx(1.0)
    assert visual_actions[-1, 7] == pytest.approx(actions[-1, 7])
