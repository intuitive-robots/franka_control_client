import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from franka_control_client.control_pair.cartesian_policy_panda_control_pair import (
    PolicyPandaRobotiqDeltaCartesianControlPair,
)


class FakeArm:
    current_state = {
        "EE_pos": np.array([100.0, 100.0, 100.0], dtype=np.float32),
        "EE_quat": np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32),
    }
    current_ee_position = current_state["EE_pos"]
    current_ee_rotation = current_state["EE_quat"]

    def send_cartesian_pose_command(self, *args, **kwargs):
        pass

    def set_franka_arm_control_mode(self, *args, **kwargs):
        pass


class FakeGripper:
    def send_grasp_command(self, *args, **kwargs):
        pass


def make_pair(chunk_size=10):
    return PolicyPandaRobotiqDeltaCartesianControlPair(
        FakeArm(), FakeGripper(), action_chunk_size=chunk_size
    )


def test_batched_absolute_cartesian_chunk_is_accepted_without_delta_conversion():
    pair = make_pair(chunk_size=10)
    chunk = np.arange(80, dtype=np.float32).reshape(1, 10, 8)

    pair.update_action_chunk(chunk)

    stored_chunk = pair.action_buffer.get_action()
    assert stored_chunk.shape == (10, 8)
    np.testing.assert_array_equal(stored_chunk, chunk[0])


def test_unbatched_absolute_cartesian_chunk_is_accepted():
    pair = make_pair(chunk_size=10)
    chunk = np.arange(80, dtype=np.float32).reshape(10, 8)

    pair.update_action_chunk(chunk)

    stored_chunk = pair.action_buffer.get_action()
    assert stored_chunk.shape == (10, 8)
    np.testing.assert_array_equal(stored_chunk, chunk)


def test_single_absolute_cartesian_action_is_accepted_when_chunk_size_is_one():
    pair = make_pair(chunk_size=1)
    action = np.arange(8, dtype=np.float32)

    pair.update_action_chunk(action)

    stored_chunk = pair.action_buffer.get_action()
    assert stored_chunk.shape == (1, 8)
    np.testing.assert_array_equal(stored_chunk[0], action)


def test_incoming_action_chunk_horizon_can_differ_from_initial_chunk_size():
    pair = make_pair(chunk_size=10)
    chunk = np.arange(240, dtype=np.float32).reshape(30, 8)

    pair.update_action_chunk(chunk)

    stored_chunk = pair.action_buffer.get_action()
    assert stored_chunk.shape == (30, 8)
    np.testing.assert_array_equal(stored_chunk, chunk)


def test_invalid_cartesian_chunk_shapes_raise_clear_errors():
    pair = make_pair(chunk_size=10)

    with pytest.raises(ValueError, match="Expected absolute cartesian action chunk shape"):
        pair.update_action_chunk(np.zeros((1, 1, 10, 8), dtype=np.float32))

    with pytest.raises(ValueError, match="Expected absolute cartesian action size >= 8"):
        pair.update_action_chunk(np.zeros((10, 7), dtype=np.float32))
