import numpy as np
import pytest

from franka_control_client.data_collection.pil_irl_vr_data_collection import (
    _normalize_policy_control_signal,
)


def test_policy_control_signal_accepts_single_action():
    action = np.arange(8, dtype=np.float32)

    normalized = _normalize_policy_control_signal(action)

    assert normalized.shape == (8,)
    np.testing.assert_array_equal(normalized, action)


def test_policy_control_signal_collapses_unbatched_chunk_to_first_action():
    chunk = np.arange(80, dtype=np.float32).reshape(10, 8)

    normalized = _normalize_policy_control_signal(chunk)

    assert normalized.shape == (8,)
    np.testing.assert_array_equal(normalized, chunk[0])


def test_policy_control_signal_collapses_batched_chunk_to_first_action():
    chunk = np.arange(80, dtype=np.float32).reshape(1, 10, 8)

    normalized = _normalize_policy_control_signal(chunk)

    assert normalized.shape == (8,)
    np.testing.assert_array_equal(normalized, chunk[0, 0])


def test_policy_control_signal_rejects_short_action():
    with pytest.raises(ValueError, match="Expected policy control signal size >= 8"):
        _normalize_policy_control_signal(np.zeros(7, dtype=np.float32))
