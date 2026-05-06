import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from franka_control_client.data_collection.irl_wrapper import PandaArmDataWrapper


class FakeArm:
    current_state = {
        "EE_pos": [0.1, 0.2, 0.3],
        "EE_quat": [0.0, 0.0, 0.0, 1.0],
        "O_T_EE": list(range(16)),
        "q": list(range(7)),
        "dq": list(range(7)),
        "tau_ext_hat_filtered": list(range(7)),
        "O_F_ext_hat_K": list(range(6)),
    }


def test_panda_arm_data_wrapper_returns_full_arm_state_dict():
    state = PandaArmDataWrapper(FakeArm()).capture_step()

    assert set(FakeArm.current_state).issubset(state)
    assert state["q"].shape == (7,)
    assert state["EE_pos"].shape == (3,)
    assert all(isinstance(value, np.ndarray) for value in state.values())
