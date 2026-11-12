"""IRLLeRobotDataCollection with fake hardware: frames, action offset, save/drop/discard, resume."""

import numpy as np
import pytest

pytest.importorskip("lerobot")

from franka_control_client.data_collection.data_collection_manager import DataCollectionState  # noqa: E402
from franka_control_client.data_collection.irl_lerobot_data_collection import (  # noqa: E402
    IRLLeRobotDataCollection,
)
from franka_control_client.data_collection.irl_wrapper import (  # noqa: E402
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
)

CAM_HW = (96, 128)  # camera (H, W)
OUT = (64, 48)  # stored (W, H)


class _Fake(IRL_HardwareDataWrapper):
    def __init__(self, hw_type, hw_name, fn):
        super().__init__(hw_type, hw_name)
        self.fn = fn

    def capture_step(self):
        return self.fn()

    def discard(self):
        pass

    def reset(self):
        pass

    def close(self):
        pass


class _FakeCamera:
    size = CAM_HW

    def __init__(self):
        self.rng = np.random.default_rng(0)

    def get_image(self):
        return self.rng.integers(0, 255, (*CAM_HW, 3), dtype=np.uint8)


def _manager(root, image_size=OUT, **kw):
    step = {"n": 0}

    def leader():
        step["n"] += 1  # leader is sampled first each step: n = step index + 1
        return {"EE_pos": [float(step["n"]), 0, 0], "EE_quat": [0, 0, 0, 1], "gripper_width": [1.0]}

    def arm():
        return {
            "q": [float(step["n"])] * 7, "EE_pos": [0.5, 0, 0.4], "EE_quat": [1, 0, 0, 0],
            "tau_ext_hat_filtered": [0.0] * 7, "O_F_ext_hat_K": [0.0] * 6,
        }

    collectors = [
        _Fake("leader_robot", "webxr", leader),
        _Fake("follower_arm", "FrankaPanda", arm),
        _Fake("follower_gripper", "FrankaPanda", lambda: {"position": 0.0, "current": 0.2}),
        ImageDataWrapper(_FakeCamera(), hw_name="zed_left"),
        ImageDataWrapper(_FakeCamera(), hw_name="zed_wrist"),
    ]
    kw.setdefault("vcodec", "h264")
    return IRLLeRobotDataCollection(collectors, root, "pick up the knife", fps=50, image_size=image_size, **kw)


def _record(m, steps, end="s"):
    m._handle_keypress("n")
    for _ in range(steps):
        m._collect_step()
    m._handle_keypress(end)
    m._reset_to_waiting()


def _load(root):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    return LeRobotDataset("t", root=root)


def test_records_episodes_with_shifted_action(tmp_path):
    root = tmp_path / "ds"
    m = _manager(root)
    _record(m, 10)
    _record(m, 6)
    m._handle_keypress("q")  # writes the second episode and finalizes
    assert m._state_machine.state == DataCollectionState.EXITING

    ds = _load(root)
    assert ds.meta.total_episodes == 2
    assert ds.meta.total_frames == (10 - 1) + (6 - 1)  # the last action_offset steps have no action
    assert ds.fps == 50
    item = ds[0]
    assert tuple(item["observation.images.left"].shape) == (3, OUT[1], OUT[0])
    assert "observation.images.wrist" in item
    for i in range(9):
        f = ds[i]
        # observation from step i, action = leader command of step i + 1
        assert f["observation.state"][0].item() == pytest.approx(i + 1)
        assert f["action"][0].item() == pytest.approx(i + 2)
    assert f["observation.state"][7].item() == pytest.approx(0.0018)  # gripper 0.0 padded
    assert ds[0]["task"] == "pick up the knife"


def test_x_drops_only_the_unwritten_episode_and_discard_keeps_nothing(tmp_path):
    root = tmp_path / "ds"
    m = _manager(root)
    _record(m, 5)
    m._handle_keypress("x")  # drops it before it is written
    _record(m, 5, end="d")  # discarded
    _record(m, 7)
    m._handle_keypress("n")  # writes the 7-step episode
    m._handle_keypress("s")
    m._reset_to_waiting()  # 0 frames: nothing kept
    m._handle_keypress("x")  # nothing to drop, written episodes stay
    m._close()
    ds = _load(root)
    assert ds.meta.total_episodes == 1 and ds.meta.total_frames == 6


def test_quit_while_recording_discards_running_episode_and_resume_appends(tmp_path):
    root = tmp_path / "ds"
    m = _manager(root)
    _record(m, 5)
    m._handle_keypress("n")  # writes the first one
    m._collect_step()
    m._collect_step()
    m._close()  # Ctrl-C path: the running episode is dropped
    assert _load(root).meta.total_episodes == 1

    m2 = _manager(root)
    _record(m2, 4)
    m2._close()
    ds = _load(root)
    assert ds.meta.total_episodes == 2 and ds.meta.total_frames == 4 + 3
    assert ds[ds.meta.total_frames - 1]["episode_index"].item() == 1


def test_resume_rejects_a_different_image_size(tmp_path):
    root = tmp_path / "ds"
    m = _manager(root)
    _record(m, 3)
    m._close()
    with pytest.raises(ValueError, match="different features"):
        _manager(root, image_size=(32, 32))


def test_loop_holds_the_rate(tmp_path):
    import time

    m = _manager(tmp_path / "ds")
    m._handle_keypress("n")
    t0 = time.perf_counter()
    for _ in range(50):
        m._collect_step()
    dt = time.perf_counter() - t0
    assert 0.95 <= dt <= 1.1  # 50 steps at 50 Hz, no drift
    assert m._overruns == 0
    m._close()
