"""Tests for the data collection GUI backend: IRL format, IRL -> LeRobot timeline, and the
session in simulation (no hardware or network needed)."""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("cv2")

from franka_control_client.data_collection_gui.config import CollectionConfig  # noqa: E402
from franka_control_client.data_collection_gui.irl_format import (  # noqa: E402
    IRLEpisodeWriter,
    detect_layout,
    find_episodes,
    follower_columns,
    gello_columns,
    summarize_episode,
)
from franka_control_client.data_collection_gui.irl_to_lerobot import (  # noqa: E402
    ConvertOptions,
    EpisodePlan,
    image_key,
)


def _arm_state(q):
    T = np.eye(4)
    T[:3, 3] = [0.4, 0.0, 0.5]
    return {
        "q": list(q), "dq": [0.0] * 7, "O_T_EE": T.T.reshape(-1).tolist(),
        "tau_ext_hat_filtered": [0.1] * 7, "O_F_ext_hat_K": [0.2] * 6,
        "EE_pos": [0.4, 0.0, 0.5], "EE_quat": [0.0, 0.0, 0.0, 1.0],
    }


def write_episode(path: Path, n=200, hz=100.0, cam_every=4, leader=True, timestamps=True) -> Path:
    """Synthetic GELLO episode: q0 = sample index, leader q0 = index + 0.5, one camera every
    `cam_every` samples whose pixel value encodes the sample index."""
    w = IRLEpisodeWriter(path, "FrankaPanda", "Gello" if leader else None, ["zed_right"], max_workers=2)
    for i in range(n):
        t = 100.0 + i / hz
        grip = {"position": 0.5, "current": 0.0, "commanded_position": 0.5}
        values = follower_columns("FrankaPanda", _arm_state([float(i)] + [0.0] * 6), grip, 0.5)
        if leader:
            values.update(gello_columns("Gello", {
                "gello_arm_state": {"joint_state": [i + 0.5] + [0.0] * 6},
                "gello_gripper_state": {"gripper": [0.25]},
            }, grip))
        else:
            values["FrankaPanda/gripper_command.pt"] = 1.0
        w.add_sample(t, values)
        if i % cam_every == 0:
            img = np.full((24, 32, 3), i % 256, dtype=np.uint8)
            w.add_frame("zed_right", t + 0.001, img)
    w.save({"mode": "gello" if leader else "kinesthetic", "task": "unit_test"})
    if not timestamps:
        (path / "camera_timestamps.pt").unlink()
    return path


def test_writer_layout_matches_irl_format(tmp_path):
    ep = write_episode(tmp_path / "task" / "ep0")
    assert (ep / "timestamps.pt").exists()
    for f in ("joint_pos.pt", "joint_vel.pt", "ee_pos.pt", "external_joint_torque.pt", "external_wrench.pt",
              "gripper_state.pt", "gripper_current.pt"):
        assert (ep / "FrankaPanda" / f).exists(), f
    assert torch.load(ep / "FrankaPanda" / "joint_pos.pt").shape == (200, 7)
    assert torch.load(ep / "FrankaPanda" / "ee_pos.pt").shape == (200, 16)
    assert torch.load(ep / "Gello" / "gripper_state.pt").shape == (200,)
    assert len(list((ep / "sensors" / "zed_right").glob("*.png"))) == 50
    layout = detect_layout(ep)
    assert (layout.follower, layout.leader, layout.leader_space, layout.cameras) == ("FrankaPanda", "Gello", "joint", ["zed_right"])
    assert find_episodes(tmp_path) == [ep]
    info = summarize_episode(ep, tmp_path)
    assert info["ok"] and info["samples"] == 200 and info["cameras"]["zed_right"]["hz"] == pytest.approx(25, rel=0.01)


def test_plan_aligns_cameras_to_robot_samples(tmp_path):
    ep = write_episode(tmp_path / "ep")
    plan = EpisodePlan(ep, ConvertOptions(fps=25, image_size=None))
    assert plan.state_space == "joint" and plan.action_source == "leader"
    frames = list(plan.frames())
    assert len(frames) == plan.n_frames == 49  # 2 s at 25 fps, minus the slot the last action needs
    for k, fr in enumerate(frames):
        assert fr["observation.state"][0] == pytest.approx(4 * k)  # 100 Hz robot, 25 fps grid
        assert fr["action"][0] == pytest.approx(4 * k + 0.5)  # leader at the same sample
        assert fr["observation.state"][7] == pytest.approx(0.5) and fr["action"][7] == pytest.approx(0.25)
        assert int(fr[image_key("zed_right")][0, 0, 0]) == (4 * k) % 256  # frame of the same loop step
    assert frames[0]["observation.force_torque"].shape == (14,)
    assert frames[0]["task"] == "unit test"


def test_plan_without_camera_timestamps_spreads_frames_evenly(tmp_path):
    ep = write_episode(tmp_path / "ep", timestamps=False)
    plan = EpisodePlan(ep, ConvertOptions(fps=25, image_size=None))
    assert plan.camera_hz["zed_right"] == pytest.approx(25.0)
    for k, fr in enumerate(plan.frames()):
        assert int(fr[image_key("zed_right")][0, 0, 0]) == (4 * k) % 256


def test_plan_next_state_actions_and_cartesian(tmp_path):
    ep = write_episode(tmp_path / "ep", leader=False)
    plan = EpisodePlan(ep, ConvertOptions(fps=50, image_size=(16, 12)))
    assert plan.action_source == "next_state"
    frames = list(plan.frames())
    for k, fr in enumerate(frames[:-1]):
        assert fr["action"][0] == pytest.approx(frames[k + 1]["observation.state"][0])
        assert fr["action"][7] == pytest.approx(1.0)  # recorded gripper command
    assert frames[0][image_key("zed_right")].shape == (12, 16, 3)
    cart = EpisodePlan(ep, ConvertOptions(fps=50, state_space="cartesian", image_size=None))
    st = next(cart.frames())["observation.state"]
    np.testing.assert_allclose(st[:7], [0.4, 0, 0.5, 0, 0, 0, 1], atol=1e-6)
    with pytest.raises(ValueError, match="leader"):
        EpisodePlan(ep, ConvertOptions(action_source="leader"))


def test_config_validation():
    with pytest.raises(ValueError):
        CollectionConfig.from_dict({"mode": "bogus"})
    with pytest.raises(ValueError):
        CollectionConfig.from_dict({"record_hz": 10, "lerobot_fps": 30})
    cfg = CollectionConfig.from_dict({"task": "pick", "data_dir": "/data", "cameras": [{"name": "zed_wrist", "hz": "15"}]})
    assert cfg.lerobot_dir == Path("/data/lerobot_data/pick") and cfg.instruction == "pick"
    assert cfg.cameras[0].hz == 15.0


@pytest.mark.parametrize("mode", ["gello", "kinesthetic"])
def test_sim_session_records_irl(tmp_path, mode):
    from franka_control_client.data_collection_gui.session import ActionError, CollectionSession, State

    cfg = CollectionConfig.from_dict(dict(
        sim=True, mode=mode, data_dir=str(tmp_path), task="t", record_hz=50,
        cameras=[{"name": "cam", "hz": 10}], save_irl=True, save_lerobot=False,
    ))
    s = CollectionSession(cfg)

    def wait(cond, timeout=20):
        t0 = time.time()
        while not cond():
            assert time.time() - t0 < timeout, s.state
            time.sleep(0.02)

    try:
        s.connect()
        wait(lambda: s.state == State.IDLE)
        with pytest.raises(ActionError):
            s.finish(save=True, reason="nothing to save")
        s.start()
        time.sleep(1.0)
        s.finish(save=True, reason="test")
        wait(lambda: s.state == State.IDLE)
        eps = s.episodes()["episodes"]
        assert len(eps) == 1 and eps[0]["ok"]
        assert 40 <= eps[0]["samples"] <= 60
        assert 8 <= eps[0]["cameras"]["cam"]["frames"] <= 12
        meta = json.loads((Path(eps[0]["path"]) / "metadata.json").read_text())
        assert meta["mode"] == mode and meta["measured_hz"] == pytest.approx(50, rel=0.1)
        assert eps[0]["leader"] == ("Gello" if mode == "gello" else None)
        s.start()
        time.sleep(0.3)
        s.finish(save=False, reason="test")
        wait(lambda: s.state == State.IDLE)
        assert len(s.episodes()["episodes"]) == 1
    finally:
        if s.state == State.IDLE:
            s.disconnect()
        s.shutdown()


def test_lerobot_append(tmp_path):
    pytest.importorskip("lerobot")
    from franka_control_client.data_collection_gui.irl_to_lerobot import convert
    from franka_control_client.data_collection_gui.lerobot_viewer import DatasetView

    eps = [write_episode(tmp_path / "irl" / f"ep{i}") for i in range(2)]
    opts = ConvertOptions(fps=25, image_size=None, use_videos=False)
    r1 = convert(eps[:1], tmp_path / "ds", opts)
    r2 = convert(eps[1:], tmp_path / "ds", opts, existing="append")
    assert not r1["skipped"] and not r2["skipped"]
    assert r2["written"][0]["episode_index"] == 1
    with pytest.raises(ValueError, match="exists"):
        convert(eps[:1], tmp_path / "ds", opts)
    view = DatasetView(tmp_path / "ds")
    s = view.summary()
    assert s["total_episodes"] == 2 and s["total_frames"] == 98
    ep = view.episode(1)
    assert ep["series"]["observation.state"]["values"][0][:3] == pytest.approx([0, 4, 8])
    assert view.frame_jpeg(1, image_key("zed_right"), 3)[:2] == b"\xff\xd8"


# ------------------------------------------------------------------ keyboard teleop
def test_keyboard_controller_directions_bounds_and_lead():
    from franka_control_client.keyboard.keyboard_controller import KeyboardController

    kb = KeyboardController(speed=0.1, z_speed=0.05, slow_factor=0.5, bounds=((0.3, -0.2, 0.1), (0.7, 0.2, 0.5)), max_lead=0.02)
    kb.anchor([0.5, 0.0, 0.3], [1, 0, 0, 0])
    kb.set_keys(["up", "left"])
    v = kb.direction()
    np.testing.assert_allclose(v, [0.1 / np.sqrt(2), 0.1 / np.sqrt(2), 0], atol=1e-9)  # diagonal: same speed
    kb.set_keys(["q"], slow=True)
    np.testing.assert_allclose(kb.direction(), [0, 0, 0.025])
    kb.yaw_deg = 180.0  # operator facing the robot: up = base -x
    kb.set_keys(["up"])
    np.testing.assert_allclose(kb.direction(), [-0.1, 0, 0], atol=1e-9)
    kb.yaw_deg = 0.0
    # workspace clamp (no measured pose -> no lead limit)
    for _ in range(100):
        pos, quat, _ = kb.step(0.1)
    assert pos[0] == pytest.approx(0.7) and list(quat) == [1, 0, 0, 0]
    # lead limit: the tool is blocked at z = 0.3, E held -> target stops 2 cm below it
    kb.anchor([0.5, 0.0, 0.3], [0, 0, 0, 1])
    kb.set_keys(["e"])
    for _ in range(50):
        pos, _, vel = kb.step(0.05, measured_pos=[0.5, 0.0, 0.3])
    assert pos[2] == pytest.approx(0.28) and vel[2] == pytest.approx(0.0)
    kb.set_keys(["q"])  # moving back toward the measured pose is always allowed
    pos, _, _ = kb.step(0.05, measured_pos=[0.5, 0.0, 0.3])
    assert pos[2] > 0.28
    # dead-man: no refresh -> released
    kb.timeout_s = 0.05
    kb.set_keys(["up"])
    time.sleep(0.1)
    assert kb.held_keys()[0] == frozenset() and not kb.direction().any()


def test_sim_keyboard_session_moves_and_records(tmp_path):
    from franka_control_client.data_collection_gui.session import CollectionSession, State

    cfg = CollectionConfig.from_dict(dict(sim=True, mode="keyboard", data_dir=str(tmp_path), task="push_t",
                                          record_hz=50, cameras=[], kb_speed=0.05))
    s = CollectionSession(cfg)

    def wait(cond, timeout=20):
        t0 = time.time()
        while not cond():
            assert time.time() - t0 < timeout, s.state
            time.sleep(0.02)

    try:
        s.connect()
        wait(lambda: s.state == State.IDLE)
        s.gripper(close=True)
        s.start()
        p0 = np.array(s.rig.arm.current_state["EE_pos"])
        t0 = time.time()
        while time.time() - t0 < 0.8:
            s.do("keys", {"keys": ["left"]})
            time.sleep(0.1)
        s.do("keys", {"keys": []})
        time.sleep(0.4)
        p1 = np.array(s.rig.arm.current_state["EE_pos"])
        assert p1[1] - p0[1] == pytest.approx(0.04, abs=0.008) and abs(p1[0] - p0[0]) < 0.003
        s.finish(save=True, reason="test")
        wait(lambda: s.state == State.IDLE)
        ep = Path(s.episodes()["episodes"][0]["path"])
        assert detect_layout(ep).leader == "Keyboard" and detect_layout(ep).leader_space == "cartesian"
        target = torch.load(ep / "Keyboard" / "EE_pos.pt").numpy()
        assert target[-1, 1] - target[0, 1] == pytest.approx(0.04, abs=0.008)
        assert torch.load(ep / "Keyboard" / "gripper_state.pt").numpy().min() == 1.0  # gripper kept closed
        assert s.rig.gripper.current_state["commanded_position"] == 1.0  # episode end did not open it
        plan = EpisodePlan(ep, ConvertOptions(fps=10, image_size=None))
        assert (plan.state_space, plan.action_source) == ("cartesian", "leader")
    finally:
        if s.state == State.IDLE:
            s.disconnect()
        s.shutdown()
