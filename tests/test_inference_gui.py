"""Tests for the inference GUI backend, using the simulated robot and the
dummy policy server (no hardware or network needed)."""

from __future__ import annotations

import json
import threading
import time

import numpy as np
import pytest

from franka_control_client.inference_gui.config import CameraConfig, SessionConfig
from franka_control_client.inference_gui.policy_server import ACTION_SPACES, DummyPolicy, PolicyServer
from franka_control_client.inference_gui.recorder import read_batch_index
from franka_control_client.inference_gui.session import (
    ActionError,
    InferenceSession,
    SessionState,
    resample_chunk,
    speed_limit_chunk,
)


def test_resample_joint_reaches_last_target():
    rows = np.zeros((4, 8))
    rows[:, 0] = [0.1, 0.2, 0.3, 0.4]
    rows[:, 7] = [0, 0, 1, 1]
    out = resample_chunk(rows, (1 + np.arange(4)) / 10.0, np.zeros(7), 50.0, ACTION_SPACES["joint"])
    assert out.shape == (20, 8)
    assert out[-1, 0] == pytest.approx(0.4)
    assert out[4, 0] == pytest.approx(0.1)  # t = 0.1 s lands on row 0
    assert np.all(np.diff(out[:, 0]) > 0)
    assert out[0, 7] == 0 and out[-1, 7] == 1


def test_resample_cartesian_keeps_unit_quaternions():
    anchor = np.array([0.5, 0, 0.4, 1, 0, 0, 0], dtype=float)
    rows = np.array([[0.5, 0.0, 0.4, 0, 1, 0, 0, 1.0], [0.5, 0.0, 0.4, 0, -1, 0, 0, 1.0]])
    out = resample_chunk(rows, np.array([0.1, 0.2]), anchor, 20.0, ACTION_SPACES["cartesian"])
    assert np.allclose(np.linalg.norm(out[:, 3:7], axis=1), 1.0)


def test_config_roundtrip_and_validation(tmp_path):
    cfg = SessionConfig(cameras=[CameraConfig("cam", "observation.images.ext")])
    path = tmp_path / "cfg.json"
    cfg.save(path)
    assert SessionConfig.load(path) == cfg
    with pytest.raises(ValueError):
        cfg.merged({"control_pair": "torque"})
    assert cfg.merged({"control_hz": "30"}).control_hz == 30.0


def test_home_q_config():
    cfg = SessionConfig()
    assert cfg.home_q == [0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0]
    assert cfg.merged({"home_q": "0.1, 0, 0, -2, 0, 2, 0.5"}).home_q == [0.1, 0, 0, -2, 0, 2, 0.5]
    for bad in ([0.0] * 6, [0, 0, 0, 0, 0, 2, 0], [0, 0, 0, -2, 0, 2, float("nan")], [0, 0, 0, -2, 0, 2, "x"]):
        with pytest.raises(ValueError):
            cfg.merged({"home_q": bad})  # wrong length / q3 above its limit / nan / not a number


@pytest.fixture()
def sim_session(tmp_path):
    server = PolicyServer(DummyPolicy(action_space="joint", latency_s=0.01), "127.0.0.1", 0)
    # bind to a free port
    import socket

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server.port = port
    threading.Thread(target=server.serve_forever, daemon=True).start()
    time.sleep(0.2)
    cfg = SessionConfig(
        sim=True, control_hz=50, policy_port=port, cameras=[CameraConfig("ext")],
        save_root=str(tmp_path), batch_name="batch", task="test",
    )
    session = InferenceSession(cfg)
    yield session
    session.shutdown()
    server.shutdown()


def _wait(session, *states, timeout=60.0):  # first connect imports torch, slow on a cold cache
    end = time.time() + timeout
    while session.state not in states and time.time() < end:
        time.sleep(0.05)
    assert session.state in states, (session.state, list(session._logs))


def test_full_rollout_save_and_edit(sim_session):
    s = sim_session
    s.connect()
    _wait(s, SessionState.IDLE)
    with pytest.raises(ActionError):
        s.update_config({"sim": False})  # connection field while connected
    s.start_episode(notes="cup left")
    _wait(s, SessionState.RUNNING)
    time.sleep(1.5)
    s.stop_episode("stopped by user")
    assert s.state == SessionState.REVIEW
    folder = s.save_episode("", "my description", "success", wait=True)["folder"]
    data = np.load(f"{folder}/episode.npz")
    assert data["state/q"].shape[1] == 7 and len(data["state/t"]) > 10
    assert data["command/arm"].shape[0] > 10
    assert data["policy/action_raw"].shape[0] >= 1
    meta = json.load(open(f"{folder}/meta.json"))
    assert meta["description"] == "my description"
    assert meta["notes"]["pre_start_notes"] == "cup left"
    assert meta["name"] == "batch_ep000"

    from pathlib import Path

    s.edit_saved(Path(folder).name, {"description": "edited", "outcome": "failure"})
    row = read_batch_index(s.batch_dir)[0]
    assert row["description"] == "edited" and row["outcome"] == "failure"


def test_estop_reason_and_discard(sim_session):
    s = sim_session
    s.connect()
    _wait(s, SessionState.IDLE)
    s.start_episode()
    _wait(s, SessionState.RUNNING)
    time.sleep(0.5)
    s.software_estop()
    assert s.state == SessionState.REVIEW
    assert s.episode["stop_reason"] == "software e-stop"
    s.discard_episode()
    assert s.state == SessionState.IDLE
    assert read_batch_index(s.batch_dir) == []


def test_lerobot_rollouts_and_merge(sim_session, tmp_path):
    pytest.importorskip("lerobot")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    s = sim_session
    s.update_config({"save_lerobot": True, "dataset_fps": 10, "dataset_image_width": 64, "dataset_image_height": 48})
    s.connect()
    _wait(s, SessionState.IDLE)
    outcomes = ["success", "failure", "success"]
    for outcome in outcomes:
        s.start_episode()
        _wait(s, SessionState.RUNNING)
        time.sleep(1.5)
        s.stop_episode("stopped by user")
        folder = s.save_episode("", outcome, outcome, wait=True)["folder"]
        meta = json.load(open(f"{folder}/meta.json"))
        assert meta["lerobot"]["frames"] >= 10, meta["lerobot"]

    ds = LeRobotDataset("local/x", root=f"{folder}/lerobot")
    assert ds.num_episodes == 1
    item = ds[0]
    assert tuple(item["observation.state"].shape) == (8,)
    assert tuple(item["action"].shape) == (8,)
    assert tuple(item["observation.force_torque"].shape) == (14,)
    assert tuple(item["observation.images.ext"].shape) == (3, 48, 64)

    out = tmp_path / "merged"
    status = s.export_dataset({"outcomes": ["success"], "output": str(out)}, wait=True)
    assert status["error"] is None, status
    assert status["result"]["episodes"] == 2
    merged = LeRobotDataset("local/merged", root=out)
    assert merged.num_episodes == 2
    index = json.load(open(out / "meta" / "rollouts.json"))
    assert [r["outcome"] for r in index] == ["success", "success"]
    # an existing output is not overwritten unless asked
    status = s.export_dataset({"outcomes": ["success"], "output": str(out)}, wait=True)
    assert "exists" in status["error"]


def test_speed_limit_chunk_caps_joint_speed():
    rows = np.zeros((8, 8))
    rows[:, 0] = 0.691  # far target, as from an out-of-distribution first action
    rows[:, 1] = 0.01   # small move, must pass unchanged
    rows[:, 7] = 1.0
    t = (1 + np.arange(8)) / 15.0
    out, limited = speed_limit_chunk(rows, t, np.zeros(7), ACTION_SPACES["joint"], 1.0, 0.15)
    assert limited
    assert np.all(np.abs(np.diff(np.r_[0.0, out[:, 0]])) <= 1.0 / 15.0 + 1e-9)
    assert out[-1, 0] == pytest.approx(8 / 15.0)  # moved toward the target at the cap
    assert np.allclose(out[:, 1], 0.01) and np.allclose(out[:, 7], 1.0)
    same, limited = speed_limit_chunk(rows * 0.01, t, np.zeros(7), ACTION_SPACES["joint"], 1.0, 0.15)
    assert not limited and np.allclose(same, rows * 0.01)


class _JumpPolicy(DummyPolicy):
    """Always asks for joint 0 at +0.69 rad from the current state."""

    def infer(self, observation):
        state = np.asarray(observation["observation.state"], dtype=np.float64)
        chunk = np.repeat(state[None, :8], 16, axis=0)
        chunk[:, 0] += 0.691
        return chunk


@pytest.mark.parametrize("mode", ["stop", "limit"])
def test_first_step_jump_mode(tmp_path, mode):
    import socket

    with socket.socket() as sk:
        sk.bind(("127.0.0.1", 0))
        port = sk.getsockname()[1]
    server = PolicyServer(_JumpPolicy(action_space="joint"), "127.0.0.1", port)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    time.sleep(0.2)
    cfg = SessionConfig(
        sim=True, control_hz=50, policy_port=port, cameras=[CameraConfig("ext")],
        save_root=str(tmp_path), batch_name="b", task="t",
        first_step_jump_mode=mode, max_joint_speed=0.5,
    )
    s = InferenceSession(cfg)
    try:
        s.connect()
        _wait(s, SessionState.IDLE)
        s.start_episode()
        if mode == "stop":
            _wait(s, SessionState.REVIEW, timeout=10)
            assert s.episode["stop_reason"].startswith("safety")
            return
        _wait(s, SessionState.RUNNING)
        time.sleep(2.0)
        assert s.state == SessionState.RUNNING  # not stopped by the jump
        assert s.telemetry.counters.get("limited_chunks", 0) >= 1
        snap = s.telemetry.snapshot(0.0)["series"]
        gen, cmd = np.array(snap["gen"]), np.array(snap["cmd"])
        assert gen.shape[1] == 1 + 8 and cmd.shape[1] == 1 + 7
        # the dashboard sees the raw +0.69 rad request next to the limited command
        assert np.max(gen[:, 1] - np.interp(gen[:, 0], cmd[:, 0], cmd[:, 1])) > 0.3
        s.stop_episode("stopped by user")
        folder = s.save_episode("", "", "unlabeled", wait=True)["folder"]
        d = np.load(f"{folder}/episode.npz")
        q0, t = d["command/arm"][:, 0], d["command/arm_t"]
        speed = np.abs(np.diff(q0)) / np.maximum(np.diff(t), 1e-3)
        # commanded joint-0 speed stays near the 0.5 rad/s cap (control-loop jitter aside)
        assert np.percentile(speed, 95) < 0.5 * 1.5
    finally:
        s.shutdown()
        server.shutdown()


def test_home_pose_from_gui(sim_session):
    s = sim_session
    s.connect()
    _wait(s, SessionState.IDLE)
    target = [0.3, -0.2, 0.1, -2.0, 0.05, 1.9, 0.7]
    s.update_config({"home_q": target})
    s.go_home()
    _wait(s, SessionState.BUSY, timeout=2.0)
    _wait(s, SessionState.IDLE)
    assert np.allclose(s.hw.arm.current_state["q"], target, atol=1e-3)
    with pytest.raises(ActionError):
        s.update_config({"home_q": [0, 0, 0, 0, 0, 0, 0]})  # q3 = 0 is outside the limits
    s.update_config({"home_q": [0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0]})
    home = s.set_home_to_current()["home_q"]  # arm is still at `target`
    assert np.allclose(home, target, atol=1e-3) and s.cfg.home_q == home


def test_env_frequency_streams_observations(tmp_path):
    import socket

    seen = []

    class RecordingDummy(DummyPolicy):
        def infer(self, observation):
            seen.append(bool(observation.get("env_stream")))
            return super().infer(observation)

    policy = RecordingDummy(action_space="joint", latency_s=0.05)
    server = PolicyServer(policy, "127.0.0.1", 0)
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        server.port = sock.getsockname()[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    time.sleep(0.2)
    with pytest.raises(ValueError):
        SessionConfig(env_hz=-1).validate()
    cfg = SessionConfig(sim=True, control_hz=50, policy_port=server.port, cameras=[CameraConfig("ext")],
                        save_root=str(tmp_path), batch_name="b", task="t", env_hz=20)
    s = InferenceSession(cfg)
    try:
        s.connect()
        _wait(s, SessionState.IDLE)
        assert s.health["supports_observe"] is True
        s.start_episode()
        _wait(s, SessionState.RUNNING)
        time.sleep(1.5)
        s.stop_episode("stopped by user")
        idx = list(policy.observed)
        assert idx[0] == 0 and idx == sorted(set(idx))  # increasing frame indices from 0
        assert 20 <= idx[-1] <= 40  # ~1.5 s at 20 Hz (start-up included)
        assert len(idx) >= 0.8 * (idx[-1] + 1)  # few frames skipped
        assert seen and all(seen)  # infer observations are flagged while streaming
        assert s.telemetry.counters["observations"] == len(idx)
        n = len(policy.observed)
        time.sleep(0.3)
        assert len(policy.observed) == n  # stream stops with the rollout
    finally:
        s.shutdown()
        server.shutdown()
