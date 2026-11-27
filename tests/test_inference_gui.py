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
    clamp_stream_lead,
    resample_chunk,
    speed_limit_chunk,
    streaming_action_anchor,
    streaming_action_duration,
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


def test_streaming_action_duration_reschedules_late_inference():
    # A 20 ms response still reaches the original 50 ms policy deadline.
    assert streaming_action_duration(0.020, fps=20.0, control_hz=50.0) == pytest.approx(0.030)
    # A response that already missed the deadline gets a new ramp matching its
    # measured latency instead of collapsing to the 20 ms control-period floor.
    assert streaming_action_duration(0.090, fps=20.0, control_hz=50.0) == pytest.approx(0.090)
    # A cold or stalled request cannot authorize an arbitrarily long move.
    assert streaming_action_duration(0.250, fps=20.0, control_hz=50.0) == pytest.approx(0.100)

    anchor = np.array([0.5, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0])
    rows = np.array([[0.6, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0, 0.0]])
    duration = streaming_action_duration(0.090, fps=20.0, control_hz=50.0)
    limited, changed = speed_limit_chunk(
        rows, np.array([duration]), anchor, ACTION_SPACES["cartesian"], 0.5, 0.15
    )
    assert changed
    assert limited[0, 0] - anchor[0] == pytest.approx(0.15 * 0.090)


def test_streaming_action_anchor_supports_current_and_old_following_modes():
    measured = np.array([0.50, 0.0, 0.40, 1.0, 0.0, 0.0, 0.0])
    previous = np.array([0.53, 0.0, 0.40, 1.0, 0.0, 0.0, 0.0])

    assert streaming_action_anchor(measured, previous, "persistent-setpoint") is previous
    assert streaming_action_anchor(measured, previous, "measured-pose") is measured
    assert streaming_action_anchor(measured, None, "persistent-setpoint") is measured
    with pytest.raises(ValueError):
        streaming_action_anchor(measured, previous, "unknown")


def test_inference_session_rejects_unknown_stream_anchor_mode():
    with pytest.raises(ValueError):
        InferenceSession(SessionConfig(), stream_anchor_mode="unknown")


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


def test_streaming_policy_health_and_shutdown():
    class StreamingDummy(DummyPolicy):
        query_every_step = True

        def __init__(self):
            super().__init__(action_space="joint", fps=20, exec_steps=1, latency_s=0)
            self.closed = False

        def close(self):
            self.closed = True

    policy = StreamingDummy()
    server = PolicyServer(policy, "127.0.0.1", 0)
    health = server._handle({"type": "health"})
    assert health["query_every_step"] is True
    assert health["fps"] == 20 and health["exec_steps"] == 1
    server.shutdown()
    assert policy.closed

    session = InferenceSession(SessionConfig(policy_fps=15, exec_steps=9))
    session.health = health
    session._apply_policy_timing()
    assert session.policy_fps == 20
    assert session.exec_steps == 1


def test_policy_server_forwards_optional_inference_diagnostics():
    class DiagnosticPolicy(DummyPolicy):
        def inference_diagnostics(self):
            return {
                "obs_timestep": 3,
                "sys1_plan": np.ones((2, 8)),
                "sys2_plan": np.zeros((3, 8)),
            }

    policy = DiagnosticPolicy(action_space="joint", latency_s=0)
    response = PolicyServer(policy, "127.0.0.1", 0)._handle(
        {
            "type": "infer",
            "observation": {"observation.state": np.zeros(8), "task": "test"},
        }
    )
    assert response["ok"]
    assert response["diagnostics"]["obs_timestep"] == 3
    assert response["diagnostics"]["sys1_plan"].shape == (2, 8)


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
    # cam_*.mp4 is on the real time axis: duration ~= rollout duration
    import cv2

    cap = cv2.VideoCapture(f"{folder}/cam_ext.mp4")
    n_video, fps_video = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), cap.get(cv2.CAP_PROP_FPS)
    cap.release()
    assert n_video == len(data["camera/ext/t"])
    assert abs(n_video / fps_video - meta["stats"]["duration_s"]) < 0.3, (n_video, fps_video, meta["stats"]["duration_s"])
    assert len(np.unique(data["camera/ext/t"])) == len(data["policy/t_obs"])  # one new frame per request

    from pathlib import Path

    s.edit_saved(Path(folder).name, {"description": "edited", "outcome": "failure"})
    row = read_batch_index(s.batch_dir)[0]
    assert row["description"] == "edited" and row["outcome"] == "failure"

    # delete: folder moves to .trash, index row dropped, number not reused
    name = Path(folder).name
    with pytest.raises(ActionError):
        s.delete_saved("../" + name)
    with pytest.raises(ActionError):
        s.delete_saved("nope")
    r = s.delete_saved(name)
    assert not Path(folder).exists() and Path(r["moved_to"]).is_dir()
    assert Path(r["moved_to"]).parent.name == ".trash"
    assert read_batch_index(s.batch_dir) == []
    from franka_control_client.inference_gui.recorder import next_episode_index
    assert next_episode_index(s.batch_dir) == 1


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


def test_clamp_stream_lead():
    measured = np.array([0.5, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0])
    rows = np.array([[0.6, 0.0, 0.4, 0.0, 1.0, 0.0, 0.0, 0.7]])
    out, clamped = clamp_stream_lead(rows, measured, ACTION_SPACES["cartesian"], 0.05, 0.3)
    assert clamped
    assert out[0, 0] == pytest.approx(0.55)  # pulled onto the 5 cm sphere
    assert np.allclose(out[0, 3:], rows[0, 3:])  # orientation and gripper untouched
    near = rows.copy()
    near[0, 0] = 0.53
    same, clamped = clamp_stream_lead(near, measured, ACTION_SPACES["cartesian"], 0.05, 0.3)
    assert not clamped and np.allclose(same, near)

    joint_rows = np.zeros((1, 8))
    joint_rows[0, :2] = [0.5, -0.05]
    out, clamped = clamp_stream_lead(joint_rows, np.zeros(7), ACTION_SPACES["joint"], 0.05, 0.1)
    assert clamped and out[0, 0] == pytest.approx(0.1) and out[0, 1] == pytest.approx(-0.05)


class _LeadingStreamPolicy(DummyPolicy):
    """Streaming policy whose target always leads the measured joint 0 by 0.5 rad."""

    query_every_step = True

    def __init__(self):
        super().__init__(action_space="joint", fps=20, exec_steps=1, latency_s=0)

    def infer(self, observation):
        state = np.asarray(observation["observation.state"], dtype=np.float64)
        row = state[None, :8].copy()
        row[:, 0] += 0.5
        return row


def test_streaming_setpoint_ramps_from_previous_setpoint(tmp_path):
    import socket

    with socket.socket() as sk:
        sk.bind(("127.0.0.1", 0))
        port = sk.getsockname()[1]
    server = PolicyServer(_LeadingStreamPolicy(), "127.0.0.1", port)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    time.sleep(0.2)
    cfg = SessionConfig(
        sim=True, control_hz=50, policy_port=port, cameras=[CameraConfig("ext")],
        save_root=str(tmp_path), batch_name="b", task="t",
        first_step_jump_mode="limit", max_joint_speed=0.5, max_stream_lead_joint=0.1,
    )
    s = InferenceSession(cfg)
    try:
        s.connect()
        _wait(s, SessionState.IDLE)
        queued = []
        s.start_episode()
        _wait(s, SessionState.RUNNING)
        original = s.pair.update_action_chunk

        def spy(rows):
            queued.append((np.array(rows, copy=True), np.asarray(s.hw.arm.current_state["q"], dtype=float)))
            original(rows)

        s.pair.update_action_chunk = spy
        time.sleep(1.5)
        s.stop_episode("stopped by user")
        assert len(queued) >= 5
        # A ramp can collapse into one control tick, so bound it by one policy
        # period at max_joint_speed -- far below the 0.5 rad request.
        step_cap = 0.5 / 20.0
        for (prev, _), (cur, _) in zip(queued, queued[1:]):
            # each ramp continues from the previous setpoint: no snap back to the measured state
            assert abs(cur[0, 0] - prev[-1, 0]) <= step_cap + 1e-6
        for rows, q in queued:
            # the held setpoint never leads the measured state by more than the bound
            assert rows[-1, 0] - q[0] <= 0.1 + 0.02
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


def test_lerobot_keeps_real_duration_with_slow_sampler(sim_session):
    """A sampler slower than the dataset rate must not shorten the episode:
    missed slots are filled with repeats (frames ~= duration * fps)."""
    pytest.importorskip("lerobot")
    s = sim_session
    s.update_config({"save_lerobot": True, "dataset_fps": 10, "dataset_image_width": 64, "dataset_image_height": 48})
    s.connect()
    _wait(s, SessionState.IDLE)
    cam = s.hw.cameras["ext"]
    fast = cam.get_image

    def slow():
        if s.state == SessionState.RUNNING:
            time.sleep(0.25)  # 4 Hz camera read for a 10 Hz dataset
        return fast()

    cam.get_image = slow
    s.start_episode()
    _wait(s, SessionState.RUNNING)
    time.sleep(3.0)
    s.stop_episode("stopped by user")
    folder = s.save_episode("", "", "success", wait=True)["folder"]
    meta = json.load(open(f"{folder}/meta.json"))
    expected = meta["stats"]["duration_s"] * 10
    assert abs(meta["lerobot"]["frames"] - expected) <= 4, (meta["lerobot"], expected)
    assert meta["lerobot"]["duplicated_frames"] > 10
