"""Inference session: owns hardware, control pair, policy client and recorder.

It replaces the keyboard state machine of ``PolicyInferenceManager`` with
explicit actions the GUI calls:

    DISCONNECTED --connect--> CONNECTING --> IDLE
    IDLE --start--> RUNNING --stop / e-stop / timeout / safety / error--> REVIEW
    REVIEW --save | discard--> IDLE
    IDLE | REVIEW --home--> BUSY --> (previous state)
"""

from __future__ import annotations

import enum
import json
import logging
import threading
import time
import traceback
from collections import deque
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

import numpy as np

from ..policy_inference.policy_server_client import PolicyServerClient
from .config import CONNECTION_FIELDS, SessionConfig
from .hardware import Q_LOWER, Q_UPPER, Hardware, build_hardware, gripper_closedness
from .policy_server import ACTION_SPACES
from .recorder import EpisodeRecorder, next_episode_index, read_batch_index, update_saved_rollout
from .telemetry import Telemetry

LOG = logging.getLogger("inference_gui")

OUTCOMES = ("success", "partial", "failure", "unlabeled")
POLICY_CONNECT_TIMEOUT_S = 5.0


class SessionState(str, enum.Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    RUNNING = "running"
    REVIEW = "review"  # rollout finished, waiting for save / discard
    BUSY = "busy"  # homing / starting
    ERROR = "error"


class ActionError(Exception):
    """Rejected GUI action; the message is shown to the user."""


# --------------------------------------------------------------------- helpers
def resample_chunk(
    rows: np.ndarray,
    row_times: np.ndarray,
    anchor: np.ndarray,
    control_hz: float,
    action_space: str,
) -> np.ndarray:
    """Interpolate chunk rows (at ``row_times`` s from now) onto the control
    rate, starting from the measured ``anchor`` at t = 0.

    Joints / positions are linear, quaternions normalised-linear with sign
    alignment, the gripper (last column) is held per row."""
    n, dim = rows.shape
    t_end = float(row_times[-1])
    n_out = max(1, int(round(t_end * control_hz)))
    t_out = np.arange(1, n_out + 1) * (t_end / n_out)
    t_src = np.concatenate([[0.0], row_times])
    src = np.vstack([anchor[None, : dim - 1], rows[:, : dim - 1]])

    cartesian = action_space == ACTION_SPACES["cartesian"]
    if cartesian:  # keep consecutive quaternions in the same hemisphere
        for i in range(1, src.shape[0]):
            if np.dot(src[i, 3:7], src[i - 1, 3:7]) < 0:
                src[i, 3:7] *= -1
    out = np.empty((n_out, dim), dtype=np.float64)
    for j in range(dim - 1):
        out[:, j] = np.interp(t_out, t_src, src[:, j])
    if cartesian:
        out[:, 3:7] /= np.linalg.norm(out[:, 3:7], axis=1, keepdims=True)
    row_idx = np.clip(np.searchsorted(row_times, t_out - 1e-9), 0, n - 1)
    out[:, dim - 1] = rows[row_idx, dim - 1]
    return out


class InferenceSession:
    def __init__(self, cfg: SessionConfig, config_path: Optional[Path] = None) -> None:
        self.cfg = cfg
        self.config_path = config_path
        self.state = SessionState.DISCONNECTED
        self.error: Optional[str] = None
        self.busy_label = ""
        self._lock = threading.RLock()

        self.telemetry = Telemetry()
        self.recorder = EpisodeRecorder()
        self.hw: Optional[Hardware] = None
        self.pair: Any = None
        self.client: Optional[PolicyServerClient] = None
        self.policy_connected = False
        self.policy_error: Optional[str] = None
        self.health: Dict[str, Any] = {}
        self.policy_fps = float(cfg.policy_fps)
        self.exec_steps = int(cfg.exec_steps)

        self._pyzlc_initialised = False
        self._pyzlc_network: Optional[Tuple[Any, ...]] = None
        self._monitor_stop = threading.Event()
        self._monitor_thread: Optional[threading.Thread] = None
        self._episode_stop = threading.Event()
        self._infer_thread: Optional[threading.Thread] = None
        self._episode_id = 0
        self._stop_reason = ""
        self._last_arm_cmd: Optional[np.ndarray] = None
        self._last_grip_cmd: Optional[float] = None
        self._dataset_thread: Optional[threading.Thread] = None
        self._lerobot_error: Optional[str] = None  # why lerobot cannot be imported
        self._lerobot_checked = False
        self.export_status: Dict[str, Any] = {"running": False, "message": "", "result": None, "error": None}
        self.episode: Dict[str, Any] = {}
        self.last_summary: Optional[Dict[str, Any]] = None
        self.latest_robot: Dict[str, Any] = {}

        self._logs: Deque[Tuple[int, float, str, str]] = deque(maxlen=400)
        self._log_seq = 0
        self._ema_rtt = 0.0

    # =============================================================== logging
    def log(self, msg: str, level: str = "info") -> None:
        with self._lock:
            self._log_seq += 1
            self._logs.append((self._log_seq, time.time(), level, msg))
        getattr(LOG, "warning" if level == "warn" else level, LOG.info)(msg)

    def _spawn(self, target: Callable[[], None], name: str) -> None:
        threading.Thread(target=target, name=name, daemon=True).start()

    def _require(self, *states: SessionState) -> None:
        if self.state not in states:
            allowed = ", ".join(s.value for s in states)
            raise ActionError(f"Not possible while {self.state.value} (needs: {allowed}).")

    # ============================================================ dispatching
    def do(self, action: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        handlers: Dict[str, Callable[[Dict[str, Any]], Any]] = {
            "connect": lambda p: self.connect(),
            "disconnect": lambda p: self.disconnect(),
            "reconnect_policy": lambda p: self.reconnect_policy(),
            "start": lambda p: self.start_episode(str(p.get("notes", ""))),
            "stop": lambda p: self.stop_episode("stopped by user"),
            "estop": lambda p: self.software_estop(),
            "home": lambda p: self.go_home(),
            "save": lambda p: self.save_episode(
                p.get("name", ""), p.get("description", ""), p.get("outcome", "unlabeled")
            ),
            "discard": lambda p: self.discard_episode(),
            "edit_saved": lambda p: self.edit_saved(str(p.get("folder", "")), p),
            "export_dataset": lambda p: self.export_dataset(p),
            "clear_error": lambda p: self.clear_error(),
        }
        if action not in handlers:
            raise ActionError(f"Unknown action {action!r}")
        result = handlers[action](payload)
        return result if isinstance(result, dict) else {}

    # ================================================================ config
    def update_config(self, update: Dict[str, Any]) -> SessionConfig:
        with self._lock:
            if self.state in (SessionState.RUNNING, SessionState.BUSY, SessionState.CONNECTING):
                raise ActionError(f"Cannot change settings while {self.state.value}.")
            try:
                new_cfg = self.cfg.merged(update)
            except (TypeError, ValueError) as exc:
                raise ActionError(f"Invalid settings: {exc}") from exc
            old = self.cfg.to_dict()
            new = new_cfg.to_dict()
            changed = {k for k in new if new[k] != old.get(k)}
            connected = self.hw is not None
            if connected and changed & CONNECTION_FIELDS:
                raise ActionError(
                    f"Disconnect first to change: {', '.join(sorted(changed & CONNECTION_FIELDS))}."
                )
            self.cfg = new_cfg
            if self.config_path is not None:
                self.cfg.save(self.config_path)
            if changed:
                self.log(f"Settings updated: {', '.join(sorted(changed))}")
            policy_fields = {"policy_host", "policy_port", "request_timeout_s", "control_pair"}
            if connected and changed & {"control_pair", "control_hz"}:
                self._build_control_pair()
            if not (changed & policy_fields) and changed & {"policy_fps", "exec_steps"}:
                self._apply_policy_timing()
            if connected and changed & policy_fields:
                self._spawn(self._connect_policy, "policy-connect")
            return self.cfg

    # ============================================================ connection
    def connect(self) -> None:
        with self._lock:
            self._require(SessionState.DISCONNECTED, SessionState.ERROR)
            if self.hw is not None:
                raise ActionError("Already connected.")
            self.state = SessionState.CONNECTING
            self.error = None
        self._spawn(self._connect_worker, "connect")

    def _connect_worker(self) -> None:
        cfg = self.cfg
        try:
            if not cfg.sim:
                self._init_pyzlc(cfg)
            hw = build_hardware(cfg, self.log)
            hw.install_command_tap(self._on_command)
            with self._lock:
                self.hw = hw
                self._build_control_pair()
            self._start_monitor()
            self._connect_policy()
            with self._lock:
                self.state = SessionState.IDLE
            self.log("Connected. Ready to start a rollout." if self.policy_connected
                     else "Robot connected; policy server not reachable yet.")
        except Exception as exc:
            self.log(f"Connection failed: {exc}", "error")
            LOG.debug(traceback.format_exc())
            self._teardown_hardware()
            with self._lock:
                self.state = SessionState.ERROR
                self.error = f"Connection failed: {exc}"

    def _init_pyzlc(self, cfg: SessionConfig) -> None:
        import pyzlc

        network = (cfg.node_name, cfg.node_ip, cfg.group_name, cfg.group_port)
        if self._pyzlc_initialised:
            if network != self._pyzlc_network:
                self.log("pyzlc network settings changed: restart the app for them to apply.", "warn")
            return
        self.log(f"Starting pyzlc node '{cfg.node_name}' on {cfg.node_ip} ({cfg.group_name}:{cfg.group_port})")
        pyzlc.init(cfg.node_name, cfg.node_ip, group_name=cfg.group_name, group_port=cfg.group_port)
        self._pyzlc_initialised = True
        self._pyzlc_network = network

    def reconnect_policy(self) -> None:
        with self._lock:
            self._require(SessionState.IDLE, SessionState.REVIEW)
        self._spawn(self._connect_policy, "policy-connect")

    def _connect_policy(self) -> None:
        cfg = self.cfg
        with self._lock:
            old = self.client
            self.client = None
            self.policy_connected = False
            self.policy_error = None
        _force_close(old)
        self.log(f"Connecting to policy server {cfg.policy_host}:{cfg.policy_port}...")
        try:
            client = PolicyServerClient(cfg.policy_host, cfg.policy_port, timeout_s=POLICY_CONNECT_TIMEOUT_S)
            health = client.health()
            client.timeout_s = cfg.timeout
            if client._sock is not None:
                client._sock.settimeout(cfg.timeout)
                try:
                    import socket

                    client._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    pass
            expected = ACTION_SPACES[cfg.control_pair]
            space = health.get("action_space")
            if space is not None and space != expected:
                client.close()
                raise RuntimeError(
                    f"server action space is {space!r} but the '{cfg.control_pair}' control pair "
                    f"needs {expected!r}"
                )
            if space is None:
                self.log("Policy server does not report action_space; assuming it matches.", "warn")
            with self._lock:
                self.client = client
                self.health = health
                self.policy_connected = True
                self._apply_policy_timing()
            self.log(f"Policy server OK: {json.dumps(health, default=str)}")
        except Exception as exc:
            with self._lock:
                self.policy_error = str(exc)
                self.health = {}
            self.log(f"Policy server not available: {exc}", "error")

    def _apply_policy_timing(self) -> None:
        cfg = self.cfg
        server_fps = self.health.get("fps")
        if server_fps is not None and abs(float(server_fps) - cfg.policy_fps) > 1e-6:
            self.log(f"Using the server's chunk rate {server_fps} Hz (settings say {cfg.policy_fps}).", "warn")
        self.policy_fps = float(server_fps) if server_fps is not None else float(cfg.policy_fps)
        self.exec_steps = int(cfg.exec_steps) or int(self.health.get("exec_steps") or 0)

    def _build_control_pair(self) -> None:
        assert self.hw is not None
        cfg = self.cfg
        if self.pair is not None and self.pair.is_running:
            raise ActionError("Control pair is running.")
        if cfg.control_pair == "joint":
            from ..control_pair.policy_panda_control_pair import PolicyPandaControlPair as PairCls
        else:
            from ..control_pair.cartesian_policy_panda_control_pair import (
                CartesianPolicyPandaControlPair as PairCls,
            )
        pair = PairCls(self.hw.arm, self.hw.gripper, cfg.control_hz)
        original_step = pair.control_step
        telemetry, recorder = self.telemetry, self.recorder

        def instrumented_step() -> None:
            t = time.time()
            telemetry.control_rate.tick(t)
            telemetry.count("control_steps")
            recorder.add_control_step(t)
            original_step()

        pair.control_step = instrumented_step  # instance attribute shadows the method
        self.pair = pair
        self.log(f"Control pair: {PairCls.__name__} @ {cfg.control_hz:g} Hz")

    def disconnect(self) -> None:
        with self._lock:
            self._require(SessionState.IDLE, SessionState.ERROR, SessionState.REVIEW)
            if self.state == SessionState.REVIEW and self.recorder.has_data:
                raise ActionError("Save or discard the last rollout first.")
        self._teardown_hardware()
        with self._lock:
            self.state = SessionState.DISCONNECTED
            self.error = None
        self.log("Disconnected.")

    def _teardown_hardware(self) -> None:
        self._monitor_stop.set()
        if self._monitor_thread is not None:
            self._monitor_thread.join(timeout=2)
            self._monitor_thread = None
        if self.pair is not None and self.pair.is_running:
            self.pair.stop_control_pair()
        self.pair = None
        _force_close(self.client)
        self.client = None
        self.policy_connected = False
        if self.hw is not None:
            self.hw.close()
        self.hw = None
        self.latest_robot = {}

    # ========================================================= state monitor
    def _start_monitor(self) -> None:
        self._monitor_stop.clear()
        self._monitor_thread = threading.Thread(target=self._monitor_loop, name="state-monitor", daemon=True)
        self._monitor_thread.start()

    def _monitor_loop(self) -> None:
        last_time_ms = None
        last_rate_sample = 0.0
        while not self._monitor_stop.is_set():
            period = 1.0 / max(1.0, float(self.cfg.state_record_hz))
            t = time.time()
            hw = self.hw
            if hw is None:
                break
            try:
                state = hw.arm.current_state
                if state is not None:
                    if state.get("time_ms") != last_time_ms:
                        self.telemetry.state_rate.tick(t)
                        last_time_ms = state.get("time_ms")
                    grip = gripper_closedness(hw.gripper)
                    row = {
                        "q": list(state["q"]),
                        "dq": list(state.get("dq", [np.nan] * 7)),
                        "q_d": list(state.get("q_d", [np.nan] * 7)),
                        "ee_pos": list(state["EE_pos"]),
                        "ee_quat": list(state["EE_quat"]),
                        "gripper": np.nan if grip is None else grip,
                        "tau_ext": list(state.get("tau_ext_hat_filtered", [np.nan] * 7)),
                    }
                    self.latest_robot = {**row, "t": t}
                    self.telemetry.state_series.append(t, row["q"] + [0.0 if grip is None else grip])
                    self.telemetry.ee_series.append(t, row["ee_pos"] + row["ee_quat"])
                    self.recorder.add_state(t, row)
            except Exception as exc:  # keep monitoring through transient errors
                LOG.debug("state monitor: %s", exc)
            if t - last_rate_sample >= 0.25:
                rates = self.telemetry
                rates.rate_series.append(t, [rates.control_rate.rate(t), rates.infer_rate.rate(t)])
                last_rate_sample = t
            self._monitor_stop.wait(max(0.0, period - (time.time() - t)))

    def _on_command(self, t: float, kind: str, values: Any) -> None:
        values = np.asarray(values, dtype=np.float64).reshape(-1)
        if kind != "gripper":
            self.telemetry.command_rate.tick(t)
            self.telemetry.count("commands")
            self.telemetry.cmd_series.append(t, values[:7].tolist())
            self._last_arm_cmd = values[:7].copy()
        else:
            self._last_grip_cmd = self._gripper_command_value(float(values[0]))
        self.recorder.add_command(t, kind, values.tolist())

    def _gripper_command_value(self, raw: float) -> float:
        """The gripper command as the policy expressed it (the pair's 0/1).
        Robotiq: the position itself; Franka hand: width / max_width, the inverse
        of how PolicyPandaControlPair scales it."""
        hw = self.hw
        if hw is None:
            return raw
        state = hw.gripper.current_state or {}
        if "position" in state:
            return raw
        max_width = float(state.get("max_width", 0.0) or 0.0)
        return raw / max_width if max_width > 0 else raw

    # ========================================================= LeRobot dataset
    def _dataset_fps(self) -> float:
        return float(self.cfg.dataset_fps) or float(self.policy_fps)

    def _measured_dataset_state(self, hw: Hardware) -> Tuple[np.ndarray, np.ndarray, float]:
        """observation.state, observation.force_torque, measured gripper closedness."""
        state = hw.arm.current_state
        grip = gripper_closedness(hw.gripper)
        grip = 0.0 if grip is None else grip
        if self.cfg.control_pair == "joint":
            vec = list(state["q"]) + [grip]
        else:
            vec = list(state["EE_pos"]) + list(state["EE_quat"]) + [grip]
        gstate = hw.gripper.current_state or {}
        ft = (
            list(state.get("tau_ext_hat_filtered", [0.0] * 7))
            + list(state.get("O_F_ext_hat_K", [0.0] * 6))
            + [float(gstate.get("current", 0.0) or 0.0)]
        )
        return np.asarray(vec, dtype=np.float32), np.asarray(ft, dtype=np.float32), grip

    def _create_dataset_writer(self, index: int) -> Any:
        from .lerobot_export import DATASET_DIR, RolloutDatasetWriter, build_features, resize_frame

        cfg, hw = self.cfg, self.hw
        assert hw is not None and self.recorder.tmp_dir is not None
        size = cfg.dataset_image_size
        shapes = {}
        for cam in cfg.cameras:
            frame = resize_frame(hw.cameras[cam.name].get_image(), size)
            shapes[cam.policy_key] = tuple(frame.shape)
        features = build_features(cfg.control_pair, shapes, cfg.dataset_use_videos, cfg.dataset_force_torque)
        return RolloutDatasetWriter(
            root=self.recorder.tmp_dir / DATASET_DIR,
            repo_id=f"local/{cfg.batch_name}_ep{index:03d}",
            fps=self._dataset_fps(),
            features=features,
            task=cfg.task,
            robot_type=cfg.robot_type,
            use_videos=cfg.dataset_use_videos,
        )

    def _dataset_loop(self, writer: Any, stop: threading.Event) -> None:
        """Samples the rollout at the dataset rate into the writer."""
        from .lerobot_export import resize_frame

        cfg, hw = self.cfg, self.hw
        assert hw is not None
        period = 1.0 / writer.fps
        size = cfg.dataset_image_size
        next_t = time.perf_counter()
        late = 0
        while not stop.is_set():
            try:
                state, ft, grip = self._measured_dataset_state(hw)
                frame: Dict[str, Any] = {"observation.state": state}
                if cfg.dataset_force_torque:
                    frame["observation.force_torque"] = ft
                for cam in cfg.cameras:
                    img = resize_frame(hw.cameras[cam.name].get_image(), size)
                    frame[cam.policy_key] = np.ascontiguousarray(np.transpose(img, (2, 0, 1)))
                arm = self._last_arm_cmd if self._last_arm_cmd is not None else state[:7]
                g = self._last_grip_cmd if self._last_grip_cmd is not None else grip
                writer.add_sample(frame, np.concatenate([np.asarray(arm, dtype=np.float32), [g]]).astype(np.float32))
            except Exception as exc:
                self.log(f"Dataset sampling error: {exc}", "error")
            next_t += period
            delay = next_t - time.perf_counter()
            if delay < -period:  # fell behind: resync rather than burst
                late += 1
                next_t = time.perf_counter()
            elif delay > 0:
                stop.wait(delay)
        if late:
            self.log(f"Dataset sampler fell behind {late} time(s) at {writer.fps:g} Hz.", "warn")

    # =============================================================== episode
    @property
    def batch_dir(self) -> Path:
        return Path(self.cfg.save_root).expanduser() / self.cfg.batch_name

    def start_episode(self, notes: str = "") -> None:
        """``notes``: per-rollout description written before the start (scene
        setup, object placement...); it pre-fills the description at save time."""
        with self._lock:
            self._require(SessionState.IDLE)
            if not self.policy_connected or self.client is None:
                raise ActionError("Policy server is not connected.")
            if self._infer_thread is not None and self._infer_thread.is_alive():
                raise ActionError("Previous inference call is still pending; reconnect the policy server.")
            if self.hw is None or self.hw.arm.current_state is None:
                raise ActionError("No robot state received yet.")
            if not self.cfg.task.strip():
                self.log("Task instruction is empty.", "warn")
            if self.cfg.save_lerobot and self._lerobot_import_error():
                raise ActionError(f"LeRobot dataset saving is on but lerobot cannot be imported: {self._lerobot_error}")
            self.state = SessionState.BUSY
            self.busy_label = "Starting rollout"
        self._spawn(lambda: self._start_worker(notes.strip()), "episode-start")

    def _start_worker(self, notes: str) -> None:
        cfg = self.cfg
        try:
            batch_dir = self.batch_dir
            batch_dir.mkdir(parents=True, exist_ok=True)
            self._write_batch_info(batch_dir)
            index = next_episode_index(batch_dir)
            assert self.client is not None and self.pair is not None
            self.client.request({"type": "reset", "task": cfg.task})
            self.telemetry.reset_episode()
            self._ema_rtt = 0.0
            self.pair.reset_action()
            context = {
                "batch_name": cfg.batch_name,
                "batch_description": cfg.batch_description,
                "task": cfg.task,
                "control_pair": cfg.control_pair,
                "control_hz": cfg.control_hz,
                "action_space": ACTION_SPACES[cfg.control_pair],
                "policy_fps": self.policy_fps,
                "exec_steps": self.exec_steps,
                "inference_mode": cfg.inference_mode,
                "sim": cfg.sim,
                "policy_server": {"host": cfg.policy_host, "port": cfg.policy_port, "health": self.health},
                "cameras": [c.__dict__ for c in cfg.cameras],
                "config": cfg.to_dict(),
            }
            self.recorder.start(batch_dir, index, cfg.record_images and bool(cfg.cameras), self.policy_fps, context)
            self._last_arm_cmd = None
            self._last_grip_cmd = None
            writer = None
            if cfg.save_lerobot:
                writer = self._create_dataset_writer(index)
                self.recorder.dataset_writer = writer
            with self._lock:
                self._episode_id += 1
                episode_id = self._episode_id
                self._episode_stop = threading.Event()
                self._stop_reason = ""
                self.episode = {
                    "id": episode_id,
                    "index": index,
                    "t_start": time.time(),
                    "default_name": f"{cfg.batch_name}_ep{index:03d}",
                    "notes": notes,
                    "stop_reason": None,
                }
                self.pair.start_control_pair()
                self._infer_thread = threading.Thread(
                    target=self._infer_loop, args=(episode_id, self._episode_stop), name="inference", daemon=True
                )
                self.state = SessionState.RUNNING
                self._infer_thread.start()
                self._dataset_thread = None
                if writer is not None:
                    self._dataset_thread = threading.Thread(
                        target=self._dataset_loop, args=(writer, self._episode_stop), name="dataset-sampler", daemon=True
                    )
                    self._dataset_thread.start()
            self.log(
                f"Rollout #{index} started (task: {cfg.task!r})"
                + (f", recording a LeRobot episode at {writer.fps:g} Hz." if writer is not None else ".")
            )
        except Exception as exc:
            self.log(f"Failed to start rollout: {exc}", "error")
            LOG.debug(traceback.format_exc())
            if self.pair is not None and self.pair.is_running:
                self.pair.stop_control_pair()
            self.recorder.discard()
            with self._lock:
                self.state = SessionState.IDLE

    def _write_batch_info(self, batch_dir: Path) -> None:
        path = batch_dir / "batch.json"
        info: Dict[str, Any] = {}
        if path.exists():
            try:
                info = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                info = {}
        info.setdefault("created", time.strftime("%Y-%m-%dT%H:%M:%S"))
        info.update(
            {
                "batch_name": self.cfg.batch_name,
                "batch_description": self.cfg.batch_description,
                "task": self.cfg.task,
                "updated": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "config": self.cfg.to_dict(),
            }
        )
        path.write_text(json.dumps(info, indent=2), encoding="utf-8")

    # ---------------------------------------------------------- the loop
    def _infer_loop(self, episode_id: int, stop: threading.Event) -> None:
        cfg = self.cfg
        client, pair, hw = self.client, self.pair, self.hw
        assert client is not None and pair is not None and hw is not None
        action_space = ACTION_SPACES[cfg.control_pair]
        fps = self.policy_fps
        t_start = time.time()
        reason = ""
        try:
            while not stop.is_set():
                if cfg.max_episode_s > 0 and time.time() - t_start > cfg.max_episode_s:
                    reason = f"time limit ({cfg.max_episode_s:g} s)"
                    break
                if not pair.is_running or (pair.control_task_thread is not None and not pair.control_task_thread.is_alive()):
                    reason = "control loop stopped unexpectedly"
                    break

                t_obs = time.time()
                obs, frames, obs_state = self._build_observation(hw)
                t_send = time.time()
                response = client.infer(obs)
                t_recv = time.time()
                if stop.is_set():
                    break
                chunk = self._extract_chunk(response)
                n_exec = int(response.get("exec_steps") or 0) if not cfg.exec_steps else int(cfg.exec_steps)
                n_exec = min(n_exec or chunk.shape[0], chunk.shape[0])

                if cfg.inference_mode == "async":
                    # Row k targets t_obs + (k + 1) / fps; drop rows already in the past.
                    age = time.time() - t_obs
                    skipped = min(int(age * fps), chunk.shape[0] - 1)
                    rows = chunk[skipped : skipped + n_exec]
                    row_times = (skipped + 1 + np.arange(rows.shape[0])) / fps - age
                    row_times = np.maximum(row_times, 1e-3 * (1 + np.arange(rows.shape[0])))
                    anchor = self._measured_vector(hw, action_space)
                else:
                    # Arm holds while the server computes; the chunk starts now.
                    skipped = 0
                    rows = chunk[:n_exec]
                    row_times = (1 + np.arange(rows.shape[0])) / fps
                    anchor = obs_state[:7]

                self._check_first_step(rows[0], anchor, action_space)
                if action_space == ACTION_SPACES["joint"] and cfg.clip_joint_limits:
                    rows = rows.copy()
                    rows[:, :7] = np.clip(rows[:, :7], Q_LOWER + 0.01, Q_UPPER - 0.01)
                exec_rows = resample_chunk(rows, row_times, anchor, pair.control_hz, action_space)
                pair.update_action_chunk(exec_rows)

                rtt = t_recv - t_send
                model_s = float(response.get("server_infer_s", np.nan))
                self._ema_rtt = rtt if self._ema_rtt == 0 else 0.8 * self._ema_rtt + 0.2 * rtt
                self._record_inference(t_obs, t_recv, rtt, model_s, obs_state, chunk, exec_rows, skipped, rows.shape[0], frames)

                if cfg.inference_mode == "async":
                    wait = float(row_times[-1]) - self._ema_rtt
                else:
                    wait = float(row_times[-1])
                if wait > 0:
                    stop.wait(wait)
        except _SafetyStop as exc:
            reason = f"safety: {exc}"
            self.telemetry.count("rejected_chunks")
            self.log(f"Rejected action chunk: {exc}", "error")
        except Exception as exc:
            reason = f"error: {exc}"
            self.log(f"Inference loop error: {exc}", "error")
            LOG.error(traceback.format_exc())
        # A stop requested from outside (user / e-stop) names its own reason.
        self._finish_episode(episode_id, reason or self._stop_reason or "stopped by user")

    def _build_observation(self, hw: Hardware) -> Tuple[Dict[str, Any], Dict[str, np.ndarray], np.ndarray]:
        cfg = self.cfg
        state = hw.arm.current_state
        if state is None:
            raise RuntimeError("Arm state missing.")
        grip = gripper_closedness(hw.gripper)
        if grip is None:
            raise RuntimeError("Gripper state missing.")
        if cfg.control_pair == "joint":
            vec = list(state["q"]) + [grip]
        else:
            vec = list(state["EE_pos"]) + list(state["EE_quat"]) + [grip]
        obs_state = np.asarray(vec, dtype=np.float64)
        obs: Dict[str, Any] = {"observation.state": obs_state.astype(np.float32), "task": cfg.task}
        frames: Dict[str, np.ndarray] = {}
        for cam in cfg.cameras:
            frame = hw.cameras[cam.name].get_image()
            if frame is None:
                raise RuntimeError(f"No image from camera {cam.name}.")
            frames[cam.name] = frame
            if cam.send_to_policy:
                obs[cam.policy_key] = np.ascontiguousarray(frame)
        return obs, frames, obs_state

    def _measured_vector(self, hw: Hardware, action_space: str) -> np.ndarray:
        state = hw.arm.current_state
        if action_space == ACTION_SPACES["joint"]:
            return np.asarray(state["q"], dtype=np.float64)
        return np.asarray(list(state["EE_pos"]) + list(state["EE_quat"]), dtype=np.float64)

    def _extract_chunk(self, response: Dict[str, Any]) -> np.ndarray:
        expected = ACTION_SPACES[self.cfg.control_pair]
        space = response.get("action_space", expected)
        if space != expected:
            raise RuntimeError(f"Unexpected action space {space!r} (expected {expected!r}).")
        chunk = np.asarray(response["action"], dtype=np.float64)
        if chunk.ndim == 3:
            if chunk.shape[0] != 1:
                raise RuntimeError(f"Expected batch size 1, got {chunk.shape}.")
            chunk = chunk[0]
        if chunk.ndim == 1:
            chunk = chunk[None]
        if chunk.ndim != 2 or chunk.shape[1] != 8 or chunk.shape[0] < 1:
            raise RuntimeError(f"Expected action chunk [T, 8], got shape {chunk.shape}.")
        if not np.all(np.isfinite(chunk)):
            raise RuntimeError("Policy returned non-finite actions.")
        chunk = chunk.copy()
        chunk[:, 7] = np.clip(chunk[:, 7], 0.0, 1.0)
        return chunk

    def _check_first_step(self, first_row: np.ndarray, anchor: np.ndarray, action_space: str) -> None:
        if action_space == ACTION_SPACES["joint"]:
            jump = float(np.max(np.abs(first_row[:7] - anchor[:7])))
            limit = self.cfg.max_first_step_jump_joint
            unit = "rad"
        else:
            jump = float(np.linalg.norm(first_row[:3] - anchor[:3]))
            limit = self.cfg.max_first_step_jump_cartesian
            unit = "m"
        if limit > 0 and jump > limit:
            raise _SafetyStop(f"first action is {jump:.3f} {unit} from the measured state (limit {limit:.3f} {unit})")

    def _record_inference(
        self,
        t_obs: float,
        t_recv: float,
        rtt: float,
        model_s: float,
        obs_state: np.ndarray,
        chunk: np.ndarray,
        exec_rows: np.ndarray,
        skipped: int,
        n_exec: int,
        frames: Dict[str, np.ndarray],
    ) -> None:
        tel = self.telemetry
        tel.infer_rate.tick(t_recv)
        tel.count("infer")
        tel.latency_ms.add(rtt * 1000)
        if np.isfinite(model_s):
            tel.model_ms.add(model_s * 1000)
        tel.latency_series.append(t_recv, [rtt * 1000, model_s * 1000 if np.isfinite(model_s) else 0.0])
        with tel._lock:
            tel.last_chunk = {
                "t": t_recv,
                "rows": int(chunk.shape[0]),
                "executed": int(n_exec),
                "skipped": int(skipped),
                "control_rows": int(exec_rows.shape[0]),
                "first": [round(float(v), 4) for v in chunk[skipped]],
                "last": [round(float(v), 4) for v in chunk[min(skipped + n_exec, chunk.shape[0]) - 1]],
                # whole raw chunk, so the dashboard can draw the planned trajectory
                "chunk": np.round(chunk, 4).tolist(),
            }
        self.recorder.add_inference(
            {
                "t_obs": t_obs,
                "t_recv": t_recv,
                "rtt_s": rtt,
                "server_infer_s": model_s,
                "obs_state": obs_state.tolist(),
                "action_raw": chunk,
                "action_exec": exec_rows,
                "skipped_rows": skipped,
            }
        )
        if self.cfg.record_images:
            self.recorder.add_frames(t_obs, frames)

    # ----------------------------------------------------------- stopping
    def stop_episode(self, reason: str) -> None:
        with self._lock:
            self._require(SessionState.RUNNING)
            episode_id = self._episode_id
            self._stop_reason = reason
            self._episode_stop.set()
        # Stop the arm right away, even if the infer call is still blocked.
        self._halt_control()
        thread = self._infer_thread
        if thread is not None:
            thread.join(timeout=1.0)
        self._finish_episode(episode_id, reason)

    def software_estop(self) -> None:
        with self._lock:
            self._stop_reason = "software e-stop"
            self._episode_stop.set()
            episode_id = self._episode_id
            running = self.state == SessionState.RUNNING
        self._halt_control()
        if self.hw is not None:
            self.hw.set_idle()
        self.log("SOFTWARE STOP: control loop halted, arm set to Idle.", "error")
        if running:
            self._finish_episode(episode_id, "software e-stop")

    def _halt_control(self) -> None:
        pair = self.pair
        if pair is not None and pair.is_running:
            try:
                pair.stop_control_pair()
            except Exception as exc:
                self.log(f"Error while stopping control pair: {exc}", "error")

    def _finish_episode(self, episode_id: int, reason: str) -> None:
        with self._lock:
            if episode_id != self._episode_id or self.state != SessionState.RUNNING:
                return  # already finished
            self._episode_stop.set()
            self.state = SessionState.REVIEW
            self.episode["stop_reason"] = reason
        self._halt_control()
        thread = self._dataset_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=5.0)
        self.recorder.stop(reason)
        self.last_summary = self.recorder.summary()
        self.log(f"Rollout #{self.episode.get('index')} ended: {reason}.",
                 "error" if reason.startswith(("safety", "error")) else "info")

    def save_episode(self, name: str, description: str, outcome: str, wait: bool = False) -> Dict[str, Any]:
        """Saving runs in the background (encoding the LeRobot videos can take a
        while); the session is BUSY until it is done. ``wait`` blocks (tests)."""
        with self._lock:
            self._require(SessionState.REVIEW)
            if outcome not in OUTCOMES:
                raise ActionError(f"Outcome must be one of {OUTCOMES}.")
            name = name.strip() or self.episode.get("default_name", "rollout")
            self.state = SessionState.BUSY
            self.busy_label = "Saving rollout"
        done: Dict[str, Any] = {}

        def work() -> None:
            try:
                folder = self.recorder.save(
                    name,
                    description.strip(),
                    outcome,
                    {"stop_reason": self.episode.get("stop_reason"), "pre_start_notes": self.episode.get("notes", "")},
                    progress=lambda msg: setattr(self, "busy_label", msg.rstrip(".")),
                )
                done["folder"] = str(folder)
                self.log(f"Saved rollout '{name}' ({outcome}) -> {folder}")
                next_state = SessionState.IDLE
            except Exception as exc:
                done["error"] = str(exc)
                self.log(f"Saving the rollout failed: {exc}", "error")
                LOG.error(traceback.format_exc())
                next_state = SessionState.REVIEW if self.recorder.has_data else SessionState.IDLE
            with self._lock:
                self.state = next_state

        if wait:
            work()
            if "error" in done:
                raise ActionError(done["error"])
            return done
        self._spawn(work, "save")
        return {"saving": True}

    # ------------------------------------------------------- dataset merge
    def _lerobot_import_error(self) -> Optional[str]:
        if not self._lerobot_checked:
            from .lerobot_export import lerobot_available

            self._lerobot_error = lerobot_available()
            self._lerobot_checked = True
        return self._lerobot_error

    def export_dataset(self, payload: Dict[str, Any], wait: bool = False) -> Dict[str, Any]:
        """Merge the batch's rollout datasets (filtered by outcome) into one."""
        with self._lock:
            if self.export_status["running"]:
                raise ActionError("A merge is already running.")
            if self.state == SessionState.RUNNING:
                raise ActionError("Merge after the rollout, not during it.")
            if self._lerobot_import_error():
                raise ActionError(f"lerobot cannot be imported: {self._lerobot_error}")
            self.export_status = {"running": True, "message": "Starting merge...", "result": None, "error": None}
        outcomes = [str(o) for o in payload.get("outcomes") or ["success"]]
        output = str(payload.get("output") or "").strip() or None
        overwrite = bool(payload.get("overwrite"))
        batch_dir = self.batch_dir

        def work() -> None:
            from .lerobot_export import merge_rollouts

            def progress(msg: str) -> None:
                self.export_status["message"] = msg
                self.log(msg)

            try:
                result = merge_rollouts(batch_dir, outcomes, output, overwrite, progress)
                self.export_status.update(running=False, result=result, message=f"Merged {result['episodes']} rollouts")
                self.log(f"Merged dataset: {result['episodes']} episodes -> {result['output']} ({result['repo_id']})")
            except Exception as exc:
                self.export_status.update(running=False, error=str(exc), message="")
                self.log(f"Dataset merge failed: {exc}", "error")
                LOG.debug(traceback.format_exc())

        if wait:
            work()
            return dict(self.export_status)
        self._spawn(work, "dataset-merge")
        return {"started": True}

    def discard_episode(self) -> None:
        with self._lock:
            self._require(SessionState.REVIEW)
            self.recorder.discard()
            self.state = SessionState.IDLE
        self.log(f"Rollout #{self.episode.get('index')} discarded.")

    def edit_saved(self, folder: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        fields = {k: str(payload[k]).strip() for k in ("description", "outcome") if k in payload}
        if "outcome" in fields and fields["outcome"] not in OUTCOMES:
            raise ActionError(f"Outcome must be one of {OUTCOMES}.")
        try:
            changed = update_saved_rollout(self.batch_dir, folder, fields)
        except ValueError as exc:
            raise ActionError(str(exc)) from exc
        self.log(f"Updated saved rollout {folder}: {', '.join(changed)}")
        return {"updated": changed}

    # ------------------------------------------------------------- misc
    def go_home(self) -> None:
        with self._lock:
            self._require(SessionState.IDLE, SessionState.REVIEW)
            back_to = self.state
            self.state = SessionState.BUSY
            self.busy_label = "Moving home"
        self._spawn(lambda: self._home_worker(back_to), "home")

    def _home_worker(self, back_to: SessionState) -> None:
        try:
            self.log("Moving to home pose...")
            self.pair.go_home()
            self.log("Home pose reached.")
        except Exception as exc:
            self.log(f"Homing failed: {exc}", "error")
        with self._lock:
            if self.state == SessionState.BUSY:
                self.state = back_to

    def clear_error(self) -> None:
        with self._lock:
            self.error = None
            if self.state == SessionState.ERROR:
                self.state = SessionState.DISCONNECTED if self.hw is None else SessionState.IDLE

    def shutdown(self) -> None:
        if self.state == SessionState.RUNNING:
            try:
                self.stop_episode("app shutdown")
            except ActionError:
                pass
        if self.recorder.has_data:
            self.log("Unsaved rollout discarded at shutdown.", "warn")
            self.recorder.discard()
        self._teardown_hardware()

    # ============================================================ dashboard
    def status(self, since: float, log_after: int) -> Dict[str, Any]:
        with self._lock:
            logs = [
                {"seq": s, "t": t, "level": lvl, "msg": m} for (s, t, lvl, m) in self._logs if s > log_after
            ]
            episode = dict(self.episode)
            state = self.state
        if state == SessionState.RUNNING and episode:
            episode["elapsed_s"] = time.time() - episode["t_start"]
        if state == SessionState.REVIEW:
            episode["summary"] = self.last_summary
        return {
            "now": time.time(),
            "state": state.value,
            "busy_label": self.busy_label if state == SessionState.BUSY else "",
            "error": self.error,
            "connected": self.hw is not None,
            "sim": self.cfg.sim,
            "policy": {
                "connected": self.policy_connected,
                "error": self.policy_error,
                "health": self.health,
                "fps": self.policy_fps,
                "exec_steps": self.exec_steps,
                "action_space": ACTION_SPACES[self.cfg.control_pair],
            },
            "control": {
                "pair": self.cfg.control_pair,
                "target_hz": self.cfg.control_hz,
                "running": bool(self.pair is not None and self.pair.is_running),
            },
            "episode": episode,
            "dataset": {
                "enabled": self.cfg.save_lerobot,
                "fps": self._dataset_fps(),
                "recording": bool(self._dataset_thread is not None and self._dataset_thread.is_alive()),
                "frames": getattr(self.recorder.dataset_writer, "n_frames", None),
                "lerobot_error": self._lerobot_error,
                "export": self.export_status,
            },
            "robot": self.latest_robot,
            "cameras": [c.name for c in self.cfg.cameras] if self.hw is not None else [],
            "telemetry": self.telemetry.snapshot(since),
            "logs": logs,
        }

    def episodes(self) -> List[Dict[str, Any]]:
        return read_batch_index(self.batch_dir)

    def camera_frame(self, name: str) -> Optional[np.ndarray]:
        hw = self.hw
        if hw is None or name not in hw.cameras:
            return None
        return hw.cameras[name].get_image()


class _SafetyStop(Exception):
    pass


def _force_close(client: Optional[PolicyServerClient]) -> None:
    """Close without the polite 'close' request: that request would wait on the
    client lock forever if another thread is stuck in infer()."""
    if client is None or client._sock is None:
        return
    import socket

    sock, client._sock = client._sock, None
    try:
        sock.shutdown(socket.SHUT_RDWR)  # unblocks a recv() in another thread
    except OSError:
        pass
    sock.close()
