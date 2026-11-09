"""Data collection session: state machine, recording loop, LeRobot export, import/convert, viewer.

States::

    DISCONNECTED --connect--> CONNECTING --> IDLE
    IDLE --start--> RECORDING --save/discard--> BUSY (saving) --> IDLE
    IDLE --reset/home--> BUSY --> IDLE
    IDLE --replay (teach_replay)--> BUSY (move to start) --> RECORDING (replay) --> auto save --> IDLE

A recording samples the robot at ``record_hz`` (absolute schedule, not sleep-after-work), each
camera at its own rate, and writes the IRL format (``irl_format.IRLEpisodeWriter``). If
``save_lerobot`` is on, every saved episode is queued and converted into the LeRobot dataset in
the background with the same converter as the Convert tab (``irl_to_lerobot``); with
``save_irl`` off the IRL episode is a staging copy that is removed after the conversion.
"""

from __future__ import annotations

import enum
import json
import logging
import queue
import shutil
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

import numpy as np

from .config import CONNECTION_FIELDS, CollectionConfig
from .hardware import GAINS, Rig, apply_keyboard_settings, build_rig
from .irl_format import (
    METADATA,
    IRLEpisodeWriter,
    find_episodes,
    follower_columns,
    gello_columns,
    is_episode_dir,
    keyboard_columns,
    mq3_columns,
    summarize_episode,
)

LOG = logging.getLogger("data_collection_gui")
STAGING = ".staging"


class State(str, enum.Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    IDLE = "idle"
    RECORDING = "recording"
    BUSY = "busy"
    ERROR = "error"


class ActionError(Exception):
    """A user action that is not possible right now (HTTP 409)."""


class _Unavailable(Exception):
    pass


class Recording:
    def __init__(self, kind: str, writer: IRLEpisodeWriter, staging: bool) -> None:
        self.kind = kind  # episode | demo | replay
        self.writer = writer
        self.staging = staging
        self.t_start = time.time()
        self.t_first: Optional[float] = None
        self.stop = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.missing = 0
        self.camera_errors = 0
        self.overruns = 0
        self.auto_reason = ""
        self.recent: Deque[float] = deque(maxlen=400)

    def rate(self) -> Optional[float]:
        ts = list(self.recent)
        if len(ts) < 2:
            return None
        cutoff = ts[-1] - 1.0
        window = [t for t in ts if t >= cutoff]
        if len(window) < 2:
            window = ts[-2:]
        return (len(window) - 1) / max(window[-1] - window[0], 1e-9)


class CollectionSession:
    def __init__(self, cfg: CollectionConfig, config_path: Optional[Path] = None) -> None:
        self.cfg = cfg
        self.config_path = config_path
        self.state = State.DISCONNECTED
        self.error: Optional[str] = None
        self.busy_label = ""
        self._lock = threading.RLock()
        self.rig: Optional[Rig] = None
        self.rec: Optional[Recording] = None
        self._pyzlc_initialised = False
        self._pyzlc_network: Optional[Tuple[Any, ...]] = None
        self._gripper_cmd = 0.0
        self._last_gripper_pos = 0.0018  # Robotiq sometimes reports 0.0: keep the last value
        self.saved_rev = 0  # bumped whenever the episode list changes
        self.last_saved: Optional[Dict[str, Any]] = None
        self._episode_cache: Dict[str, Tuple[float, Dict[str, Any]]] = {}

        # teach & replay
        self.demo: Any = None  # teach_replay_collection.Demo
        self.replayer: Any = None

        # live LeRobot export
        self._lerobot_q: "queue.Queue[Optional[Dict[str, Any]]]" = queue.Queue()
        self.lerobot_status: Dict[str, Any] = {"pending": 0, "current": "", "done": 0, "failed": 0, "last": None}
        self._lerobot_thread = threading.Thread(target=self._lerobot_worker, name="lerobot-export", daemon=True)
        self._lerobot_thread.start()

        # import / convert tab
        self.convert_status: Dict[str, Any] = {"running": False, "message": "", "fraction": 0.0, "result": None, "error": None}
        self._convert_cancel = threading.Event()

        # viewer tab
        self.viewer: Any = None

        self._logs: Deque[Tuple[int, float, str, str]] = deque(maxlen=500)
        self._log_seq = 0

    # ================================================================ basics
    def log(self, msg: str, level: str = "info") -> None:
        with self._lock:
            self._log_seq += 1
            self._logs.append((self._log_seq, time.time(), level, msg))
        getattr(LOG, "warning" if level == "warn" else level, LOG.info)(msg)

    def _spawn(self, target: Callable[[], None], name: str) -> None:
        threading.Thread(target=target, name=name, daemon=True).start()

    def _require(self, *states: State) -> None:
        if self.state not in states:
            raise ActionError(f"Not possible while {self.state.value} (needs {', '.join(s.value for s in states)}).")

    def _set_busy(self, label: str) -> None:
        self.state = State.BUSY
        self.busy_label = label

    def do(self, action: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        handlers: Dict[str, Callable[[Dict[str, Any]], Any]] = {
            "connect": lambda p: self.connect(),
            "disconnect": lambda p: self.disconnect(),
            "start": lambda p: self.start(),
            "save": lambda p: self.finish(save=True, reason="saved by user"),
            "discard": lambda p: self.finish(save=False, reason="discarded by user"),
            "reset": lambda p: self.reset(),
            "home": lambda p: self.home(),
            "set_home_current": lambda p: self.set_home_to_current(),
            "gripper": lambda p: self.gripper(bool(p.get("close"))),
            "keys": lambda p: self.keys(p.get("keys") or [], bool(p.get("slow"))),
            "kb_floor_current": lambda p: self.keyboard_floor_to_current(),
            "replay": lambda p: self.replay(),
            "use_demo": lambda p: self.use_demo(str(p.get("path", ""))),
            "delete_episode": lambda p: self.delete_episode(str(p.get("path", ""))),
            "requeue_lerobot": lambda p: self.requeue_lerobot(str(p.get("path", ""))),
            "convert": lambda p: self.start_convert(p),
            "cancel_convert": lambda p: self.cancel_convert(),
            "open_viewer": lambda p: self.open_viewer(str(p.get("path", ""))),
            "rerun": lambda p: self.launch_rerun(int(p.get("episode", 0))),
            "clear_error": lambda p: self.clear_error(),
        }
        if action not in handlers:
            raise ActionError(f"Unknown action {action!r}")
        result = handlers[action](payload)
        return result if isinstance(result, dict) else {}

    # ================================================================ config
    def update_config(self, update: Dict[str, Any]) -> CollectionConfig:
        with self._lock:
            if self.state in (State.RECORDING, State.BUSY, State.CONNECTING):
                raise ActionError(f"Cannot change settings while {self.state.value}.")
            try:
                new_cfg = self.cfg.merged(update)
            except (TypeError, ValueError) as exc:
                raise ActionError(f"Invalid settings: {exc}") from exc
            old, new = self.cfg.to_dict(), new_cfg.to_dict()
            changed = {k for k in new if new[k] != old.get(k)}
            if self.rig is not None and changed & CONNECTION_FIELDS:
                raise ActionError(f"Disconnect first to change: {', '.join(sorted(changed & CONNECTION_FIELDS))}.")
            self.cfg = new_cfg
            if self.config_path is not None:
                self.cfg.save(self.config_path)
            if changed:
                self.log(f"Settings updated: {', '.join(sorted(changed))}")
                self.saved_rev += 1  # task / data dir may have changed
            if "mode" in changed or "task" in changed:
                self.demo = None
            if self.rig is not None and self.rig.mode == "keyboard":
                apply_keyboard_settings(self.rig.leader, self.cfg)
                self.rig.pair.home_position = tuple(self.cfg.home_q)
            return self.cfg

    # ============================================================ connection
    def connect(self) -> None:
        with self._lock:
            self._require(State.DISCONNECTED, State.ERROR)
            self.state = State.CONNECTING
            self.error = None
        self._spawn(self._connect_worker, "connect")

    def _connect_worker(self) -> None:
        cfg = self.cfg
        try:
            if not cfg.sim:
                self._init_pyzlc(cfg)
            rig = build_rig(cfg, self.log)
            with self._lock:
                self.rig = rig
                self.state = State.IDLE
            self.log(f"Connected ({cfg.mode}{', SIMULATION' if cfg.sim else ''}). "
                     f"Press Reset to {RESET_TEXT[cfg.mode]}.")
        except Exception as exc:
            LOG.exception("connect failed")
            with self._lock:
                self.state = State.ERROR
                self.error = f"Connect failed: {exc}"
            self.log(self.error, "error")

    def _init_pyzlc(self, cfg: CollectionConfig) -> None:
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

    def disconnect(self) -> None:
        with self._lock:
            self._require(State.IDLE, State.ERROR)
            rig, self.rig = self.rig, None
            self.state = State.DISCONNECTED
        if rig is not None:
            rig.close()
            if rig.sim is False:
                rig.set_idle()
        self.demo = None
        self.log("Disconnected.")

    # ============================================================== recording
    def _new_episode_dir(self, kind: str, staging: bool) -> Path:
        cfg = self.cfg
        base = Path(cfg.data_dir).expanduser() / STAGING / cfg.task if staging else cfg.task_dir
        if kind in ("demo", "replay"):
            base = base / kind
        base.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        path, n = base / stamp, 1
        while path.exists():
            path, n = base / f"{stamp}_{n}", n + 1
        return path

    def _begin(self, kind: str) -> Recording:
        """Create the writer + recording thread (state must be set by the caller)."""
        cfg, rig = self.cfg, self.rig
        assert rig is not None
        staging = not cfg.save_irl
        leader = rig.leader_dir if kind == "episode" else ("Traj" if kind == "replay" else None)
        writer = IRLEpisodeWriter(self._new_episode_dir(kind, staging), cfg.robot_name, leader, list(rig.cameras))
        rec = Recording(kind, writer, staging)
        rec.thread = threading.Thread(target=self._record_loop, args=(rec,), name="record", daemon=True)
        self.rec = rec
        rec.thread.start()
        return rec

    def start(self) -> None:
        with self._lock:
            self._require(State.IDLE)
            cfg, rig = self.cfg, self.rig
            if rig is None:
                raise ActionError("Not connected.")
            if not cfg.save_irl and not cfg.save_lerobot:
                raise ActionError("Neither IRL nor LeRobot saving is on: nothing would be saved.")
            if cfg.save_lerobot:
                self._check_lerobot_options()
            kind = "demo" if cfg.mode == "teach_replay" else "episode"
            if cfg.mode in ("kinesthetic", "teach_replay", "keyboard"):
                state = rig.gripper.current_state
                if isinstance(state, dict) and 0.0 <= float(state.get("commanded_position", -1)) <= 1.0:
                    self._gripper_cmd = float(state["commanded_position"])
                if cfg.mode == "keyboard":
                    rig.leader.gripper = self._gripper_cmd
                    rig.leader.release_all()
                    rig.pair._anchor_to_measured()  # target = measured pose before the first sample
            self._set_busy("Starting")
        try:
            rig.pair.start_control_pair()
            rec = self._begin(kind)
        except Exception as exc:
            with self._lock:
                self.state = State.IDLE
            if rig.pair.is_running:
                rig.pair.stop_control_pair()
            raise ActionError(f"Could not start: {exc}") from exc
        with self._lock:
            self.state = State.RECORDING
        self.log(f"Recording {kind} -> {rec.writer.record_dir}" + (" (staging, LeRobot only)" if rec.staging else ""))

    def _sample(self, rig: Rig, rec: Recording) -> Dict[str, Any]:
        cfg = self.cfg
        arm = rig.arm.current_state
        grip = rig.gripper.current_state
        if arm is None or not isinstance(grip, dict):
            raise _Unavailable("arm or gripper state missing")
        position = float(grip.get("position", 0.0))
        if position == 0.0:
            position = self._last_gripper_pos
        else:
            self._last_gripper_pos = position
        values = follower_columns(cfg.robot_name, arm, grip, position)
        if rec.kind == "episode" and cfg.mode == "gello":
            state = rig.leader.current_state
            if state is None or state.get("gello_arm_state") is None or state.get("gello_gripper_state") is None:
                raise _Unavailable("GELLO state missing")
            values.update(gello_columns(rig.leader_dir, state, grip))
        elif rec.kind == "episode" and cfg.mode == "vr":
            signal = rig.leader.current_control_signal
            if signal is None:
                raise _Unavailable("VR control signal missing")
            values.update(mq3_columns(rig.leader_dir, signal))
        elif rec.kind == "episode" and cfg.mode == "keyboard":
            if not rig.pair.is_running or not rig.leader.anchored:
                raise _Unavailable("keyboard target not anchored yet")
            values.update(keyboard_columns(rig.leader_dir, rig.leader.current_control_signal))
        elif rec.kind == "replay":
            replayer = self.replayer
            now = time.perf_counter()
            q, dq, g = replayer.command_at_time(now)
            values.update({
                "Traj/joint_pos.pt": q, "Traj/joint_vel.pt": dq, "Traj/gripper_state.pt": g,
                "Traj/ee_wrench.pt": replayer.wrench_at_time(now),
                f"{cfg.robot_name}/gripper_command.pt": g,
            })
        if rec.kind in ("demo",) or (rec.kind == "episode" and cfg.mode == "kinesthetic"):
            values[f"{cfg.robot_name}/gripper_command.pt"] = self._gripper_cmd
        return values

    def _record_loop(self, rec: Recording) -> None:
        cfg, rig = self.cfg, self.rig
        assert rig is not None
        period = 1.0 / float(cfg.record_hz)
        cam_period = {c.name: (1.0 / c.hz if c.hz > 0 else 0.0) for c in cfg.cameras if c.name in rig.cameras}
        t_next = time.perf_counter()
        cam_next = {name: t_next for name in cam_period}
        auto_limit = float(cfg.max_episode_s)
        try:
            while not rec.stop.is_set():
                t = time.perf_counter()
                try:
                    values = self._sample(rig, rec)
                except _Unavailable as exc:
                    rec.missing += 1
                    if rec.missing in (1, 50, 500) or rec.missing % 5000 == 0:
                        self.log(f"Skipping samples: {exc} ({rec.missing} so far)", "warn")
                    values = None
                if values is not None:
                    rec.writer.add_sample(t, values)
                    rec.recent.append(t)
                    if rec.t_first is None:
                        rec.t_first = t
                    for name, cp in cam_period.items():
                        if t + 0.002 < cam_next[name]:
                            continue
                        try:
                            img = rig.cameras[name].get_image()
                            if img is not None:
                                rec.writer.add_frame(name, time.perf_counter(), img)
                        except Exception as exc:
                            rec.camera_errors += 1
                            if rec.camera_errors in (1, 50, 500):
                                self.log(f"Camera {name}: {exc}", "warn")
                        cam_next[name] = cam_next[name] + cp if cam_next[name] + cp > t - cp else t + cp
                # automatic end: replay finished / time limit
                if rec.kind == "replay" and self.replayer is not None and \
                        self.replayer.elapsed() >= self.demo.duration + float(cfg.replay_settle_s):
                    rec.auto_reason = "replay finished"
                elif auto_limit > 0 and rec.kind != "replay" and rec.t_first is not None and t - rec.t_first >= auto_limit:
                    rec.auto_reason = f"time limit {auto_limit:g} s"
                if rec.auto_reason:
                    self._spawn(lambda: self._auto_finish(rec), "auto-save")
                    break
                t_next += period
                delay = t_next - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    rec.overruns += 1
                    if delay < -period:  # far behind: restart the schedule instead of bursting
                        t_next = time.perf_counter()
        except Exception as exc:
            LOG.exception("recording loop failed")
            rec.auto_reason = f"recording error: {exc}"
            self.log(f"Recording loop failed: {exc}", "error")
            self._spawn(lambda: self._auto_finish(rec, save=False), "auto-discard")

    def _auto_finish(self, rec: Recording, save: bool = True) -> None:
        try:
            self.finish(save=save, reason=rec.auto_reason, expect=rec)
        except ActionError:
            pass

    def finish(self, save: bool, reason: str, expect: Optional[Recording] = None) -> None:
        with self._lock:
            self._require(State.RECORDING)
            rec = self.rec
            if rec is None or (expect is not None and rec is not expect):
                raise ActionError("No recording.")
            self._set_busy("Saving" if save else "Discarding")
        rec.stop.set()
        if rec.thread is not None and rec.thread is not threading.current_thread():
            rec.thread.join(timeout=5)
        self._spawn(lambda: self._finish_worker(rec, save, reason), "finish")

    def _finish_worker(self, rec: Recording, save: bool, reason: str) -> None:
        cfg, rig = self.cfg, self.rig
        try:
            if rec.kind == "replay":
                if self.replayer is not None:
                    self.replayer.stop()
                    self.replayer = None
            elif rig is not None and rig.pair.is_running:
                rig.pair.stop_control_pair()  # control_end(): arm Idle (+ gripper open for teleop)
            writer = rec.writer
            if save and writer.n_samples < 2:
                self.log(f"Only {writer.n_samples} samples recorded, discarding.", "warn")
                save = False
            if not save:
                writer.discard()
                self.log(f"{rec.kind.capitalize()} discarded ({reason}).")
                return
            meta = self._metadata(rec)
            if rec.kind == "demo":
                self._keep_demo(writer)
            path = writer.save(meta)
            hz = meta["measured_hz"]
            self.log(
                f"{rec.kind.capitalize()} saved ({reason}): {meta['num_steps']} samples, {meta['duration_s']:.1f} s, "
                f"{hz:.1f} Hz" + "".join(f", {c} {n} frames" for c, n in meta["camera_frames"].items())
                + ("" if rec.staging else f" -> {path}")
            )
            dropped = {c: n for c, n in writer.dropped_frames.items() if n}
            if dropped or writer.write_errors:
                self.log(f"Frames dropped (disk too slow): {dropped}, write errors: {writer.write_errors}", "warn")
            self.last_saved = {"path": str(path), "kind": rec.kind, **{k: meta[k] for k in ("num_steps", "duration_s")}}
            if cfg.save_lerobot:
                self._queue_lerobot(path, cleanup=rec.staging)
            self.saved_rev += 1
        except Exception as exc:
            LOG.exception("finishing the recording failed")
            self.log(f"Saving failed: {exc}", "error")
        finally:
            with self._lock:
                if self.rec is rec:
                    self.rec = None
                if self.state == State.BUSY:
                    self.state = State.IDLE

    def _metadata(self, rec: Recording) -> Dict[str, Any]:
        cfg, w = self.cfg, rec.writer
        ts = w.timestamps
        duration = float(ts[-1] - ts[0]) if len(ts) > 1 else 0.0
        meta: Dict[str, Any] = {
            "kind": rec.kind,
            "mode": cfg.mode,
            "task": cfg.task,
            "task_instruction": cfg.instruction,
            "fps": cfg.record_hz,
            "record_hz": cfg.record_hz,
            "num_steps": len(ts),
            "duration_s": duration,
            "measured_hz": (len(ts) - 1) / duration if duration > 0 else 0.0,
            "created": datetime.now().isoformat(timespec="seconds"),
            "robot_name": cfg.robot_name,
            "leader": w.leader_name,
            "cameras": {c.name: c.hz for c in cfg.cameras},
            "camera_frames": w.frame_counts(),
            "dropped_frames": w.dropped_frames,
            "skipped_samples": rec.missing,
            "schedule_overruns": rec.overruns,
            "sim": cfg.sim,
            "saved_irl": not rec.staging,
        }
        if cfg.mode == "gello":
            meta["gello_gains"] = cfg.gello_gains
        if rec.kind == "replay" and self.demo is not None:
            meta.update(
                demo=str(self.demo.source) if self.demo.source else None,
                demo_duration_s=self.demo.duration,
                control_mode=GAINS[cfg.replay_gains].value,
                replay_rate_hz=cfg.replay_hz,
                velocity_feedforward=cfg.replay_feedforward,
                settle_time_s=cfg.replay_settle_s,
            )
        return meta

    # ============================================================ robot ops
    def gripper(self, close: bool) -> None:
        with self._lock:
            if self.rig is None:
                raise ActionError("Not connected.")
            if self.state not in (State.IDLE, State.RECORDING):
                raise ActionError(f"Not possible while {self.state.value}.")
            if self.rec is not None and self.rec.kind == "replay":
                raise ActionError("The replay controls the gripper.")
            if self.cfg.mode == "gello" and self.state == State.RECORDING:
                raise ActionError("The GELLO trigger controls the gripper while recording.")
            self._gripper_cmd = 1.0 if close else 0.0
            if self.rig.mode == "keyboard":
                self.rig.leader.gripper = self._gripper_cmd
        self.rig.send_gripper(self._gripper_cmd)

    def keys(self, keys: List[str], slow: bool) -> Dict[str, Any]:
        """Held keys from the browser (keyboard mode); refreshed while held (dead-man)."""
        rig = self.rig
        if rig is None or rig.mode != "keyboard":
            raise ActionError("Keyboard teleop needs the keyboard mode and a connection.")
        rig.leader.set_keys([str(k) for k in keys], slow)
        return {}

    def keyboard_floor_to_current(self) -> Dict[str, Any]:
        """Workspace z minimum := the measured EE height (lower the pusher to the table first)."""
        with self._lock:
            self._require(State.IDLE)
            state = self.rig.arm.current_state if self.rig is not None else None
        if not state or "EE_pos" not in state:
            raise ActionError("Arm state missing.")
        z = round(float(state["EE_pos"][2]), 4)
        lo = list(self.cfg.kb_workspace_min)
        lo[2] = z
        cfg = self.update_config({"kb_workspace_min": lo})
        self.log(f"Keyboard workspace: z floor set to the current EE height {z:.4f} m.")
        return {"kb_workspace_min": cfg.kb_workspace_min}

    def reset(self) -> None:
        with self._lock:
            self._require(State.IDLE)
            if self.rig is None:
                raise ActionError("Not connected.")
            rig = self.rig
            self._set_busy(RESET_LABEL[self.cfg.mode])

        def work() -> None:
            try:
                if rig.mode == "keyboard":
                    rig.pair.home_position = tuple(self.cfg.home_q)
                rig.pair.control_reset()
                self.log(f"Reset done ({RESET_TEXT[self.cfg.mode]}).")
            except Exception as exc:
                self.log(f"Reset failed: {exc}", "error")
            finally:
                with self._lock:
                    if self.state == State.BUSY:
                        self.state = State.IDLE

        self._spawn(work, "reset")

    def home(self) -> None:
        from ..franka_robot.panda_arm import ControlMode

        with self._lock:
            self._require(State.IDLE)
            if self.rig is None:
                raise ActionError("Not connected.")
            rig = self.rig
            self._set_busy("Moving home")

        def work() -> None:
            try:
                home = [float(q) for q in self.cfg.home_q]
                self.log(f"Moving to home [{', '.join(f'{q:.2f}' for q in home)}]...")
                rig.arm.set_franka_arm_control_mode(ControlMode.IDLE)
                rig.arm.move_franka_arm_to_joint_position(home)
                if rig.mode != "keyboard":  # keyboard: keep holding the pusher
                    rig.send_gripper(0.0)
                    self._gripper_cmd = 0.0
                if rig.mode == "vr" and rig.leader is not None:
                    rig.leader.reset()  # re-anchor the VR target at the new pose
                self.log("Home reached.")
            except Exception as exc:
                self.log(f"Homing failed: {exc}", "error")
            finally:
                with self._lock:
                    if self.state == State.BUSY:
                        self.state = State.IDLE

        self._spawn(work, "home")

    def set_home_to_current(self) -> Dict[str, Any]:
        with self._lock:
            self._require(State.IDLE)
            state = self.rig.arm.current_state if self.rig is not None else None
        if not state or state.get("q") is None:
            raise ActionError("Arm state missing.")
        cfg = self.update_config({"home_q": [round(float(q), 4) for q in state["q"]][:7]})
        return {"home_q": list(cfg.home_q)}

    # ========================================================== teach & replay
    def _keep_demo(self, writer: IRLEpisodeWriter) -> None:
        from ..data_collection.teach_replay_collection import Demo

        cols, name = writer.arrays(), self.cfg.robot_name
        self.demo = Demo.from_arrays(
            writer.timestamps, cols[f"{name}/joint_pos.pt"], cols[f"{name}/joint_vel.pt"],
            cols[f"{name}/gripper_command.pt"], source=writer.record_dir, O_T_EE=cols[f"{name}/ee_pos.pt"],
        )
        self.log(f"Demo ready for replay: {self.demo.duration:.1f} s, {len(self.demo.t)} samples.")

    def use_demo(self, path: str) -> Dict[str, Any]:
        from ..data_collection.teach_replay_collection import Demo

        with self._lock:
            self._require(State.IDLE, State.DISCONNECTED)
        if self.cfg.mode != "teach_replay":
            raise ActionError("Switch to the teach & replay mode first.")
        try:
            self.demo = Demo.load(Path(path), self.cfg.robot_name)
        except Exception as exc:
            raise ActionError(f"Cannot load the demo: {exc}") from exc
        self.log(f"Demo loaded from {path}: {self.demo.duration:.1f} s.")
        return {}

    def replay(self) -> None:
        from ..data_collection.teach_replay_collection import DemoReplayer

        with self._lock:
            self._require(State.IDLE)
            if self.cfg.mode != "teach_replay":
                raise ActionError("Replay is only available in the teach & replay mode.")
            if self.rig is None or self.demo is None:
                raise ActionError("Record (or choose) a demo first.")
            if not self.cfg.save_irl and not self.cfg.save_lerobot:
                raise ActionError("Neither IRL nor LeRobot saving is on.")
            if self.cfg.save_lerobot:
                self._check_lerobot_options()
            rig, demo, cfg = self.rig, self.demo, self.cfg
            self._set_busy("Moving to the demo start (keep clear)")

        def work() -> None:
            try:
                self.replayer = DemoReplayer(
                    rig.robot, demo, GAINS[cfg.replay_gains], cfg.replay_hz, cfg.replay_feedforward
                )
                self.log("Moving to the demo start pose, keep clear of the robot...")
                self.replayer.move_to_start()
                self.replayer.start()
                rec = self._begin("replay")
                with self._lock:
                    self.state = State.RECORDING
                self.log(f"Replaying the {demo.duration:.1f} s demo, recording -> {rec.writer.record_dir}")
            except Exception as exc:
                if self.replayer is not None:
                    self.replayer.stop()
                    self.replayer = None
                self.log(f"Replay failed: {exc}", "error")
                with self._lock:
                    if self.state == State.BUSY:
                        self.state = State.IDLE

        self._spawn(work, "replay")

    # ============================================================== episodes
    def episodes(self) -> Dict[str, Any]:
        root = self.cfg.task_dir
        items = []
        if root.is_dir():
            for ep in find_episodes(root):
                key = str(ep)
                try:
                    mtime = (ep / METADATA).stat().st_mtime if (ep / METADATA).exists() else ep.stat().st_mtime
                except OSError:
                    continue
                cached = self._episode_cache.get(key)
                if cached is None or cached[0] != mtime:
                    cached = (mtime, summarize_episode(ep, root))
                    meta_path = ep / METADATA
                    if meta_path.exists():
                        try:
                            cached[1]["lerobot"] = json.loads(meta_path.read_text(encoding="utf-8")).get("lerobot")
                        except ValueError:
                            pass
                    self._episode_cache[key] = cached
                items.append(cached[1])
        return {"root": str(root), "episodes": items, "rev": self.saved_rev,
                "demo": str(self.demo.source) if self.demo is not None and self.demo.source else None}

    def _check_path_in_task(self, path: str) -> Path:
        p = Path(path).expanduser().resolve()
        root = self.cfg.task_dir.resolve()
        if root not in p.parents or not is_episode_dir(p):
            raise ActionError("Not an episode of the current task folder.")
        return p

    def delete_episode(self, path: str) -> Dict[str, Any]:
        with self._lock:
            if self.state in (State.RECORDING, State.BUSY):
                raise ActionError(f"Not possible while {self.state.value}.")
            p = self._check_path_in_task(path)
        shutil.rmtree(p)
        self._episode_cache.pop(str(p), None)
        self.saved_rev += 1
        self.log(f"Deleted {p} (its LeRobot episode, if any, stays in the dataset; rebuild it in the Convert tab).", "warn")
        return {}

    # ========================================================= live LeRobot
    def _lerobot_options(self):
        from .irl_to_lerobot import ConvertOptions

        cfg = self.cfg
        return ConvertOptions(
            fps=cfg.lerobot_fps,
            state_space=cfg.lerobot_state_space,
            action_source=cfg.lerobot_action_source,
            image_size=cfg.lerobot_image_size,
            use_videos=cfg.lerobot_use_videos,
            force_torque=cfg.lerobot_force_torque,
            task=cfg.instruction,
            robot_type=cfg.robot_type,
        )

    def _check_lerobot_options(self) -> None:
        cfg = self.cfg
        try:
            import lerobot.datasets.lerobot_dataset  # noqa: F401
        except Exception as exc:
            raise ActionError(f"LeRobot saving is on but lerobot cannot be imported: {exc}") from exc
        if cfg.mode in ("kinesthetic", "teach_replay") and cfg.lerobot_action_source == "leader":
            raise ActionError("Kinesthetic demos have no leader device: use auto or next_state LeRobot actions.")
        if cfg.lerobot_action_source == "leader":
            leader_space = {"gello": "joint", "vr": "cartesian", "keyboard": "cartesian"}.get(cfg.mode)
            space = cfg.lerobot_state_space
            if leader_space and space != "auto" and space != leader_space:
                raise ActionError(f"The {cfg.mode} leader is {leader_space}-space: pick that state space or next_state actions.")

    def _queue_lerobot(self, path: Path, cleanup: bool) -> None:
        job = {"path": Path(path), "cleanup": cleanup, "root": self.cfg.lerobot_dir, "opts": self._lerobot_options()}
        with self._lock:
            self.lerobot_status["pending"] += 1
        self._lerobot_q.put(job)

    def requeue_lerobot(self, path: str) -> Dict[str, Any]:
        p = self._check_path_in_task(path)
        self._check_lerobot_options()
        self._queue_lerobot(p, cleanup=False)
        self.log(f"Queued {p.name} for the LeRobot dataset {self.cfg.lerobot_dir}.")
        return {}

    def _lerobot_worker(self) -> None:
        from .irl_to_lerobot import EpisodePlan, LeRobotSink

        while True:
            job = self._lerobot_q.get()
            if job is None:
                return
            path, root, opts = job["path"], job["root"], job["opts"]

            def progress(msg: str, frac: Optional[float]) -> None:
                self.lerobot_status["current"] = msg

            try:
                plan = EpisodePlan(path, opts)
                sink = LeRobotSink(root, opts.fps, plan.features(), opts.robot_type, opts.use_videos)
                sink.open()
                try:
                    index = sink.add_episode(plan, progress)
                finally:
                    sink.close()
                info = {"root": str(root), "episode_index": index, "frames": plan.n_frames, "fps": opts.fps}
                with self._lock:
                    self.lerobot_status["done"] += 1
                    self.lerobot_status["last"] = {"source": str(path), **info}
                self.log(f"LeRobot: {path.name} -> episode {index} of {root} ({plan.n_frames} frames at {opts.fps:g} fps)"
                         + (f"; {'; '.join(plan.notes)}" if plan.notes and not job["cleanup"] else ""))
                if job["cleanup"]:
                    shutil.rmtree(path, ignore_errors=True)
                else:
                    meta_path = path / METADATA
                    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.exists() else {}
                    meta["lerobot"] = info
                    meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
                    self.saved_rev += 1
            except Exception as exc:
                LOG.exception("LeRobot export failed")
                with self._lock:
                    self.lerobot_status["failed"] += 1
                self.log(f"LeRobot export of {path} failed: {exc}"
                         + (" (the IRL copy is kept there)" if job["cleanup"] else ""), "error")
            finally:
                with self._lock:
                    self.lerobot_status["pending"] -= 1
                    self.lerobot_status["current"] = ""

    # ======================================================= convert (import)
    def scan_irl(self, path: str) -> Dict[str, Any]:
        root = Path(path).expanduser()
        if not root.is_dir():
            raise ActionError(f"{root} is not a folder.")
        eps = find_episodes(root)
        return {"root": str(root), "episodes": [summarize_episode(ep, root) for ep in eps]}

    def start_convert(self, p: Dict[str, Any]) -> Dict[str, Any]:
        from .irl_to_lerobot import ConvertOptions, ConversionCancelled, convert

        if self.convert_status["running"]:
            raise ActionError("A conversion is already running.")
        episodes = [Path(e) for e in p.get("episodes") or []]
        if not episodes:
            raise ActionError("Select at least one episode.")
        output = str(p.get("output") or "").strip()
        if not output:
            raise ActionError("Choose an output folder.")
        w, h = int(p.get("width") or 0), int(p.get("height") or 0)
        try:
            opts = ConvertOptions(
                fps=float(p.get("fps") or 15),
                state_space=str(p.get("state_space") or "auto"),
                action_source=str(p.get("action_source") or "auto"),
                image_size=(w, h) if w > 0 and h > 0 else None,
                use_videos=bool(p.get("use_videos", True)),
                force_torque=bool(p.get("force_torque", True)),
                cameras=list(p.get("cameras")) if p.get("cameras") is not None else None,
                camera_hz={k: float(v) for k, v in (p.get("camera_hz") or {}).items() if v not in (None, "")},
                task=str(p.get("task") or ""),
                robot_type=str(p.get("robot_type") or "franka_panda"),
            )
        except (TypeError, ValueError) as exc:
            raise ActionError(f"Invalid options: {exc}") from exc
        if not 1 <= opts.fps <= 200:
            raise ActionError("fps must be within [1, 200]")
        existing = str(p.get("existing") or "fail")
        self._convert_cancel.clear()
        self.convert_status = {"running": True, "message": "Starting...", "fraction": 0.0, "result": None, "error": None}

        def progress(msg: str, frac: Optional[float]) -> None:
            self.convert_status["message"] = msg
            if frac is not None:
                self.convert_status["fraction"] = float(frac)

        def work() -> None:
            self.log(f"Converting {len(episodes)} IRL episodes -> {output} at {opts.fps:g} fps...")
            try:
                result = convert(episodes, Path(output), opts, existing, progress, self._convert_cancel)
                self.convert_status.update(result=result, message=f"Done: {len(result['written'])} episodes, "
                                           f"{result['frames']} frames, {len(result['skipped'])} skipped")
                self.log(f"Conversion done: {len(result['written'])} episodes -> {result['output']}"
                         + (f", {len(result['skipped'])} skipped" if result["skipped"] else ""))
                for s in result["skipped"][:10]:
                    self.log(f"  skipped {s['source']}: {s['error']}", "warn")
            except ConversionCancelled:
                self.convert_status.update(error="cancelled", message="Cancelled (finished episodes are kept)")
                self.log("Conversion cancelled; the episodes finished so far are in the dataset.", "warn")
            except Exception as exc:
                LOG.exception("conversion failed")
                self.convert_status.update(error=str(exc), message=f"Failed: {exc}")
                self.log(f"Conversion failed: {exc}", "error")
            finally:
                self.convert_status["running"] = False

        self._spawn(work, "convert")
        return {}

    def cancel_convert(self) -> None:
        if self.convert_status["running"]:
            self._convert_cancel.set()

    # ================================================================ viewer
    def open_viewer(self, path: str) -> Dict[str, Any]:
        from .lerobot_viewer import DatasetView

        try:
            view = DatasetView(Path(path))
        except Exception as exc:
            raise ActionError(str(exc)) from exc
        self.viewer = view
        return view.summary()

    def viewer_or_error(self):
        if self.viewer is None:
            raise ActionError("Open a dataset first.")
        return self.viewer

    def launch_rerun(self, episode: int) -> Dict[str, Any]:
        from .lerobot_viewer import launch_rerun

        view = self.viewer_or_error()
        launch_rerun(view.root, episode)
        self.log(f"Opening episode {episode} of {view.root} in Rerun (window on this machine)...")
        return {}

    # ================================================================ misc
    def clear_error(self) -> None:
        with self._lock:
            self.error = None
            if self.state == State.ERROR:
                self.state = State.DISCONNECTED if self.rig is None else State.IDLE

    def shutdown(self) -> None:
        if self.state == State.RECORDING and self.rec is not None:
            self.log("Recording discarded at shutdown.", "warn")
            rec = self.rec
            rec.stop.set()
            if rec.thread is not None:
                rec.thread.join(timeout=3)
            if self.replayer is not None:
                self.replayer.stop()
            rec.writer.discard()
        self._convert_cancel.set()
        pending = self.lerobot_status["pending"]
        if pending:
            self.log(f"Waiting for {pending} LeRobot export(s) to finish...", "warn")
        self._lerobot_q.put(None)
        self._lerobot_thread.join()
        if self.rig is not None:
            self.rig.close()
            if not self.rig.sim:
                self.rig.set_idle()
            self.rig = None

    def robot_snapshot(self) -> Dict[str, Any]:
        rig = self.rig
        if rig is None:
            return {}
        out: Dict[str, Any] = {}
        try:
            arm = rig.arm.current_state
            if arm:
                out["q"] = [float(v) for v in arm["q"]][:7]
                if "EE_pos" in arm:
                    out["ee_pos"] = [float(v) for v in arm["EE_pos"]]
            grip = rig.gripper.current_state
            if isinstance(grip, dict):
                out["gripper"] = float(grip.get("position", 0.0))
            if rig.mode == "gello" and rig.leader is not None:
                st = rig.leader.current_state
                if st and st.get("gello_arm_state"):
                    out["leader_q"] = [float(v) for v in st["gello_arm_state"]["joint_state"]][:7]
                if st and st.get("gello_gripper_state"):
                    out["leader_gripper"] = float(np.asarray(st["gello_gripper_state"]["gripper"]).reshape(-1)[0])
            elif rig.mode == "keyboard" and rig.leader is not None:
                keys, slow = rig.leader.held_keys()
                out["keys"] = sorted(keys) + (["shift"] if slow and keys else [])
                sig = rig.leader.current_control_signal
                if sig and rig.pair.is_running:
                    out["leader_pos"] = [float(v) for v in sig["pos"]]
                out["workspace"] = [rig.leader.lo.tolist(), rig.leader.hi.tolist()]
            elif rig.mode == "vr" and rig.leader is not None:
                sig = rig.leader.current_control_signal
                if sig:
                    out["leader_pos"] = [float(v) for v in sig["pos"]]
                    out["leader_gripper"] = float(np.asarray(sig["gripper_width"]).reshape(-1)[0])
        except Exception as exc:  # status must never fail because of a device
            out["error"] = str(exc)
        return out

    def status(self, log_after: int) -> Dict[str, Any]:
        with self._lock:
            logs = [{"seq": s, "t": t, "level": lvl, "msg": m} for (s, t, lvl, m) in self._logs if s > log_after]
            state, rec, cfg = self.state, self.rec, self.cfg
        recording = None
        if rec is not None:
            w = rec.writer
            recording = {
                "kind": rec.kind,
                "elapsed_s": time.time() - rec.t_start,
                "samples": w.n_samples,
                "hz": rec.rate(),
                "frames": w.frame_counts(),
                "dropped": w.dropped_frames,
                "pending_writes": w.pending_writes,
                "missing": rec.missing,
                "overruns": rec.overruns,
                "dir": str(w.record_dir),
                "staging": rec.staging,
                "limit_s": (self.demo.duration + cfg.replay_settle_s) if rec.kind == "replay" and self.demo
                else (cfg.max_episode_s or None),
            }
        return {
            "now": time.time(),
            "state": state.value,
            "busy_label": self.busy_label if state == State.BUSY else "",
            "error": self.error,
            "connected": self.rig is not None,
            "sim": cfg.sim,
            "mode": cfg.mode,
            "recording": recording,
            "robot": self.robot_snapshot(),
            "cameras": list(self.rig.cameras) if self.rig is not None else [],
            "demo": {"source": str(self.demo.source) if self.demo.source else None, "duration_s": self.demo.duration,
                     "samples": len(self.demo.t)} if self.demo is not None else None,
            "saved_rev": self.saved_rev,
            "last_saved": self.last_saved,
            "lerobot": {**self.lerobot_status, "enabled": cfg.save_lerobot, "root": str(cfg.lerobot_dir)},
            "convert": self.convert_status,
            "paths": {"task_dir": str(cfg.task_dir), "lerobot_dir": str(cfg.lerobot_dir)},
            "logs": logs,
        }

    def camera_frame(self, name: str) -> Optional[np.ndarray]:
        rig = self.rig
        if rig is None or name not in rig.cameras:
            return None
        return rig.cameras[name].get_image()


RESET_TEXT = {
    "gello": "move the arm to the GELLO pose",
    "vr": "go home and re-anchor the VR controller",
    "kinesthetic": "go home and enable hand guiding",
    "teach_replay": "go home and enable hand guiding",
    "keyboard": "go home (gripper untouched)",
}
RESET_LABEL = {
    "gello": "Aligning the arm to the GELLO",
    "vr": "Resetting (home + VR anchor)",
    "kinesthetic": "Going home, then hand guiding",
    "teach_replay": "Going home, then hand guiding",
    "keyboard": "Going home",
}


def list_dir(path: str) -> Dict[str, Any]:
    """Folder browser for the GUI: sub folders, tagged as IRL episode / LeRobot dataset."""
    p = Path(path or "~").expanduser()
    if not p.is_dir():
        p = p.parent if p.parent.is_dir() else Path.home()
    p = p.resolve()
    entries = []
    try:
        children = sorted((c for c in p.iterdir() if c.is_dir() and not c.name.startswith(".")), key=lambda c: c.name)
    except OSError as exc:
        raise ActionError(f"Cannot list {p}: {exc}") from exc
    for c in children[:2000]:
        kind = ""
        if (c / "meta" / "info.json").is_file():
            kind = "lerobot"
        elif (c / "timestamps.pt").is_file():
            kind = "irl_episode"
        entries.append({"name": c.name, "path": str(c), "kind": kind})
    return {
        "path": str(p),
        "parent": str(p.parent) if p.parent != p else None,
        "kind": "lerobot" if (p / "meta" / "info.json").is_file() else ("irl_episode" if (p / "timestamps.pt").is_file() else ""),
        "entries": entries,
    }
