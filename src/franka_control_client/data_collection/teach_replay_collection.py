"""Kinesthetic teaching + replay data collection.

The arm is hand-guided in the proxy's HumanControl mode (libfranka torque control with zero
commanded torque: libfranka compensates gravity, so the arm floats). A demonstration is recorded,
then replayed by the robot itself in a joint impedance mode while everything is recorded again.

Layout (every episode has the usual IRLDataCollection structure, the dataset tools find them):
    <data_dir>/demo/<ts>/{timestamps.pt, FrankaPanda/*.pt, sensors/<cam>/*.png, metadata.json}
    <data_dir>/replay/<ts>/{... same ..., Traj/{joint_pos,joint_vel,gripper_state}.pt}
FrankaPanda/gripper_command.pt holds the commanded gripper position (what the replay sends);
replay Traj/ holds the commands sent at each recorded step (same length as the robot state).

Phases and keys (case-insensitive, shown in the bottom hint line):
    FREE        hand guiding, no recording   space/n: start demo, o/p: gripper, h: home, q: quit
    TEACHING    hand guiding, recording      space/s: save demo, d: discard, o/p: gripper
    DEMO_READY  demo in memory               space/n: replay, t: teach again (here), h: home + teach
    REPLAYING   robot replays, recording     s: stop + save, d/esc: abort + discard
    BUSY        robot moving (home, start)   keys ignored
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np
import pyzlc
import torch

from ..franka_robot.panda_arm import ControlMode
from ..franka_robot.panda_robotiq import PandaRobotiq
from .irl_data_collection import IRLDataCollection
from .irl_wrapper import IRL_HardwareDataWrapper
from .utils import NonBlockingKeyPress

GRIPPER_SPEED = 0.7
GRIPPER_FORCE = 0.3
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 1.0
GRIPPER_DEADBAND = 1e-3
HOME_POSITION = (0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0)


@dataclass
class Demo:
    """A demonstration, time relative to its first sample [s]."""

    t: np.ndarray  # (N,)
    q: np.ndarray  # (N, 7)
    dq: np.ndarray  # (N, 7)
    gripper: np.ndarray  # (N,) commanded gripper position
    source: Optional[Path] = None
    ee: Optional[np.ndarray] = None  # (N, 4, 4) end-effector pose O_T_EE

    @property
    def duration(self) -> float:
        return float(self.t[-1])

    def insertion(self, zone: float) -> Tuple[np.ndarray, float, float]:
        """Insertion axis (tool z at the deepest point, base frame), time the demo enters the last
        `zone` [m] before its deepest point along that axis, and time of the deepest point."""
        if self.ee is None:
            raise ValueError("Demo has no end-effector poses (ee_pos.pt)")
        pos = self.ee[:, :3, 3]
        k_deep = int(np.argmax(pos @ self.ee[-1, :3, 2]))
        axis = self.ee[k_deep, :3, 2]
        depth = pos @ axis
        outside = np.where(depth[: k_deep + 1] < depth[k_deep] - zone)[0]
        k_start = int(outside[-1]) + 1 if len(outside) else 0
        return axis, float(self.t[k_start]), float(self.t[k_deep])

    def command_at(self, t: float) -> Tuple[np.ndarray, np.ndarray, float]:
        """Joint position/velocity (linear interpolation) and gripper command (held) at time t."""
        t = float(np.clip(t, 0.0, self.duration))
        q = np.array([np.interp(t, self.t, self.q[:, j]) for j in range(7)])
        dq = np.array([np.interp(t, self.t, self.dq[:, j]) for j in range(7)])
        idx = max(int(np.searchsorted(self.t, t, side="right")) - 1, 0)
        return q, dq, float(self.gripper[idx])

    @classmethod
    def from_arrays(cls, timestamps, q, dq, gripper, source: Optional[Path] = None, O_T_EE=None) -> "Demo":
        """O_T_EE: (N, 16) as libfranka sends it (column-major 4x4), optional."""
        t = np.asarray(timestamps, dtype=np.float64).reshape(-1)
        n = min(len(t), len(q), len(dq), len(gripper))
        ee = None
        if O_T_EE is not None:
            ee = np.asarray(O_T_EE, dtype=np.float64).reshape(-1, 4, 4).transpose(0, 2, 1)
            n = min(n, len(ee))
            ee = ee[:n]
        if n < 2:
            raise ValueError(f"Demo too short ({n} samples)")
        t = t[:n] - t[0]
        if np.any(np.diff(t) <= 0):
            raise ValueError("Demo timestamps are not strictly increasing")
        return cls(
            t=t,
            q=np.asarray(q, dtype=np.float64).reshape(-1, 7)[:n],
            dq=np.asarray(dq, dtype=np.float64).reshape(-1, 7)[:n],
            gripper=np.clip(np.asarray(gripper, dtype=np.float64).reshape(-1)[:n], 0.0, 1.0),
            source=source,
            ee=ee,
        )

    @classmethod
    def load(cls, episode_dir: Path, arm_name: str = "FrankaPanda") -> "Demo":
        """Load a recorded episode (demo or any other IRLDataCollection episode)."""
        episode_dir = Path(episode_dir)
        arm_dir = episode_dir / arm_name
        load = lambda p: torch.load(p, map_location="cpu").numpy()
        gripper_path = arm_dir / "gripper_command.pt"
        if not gripper_path.exists():
            # older episodes: measured gripper position instead of the command
            gripper_path = arm_dir / "gripper_state.pt"
        return cls.from_arrays(
            load(episode_dir / "timestamps.pt"),
            load(arm_dir / "joint_pos.pt"),
            load(arm_dir / "joint_vel.pt"),
            load(gripper_path),
            source=episode_dir,
            O_T_EE=load(arm_dir / "ee_pos.pt") if (arm_dir / "ee_pos.pt").exists() else None,
        )


@dataclass
class PushProfile:
    """Feed-forward force along the insertion axis during a time window of the demo.

    Position replay only pushes as hard as stiffness x tracking error. A kinesthetic demo cannot
    tell how hard the human pushed (hand and contact forces cancel in the measured external
    wrench), so the insertion force is a parameter: ramped in until the demo enters the last
    `zone` before its deepest point, held until `hold` after it, ramped out.
    """

    force: float  # [N]
    axis: np.ndarray  # (3,) unit vector, base frame
    t_on: float  # full force from here [s, demo time]
    t_off: float  # ramp out from here
    ramp: float  # [s]

    @classmethod
    def from_demo(cls, demo: Demo, force: float, zone: float, hold: float, ramp: float) -> "PushProfile":
        axis, t_on, t_deep = demo.insertion(zone)
        if not np.linalg.norm(axis) > 0.5:
            raise ValueError(f"Invalid insertion axis {axis} (end-effector poses of the demo broken?)")
        # the wrench has to be back at zero before the replay stops sending commands
        t_off = max(t_on, min(t_deep + hold, demo.duration - ramp))
        return cls(force=force, axis=axis / np.linalg.norm(axis), t_on=t_on, t_off=t_off, ramp=ramp)

    def wrench_at(self, t: float) -> np.ndarray:
        ramp = max(self.ramp, 1e-3)
        scale = np.clip((t - (self.t_on - ramp)) / ramp, 0.0, 1.0) * np.clip(
            (self.t_off + ramp - t) / ramp, 0.0, 1.0
        )
        return np.concatenate([self.force * scale * self.axis, np.zeros(3)])


class DemoReplayer:
    """Streams a demo to the arm with its original timing (from the recorded timestamps)."""

    def __init__(
        self,
        robot: PandaRobotiq,
        demo: Demo,
        control_mode: ControlMode,
        rate_hz: float = 200.0,
        velocity_feedforward: bool = True,
        push: Optional[PushProfile] = None,
    ) -> None:
        self.robot = robot
        self.demo = demo
        self.control_mode = control_mode
        self.rate_hz = rate_hz
        self.velocity_feedforward = velocity_feedforward
        self.push = push
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._t0: Optional[float] = None
        self._last_gripper: Optional[float] = None

    def move_to_start(self) -> None:
        """Blocking: switch to the replay controller and move to the first demo sample."""
        q0, _, g0 = self.demo.command_at(0.0)
        # impedance mode first: it restarts at the start pose after the move and latches it
        self.robot.panda_arm.set_franka_arm_control_mode(self.control_mode)
        self.robot.panda_arm.move_franka_arm_to_joint_position(q0)
        self.robot.robotiq_gripper.send_grasp_command(
            position=g0, speed=GRIPPER_SPEED, force=GRIPPER_FORCE, blocking=True
        )
        self._last_gripper = g0

    def start(self) -> float:
        """Start streaming; returns the start time (time.perf_counter, like the recorder)."""
        self._t0 = time.perf_counter()
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self._t0

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        if self.push is not None:
            self.robot.panda_arm.send_ee_wrench_command(np.zeros(6))

    def elapsed(self) -> float:
        return 0.0 if self._t0 is None else time.perf_counter() - self._t0

    def command_at_time(self, perf_time: float) -> Tuple[np.ndarray, np.ndarray, float]:
        """Command sent at time.perf_counter() == perf_time (the vel actually sent)."""
        assert self._t0 is not None
        q, dq, g = self.demo.command_at(perf_time - self._t0)
        if not self.velocity_feedforward or perf_time - self._t0 >= self.demo.duration:
            dq = np.zeros(7)
        return q, dq, g

    def wrench_at_time(self, perf_time: float) -> np.ndarray:
        """Feed-forward wrench sent at time.perf_counter() == perf_time."""
        if self.push is None or self._t0 is None or perf_time - self._t0 >= self.demo.duration:
            return np.zeros(6)
        return self.push.wrench_at(perf_time - self._t0)

    def _run(self) -> None:
        period = 1.0 / self.rate_hz
        next_tick = time.perf_counter()
        while self._running:
            now = time.perf_counter()
            q, dq, g = self.command_at_time(now)
            self.robot.panda_arm.send_joint_position_command(q, dq)
            if self.push is not None:
                self.robot.panda_arm.send_ee_wrench_command(self.wrench_at_time(now))
            if self._last_gripper is None or abs(g - self._last_gripper) > GRIPPER_DEADBAND:
                self.robot.robotiq_gripper.send_grasp_command(
                    position=g, speed=GRIPPER_SPEED, force=GRIPPER_FORCE, blocking=False
                )
                self._last_gripper = g
            if now - self._t0 >= self.demo.duration:
                # last sample sent with zero velocity: the controller holds it
                break
            next_tick += period
            time.sleep(max(0.0, next_tick - time.perf_counter()))


class Phase(str, Enum):
    FREE = "free"
    TEACHING = "teaching"
    DEMO_READY = "demo_ready"
    REPLAYING = "replaying"
    BUSY = "busy"
    EXITING = "exiting"


HINTS = {
    Phase.FREE: "[FREE: hand guiding] space/n: start demo | o/p: open/close gripper | h: home | q: quit",
    Phase.TEACHING: "[TEACHING: recording] space/s: save demo | d: discard | o/p: open/close gripper",
    Phase.DEMO_READY: "[DEMO READY] space/n: replay (ROBOT MOVES to start) | t: teach new demo | h: home + teach | q: quit",
    Phase.REPLAYING: "[REPLAYING: recording] s: stop + save | d/esc: abort + discard",
    Phase.BUSY: "[BUSY] robot is moving, keys are ignored...",
    Phase.EXITING: "Exiting",
}


class TeachReplayCollection(IRLDataCollection):
    """Kinesthetic demo -> robot replay, both recorded (see module docstring)."""

    def __init__(
        self,
        robot: PandaRobotiq,
        data_collectors: List[IRL_HardwareDataWrapper],
        data_dir: Path,
        task: str,
        fps: int = 100,
        replay_mode: ControlMode = ControlMode.HybridJointImpedance,
        replay_rate_hz: float = 200.0,
        velocity_feedforward: bool = True,
        settle_time: float = 1.0,
        demo: Optional[Demo] = None,
        push_force: float = 0.0,
        push_zone: float = 0.01,
        push_hold: float = 0.5,
        push_ramp: float = 0.3,
    ) -> None:
        # no control pair: this class drives the robot itself
        super().__init__(data_collectors, Path(data_dir), task, control_pair=None, fps=fps)
        self.robot = robot
        self.replay_mode = replay_mode
        self.replay_rate_hz = replay_rate_hz
        self.velocity_feedforward = velocity_feedforward
        self.settle_time = settle_time
        self.demo = demo
        self.push_force = push_force
        self.push_zone = push_zone
        self.push_hold = push_hold
        self.push_ramp = push_ramp
        self.phase = Phase.BUSY
        self._phase_lock = threading.Lock()
        self._busy_thread: Optional[threading.Thread] = None
        self._episode_kind = "demo"
        self._gripper_cmd = GRIPPER_OPEN
        self._gripper_cmd_list: List[float] = []
        # commands at each replay step: joint pos, joint vel, gripper, ee wrench
        self._traj_lists: Tuple[List[np.ndarray], List[np.ndarray], List[float], List[np.ndarray]] = ([], [], [], [])
        self.replayer: Optional[DemoReplayer] = None
        self._replay_count = 0

    # ------------------------------------------------------------------ main loop

    def run(self) -> None:
        if self.demo is not None:
            self._log_demo("Loaded demo")
            self._run_busy(self._hold_position, Phase.DEMO_READY)
        else:
            self._run_busy(self._enable_hand_guiding, Phase.FREE)
        try:
            with NonBlockingKeyPress() as kp:
                while self.phase != Phase.EXITING:
                    key = kp.get_data()
                    if key:
                        self._handle_key(key)
                    if self.phase in (Phase.TEACHING, Phase.REPLAYING):
                        self._collect_step()
                        if self.phase == Phase.REPLAYING and self._replay_done():
                            self._finish_replay(save=True)
                    else:
                        time.sleep(0.005)
        finally:
            self._shutdown()

    def _enter(self, phase: Phase) -> None:
        with self._phase_lock:
            self.phase = phase
            self._ui_console.update_hint(HINTS[phase])

    def _run_busy(self, action: Callable[[], None], next_phase: Phase, fail_phase: Optional[Phase] = None) -> None:
        """Run a blocking robot action in the background; the key loop keeps running."""
        self._enter(Phase.BUSY)

        def task() -> None:
            try:
                action()
                self._enter(next_phase)
            except Exception as exc:
                self._ui_console.log(f"Robot action failed: {exc}")
                self._enter(fail_phase or (Phase.DEMO_READY if self.demo is not None else Phase.FREE))

        self._busy_thread = threading.Thread(target=task, daemon=True)
        self._busy_thread.start()

    def _handle_key(self, key: str) -> None:
        phase = self.phase
        if phase == Phase.BUSY:
            return
        if key == "q" and phase in (Phase.FREE, Phase.DEMO_READY):
            self._enter(Phase.EXITING)
        elif key in ("o", "p") and phase in (Phase.FREE, Phase.TEACHING):
            self._send_gripper(GRIPPER_OPEN if key == "o" else GRIPPER_CLOSED)
        elif phase == Phase.FREE:
            if key in (" ", "n"):
                self._start_episode("demo")
                self._enter(Phase.TEACHING)
            elif key == "h":
                self._run_busy(self._go_home_and_guide, Phase.FREE)
        elif phase == Phase.TEACHING:
            if key in (" ", "s"):
                self._finish_demo(save=True)
            elif key == "d":
                self._finish_demo(save=False)
        elif phase == Phase.DEMO_READY:
            if key in (" ", "n"):
                self._run_busy(self._prepare_replay, Phase.REPLAYING)
            elif key == "t":
                self._run_busy(self._enable_hand_guiding, Phase.FREE)
            elif key == "h":
                self._run_busy(self._go_home_and_guide, Phase.FREE)
        elif phase == Phase.REPLAYING:
            if key == "s":
                self._finish_replay(save=True)
            elif key in ("d", "esc"):
                self._finish_replay(save=False)

    # ------------------------------------------------------------------ robot actions

    def _enable_hand_guiding(self) -> None:
        self.robot.panda_arm.set_franka_arm_control_mode(ControlMode.HumanControl)
        self._ui_console.log("Hand guiding enabled: the arm floats (gravity compensated), guide it by hand.")

    def _go_home_and_guide(self) -> None:
        self.robot.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)
        self.robot.panda_arm.move_franka_arm_to_joint_position(HOME_POSITION)
        self._enable_hand_guiding()

    def _hold_position(self) -> None:
        # impedance mode latches the current position
        self.robot.panda_arm.set_franka_arm_control_mode(self.replay_mode)

    def _send_gripper(self, position: float) -> None:
        self._gripper_cmd = position
        self.robot.robotiq_gripper.send_grasp_command(
            position=position, speed=GRIPPER_SPEED, force=GRIPPER_FORCE, blocking=False
        )

    def _prepare_replay(self) -> None:
        assert self.demo is not None
        self._ui_console.log("Moving to the demo start pose, keep clear of the robot...")
        push = None
        if self.push_force > 0:
            push = PushProfile.from_demo(
                self.demo, self.push_force, self.push_zone, self.push_hold, self.push_ramp
            )
            self._ui_console.log(
                f"Insertion push {push.force:.0f} N along {np.round(push.axis, 2)} (base) "
                f"from t={push.t_on:.1f} s to t={push.t_off:.1f} s (ramp {push.ramp:.1f} s)"
            )
        self.replayer = DemoReplayer(
            self.robot, self.demo, self.replay_mode, self.replay_rate_hz, self.velocity_feedforward, push
        )
        self.replayer.move_to_start()
        # recording and streaming start together (from the main loop the next tick would be later)
        self._start_episode("replay")
        self.replayer.start()
        self._ui_console.log(f"Replaying {self.demo.duration:.1f} s demo...")

    # ------------------------------------------------------------------ episodes

    def _start_episode(self, kind: str) -> None:
        self._episode_kind = kind
        if kind == "demo":
            state = self.robot.robotiq_gripper.current_state
            if state is not None and 0.0 <= state["commanded_position"] <= 1.0:
                self._gripper_cmd = float(state["commanded_position"])
        self._gripper_cmd_list = []
        self._traj_lists = ([], [], [], [])
        self._start_collecting()

    def _collect_step(self) -> None:
        if self._episode_kind == "demo":
            # sampled before the step, the step timestamp is taken right after
            self._gripper_cmd_list.append(self._gripper_cmd)
        super()._collect_step()
        if self._episode_kind == "replay" and self.replayer is not None:
            q, dq, g = self.replayer.command_at_time(self.timestamps[-1])
            self._traj_lists[0].append(q)
            self._traj_lists[1].append(dq)
            self._traj_lists[2].append(g)
            self._traj_lists[3].append(self.replayer.wrench_at_time(self.timestamps[-1]))
            self._gripper_cmd_list.append(g)

    def _create_new_recording_dir(self):
        base = self.data_dir
        self.data_dir = base / self._episode_kind
        self.data_dir.mkdir(parents=True, exist_ok=True)
        try:
            super()._create_new_recording_dir()
        finally:
            self.data_dir = base

    def _save_data_task(self) -> None:
        super()._save_data_task()
        torch.save(
            torch.tensor(self._gripper_cmd_list, dtype=torch.float64),
            self.follower_robot_dir / "gripper_command.pt",
        )
        meta = {
            "kind": self._episode_kind,
            "task": self.task,
            "fps": self.fps,
            "num_steps": len(self.timestamps),
            "duration_s": float(self.timestamps[-1] - self.timestamps[0]) if len(self.timestamps) > 1 else 0.0,
            "created": datetime.now().isoformat(timespec="seconds"),
        }
        if self._episode_kind == "replay":
            traj_dir = self.record_dir / "Traj"
            traj_dir.mkdir(exist_ok=True)
            q_list, dq_list, g_list, w_list = self._traj_lists
            torch.save(torch.tensor(np.array(q_list), dtype=torch.float64), traj_dir / "joint_pos.pt")
            torch.save(torch.tensor(np.array(dq_list), dtype=torch.float64), traj_dir / "joint_vel.pt")
            torch.save(torch.tensor(g_list, dtype=torch.float64), traj_dir / "gripper_state.pt")
            torch.save(torch.tensor(np.array(w_list), dtype=torch.float64), traj_dir / "ee_wrench.pt")
            assert self.demo is not None
            push = self.replayer.push if self.replayer is not None else None
            if push is not None:
                meta["push"] = dict(
                    force_n=push.force, axis=push.axis.tolist(), t_on_s=push.t_on, t_off_s=push.t_off, ramp_s=push.ramp
                )
            meta.update(
                demo=str(self.demo.source) if self.demo.source else None,
                demo_duration_s=self.demo.duration,
                control_mode=self.replay_mode.value,
                replay_rate_hz=self.replay_rate_hz,
                velocity_feedforward=self.velocity_feedforward,
                settle_time_s=self.settle_time,
            )
        (self.record_dir / "metadata.json").write_text(json.dumps(meta, indent=2))

    def _finish_demo(self, save: bool) -> None:
        if not save or len(self.timestamps) < 2:
            if save:
                self._ui_console.log("Demo too short, discarded.")
            self._discard_collecting()
            self._enter(Phase.FREE)
            return
        data = self.follower_robot_data
        demo = Demo.from_arrays(
            self.timestamps,
            torch.stack(data.q_list).numpy(),
            torch.stack(data.dq_list).numpy(),
            self._gripper_cmd_list,
            source=self.record_dir,
            O_T_EE=torch.stack(data.O_T_EE_list).numpy(),
        )
        self._save_episode()
        self.demo = demo
        self._replay_count = 0
        self._log_demo("Demo saved")
        # the arm keeps floating: let go of it before replaying (it moves to the start pose first)
        self._enter(Phase.DEMO_READY)

    def _replay_done(self) -> bool:
        assert self.demo is not None and self.replayer is not None
        return self.replayer.elapsed() >= self.demo.duration + self.settle_time

    def _finish_replay(self, save: bool) -> None:
        assert self.replayer is not None
        self.replayer.stop()
        if save and len(self.timestamps) < 2:
            self._ui_console.log("Replay too short, discarded.")
            save = False
        if save:
            self._save_episode()
            self._replay_count += 1
            self._ui_console.log(f"Replay {self._replay_count} saved to {self.record_dir}")
        else:
            self._discard_collecting()
        # the impedance controller keeps holding the last commanded pose
        self.replayer = None
        self._enter(Phase.DEMO_READY)

    def _log_demo(self, prefix: str) -> None:
        assert self.demo is not None
        rate = (len(self.demo.t) - 1) / self.demo.duration if self.demo.duration > 0 else 0.0
        self._ui_console.log(
            f"{prefix}: {self.demo.duration:.1f} s, {len(self.demo.t)} samples ({rate:.0f} Hz) from {self.demo.source}"
        )

    def _shutdown(self) -> None:
        if self.replayer is not None:
            self.replayer.stop()
        if self._busy_thread is not None and self._busy_thread.is_alive():
            self._ui_console.log("Waiting for the robot action to finish...")
            self._busy_thread.join()
        self._writer_pool.shutdown(wait=True)
        try:
            self.robot.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)
        except Exception as exc:
            self._ui_console.log(f"Could not switch the arm to Idle: {exc}")
        print()
