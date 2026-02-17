from __future__ import annotations

import threading
import time
from typing import Optional, Union

import numpy as np
import pyzlc

from .control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode, RemotePandaArm
from ..franka_robot.panda_gripper import RemotePandaGripper
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper


DEFAULT_CONTROL_HZ: float = 10.0
GRIPPER_DEADBAND: float = 1e-3
GRIPPER_SPEED = 0.7
GRIPPER_FORCE= 0.3
ACTION_LOG_INTERVAL_S: float = 0.5
GRIPPER_TOGGLE_WARN_WINDOW_S: float = 3.0
GRIPPER_TOGGLE_WARN_COUNT: int = 6

class PolicyPandaControlPair(ControlPair):
    """
    Apply policy actions to a Panda arm with a gripper.

    Action semantics: [q0..q6, gripper] (joint positions + gripper).
    Gripper value is normalized in [0, 1]. It is scaled to device range.
    """

    def __init__(
        self,
        panda_arm: RemotePandaArm,
        gripper: Union[RemotePandaGripper, RemoteRobotiqGripper],
        control_hz: float = DEFAULT_CONTROL_HZ,
    ) -> None:
        super().__init__()
        self.panda_arm = panda_arm
        self.gripper = gripper
        self.control_hz = float(control_hz)
        self._action_lock = threading.Lock() #only one of the update_action and control_step visit latest_action at the same time 
        self._latest_action: Optional[np.ndarray] = None
        self._last_gripper_cmd: Optional[float] = None
        self._last_action_log_ts: float = 0.0
        self._last_gripper_binary: Optional[int] = None
        self._gripper_toggle_window_start_ts: float = time.time()
        self._gripper_toggle_count: int = 0

    def update_action(self, action: np.ndarray) -> None:
        """Update the latest action used by the control loop."""
        arr = np.asarray(action, dtype=np.float64).reshape(-1)
        if arr.size < 8:
            raise ValueError(f"Expected action size >= 8, got {arr.size}")
        with self._action_lock:
            self._latest_action = arr

    def _get_latest_action(self) -> Optional[np.ndarray]:
        with self._action_lock:
            if self._latest_action is None:
                return None
            return self._latest_action.copy()

    def control_rest(self) -> None:
        #todo:reset to default position
        self.panda_arm.set_franka_arm_control_mode(
            ControlMode.HybridJointImpedance
        )

    def control_step(self) -> None:
        action = self._get_latest_action()
        if action is None:
            pyzlc.sleep(1.0 / self.control_hz)
            return

        joint_pos = action[:7]
        gripper_cmd = float(np.clip(action[7], 0.0, 1.0))#binary for now
        self._log_action_debug(joint_pos, gripper_cmd)

        self.panda_arm.send_joint_position_command(joint_pos)

        if isinstance(self.gripper, RemoteRobotiqGripper):
            if (
                self._last_gripper_cmd is None
                or abs(gripper_cmd - self._last_gripper_cmd)
                > GRIPPER_DEADBAND
            ):
                self.gripper.send_grasp_command(
                    position=gripper_cmd,
                    speed=GRIPPER_SPEED,
                    force=GRIPPER_FORCE,
                    blocking=False,
                )
                self._last_gripper_cmd = gripper_cmd
        else:
            max_width = None
            state = self.gripper.current_state
            if state is not None:
                max_width = float(state.get("max_width", 0.0))
            if max_width is None or max_width <= 0.0:
                width = gripper_cmd
            else:
                width = gripper_cmd * max_width
            self.gripper.send_gripper_command(width=width, speed=0.1)

        pyzlc.sleep(1.0 / self.control_hz)

    def _log_action_debug(self, joint_pos: np.ndarray, gripper_cmd: float) -> None:
        now = time.time()
        if (now - self._last_action_log_ts) >= ACTION_LOG_INTERVAL_S:
            pyzlc.info(
                "Policy action: "
                f"q=[{', '.join(f'{x:.3f}' for x in joint_pos)}], "
                f"gripper={gripper_cmd:.3f}"
            )
            self._last_action_log_ts = now

        gripper_binary = 1 if gripper_cmd >= 0.5 else 0
        if self._last_gripper_binary is None:
            self._last_gripper_binary = gripper_binary
            self._gripper_toggle_window_start_ts = now
            self._gripper_toggle_count = 0
            return

        if gripper_binary != self._last_gripper_binary:
            self._gripper_toggle_count += 1
            self._last_gripper_binary = gripper_binary

        window_elapsed = now - self._gripper_toggle_window_start_ts
        if window_elapsed >= GRIPPER_TOGGLE_WARN_WINDOW_S:
            if self._gripper_toggle_count >= GRIPPER_TOGGLE_WARN_COUNT:
                pyzlc.warn(
                    "Gripper action toggling frequently: "
                    f"{self._gripper_toggle_count} toggles in "
                    f"{window_elapsed:.2f}s (threshold={GRIPPER_TOGGLE_WARN_COUNT}/"
                    f"{GRIPPER_TOGGLE_WARN_WINDOW_S:.1f}s)."
                )
            self._gripper_toggle_window_start_ts = now
            self._gripper_toggle_count = 0

    def control_end(self) -> None:
        self.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)

    def _control_task(self) -> None:
        try:
            self.control_rest()
            while self.is_running:
                self.control_step()
            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
