from __future__ import annotations

import threading
from typing import Optional

import numpy as np
import pyzlc

from .control_pair import ControlPair
from ..franka_robot.franka_panda import FrankaPanda
from ..franka_robot.panda_arm import ControlMode

class RolloutSingleFrankaControlPair(ControlPair):
    """
    Apply policy action [q0..q6, gripper_width] to a single follower Franka.
    """

    def __init__(
        self,
        follower: FrankaPanda,
        control_dt_s: float,
        gripper_speed: float,
        align_q: tuple[float, ...],
    ) -> None:
        super().__init__()
        self.follower = follower
        self.control_dt_s = float(control_dt_s)
        self.gripper_speed = float(gripper_speed)
        self._align_q = tuple(float(v) for v in align_q)

        self._action_lock = threading.Lock()
        self._latest_action: Optional[np.ndarray] = None

    def update_action(self, action: np.ndarray) -> None:
        action_arr = np.asarray(action, dtype=np.float64).reshape(-1)
        if action_arr.size != 8:
            pyzlc.error(f"Invalid action size: {action_arr.size}, expected 8")
            return
        with self._action_lock:
            self._latest_action = action_arr

    def _get_latest_action(self) -> Optional[np.ndarray]:
        with self._action_lock:
            if self._latest_action is None:
                return None
            return self._latest_action.copy()

    def stop_control_pair(self) -> None:
        if not self.is_running:
            return
        self.is_running = False
        if self.control_task_thread is not None:
            self.control_task_thread.join()
            self.control_task_thread = None

    def control_rest(self) -> None:
        self.follower.panda_arm.set_franka_arm_control_mode(
            ControlMode.HybridJointImpedance
        )

    def control_step(self) -> None:
        action = self._get_latest_action()
        if action is None:
            pyzlc.sleep(self.control_dt_s)
            return

        joint_cmd = action[:7]
        gripper_width = float(action[7])

        self.follower.panda_arm.send_joint_position_command(joint_cmd)

        grip_state = self.follower.panda_gripper.current_state
        max_width = float(grip_state["max_width"]) if grip_state is not None else 0.0
        if max_width > 0.0:
            gripper_width = float(np.clip(gripper_width, 0.0, max_width))

        self.follower.panda_gripper.send_gripper_command(
            width=gripper_width,
            speed=self.gripper_speed,
        )

        pyzlc.sleep(self.control_dt_s)

    def control_end(self) -> None:
        self.follower.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)
        self.follower.panda_arm.move_franka_arm_to_joint_position(self._align_q)

        self.follower.panda_gripper.send_gripper_command(
                width=0.07,
                speed=self.gripper_speed,
            )
