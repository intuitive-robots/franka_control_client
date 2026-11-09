"""Keyboard -> Panda in Cartesian impedance (planar pushing, e.g. push-T).

Each control step integrates the held keys into the target pose (``KeyboardController.step``)
and sends it with the target velocity as feed-forward. The loop is paced to ``control_hz``.

Unlike the GELLO / VR pairs, nothing here touches the gripper: a pusher held in the gripper
must not be dropped when an episode starts or ends, or when the arm goes home. Open/close it
explicitly (the GUI's O/P keys).
"""

from __future__ import annotations

import time
import traceback
from typing import Optional, Sequence

import numpy as np

from ..franka_robot.panda_arm import ControlMode
from ..franka_robot.panda_robotiq import PandaRobotiq
from ..keyboard.keyboard_controller import KeyboardController
from .control_pair import ControlPair

CONTROL_HZ = 100.0
DEFAULT_POSITION = (0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0)


class KeyboardPandaControlPair(ControlPair):
    def __init__(
        self,
        leader: KeyboardController,
        follower: PandaRobotiq,
        control_hz: float = CONTROL_HZ,
        home_position: Sequence[float] = DEFAULT_POSITION,
    ) -> None:
        super().__init__()
        self.leader = leader
        self.follower = follower
        self.control_hz = float(control_hz)
        self.home_position = tuple(float(q) for q in home_position)
        self._t_last: Optional[float] = None
        self._t_next = 0.0

    def _anchor_to_measured(self) -> None:
        state = self.follower.panda_arm.current_state
        if state is None:
            raise RuntimeError("No arm state: cannot anchor the keyboard target.")
        self.leader.anchor(state["EE_pos"], state["EE_quat"])

    def control_reset(self) -> None:
        """Go home (gripper untouched), then re-anchor the target there."""
        self.follower.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)
        self.follower.panda_arm.move_franka_arm_to_joint_position(self.home_position)
        self._anchor_to_measured()

    def control_step(self) -> None:
        now = time.perf_counter()
        dt = 1.0 / self.control_hz if self._t_last is None else min(now - self._t_last, 0.1)
        self._t_last = now
        state = self.follower.panda_arm.current_state
        measured = state["EE_pos"] if state is not None else None
        pos, quat, vel = self.leader.step(dt, measured)
        self.follower.panda_arm.send_cartesian_pose_command(pos=pos, rot=quat, pos_vel=vel)
        self._t_next = max(self._t_next + 1.0 / self.control_hz, time.perf_counter() - 1.0 / self.control_hz)
        delay = self._t_next - time.perf_counter()
        if delay > 0:
            time.sleep(delay)

    def control_end(self) -> None:
        self.leader.release_all()
        self.follower.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)

    def _control_task(self) -> None:
        try:
            self._anchor_to_measured()  # start exactly where the arm is
            self.follower.panda_arm.set_franka_arm_control_mode(ControlMode.CartesianImpedance)
            self._t_last, self._t_next = None, time.perf_counter()
            while self.is_running:
                self.control_step()
            self.control_end()
        except Exception as e:
            print(f"Keyboard control task encountered an error: {e}")
            traceback.print_exc()
            try:
                self.follower.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)
            except Exception:
                pass

    def open_gripper(self, speed: float = 0.7, force: float = 0.3) -> None:
        self.leader.gripper = 0.0
        self.follower.robotiq_gripper.send_grasp_command(position=0.0, speed=speed, force=force, blocking=False)

    def close_gripper(self, speed: float = 0.7, force: float = 0.3) -> None:
        self.leader.gripper = 1.0
        self.follower.robotiq_gripper.send_grasp_command(position=1.0, speed=speed, force=force, blocking=False)
