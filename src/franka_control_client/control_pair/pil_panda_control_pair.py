from __future__ import annotations

import threading
import time
import traceback
from typing import Optional, Union

import numpy as np
import enum
import pyzlc

from franka_control_client.control_pair.policy_panda_control_pair import (
    PolicyPandaControlPair,
)
from ..data_collection.pil_irl_vr_data_collection import PILIRLDataCollection

from ..franka_robot.panda_robotiq import PandaRobotiq

from .mq3_panda_control_pair import MQ3PandaControlPair

from ..vr.meta_quest3 import MQ3Controller

from .control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode, RemotePandaArm
from ..franka_robot.panda_gripper import RemotePandaGripper
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper
from .cartesian_policy_panda_control_pair import (
    PolicyPandaRobotiqDeltaCartesianControlPair,
)

DEFAULT_CONTROL_HZ: float = 1000
GRIPPER_DEADBAND: float = 1e-3
GRIPPER_SPEED = 0.7
GRIPPER_FORCE = 0.3
ACTION_LOG_INTERVAL_S: float = 0.5
GRIPPER_TOGGLE_WARN_WINDOW_S: float = 3.0
GRIPPER_TOGGLE_WARN_COUNT: int = 6
DEFAULT_POSITION = (0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0)

# Calculate velocity limits using the standard approach from training
VELOCITY_LIMITS = np.array([[-4 * np.pi / 2, 4 * np.pi / 2]] * 7).T / 32
VELOCITY_LIMITS_NORM = np.linalg.norm(VELOCITY_LIMITS)


class PILMode(enum.Enum):
    POLICY = "policy"
    INTERRUPT = "interrupt"
    REPLAY = "replay"


class PILPandaControlPair(PolicyPandaRobotiqDeltaCartesianControlPair):
    """
    Apply policy actions to a Panda arm with a gripper.

    Action semantics: [q0..q6, gripper] (joint positions + gripper).
    Gripper value is normalized in [0, 1]. It is scaled to device range.
    Includes velocity and acceleration limiting for safety.
    """

    def __init__(
        self,
        panda_arm: RemotePandaArm,
        gripper: Union[RemotePandaGripper, RemoteRobotiqGripper],
        mq3_controller: MQ3PandaControlPair,
        control_hz: float = DEFAULT_CONTROL_HZ,
    ) -> None:
        super().__init__(panda_arm, gripper, control_hz)
        self.policy_pair = self
        self.panda_arm = panda_arm
        self.gripper = gripper
        self.mq3_controller = mq3_controller

        self.interrupt_control_pair = MQ3PandaControlPair(
            self.mq3_controller, PandaRobotiq("MQ3Panda", panda_arm, gripper)
        )
        self.current_control_pair: ControlPair = self.policy_pair
        self.current_state = PILMode.POLICY
        self.control_pair_lock = threading.Lock()

        self.mq3_controller.mq3.register_trigger_press_event(
            "hand_trigger", "right", self.switch_to_interrupt_control
        )
        self.mq3_controller.mq3.register_trigger_release_event(
            "hand_trigger", "right", self.switch_to_policy_control
        )

        self.data_manager: Optional[PILIRLDataCollection] = None

    def register_history(self, data_manager: PILIRLDataCollection) -> None:
        self.data_manager = data_manager

    def switch_to_policy_control(self) -> None:
        self.current_state = PILMode.POLICY
        with self.control_pair_lock:
            self.current_control_pair = self.policy_pair
        self.clear_lastest_command()  # Clear any residual commands when switching back to policy control
        # print("Switched to policy control")

    def switch_to_interrupt_control(self) -> None:
        self.current_state = PILMode.INTERRUPT
        with self.control_pair_lock:
            self.current_control_pair = self.interrupt_control_pair
        self.clear_lastest_command()  # Clear any residual commands when switching to interrupt control
        # print("Switched to interrupt control")

    def _control_task(self) -> None:
        try:
            self.control_reset()
            while self.is_running:
                start = time.perf_counter()
                self._replay()
                with self.control_pair_lock:
                    self.current_control_pair.control_step()
                # end_time = time.perf_counter()
                # print(f"Control step took {end_time - start:.3f} seconds")
                if time.perf_counter() - start < (1.0 / self.control_hz):
                    pyzlc.sleep(
                        (1.0 / self.control_hz) - (time.perf_counter() - start)
                    )
            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
            traceback.print_exc()

    def control_reset(self) -> None:
        self.mq3_controller.reset()
        super().control_reset()
        self.interrupt_control_pair.control_reset()

    def control_end(self) -> None:
        super().control_end()
        self.interrupt_control_pair.control_end()

    def reset_action(self):
        super().reset_action()

    def _replay(self):
        previous_state = self.current_state
        while True:
            data = (
                self.mq3_controller.mq3.get_controller_data()
            )  # Ensure we have the latest data from MQ3, even if not used in policy control
            if data is None or not data["A"] or self.data_manager is None:
                self.data_manager.set_pause(False)
                break
            self.data_manager.set_pause(True)
            self.current_state = PILMode.REPLAY
            result = self.data_manager.pop()
            # Pop the oldest data point to maintain sync with control steps
            if result is None:
                continue
            pos, quat, gripper_width = result
            self.panda_arm.send_cartesian_pose_command(pos=pos, rot=quat)
            self.gripper.send_grasp_command(
                position=float(gripper_width),
                speed=GRIPPER_SPEED,
                force=GRIPPER_FORCE,
                blocking=False,
            )
            self.reset_action()
            pyzlc.sleep(0.1)
        self.current_state = previous_state
