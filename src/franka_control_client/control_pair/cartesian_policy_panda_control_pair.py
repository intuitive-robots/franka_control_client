from __future__ import annotations

import time
import traceback
from typing import Optional
from scipy.spatial.transform import Rotation as R
import numpy as np
import pyzlc

from ..control_pair.policy_panda_control_pair import PolicyPandaControlPair
from ..franka_robot.panda_arm import ControlMode, RemotePandaArm
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper
from .action_chunking_buffer import DeltaActionChunkingBuffer

DEFAULT_CONTROL_HZ: float = 500
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


class PolicyPandaRobotiqDeltaCartesianControlPair(PolicyPandaControlPair):
    """
    Apply policy actions to a Panda arm with a gripper.

    Action semantics: [x, y, z, qx, qy, qz, qw, gripper] (cartesian pose + gripper).
    Gripper value is normalized in [0, 1]. It is scaled to device range.
    Includes velocity and acceleration limiting for safety.
    """

    def __init__(
        self,
        panda_arm: RemotePandaArm,
        gripper: RemoteRobotiqGripper,
        control_hz: float = DEFAULT_CONTROL_HZ,
    ) -> None:
        super().__init__(panda_arm, gripper, control_hz)
        # self.panda_arm = panda_arm
        # self.gripper = gripper
        # self.control_hz = float(control_hz)
        # self._action_lock = (
        #     threading.Lock()
        # )  # only one of the update_action and control_step visit latest_action at the same time
        self.action_buffer = DeltaActionChunkingBuffer(
            robot_arm=panda_arm,
            action_dt=1.0 / self.control_hz,
            chunk_size=10,
            action_dim=7,
        )

        # Velocity limiting state
        self._last_joint_pos: Optional[np.ndarray] = None
        self._last_control_time: Optional[float] = None
        self._dt = 1.0 / self.control_hz  # time delta between control steps

    def get_lastest_command(self) -> Optional[np.ndarray]:
        return self._get_latest_action_from_chunk()

    def clear_lastest_command(self) -> None:
        self.action_buffer.clear()

    # using by policy side to update the latest action, and control loop will read the latest action and execute it
    def update_action(self, action: np.ndarray) -> None:
        """Update the latest action used by the control loop."""
        arr = np.asarray(action, dtype=np.float64).reshape(-1)
        if arr.size < 8:
            raise ValueError(f"Expected action size >= 8, got {arr.size}")
        with self._action_lock:
            self._latest_action = arr

    # using by policy side to update the latest action_chunk, and control loop will read the latest action and execute it
    def update_action_chunk(self, action_chunk: np.ndarray) -> None:
        """Update the latest action chunk used by the control loop."""
        self.action_buffer.add_new_action_chunk(action_chunk)

    def _get_latest_action(self) -> Optional[np.ndarray]:
        with self._action_lock:
            if self._latest_action is None:
                return None
            return self._latest_action.copy()

    def _get_latest_action_from_chunk(self) -> Optional[np.ndarray]:
        return self.action_buffer.get_action()

    def reset_action(self) -> None:
        """Reset the latest action state when starting a new episode."""
        with self._action_lock:
            self._latest_action = None
            self.action_buffer.clear()
        self.clear_lastest_command()
        self._last_gripper_cmd = None
        self._last_gripper_binary = None
        self._gripper_toggle_count = 0
        self._gripper_toggle_window_start_ts = time.time()
        self._last_cartesian_pos = self._get_current_cartesian_pose()
        pyzlc.info("Action state reset for new episode")

    def control_step(self) -> None:
        action = self.action_buffer.apply_action()
        self.panda_arm.send_cartesian_pose_command(action[:3], action[3:7])

        # Gripper command
        gripper_cmd = float(action[-1])
        gripper_cmd = 1 if gripper_cmd >= 0.5 else 0
        action[-1] = gripper_cmd
        if (
            self._last_gripper_cmd is None
            or abs(gripper_cmd - self._last_gripper_cmd) > GRIPPER_DEADBAND
        ):
            self.gripper.send_grasp_command(
                position=gripper_cmd,
                speed=GRIPPER_SPEED,
                force=GRIPPER_FORCE,
                blocking=False,
            )
            self._last_gripper_cmd = gripper_cmd
        # End_time = time.perf_counter()
        # print(f"command took {End_time - start_time:.3f} seconds")

    def control_reset(self) -> None:
        self.panda_arm.set_franka_arm_control_mode(ControlMode.CartesianImpedance)
        current_pose = self._get_current_cartesian_pose()
        self.panda_arm.send_cartesian_pose_command(current_pose[:3], current_pose[3:])

    def control_end(self) -> None:
        self.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)

    def _control_task(self) -> None:
        try:
            self.control_reset()
            while self.is_running:
                start = time.perf_counter()
                self.control_step()
                if time.perf_counter() - start < (1.0 / self.control_hz):
                    pyzlc.sleep(
                        (1.0 / self.control_hz) - (time.perf_counter() - start)
                    )
            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
            traceback.print_exc()

    def _get_current_cartesian_pose(self) -> np.ndarray:
        current_state = self.panda_arm.current_state
        if current_state is None or "EE_pos" not in current_state:
            raise ValueError(
                "Current arm state is not available or missing end-effector position"
            )
        cartesian_pos = np.asarray(
            current_state["EE_pos"], dtype=np.float32
        ).reshape(-1)
        cartesian_rot = np.asarray(
            current_state["EE_quat"], dtype=np.float32
        ).reshape(-1)
        cartesian_pose = np.concatenate([cartesian_pos, cartesian_rot])
        if cartesian_pose.size != 7:
            pyzlc.error(
                f"Unexpected current arm state size during control init: {cartesian_pose.size}"
            )
            raise ValueError("Unexpected current arm state size")
        return cartesian_pose
