from __future__ import annotations

from enum import Enum
from typing import TypedDict, Tuple, List, Optional
import pyzlc
import numpy as np

from ..core.latest_msg_subscriber import LatestMsgSubscriber
from ..core.exception import CommandError
from ..core.message import FrankaResponseCode
from ..core.remote_device import RemoteDevice


class ControlMode(str, Enum):
    IDLE = "IDLE"
    HybridJointImpedance = "HybridJointImpedance"
    # JOINT_POSITION = "JointPosition"
    # JOINT_VELOCITY = "JointVelocity"
    # CARTESIAN_POSE = "CartesianPose"
    # CARTESIAN_VELOCITY = "CartesianVelocity"
    # JOINT_TORQUE = "JointTorque"
    # GRAVITY_COMP = "GravityComp"


class PandaArmState(TypedDict):
    """
    Franka arm state structure.
    """

    time_ms: int
    O_T_EE: List[float]
    O_T_EE_d: List[float]
    q: List[float]
    q_d: List[float]
    dq: List[float]
    dq_d: List[float]
    tau_ext_hat_filtered: List[float]
    O_F_ext_hat_K: List[float]
    K_F_ext_hat_K: List[float]


class JointPositionCommand(TypedDict):
    """
    Joint position command structure.
    """

    pos: List[float]  # 7 joint angles in radians


class CartesianPoseCommand(TypedDict):
    """
    Cartesian pose command structure.
    """

    pos: List[float]  # x, y, z and quaternion x, y, z, w


class CartesianVelocityCommand(TypedDict):
    """
    Cartesian velocity command structure.
    """

    vel: List[float]  # vx, vy, vz, wx, wy, wz


class RemotePandaArm(RemoteDevice):
    """
    RemotePandaArm class for controlling a Franka robot.

    This class extends the RemoteDevice class and provides
    specific functionality for interacting with a Franka robot.
    Attributes:
        robot_name (str): Name of the Franka robot.
    """

    def __init__(
        self, robot_name: str, enable_publishers: bool = True
    ) -> None:
        """
        Initialize the RemotePandaArm instance.

        Args:
            robot_name (str): The name of the Franka robot.
        """
        super().__init__(robot_name)
        self.default_pose: Tuple[float, ...] = (-1,)
        self.arm_state_sub = LatestMsgSubscriber(
            f"{robot_name}/franka_arm_state"
        )
        self._enable_publishers = enable_publishers
        self.joint_position_publisher = pyzlc.Publisher(
            f"{robot_name}/joint_position_command"
        )
        self.joint_velocity_publisher = pyzlc.Publisher(
            f"{robot_name}/joint_velocity_command"
        )
        self.cartesian_pose_publisher = pyzlc.Publisher(
            f"{robot_name}/cartesian_pose_command"
        )
        self.cartesian_velocity_publisher = pyzlc.Publisher(
            f"{robot_name}/cartesian_velocity_command"
        )
        self.joint_torque_publisher = pyzlc.Publisher(
            f"{robot_name}/joint_torque_command"
        )

    def connect(self) -> None:
        """
        Connect to the Franka robot.

        Raises:
            DeviceConnectionError: If the connection fails.
        """
        super().connect()
        for _ in range(5):
            if self.arm_state_sub.last_message is not None:
                pyzlc.info("Franka arm state subscriber connected.")
                return
            pyzlc.info("Waiting for Franka arm state...")
            pyzlc.sleep(1)

    @property
    def current_state(self) -> Optional[PandaArmState]:
        """Return the latest Franka arm state."""
        return self.arm_state_sub.get_latest()

    def get_franka_arm_state(self) -> PandaArmState:
        """Return a single state sample"""
        return pyzlc.call(f"{self._name}/get_franka_arm_state", pyzlc.empty)

    def get_franka_arm_control_mode(self) -> str:
        """Return the currently active control mode."""
        return pyzlc.call(
            f"{self._name}/get_franka_arm_control_mode", pyzlc.empty
        )

    def set_franka_arm_control_mode(self, mode: ControlMode) -> None:
        """Set the control mode of the Franka arm."""
        pyzlc.call(f"{self._name}/set_franka_arm_control_mode", mode.value)
        print(f"Set Franka arm control mode to {mode.value}")

    def move_franka_arm_to_joint_position(
        self, joint_positions: Tuple[float, ...]
    ) -> None:
        """
        Move the Franka arm to the specified joint position.

        Args:
            joint_positions (tuple of 7 floats): Target joint angles (radians).
        Raises:
            CommandError: If packing or response fails.
        """
        if len(joint_positions) != 7:
            raise CommandError(
                f"Expected 7 joint values, got {len(joint_positions)}"
            )
        header, _ = pyzlc.call(
            f"{self._name}/move_franka_arm_to_joint_position",
            list(joint_positions),
            10.0,
        )

        if header is None or header != FrankaResponseCode.SUCCESS.value:
            raise CommandError(
                f"MOVE_FRANKA_ARM_TO_JOINT_POSITION failed (status={header})"
            )

    def move_franka_arm_to_cartesian_position(
        self, pose_matrix: Tuple[float, ...]
    ) -> None:
        """
        Move the Franka arm to the specified Cartesian pose.

        Args:
            pose_matrix (tuple of 16 floats): 4x4 transformation matrix (row-major).
        Raises:
            CommandError: If packing or command execution fails.
        """
        if len(pose_matrix) != 16:
            raise CommandError(
                f"Expected 16 pose values, got {len(pose_matrix)}"
            )
        raise NotImplementedError

    def send_joint_position_command(self, joint_positions) -> None:
        """
        Send a joint position command to the Franka arm.
        Accepts tuple, list, numpy array, or torch tensor (no type checking).
        """
        if not self._enable_publishers:
            raise RuntimeError(
                "Publishers disabled for this RemotePandaArm instance."
            )
        arr = np.asarray(joint_positions, dtype=np.float64).reshape(-1)
        if arr.size != 7:
            raise ValueError(f"Expected 7 joint angles, got {arr.size}")
        self.joint_position_publisher.publish(
            JointPositionCommand(pos=arr.tolist())
        )

    def send_cartesian_pose_command(self, pose) -> None:
        """
        Send a Cartesian pose command to the Franka arm.

        Args:
            pose (tuple of 7 floats): translation (x, y, z) and orientation (quaternion).
        Raises:
            CommandError: If packing or command execution fails.
        """
        if not self._enable_publishers:
            raise RuntimeError(
                "Publishers disabled for this RemotePandaArm instance."
            )
        arr = np.asarray(pose, dtype=np.float64).reshape(-1)
        if arr.size != 7:
            raise ValueError(f"Expected 7 pose values, got {arr.size}")
        raise NotImplementedError

    def send_joint_velocity_command(self, joint_velocities) -> None:
        """
        Send a joint velocity command to the Franka arm.
        Accepts tuple, list, numpy array, or torch tensor (no type checking).
        """
        if not self._enable_publishers:
            raise RuntimeError(
                "Publishers disabled for this RemotePandaArm instance."
            )
        arr = np.asarray(joint_velocities, dtype=np.float64).reshape(-1)
        if arr.size != 7:
            raise ValueError(f"Expected 7 joint velocities, got {arr.size}")
        raise NotImplementedError

    def send_cartesian_velocity_command(self, cartesian_velocities) -> None:
        """
        Send a Cartesian velocity command to the Franka arm.

        Args:
            cartesian_velocities (tuple of 6 floats): Target Cartesian velocities (vx, vy, vz, wx, wy, wz).
        Raises:
            CommandError: If packing or command execution fails.
        """
        if not self._enable_publishers:
            raise RuntimeError(
                "Publishers disabled for this RemotePandaArm instance."
            )
        arr = np.asarray(cartesian_velocities, dtype=np.float64).reshape(-1)
        if arr.size != 6:
            raise ValueError(
                f"Expected 6 Cartesian velocities, got {arr.size}"
            )
        self.cartesian_velocity_publisher.publish(
            CartesianVelocityCommand(vel=arr.tolist())
        )

    def send_joint_torque_command(self, joint_torques) -> None:
        """
        Send a joint torque command to the Franka arm.
        Accepts tuple, list, numpy array, or torch tensor (no type checking).
        """
        if not self._enable_publishers:
            raise RuntimeError(
                "Publishers disabled for this RemotePandaArm instance."
            )
        arr = np.asarray(joint_torques, dtype=np.float64).reshape(-1)
        if arr.size != 7:
            raise ValueError(f"Expected 7 joint torques, got {arr.size}")
        raise NotImplementedError
