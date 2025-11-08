from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple

import numpy as np

from ..core.async_loop_thread import CommandPublisher, LatestMsgSubscriber
from ..core.exception import CommandError
from ..core.message import BinaryMsg, MsgID, RequestResultID
from ..core.remote_device import RemoteDevice
from ..core.utils import get_local_ip_in_same_subnet


class FrankaArmServiceID(MsgID):
    """Request IDs for Franka arm commands."""

    GET_FRANKA_ARM_STATE = 0
    GET_FRANKA_ARM_CONTROL_MODE = 1
    SET_FRANKA_ARM_CONTROL_MODE = 2
    GET_FRANKA_ARM_STATE_PUB_PORT = 3
    MOVE_FRANKA_ARM_TO_JOINT_POSITION = 4
    MOVE_FRANKA_ARM_TO_CARTESIAN_POSITION = 5


class FrankaArmTopicID(MsgID):
    """Response IDs for Franka arm commands."""

    # Publishers (robot → client)
    FRANKA_ARM_STATE_PUB = 100  # Publish the current robot arm state
    FRANKA_GRIPPER_STATE_PUB = 101  # Publish the current robot gripper state

    # Commands (client → robot)
    JOINT_POSITION_CMD = 110  # Command robot joints to reach target positions
    JOINT_VELOCITY_CMD = 111  # Command robot joints with velocity targets
    CARTESIAN_POSE_CMD = 112  # Command robot end-effector to a Cartesian pose
    CARTESIAN_VELOCITY_CMD = (
        113  # Command robot end-effector with Cartesian velocities
    )
    JOINT_TORQUE_CMD = 114  # Command robot joints torques


class ControlMode(IntEnum):
    """Control modes supported by the robot."""

    IDLE = 0  # Robot idle / no control command
    JOINT_POSITION = 1  # Position control in joint space
    JOINT_VELOCITY = 2  # Velocity control in joint space
    CARTESIAN_POSE = 3  # Pose control in Cartesian space
    CARTESIAN_VELOCITY = 4  # Velocity control in Cartesian space
    JOINT_TORQUE = 5  # Joint torque control mode
    GRAVITY_COMP = 6  # Gravity compensation mode


@dataclass
class FrankaArmState(BinaryMsg):
    """
    Represents the 636-byte Robot Arm State message.

    Format: !I 16d16d 7d7d7d7d7d 6d6d  (big-endian)
      timestamp_ms          (uint32)
      O_T_EE                (16 x float64)
      O_T_EE_d              (16 x float64)
      q                     (7 x float64)
      q_d                   (7 x float64)
      dq                    (7 x float64)
      dq_d                  (7 x float64)
      tau_ext_hat_filtered  (7 x float64)
      O_F_ext_hat_K         (6 x float64)
      K_F_ext_hat_K         (6 x float64)
    """

    timestamp_ms: int
    O_T_EE: tuple[float, ...]
    O_T_EE_d: tuple[float, ...]
    q: tuple[float, ...]
    q_d: tuple[float, ...]
    dq: tuple[float, ...]
    dq_d: tuple[float, ...]
    tau_ext_hat_filtered: tuple[float, ...]
    O_F_ext_hat_K: tuple[float, ...]
    K_F_ext_hat_K: tuple[float, ...]

    _STRUCT = struct.Struct("!I 16d16d 7d7d7d7d7d 6d6d")
    SIZE = _STRUCT.size  # 636 bytes

    @classmethod
    def from_bytes(cls, data: bytes) -> FrankaArmState:
        """Unpack bytes into a FrankaArmState."""
        if len(data) < cls.SIZE:
            raise ValueError(
                f"Data too short: expected {cls.SIZE} bytes, got {len(data)}"
            )

        unpacked = cls._STRUCT.unpack_from(data)
        timestamp_ms = unpacked[0]
        offset = 1
        return cls(
            timestamp_ms=timestamp_ms,
            O_T_EE=tuple(unpacked[offset : offset + 16]),
            O_T_EE_d=tuple(unpacked[offset + 16 : offset + 32]),
            q=tuple(unpacked[offset + 32 : offset + 39]),
            q_d=tuple(unpacked[offset + 39 : offset + 46]),
            dq=tuple(unpacked[offset + 46 : offset + 53]),
            dq_d=tuple(unpacked[offset + 53 : offset + 60]),
            tau_ext_hat_filtered=tuple(unpacked[offset + 60 : offset + 67]),
            O_F_ext_hat_K=tuple(unpacked[offset + 67 : offset + 73]),
            K_F_ext_hat_K=tuple(unpacked[offset + 73 : offset + 79]),
        )

    def __repr__(self):
        return (
            f"RobotArmState(ts={self.timestamp_ms}ms, "
            f"q={tuple(round(v, 3) for v in self.q)}, "
            f"dq={tuple(round(v, 3) for v in self.dq)})"
        )


class RemoteFranka(RemoteDevice):
    """
    RemoteFranka class for controlling a Franka robot.

    This class extends the RemoteDevice class and provides
    specific functionality for interacting with a Franka robot.
    Attributes:
        device_addr (str): Address of the Franka robot.
        device_port (int): Port for communication with the Franka robot.
    """

    def __init__(self, device_addr: str, device_port: int) -> None:
        """
        Initialize the RemoteFranka instance.

        Args:
            device_addr (str): The IP address of the Franka robot.
            device_port (int): The port number for communication with the Franka robot.
        """
        super().__init__(device_addr, device_port)
        self.default_pose: Tuple[float, ...] = (-1,)
        local_ip = get_local_ip_in_same_subnet(device_addr)
        if local_ip is None:
            raise ConnectionError(
                f"No local interface found in the same subnet as {device_addr}"
            )
        self.command_publisher = CommandPublisher(local_ip)
        self.arm_state_sub = LatestMsgSubscriber(
            f"tcp://{device_addr}:{self.get_franka_arm_state_pub_port()}"
        )

    def get_franka_arm_state(self) -> FrankaArmState:
        """Return a single state sample"""
        _, payload = self.request(FrankaArmServiceID.GET_FRANKA_ARM_STATE, b"")
        if payload is None:
            raise CommandError("Failed to get Franka arm state")
        return FrankaArmState.from_bytes(payload)

    def get_franka_arm_state_pub_port(self) -> Optional[int]:
        """Return the latest published state sample, if any."""
        _, payload = self.request(
            FrankaArmServiceID.GET_FRANKA_ARM_STATE_PUB_PORT, b""
        )
        if payload is None:
            raise CommandError("Failed to get Franka arm state pub port")
        (port,) = struct.unpack("!H", payload[:2])
        return port

    def get_franka_arm_control_mode(self) -> ControlMode:
        """Return the currently active control mode."""
        header, payload = self.request(
            FrankaArmServiceID.GET_FRANKA_ARM_CONTROL_MODE, b""
        )
        if header is None:
            raise CommandError("Failed to get Franka arm control mode")
        if header.message_id != RequestResultID.SUCCESS:
            raise CommandError(
                f"GET_FRANKA_ARM_CONTROL_MODE failed (status={header.message_id})"
            )
        (mode_value,) = struct.unpack("!B", payload[:1])
        return ControlMode(mode_value)

    def set_franka_arm_control_mode(self, mode: ControlMode) -> None:
        """
        Switch the Franka arm into the desired control mode.

        Args:
            mode (ControlMode): Target control mode.
            controller_ip (str, optional): IP of the controller (for subscriber-based control).
            controller_port (int, optional): Port of the controller.
        Raises:
            CommandError: If mode switch fails or arguments are invalid.
        """
        header, payload = self.request(
            FrankaArmServiceID.SET_FRANKA_ARM_CONTROL_MODE,
            bytes([mode.value]) + self.command_publisher.url.encode(),
        )

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

        try:
            payload = struct.pack("!7d", *joint_positions)
        except struct.error as exc:
            raise CommandError(f"Failed to pack joint position data: {exc}")
        header, _ = self.request(
            FrankaArmServiceID.MOVE_FRANKA_ARM_TO_JOINT_POSITION, payload
        )

        if header is None or header.message_id != RequestResultID.SUCCESS:
            raise CommandError(
                f"MOVE_FRANKA_ARM_TO_JOINT_POSITION failed (status={getattr(header, 'message_id', None)})"
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

        try:
            payload = struct.pack("!16d", *pose_matrix)
        except struct.error as exc:
            raise CommandError(f"Failed to pack pose data: {exc}")

        header, _ = self.request(
            FrankaArmServiceID.MOVE_FRANKA_ARM_TO_CARTESIAN_POSITION, payload
        )

        if header is None or header.message_id != RequestResultID.SUCCESS:
            raise CommandError(
                f"MOVE_FRANKA_ARM_TO_CARTESIAN_POSITION failed (status={getattr(header, 'message_id', None)})"
            )

    def send_joint_position_command(self, joint_positions) -> None:
        """
        Send a joint position command to the Franka arm.
        Accepts tuple, list, numpy array, or torch tensor (no type checking).
        """
        arr = np.asarray(joint_positions, dtype=np.float64).reshape(-1)
        if arr.size != 7:
            raise ValueError(f"Expected 7 joint angles, got {arr.size}")
        self.command_publisher.send_command(
            FrankaArmTopicID.JOINT_POSITION_CMD, struct.pack("!7d", *arr)
        )

    def send_cartesian_pose_command(self, pose) -> None:
        """
        Send a Cartesian pose command to the Franka arm.

        Args:
            pose (tuple of 7 floats): translation (x, y, z) and orientation (quaternion).
        Raises:
            CommandError: If packing or command execution fails.
        """
        arr = np.asarray(pose, dtype=np.float64).reshape(-1)
        if arr.size != 7:
            raise ValueError(f"Expected 7 pose values, got {arr.size}")
        self.command_publisher.send_command(
            FrankaArmTopicID.CARTESIAN_POSE_CMD,
            struct.pack("!7d", *arr),
        )

    def send_joint_velocity_command(self, joint_velocities) -> None:
        """
        Send a joint velocity command to the Franka arm.
        Accepts tuple, list, numpy array, or torch tensor (no type checking).
        """
        arr = np.asarray(joint_velocities, dtype=np.float64).reshape(-1)
        if arr.size != 7:
            raise ValueError(f"Expected 7 joint velocities, got {arr.size}")
        self.command_publisher.send_command(
            FrankaArmTopicID.JOINT_VELOCITY_CMD, struct.pack("!7d", *arr)
        )

    def send_cartesian_velocity_command(self, cartesian_velocities) -> None:
        """
        Send a Cartesian velocity command to the Franka arm.

        Args:
            cartesian_velocities (tuple of 6 floats): Target Cartesian velocities (vx, vy, vz, wx, wy, wz).
        Raises:
            CommandError: If packing or command execution fails.
        """
        arr = np.asarray(cartesian_velocities, dtype=np.float64).reshape(-1)
        if arr.size != 6:
            raise ValueError(
                f"Expected 6 Cartesian velocities, got {arr.size}"
            )
        self.command_publisher.send_command(
            FrankaArmTopicID.CARTESIAN_VELOCITY_CMD, struct.pack("!6d", *arr)
        )

    def send_joint_torque_command(self, joint_torques) -> None:
        """
        Send a joint torque command to the Franka arm.
        Accepts tuple, list, numpy array, or torch tensor (no type checking).
        """
        arr = np.asarray(joint_torques, dtype=np.float64).reshape(-1)
        if arr.size != 7:
            raise ValueError(f"Expected 7 joint torques, got {arr.size}")
        self.command_publisher.send_command(
            FrankaArmTopicID.JOINT_TORQUE_CMD, struct.pack("!7d", *arr)
        )
