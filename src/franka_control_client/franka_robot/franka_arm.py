from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple

from ..core.exception import CommandError
from ..core.message import BinaryMsg, MsgID, RequestResultID
from ..core.remote_device import RemoteDevice


class FrankaArmRequestID(MsgID):
    GET_FRANKA_ARM_STATE = 0
    GET_FRANKA_ARM_CONTROL_MODE = 1
    SET_FRANKA_ARM_CONTROL_MODE = 2
    GET_FRANKA_ARM_STATE_PUB_PORT = 3
    MOVE_FRANKA_ARM_TO_JOINT_POSITION = 4
    GET_FRANKA_GRIPPER_STATE = 5


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

    def to_bytes(self) -> bytes:
        """Pack the structure into bytes."""
        return self._STRUCT.pack(
            self.timestamp_ms,
            *self.O_T_EE,
            *self.O_T_EE_d,
            *self.q,
            *self.q_d,
            *self.dq,
            *self.dq_d,
            *self.tau_ext_hat_filtered,
            *self.O_F_ext_hat_K,
            *self.K_F_ext_hat_K,
        )

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
        # self.arm_state_sub = LatestMsgSubscriber(
        #     f"tcp://{device_addr}:{self.get_franka_arm_state_pub_port()}"
        # )

    def get_franka_arm_state(self) -> FrankaArmState:
        """Return a single state sample"""
        _, payload = self.request(FrankaArmRequestID.GET_FRANKA_ARM_STATE, b"")
        if payload is None:
            raise CommandError("Failed to get Franka arm state")
        return FrankaArmState.from_bytes(payload)

    def get_franka_arm_state_pub_port(self) -> Optional[int]:
        """Return the latest published state sample, if any."""
        _, payload = self.request(
            FrankaArmRequestID.GET_FRANKA_ARM_STATE_PUB_PORT, b""
        )
        if payload is None:
            raise CommandError("Failed to get Franka arm state pub port")
        (port,) = struct.unpack("!H", payload[:2])
        return port

    def get_franka_arm_control_mode(self) -> ControlMode:
        """Return the currently active control mode."""
        header, payload = self.request(
            FrankaArmRequestID.GET_FRANKA_ARM_CONTROL_MODE, b""
        )
        if header is None:
            raise CommandError("Failed to get Franka arm control mode")
        if header.message_id != RequestResultID.SUCCESS:
            raise CommandError(
                f"GET_FRANKA_ARM_CONTROL_MODE failed (status={header.message_id})"
            )
        (mode_value,) = struct.unpack("!B", payload[:1])
        return ControlMode(mode_value)

    # def set_control_mode(
    #     self,
    #     mode: ControlMode,
    #     *,
    #     controller_ip: Optional[str] = None,
    #     controller_port: Optional[int] = None,
    #     subscribe_server: bool = False,
    # ) -> None:
    #     """
    #     Switch the robot into *mode*.

    #     For any mode **other than** :pyattr:`ControlMode.HUMAN_MODE` you *must*
    #     provide ``controller_ip`` and ``controller_port`` - these parameters
    #     tell the robot where your *SUB* socket lives.  If ``subscribe_server``
    #     is *True* we start a background thread that subscribes to
    #     state updates.
    #     """
    #     payload = bytearray([mode.value])

    #     if mode != ControlMode.HUMAN_MODE:
    #         if controller_ip is None or controller_port is None:
    #             raise ValueError("Controller_ip and Controller_port required")

    #         try:
    #             payload += socket.inet_aton(controller_ip)
    #         except OSError:
    #             raise ValueError(f"Invalid IPv4 address: {controller_ip}")

    #         payload += struct.pack("!H", controller_port)

    #     _, status = self.request(
    #         FrankaArmRequestID.SET_FRANKA_ARM_CONTROL_MODE, bytes(payload)
    #     )

    # def move_to_default_pose(self) -> None:
    #     """
    #     Move the robot to the previously stored default pose.

    #     Raises:
    #         CommandError: If no default pose has been set or movement fails.
    #     """
    #     if self.default_pose[0] == -1:
    #         raise CommandError("Default pose not set")
    #     self.move_to_position(self.default_pose)

    # def move_to_position(self, o_t_ee_d: Tuple[float, ...]) -> None:
    #     """
    #     Move the robot to the specified end-effector position.

    #     Args:
    #         o_t_ee_d: Target end-effector pose as 4x4 transformation matrix.

    #     Raises:
    #         CommandError: If position data is invalid or movement command fails.
    #     """
    #     if len(o_t_ee_d) != 16:
    #         raise CommandError(f"Expected 16 pose values, got {len(o_t_ee_d)}")
    #     try:
    #         payload = struct.pack("16d", *o_t_ee_d)
    #         header, _ = self.request(
    #             FrankaArmRequestID.MOVE_FRANKA_ARM_TO_JOINT_POSITION, payload
    #         )
    #         if header is None:
    #             raise CommandError("Failed to receive response")
    #         if header.message_id != RequestResultID.SUCCESS:
    #             raise CommandError(
    #                 f"MOVE_TO_POSITION failed (status={header.message_id})"
    #             )
    #     except struct.error as exc:
    #         raise CommandError(f"Failed to pack position data: {exc}")
