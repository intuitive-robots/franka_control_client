from __future__ import annotations

import struct
import threading
from dataclasses import dataclass
from typing import Final

import numpy as np

from ..core.async_loop_thread import CommandPublisher, LatestMsgSubscriber
from ..core.exception import CommandError, DeviceConnectionError
from ..core.remote_device import RemoteDevice, State
from ..core.utils import get_local_ip_in_same_subnet

_STATE_STRUCT: Final = struct.Struct(
    "!Idd?H"
)  # timestamp, width, max_width, grasped, temperature
_STATE_SIZE: Final = _STATE_STRUCT.size


@dataclass(frozen=True)
class GripperState:
    """State of the gripper."""

    width: float
    max_width: float
    grasped: bool
    temperature: int

    _STRUCT = struct.Struct("!Idd1H")
    SIZE = _STRUCT.size

    @classmethod
    def from_bytes(cls, data: bytes) -> GripperState:
        if len(data) != cls.SIZE:
            raise CommandError("State payload size mismatch")
        width, max_width, grasped, temperature = cls._STRUCT.unpack(data)
        return cls(
            width=width,
            max_width=max_width,
            grasped=grasped,
            temperature=temperature,
        )


class RemoteFrankaGripper(RemoteDevice):
    """Remote client for a gripper device."""

    def __init__(self, device_addr: str, device_port: int) -> None:
        super().__init__(device_addr, device_port)
        local_ip = get_local_ip_in_same_subnet(device_addr)
        if local_ip is None:
            raise DeviceConnectionError(
                f"No local interface found in the same subnet as {device_addr}"
            )
        self.command_publisher = CommandPublisher(local_ip)
        self.state_subscriber = LatestMsgSubscriber()

    # def get_state(self) -> GripperState:
    #     """Return a single state sample"""
    #     self._send(MsgID.GET_STATE_REQ, b"")
    #     payload = self._recv_expect(MsgID.GET_STATE_RESP)
    #     return self._decode_state(payload)

    # def start_state_listener(
    #     self,
    #     *,
    #     port: int | None = None,
    #     buffer_size: int = 2048,
    #     topic: bytes = b"franka_gripper",
    # ) -> None:
    #     return super().start_state_listener(
    #         port=port,
    #         buffer_size=buffer_size,
    #         topic=topic,
    #     )

    # @staticmethod
    # def _decode_state(buf: bytes) -> GripperState:
    #     if len(buf) != _STATE_SIZE:
    #         raise CommandError("State payload size mismatch")
    #     timestamp, width, max_width, grasped, temperature = (
    #         _STATE_STRUCT.unpack(buf)
    #     )
    #     return GripperState(
    #         timestamp_ms=timestamp,
    #         width=width,
    #         max_width=max_width,
    #         grasped=grasped,
    #         temperature=temperature,
    #     )

    def open(self) -> None:
        """Open the gripper."""
        command = np.array([0.8, 0.1, 60], dtype=np.float64)
        threading.Thread(
            target=self.request,
            args=("SET_FRANKA_GRIPPER_WIDTH", struct.pack("!3d", *command)),
            daemon=True,
        ).start()
        # self.request("SET_FRANKA_GRIPPER_WIDTH", struct.pack("!3d", *command))

    def close(self) -> None:
        """Close the gripper."""
        command = np.array([0.0, 0.1, 60], dtype=np.float64)
        threading.Thread(
            target=self.request,
            args=("SET_FRANKA_GRIPPER_WIDTH", struct.pack("!3d", *command)),
            daemon=True,
        ).start()
        # self.request("SET_FRANKA_GRIPPER_WIDTH", struct.pack("!3d", *command))

    def start_gripper_control(self) -> None:
        """Start the gripper control communication."""
        self.request(
            "START_FRANKA_GRIPPER_CONTROL",
            self.command_publisher.url.encode("utf-8"),
        )

    def send_gripper_command(
        self, width: float, speed: float = 0.01, force: float = 1
    ) -> None:
        """Send a gripper command."""
        command = np.array([width, speed, force], dtype=np.float64)
        self.command_publisher.send_command(struct.pack("!3d", *command))
