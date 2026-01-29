from __future__ import annotations

from typing import TypedDict
import pyzlc

from ..core.remote_device import RemoteDevice
from ..core.latest_msg_subscriber import LatestMsgSubscriber


class GripperStateMsg(TypedDict, total=True):
    width: float
    max_width: float
    is_grasped: bool
    temperature: int
    time: int


class GraspCommand(TypedDict, total=True):
    width: float
    speed: float


class RemoteFrankaGripper(RemoteDevice):
    """Remote client for a gripper device."""

    def __init__(self, device_name: str) -> None:
        super().__init__(device_name)
        self.command_publisher = pyzlc.Publisher(
            f"{self._name}/franka_gripper_command",
        )
        self.state_subscriber = LatestMsgSubscriber(
            f"{self._name}/franka_gripper_state"
        )

    def open(self, speed: float = 0.1) -> None:
        """Open the gripper."""
        self.send_gripper_command(width=0.8, speed=speed)

    def close(self, speed: float = 0.1) -> None:
        """Close the gripper."""
        self.send_gripper_command(width=0.0, speed=speed)

    def send_gripper_command(self, width: float, speed: float = 0.01) -> None:
        """Send a gripper command."""
        self.command_publisher.publish(GraspCommand(width=width, speed=speed))
