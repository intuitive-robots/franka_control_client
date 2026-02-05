from __future__ import annotations

from typing import TypedDict, Optional
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


class RemotePandaGripper(RemoteDevice):
    """Remote client for a gripper device."""

    def __init__(self, device_name: str) -> None:
        super().__init__(device_name)
        self.command_publisher = pyzlc.Publisher(
            f"{self._name}/franka_gripper_command",
        )
        self.state_subscriber = LatestMsgSubscriber(
            f"{self._name}/franka_gripper_state"
        )

    @property
    def current_state(self) -> Optional[GripperStateMsg]:
        """Return the latest gripper state."""
        msg = self.state_subscriber.last_message
        if msg is None:
            return None
        return GripperStateMsg(
            width=msg["width"],
            max_width=msg["max_width"],
            is_grasped=msg["is_grasped"],
            temperature=msg["temperature"],
            time=msg["time"],
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
