from __future__ import annotations

from typing import TypedDict, Optional
import pyzlc

from ..core.remote_device import RemoteDevice
from ..core.latest_msg_subscriber import LatestMsgSubscriber


class RobotiqGripperStateMsg(TypedDict, total=True):
    commanded_position: float
    commanded_speed: float
    commanded_force: float
    position: float
    current: float
    raw_commanded_position: int
    raw_position: int


class RobotiqGraspCommand(TypedDict, total=True):
    position: float
    speed: float
    force: float
    blocking: bool


class RemoteRobotiqGripper(RemoteDevice):
    """Remote client for a Robotiq gripper device."""

    def __init__(self, device_name: str, enable_publishers: bool = True) -> None:
        super().__init__(device_name)
        self._enable_publishers = enable_publishers
        if enable_publishers:
            self.command_publisher = pyzlc.Publisher(
                f"{self._name}/robotiq_gripper_command",
            )
        else:
            self.command_publisher = None
        self.state_subscriber = LatestMsgSubscriber(
            f"{self._name}/robotiq_gripper_state"
        )

    @property
    def current_state(self) -> Optional[RobotiqGripperStateMsg]:
        """Return the latest Robotiq gripper state."""
        return self.state_subscriber.get_latest()

    def send_grasp_command(
        self,
        position: float,
        speed: float = 0.1,
        force: float = 0.1,
        blocking: bool = False,
    ) -> None:
        """
        Send a grasp command.

        Args:
            position (float): Desired position, scaled by config scale_alpha/scale_beta.
            speed (float): Desired speed, scaled by config scale_alpha/scale_beta.
            force (float): Desired force, scaled by config scale_alpha/scale_beta.
            blocking (bool): Wait for completion if true.
        """
        if not self._enable_publishers:
            raise RuntimeError(
                "Publishers disabled for this RemoteRobotiqGripper instance."
            )
        self.command_publisher.publish(
            RobotiqGraspCommand(
                position=position,
                speed=speed,
                force=force,
                blocking=blocking,
            )
        
        )
        # print("Published grasp command", position, speed, force, blocking)

    def open(self, speed: float = 0.5, force: float = 0.1) -> None:
        """Open the gripper (position=0.0)."""
        self.send_grasp_command(
            position=0.0, speed=speed, force=force, blocking=False
        )

    def close(self, speed: float = 0.5, force: float = 0.1) -> None:
        """Close the gripper (position=1.0)."""
        self.send_grasp_command(
            position=1.0, speed=speed, force=force, blocking=False
        )
