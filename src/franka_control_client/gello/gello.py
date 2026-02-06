from __future__ import annotations

from enum import Enum
from typing import TypedDict, Tuple, List, Optional
import pyzlc
import numpy as np

from ..core.latest_msg_subscriber import LatestMsgSubscriber
from ..core.exception import CommandError
from ..core.message import FrankaResponseCode
from ..core.remote_device import RemoteDevice

class GelloState(TypedDict):
    """
    Gello state structure.
    """

    gello_arm_state: List[float]
    gello_gripper_state: List[float]
   


class RemoteGello(RemoteDevice):
    """
    RemoteGello class for reading a Gello robot.

    This class extends the RemoteDevice class and provides
    specific functionality for interacting with a Gello robot.
    Attributes:
        robot_name (str): Name of the Gello robot.
    """

    def __init__(self, robot_name: str, enable_publishers: bool = False) -> None:
        """
        Initialize the RemoteGello instance.

        Args:
            robot_name (str): The name of the Gello robot.
        """
        super().__init__(robot_name)
        # self.default_pose: Tuple[float, ...] = (-1,)
        self.arm_state_sub = LatestMsgSubscriber(
            f"{robot_name}/gello_arm_state"
        )
        self.gripper_state_sub = LatestMsgSubscriber(
            f"{robot_name}/gello_gripper_state"
        )
        self._enable_publishers = enable_publishers
        # command publishers (optional for read-only clients)
        #for remote gello no need to pulish anything,just keep the socket
        if enable_publishers:
            self.arm_state_pub = pyzlc.Publisher(
                f"{robot_name}/gello_arm_state"
            )
            self.gripper_state_pub = pyzlc.Publisher(
                f"{robot_name}/gello_gripper_state"
            )

    def connect(self) -> None:
        """
        Connect to the Gello robot.

        Raises:
            DeviceConnectionError: If the connection fails.
        """
        super().connect()
        for _ in range(5):
            if self.arm_state_sub.last_message is not None:
                pyzlc.info("Gello arm state subscriber connected.")
                return
            pyzlc.info("Waiting for Gello arm state...")
            pyzlc.sleep(1)

    @property
    def current_state(self) -> Optional[GelloState]:
        """Return the latest Gello state."""
        return {
            "gello_arm_state": self.arm_state_sub.get_latest(),
            "gello_gripper_state": self.gripper_state_sub.get_latest(),
        }
