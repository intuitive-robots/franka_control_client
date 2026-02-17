from __future__ import annotations

from ..core.remote_device import RemoteDevice
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper
from .panda_arm import RemotePandaArm


class PandaRobotiq(RemoteDevice):
    """
    Franka Panda arm with Robotiq gripper interface.
    """

    def __init__(
        self,
        name: str,
        panda_arm: RemotePandaArm,
        robotiq_gripper: RemoteRobotiqGripper,
    ) -> None:
        super().__init__(name)
        self.panda_arm = panda_arm
        self.robotiq_gripper = robotiq_gripper

    def connect(self) -> None:
        """
        Connect to the Franka Panda arm and Robotiq gripper.
        """
        self.panda_arm.connect()
        self.robotiq_gripper.connect()
