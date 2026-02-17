from __future__ import annotations

from ..core.remote_device import RemoteDevice
from .panda_arm import RemotePandaArm
from .panda_gripper import RemotePandaGripper


class FrankaPanda(RemoteDevice):
    """
    Franka Panda robot arm interface.
    """

    def __init__(
        self,
        name: str,
        panda_arm: RemotePandaArm,
        panda_gripper: RemotePandaGripper,
    ) -> None:
        super().__init__(name)
        self.panda_arm = panda_arm
        self.panda_gripper = panda_gripper

    def connect(self) -> None:
        """
        Connect to the Franka Panda robot arm and gripper.
        """
        self.panda_arm.connect()
        self.panda_gripper.connect()
