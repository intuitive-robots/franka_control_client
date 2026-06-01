from __future__ import annotations

import threading
import time
from typing import Optional, Union

import numpy as np
import pyzlc

from .control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode, RemotePandaArm
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper

DEFAULT_CONTROL_HZ: float = 10.0
GRIPPER_DEADBAND: float = 1e-3
GRIPPER_SPEED = 0.7
GRIPPER_FORCE= 0.3

class CartesianPolicyPandaControlPair(ControlPair):
    """
    Apply absolute Cartesian policy actions to a Panda arm with Robotique gripper.

    Action semantics: 
      [x, y, z, rot_x, rot_y, rot_z, rot_w, gripper]
    """

    def __init__(
        self,
        panda_arm: RemotePandaArm,
        gripper: Union[RemotePandaGripper, RemoteRobotiqGripper],
        control_hz: float = DEFAULT_CONTROL_HZ,
    ) -> None:
        super().__init__()
        self.panda_arm = panda_arm
        self.gripper = gripper
        self.control_hz = float(control_hz)
        self._action_lock = threading.Lock() #only one of the update_action and control_step visit latest_action at the same time 
        self._latest_action: Optional[np.ndarray] = None