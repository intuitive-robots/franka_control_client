import time
import os
import sys

# Add hardware directory to path to import GelloAgent
# sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
# from hardware.gello_zlc import GelloAgent

import time
from typing import List

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.data_collection.lerobot_data_collection import (
    LeRobotDataCollection,
)
from franka_control_client.data_collection.wrapper import (
    HardwareDataWrapper,
    ImageDataWrapper,
)
from franka_control_client.data_collection.wrapper import (
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
    GelloDataWrapper
)
from franka_control_client.franka_robot.franka_panda import (
    RemotePandaArm,
)

from franka_control_client.gello.gello import(
    RemoteGello
)
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper
)
from franka_control_client.franka_robot.panda_robotiq import (
    PandaRobotiq
)
from franka_control_client.control_pair.gello_panda_control_pair import (
    GelloPandControlPair,
)

if __name__ == "__main__":
    pyzlc.init(
        "data_collection",
        "192.168.0.117",
        group_name="DroidGroup",
        group_port=7730
    )
    leader = RemoteGello("gello")
    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    control_pair = GelloPandControlPair(leader, follower)
    camera_left = ImageDataWrapper(CameraDevice("zed_left", preview=False))
    # camera_right = ImageDataWrapper(CameraDevice("zed_right", preview=False))
    # camera_wrist = ImageDataWrapper(CameraDevice("zed_wrist", preview=False))
    data_collectors: List[HardwareDataWrapper] = []
    data_collectors.append(camera_left)
    # data_collectors.append(camera_right)
    # data_collectors.append(camera_wrist)
    data_collectors.append(GelloDataWrapper(leader))
    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))
    name = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    data_collection_manager = LeRobotDataCollection(
        data_collectors, f"/home/irl-admin/Franka_Control_Feb/dataset/{name}", task="pepper"
    )
    data_collection_manager.register_start_collecting_event(
        control_pair.start_control_pair
    )
    data_collection_manager.register_stop_collecting_event(
        control_pair.stop_control_pair
    )
    data_collection_manager.run()
    pyzlc.shutdown()
