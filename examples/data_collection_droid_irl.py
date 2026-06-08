import time
import os
import sys

# Add hardware directory to path to import GelloAgent
# sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
# from hardware.gello_zlc import GelloAgent

from typing import List

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.data_collection.irl_data_collection import (
    IRLDataCollection,
)
from franka_control_client.data_collection.irl_wrapper import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
)
from franka_control_client.data_collection.irl_wrapper import (
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
    GelloDataWrapper,
)
from franka_control_client.franka_robot.franka_panda import (
    RemotePandaArm,
)

from franka_control_client.gello.gello import RemoteGello
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.control_pair.gello_panda_control_pair import (
    GelloPandControlPair,
)

if __name__ == "__main__":
    pyzlc.init(
        "data_collection",
        "192.168.1.1",
        group_name="DroidGroup",
        group_port=7730,
    )
    leader = RemoteGello("gello")
    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    control_pair = GelloPandControlPair(leader, follower)
    #camera frequency controlled individually
    #cams 25hz 0.04
    #cams 40hz 0.025
    camera_left = ImageDataWrapper(CameraDevice("zed_left", preview=False),capture_interval=0.04,hw_name="zed_left")
    camera_right = ImageDataWrapper(CameraDevice("zed_right", preview=False),capture_interval=0.04,hw_name="zed_right")
    camera_wrist = ImageDataWrapper(CameraDevice("zed_wrist", preview=False),capture_interval=0.04,hw_name="zed_wrist")
    data_collectors: List[IRL_HardwareDataWrapper] = []
    data_collectors.append(camera_left)
    data_collectors.append(camera_right)
    data_collectors.append(camera_wrist) 
    data_collectors.append(GelloDataWrapper(leader))
    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))
    # name = time.strftim  e("%Y%m%d_%H%M%S", time.localtime())
    task = "pick_up_knife" #battery_100hz" #insert_blue_bird_100hz_cam_25hz
    data_collection_manager = IRLDataCollection(
        data_collectors, 
        f"/home/irl-admin/new_data_collection/{task}", 
        task, 
        fps=4,
        control_pair=control_pair
    )
    control_pair.control_reset()
    data_collection_manager.run()
    pyzlc.shutdown()
