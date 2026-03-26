import time
import os
import sys

# Add hardware directory to path to import GelloAgent
# sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
# from hardware.gello_zlc import GelloAgent

from typing import List

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.data_collection.irl_vr_data_collection import (
    IRLDataCollection,
)
from franka_control_client.data_collection.irl_wrapper import (
    IRLDataWrapper,
    ImageDataWrapper,
)
from franka_control_client.data_collection.irl_wrapper import (
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
    MQ3DataWrapper,
)
from franka_control_client.franka_robot.franka_panda import (
    RemotePandaArm,
)

from franka_control_client.vr.meta_quest3 import MQ3Controller
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.control_pair.mq3_panda_control_pair import (
    MQ3PandaControlPair,
)

if __name__ == "__main__":
    pyzlc.init(
        "data_collection",
        "192.168.1.1",
        group_name="DroidGroup",
        group_port=7730,
    )
    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    leader = MQ3Controller("IRL-MQ3-2", "192.168.0.117", follower.panda_arm)
    leader.mq3.wait_for_connection()
    control_pair = MQ3PandaControlPair(leader, follower)
    # for now capture_interval is not using only using the global frequency fps
    camera_left = ImageDataWrapper(
        CameraDevice("zed_left", preview=False),
        capture_interval=0.033,
        hw_name="zed_left",
    )
    camera_right = ImageDataWrapper(
        CameraDevice("zed_right", preview=False),
        capture_interval=0.033,
        hw_name="zed_right",
    )
    camera_wrist = ImageDataWrapper(
        CameraDevice("zed_wrist", preview=False),
        capture_interval=0.033,
        hw_name="zed_wrist",
    )
    data_collectors: List[IRLDataWrapper] = []
    data_collectors.append(camera_left)
    data_collectors.append(camera_right)
    data_collectors.append(camera_wrist)
    data_collectors.append(MQ3DataWrapper(leader))
    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))
    # name = time.strftim  e("%Y%m%d_%H%M%S", time.localtime())
    task = "pick_up_banana"
    data_collection_manager = IRLDataCollection(
        data_collectors,
        f"/home/irl-admin/xinkai/data_collection/{task}",
        task,
        fps=40,
        control_pair=control_pair,
    )
    control_pair.control_reset()
    data_collection_manager.run()
    pyzlc.shutdown()
