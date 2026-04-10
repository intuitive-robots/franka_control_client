import time
import os
import sys

# Add hardware directory to path to import GelloAgent
# sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
# from hardware.gello_zlc import GelloAgent

from typing import List

from franka_control_client.data_collection.pil_irl_vr_data_collection import PILIRLDataCollection
import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.data_collection.irl_vr_data_collection import (
    IRLDataCollection,
)
from franka_control_client.data_collection.irl_wrapper import (
    IRL_HardwareDataWrapper,
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
        "141.3.53.25",
        group="224.0.0.1",
        group_name="robot_lab_robotiq_202",
        group_port=7725,
        
    )
    
    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    leader = MQ3Controller("IRL-MQ3-2", "192.168.0.117", follower.panda_arm)
    leader.mq3.wait_for_connection()
    control_pair = MQ3PandaControlPair(leader, follower)
    static_cam = ImageDataWrapper(CameraDevice("static_cam", preview=True), hw_name="static_cam")
    wrist_cam = ImageDataWrapper(CameraDevice("wrist_cam", preview=True), hw_name="wrist_cam")
    data_collectors: List[IRL_HardwareDataWrapper] = []
    data_collectors.append(static_cam)
    data_collectors.append(wrist_cam)
    data_collectors.append(MQ3DataWrapper(leader))
    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))
    name = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    task = "folding"
    data_collection_manager = PILIRLDataCollection(
        data_collectors, 
        f"/home/jjiang/jing/dataset/{task}", 
        task, 
        fps=40,
        control_pair=control_pair
    )
    control_pair.control_reset()
    data_collection_manager.run()
    pyzlc.shutdown()
