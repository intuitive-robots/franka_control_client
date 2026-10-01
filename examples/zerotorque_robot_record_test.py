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
)
from franka_control_client.franka_robot.franka_panda import (
    RemotePandaArm,
)

from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.control_pair.human_control import (
    HumanPandaControlPair,
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
    control_pair = HumanPandaControlPair(follower)

    data_collectors: List[IRL_HardwareDataWrapper] = []
    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))
    camera_right = ImageDataWrapper(CameraDevice("zed_right", preview=False),capture_interval=0.067,hw_name="zed_right")
    camera_wrist = ImageDataWrapper(CameraDevice("zed_wrist", preview=False),capture_interval=0.067,hw_name="zed_wrist")
    # data_collectors.append(camera_left)
    data_collectors.append(camera_right)
    data_collectors.append(camera_wrist) 
    # name = time.strftim  e("%Y%m%d_%H%M%S", time.localtime())
    task = "105_test" #insert_blue_bird_100hz_cam_25hz
    data_collection_manager = IRLDataCollection(
        data_collectors, 
        f"/home/irl-admin/new_data_collection/robot_test/{task}", 
        task, 
        fps=100,
        control_pair=control_pair
    )
    control_pair.control_reset()
    data_collection_manager.run()
    pyzlc.shutdown()
