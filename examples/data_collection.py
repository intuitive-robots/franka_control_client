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
    PandaGripperDataWrapper,
)
from franka_control_client.franka_robot.franka_panda import (
    FrankaPanda,
    RemotePandaArm,
    RemotePandaGripper,
)
from franka_control_client.control_pair.single_panda_control_pair import (
    SinglePandaKTControlPair,
)

if __name__ == "__main__":
    pyzlc.init(
        "data_collection",
        "192.168.0.134",
        group_name="hardware_collection",
    )
    leader = FrankaPanda(
        "franka_panda",
        RemotePandaArm("Panda201"),
        RemotePandaGripper("Panda201Gripper"),
    )
    follower = FrankaPanda(
        "franka_panda_follower",
        RemotePandaArm("Panda202"),
        RemotePandaGripper("Panda202Gripper"),
    )
    control_pair = SinglePandaKTControlPair(leader, follower)
    camera = ImageDataWrapper(CameraDevice("depthai_camera", preview=True))
    data_collectors: List[HardwareDataWrapper] = []
    data_collectors.append(camera)
    data_collectors.append(PandaArmDataWrapper(leader.panda_arm))
    data_collectors.append(PandaGripperDataWrapper(leader.panda_gripper))
    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(PandaGripperDataWrapper(follower.panda_gripper))
    name = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    data_collection_manager = LeRobotDataCollection(
        data_collectors, f"/home/xinkai/datasets/{name}", task="pick_and_place"
    )
    data_collection_manager.register_start_collecting_event(
        control_pair.start_control_pair
    )
    data_collection_manager.register_stop_collecting_event(
        control_pair.stop_control_pair
    )
    data_collection_manager.run()
    pyzlc.shutdown()
