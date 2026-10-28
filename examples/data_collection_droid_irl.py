import argparse
import time
import os
import sys
from pathlib import Path

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

from franka_control_client.franka_robot.panda_arm import ControlMode
from franka_control_client.gello.gello import RemoteGello
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.control_pair.gello_panda_control_pair import (
    GelloPandControlPair,
)

CONTROL_MODES = {
    "stiff": ControlMode.HybridJointImpedance,
    "compliant": ControlMode.HybridJointImpedanceCompliant,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GELLO teleoperation data collection (DROID setup)")
    parser.add_argument("--task", default="pick_up_knife", help="Task name, recordings go to <data-dir>/<task>")
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path("/home/irl-admin/new_data_collection"),
        help="Recordings go to <data-dir>/<task>",
    )
    parser.add_argument(
        "--control-mode",
        choices=list(CONTROL_MODES),
        default="stiff",
        help="Controller gain preset for teleoperation",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
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
    control_mode = CONTROL_MODES[args.control_mode]
    pyzlc.info(f"Teleoperation control mode: {args.control_mode} ({control_mode.value})")
    control_pair = GelloPandControlPair(leader, follower, control_mode=control_mode)
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
    task = args.task  # previous tasks: battery_100hz, insert_blue_bird_100hz_cam_25hz
    data_collection_manager = IRLDataCollection(
        data_collectors,
        str(args.data_dir / task),
        task,
        fps=4,
        control_pair=control_pair
    )
    control_pair.control_reset()
    data_collection_manager.run()
    pyzlc.shutdown()
