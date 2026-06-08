from pathlib import Path
from typing import List

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.data_collection.irl_data_collection import (
    IRLDataCollection,
)
from franka_control_client.data_collection.irl_wrapper import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
)
from franka_control_client.franka_robot.franka_panda import (
    RemotePandaArm,
)
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)
from franka_control_client.control_pair.trajectory_with_torque_panda_control_pair import (
    TrajectoryTorquePandaControlPair,
)


# DEFAULT_TRAJECTORY_DIR = Path(
#     "/home/irl-admin/new_data_collection/usb_25_100hz/"
#     "2026_04_29-16_35_34/FrankaPanda"
# )
DEFAULT_TRAJECTORY_DIR = Path(
    "/home/irl-admin/new_data_collection/human_button_task"
    "2026_05_03-17_08_27/FrankaPanda"
)

class ReplayIRLDataCollection(IRLDataCollection):
    def _save_data_task(self) -> None:
        super()._save_data_task()
        self.control_pair.save_trajectory(self.record_dir / "Traj")


if __name__ == "__main__":
    pyzlc.init(
        "replay_collection",
        "192.168.1.1",
        group_name="DroidGroup",
        group_port=7730,
    )
    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    control_pair = TrajectoryTorquePandaControlPair(
        DEFAULT_TRAJECTORY_DIR,
        follower,
        control_hz=100,
    )
    # camera frequency controlled individually
    # cams 25hz
    camera_left = ImageDataWrapper(CameraDevice("zed_left", preview=False), capture_interval=0.04, hw_name="zed_left")
    camera_right = ImageDataWrapper(CameraDevice("zed_right", preview=False), capture_interval=0.04, hw_name="zed_right")
    camera_wrist = ImageDataWrapper(CameraDevice("zed_wrist", preview=False), capture_interval=0.04, hw_name="zed_wrist")

    data_collectors: List[IRL_HardwareDataWrapper] = []
    data_collectors.append(camera_left)
    data_collectors.append(camera_right)
    data_collectors.append(camera_wrist)
    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))

    task = "replay_demo_test"
    data_collection_manager = ReplayIRLDataCollection(
        data_collectors,
        f"/home/irl-admin/new_data_collection/{task}",
        task,
        fps=100,
        control_pair=control_pair,
    )
    data_collection_manager.run()
    pyzlc.shutdown()
