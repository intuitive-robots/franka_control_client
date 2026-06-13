from typing import List

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.control_pair.cartesian_policy_panda_control_pair import (
    CartesianPolicyPandaControlPair,
)
from franka_control_client.franka_robot.panda_arm import RemotePandaArm
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.policy_inference.irl_wrapper import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
)
from franka_control_client.policy_inference.remote_policy_inference import (
    RemotePolicyInference,
)
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)


if __name__ == "__main__":
    POLICY_FPS = 4
    CONTROL_HZ = 4

    pyzlc.init(
        "policy_inference",
        "192.168.1.1",
        group_name="DroidGroup",
        group_port=7730,
    )

    try:
        task = "" #"Pick up the bell pepper and place it in the bowl."
        policy_server_host = "127.0.0.1"
        policy_server_port = 8765

        follower = PandaRobotiq(
            "PandaRobotiq",
            RemotePandaArm("FrankaPanda"),
            RemoteRobotiqGripper("FrankaPanda"),
        )
        control_pair = CartesianPolicyPandaControlPair(
            follower.panda_arm,
            follower.robotiq_gripper,
            CONTROL_HZ,
            action_rotation_mode="euler",
            action_pose_mode="delta",
            action_gripper_mode="absolute",
        )

        camera_left = ImageDataWrapper(
            CameraDevice("zed_left", preview=False, final_size=(256, 256)),
            capture_interval=1.0 / POLICY_FPS,
            hw_name="zed_left",
        )
        camera_wrist = ImageDataWrapper(
            CameraDevice("zed_wrist", preview=False, final_size=(256, 256)),
            capture_interval=1.0 / POLICY_FPS,
            hw_name="zed_wrist",
        )

        data_collectors: List[IRL_HardwareDataWrapper] = []
        data_collectors.append(camera_left)
        data_collectors.append(camera_wrist)
        data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
        data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))

        inference_manager = RemotePolicyInference(
            data_collectors=data_collectors,
            control_pair=control_pair,
            task=task,
            fps=POLICY_FPS,
            server_host=policy_server_host,
            server_port=policy_server_port,
            state_mode="ee_euler_gripper",
            expected_image_shape=(256, 256, 3),
        )

        inference_manager.run()
    finally:
        pyzlc.shutdown()
