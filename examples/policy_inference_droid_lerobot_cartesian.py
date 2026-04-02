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
from franka_control_client.policy_inference.lerobot_policy_inference import (
    LeRobotPolicyInferenceConfig,
)
from franka_control_client.policy_inference.lerobot_policy_inference import LeRobotPolicyInference
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)


if __name__ == "__main__":
    pyzlc.init(
        "policy_inference",
        "192.168.1.1",
        group_name="DroidGroup",
        group_port=7730,
    )

    # Checkpoint path from eval_config.yaml
    checkpoint_path = "/home/irl-admin/chekpoints/4th_March_folding/pretrained_model"
    checkpoint_path = "/home/irl-admin/xinkai/xvla_checkpoints/100000/pretrained_model"
    checkpoint_path = "/home/irl-admin/xinkai/xvla_checkpoints/put_banana/200000/pretrained_model"
    task = "pick_up_banana" #"Pick up the bell pepper and place it in the bowl."
    dataset_path = "/home/irl-admin/chekpoints/4th_March_folding"
    dataset_path = "/home/irl-admin/xinkai/lerobot_format/pick_up_banana_40hz"

    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    control_pair = CartesianPolicyPandaControlPair(follower.panda_arm, follower.robotiq_gripper, 50)

    # Camera capture interval matches inference frequency (30 Hz = 0.033s)
    camera_left = ImageDataWrapper(
        CameraDevice("zed_left", preview=False), capture_interval=0.033, hw_name="zed_left"
    )
    camera_right = ImageDataWrapper(
        CameraDevice("zed_right", preview=False), capture_interval=0.033, hw_name="zed_right"
    )
    camera_wrist = ImageDataWrapper(
        CameraDevice("zed_wrist", preview=False), capture_interval=0.033, hw_name="zed_wrist"
    )

    data_collectors: List[IRL_HardwareDataWrapper] = []
    data_collectors.append(camera_left)
    data_collectors.append(camera_right)
    data_collectors.append(camera_wrist)
    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))

    inference_cfg = LeRobotPolicyInferenceConfig(
        checkpoint_path=checkpoint_path,
        task=task,
        fps=3,
        device="cuda",
        policy_dtype="bfloat16",
        dataset_path=dataset_path,
    )
    inference_manager = LeRobotPolicyInference(
        data_collectors=data_collectors,
        control_pair=control_pair,
        cfg=inference_cfg,
    )
    
    try:
        inference_manager.run()
    finally:
        pyzlc.shutdown()
