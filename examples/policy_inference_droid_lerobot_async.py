from typing import List

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.control_pair.policy_panda_control_pair_chunk import (
    PolicyPandaControlPair,
)
from franka_control_client.franka_robot.panda_arm import RemotePandaArm
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.policy_inference.irl_wrapper_async import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
)
from franka_control_client.policy_inference.lerobot_policy_inference_async import (
    LeRobotPolicyInference,
    LeRobotPolicyInferenceConfig,
)
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
    # checkpoint_path = "/home/irl-admin/dual_async_vla/040000/pretrained_model"
    checkpoint_path = "/home/irl-admin/LZY/julich_models/dualratevla/forcevis100hz_t3_consehist/064000/pretrained_model"
    # checkpoint_path = "/home/irl-admin/LZY/julich_models/dualratevla/forcevis100hz_t4_consehist/064000/pretrained_model"
    # checkpoint_path = "/home/irl-admin/LZY/julich_models/dualratevla/forcevis100hz_t4_gaphist/last/pretrained_model"
    task = "fold the scarf on the table." #"Pick up the bell pepper and place it in the bowl."
    # dataset_path = "/home/irl-admin/new_data_collection/lerobot_meta/new_scarf_40hz"
    dataset_path = "/home/irl-admin/new_data_collection/lerobot_meta/scarf_25hz_100hz_ft"
    observation_history_lengths = {
        "state": 1,
        "camera": 6,
    }
    observation_buffer_capacities = {
        "state": 8,
        "camera": 8,
    }

    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    control_pair = PolicyPandaControlPair(follower.panda_arm, follower.robotiq_gripper)

    # Camera capture interval matches inference frequency (30 Hz = 0.033s)
    # camera_left = ImageDataWrapper(
    #     CameraDevice("zed_left", preview=False), capture_interval=0.025, hw_name="zed_left"
    # )
    camera_right = ImageDataWrapper(
        CameraDevice("zed_right", preview=False), capture_interval=0.025, hw_name="zed_right"
    )
    camera_wrist = ImageDataWrapper(
        CameraDevice("zed_wrist", preview=False), capture_interval=0.025, hw_name="zed_wrist"
    )

    data_collectors: List[IRL_HardwareDataWrapper] = []
    # data_collectors.append(camera_left)
    data_collectors.append(camera_right)
    data_collectors.append(camera_wrist)
    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))

    inference_cfg = LeRobotPolicyInferenceConfig(
        checkpoint_path=checkpoint_path,
        task=task,
        fps=400,
        device="cuda",
        policy_dtype="bfloat16",
        dataset_path=dataset_path,
        observation_history_lengths=observation_history_lengths,
        observation_buffer_capacities=observation_buffer_capacities,
        pad_history_with_oldest=True,
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
