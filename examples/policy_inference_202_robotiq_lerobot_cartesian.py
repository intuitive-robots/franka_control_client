from typing import List

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.control_pair.cartesian_policy_panda_control_pair import (
    PolicyPandaRobotiqDeltaCartesianControlPair,
)

from franka_control_client.franka_robot.panda_arm import RemotePandaArm
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.data_collection.irl_wrapper import (
    IRLDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
)
from franka_control_client.policy_inference.lerobot_policy_inference import (
    LeRobotPolicyInferenceConfig,
)
from franka_control_client.policy_inference.mq3_traj_visual_lerobot_inference import (
    MQ3TrajVisualLeRobotInference,
)
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)


if __name__ == "__main__":
    pyzlc.init(
        "policy_inference",
        "141.3.53.25",
        group_name="robot_lab_robotiq_202",
        group_port=7725,
    )

    # Checkpoint path from eval_config.yaml
    checkpoint_path = (
        "/home/jjiang/jing/model/xvla/checkpoints/080000/pretrained_model" 
    )
    task = "pick up banana."
    dataset_path = "/home/jjiang/jing/dataset/lerobot/pick_up_banana_20hz_gripper_0_5_to_1_delta_cartesian_euler" 

    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    control_pair = PolicyPandaRobotiqDeltaCartesianControlPair(
        follower.panda_arm, follower.robotiq_gripper, 100, 1, 0.1
    )

    # Camera capture interval matches inference frequency (30 Hz = 0.033s)
    static_cam = ImageDataWrapper(CameraDevice("static_cam", preview=False), hw_name="static_cam")
    wrist_cam = ImageDataWrapper(CameraDevice("wrist_cam", preview=False), hw_name="wrist_cam")

    data_collectors: List[IRLDataWrapper] = []
    data_collectors.append(static_cam)
    data_collectors.append(wrist_cam)

    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))

    inference_cfg = LeRobotPolicyInferenceConfig(
        checkpoint_path=checkpoint_path,
        task=task,
        fps=10,
        device="cuda",
        dataset_path=dataset_path,
    )
    inference_manager = MQ3TrajVisualLeRobotInference(
        data_collectors=data_collectors,
        control_pair=control_pair,
        cfg=inference_cfg,
    )
    try:
        inference_manager.run()
    finally:
        pyzlc.shutdown()
