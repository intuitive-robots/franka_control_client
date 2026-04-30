from typing import List

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.control_pair.cartesian_policy_panda_control_pair import PolicyPandaRobotiqDeltaCartesianControlPair
from franka_control_client.control_pair.pil_panda_control_pair import PILPandaControlPair
from franka_control_client.control_pair.policy_panda_control_pair import (
    PolicyPandaControlPair,
)
from franka_control_client.franka_robot.panda_arm import RemotePandaArm
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.data_collection.irl_wrapper import (
    IRLDataWrapper,
    ImageDataWrapper,
    MQ3DataWrapper,
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
)
from franka_control_client.policy_inference.lerobot_policy_inference import (
    LeRobotPolicyInference,
    LeRobotPolicyInferenceConfig,
)
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)
from franka_control_client.vr.meta_quest3 import MQ3Controller


if __name__ == "__main__":
    pyzlc.init(
        "policy_inference",
        "141.3.53.25",
        group_name="robot_lab_robotiq_202",
        group_port=7725,
    )

    # Checkpoint path from eval_config.yaml
    checkpoint_path = (
        "/home/jjiang/jing/model/beso/bestmodel_fold_abs_car/checkpoints/010000/pretrained_model" 
    )
    task = "folding"
    dataset_path = "/home/jjiang/jing/dataset/lerobot/folding_20hz_abs_cartesian_action" 

    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    leader = MQ3Controller("IRL-MQ3-2", "192.168.0.117", follower.panda_arm)
    leader.mq3.wait_for_connection()
    control_pair = PILPandaControlPair(
        follower.panda_arm, follower.robotiq_gripper, leader, 50
    )

    # Camera capture interval matches inference frequency (30 Hz = 0.033s)
    static_cam = ImageDataWrapper(
        CameraDevice("static_cam", preview=False),
        capture_interval=0.033,
        hw_name="static_cam",
    )
    wrist_cam = ImageDataWrapper(
        CameraDevice("wrist_cam", preview=False),
        capture_interval=0.033,
        hw_name="wrist_cam",
    )
#    camera_wrist = ImageDataWrapper(
#        CameraDevice("zed_wrist", preview=False),
#        capture_interval=0.033,
#        hw_name="zed_wrist",
#    )

    data_collectors: List[IRLDataWrapper] = []
    data_collectors.append(MQ3DataWrapper(leader))
    data_collectors.append(static_cam)
    data_collectors.append(wrist_cam)
#    data_collectors.append(camera_wrist)
    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))

    inference_cfg = LeRobotPolicyInferenceConfig(
        checkpoint_path=checkpoint_path,
        task=task,
        fps=10,
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