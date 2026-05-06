from typing import List
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from digital_twin import RobotMirror, RobotModelId
import pyzlc

from franka_control_client.camera.camera import CameraDevice
# from franka_control_client.control_pair.cartesian_policy_panda_control_pair import (
#     CartesianPolicyPandaControlPair,
# )
from franka_control_client.control_pair.pil_panda_control_pair import (
    PILPandaControlPair,
)
from franka_control_client.franka_robot.panda_arm import RemotePandaArm
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.data_collection.irl_wrapper import (
    IRLDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
)
from franka_control_client.data_collection.irl_wrapper import MQ3DataWrapper

from franka_control_client.policy_inference.lerobot_policy_inference import (
    LeRobotPolicyInferenceConfig,
)
from franka_control_client.policy_inference.mq3_traj_visual_data_collection import (
    MQ3TrajVisualDataCollectionInference,
)
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)
from franka_control_client.vr.meta_quest3 import MQ3Controller
# from franka_control_client.control_pair.mq3_panda_control_pair import (
#     MQ3PandaControlPair,
# )


if __name__ == "__main__":
    pyzlc.init(
        "data_collection",
        "141.3.53.25",
        group="224.0.0.1",
        group_name="robot_lab_robotiq_202",
        group_port=7725,
        
    )
    # Checkpoint path from eval_config.yaml
    checkpoint_path = (
        "/home/jjiang/jing/model/beso/bestmodel_fold_abs_car/checkpoints/last/pretrained_model"
    )
    task = "folding"
    dataset_path = "/home/jjiang/jing/dataset/lerobot/folding_20hz" 

    # task = "pick_up_cylinder_on_the_top_of_cube"  # "Pick up the bell pepper and place it in the bowl."
    # dataset_path = "/home/irl-admin/chekpoints/4th_March_folding"
    # dataset_path = "/home/jjiang/ahmad/bowl_on_blender_abs_cartesian_action"
    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    mirror = RobotMirror.from_model_id(
        RobotModelId.FRANKA_PANDA_ROBOTIQ
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
        hw_name="image",
    )
    wrist_cam = ImageDataWrapper(
        CameraDevice("wrist_cam", preview=False),
        capture_interval=0.033,
        hw_name="image2",
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
 #   data_collectors.append(camera_wrist)
    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))

    inference_cfg = LeRobotPolicyInferenceConfig(
        checkpoint_path=checkpoint_path,
        task=task,
        fps=10,
        device="cuda",
        # policy_dtype="bfloat16",
        dataset_path=dataset_path,
    )
    inference_manager = MQ3TrajVisualDataCollectionInference(
        data_collectors=data_collectors,
        control_pair=control_pair,
        task="pick_up_cylinder_on_the_top_of_cube",
        cfg=inference_cfg,
        save_path="/home/jjiang/ahmad/dataset/lerobot/pick_up_cylinder_on_the_top_of_cube_mq3_data_collection",
        mirror = mirror
    )

    try:
        inference_manager.run()
    finally:
        pyzlc.shutdown()
