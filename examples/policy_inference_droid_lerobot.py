import time
from typing import List

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.control_pair.policy_panda_control_pair import (
    PolicyPandaControlPair,
)
from franka_control_client.franka_robot.panda_arm import RemotePandaArm
from franka_control_client.policy_inference.irl_wrapper import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
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


if __name__ == "__main__":
    pyzlc.init(
        "policy_inference",
        "192.168.0.117",
        group_name="DroidGroup",
        group_port=7730,
    )

    policy_name = "xvla"
    obs_topic = f"{policy_name}/observation"
    action_topic = f"{policy_name}/action"

    panda_arm = RemotePandaArm("FrankaPanda")
    robotiq_gripper = RemoteRobotiqGripper("FrankaPanda")
    control_pair = PolicyPandaControlPair(panda_arm, robotiq_gripper)

    camera_left = ImageDataWrapper(CameraDevice("zed_left", preview=False))

    data_collectors: List[IRL_HardwareDataWrapper] = [
        camera_left,
        PandaArmDataWrapper(panda_arm),
        RobotiqGripperDataWrapper(robotiq_gripper),
    ]

    inference_cfg = LeRobotPolicyInferenceConfig(
        policy_name=policy_name,
        task="pepper",
        fps=30,
        obs_topic=obs_topic,
        action_topic=action_topic,
    )
    inference_manager = LeRobotPolicyInference(
        data_collectors=data_collectors,
        control_pair=control_pair,
        cfg=inference_cfg,
    )
    inference_manager.run()
    pyzlc.shutdown()
