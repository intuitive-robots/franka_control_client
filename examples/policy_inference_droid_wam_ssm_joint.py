from typing import List

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.control_pair.policy_panda_control_pair import (
    PolicyPandaControlPair,
)
from franka_control_client.franka_robot.panda_arm import RemotePandaArm
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.policy_inference.irl_wrapper import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
)
from franka_control_client.policy_inference.remote_joint_policy_inference import (
    RemoteJointPolicyInference,
)
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)


if __name__ == "__main__":
    # The WamSSM DROID model predicts 15 Hz chunks of absolute joint targets +
    # gripper closedness; 8 rows are executed per request (one memory step).
    POLICY_FPS = 15
    # The control pair plays the chunk rows at the policy rate (one row per step).
    CONTROL_HZ = POLICY_FPS

    pyzlc.init(
        "policy_inference",
        "192.168.1.1",
        group_name="DroidGroup",
        group_port=7730,
    )

    try:
        # Must be in the model's text-embedding table (DROID phrasings), the
        # server matches it case- and punctuation-insensitively.
        task = "put the marker in the cup"
        policy_server_host = "127.0.0.1"
        policy_server_port = 8767  # must match wam_ssm_droid_policy_server.py --port

        follower = PandaRobotiq(
            "PandaRobotiq",
            RemotePandaArm("FrankaPanda"),
            RemoteRobotiqGripper("FrankaPanda"),
        )
        # Joint-space control pair: action = [q0..q6, gripper], absolute joint
        # targets, gripper >= 0.5 closes the Robotiq.
        control_pair = PolicyPandaControlPair(
            follower.panda_arm, follower.robotiq_gripper, CONTROL_HZ
        )

        # Full-resolution frames; the server resizes them to DROID's 180x320.
        camera_right = ImageDataWrapper(
            CameraDevice("zed_right", preview=False),
            capture_interval=1.0 / POLICY_FPS,
            hw_name="zed_right",
        )
        camera_wrist = ImageDataWrapper(
            CameraDevice("zed_wrist", preview=False),
            capture_interval=1.0 / POLICY_FPS,
            hw_name="zed_wrist",
        )

        data_collectors: List[IRL_HardwareDataWrapper] = []
        data_collectors.append(camera_right)
        data_collectors.append(camera_wrist)
        data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
        data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))

        inference_manager = RemoteJointPolicyInference(
            data_collectors=data_collectors,
            control_pair=control_pair,
            task=task,
            fps=POLICY_FPS,
            server_host=policy_server_host,
            server_port=policy_server_port,
            # None: wait as long as the server needs (the first step warms up).
            request_timeout_s=None,
            # The model was trained on DROID's right exterior camera + wrist camera:
            #   observation.images.external -> camera_external_right
            #   observation.images.wrist    -> camera_wrist_left
            camera_key_map={
                "zed_right": "observation.images.external",
                "zed_wrist": "observation.images.wrist",
            },
        )

        inference_manager.run()
    finally:
        pyzlc.shutdown()
