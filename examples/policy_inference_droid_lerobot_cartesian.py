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
    POLICY_FPS = 50
    CONTROL_HZ = 50

    pyzlc.init(
        "policy_inference",
        "192.168.1.1",
        group_name="DroidGroup",
        group_port=7730,
    )

    try:
        task = "" #"Pick up the bell pepper and place it in the bowl."
        policy_server_host = "127.0.0.1"
        policy_server_port = 8766  # must match valpa_roboarena_policy_server.py --port

        follower = PandaRobotiq(
            "PandaRobotiq",
            RemotePandaArm("FrankaPanda"),
            RemoteRobotiqGripper("FrankaPanda"),
        )
        # Start/reset configuration: joint_pos of frame 105 from
        # /home/irl-admin/new_data_collection/test/2026_06_16-08_56_39/FrankaPanda
        # When 'r' (reset) is pressed the robot moves to this joint position.
        START_JOINT_POSITION = (
            0.5177174113466025,
            0.3531982815893073,
            0.10890358011644215,
            -2.049610618091363,
            -0.056924680449064076,
            2.4857384326739114,
            -0.8929762131281559,
        )

        control_pair = CartesianPolicyPandaControlPair(
            follower.panda_arm,
            follower.robotiq_gripper,
            CONTROL_HZ,
            action_rotation_mode="euler",
            action_pose_mode="delta",
            # The valpa server returns a gripper *delta* (closedness), not an
            # absolute target. See compute_new_pose: new = clip(current + d, 0, 1).
            action_gripper_mode="delta",
            home_joint_position=START_JOINT_POSITION,
            # Close the gripper when moving to the start/reset pose.
            home_gripper_position=1.0,
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
            # None disables the request timeout: the client waits indefinitely
            # for the server's response (the first inference can be slow to warm
            # up). Key presses stay responsive because infer runs in a worker.
            request_timeout_s=None,
            state_mode="ee_euler_gripper",
            expected_image_shape=(256, 256, 3),
            # The valpa server ignores force/torque, so don't send it.
            include_force_torque=False,
            # The valpa server reads exactly these two image keys:
            #   observation.images.image  -> external/side camera
            #   observation.images.image2 -> wrist camera
            camera_key_map={
                "zed_left": "observation.images.image",
                "zed_wrist": "observation.images.image2",
            },
            # Goal-conditioned server: goal images (+ optional goal state) are
            # loaded from this directory at the start of each episode and sent
            # via set_goal. Expected layout:
            #   /home/irl-admin/jakub/goal_images/left.png   (external/side cam)
            #   /home/irl-admin/jakub/goal_images/wrist.png  (wrist cam)
            #   /home/irl-admin/jakub/goal_images/state.npy  (optional 7-D goal
            #       state [x, y, z, roll, pitch, yaw, gripper]; enables the
            #       goal-reached auto-stop)
            goal_dir="/home/irl-admin/jakub/goal_images",
        )

        inference_manager.run()
    finally:
        pyzlc.shutdown()
