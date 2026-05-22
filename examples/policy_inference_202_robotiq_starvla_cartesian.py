# =============================================================================
# SETUP
#
# On the robot PC (franka_control_client env):
#   pip install websockets msgpack scipy opencv-python
#
# HOW TO RUN EVAL
#
# Step 1 — start the policy server on the GPU node (starVLA repo):
#   cd starVLA/
#   python deployment/model_server/server_poli
# cy.py \
#       --ckpt_path /path/to/checkpoint.pt \
#       --port 10093 \
#       --use_bf16


#   python deployment/model_server/server_poli
# cy.py \
#       --ckpt_path /home/jjiang/nils/models/GR00T_real_robot_abs_eef_qwen08ft/final_model/pytorch_model.pt \
#       --port 10093 \
#       --use_bf16



#   Check logs for available_unnorm_keys and action_chunk_size.
#
# Step 2 — smoke test (no robot, verifies server connection + action shape):
#   cd starVLA/
#   python franka_control_client/src/franka_control_client/policy_inference/starvla_server_inference.py \
#       --host <server_ip> --port 10093 \
#       --task "put red cylinder on green cube" \
#       --n_cameras 2 --n_infer 3


#   python franka_control_client/src/franka_control_client/policy_inference/starvla_server_inference.py \
#       --port 10093 \
#       --task "put red cylinder on green cube" \
#       --n_cameras 2 --n_infer 3
#   Expected: actions shape=(24, 8) and "Smoke test PASSED".
#   Add --include_state only if model was trained with include_state: true.
#
# Step 3 — deploy on robot (this script):
#   Set SERVER_HOST below to the GPU node IP if server is on a different machine.
#   cd starVLA/
#   python franka_control_client/examples/policy_inference_202_robotiq_starvla_cartesian.py
#   Keyboard: n=start episode  s=stop  d=discard  r=reset arm  q=quit
# =============================================================================

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
from franka_control_client.policy_inference.starvla_server_inference import (
    StarVLAServerInference,
)
from franka_control_client.robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper


if __name__ == "__main__":
    pyzlc.init(
        "starvla_policy_inference",
        "141.3.53.25",
        group_name="robot_lab_robotiq_202",
        group_port=7725,
    )

    # ----------------------------------------------------------------
    # Configuration — adapt to your deployment
    # ----------------------------------------------------------------
    TASK = "pick_up_banana"

    TASK = 'Put the orange lego block into the red dustpan'

    TASK_LIST = [
        "Put the orange carrot into the red bowl",
        "Put the orange carrot into the red dustpan",
        "Put the orange lego block into the red bowl",
        "Put the orange lego block into the red dustpan",
        "Put the yellow cube into the red bowl",
        "Put the yellow cube into the red dustpan",
        "Put the yellow cuboid into the red bowl",
        "Put the yellow cuboid into the red dustpan",
        "Put the yellow lego block into the red bowl",
        "Put the yellow lego block into the red dustpan",

    ]

    SERVER_HOST = "127.0.0.1"  # IP of the machine running server_policy.py
    SERVER_PORT = 10093

    # unnorm_key must match the dataset key from training.
    # Check server startup logs for available_unnorm_keys if unsure.
    UNNORM_KEY = None  # set e.g. "your_dataset_name" if the server has multiple keys

    INFERENCE_FPS = 10          # how fast the inference loop runs
    IMAGE_SIZE = (224, 224)     # must match training obs_image_size
    ACTION_CHUNK_DT = 0.05      # seconds per chunk step at the control pair level
    CONTROL_HZ = 200

    # ----------------------------------------------------------------
    # Hardware setup (same as existing BESO example)
    # ----------------------------------------------------------------
    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    control_pair = PolicyPandaRobotiqDeltaCartesianControlPair(
        follower.panda_arm,
        follower.robotiq_gripper,
        CONTROL_HZ,
        action_chunk_size=1,       # buffer size for the physical control loop
        action_chunk_dt=ACTION_CHUNK_DT,
    )

    # Camera order must match training: primary (static) first, wrist second
    static_cam = ImageDataWrapper(CameraDevice("static_cam", preview=False), hw_name="static_cam")
    wrist_cam = ImageDataWrapper(CameraDevice("wrist_cam", preview=False), hw_name="wrist_cam")

    data_collectors: List[IRLDataWrapper] = [
        static_cam,
        wrist_cam,
        PandaArmDataWrapper(follower.panda_arm),
        RobotiqGripperDataWrapper(follower.robotiq_gripper),
    ]

    # ----------------------------------------------------------------
    # Inference manager
    # ----------------------------------------------------------------
    inference_manager = StarVLAServerInference(
        data_collectors=data_collectors,
        control_pair=control_pair,
        task=TASK,
        fps=INFERENCE_FPS,
        host=SERVER_HOST,
        port=SERVER_PORT,
        unnorm_key=UNNORM_KEY,
        image_size=IMAGE_SIZE,
        include_state=False,  # set True if model was trained with include_state: true
    )

    try:
        inference_manager.run()
    finally:
        pyzlc.shutdown()
