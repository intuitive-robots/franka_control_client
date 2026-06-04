from typing import List
import sys
from pathlib import Path
from contextlib import contextmanager

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


@contextmanager
def _pyzlc_nonblocking_executor_shutdown():
    """Avoid hanging forever in pyzlc executor shutdown during process exit."""
    try:
        from pyzlc.nodes.loop_manager import DaemonThreadPoolExecutor
    except Exception:
        yield
        return

    original_shutdown = DaemonThreadPoolExecutor.shutdown

    def shutdown_without_join(self, wait=True, *, cancel_futures=False):
        try:
            return original_shutdown(
                self, wait=False, cancel_futures=True
            )
        except TypeError:
            return original_shutdown(self, wait=False)

    DaemonThreadPoolExecutor.shutdown = shutdown_without_join
    try:
        yield
    finally:
        DaemonThreadPoolExecutor.shutdown = original_shutdown


def _detach_pyzlc_executor_threads() -> None:
    """Prevent Python's ThreadPoolExecutor exit hook from joining pyzlc forever."""
    try:
        import concurrent.futures.thread as futures_thread
        import threading
    except Exception:
        return

    lancom_threads = []
    with futures_thread._global_shutdown_lock:
        for thread, work_queue in list(futures_thread._threads_queues.items()):
            if not thread.name.startswith("LanComPool"):
                continue
            lancom_threads.append(thread)
            try:
                work_queue.put_nowait(None)
            except Exception:
                pass
            try:
                del futures_thread._threads_queues[thread]
            except KeyError:
                pass

    if not hasattr(threading, "_shutdown_locks"):
        return
    with threading._shutdown_locks_lock:
        for thread in lancom_threads:
            lock = getattr(thread, "_tstate_lock", None)
            if lock is not None:
                threading._shutdown_locks.discard(lock)


def _shutdown_pyzlc() -> None:
    try:
        with _pyzlc_nonblocking_executor_shutdown():
            pyzlc.shutdown()
    finally:
        _detach_pyzlc_executor_threads()


if __name__ == "__main__":
    pyzlc.init(
        "data_collection",
        "141.3.53.25",
        group="224.0.0.1",
        group_name="robot_lab_robotiq_202",
        group_port=7725,
        # log_level=pyzlc.LogLevel.DEBUG,

        
    )
    # Checkpoint path from eval_config.yaml
    checkpoint_path = (
        "/home/jjiang/jing/model/xvla/cylinder_split_03/080000/pretrained_model" 
    )
    task = "put green cylinder on yellow cube."
    dataset_path = "/home/jjiang/jing/dataset/lerobot/cylinder_full_balanced_splits/split_03" 
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
        follower.panda_arm,
        follower.robotiq_gripper,
        leader,
        control_hz=1000,
        # action_chunk_size=10,
        # action_chunk_dt=100,
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
        fps=5,
        device="cuda",
        # policy_dtype="bfloat16",
        dataset_path=dataset_path,
    )
    inference_manager = MQ3TrajVisualDataCollectionInference(
        data_collectors=data_collectors,
        control_pair=control_pair,
        task=task,
        cfg=inference_cfg,
        save_path="/home/jjiang/ahmad/dataset/",
        mirror = mirror,
        visualization_hz=30.0,
        action_buffer_refill_threshold=0,
    )

    try:
        inference_manager.run()
    finally:
        _shutdown_pyzlc()
