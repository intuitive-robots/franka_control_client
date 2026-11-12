import argparse
import logging
import time
from pathlib import Path

from typing import List

import pyzlc

from franka_control_client.core.zlc_heartbeat import patch_pyzlc_heartbeat

from franka_control_client.camera.camera import CameraDevice
# the VR variant records the leader's EE target; irl_data_collection expects GELLO joints
from franka_control_client.data_collection.irl_vr_data_collection import (
    IRLDataCollection,
)
from franka_control_client.data_collection.irl_lerobot_data_collection import (
    IRLLeRobotDataCollection,
)
from franka_control_client.data_collection.irl_wrapper import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
)
from franka_control_client.data_collection.irl_wrapper import (
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
    MQ3DataWrapper,
)
from franka_control_client.franka_robot.franka_panda import (
    RemotePandaArm,
)

from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.control_pair.mq3_panda_control_pair import (
    MQ3PandaControlPair,
)
from franka_control_client.vr.webxr_buttons import WebXRButtons
from franka_control_client.vr.webxr_controller import WebXRController, WorkspaceBounds
from franka_control_client.vr.webxr_server import WebXRServer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Quest 3 WebXR teleoperation data collection (DROID setup)")
    parser.add_argument("--task", default="pick_up_knife", help="Task name, recordings go to <data-dir>/<task>")
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=Path("/home/irl-admin/new_data_collection"),
        help="Recordings go to <data-dir>/<task>",
    )
    parser.add_argument("--yaw", type=float, default=0.0, help="Operator heading in the base frame, deg CCW from +X")
    parser.add_argument("--host", default="127.0.0.1", help="WebXR server address (adb reverse targets 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--bounds",
        type=float,
        nargs=6,
        metavar=("XMIN", "YMIN", "ZMIN", "XMAX", "YMAX", "ZMAX"),
        help="EE target workspace box in the base frame (m)",
    )
    parser.add_argument(
        "--stop-after-episode",
        action="store_true",
        help="Stop teleop (arm Idle, gripper opens) when an episode is saved/discarded; by default it keeps running",
    )
    rec = parser.add_argument_group("recording (LeRobot dataset in <data-dir>/lerobot/<task>, resumed if it exists)")
    rec.add_argument("--fps", type=int, default=50, help="Recording rate for robot state, action and cameras")
    rec.add_argument("--instruction", help="Language instruction stored with every frame (default: task name)")
    rec.add_argument("--image-size", type=int, nargs=2, default=[256, 256], metavar=("W", "H"),
                     help="Stored camera size; 0 0 keeps the camera resolution")
    rec.add_argument("--action-offset", type=int, default=1, help="action[t] = leader command at t + offset")
    rec.add_argument("--vcodec", default="libsvtav1",
                     help="libsvtav1 (LeRobot default), h264, hevc, h264_nvenc or auto (hardware if available)")
    rec.add_argument("--encoder-threads", type=int, help="Threads per camera encoder (default: codec default)")
    rec.add_argument("--raw", action="store_true",
                     help="Old format instead: per-episode .pt files + PNGs in <data-dir>/<task>, convert later")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="[webxr] %(message)s")
    # pyzlc stops heartbeating for good on one send error; keep it alive across network blips
    patch_pyzlc_heartbeat()
    pyzlc.init(
        "data_collection",
        "192.168.1.1",
        group_name="DroidGroup",
        group_port=7730,
    )
    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    # WebXRController seeds its target from the current EE pose
    for _ in range(50):
        if follower.panda_arm.current_state is not None:
            break
        time.sleep(0.1)
    bounds = WorkspaceBounds(tuple(args.bounds[:3]), tuple(args.bounds[3:])) if args.bounds else WorkspaceBounds()
    leader = WebXRController(
        "webxr",
        follower.panda_arm,
        server=WebXRServer(args.host, args.port),
        yaw_deg=args.yaw,
        bounds=bounds,
    )
    pyzlc.info(f"WebXR teleop: yaw {args.yaw} deg, bounds {bounds.lower}..{bounds.upper}")
    control_pair = MQ3PandaControlPair(leader, follower)
    #camera frequency controlled individually
    #cams 25hz 0.04
    #cams 40hz 0.025
    camera_left = ImageDataWrapper(CameraDevice("zed_left", preview=False),capture_interval=0.04,hw_name="zed_left")
    camera_right = ImageDataWrapper(CameraDevice("zed_right", preview=False),capture_interval=0.04,hw_name="zed_right")
    camera_wrist = ImageDataWrapper(CameraDevice("zed_wrist", preview=False),capture_interval=0.04,hw_name="zed_wrist")
    data_collectors: List[IRL_HardwareDataWrapper] = []
    data_collectors.append(camera_left)
    data_collectors.append(camera_right)
    data_collectors.append(camera_wrist)
    data_collectors.append(MQ3DataWrapper(leader, hw_name="webxr"))
    data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))
    task = args.task
    if args.raw:
        data_collection_manager = IRLDataCollection(
            data_collectors,
            str(args.data_dir / task),
            task,
            fps=args.fps,
            control_pair=control_pair,
            stop_control_on_episode_end=args.stop_after_episode,
        )
    else:
        data_collection_manager = IRLLeRobotDataCollection(
            data_collectors,
            args.data_dir / "lerobot" / task,
            args.instruction or task.replace("_", " "),
            control_pair=control_pair,
            fps=args.fps,
            repo_id=task,
            image_size=tuple(args.image_size) if all(args.image_size) else None,
            action_offset=args.action_offset,
            vcodec=args.vcodec,
            encoder_threads=args.encoder_threads,
            stop_control_on_episode_end=args.stop_after_episode,
        )
    # Quest buttons, same as the keys: A 'n' start, B 's' save, X 'x' delete running/last episode, Y 'r' home
    data_collection_manager.register_input_source(WebXRButtons(leader.server).poll)
    pyzlc.info("WebXR: A = start episode, B = save, X = delete running/last episode, Y = home robot")
    control_pair.control_reset()
    data_collection_manager.run()
    pyzlc.shutdown()
