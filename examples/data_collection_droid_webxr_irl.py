import argparse
import logging
import time
from pathlib import Path

from typing import List

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.data_collection.irl_data_collection import (
    IRLDataCollection,
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
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="[webxr] %(message)s")
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
    data_collection_manager = IRLDataCollection(
        data_collectors,
        str(args.data_dir / task),
        task,
        fps=4,
        control_pair=control_pair
    )
    control_pair.control_reset()
    data_collection_manager.run()
    pyzlc.shutdown()
