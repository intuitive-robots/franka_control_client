"""Kinesthetic teaching + replay data collection (DROID setup, Panda + Robotiq + 3 ZED cameras).

1. The arm floats in hand-guiding mode (proxy mode HumanControl). Press space to record a demo,
   guide the arm, use o/p to open/close the gripper, press space again to save it.
2. Let go of the arm and press space: the robot moves to the demo start pose and replays the demo
   with its original timing in a joint impedance mode. Robot state and cameras are recorded again
   and the replay is saved automatically when it ends (+ --settle). Replay as often as you like.
3. Press t to teach a new demo from where the arm is, or h to go home first.

Data: <data-dir>/<task>/demo/<ts>/ and <data-dir>/<task>/replay/<ts>/ (see teach_replay_collection.py).
Before the first run check the end-effector load in Desk (pixi run ee-load-check), otherwise
the floating arm drifts up or down.
"""

import argparse
from pathlib import Path
from typing import List

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.data_collection.irl_wrapper import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
)
from franka_control_client.data_collection.teach_replay_collection import (
    Demo,
    TeachReplayCollection,
)
from franka_control_client.franka_robot.franka_panda import RemotePandaArm
from franka_control_client.franka_robot.panda_arm import ControlMode
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper

REPLAY_MODES = {
    "stiff": ControlMode.HybridJointImpedance,
    "compliant": ControlMode.HybridJointImpedanceCompliant,
}
CAMERAS = ("zed_left", "zed_right", "zed_wrist")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", default="kinesthetic_task")
    parser.add_argument("--data-dir", type=Path, default=Path("/home/irl-admin/new_data_collection"), help="Recordings go to <data-dir>/<task>")
    parser.add_argument("--robot-name", default="FrankaPanda")
    parser.add_argument("--local-ip", default="192.168.1.1")
    parser.add_argument("--group-name", default="DroidGroup")
    parser.add_argument("--group-port", type=int, default=7730)
    parser.add_argument("--fps", type=int, default=100, help="Robot state recording rate [Hz]")
    parser.add_argument("--camera-hz", type=float, default=25.0, help="Camera recording rate [Hz]")
    parser.add_argument("--no-cameras", action="store_true", help="Record robot state only")
    parser.add_argument(
        "--replay-mode",
        choices=list(REPLAY_MODES),
        default="stiff",
        help="stiff: HybridJointImpedance, compliant: HybridJointImpedanceCompliant (contact-rich tasks)",
    )
    parser.add_argument("--replay-hz", type=float, default=200.0, help="Command rate during replay [Hz]")
    parser.add_argument(
        "--no-feedforward", action="store_true", help="Replay positions only (no recorded joint velocity)"
    )
    parser.add_argument("--settle", type=float, default=1.0, help="Keep recording after the replay ends [s]")
    parser.add_argument("--demo", type=Path, help="Replay an existing demo episode dir instead of teaching first")
    parser.add_argument(
        "--push-force",
        type=float,
        default=0.0,
        help="Insertion force [N] along the tool z axis around the deepest demo point, 0 = off "
        "(position replay alone pushes only stiffness x tracking error; proxy caps at max_ee_force)",
    )
    parser.add_argument(
        "--push-zone", type=float, default=10.0, help="Push from this distance before the deepest point [mm]"
    )
    parser.add_argument("--push-hold", type=float, default=0.5, help="Keep pushing after the deepest point [s]")
    parser.add_argument("--push-ramp", type=float, default=0.3, help="Force ramp in/out time [s]")
    args = parser.parse_args()

    pyzlc.init("teach_and_replay", args.local_ip, group_name=args.group_name, group_port=args.group_port)
    robot = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm(args.robot_name),
        RemoteRobotiqGripper(args.robot_name),
    )

    data_collectors: List[IRL_HardwareDataWrapper] = []
    if not args.no_cameras:
        for cam in CAMERAS:
            data_collectors.append(
                ImageDataWrapper(CameraDevice(cam, preview=False), capture_interval=1.0 / args.camera_hz, hw_name=cam)
            )
    data_collectors.append(PandaArmDataWrapper(robot.panda_arm, hw_name=args.robot_name))
    data_collectors.append(RobotiqGripperDataWrapper(robot.robotiq_gripper, hw_name=args.robot_name))

    session = TeachReplayCollection(
        robot,
        data_collectors,
        args.data_dir / args.task,
        args.task,
        fps=args.fps,
        replay_mode=REPLAY_MODES[args.replay_mode],
        replay_rate_hz=args.replay_hz,
        velocity_feedforward=not args.no_feedforward,
        settle_time=args.settle,
        demo=Demo.load(args.demo, args.robot_name) if args.demo else None,
        push_force=args.push_force,
        push_zone=args.push_zone / 1000.0,
        push_hold=args.push_hold,
        push_ramp=args.push_ramp,
    )
    session.run()
    pyzlc.shutdown()


if __name__ == "__main__":
    main()
