import traceback

from .control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode
from ..franka_robot.panda_robotiq import PandaRobotiq
from ..vr.control_signal import EEControlSource

GRIPPER_SPEED = 0.7
GRIPPER_FORCE = 0.3
CONTROL_HZ: float = 500
GRIPPER_DEADBAND: float = 1e-3


class MQ3PandaControlPair(ControlPair):
    def __init__(self, leader: EEControlSource, follower: PandaRobotiq) -> None:
        super().__init__()
        self.leader = leader
        self.follower = follower

    def control_step(self) -> None:
        control_signal = self.leader.current_control_signal
        if control_signal is None:
            return
        desired_position = control_signal["pos"]
        desired_rot = control_signal["rot"]
        self.follower.panda_arm.send_cartesian_pose_command(
            pos=desired_position, rot=desired_rot
        )
        self.follower.robotiq_gripper.send_grasp_command(
            position=control_signal["gripper_width"],
            speed=GRIPPER_SPEED,
            force=GRIPPER_FORCE,
            blocking=False,
        )

    def control_reset(self) -> None:
        # No special reset needed for MQ3 control, but we can reset the gripper command state
        self.follower.panda_arm.set_franka_arm_control_mode(
            ControlMode.CartesianImpedance
        )
        self.follower.robotiq_gripper.open()
        self.follower.panda_arm.move_franka_arm_to_joint_position(
            [0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0]
        )
        self.leader.reset()

    def control_end(self) -> None:
        self.follower.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)
        self.follower.robotiq_gripper.send_grasp_command(
            position=0.0,
            speed=GRIPPER_SPEED,
            force=GRIPPER_FORCE,
            blocking=True,
        )

    def _control_task(self) -> None:
        try:
            # pyzlc.info("Resetting...")
            # self.control_reset()
            # pyzlc.sleep(1)
            self.follower.panda_arm.set_franka_arm_control_mode(ControlMode.CartesianImpedance)
            while self.is_running:
                self.control_step()
            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
            traceback.print_exc()