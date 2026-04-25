import pyzlc

from .control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode
from ..franka_robot.panda_robotiq import PandaRobotiq


CONTROL_HZ: float = 100
CONTROL_MODE: ControlMode = ControlMode.HumanControl
GRIPPER_SPEED = 0.7
GRIPPER_FORCE = 0.3
DEFAULT_POSITION = (0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0)


class HumanPandaControlPair(ControlPair):
    """Human hand-guided control for a single Panda follower."""

    def __init__(self, follower: PandaRobotiq) -> None:
        super().__init__()
        self.follower = follower

    def control_reset(self) -> None:
        self.go_home()
        self.follower.panda_arm.set_franka_arm_control_mode(CONTROL_MODE)

    def go_home(self) -> None:
        self.follower.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)
        self.follower.panda_arm.move_franka_arm_to_joint_position(DEFAULT_POSITION)
        self.follower.robotiq_gripper.send_grasp_command(
            position=0.0,
            speed=GRIPPER_SPEED,
            force=GRIPPER_FORCE,
            blocking=True,
        )

    def open_gripper(self) -> None:
        self.follower.robotiq_gripper.send_grasp_command(
            position=0.0,
            speed=GRIPPER_SPEED,
            force=GRIPPER_FORCE,
            blocking=False,
        )

    def close_gripper(self) -> None:
        self.follower.robotiq_gripper.send_grasp_command(
            position=1.0,
            speed=GRIPPER_SPEED,
            force=GRIPPER_FORCE,
            blocking=False,
        )

    def control_step(self) -> None:
        pyzlc.sleep(1 / CONTROL_HZ)

    def control_end(self) -> None:
        self.follower.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)

    def _control_task(self) -> None:
        try:
            self.follower.panda_arm.set_franka_arm_control_mode(CONTROL_MODE)
            while self.is_running:
                self.control_step()
            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
