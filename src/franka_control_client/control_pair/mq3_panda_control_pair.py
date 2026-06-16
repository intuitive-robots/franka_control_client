import traceback
import pyzlc

from .control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode
from ..franka_robot.panda_robotiq import PandaRobotiq
from ..vr.meta_quest3 import MQ3Controller

GRIPPER_SPEED = 0.7
GRIPPER_FORCE = 0.3
CONTROL_HZ: float = 500
GRIPPER_DEADBAND: float = 1e-3
DEFAULT_POSITION = [0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0]
DEFAULT_POSITION2 = [-0.30406295828651964, -0.10313557727047261, -0.17526436078783472, -2.1124992416113964, -0.03802868800288364, 2.001340827254022, -0.46061141536409717]

class MQ3PandaControlPair(ControlPair):
    def __init__(self, leader: MQ3Controller, follower: PandaRobotiq) -> None:
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
            DEFAULT_POSITION2
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
                pyzlc.sleep(1.0 / CONTROL_HZ)
            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
            traceback.print_exc()