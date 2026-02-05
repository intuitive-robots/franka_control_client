import pyzlc

from .control_pair import ControlPair
from ..franka_robot.franka_panda import FrankaPanda


GRIPPER_SPEED = 0.5
GRIPPER_THRESHOLD = 0.05
FOLLOWER_GRIPPER_CLOSE = 0.025
FOLLOWER_GRIPPER_OPEN = 0.07


class SinglePandaKTControlPair(ControlPair):
    def __init__(self, leader: FrankaPanda, follower: FrankaPanda) -> None:
        self.leader = leader
        self.follower = follower

    def control_step(self) -> None:
        leader_arm_state = self.leader.panda_arm.current_state
        if leader_arm_state is None:
            return
        self.follower.panda_arm.send_joint_position_command(
            leader_arm_state["q"]
        )
        leader_gripper_state = self.leader.panda_gripper.current_state
        if leader_gripper_state is not None:
            w = float(leader_gripper_state["width"])
            desired_w = (
                FOLLOWER_GRIPPER_CLOSE
                if w < GRIPPER_THRESHOLD
                else FOLLOWER_GRIPPER_OPEN
            )
            self.follower.panda_gripper.send_gripper_command(
                width=desired_w, speed=GRIPPER_SPEED
            )
            pyzlc.sleep(0.001)
