import pyzlc

from .control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode
from ..franka_robot.panda_robotiq import PandaRobotiq
from ..gello.gello import RemoteGello
import numpy as np
from typing import Optional
GRIPPER_SPEED = 0.7
GRIPPER_FORCE= 0.3
CONTROL_HZ: float = 500
GRIPPER_DEADBAND: float = 1e-3
CONTROL_MODE: ControlMode = ControlMode.HybridJointImpedance

class GelloPandControlPair(ControlPair):
    def __init__(self, leader: RemoteGello, follower: PandaRobotiq) -> None:
        super().__init__()
        self.leader = leader
        self.follower = follower
        self._last_gripper_cmd: Optional[float] = None
    
    def control_rest(self)-> None:
        leader_arm_state = self.leader.current_state["gello_arm_state"]
        if leader_arm_state is None:
            pyzlc.error(
                "No Gello arm state available for align."
            )
            return
        arm_state = np.asarray(leader_arm_state["joint_state"], dtype=np.float64).reshape(-1)
        self.follower.panda_arm.move_franka_arm_to_joint_position(arm_state)
        # self.follower.panda_arm.set_franka_arm_control_mode(CONTROL_MODE)
        leader_gripper_state = self.leader.current_state["gello_gripper_state"]
        if leader_gripper_state is None:
            return
        gripper_value = np.asarray(
            leader_gripper_state["gripper"], dtype=np.float64
        ).reshape(-1)
        if gripper_value.size < 1:
            return
        gripper_cmd = float(np.clip(gripper_value[0], 0.0, 1.0))
        self.follower.robotiq_gripper.send_grasp_command(
                position=gripper_cmd,
                speed=GRIPPER_SPEED,
                force=GRIPPER_FORCE,
                blocking=True,
            )

    def control_step(self) -> None:
        leader_arm_state = self.leader.current_state["gello_arm_state"]
        if leader_arm_state is None:
            return
        self.follower.panda_arm.send_joint_position_command(
            np.asarray(leader_arm_state["joint_state"], dtype=np.float64).reshape(-1)
        )
        leader_gripper_state = self.leader.current_state["gello_gripper_state"]
        if leader_gripper_state is None:
            return
        gripper_value = np.asarray(
            leader_gripper_state["gripper"], dtype=np.float64
        ).reshape(-1)
        if gripper_value.size < 1:
            return
        gripper_cmd = float(np.clip(gripper_value[0], 0.0, 1.0))
        if (
            self._last_gripper_cmd is None
            or abs(gripper_cmd - self._last_gripper_cmd)
            > GRIPPER_DEADBAND
        ):
            self.follower.robotiq_gripper.send_grasp_command(
                position=gripper_cmd,
                speed=GRIPPER_SPEED,
                force=GRIPPER_FORCE,
                blocking=False,
            )
            self._last_gripper_cmd = gripper_cmd
        pyzlc.sleep(1/CONTROL_HZ) #todo:need to be smarter to control frequency 

    def control_end(self) -> None:
        self.follower.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)
        self.follower.robotiq_gripper.send_grasp_command(
            position=0.0,
            speed=GRIPPER_SPEED,
            force=GRIPPER_FORCE,
            blocking=True
            )
        
    def _control_task(self) -> None:
        try:
            # pyzlc.info("Resetting...")
            # self.control_rest()
            # pyzlc.sleep(1)
            self.follower.panda_arm.set_franka_arm_control_mode(CONTROL_MODE)
            while self.is_running:
                self.control_step()
            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
