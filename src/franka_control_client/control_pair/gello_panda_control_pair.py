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
# How far the leader trigger has to move away from its value at episode start
# before the follower gripper starts following it again (hold_gripper_on_start).
GRIPPER_ENGAGE_THRESHOLD: float = 0.05

class GelloPandControlPair(ControlPair):
    def __init__(
        self,
        leader: RemoteGello,
        follower: PandaRobotiq,
        hold_gripper_on_start: bool = False,
    ) -> None:
        """
        Args:
            hold_gripper_on_start: keep the follower gripper where it is when an
                episode starts instead of immediately jumping to the leader
                trigger value. The gripper starts following the leader again as
                soon as the trigger is moved by GRIPPER_ENGAGE_THRESHOLD.
        """
        super().__init__()
        self.leader = leader
        self.follower = follower
        self.hold_gripper_on_start = hold_gripper_on_start
        self._last_gripper_cmd: Optional[float] = None
        self._gripper_engaged: bool = True
        self._hold_reference_gripper: Optional[float] = None
    
    def control_reset(self)-> None:
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
        if leader_arm_state is not None:
            self.follower.panda_arm.send_joint_position_command(
                np.asarray(leader_arm_state["joint_state"], dtype=np.float64).reshape(-1)
            )
            self._follow_leader_gripper()
        pyzlc.sleep(1/CONTROL_HZ) #todo:need to be smarter to control frequency 

    def _follow_leader_gripper(self) -> None:
        leader_gripper_state = self.leader.current_state["gello_gripper_state"]
        if leader_gripper_state is None:
            return
        gripper_value = np.asarray(
            leader_gripper_state["gripper"], dtype=np.float64
        ).reshape(-1)
        if gripper_value.size < 1:
            return
        gripper_cmd = float(np.clip(gripper_value[0], 0.0, 1.0))
        if not self._gripper_engaged and not self._engage_gripper(gripper_cmd):
            # Leave the gripper where it was when the episode started.
            return
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

    def _engage_gripper(self, gripper_cmd: float) -> bool:
        """Return True once the leader trigger has been moved far enough."""
        if self._hold_reference_gripper is None:
            self._hold_reference_gripper = gripper_cmd
            return False
        if (
            abs(gripper_cmd - self._hold_reference_gripper)
            <= GRIPPER_ENGAGE_THRESHOLD
        ):
            return False
        self._gripper_engaged = True
        pyzlc.info("Gello trigger moved, gripper follows the leader again.")
        return True

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
            # self.control_reset()
            # pyzlc.sleep(1)
            if self.hold_gripper_on_start:
                # Keep the gripper as it is until the leader trigger is moved.
                self._gripper_engaged = False
                self._hold_reference_gripper = None
                self._last_gripper_cmd = None
                pyzlc.info(
                    "Holding current gripper state until the Gello trigger is moved."
                )
            self.follower.panda_arm.set_franka_arm_control_mode(CONTROL_MODE)
            while self.is_running:
                self.control_step()
            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")

    def close_gripper(self) -> None:
        self.follower.robotiq_gripper.send_grasp_command(
            position=1.0,
            speed=GRIPPER_SPEED,
            force=GRIPPER_FORCE,
            blocking=False,
        )

    def open_gripper(self) -> None:
        self.follower.robotiq_gripper.send_grasp_command(
            position=0.0,
            speed=GRIPPER_SPEED,
            force=GRIPPER_FORCE,
            blocking=False,
        )