import pyzlc
from simpub import init_xr_node_manager, MetaQuest3
from simpub.core.utils import ZLC_GROUP_NAME
from scipy.spatial.transform import Rotation as R
from typing import List, Optional, TypedDict
import numpy as np
import threading

from ..core.remote_device import RemoteDevice
from ..franka_robot.panda_arm import RemotePandaArm


class MQ3ControlSignal(TypedDict):
    pos: List[float]
    rot: List[float]
    pos_vel: List[float]
    rot_vel: List[float]
    gripper_width: float


class MQ3Controller(RemoteDevice):
    def __init__(
        self, name: str, node_ip: str, panda_arm: RemotePandaArm
    ) -> None:
        super().__init__(name)
        init_xr_node_manager("MQ3RealRobotControl", node_ip)
        self.panda_arm = panda_arm
        self._state_lock = threading.Lock()
        self.mq3 = MetaQuest3(name)
        self.mq3.register_trigger_press_event(
            "hand_trigger", "right", self.start_control
        )
        self.mq3.register_trigger_release_event(
            "hand_trigger", "right", self.stop_control
        )
        self.on_control = False
        self.base_ee_position: Optional[List[float]] = None
        self.base_ee_rotation: Optional[List[float]] = None
        self.base_mq3_position: Optional[List[float]] = None
        self.base_mq3_rotation: Optional[List[float]] = None
        self.reset()

    def reset(self) -> None:
        self.stop_control()
        panda_state = self.panda_arm.current_state
        if panda_state is None:
            raise ValueError("No initial state data received from the robot.")
        self._current_control_signal: Optional[MQ3ControlSignal] = MQ3ControlSignal(
            pos=panda_state["EE_pos"],
            rot=panda_state["EE_quat"],
            pos_vel=[0.0, 0.0, 0.0],
            rot_vel=[0.0, 0.0, 0.0],
            gripper_width=0.08,
        )

    @property
    def current_control_signal(self) -> Optional[MQ3ControlSignal]:
        with self._state_lock:
            if not self.on_control:
                return self._current_control_signal
            controller_data = self.mq3.get_controller_data()
            if controller_data is None or self.base_mq3_position is None:
                return self._current_control_signal
            right_data = controller_data["right"]
            mq3_position, mq3_rotation = right_data["pos"], right_data["rot"]
            delta_position = np.array(mq3_position) - np.array(
                self.base_mq3_position
            )
            desired_position = np.array(self.base_ee_position) + delta_position
            # print(f"Control: Base MQ3 position: {self.base_mq3_position}, mq3_position: {mq3_position} delta_position: {delta_position}, desired_position: {desired_position}")
            r_mq3 = R.from_quat(mq3_rotation)
            r_mq3_base = R.from_quat(self.base_mq3_rotation)

            # MQ3 relative rotation
            delta_rot = r_mq3 * r_mq3_base.inv()

            r_ee_base = R.from_quat(self.base_ee_rotation)

            # apply delta rotation to EE
            desired_rot = delta_rot * r_ee_base
            # self.panda_arm.send_cartesian_pose_command(
            #     pos=desired_position.tolist(), rot=desired_rot.as_quat().tolist()
            # )
            self._current_control_signal = MQ3ControlSignal(
                pos=desired_position.tolist(),
                rot=desired_rot.as_quat().tolist(),
                pos_vel=[0.0, 0.0, 0.0],
                rot_vel=[0.0, 0.0, 0.0],
                gripper_width=right_data["index_trigger"],
            )
            return self._current_control_signal

    def connect(self) -> None:
        return super().connect()

    def start_control(self) -> None:
        with self._state_lock:
            arm_state = self.panda_arm.current_state
            if arm_state is None:
                raise ValueError("No arm state data received from the robot.")
            controller_data = self.mq3.get_controller_data()
            if controller_data is None:
                raise ValueError("No controller data received from Meta Quest 3.")
            right_data = controller_data["right"]
            self.base_ee_position, self.base_ee_rotation = (
                np.array(arm_state["EE_pos"]),
                np.array(arm_state["EE_quat"]),
            )
            self.base_mq3_position, self.base_mq3_rotation = (
                np.array(right_data["pos"]),
                np.array(right_data["rot"]),
            )
            self.on_control = True

    def stop_control(self) -> None:
        with self._state_lock:
            self.on_control = False
            self.base_ee_position = None
            self.base_ee_rotation = None
            self.base_mq3_position = None
            self.base_mq3_rotation = None
