import numpy as np
from typing import Dict, Any

from ...camera.camera import CameraDevice
from ...franka_robot.panda_arm import RemotePandaArm
from ...franka_robot.panda_gripper import RemotePandaGripper
from ...robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper
from ...gello.gello import RemoteGello
from .wrapper import (
    HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    PandaGripperDataWrapper,
    RobotiqGripperDataWrapper,
    GelloDataWrapper,
)


class LeRobotDataWrapper(HardwareDataWrapper):

    def __init__(self, feature: Dict[str, Dict[str, Any]]):
        self.feature = feature
        super().__init__()


class LeRobotImageDataWrapper(LeRobotDataWrapper):
    def __init__(self, camera_device: CameraDevice) -> None:
        self.raw_wrapper = ImageDataWrapper(camera_device)
        self.key = f"observation.image.{camera_device._name}"
        feature = {
            self.key: {
                "dtype": "video",
                "shape": (camera_device.size[0], camera_device.size[1], 3),
            },
        }
        super().__init__(feature)

    def capture_step(self) -> Dict[str, np.ndarray]:
        # Implement the logic to save image data from the camera device
        return {self.key: self.raw_wrapper.capture_step()}


class LeRobotPandaArmDataWrapper(LeRobotDataWrapper):
    def __init__(self, arm: RemotePandaArm) -> None:
        self.raw_wrapper = PandaArmDataWrapper(arm)
        self.key = f"observation.state.q.{arm._name}"
        feature = {
            self.key: {"dtype": "float32", "shape": (7,)},
        }
        super().__init__(feature)

    def capture_step(self) -> Dict[str, np.ndarray]:
        # Implement the logic to save robot state data
        state = self.raw_wrapper.capture_step()
        return {self.key: np.array(state["q"], dtype=np.float32)}


class LeRobotPandaGripperDataWrapper(LeRobotDataWrapper):
    def __init__(self, gripper: RemotePandaGripper) -> None:
        self.raw_wrapper = PandaGripperDataWrapper(gripper)
        self.key = f"observation.state.gripper_width.{gripper._name}"
        feature = {
            self.key: {"dtype": "float32", "shape": (1,)},
        }
        super().__init__(feature)

    def capture_step(self) -> Dict[str, np.ndarray]:
        state = self.raw_wrapper.capture_step()
        if state is None:
            raise ValueError("No gripper state data received from the robot.")
        return {self.key: np.array([state["width"]], dtype=np.float32)}


class LeRobotRobotiqGripperDataWrapper(LeRobotDataWrapper):
    def __init__(self, gripper: RemoteRobotiqGripper) -> None:
        self.raw_wrapper = RobotiqGripperDataWrapper(gripper)
        self.key = f"observation.state.robotiq_gripper.{gripper._name}"
        feature = {
            self.key: {"dtype": "float32", "shape": (2,)},
        }
        super().__init__(feature)

    def capture_step(self) -> Dict[str, np.ndarray]:
        state = self.raw_wrapper.capture_step()
        if state is None:
            raise ValueError("No Robotiq gripper state data received.")
        data = np.array(
            [
                state["position"],
                state["current"],
            ],
            dtype=np.float32,
        )
        return {self.key: data}


class LeRobotGelloDataWrapper(LeRobotDataWrapper):
    def __init__(self, gello: RemoteGello) -> None:
        self.raw_wrapper = GelloDataWrapper(gello)
        self.arm_key = f"action.gripper.gello_arm.{gello._name}"
        self.gripper_key = f"action.gripper.gello_gripper.{gello._name}"
        feature = {
            self.arm_key: {"dtype": "float32", "shape": (7,)},
            self.gripper_key: {"dtype": "float32", "shape": (1,)},
        }
        super().__init__(feature)

    def capture_step(self) -> Dict[str, np.ndarray]:
        state = self.raw_wrapper.capture_step()
        if state is None:
            raise ValueError("No Gello state data received.")
        arm_state = state["gello_arm_state"]
        gripper_state = state["gello_gripper_state"]
        if arm_state is None:
            raise ValueError("No Gello arm state data received.")
        joints = np.asarray(arm_state, dtype=np.float32).reshape(-1)
        if joints.size != 7:
            raise ValueError(
                f"Expected 7 Gello arm joints, got {joints.size}."
            )
        if gripper_state is None:
            raise ValueError("No Gello gripper state data received.")
        gripper = np.asarray(gripper_state, dtype=np.float32).reshape(-1)
        if gripper.size != 1:
            raise ValueError(
                f"Expected 1 Gello gripper value, got {gripper.size}."
            )

        return {self.arm_key: joints, self.gripper_key: gripper}
