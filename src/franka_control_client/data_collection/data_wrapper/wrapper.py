import abc
import numpy as np
from typing import Dict

from ...camera.camera import CameraDevice
from ...franka_robot.panda_arm import RemotePandaArm, PandaArmState
from ...franka_robot.panda_gripper import RemotePandaGripper, PandaGripperState
from ...robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
    RobotiqGripperState,
)
from ...gello.gello import RemoteGello, GelloState


class HardwareDataWrapper(abc.ABC):

    @abc.abstractmethod
    def capture_step(self) -> Dict[str, np.ndarray]:
        raise NotImplementedError(
            "Subclasses must implement capture_step method."
        )

    def discard(self) -> None:
        # Default implementation does nothing, can be overridden by subclasses if needed
        pass

    def reset(self) -> None:
        # Default implementation does nothing, can be overridden by subclasses if needed
        pass

    def close(self) -> None:
        # Default implementation does nothing, can be overridden by subclasses if needed
        pass


class ImageDataWrapper(HardwareDataWrapper):
    def __init__(self, camera_device: CameraDevice) -> None:
        self.camera_device = camera_device
        super().__init__()

    def capture_step(self) -> np.ndarray:
        # Implement the logic to save image data from the camera device
        image_data = self.camera_device.get_image()
        # check if image_data is None and data side shape
        if image_data is None:
            raise ValueError("No image data received from camera device.")
        if image_data.shape != (
            self.camera_device.size[0],
            self.camera_device.size[1],
            3,
        ):
            raise ValueError(
                f"Unexpected image shape: expected "
                f"({self.camera_device.size[0]}, {self.camera_device.size[1]}, 3), "
                f"got {image_data.shape}"
            )
        return image_data


class PandaArmDataWrapper(HardwareDataWrapper):
    def __init__(self, arm: RemotePandaArm) -> None:
        self.arm = arm
        super().__init__()

    def capture_step(self) -> PandaArmState:
        state = self.arm.current_state
        if state is None:
            raise ValueError("No arm state data received from the robot.")
        return state


class PandaGripperDataWrapper(HardwareDataWrapper):
    def __init__(self, gripper: RemotePandaGripper) -> None:
        self.gripper = gripper
        super().__init__()

    def capture_step(self) -> PandaGripperState:
        state = self.gripper.current_state
        if state is None:
            raise ValueError("No gripper state data received from the robot.")
        return state


class RobotiqGripperDataWrapper(HardwareDataWrapper):
    def __init__(self, gripper: RemoteRobotiqGripper) -> None:
        self.gripper = gripper
        super().__init__()

    def capture_step(self) -> RobotiqGripperState:
        state = self.gripper.current_state
        if state is None:
            raise ValueError("No Robotiq gripper state data received.")
        return state


class GelloDataWrapper(HardwareDataWrapper):
    def __init__(self, gello: RemoteGello) -> None:
        self.gello = gello
        super().__init__()

    def capture_step(self) -> GelloState:
        state = self.gello.current_state
        if state is None:
            raise ValueError("No Gello state data received.")
        arm_state = state["gello_arm_state"]
        gripper_state = state["gello_gripper_state"]
        if arm_state is None:
            raise ValueError("No Gello arm state data received.")
        if gripper_state is None:
            raise ValueError("No Gello gripper state data received.")
        return state
