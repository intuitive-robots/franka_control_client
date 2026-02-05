import abc
import numpy as np
from typing import Dict

from ..camera.camera import CameraDevice
from ..core.latest_msg_subscriber import LatestMsgSubscriber
from ..franka_robot.panda_arm import RemotePandaArm
from ..franka_robot.panda_gripper import RemotePandaGripper
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper


class HardwareDataWrapper(abc.ABC):

    def __init__(self, feature: dict) -> None:
        self.feature = feature

    @abc.abstractmethod
    def capture_step(self) -> Dict[str, np.ndarray]:
        raise NotImplementedError(
            "Subclasses must implement capture_step method."
        )

    @abc.abstractmethod
    def discard(self) -> None:
        raise NotImplementedError("Subclasses must implement discard method.")

    @abc.abstractmethod
    def reset(self) -> None:
        raise NotImplementedError("Subclasses must implement reset method.")

    @abc.abstractmethod
    def close(self) -> None:
        raise NotImplementedError("Subclasses must implement close method.")


class ImageDataWrapper(HardwareDataWrapper):
    def __init__(self, camera_device: CameraDevice) -> None:
        self.camera_device = camera_device
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
        return {self.key: image_data}

    def discard(self) -> None:
        # Implement the logic to discard the captured image data if needed
        pass

    def reset(self) -> None:
        # Implement the logic to reset the camera device if needed
        pass

    def close(self) -> None:
        # Implement the logic to close the camera device if needed
        pass


class PandaArmDataWrapper(HardwareDataWrapper):
    def __init__(self, arm: RemotePandaArm) -> None:
        self.arm = arm
        self.key = f"observation.state.q.{arm._name}"
        feature = {
            self.key: {"dtype": "float32", "shape": (7,)},
        }
        super().__init__(feature)

    def capture_step(self) -> Dict[str, np.ndarray]:
        # Implement the logic to save robot state data
        state = self.arm.current_state
        if state is None:
            raise ValueError("No arm state data received from the robot.")
        return {self.key: np.array(state["q"], dtype=np.float32)}

    def __getattr__(self, name):
        return getattr(self.arm, name)

    def discard(self) -> None:
        pass

    def reset(self) -> None:
        pass

    def close(self) -> None:
        pass


class RobotiqGripperDataWrapper(HardwareDataWrapper):
    def __init__(self, gripper: RemoteRobotiqGripper) -> None:
        self.gripper = gripper
        self.key = f"observation.state.robotiq_gripper.{gripper._name}"
        feature = {self.key: {"dtype": "float32", "shape": (2,)}}
        super().__init__(feature)

    def capture_step(self) -> Dict[str, np.ndarray]:
        state = self.gripper.current_state
        if state is None:
            raise ValueError("No Robotiq gripper state data received.")
        data = np.array(
            [
                # state["commanded_position"],
                # state["commanded_speed"],
                # state["commanded_force"],
                state["position"],
                state["current"],
                # state["raw_commanded_position"],
                # state["raw_position"],
            ],
            dtype=np.float32,
        )
        return {self.key: data}

    def discard(self) -> None:
        pass

    def reset(self) -> None:
        pass

    def close(self) -> None:
        pass

    def __getattr__(self, name):
        return getattr(self.gripper, name)


class GelloArmDataWrapper(HardwareDataWrapper):
    def __init__(self, name: str = "gello") -> None:
        self._name = name
        self.arm_state_sub = LatestMsgSubscriber(f"{name}/gello_arm_state")
        self.key = f"action.joint_position.{name}"
        feature = {self.key: {"dtype": "float32", "shape": (7,)}}
        super().__init__(feature)

    def capture_step(self) -> Dict[str, np.ndarray]:
        state = self.arm_state_sub.get_latest()
        if state is None:
            raise ValueError("No Gello arm state data received.")
        joints = np.asarray(state["joint_state"], dtype=np.float32).reshape(-1)
        if joints.size != 7:
            raise ValueError(
                f"Expected 7 Gello arm joints, got {joints.size}."
            )
        return {self.key: joints}

    def discard(self) -> None:
        pass

    def reset(self) -> None:
        pass

    def close(self) -> None:
        pass


class GelloGripperDataWrapper(HardwareDataWrapper):
    def __init__(self, name: str = "gello") -> None:
        self._name = name
        self.gripper_state_sub = LatestMsgSubscriber(
            f"{name}/gello_gripper_state"
        )
        self.key = f"action.gripper.{name}"
        feature = {self.key: {"dtype": "float32", "shape": (1,)}}
        super().__init__(feature)

    def capture_step(self) -> Dict[str, np.ndarray]:
        state = self.gripper_state_sub.get_latest()
        if state is None:
            raise ValueError("No Gello gripper state data received.")
        gripper = np.asarray(state["gripper"], dtype=np.float32).reshape(-1)
        if gripper.size != 1:
            raise ValueError(
                f"Expected 1 Gello gripper value, got {gripper.size}."
            )
        return {self.key: gripper}

    def discard(self) -> None:
        pass

    def reset(self) -> None:
        pass

    def close(self) -> None:
        pass
