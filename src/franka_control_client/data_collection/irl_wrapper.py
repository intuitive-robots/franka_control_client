import abc
import time
from typing import Dict, Optional

import cv2
import numpy as np

from ..camera.camera import CameraDevice
from ..core.latest_msg_subscriber import LatestMsgSubscriber
from ..franka_robot.panda_arm import RemotePandaArm
from ..franka_robot.panda_gripper import RemotePandaGripper
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper
from ..gello.gello import RemoteGello

class IRL_HardwareDataWrapper(abc.ABC):

    def __init__(self, hw_type:str,hw_name:str) -> None:
        self.hw_type = hw_type
        self.hw_name = hw_name

    @abc.abstractmethod
    def capture_step(self) -> Optional[np.ndarray]:
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


class ImageDataWrapper(IRL_HardwareDataWrapper):
    def __init__(self, camera_device: CameraDevice,hw_name:str , hw_type:str = "camera",capture_interval:int = 0.033) -> None:
        self.camera_device = camera_device
        self.capture_interval = capture_interval
        super().__init__(hw_type,hw_name)

    def capture_step(self) -> Optional[np.ndarray]:
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

    def discard(self) -> None:
        # Implement the logic to discard the captured image data if needed
        pass

    def reset(self) -> None:
        # Implement the logic to reset the camera device if needed
        pass

    def close(self) -> None:
        # Implement the logic to close the camera device if needed
        pass


class PandaArmDataWrapper(IRL_HardwareDataWrapper):
    def __init__(self, arm: RemotePandaArm,hw_name:str = "FrankaPanda" , hw_type:str = "follower_arm") -> None:
        self.arm = arm
        super().__init__(hw_type,hw_name)

    def capture_step(self) -> Dict[str, np.ndarray]:
        # Implement the logic to save robot state data
        state = self.arm.current_state
        if state is None:
            raise ValueError("No arm state data received from the robot.")
        # data = np.array(
        #     [
        #         state["q"]
        #         #can extract here
        #     ],
        #     dtype=np.float32,
        # )
        return state

    def __getattr__(self, name):
        return getattr(self.arm, name)

    def discard(self) -> None:
        pass

    def reset(self) -> None:
        pass

    def close(self) -> None:
        pass


class PandaGripperDataWrapper(IRL_HardwareDataWrapper):
    def __init__(self, gripper: RemotePandaGripper,hw_name:str = "FrankaPanda" , hw_type:str = "follower_gripper") -> None:
        self.gripper = gripper
        super().__init__(hw_type,hw_name)

    def capture_step(self) -> Dict[str, np.ndarray]:
        state = self.gripper.current_state
        if state is None:
            raise ValueError("No gripper state data received from the robot.")
        # data = np.array(
        #     [
        #         state["width"]
        #     ],
        #     dtype=np.float32,
        # )
        return state

    def discard(self) -> None:
        pass

    def reset(self) -> None:
        pass

    def close(self) -> None:
        pass


class RobotiqGripperDataWrapper(IRL_HardwareDataWrapper):
    def __init__(self, gripper: RemoteRobotiqGripper,hw_name:str = "FrankaPanda" , hw_type:str = "follower_gripper") -> None:
        self.gripper = gripper
        super().__init__(hw_type,hw_name)

    def capture_step(self) -> Dict[str, np.ndarray]:
        state = self.gripper.current_state
        if state is None:
            raise ValueError("No Robotiq gripper state data received.")
        # data = np.array(
        #     [
        #         # state["commanded_position"],
        #         # state["commanded_speed"],
        #         # state["commanded_force"],
        #         state["position"],
        #         # state["current"],
        #         # state["raw_commanded_position"],
        #         # state["raw_position"],
        #     ],
        #     dtype=np.float32,
        # )
        return state

    def discard(self) -> None:
        pass

    def reset(self) -> None:
        pass

    def close(self) -> None:
        pass

    def __getattr__(self, name):
        return getattr(self.gripper, name)

class GelloDataWrapper(IRL_HardwareDataWrapper):
    def __init__(self, robot: RemoteGello,hw_name:str = "Gello" , hw_type:str = "leader_robot") -> None:
        self.robot=robot
        self.hw_name=hw_name
        super().__init__(hw_type,hw_name)

    def capture_step(self) -> Dict[str, np.ndarray]:
        state = self.robot.current_state
        arm_state = self.robot.current_state["gello_arm_state"]
        gripper_state = self.robot.current_state["gello_gripper_state"]
        if arm_state is None:
            raise ValueError("No Gello arm state data received.")
        # joints = np.asarray(arm_state["joint_state"], dtype=np.float32).reshape(
        #     -1
        # )
        # if joints.size != 7:
        #     raise ValueError(
        #         f"Expected 7 Gello arm joints, got {joints.size}."
        #     )
        if gripper_state is None:
            raise ValueError("No Gello gripper state data received.")
        # gripper = np.asarray(
        #     gripper_state["gripper"], dtype=np.float32
        # ).reshape(-1)
        # if gripper.size != 1:
        #     raise ValueError(
        #         f"Expected 1 Gello gripper value, got {gripper.size}."
        #     )
        return state

    def discard(self) -> None:
        pass

    def reset(self) -> None:
        pass

    def close(self) -> None:
        pass
