import time
import os
import sys

# Add hardware directory to path to import GelloAgent
# sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
# from hardware.gello_zlc import GelloAgent

from typing import List, Union

import pyzlc

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.control_pair.mq3_panda_control_pair import (
    MQ3PandaControlPair,
)
from franka_control_client.data_collection.irl_data_collection import (
    IRLDataCollection,
)
from franka_control_client.data_collection.irl_wrapper import (
    IRLDataWrapper,
    ImageDataWrapper,
)
from franka_control_client.data_collection.irl_wrapper import (
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
    GelloDataWrapper,
)
from franka_control_client.franka_robot.franka_panda import (
    RemotePandaArm,
)

from franka_control_client.franka_robot.panda_gripper import RemotePandaGripper
from franka_control_client.gello.gello import RemoteGello
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.control_pair.pil_panda_control_pair import (
    PILPandaControlPair,
    GRIPPER_SPEED,
    GRIPPER_FORCE,
    DEFAULT_CONTROL_HZ,
    GRIPPER_DEADBAND,
    PILMode,
)

import cv2
import torch
from pathlib import Path
import numpy as np
from typing import Optional, Tuple, Any
from scipy.spatial.transform import Rotation as R

from franka_control_client.vr.meta_quest3 import MQ3Controller

import cv2
import pytest
import torch
import numpy as np
import threading
import queue
from pathlib import Path
from typing import Optional, Tuple


class CameraDisplayThread(threading.Thread):
    def __init__(self):
        # daemon=True ensures this thread closes automatically when the main script stops
        super().__init__(daemon=True)
        self.frame_queue = queue.Queue(maxsize=30)
        self.running = True

    def run(self):
        """
        All cv2 UI operations happen inside this single, isolated thread.
        """
        while self.running:
            try:
                # Block for 0.1s to check for new images, then loop back to check self.running
                cam_name, img = self.frame_queue.get(timeout=0.1)
                if img is not None:
                    cv2.imshow(f"Camera View: {cam_name}", img)
                    cv2.waitKey(1)
            except queue.Empty:
                continue

        cv2.destroyAllWindows()

    def stop(self):
        self.running = False
        self.join(timeout=1.0)


class TrajectoryLoader:
    def __init__(
        self,
        record_dir: str,
        robot_name: str = "command_state",
        display_cameras: bool = True,
    ):
        self.record_dir = Path(record_dir)
        self.robot_dir = self.record_dir / robot_name
        self.sensors_dir = self.record_dir / "sensors"
        self.robot_name = robot_name
        self.display_cameras = display_cameras

        self.camera_names = []
        if self.sensors_dir.exists():
            self.camera_names = [
                d.name for d in self.sensors_dir.iterdir() if d.is_dir()
            ]

        self.current_step_idx = 0
        self.steps = self._load_trajectory()

        # Initialize and start the background display thread
        self.display_thread = None
        if self.display_cameras:
            self.display_thread = CameraDisplayThread()
            self.display_thread.start()

    def _load_trajectory(self) -> list:
        loaded_data = {}
        if self.robot_name == "command_state" and not self.robot_dir.exists():
            raise FileNotFoundError(
                f"No command_state trajectory found at {self.robot_dir}. "
                "This episode was collected before mixed command replay was supported."
            )

        pt_files = list(self.robot_dir.glob("*.pt"))

        if not pt_files:
            if self.robot_name == "command_state":
                raise FileNotFoundError(
                    f"No command_state trajectory files found in {self.robot_dir}. "
                    "This episode was collected before mixed command replay was supported."
                )
            raise FileNotFoundError(
                f"No '.pt' files found in {self.robot_dir}"
            )

        for file_path in pt_files:
            tensor_name = file_path.stem
            loaded_data[tensor_name] = torch.load(file_path, weights_only=True)

        num_steps = len(next(iter(loaded_data.values())))
        trajectory_list = []

        for i in range(num_steps):
            step_dict = {}
            for key, tensor in loaded_data.items():
                step_dict[key] = tensor[i].numpy()
            trajectory_list.append(step_dict)

        return trajectory_list

    @staticmethod
    def _format_source(source: float) -> str:
        return "policy" if source < 0.5 else "human_interrupt"

    def pop(self) -> Optional[Tuple[np.ndarray, np.ndarray, float, float]]:
        """
        Retrieves the next step, queues camera frames safely, and returns pos/quat/gripper/source.
        """
        if not self.steps:
            # Safely shut down the display thread when the trajectory is finished
            if self.display_thread is not None:
                self.display_thread.stop()
            return None

        step_data = self.steps.pop(0)

        # --- 1. Safely Queue Camera Frames ---
        if self.display_thread is not None:
            for cam_name in self.camera_names:
                img_path = (
                    self.sensors_dir
                    / cam_name
                    / f"{self.current_step_idx:06d}.png"
                )
                if img_path.exists():
                    img = cv2.imread(str(img_path))
                    # Put the image in the queue without blocking the robot's control loop
                    if not self.display_thread.frame_queue.full():
                        self.display_thread.frame_queue.put((cam_name, img))
                else:
                    print(f"Warning: Missing image frame at {img_path}")

        self.current_step_idx += 1

        # --- 2. Extract Data ---
        gripper_width = float(np.squeeze(step_data.get("gripper_state", 0.0)))
        source = float(np.squeeze(step_data.get("source", -1.0)))

        pos_data = step_data.get("EE_pos")
        quat_data = step_data.get("EE_quat")

        if pos_data is None or quat_data is None:
            raise ValueError(
                "Missing 'EE_pos' or 'EE_quat' in the loaded data."
            )

        pos = np.array(pos_data).squeeze()
        quat = np.array(quat_data).squeeze()

        # # Using your specific data points to define the behavior
        # if gripper_width < 0.05:
        #     print(
        #         f"Action: Opening the gripper completely (State: Open): {gripper_width:.3f}"
        #     )
        # elif 0.45 <= gripper_width <= 0.60:
        #     print(
        #         f"Action: Grasping an object similar to the cylinder (Value: {gripper_width:.3f})"
        #     )
        # elif gripper_width > 0.8:  # Assuming 1.0 is max
        #     print("Action: Closing the gripper completely with no object")
        # else:
        #     print(
        #         f"Action: Moving to intermediate position (Value: {gripper_width:.3f})"
        #     )

        # time.sleep(0.15)
        print(
            f"Replay step {self.current_step_idx - 1}: "
            f"source={self._format_source(source)} ({source:.1f})"
        )
        return pos, quat, gripper_width, source


def test_trajectory_loader_reads_command_state_in_order(tmp_path):
    command_dir = tmp_path / "command_state"
    command_dir.mkdir()
    torch.save(
        torch.tensor(
            [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]], dtype=torch.float32
        ),
        command_dir / "EE_pos.pt",
    )
    torch.save(
        torch.tensor(
            [
                [0.0, 0.0, 0.0, 1.0],
                [0.0, 0.0, 1.0, 0.0],
            ],
            dtype=torch.float32,
        ),
        command_dir / "EE_quat.pt",
    )
    torch.save(
        torch.tensor([0.25, 0.75], dtype=torch.float32),
        command_dir / "gripper_state.pt",
    )
    torch.save(
        torch.tensor([0.0, 1.0], dtype=torch.float32),
        command_dir / "source.pt",
    )

    loader = TrajectoryLoader(tmp_path, display_cameras=False)

    pos, quat, gripper, source = loader.pop()
    np.testing.assert_allclose(pos, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(quat, [0.0, 0.0, 0.0, 1.0])
    assert gripper == pytest.approx(0.25)
    assert source == pytest.approx(0.0)

    pos, quat, gripper, source = loader.pop()
    np.testing.assert_allclose(pos, [0.4, 0.5, 0.6])
    np.testing.assert_allclose(quat, [0.0, 0.0, 1.0, 0.0])
    assert gripper == pytest.approx(0.75)
    assert source == pytest.approx(1.0)
    assert loader.pop() is None


def test_trajectory_loader_rejects_episode_without_command_state(tmp_path):
    mq3_dir = tmp_path / "MQ3"
    mq3_dir.mkdir()
    torch.save(
        torch.zeros((1, 3), dtype=torch.float32),
        mq3_dir / "EE_pos.pt",
    )

    with pytest.raises(
        FileNotFoundError,
        match="collected before mixed command replay was supported",
    ):
        TrajectoryLoader(tmp_path, display_cameras=False)


class ReplayControlPair(PILPandaControlPair):
    def __init__(
        self,
        panda_arm: RemotePandaArm,
        gripper: Union[RemotePandaGripper, RemoteRobotiqGripper],
        mq3_controller: MQ3PandaControlPair,
        replay_path: str,
        control_hz: float = DEFAULT_CONTROL_HZ,
    ):
        super().__init__(panda_arm, gripper, mq3_controller, control_hz)
        self._last_gripper = None
        self._last_gripper_cmd = None
        self.current_state = PILMode.REPLAY
        self.data_loader = TrajectoryLoader(replay_path)

    def _replay(self):
        previous_state = self.current_state
        count = 0
        start = time.time()
        while True:
            count += 1
            self.current_state = PILMode.REPLAY
            result = self.data_loader.pop()
            # Pop the oldest data point to maintain sync with control steps
            if result is None:
                break
            pos, quat, gripper_width, _source = result
            gripper_cmd = float(gripper_width)
            self.panda_arm.send_cartesian_pose_command(pos=pos, rot=quat)
            if (
                self._last_gripper_cmd is None
                or abs(gripper_cmd - self._last_gripper_cmd) > GRIPPER_DEADBAND
            ):
                self.gripper.send_grasp_command(
                    position=gripper_cmd,
                    speed=GRIPPER_SPEED,
                    force=GRIPPER_FORCE,
                    blocking=False,
                )
                self._last_gripper_cmd = gripper_cmd
            self.reset_action()
            pyzlc.sleep(0.1)

        self.current_state = previous_state


if __name__ == "__main__":
    pyzlc.init(
        "data_collection",
        "141.3.53.25",
        group="224.0.0.1",
        group_name="robot_lab_robotiq_202",
        group_port=7725,
    )
    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    leader = MQ3Controller("IRL-MQ3-2", "192.168.0.117", follower.panda_arm)
    # leader.mq3.wait_for_connection()
    control_pair = ReplayControlPair(
        panda_arm=follower.panda_arm,
        gripper=follower.robotiq_gripper,
        mq3_controller=leader,
        replay_path="/home/jjiang/ahmad/dataset/green_on_yellow/2026_06_02-10_49_26",
        control_hz=50,
    )

    # for now capture_interval is not using only using the global frequency fps
    # camera_left = ImageDataWrapper(CameraDevice("zed_left", preview=False),capture_interval=0.033,hw_name="zed_left")
    # camera_right = ImageDataWrapper(CameraDevice("zed_right", preview=False),capture_interval=0.033,hw_name="zed_right")
    # camera_wrist = ImageDataWrapper(CameraDevice("zed_wrist", preview=False),capture_interval=0.033,hw_name="zed_wrist")
    data_collectors: List[IRLDataWrapper] = []
    # data_collectors.append(camera_left)
    # data_collectors.append(camera_right)
    # data_collectors.append(camera_wrist)
    # data_collectors.append(GelloDataWrapper(leader))
    # data_collectors.append(PandaArmDataWrapper(follower.panda_arm))
    # data_collectors.append(RobotiqGripperDataWrapper(follower.robotiq_gripper))
    # name = time.strftim  e("%Y%m%d_%H%M%S", time.localtime())
    task = "green_on_yellow"
    data_collection_manager = IRLDataCollection(
        data_collectors,
        f"/home/jjiang/ahmad/dataset/lerobot/{task}",
        task,
        fps=20,
        control_pair=control_pair,
    )
    control_pair.control_reset()
    data_collection_manager.run()
    pyzlc.shutdown()
