import os
import shutil
import time
from concurrent.futures import ThreadPoolExecutor, wait
from datetime import datetime
from pathlib import Path
from typing import List, Optional
import threading

import cv2
import numpy as np
import pyzlc
import torch

from ..control_pair.control_pair import ControlPair
from .data_collection_manager import DataCollectionManager, DataCollectionState
from .irl_wrapper import IRLDataWrapper, ImageDataWrapper
from .utils import NonBlockingKeyPress


class FollowerData:
    def __init__(self):
        # self.timestamp_ms_list = []
        self.EE_pos = []
        self.EE_quat = []
        self.O_T_EE_list = []
        # self.O_T_EE_d_list = []
        self.q_list = []
        # self.q_d_list = []
        self.dq_list = []
        # self.dq_d_list = []
        self.tau_ext_hat_filtered_list = []
        self.gripper_state_list = []
        self.O_F_ext_hat_K_list = []
        self.gripper_current_list = []

    # def append(self):
    # self.timestamp_ms_list.append(state.timestamp_ms)
    #     # self.O_T_EE_list.append(state.O_T_EE)
    #     # self.O_T_EE_d_list.append(state.O_T_EE_d)
    #     self.q_list.append(state.q)
    #     # self.q_d_list.append(state.q_d)
    #     # self.dq_list.append(state.dq)
    #     # self.dq_d_list.append(state.dq_d)
    #     # self.tau_ext_hat_filtered_list.append(state.tau_ext_hat_filtered)
    #     self.gripper_state_list[]
    def pop(self):
        if len(self.EE_pos) == 0:
            return
        self.EE_pos.pop()
        self.EE_quat.pop()
        self.O_T_EE_list.pop()
        # self.O_T_EE_d_list.pop()
        self.q_list.pop()
        # self.q_d_list.pop()
        self.dq_list.pop()
        # self.dq_d_list.pop()
        self.tau_ext_hat_filtered_list.pop()
        self.gripper_state_list.pop()
        self.O_F_ext_hat_K_list.pop()
        self.gripper_current_list.pop()

    def save(self, path: Path):

        tensor_lists = [
            # torch.tensor(self.timestamp_ms_list, dtype=torch.int64),
            torch.stack(self.EE_pos),
            torch.stack(self.EE_quat),
            torch.stack(self.O_T_EE_list),
            # torch.stack(self.O_T_EE_d_list),
            torch.stack(self.q_list),
            # torch.stack(self.q_d_list),
            torch.stack(self.dq_list),
            # torch.stack(self.dq_d_list),
            torch.stack(self.tau_ext_hat_filtered_list),
            torch.stack(self.gripper_state_list),
            torch.stack(self.O_F_ext_hat_K_list),
            torch.stack(self.gripper_current_list),
        ]
        paths = [
            path / "EE_pos.pt",
            path / "EE_quat.pt",
            # path / "timestamp_ms.pt",
            path / "ee_pos.pt",
            # path / "O_T_EE_d.pt",
            path / "joint_pos.pt",
            # path / "q_d.pt",
            path / "joint_vel.pt",
            # path / "dq_d.pt",
            path / "external_joint_torque.pt",
            path / "gripper_state.pt",
            path / "external_wrench.pt",
            path / "gripper_current.pt",
        ]

        for d, p in zip(tensor_lists, paths):

            if d.numel() == 0:
                print(f"Skip saving '{p}' since it is empty")
                continue

            torch.save(d, p)
            print(f"Successfully saved '{p}'")


class PolicyState:
    def __init__(self):
        self.EE_pos = []
        self.EE_quat = []
        self.gripper_width = []

    def pop(self):
        if len(self.EE_pos) == 0:
            return
        self.EE_pos.pop()
        self.EE_quat.pop()
        self.gripper_width.pop()

    def save(self, path: Path):
        tensor_lists = [
            torch.stack(self.EE_pos),
            torch.stack(self.EE_quat),
            torch.stack(self.gripper_width),
        ]
        paths = [
            path / "EE_pos.pt",
            path / "EE_quat.pt",
            path / "gripper_width.pt",
        ]

        for d, p in zip(tensor_lists, paths):
            if d.numel() == 0:
                print(f"Skip saving '{p}' since it is empty")
                continue

            torch.save(d, p)
            print(f"Successfully saved '{p}'")


class CommandStateData:
    def __init__(self):
        self.EE_pos = []
        self.EE_quat = []
        self.gripper_state_list = []
        self.source = []

    def append(self, action: np.ndarray, source: float, to_tensor) -> None:
        self.EE_pos.append(to_tensor(action[:3]))
        self.EE_quat.append(to_tensor(action[3:7]))
        self.gripper_state_list.append(to_tensor(action[-1]))
        self.source.append(float(source))

    def pop(self):
        if len(self.EE_pos) == 0:
            return
        self.source.pop()
        return (
            self.EE_pos.pop().detach().cpu().numpy(),
            self.EE_quat.pop().detach().cpu().numpy(),
            self.gripper_state_list.pop().detach().cpu().numpy(),
        )

    def save(self, path: Path):
        if len(self.EE_pos) == 0:
            print(f"Skip saving command_state in '{path}' since it is empty")
            return

        tensor_lists = [
            torch.stack(self.EE_pos),
            torch.stack(self.EE_quat),
            torch.stack(self.gripper_state_list),
            torch.tensor(self.source, dtype=torch.float32),
        ]
        paths = [
            path / "EE_pos.pt",
            path / "EE_quat.pt",
            path / "gripper_state.pt",
            path / "source.pt",
        ]

        for d, p in zip(tensor_lists, paths):
            if d.numel() == 0:
                print(f"Skip saving '{p}' since it is empty")
                continue

            torch.save(d, p)
            print(f"Successfully saved '{p}'")


def _normalize_policy_control_signal(policy_control_signal) -> Optional[np.ndarray]:
    if policy_control_signal is None:
        return None

    action = np.asarray(policy_control_signal, dtype=np.float64)
    if action.ndim == 3:
        action = action[0, 0]
    elif action.ndim == 2:
        action = action[0]
    elif action.ndim == 1:
        pass
    else:
        raise ValueError(
            f"Expected policy control signal shape (D,), (T, D), or (B, T, D), got {action.shape}"
        )

    action = action.reshape(-1)
    if action.size < 8:
        raise ValueError(
            f"Expected policy control signal size >= 8, got {action.size}"
        )
    return action[:8]


class LeaderData:
    def __init__(self):
        # self.timestamp_ms_list = []
        # self.O_T_EE_list = []
        # self.O_T_EE_d_list = []
        self.EE_pos = []
        self.EE_quat = []
        self.source = []
        # self.q_list = []
        # self.q_d_list = []
        # self.dq_list = []
        # self.dq_d_list = []
        # self.tau_ext_hat_filtered_list = []
        self.gripper_width_list = []
        # self.gripper_state_list = []
        # self.gripper_command_list = []#command read from robotiq

        # def append(self):
        #     self.timestamp_ms_list.append(state.timestamp_ms)
        #     # self.O_T_EE_list.append(state.O_T_EE)
        #     # self.O_T_EE_d_list.append(state.O_T_EE_d)
        #     self.q_list.append(state.q)
        #     # self.q_d_list.append(state.q_d)
        #     # self.dq_list.append(state.dq)
        #     # self.dq_d_list.append(state.dq_d)
        #     # self.tau_ext_hat_filtered_list.append(state.tau_ext_hat_filtered)
        #     self.gripper_state_list[]

    def pop(self):
        # self.q_list.pop()
        # self.q_d_list.pop()
        # self.dq_list.pop()
        # self.dq_d_list.pop()
        # self.tau_ext_hat_filtered_list.pop()
        # self.gripper_width_list.pop()
        # self.gripper_state_list.pop()
        # self.gripper_command_list.pop()
        if len(self.EE_pos) == 0:
            return
        self.source.pop()
        return (
            self.EE_pos.pop().detach().cpu().numpy(),
            self.EE_quat.pop().detach().cpu().numpy(),
            self.gripper_width_list.pop().detach().cpu().numpy(),
        )

    def save(self, path: Path):

        print(self.source)
        print(type(self.source))
        tensor_lists = [
            # torch.tensor(self.timestamp_ms_list, dtype=torch.int64),
            # torch.stack(self.O_T_EE_list),
            # torch.stack(self.O_T_EE_d_list),
            torch.stack(self.EE_pos),
            torch.stack(self.EE_quat),
            torch.FloatTensor(self.source),
            # torch.stack(self.q_list),
            # torch.stack(self.q_d_list),
            # torch.stack(self.dq_list),
            # torch.stack(self.dq_d_list),
            # torch.stack(self.tau_ext_hat_filtered_list),
            torch.stack(self.gripper_width_list),
            # torch.stack(self.gripper_command_list)
        ]
        paths = [
            # path / "timestamp_ms.pt",
            # path / "O_T_EE.pt",
            # path / "O_T_EE_d.pt",
            path / "EE_pos.pt",
            path / "EE_quat.pt",
            path / "source",
            # path / "joint_pos.pt",
            # path / "q_d.pt",
            # path / "dq.pt",
            # path / "dq_d.pt",
            # path / "tau_ext_hat_filtered.pt",
            path / "gripper_state.pt",
            path / "gripper_command.pt",
        ]

        for d, p in zip(tensor_lists, paths):

            if d.numel() == 0:
                print(f"Skip saving '{p}' since it is empty")
                continue

            torch.save(d, p)
            print(f"Successfully saved '{p}'")


class PILIRLDataCollection(DataCollectionManager):
    def __init__(
        self,
        data_collectors: List[IRLDataWrapper],
        data_dir: Path,
        task: str,
        control_pair: Optional[ControlPair] = None,
        fps: int = 50,  # for general
        writer_pool_max_workers: Optional[int] = None,
        writer_max_pending_writes: int = 4096,
    ) -> None:
        super().__init__(data_collectors, task, fps)
        # data_dir
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(exist_ok=True, parents=True)
        self.control_pair = control_pair
        if self.control_pair is not None:
            self.register_start_collecting_event(
                self.control_pair.start_control_pair
            )
            self.register_stop_collecting_event(
                self.control_pair.stop_control_pair
            )
        # writer_pool preparation for cams
        self._max_pending_writes = int(writer_max_pending_writes)
        if writer_pool_max_workers is None:
            max_workers = max(2, min(8, (os.cpu_count() or 4)))
        else:
            max_workers = max(1, int(writer_pool_max_workers))
        self._writer_pool = ThreadPoolExecutor(max_workers=max_workers)
        self._writer_futures = []
        #
        self.camera_dirs: List[Path] = []
        self.camera_names: List[str] = []
        self.camera_streams: List[ImageDataWrapper] = []
        self.camera_timestamps: List[list[float]] = []
        self.timestamps = []
        self.cur_timestep = 0
        self.capture_interval = 1.0 / fps  # in second
        for hw in data_collectors:
            if hw.hw_type == "leader_robot":
                self.leader_robot = hw
            if hw.hw_type == "follower_arm":
                self.follower_arm = hw
            if hw.hw_type == "follower_gripper":
                self.follower_gripper = hw
            if hw.hw_type == "camera":
                self.camera_names.append(hw.hw_name)
                self.camera_streams.append(hw)
        self.camera_frame_idx = [0] * len(self.camera_streams)
        self.camera_last_capture_times: List[float] = [0.0] * len(
            self.camera_streams
        )  # Track last capture time for each camera
        self._last_robot_time: Optional[float] = None
        self.data_lock = threading.Lock()
        self.pause_event = threading.Event()
        self._closed = False

    def set_pause(self, flag: bool):
        if flag:
            self.pause_event.set()
        else:
            self.pause_event.clear()

    def pop(self):
        with self.data_lock:
            self.follower_robot_data.pop()
            self.policy_state_data.pop()
            self.leader_robot_data.pop()
            return self.command_state_data.pop()

    def _start_collecting(self) -> None:
        # Emit start-collection event (e.g., start control pair).
        super()._start_collecting()
        self._create_new_recording_dir()
        self._create_empty_data()
        self.timestamps = []
        self.cur_timestep = 0
        # Ensure cameras can capture immediately on a new episode.
        self.camera_last_capture_times = [0.0] * len(self.camera_streams)
        self.last_gripper = 0.0018

    def _collect_step(
        self, policy_control_signal=None, command_source: float = 1.0
    ) -> None:
        # print("debug:time start collect")
        to_tensor = lambda x: torch.tensor(x, dtype=torch.float64)
        start_time = time.perf_counter()
        # cur_time = time.time()
        # print("debug:capture_inter:",self.capture_interval)
        # print("cur_time:",cur_time)
        # if self.timestamps==[] or cur_time - self.timestamps[-1] >= self.capture_interval:
        #     # self._capture_camera_frames()
        #     self.timestamps.append(cur_time)
        #     leader_state = self.leader_robot.capture_step() #gello
        #     follower_arm_state = self.follower_arm.capture_step()
        #     follower_gripper_state = self.follower_gripper.capture_step()
        #     #todo:using smarter way to wrapper
        #     self.leader_robot_data.q_list.append(to_tensor(leader_state["gello_arm_state"]["joint_state"]))
        #     self.leader_robot_data.gripper_state_list.append(to_tensor(leader_state["gello_gripper_state"]["gripper"]))
        #     self.follower_robot_data.q_list.append(to_tensor(follower_arm_state["q"]))
        #     self.follower_robot_data.gripper_state_list.append(to_tensor(follower_gripper_state["position"]))
        #     self.cur_timestep += 1
        # end_time = time.time()

        self.timestamps.append(start_time)
        leader_state = self.leader_robot.capture_step()  # gello
        follower_arm_state = self.follower_arm.capture_step()
        follower_gripper_state = self.follower_gripper.capture_step()
        # robotiq sometimes can not get state in time, so use last time to pad
        if follower_gripper_state["position"] == 0.0:
            follower_gripper_state["position"] = self.last_gripper
        else:
            self.last_gripper = follower_gripper_state["position"]
        # todo:using smarter way to wrapper
        # todo:maybe change gripper command to record robotiq command
        # leader
        policy_action = _normalize_policy_control_signal(policy_control_signal)
        if policy_action is not None:
            # If we're in policy control, we can override the leader state with the policy control signal for certain fields.
            # This allows us to capture what the policy is commanding while still recording the actual state of the leader robot.
            self.policy_state_data.EE_pos.append(to_tensor(policy_action[:3]))
            self.policy_state_data.EE_quat.append(to_tensor(policy_action[3:7]))
            self.policy_state_data.gripper_width.append(to_tensor(policy_action[-1]))
        else:
            # If no policy control signal, save None/zero values
            self.policy_state_data.EE_pos.append(to_tensor(np.zeros(3)))
            self.policy_state_data.EE_quat.append(to_tensor(np.zeros(4)))
            self.policy_state_data.gripper_width.append(to_tensor(0.0))

        if policy_action is not None:
            command_action = np.array(policy_action, copy=True)
            if command_source < 0.5:
                command_action[-1] = 1.0 if command_action[-1] >= 0.5 else 0.0
            self.command_state_data.append(
                command_action, command_source, to_tensor
            )
        elif command_source >= 0.5:
            interrupt_action = np.concatenate(
                [
                    np.asarray(leader_state["EE_pos"], dtype=np.float64),
                    np.asarray(leader_state["EE_quat"], dtype=np.float64),
                    np.asarray([leader_state["gripper_width"]], dtype=np.float64),
                ]
            )
            self.command_state_data.append(
                interrupt_action, command_source, to_tensor
            )

        self.leader_robot_data.source.append(command_source)
        self.leader_robot_data.EE_pos.append(to_tensor(leader_state["EE_pos"]))
        self.leader_robot_data.EE_quat.append(
            to_tensor(leader_state["EE_quat"])
        )
        # self.leader_robot_data.q_list.append(to_tensor(leader_state["gello_arm_state"]["joint_state"]))
        self.leader_robot_data.gripper_width_list.append(
            to_tensor(leader_state["gripper_width"])
        )
        # self.leader_robot_data.gripper_command_list.append(to_tensor(follower_gripper_state["commanded_position"]))
        # follower arm
        self.follower_robot_data.q_list.append(
            to_tensor(follower_arm_state["q"])
        )
        self.follower_robot_data.EE_pos.append(
            to_tensor(follower_arm_state["EE_pos"])
        )
        self.follower_robot_data.EE_quat.append(
            to_tensor(follower_arm_state["EE_quat"])
        )
        self.follower_robot_data.O_T_EE_list.append(
            to_tensor(follower_arm_state["O_T_EE"])
        )
        self.follower_robot_data.dq_list.append(
            to_tensor(follower_arm_state["dq"])
        )
        self.follower_robot_data.tau_ext_hat_filtered_list.append(
            to_tensor(follower_arm_state["tau_ext_hat_filtered"])
        )
        self.follower_robot_data.O_F_ext_hat_K_list.append(
            to_tensor(follower_arm_state["O_F_ext_hat_K"])
        )
        # follower gripper
        self.follower_robot_data.gripper_state_list.append(
            to_tensor(follower_gripper_state["position"])
        )
        self.follower_robot_data.gripper_current_list.append(
            to_tensor(follower_gripper_state["current"])
        )
        # cameras
        self._capture_camera_frames()
        self.cur_timestep += 1
        # print("1 step of collect_step",end_time-cur_time)

        # Throttle to target robot fps
        if self._last_robot_time is None:
            self._last_robot_time = start_time
        elapsed = time.perf_counter() - start_time
        sleep_time = (
            max(0.0, (1.0 / self.fps) - elapsed) - 0.0015
        )  # adjust a little
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        self._last_robot_time = time.perf_counter()

    def _save_data_task(self) -> None:
        self._ui_console.log("Data saving task started.")
        self.__flush_writes()

        timestamps_path = self.record_dir / "timestamps.pt"
        torch.save(
            torch.tensor(self.timestamps, dtype=torch.float64), timestamps_path
        )
        print(f"Successfully saved '{timestamps_path}'")
        self.leader_robot_data.save(self.leader_robot_dir)
        self.follower_robot_data.save(self.follower_robot_dir)
        self.policy_state_data.save(self.policy_state_dir)
        self.command_state_data.save(self.command_state_dir)

        self.__report_camera_rates()

        # determine average frame rate from timestamps
        if len(self.timestamps) > 1:
            print(
                f"Robot states frame rate: {len(self.timestamps) / (self.timestamps[-1] - self.timestamps[0]):.2f} Hz"
            )
        else:
            print("Robot states frame rate: only one sample captured")

    def _save_episode(self) -> None:
        self._save_data_task()
        self._stop_collecting()
        self._ui_console.log("Episode saved.")

    def _discard_collecting(self) -> None:
        self._stop_collecting()
        # Make sure all pending camera writes are finished before cleanup.
        self.__flush_writes()
        shutil.rmtree(
            self.record_dir
        )  # clean the camera framse which already saved

        for collector in self.data_collectors:
            collector.discard()
        self._ui_console.log("Episode discarded.")

    def __report_camera_rates(self) -> None:
        """Report average frame rate for each camera based on captured timestamps."""
        for name, timestamps in zip(self.camera_names, self.camera_timestamps):
            if len(timestamps) <= 1:
                print(
                    f"Camera '{name}' frame rate: insufficient samples ({len(timestamps)})"
                )
                continue

            span = timestamps[-1] - timestamps[0]
            if span <= 0:
                print(f"Camera '{name}' frame rate: invalid timestamp span")
                continue

            fps = len(timestamps) / span
            print(f"Camera '{name}' frame rate: {fps:.2f} Hz")

    def _stop_collecting(self) -> None:
        # Emit stop-collection event (e.g., stop control pair) first.
        super()._stop_collecting()

        # self.data_save_future = None

    def _reset_arm(self) -> None:
        """Reset the robot arm to home/teleoperation.

        Called only when in WAITING state (control pair is not running).
        """
        if self.control_pair is None:
            self._ui_console.log("No control pair configured for reset.")
            return
        self._ui_console.log("Resetting robot arm position...")
        try:
            # Reset the arm to home position
            self.control_pair.control_reset()
            time.sleep(3)  # Wait for the arm to reach the home position
            self._ui_console.log("Robot arm reset to home position.")
        except Exception as exc:
            self._ui_console.log(f"Failed to reset arm: {exc}")

    def _reset_to_waiting(self) -> None:
        super()._reset_to_waiting()

    def _close(self) -> None:
        if self._closed:
            return
        self._closed = True

        super()._close()

    def _create_new_recording_dir(self):
        self.record_dir = self.data_dir / datetime.now().strftime(
            "%Y_%m_%d-%H_%M_%S"
        )
        self.record_dir.mkdir()

        self.leader_robot_dir = self.record_dir / self.leader_robot.hw_name
        self.leader_robot_dir.mkdir()

        self.follower_robot_dir = self.record_dir / self.follower_arm.hw_name
        self.follower_robot_dir.mkdir()

        self.policy_state_dir = self.record_dir / "policy_state"
        self.policy_state_dir.mkdir()

        self.command_state_dir = self.record_dir / "command_state"
        self.command_state_dir.mkdir()

        self.sensors_dir = self.record_dir / "sensors"
        self.sensors_dir.mkdir()

        self.camera_dirs = []
        for cam_name in self.camera_names:
            device_dir = self.sensors_dir / cam_name
            device_dir.mkdir()
            self.camera_dirs.append(device_dir)

    def _create_empty_data(self):
        self.leader_robot_data = LeaderData()
        self.follower_robot_data = FollowerData()
        self.policy_state_data = PolicyState()
        self.command_state_data = CommandStateData()
        self.camera_timestamps = [[] for _ in self.camera_streams]
        self.camera_frame_idx = [0] * len(
            self.camera_streams
        )  # Reset frame index for all cameras
        self.camera_last_capture_times = [time.time()] * len(
            self.camera_streams
        )  # Initialize capture times

    def _capture_camera_frames(self) -> None:
        if self.camera_streams == []:
            pyzlc.info("no camera in stream")
            return
        cur_time = time.perf_counter()
        for idx, stream in enumerate(self.camera_streams):
            # Check if it's time to capture for this camera based on its capture_interval
            # begin_time = time.time()
            # print("debug:capture begin time", begin_time,stream.hw_name)
            #### To open the different frequency of cams
            # if stream.capture_interval > 0 and (cur_time - self.camera_last_capture_times[idx]) < (stream.capture_interval-0.0027):
            #     continue
            # print("debug:get camera")
            camera_dir = self.camera_dirs[idx]
            frame = stream.capture_step()
            self.camera_timestamps[idx].append(time.perf_counter())
            if frame is None:
                continue

            # Update last capture time for this camera
            self.camera_last_capture_times[idx] = cur_time

            frame_idx = self.camera_frame_idx[idx]
            self.camera_frame_idx[idx] += 1

            image_rgb = frame
            image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
            frame_path = camera_dir / f"{frame_idx:06d}.png"
            # metadata_path = camera_dir / f"{frame_idx:06d}.json"

            try:
                self._submit_frame_write(
                    frame_path,
                    # metadata_path,
                    image_bgr,
                    # frame.metadata,
                    self.camera_names[idx],
                    self.cur_timestep,
                )
                # end_time = time.time()
                # print("debug:capture end time", end_time,self.camera_names[idx])
            except Full:
                print(
                    f"Camera '{self.camera_names[idx]}' writer pool backlog full, dropping frame {self.cur_timestep}"
                )
                continue

            # self.camera_metadata_list[idx].append(frame.metadata.to_dict())

    def _submit_frame_write(
        self,
        frame_path: Path,
        # metadata_path: Path,
        image_bgr: np.ndarray,
        # metadata: FrameMetadata,
        cam_name: str,
        step: int,
    ) -> None:
        self.__prune_completed_writes()
        if len(self._writer_futures) >= self._max_pending_writes:
            raise Full

        future = self._writer_pool.submit(
            # self.__write_frame, frame_path, metadata_path, image_bgr, metadata, cam_name, step
            self.__write_frame,
            frame_path,
            image_bgr,
            cam_name,
            step,
        )
        self._writer_futures.append(future)

    def __prune_completed_writes(self) -> None:
        if not self._writer_futures:
            return
        self._writer_futures = [
            f for f in self._writer_futures if not f.done()
        ]

    def __flush_writes(self) -> None:
        if not self._writer_futures:
            return
        wait(self._writer_futures)
        self.__prune_completed_writes()

    # def __shutdown_writer_pool(self) -> None:
    #     if self._writer_pool is None:
    #         return
    #     self._writer_pool.shutdown(wait=True)
    #     self._writer_pool = None

    @staticmethod
    def __write_frame(
        frame_path: Path,
        # metadata_path: Path,
        image_bgr: np.ndarray,
        # metadata: FrameMetadata,
        cam_name: str,
        step: int,
    ) -> None:
        try:
            cv2.imwrite(str(frame_path), image_bgr)
            # metadata.save_to_file(str(metadata_path))
        except Exception as exc:
            print(
                f"Failed to write frame {frame_path} (cam {cam_name}, step {step}): {exc}"
            )
