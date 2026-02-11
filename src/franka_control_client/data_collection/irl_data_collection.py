import queue
import time
from typing import Dict, Optional, List
import numpy as np
from concurrent.futures import Future

from .data_collection_manager import DataCollectionManager, DataCollectionState
from .irl_wrapper import IRL_HardwareDataWrapper,ImageDataWrapper
import shutil
import time
import os
from datetime import datetime
from pathlib import Path
from queue import Full #used to handle full queue exception
from concurrent.futures import ThreadPoolExecutor, wait
from typing import Optional, NamedTuple, List

import cv2
import numpy as np
import pyzlc
import torch


class FollowerData:
        def __init__(self):
            # self.timestamp_ms_list = []
            # self.O_T_EE_list = []
            # self.O_T_EE_d_list = []
            self.q_list = []
            # self.q_d_list = []
            # self.dq_list = []
            # self.dq_d_list = []
            # self.tau_ext_hat_filtered_list = []
            self.gripper_state_list = []

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

        def save(self, path: Path):
            
            tensor_lists = [
                # torch.tensor(self.timestamp_ms_list, dtype=torch.int64),
                # torch.stack(self.O_T_EE_list),
                # torch.stack(self.O_T_EE_d_list),
                torch.stack(self.q_list),
                # torch.stack(self.q_d_list),
                # torch.stack(self.dq_list),
                # torch.stack(self.dq_d_list),
                # torch.stack(self.tau_ext_hat_filtered_list),
                torch.stack(self.gripper_state_list)
            ]
            paths = [
                # path / "timestamp_ms.pt",
                # path / "O_T_EE.pt",
                # path / "O_T_EE_d.pt",
                path / "q.pt",
                # path / "q_d.pt",
                # path / "dq.pt",
                # path / "dq_d.pt",
                # path / "tau_ext_hat_filtered.pt",
                path / "gripper_state.pt",
            ]

            for d, p in zip(tensor_lists, paths):

                if d.numel() == 0:
                    print(f"Skip saving '{p}' since it is empty")
                    continue

                torch.save(d, p)
                print(f"Successfully saved '{p}'")

class LeaderData:
    def __init__(self):
        # self.timestamp_ms_list = []
        # self.O_T_EE_list = []
        # self.O_T_EE_d_list = []
        self.q_list = []
        # self.q_d_list = []
        # self.dq_list = []
        # self.dq_d_list = []
        # self.tau_ext_hat_filtered_list = []
        self.gripper_state_list = []

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

    def save(self, path: Path):

        tensor_lists = [
            # torch.tensor(self.timestamp_ms_list, dtype=torch.int64),
            # torch.stack(self.O_T_EE_list),
            # torch.stack(self.O_T_EE_d_list),
            torch.stack(self.q_list),
            # torch.stack(self.q_d_list),
            # torch.stack(self.dq_list),
            # torch.stack(self.dq_d_list),
            # torch.stack(self.tau_ext_hat_filtered_list),
                torch.stack(self.gripper_state_list)
            ]
        paths = [
            # path / "timestamp_ms.pt",
            # path / "O_T_EE.pt",
            # path / "O_T_EE_d.pt",
            path / "q.pt",
            # path / "q_d.pt",
            # path / "dq.pt",
            # path / "dq_d.pt",
            # path / "tau_ext_hat_filtered.pt",
            path / "gripper_state.pt",
        ]

        for d, p in zip(tensor_lists, paths):

            if d.numel() == 0:
                print(f"Skip saving '{p}' since it is empty")
                continue

            torch.save(d, p)
            print(f"Successfully saved '{p}'")

class IRLDataCollection(DataCollectionManager):
    def __init__(
        self,
        data_collectors: List[IRL_HardwareDataWrapper],
        data_dir: Path,
        task: str,
        fps: int = 50,#for general
        writer_pool_max_workers: Optional[int] = None,
        writer_max_pending_writes: int = 4096,
        
    ) -> None:
        super().__init__(data_collectors, task, fps)
        #data_dir
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(exist_ok=True,parents=True)
        #writer_pool preparation for cams
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
        self.camera_streams:List[ImageDataWrapper] = []
        self.camera_timestamps: List[list[float]] = []
        self.timestamps = []
        self.cur_timestep = 0
        self.capture_interval = 1.0/fps # in second
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
        self.camera_last_capture_times: List[float] = [0.0] * len(self.camera_streams)  # Track last capture time for each camera
        self._last_robot_time: Optional[float] = None
        


    def _start_collecting(self) -> None:
        # Emit start-collection event (e.g., start control pair).
        super()._start_collecting()
        self._create_new_recording_dir()
        self._create_empty_data()
        self.timestamps = []
        self.cur_timestep = 0
        # Ensure cameras can capture immediately on a new episode.
        self.camera_last_capture_times = [0.0] * len(self.camera_streams)

    def _collect_step(self) -> None:
        # print("debug:time start collect")
        to_tensor = lambda x: torch.tensor(x, dtype=torch.float64)
        cur_time = time.time()
        # print("debug:capture_inter:",self.capture_interval)
        # print("cur_time:",cur_time)
        if self.timestamps==[] or cur_time - self.timestamps[-1] >= self.capture_interval:
            self._capture_camera_frames()
            self.timestamps.append(cur_time)
            leader_state = self.leader_robot.capture_step() #gello
            follower_arm_state = self.follower_arm.capture_step()
            follower_gripper_state = self.follower_gripper.capture_step()
            #todo:using smarter way to wrapper
            self.leader_robot_data.q_list.append(to_tensor(leader_state["gello_arm_state"]["joint_state"]))
            self.leader_robot_data.gripper_state_list.append(to_tensor(leader_state["gello_gripper_state"]["gripper"]))
            self.follower_robot_data.q_list.append(to_tensor(follower_arm_state["q"]))
            self.follower_robot_data.gripper_state_list.append(to_tensor(follower_gripper_state["position"]))
            self.cur_timestep += 1
            # end_time = time.time()
        
            # print("1 step of collect_step",end_time-cur_time)

        # Throttle to target robot fps
        if self._last_robot_time is None:
            self._last_robot_time = start_time
        elapsed = time.perf_counter() - start_time
        sleep_time = max(0.0, (1.0 / self.fps) - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)
        self._last_robot_time = time.perf_counter()

    def _save_data_task(self) -> None:
        self._ui_console.log("Data saving task started.")
        self.__flush_writes()
        # Ensure all queued frame writes are finished before saving metadata.
        # self.__shutdown_writer_pool()

        timestamps_path = self.record_dir / "timestamps.pt"
        torch.save(torch.tensor(self.timestamps, dtype=torch.float64), timestamps_path)
        print(f"Successfully saved '{timestamps_path}'")
        self.leader_robot_data.save(self.leader_robot_dir)
        self.follower_robot_data.save(self.follower_robot_dir)


        self.__report_camera_rates()

        # determine average frame rate from timestamps
        if len(self.timestamps) > 1:
            print(f"Robot states frame rate: {len(self.timestamps) / (self.timestamps[-1] - self.timestamps[0]):.2f} Hz")
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
        shutil.rmtree(self.record_dir)#clean the camera framse which already saved

        for collector in self.data_collectors:
            collector.discard()
        self._ui_console.log("Episode discarded.")

    def __report_camera_rates(self) -> None:
        """Report average frame rate for each camera based on captured timestamps."""
        for name, timestamps in zip(self.camera_names, self.camera_timestamps):
            if len(timestamps) <= 1:
                print(f"Camera '{name}' frame rate: insufficient samples ({len(timestamps)})")
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

    def _reset_to_waiting(self) -> None:
        super()._reset_to_waiting()

    def _close(self) -> None:
        if self._close:
            return
        self._close = True

        super()._close()

    def _create_new_recording_dir(self):
        self.record_dir = self.data_dir / datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        self.record_dir.mkdir()
        
        self.leader_robot_dir = self.record_dir / self.leader_robot.hw_name
        self.leader_robot_dir.mkdir()

        self.follower_robot_dir = self.record_dir / self.follower_arm.hw_name
        self.follower_robot_dir.mkdir()

        self.sensors_dir = self.record_dir / "sensors"
        self.sensors_dir.mkdir()

        self.camera_dirs = []
        for cam_name in self.camera_names:
            device_dir = self.sensors_dir / cam_name
            device_dir.mkdir()
            self.camera_dirs.append(device_dir)

    def _create_empty_data(self):
        self.leader_robot_data = LeaderData()
        self.follower_robot_data=FollowerData()
        self.camera_timestamps = [[] for _ in self.camera_streams]
        self.camera_frame_idx = [0] * len(self.camera_streams)  # Reset frame index for all cameras
        self.camera_last_capture_times = [time.time()] * len(self.camera_streams)  # Initialize capture times

    def _capture_camera_frames(self) -> None:
        if self.camera_streams==[] :
            pyzlc.info("no camera in stream")
            return
        cur_time = time.perf_counter()
        for idx, stream in enumerate(self.camera_streams):
            # Check if it's time to capture for this camera based on its fps
            # begin_time = time.time()
            # print("debug:capture begin time", begin_time,stream.hw_name)
            if stream.fps > 0:
                min_interval = 1.0 / stream.fps
                if (cur_time - self.camera_last_capture_times[idx]) < min_interval:
                    continue
            # print("debug:get camera")
            camera_dir = self.camera_dirs[idx]
            frame = stream.capture_step()
            if frame is None:
                continue
            self.camera_timestamps[idx].append(time.time())
            
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
                    f"Camera '{self.camera_names[idx]}' writer pool backlog full, dropping frame"
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
            self.__write_frame, frame_path, image_bgr, cam_name, step
        )
        self._writer_futures.append(future)

    def __prune_completed_writes(self) -> None:
        if not self._writer_futures:
            return
        self._writer_futures = [f for f in self._writer_futures if not f.done()]

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
            print(f"Failed to write frame {frame_path} (cam {cam_name}, step {step}): {exc}")
