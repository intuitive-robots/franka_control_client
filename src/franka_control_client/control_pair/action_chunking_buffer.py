from typing import Optional

import numpy as np
import threading
import time
from scipy.spatial.transform import Rotation as R

from ..franka_robot.franka_panda import RemotePandaArm


class DeltaActionChunkingBuffer:
    def __init__(
        self,
        robot_arm: RemotePandaArm,
        action_dt: float,
        chunk_size: int,
        action_dim: int,
    ):
        self._chunk_size = chunk_size
        self._action_dt = action_dt
        self.robot_arm = robot_arm
        self._lock = threading.Lock()
        self._last_action_time = None
        self._buffer = np.zeros((chunk_size, action_dim), dtype=np.float32)

    def add_new_action_chunk(self, new_action_chunk: np.ndarray):
        print(new_action_chunk)
        if new_action_chunk.shape != (self._chunk_size, self._buffer.shape[1]):
            raise ValueError(
                f"New action chunk shape {new_action_chunk.shape} does not match buffer shape {self._buffer[0].shape if self._buffer else 'None'}."
            )
        absolute_chunk = self.delta2absolute(new_action_chunk)
        with self._lock:
            if self._last_action_time is None:
                self._buffer = absolute_chunk
            else:
                self._fuse_action_chunks(absolute_chunk)
            self._last_action_time = time.perf_counter()

    def _fuse_action_chunks(self, absolute_chunks: np.ndarray) -> None:
        self._buffer = np.array(absolute_chunks, copy=True)

    def _get_current_action_chunk_int(self) -> int:
        return (
            int(
                (time.perf_counter() - self._last_action_time)
                / self._action_dt
            )
            if self._last_action_time is not None
            else 0
        )

    def get_action(self) -> Optional[np.ndarray]:
        with self._lock:
            if self._last_action_time is None:
                return None
            return np.array(self._buffer, copy=True)

    def apply_action(self) -> Optional[np.ndarray]:
        with self._lock:
            if self._last_action_time is None:
                return None
            current_index = self._get_current_action_chunk_int()
            clamped_index = min(current_index, len(self._buffer) - 1)
            action = np.array(self._buffer[clamped_index], copy=True)
            return action

    def clear(self) -> None:
        with self._lock:
            self._buffer = np.zeros_like(self._buffer)
            self._last_action_time = None

    def delta2absolute(self, delta_action_chunks: np.ndarray) -> np.ndarray:
        current_ee_pos = self.robot_arm.current_ee_position
        if current_ee_pos is None:
            raise ValueError("Current end-effector position is not available.")
        current_ee_pos = np.array(current_ee_pos)
        current_ee_quat = self.robot_arm.current_ee_rotation
        if current_ee_quat is None:
            raise ValueError("Current end-effector rotation is not available.")
        current_ee_quat = np.array(current_ee_quat)
        action = np.zeros_like(delta_action_chunks)
        for i in range(len(delta_action_chunks)):
            action[i][:3] = current_ee_pos + delta_action_chunks[i][:3]
            current_ee_pos = action[i][:3]
            action[i][3:7] = (
                R.from_quat(current_ee_quat)
                * R.from_euler("xyz", delta_action_chunks[i][3:6], False)
            ).as_quat()
            current_ee_quat = action[i][3:7]
            action[i][7:] = delta_action_chunks[i][6]
        return action
