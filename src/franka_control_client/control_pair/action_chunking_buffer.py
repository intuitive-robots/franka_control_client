import numpy as np
import threading
import time


class ActionChunkingBuffer:
    def __init__(self, action_dt: float, chunk_size: int, action_dim: int):
        self._chunk_size = chunk_size
        self._action_dt = action_dt
        self._lock = threading.Lock()
        self._last_action_time = None
        self._buffer = np.zeros((chunk_size, action_dim), dtype=np.float32)

    def add_new_action_chunk(self, new_action_chunk: np.ndarray):
        if new_action_chunk.shape != (self._chunk_size, self._buffer.shape[1]):
            raise ValueError(
                f"New action chunk shape {new_action_chunk.shape} does not match buffer shape {self._buffer[0].shape if self._buffer else 'None'}."
            )
        with self._lock:
            if self._last_action_time is None:
                self._buffer = new_action_chunk.copy()
            else:
                self._fuse_action_chunks(new_action_chunk)
            self._last_action_time = time.perf_counter()

    def _fuse_action_chunks(self, action_chunks: np.ndarray) -> None:
        new_buffer = np.array(action_chunks, copy=True)
        for idx in range(len(self._buffer)):
            new_buffer[idx] = (action_chunks[idx] + self._buffer[idx]) * 0.5
        self._buffer = new_buffer

    def _get_current_action_chunk_int(self) -> int:
        return (
            int(
                (time.perf_counter() - self._last_action_time)
                / self._action_dt
            )
            if self._last_action_time is not None
            else 0
        )

    def get_action(self) -> np.ndarray:
        with self._lock:
            return np.array(self._buffer, copy=True)

    def apply_action(self) -> np.ndarray:
        with self._lock:
            current_index = self._get_current_action_chunk_int()
            action = np.array(
                self._buffer[min(current_index, self._chunk_size - 1)],
                copy=True,
            )
            self._buffer = np.array(self._buffer[current_index:], copy=True)
            return action

    def clear(self) -> None:
        with self._lock:
            self._buffer = np.zeros_like(self._buffer)
            self._last_action_time = None
