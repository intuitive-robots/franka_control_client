import pyzlc
import queue
import time
from typing import Dict, Optional, Sequence
import numpy as np
from concurrent.futures import Future

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from ..data_wrapper.lerobot_wrapper import LeRobotGelloDataWrapper
from .abstract_data_saver import AbstractDataSaver


class LeRobotDataCollection(AbstractDataSaver):
    def __init__(
        self,
        data_collectors: Sequence[LeRobotGelloDataWrapper],
        data_dir: str,
        task: str,
        fps: int = 50,
    ) -> None:
        super().__init__(data_collectors, task, fps)
        features = {}
        for collector in data_collectors:
            features.update(collector.feature)
        self.dataset: LeRobotDataset = LeRobotDataset.create(
            repo_id=data_dir, features=features, fps=self.fps
        )
        self.dataset.meta.metadata_buffer_size = 1
        self.data_save_queue: queue.Queue[Optional[Dict[str, np.ndarray]]] = (
            queue.Queue()
        )
        self.data_save_future: Optional[Future] = None

    def start_collecting(self) -> None:
        # Emit start-collection event (e.g., start control pair).
        super().start_collecting()
        assert self.data_save_future is None
        # empty the queue
        while not self.data_save_queue.empty():
            self.data_save_queue.get()
        self.data_save_future = pyzlc.submit_thread_pool_task(
            self._save_data_task
        )

    def _save_data_task(self) -> None:
        count = 0
        try:
            while True:
                data = self.data_save_queue.get()
                if data is None:
                    break
                self.dataset.add_frame(data)
                count += 1
        except Exception as e:
            pyzlc.error(f"Error in data saving task: {e}")

    def _collect_step(self) -> None:
        if self.last_timestamp is None:
            self.last_timestamp = time.perf_counter()
        payload = {}
        for collector in self.data_collectors:
            payload.update(collector.capture_step())
        payload["task"] = self.task
        self.data_save_queue.put(payload)
        self.last_timestamp = time.perf_counter()
        sleep_time = max(
            0, 1.0 / self.fps - (time.perf_counter() - self.last_timestamp)
        )
        time.sleep(sleep_time)
        self.last_timestamp = time.perf_counter()

    def save_episode(self) -> None:
        super().save_episode()
        self.dataset.save_episode()

    def discard_collecting(self) -> None:
        self.stop_collecting()
        for collector in self.data_collectors:
            collector.discard()

    def stop_collecting(self) -> None:
        super().stop_collecting()
        assert self.data_save_future is not None
        self.data_save_queue.put(None)  # signal to stop saving
        self.data_save_future.result()  # wait for saving to complete
        self.data_save_future = None

    def close(self) -> None:
        # Ensure the background saver thread can't hang on Queue.get() when exiting.
        if self.data_save_future is not None:
            try:
                self.data_save_queue.put(None)
                self.data_save_future.result()
            except Exception as e:
                pyzlc.error(f"Error while stopping data saving task: {e}")
            finally:
                self.data_save_future = None

        try:
            # Flush/close parquet writers so meta/episodes/...parquet is written even for < metadata_buffer_size episodes.
            self.dataset.finalize()
        except Exception as e:
            pyzlc.error(f"Error finalizing dataset: {e}")
        super().close()
