import abc
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional

import numpy as np

from ..camera.camera import CameraDevice
from ..franka_robot.panda_arm import RemotePandaArm
from ..franka_robot.panda_gripper import RemotePandaGripper
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper


def _clone_buffer_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return np.array(value, copy=True)
    if isinstance(value, dict):
        return {key: _clone_buffer_value(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_clone_buffer_value(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_clone_buffer_value(item) for item in value)
    return value


class IRL_HardwareDataWrapper(abc.ABC):
    def __init__(
        self,
        hw_type: str,
        hw_name: str,
        capture_interval: float,
        buffer_capacity: int = 1,
    ) -> None:
        if capture_interval <= 0:
            raise ValueError(
                f"capture_interval must be > 0 for {hw_name}, got {capture_interval}"
            )
        if buffer_capacity < 1:
            raise ValueError(
                f"buffer_capacity must be >= 1 for {hw_name}, got {buffer_capacity}"
            )

        self.hw_type = hw_type
        self.hw_name = hw_name
        self.capture_interval = float(capture_interval)
        self._buffer_capacity = int(buffer_capacity)
        self._buffer: Deque[Any] = deque(maxlen=self._buffer_capacity)
        self._buffer_lock = threading.Lock()
        self._buffer_thread: Optional[threading.Thread] = None
        self._buffer_stop_event = threading.Event()
        self._last_capture_ts: Optional[float] = None

    @property
    def buffer_capacity(self) -> int:
        return self._buffer_capacity

    @property
    def buffered_length(self) -> int:
        with self._buffer_lock:
            return len(self._buffer)

    def set_buffer_capacity(self, buffer_capacity: int) -> None:
        if buffer_capacity < 1:
            raise ValueError(
                f"buffer_capacity must be >= 1 for {self.hw_name}, got {buffer_capacity}"
            )
        with self._buffer_lock:
            if buffer_capacity == self._buffer_capacity:
                return
            recent = list(self._buffer)[-buffer_capacity:]
            self._buffer_capacity = int(buffer_capacity)
            self._buffer = deque(recent, maxlen=self._buffer_capacity)

    def clear_buffer(self) -> None:
        with self._buffer_lock:
            self._buffer.clear()

    def update_buffer(self) -> Any:
        sample = self.capture_step()
        cloned_sample = _clone_buffer_value(sample)
        with self._buffer_lock:
            self._buffer.append(cloned_sample)
            self._last_capture_ts = time.time()
        return _clone_buffer_value(cloned_sample)

    def get_latest(self) -> Optional[Any]:
        with self._buffer_lock:
            if not self._buffer:
                return None
            return _clone_buffer_value(self._buffer[-1])

    def get_recent(
        self,
        count: int,
        *,
        require_full: bool = False,
        pad_with_oldest: bool = False,
    ) -> List[Any]:
        if count < 1:
            raise ValueError(f"count must be >= 1, got {count}")

        with self._buffer_lock:
            if not self._buffer:
                if require_full:
                    raise RuntimeError(
                        f"No buffered observations available for {self.hw_name}"
                    )
                return []
            samples = list(self._buffer)[-count:]

        if require_full and len(samples) < count:
            raise RuntimeError(
                f"{self.hw_name} has {len(samples)} buffered samples, requires {count}"
            )
        if pad_with_oldest and len(samples) < count:
            pad_sample = samples[0]
            samples = [pad_sample] * (count - len(samples)) + samples
        return [_clone_buffer_value(sample) for sample in samples]

    def start_buffering(self, *, clear_existing: bool = False) -> None:
        if clear_existing:
            self.clear_buffer()
        if self._buffer_thread is not None and self._buffer_thread.is_alive():
            return

        if self.buffered_length == 0:
            self.update_buffer()

        self._buffer_stop_event.clear()
        self._buffer_thread = threading.Thread(
            target=self._buffer_loop,
            name=f"{self.hw_name}_buffer",
            daemon=True,
        )
        self._buffer_thread.start()

    def stop_buffering(self) -> None:
        self._buffer_stop_event.set()
        if self._buffer_thread is not None and self._buffer_thread.is_alive():
            self._buffer_thread.join(timeout=max(1.0, 2.0 * self.capture_interval))
        self._buffer_thread = None

    def _buffer_loop(self) -> None:
        next_capture_ts = time.perf_counter()
        while not self._buffer_stop_event.is_set():
            try:
                self.update_buffer()
            except Exception:
                # Keep the sampling loop alive; callers can still inspect older samples.
                pass
            next_capture_ts += self.capture_interval
            sleep_time = next_capture_ts - time.perf_counter()
            if sleep_time <= 0:
                next_capture_ts = time.perf_counter()
                continue
            self._buffer_stop_event.wait(sleep_time)

    @abc.abstractmethod
    def capture_step(self) -> Any:
        raise NotImplementedError("Subclasses must implement capture_step method.")

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
    def __init__(
        self,
        camera_device: CameraDevice,
        hw_name: str,
        hw_type: str = "camera",
        capture_interval: float = 0.033,
        buffer_capacity: int = 1,
    ) -> None:
        self.camera_device = camera_device
        super().__init__(hw_type, hw_name, capture_interval, buffer_capacity)

    def capture_step(self) -> Optional[np.ndarray]:
        image_data = self.camera_device.get_image()
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
        self.clear_buffer()

    def reset(self) -> None:
        self.clear_buffer()

    def close(self) -> None:
        self.stop_buffering()


class PandaArmDataWrapper(IRL_HardwareDataWrapper):
    def __init__(
        self,
        arm: RemotePandaArm,
        hw_name: str = "FrankaPanda",
        hw_type: str = "follower_arm",
        capture_interval: float = 0.01,
        buffer_capacity: int = 1,
    ) -> None:
        self.arm = arm
        super().__init__(hw_type, hw_name, capture_interval, buffer_capacity)

    def capture_step(self) -> Dict[str, np.ndarray]:
        state = self.arm.current_state
        if state is None:
            raise ValueError("No arm state data received from the robot.")
        return state

    def __getattr__(self, name):
        return getattr(self.arm, name)

    def discard(self) -> None:
        self.clear_buffer()

    def reset(self) -> None:
        self.clear_buffer()

    def close(self) -> None:
        self.stop_buffering()


class PandaGripperDataWrapper(IRL_HardwareDataWrapper):
    def __init__(
        self,
        gripper: RemotePandaGripper,
        hw_name: str = "FrankaPanda",
        hw_type: str = "follower_gripper",
        capture_interval: float = 0.01,
        buffer_capacity: int = 1,
    ) -> None:
        self.gripper = gripper
        super().__init__(hw_type, hw_name, capture_interval, buffer_capacity)

    def capture_step(self) -> Dict[str, np.ndarray]:
        state = self.gripper.current_state
        if state is None:
            raise ValueError("No gripper state data received from the robot.")
        return state

    def discard(self) -> None:
        self.clear_buffer()

    def reset(self) -> None:
        self.clear_buffer()

    def close(self) -> None:
        self.stop_buffering()


class RobotiqGripperDataWrapper(IRL_HardwareDataWrapper):
    def __init__(
        self,
        gripper: RemoteRobotiqGripper,
        hw_name: str = "FrankaPanda",
        hw_type: str = "follower_gripper",
        capture_interval: float = 0.01,
        buffer_capacity: int = 1,
    ) -> None:
        self.gripper = gripper
        super().__init__(hw_type, hw_name, capture_interval, buffer_capacity)

    def capture_step(self) -> Dict[str, np.ndarray]:
        state = self.gripper.current_state
        if state is None:
            raise ValueError("No Robotiq gripper state data received.")
        return state

    def discard(self) -> None:
        self.clear_buffer()

    def reset(self) -> None:
        self.clear_buffer()

    def close(self) -> None:
        self.stop_buffering()

    def __getattr__(self, name):
        return getattr(self.gripper, name)
