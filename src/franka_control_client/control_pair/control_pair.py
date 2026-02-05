import abc
from typing import Optional
import threading


class ControlPair(abc.ABC):

    def __init__(self) -> None:
        self.control_task_thread: Optional[threading.Thread] = None
        self.is_running: bool = False

    def start_control_pair(self) -> None:
        if self.is_running:
            return
        self.is_running = True
        self.control_task_thread = threading.Thread(target=self._control_task)
        self.control_task_thread.start()

    def stop_control_pair(self) -> None:
        if not self.is_running:
            return
        self.is_running = False
        if self.control_task_thread is not None:
            self.control_task_thread.join()
            self.control_task_thread = None

    @abc.abstractmethod
    def control_step(self) -> None:
        raise NotImplementedError(
            "Subclasses must implement control_step method."
        )

    def _control_task(self) -> None:
        try:
            while self.is_running:
                self.control_step()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
