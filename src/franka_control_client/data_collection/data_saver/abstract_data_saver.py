import abc
from typing import Callable, Dict, Optional, Sequence, Tuple

from ..data_wrapper.wrapper import HardwareDataWrapper
from ..utils import VoidEvent


class AbstractDataSaver(abc.ABC):

    def __init__(
        self,
        data_collectors: Sequence[HardwareDataWrapper],
        task: str,
        fps: int = 50,
    ) -> None:
        self.data_collectors = data_collectors
        self.task = task
        self.fps = fps
        self.last_timestamp = None
        self._start_collecting_event = VoidEvent()
        self._stop_collecting_event = VoidEvent()

    def register_start_collecting_event(
        self, handler: Callable[[], None]
    ) -> None:
        self._start_collecting_event.subscribe(handler)

    def register_stop_collecting_event(
        self, handler: Callable[[], None]
    ) -> None:
        self._stop_collecting_event.subscribe(handler)

    @abc.abstractmethod
    def collect_step(self) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def start_collecting(self) -> None:
        self._start_collecting_event.emit()

    @abc.abstractmethod
    def save_episode(self) -> None:
        self.stop_collecting()

    @abc.abstractmethod
    def discard_collecting(self) -> None:
        self.stop_collecting()
        for collector in self.data_collectors:
            collector.discard()

    @abc.abstractmethod
    def stop_collecting(self) -> None:
        self._stop_collecting_event.emit()

    # @abc.abstractmethod
    # def _reset_arm(self) -> None:
    #     raise NotImplementedError

    def reset(self) -> None:
        for collector in self.data_collectors:
            collector.reset()

    def close(self) -> None:
        for collector in self.data_collectors:
            collector.close()
