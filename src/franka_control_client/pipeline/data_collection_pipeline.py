import time
import abc
from enum import Enum
from typing import Callable, Dict, Optional, List, Tuple


from .abstract_pipeline import (
    PipeLineState,
    PipeLineEvent,
    PipeLineStateMachine,
)
from ..utils import NonBlockingKeyPress, UIConsole, VoidEvent
from ..data_collection.data_saver.abstract_data_saver import AbstractDataSaver


class DataCollectionState(PipeLineState):
    WAITING = "waiting"
    COLLECTING = "collecting"
    EXITING = "exiting"


class DataCollectionEvent(PipeLineEvent):
    NEW_EPISODE = "new_episode"
    SAVE = "save"
    DISCARD = "discard"
    RESET = "reset"
    QUIT = "quit"


class DataCollectionManager(abc.ABC):

    def __init__(
        self,
        data_saver: AbstractDataSaver,
        task: str,
        fps: int = 50,
    ) -> None:
        self.data_saver = data_saver
        self.task = task
        self.fps = fps
        self.last_timestamp = None
        self._ui_console = UIConsole()
        self._start_collecting_event = VoidEvent()
        self._stop_collecting_event = VoidEvent()
        self._state_machine = PipeLineStateMachine(
            initial_state=DataCollectionState.WAITING,
        )
        self._state_machine.register_transition(
            DataCollectionState.WAITING,
            DataCollectionEvent.NEW_EPISODE,
            DataCollectionState.COLLECTING,
            action=self._start_collecting,
        )
        self._state_machine.register_transition(
            DataCollectionState.COLLECTING,
            DataCollectionEvent.SAVE,
            DataCollectionState.WAITING,
            action=self._save_episode,
        )
        self._state_machine.register_transition(
            DataCollectionState.COLLECTING,
            DataCollectionEvent.DISCARD,
            DataCollectionState.WAITING,
            action=self._discard_collecting,
        )
        self._state_machine.register_transition(
            DataCollectionState.WAITING,
            DataCollectionEvent.QUIT,
            DataCollectionState.EXITING,
            action=self._close,
        )
        self._state_machine.register_transition(
            DataCollectionState.COLLECTING,
            DataCollectionEvent.QUIT,
            DataCollectionState.EXITING,
            action=self._close,
        )
        self._state_machine.register_transition(
            DataCollectionState.WAITING,
            DataCollectionEvent.QUIT,
            DataCollectionState.EXITING,
            action=self._close,
        )
        self._state_machine.register_transition(
            DataCollectionState.WAITING,
            DataCollectionEvent.RESET,
            DataCollectionState.WAITING,
            action=self._reset,
        )
        self._state_machine.register_on_enter(self._on_state_enter)

    def register_start_collecting_event(
        self, handler: Callable[[], None]
    ) -> None:
        self._start_collecting_event.subscribe(handler)

    def register_stop_collecting_event(
        self, handler: Callable[[], None]
    ) -> None:
        self._stop_collecting_event.subscribe(handler)

    def run(self) -> None:
        self._on_state_enter(self._state_machine.state)
        try:
            with NonBlockingKeyPress() as kp:
                while self._state_machine.state != DataCollectionState.EXITING:
                    key = kp.get_data()
                    if key:
                        self._handle_keypress(key)
                    if (
                        self._state_machine.state
                        == DataCollectionState.COLLECTING
                    ):
                        self._collect_step()
        finally:
            self._close()

    def _handle_keypress(self, key: str) -> None:
        if key == "n":
            self._state_machine.trigger(DataCollectionEvent.NEW_EPISODE)
        elif key == "s":
            self._state_machine.trigger(DataCollectionEvent.SAVE)
        elif key == "d":
            self._state_machine.trigger(DataCollectionEvent.DISCARD)
        elif key == "q":
            self._state_machine.trigger(DataCollectionEvent.QUIT)
        elif key == "r":
            self._state_machine.trigger(DataCollectionEvent.RESET)

    def _on_state_enter(self, state: DataCollectionState) -> None:
        if state == DataCollectionState.WAITING:
            self._ui_console.update_hint(
                "Press 'n' to start collecting, 'r' to reset, or 'q' to quit"
            )
        elif state == DataCollectionState.COLLECTING:
            self._ui_console.update_hint(
                "Collecting... Press 's' to save, 'd' to discard, or 'q' to quit"
            )
        elif state == DataCollectionState.STOPPED:
            self._ui_console.update_hint("Collecting stopped. Resetting...")
        elif state == DataCollectionState.EXITING:
            self._ui_console.update_hint("Exiting data collection")

    @abc.abstractmethod
    def _collect_step(self) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def _start_collecting(self) -> None:
        self._ui_console.update_hint("Starting data collection...")
        self._start_collecting_event.emit()

    @abc.abstractmethod
    def _save_episode(self) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def _discard_collecting(self) -> None:
        self._stop_collecting()
        for collector in self.data_collectors:
            collector.discard()
        self._ui_console.log("Episode discarded.")

    @abc.abstractmethod
    def _stop_collecting(self) -> None:
        self._ui_console.update_hint("Stopping data collection...")
        self._stop_collecting_event.emit()

    @abc.abstractmethod
    def _reset(self) -> None:
        raise NotImplementedError

    def _reset_to_waiting(self) -> None:
        for collector in self.data_collectors:
            collector.reset()
        self._state_machine.trigger(DataCollectionEvent.STAND_BY)

    def _close(self) -> None:
        for collector in self.data_collectors:
            collector.close()
        self._ui_console.update_hint("Data collectors closed.")
