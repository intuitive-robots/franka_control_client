import time
import abc
from enum import Enum
from typing import Callable, Dict, Optional, List, Tuple


from .utils import NonBlockingKeyPress, UIConsole
from .wrapper import HardwareDataWrapper


class DataCollectionState(str, Enum):
    WAITING = "waiting"
    COLLECTING = "collecting"
    STOPPED = "stopped"
    EXITING = "exiting"


class DataCollectionEvent(str, Enum):
    NEW_EPISODE = "new_episode"
    SAVE = "save"
    DISCARD = "discard"
    STAND_BY = "stand_by"
    QUIT = "quit"


Transition = Tuple[DataCollectionState, Optional[Callable[[], None]]]
StateEventPair = Tuple[DataCollectionState, DataCollectionEvent]


class DataCollectionStateMachine:
    def __init__(
        self,
        initial_state: DataCollectionState,
        on_enter: Optional[Callable[[DataCollectionState], None]] = None,
    ) -> None:
        self._state = initial_state
        self._transitions: Dict[StateEventPair, Transition] = {}
        self._on_enter = on_enter

    @property
    def state(self) -> DataCollectionState:
        return self._state

    def register_transition(
        self,
        from_state: DataCollectionState,
        event: DataCollectionEvent,
        to_state: DataCollectionState,
        action: Optional[Callable[[], None]] = None,
    ) -> None:
        self._transitions[(from_state, event)] = (to_state, action)

    def trigger(self, event: DataCollectionEvent) -> bool:
        transition = self._transitions.get((self._state, event))
        if transition is None:
            return False

        next_state, action = transition
        if action is not None:
            action()

        self._state = next_state
        if self._on_enter is not None:
            self._on_enter(next_state)
        return True


class DataCollectionManager(abc.ABC):

    def __init__(
        self,
        data_collectors: List[HardwareDataWrapper],
        task: str,
        fps: int = 50,
    ) -> None:
        self.data_collectors = data_collectors
        self.task = task
        self.fps = fps
        self.last_timestamp = None
        self._ui_console = UIConsole()
        self._state_machine = DataCollectionStateMachine(
            initial_state=DataCollectionState.WAITING,
            on_enter=self._on_state_enter,
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
            DataCollectionState.STOPPED,
            action=self._save_episode,
        )
        self._state_machine.register_transition(
            DataCollectionState.COLLECTING,
            DataCollectionEvent.DISCARD,
            DataCollectionState.STOPPED,
            action=self._discard_collecting,
        )
        self._state_machine.register_transition(
            DataCollectionState.STOPPED,
            DataCollectionEvent.STAND_BY,
            DataCollectionState.WAITING,
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
            DataCollectionState.STOPPED,
            DataCollectionEvent.QUIT,
            DataCollectionState.EXITING,
            action=self._close,
        )

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
                    if (
                        self._state_machine.state
                        == DataCollectionState.STOPPED
                    ):
                        self._reset_to_waiting()
                    time.sleep(0.01)
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

    def _on_state_enter(self, state: DataCollectionState) -> None:
        if state == DataCollectionState.WAITING:
            self._ui_console.update_hint(
                "Press 'n' to start collecting, or 'q' to quit"
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
        raise NotImplementedError

    def _reset_to_waiting(self) -> None:
        for collector in self.data_collectors:
            collector.reset()
        self._state_machine.trigger(DataCollectionEvent.STAND_BY)

    def _close(self) -> None:
        for collector in self.data_collectors:
            collector.close()
        self._ui_console.update_hint("Data collectors closed.")
