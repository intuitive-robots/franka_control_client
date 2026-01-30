import time
from enum import Enum
from typing import Callable, Dict, Optional, List, Tuple

# from utils.keyboard_input import NonBlockingKeyPress
from .utils import DataCollector, NonBlockingKeyPress, UIConsole


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


class DataCollectionManager:

    def __init__(self, data_collectors: List[DataCollector]) -> None:
        self._last_capture_ts = 0.0
        self.data_collectors = data_collectors
        self._ui_console = UIConsole()
        self._state_machine = DataCollectionStateMachine(
            initial_state=DataCollectionState.WAITING,
            on_enter=self._on_state_enter,
        )
        self._state_machine.register_transition(
            DataCollectionState.WAITING,
            DataCollectionEvent.NEW_EPISODE,
            DataCollectionState.COLLECTING,
            action=self.__start_collecting,
        )
        self._state_machine.register_transition(
            DataCollectionState.COLLECTING,
            DataCollectionEvent.SAVE,
            DataCollectionState.STOPPED,
            action=self.__save_episode,
        )
        self._state_machine.register_transition(
            DataCollectionState.COLLECTING,
            DataCollectionEvent.DISCARD,
            DataCollectionState.STOPPED,
            action=self.__discard_collecting,
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
            action=self.__close,
        )
        self._state_machine.register_transition(
            DataCollectionState.COLLECTING,
            DataCollectionEvent.QUIT,
            DataCollectionState.EXITING,
            action=self.__close,
        )
        self._state_machine.register_transition(
            DataCollectionState.STOPPED,
            DataCollectionEvent.QUIT,
            DataCollectionState.EXITING,
            action=self.__close,
        )

    def run(self) -> None:
        self._on_state_enter(self._state_machine.state)
        with NonBlockingKeyPress() as kp:
            while self._state_machine.state != DataCollectionState.EXITING:
                key = kp.get_data()
                if key:
                    self._handle_keypress(key)
                if self._state_machine.state == DataCollectionState.COLLECTING:
                    self.__collect_step()
                    time.sleep(1)  # Adjust sleep time as needed
                if self._state_machine.state == DataCollectionState.STOPPED:
                    self.__reset_to_waiting()
                time.sleep(0.01)

    def _handle_keypress(self, key: str) -> None:
        if key == "n":
            self._state_machine.trigger(DataCollectionEvent.NEW_EPISODE)
        elif key == "s":
            self._state_machine.trigger(DataCollectionEvent.SAVE)
        elif key == "d":
            self._state_machine.trigger(DataCollectionEvent.DISCARD)
        elif key == "q":
            self._state_machine.trigger(DataCollectionEvent.QUIT)

    def __collect_step(self) -> None:
        for collector in self.data_collectors:
            collector.save_step()
        self._ui_console.log("Step collected.")

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

    def __start_collecting(self) -> None:
        self._last_capture_ts = 0.0
        self._ui_console.update_hint("Starting data collection...")

    def __save_episode(self) -> None:
        for collector in self.data_collectors:
            collector.save_episode()
        self._ui_console.log("Episode saved.")
        self.__reset_to_waiting()

    def __discard_collecting(self) -> None:
        for collector in self.data_collectors:
            collector.discard()
        self._ui_console.log("Episode discarded.")
        self.__reset_to_waiting()

    def __reset_to_waiting(self) -> None:
        for collector in self.data_collectors:
            collector.reset()
        self._state_machine.trigger(DataCollectionEvent.STAND_BY)

    def __close(self) -> None:
        for collector in self.data_collectors:
            collector.close()
        self._ui_console.update_hint("Data collectors closed.")
