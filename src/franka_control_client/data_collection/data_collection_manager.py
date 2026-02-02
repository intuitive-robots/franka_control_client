import time
from enum import Enum
import numpy as np
from typing import Callable, Dict, Optional, List, Tuple
import queue
import pyzlc
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from concurrent.futures import Future

from .utils import NonBlockingKeyPress, UIConsole
from .wrapper import HarewareDataWrapper


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

    def __init__(
        self,
        data_collectors: List[HarewareDataWrapper],
        data_dir: str,
        task: str,
        fps: int = 50,
    ) -> None:
        self.data_collectors = data_collectors
        features = {}
        for collector in data_collectors:
            features.update(collector.feature)
        self.task = task
        self.fps = fps
        self.last_timestamp = None
        self.data_save_queue: queue.Queue[Optional[Dict[str, np.ndarray]]] = (
            queue.Queue()
        )
        self.data_save_future: Optional[Future] = None
        self.dataset: LeRobotDataset = LeRobotDataset.create(
            repo_id=data_dir, features=features, fps=self.fps
        )
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
        # self._ui_console.log(f"Sleeping for {sleep_time} seconds to maintain fps.")
        self.last_timestamp = time.perf_counter()

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
        self._ui_console.update_hint("Starting data collection...")
        assert self.data_save_future is None
        self.data_save_queue.empty()
        self.data_save_future = pyzlc.submit_thread_pool_task(
            self.__save_data_task
        )

    def __save_data_task(self) -> None:
        count = 0
        self._ui_console.log("Data saving task started.")
        try:
            while self._state_machine.state == DataCollectionState.COLLECTING:
                data = self.data_save_queue.get()
                if data is None:
                    break
                self.dataset.add_frame(data)
                count += 1
            self._ui_console.log(
                f"Data saving task ended, collected {count} frames."
            )
        except Exception as e:
            pyzlc.error(f"Error in data saving task: {e}")

    def __save_episode(self) -> None:
        self.__stop_collecting()
        self.dataset.save_episode()
        self.data_save_future = None
        self._ui_console.log("Episode saved.")

    def __discard_collecting(self) -> None:
        self.__stop_collecting()
        for collector in self.data_collectors:
            collector.discard()
        self._ui_console.log("Episode discarded.")

    def __stop_collecting(self) -> None:
        assert self.data_save_future is not None
        self.data_save_queue.put(None)  # signal to stop saving
        self.data_save_future.result()  # wait for saving to complete

    def __reset_to_waiting(self) -> None:
        for collector in self.data_collectors:
            collector.reset()
        self._state_machine.trigger(DataCollectionEvent.STAND_BY)

    def __close(self) -> None:
        for collector in self.data_collectors:
            collector.close()
        self._ui_console.update_hint("Data collectors closed.")
