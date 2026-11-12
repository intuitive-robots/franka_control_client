import time
import abc
import shutil
from enum import Enum
from pathlib import Path
from typing import Callable, Dict, Optional, List, Tuple


from .utils import NonBlockingKeyPress, UIConsole, VoidEvent
from .wrapper import HardwareDataWrapper
import pyzlc

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
    RESET = "reset"
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
        self._start_collecting_event = VoidEvent()
        self._stop_collecting_event = VoidEvent()
        self._input_sources: List[Callable[[], List[str]]] = []
        self._last_saved_dir: Optional[Path] = None  # deletable with 'x' until the next episode starts
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
        self._state_machine.register_transition(
            DataCollectionState.WAITING,
            DataCollectionEvent.RESET,
            DataCollectionState.WAITING,
            action=self._reset_arm,
        )

    def register_start_collecting_event(
        self, handler: Callable[[], None]
    ) -> None:
        self._start_collecting_event.subscribe(handler)

    def register_stop_collecting_event(
        self, handler: Callable[[], None]
    ) -> None:
        self._stop_collecting_event.subscribe(handler)

    def register_input_source(self, poll: Callable[[], List[str]]) -> None:
        """Add a non-keyboard input (e.g. controller buttons).

        ``poll`` is called every loop iteration and returns the keys to handle, as if typed.
        """
        self._input_sources.append(poll)

    def run(self) -> None:
        self._on_state_enter(self._state_machine.state)
        try:
            with NonBlockingKeyPress() as kp:
                while self._state_machine.state != DataCollectionState.EXITING:
                    key = kp.get_data()
                    if key:
                        self._handle_keypress(key)
                    for poll in self._input_sources:
                        for key in poll():
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
                    # time.sleep(0.001)
        finally:
            # stop a teleop loop left on (q / Ctrl-C), its non-daemon thread would keep the process alive
            control_pair = getattr(self, "control_pair", None)
            if control_pair is not None:
                control_pair.stop_control_pair()
            self._close()

    def _handle_keypress(self, key: str) -> None:
        if key == "n":
            self._state_machine.trigger(DataCollectionEvent.NEW_EPISODE)
        elif key == "s":
            self._state_machine.trigger(DataCollectionEvent.SAVE)
        elif key == "d":
            self._state_machine.trigger(DataCollectionEvent.DISCARD)
        elif key == "x":
            # delete the running episode, or the one saved just before (only that one, once)
            if self._state_machine.state == DataCollectionState.COLLECTING:
                self._state_machine.trigger(DataCollectionEvent.DISCARD)
            elif self._state_machine.state == DataCollectionState.WAITING:
                self._delete_last_episode()
        elif key == "q":
            self._state_machine.trigger(DataCollectionEvent.QUIT)
        elif key == "r":
            # the reset motion must not fight a running teleop loop
            if self._state_machine.state == DataCollectionState.WAITING:
                self._set_teleop(False)
            self._state_machine.trigger(DataCollectionEvent.RESET)
        elif key == "t":
            if self._state_machine.state == DataCollectionState.WAITING:
                control_pair = getattr(self, "control_pair", None)
                self._set_teleop(not (control_pair is not None and control_pair.is_running))
        elif key == "o":
            control_pair = getattr(self, "control_pair", None)
            if control_pair is not None and hasattr(control_pair, "open_gripper"):
                control_pair.open_gripper()
        elif key == "p":
            control_pair = getattr(self, "control_pair", None)
            if control_pair is not None and hasattr(control_pair, "close_gripper"):
                control_pair.close_gripper()

    def _delete_last_episode(self) -> None:
        """Delete the episode saved last, if nothing was started since.

        Subclasses set ``self._last_saved_dir`` to the episode directory in ``_save_episode``.
        """
        path, self._last_saved_dir = self._last_saved_dir, None
        if path is None or not Path(path).exists():
            self._ui_console.log("No episode to delete (only the one saved last, before a new one starts).")
            return
        shutil.rmtree(path)
        self._ui_console.log(f"Deleted episode {Path(path).name}.")

    def _set_teleop(self, enabled: bool) -> None:
        """Run the control pair while WAITING, without recording.

        Starting an episode keeps it running; saving/discarding stops it unless the subclass was
        created with stop_control_on_episode_end=False.
        """
        control_pair = getattr(self, "control_pair", None)
        if control_pair is None:
            return
        if enabled and not control_pair.is_running:
            control_pair.start_control_pair()
            self._ui_console.log("Teleop ON (not recording).")
        elif not enabled and control_pair.is_running:
            control_pair.stop_control_pair()
            self._ui_console.log("Teleop OFF.")
        self._on_state_enter(self._state_machine.state)

    def _on_state_enter(self, state: DataCollectionState) -> None:
        if state == DataCollectionState.WAITING:
            control_pair = getattr(self, "control_pair", None)
            teleop = "ON" if control_pair is not None and control_pair.is_running else "OFF"
            self._ui_console.update_hint(
                f"[teleop {teleop}] Press 'n' to start collecting, 't' to toggle teleop, 'r' to reset, "
                "'x' to delete the last episode, 'o' to open gripper, 'p' to close gripper, or 'q' to quit"
            )
        elif state == DataCollectionState.COLLECTING:
            self._ui_console.update_hint(
                "Collecting... Press 'o' to open gripper, 'p' to close gripper, 's' to save, 'd'/'x' to discard, or 'q' to quit"
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
        self._last_saved_dir = None
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
    def _reset_arm(self) -> None:
        raise NotImplementedError

    def _reset_to_waiting(self) -> None:
        for collector in self.data_collectors:
            collector.reset()
        self._state_machine.trigger(DataCollectionEvent.STAND_BY)

    def _close(self) -> None:
        for collector in self.data_collectors:
            collector.close()
        self._ui_console.update_hint("Data collectors closed.")
