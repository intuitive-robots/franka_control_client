import time
import abc
from enum import Enum
from typing import Callable, Dict, Optional, List, Tuple


from .utils import NonBlockingKeyPress, UIConsole, VoidEvent
# from .wrapper import HardwareDataWrapper
import pyzlc

class PolicyInferenceState(str, Enum):
    WAITING = "waiting"
    INFERING = "infering"
    STOPPED = "stopped"
    EXITING = "exiting"


class PolicyInferenceEvent(str, Enum):
    NEW_EPISODE = "new_episode"
    SAVE = "save"
    DISCARD = "discard"
    STAND_BY = "stand_by"
    QUIT = "quit"


Transition = Tuple[PolicyInferenceState, Optional[Callable[[], None]]]
StateEventPair = Tuple[PolicyInferenceState, PolicyInferenceEvent]


class PolicyInferenceStateMachine:
    def __init__(
        self,
        initial_state: PolicyInferenceState,
        on_enter: Optional[Callable[[PolicyInferenceState], None]] = None,
    ) -> None:
        self._state = initial_state
        self._transitions: Dict[StateEventPair, Transition] = {}
        self._on_enter = on_enter

    @property
    def state(self) -> PolicyInferenceState:
        return self._state

    def register_transition(
        self,
        from_state: PolicyInferenceState,
        event: PolicyInferenceEvent,
        to_state: PolicyInferenceState,
        action: Optional[Callable[[], None]] = None,
    ) -> None:
        self._transitions[(from_state, event)] = (to_state, action)

    def trigger(self, event: PolicyInferenceEvent) -> bool:
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


class PolicyInferenceManager(abc.ABC):

    def __init__(
        self,
        # data_collectors: List[HardwareDataWrapper],
        task: str,
        fps: int = 50,
    ) -> None:
        # self.data_collectors = data_collectors
        self.task = task
        self.fps = fps
        self.last_timestamp = None
        self._ui_console = UIConsole()
        self._start_infering_event = VoidEvent()
        self._stop_infering_event = VoidEvent()
        self._state_machine = PolicyInferenceStateMachine(
            initial_state=PolicyInferenceState.WAITING,
            on_enter=self._on_state_enter,
        )
        self._state_machine.register_transition(
            PolicyInferenceState.WAITING,
            PolicyInferenceEvent.NEW_EPISODE,
            PolicyInferenceState.INFERING,
            action=self._start_infering,
        )
        self._state_machine.register_transition(
            PolicyInferenceState.INFERING,
            PolicyInferenceEvent.SAVE,
            PolicyInferenceState.STOPPED,
            action=self._save_episode,
        )
        self._state_machine.register_transition(
            PolicyInferenceState.INFERING,
            PolicyInferenceEvent.DISCARD,
            PolicyInferenceState.STOPPED,
            action=self._discard_infering,
        )
        self._state_machine.register_transition(
            PolicyInferenceState.STOPPED,
            PolicyInferenceEvent.STAND_BY,
            PolicyInferenceState.WAITING,
        )
        self._state_machine.register_transition(
            PolicyInferenceState.WAITING,
            PolicyInferenceEvent.QUIT,
            PolicyInferenceState.EXITING,
            action=self._close,
        )
        self._state_machine.register_transition(
            PolicyInferenceState.INFERING,
            PolicyInferenceEvent.QUIT,
            PolicyInferenceState.EXITING,
            action=self._close,
        )
        self._state_machine.register_transition(
            PolicyInferenceState.STOPPED,
            PolicyInferenceEvent.QUIT,
            PolicyInferenceState.EXITING,
            action=self._close,
        )

    def register_start_infering_event(
        self, handler: Callable[[], None]
    ) -> None:
        self._start_infering_event.subscribe(handler)

    def register_stop_infering_event(
        self, handler: Callable[[], None]
    ) -> None:
        self._stop_infering_event.subscribe(handler)

    def run(self) -> None:
        self._on_state_enter(self._state_machine.state)
        try:
            with NonBlockingKeyPress() as kp:
                while self._state_machine.state != PolicyInferenceState.EXITING:
                    key = kp.get_data()
                    if key:
                        self._handle_keypress(key)
                    if (
                        self._state_machine.state
                        == PolicyInferenceState.INFERING
                    ):
                        self._infer_step()
                    if (
                        self._state_machine.state
                        == PolicyInferenceState.STOPPED
                    ):
                        self._reset_to_waiting()
                    # time.sleep(0.001)
        finally:
            self._close()

    def _handle_keypress(self, key: str) -> None:
        if key == "n":
            self._state_machine.trigger(PolicyInferenceEvent.NEW_EPISODE)
        elif key == "s":
            self._state_machine.trigger(PolicyInferenceEvent.SAVE)
        elif key == "d":
            self._state_machine.trigger(PolicyInferenceEvent.DISCARD)
        elif key == "q":
            self._state_machine.trigger(PolicyInferenceEvent.QUIT)

    def _on_state_enter(self, state: PolicyInferenceState) -> None:
        if state == PolicyInferenceState.WAITING:
            self._ui_console.update_hint(
                "Press 'n' to start infering, or 'q' to quit"
            )
        elif state == PolicyInferenceState.INFERING:
            self._ui_console.update_hint(
                "Infering... Press 's' to save, 'd' to discard, or 'q' to quit"
            )
        elif state == PolicyInferenceState.STOPPED:
            self._ui_console.update_hint("Infering stopped. Resetting...")
        elif state == PolicyInferenceState.EXITING:
            self._ui_console.update_hint("Exiting policy inference")

    @abc.abstractmethod
    def _infer_step(self) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def _start_infering(self) -> None:
        self._ui_console.update_hint("Starting policy infering...")
        self._start_infering_event.emit()

    @abc.abstractmethod
    def _save_episode(self) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def _discard_infering(self) -> None:
        self._stop_infering()
        #todo:inferencer
        # for collector in self.data_collectors:
        #     collector.discard()
        self._ui_console.log("Episode discarded.")

    @abc.abstractmethod
    def _stop_infering(self) -> None:
        self._ui_console.update_hint("Stopping policy inference...")
        self._stop_infering_event.emit()

    def _reset_to_waiting(self) -> None:
        #todo:inferencer
        # for collector in self.data_collectors:
        #     collector.reset()
        self._state_machine.trigger(PolicyInferenceEvent.STAND_BY)

    def _close(self) -> None:
        #todo:inferencer
        # for collector in self.data_collectors:
        #     collector.close()
        self._ui_console.update_hint("Policy infering closed.")
