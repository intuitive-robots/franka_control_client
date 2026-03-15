import time
import abc
from enum import Enum
from typing import Callable, Dict, Optional, List, Tuple


from ..utils import NonBlockingKeyPress, UIConsole, VoidEvent

from ..data_collection.data_collection_manager import DataCollectionState


class PipeLineState(str, Enum):
    pass


class PipeLineEvent(str, Enum):
    pass


Transition = Tuple[PipeLineState, Optional[Callable[[], None]]]
StateEventPair = Tuple[PipeLineState, PipeLineEvent]


class PipeLineStateMachine:
    def __init__(self, initial_state: PipeLineState) -> None:
        self._state = initial_state
        self._transitions: Dict[StateEventPair, Transition] = {}
        self._on_enter: Dict[PipeLineState, Callable[[], None]] = {}

    @property
    def state(self) -> PipeLineState:
        return self._state

    def register_transition(
        self,
        from_state: PipeLineState,
        event: PipeLineEvent,
        to_state: PipeLineState,
        action: Optional[Callable[[], None]] = None,
    ) -> None:
        self._transitions[(from_state, event)] = (to_state, action)

    def register_on_enter(
        self, state: PipeLineState, handler: Callable[[], None]
    ) -> None:
        self._on_enter[state] = handler

    def trigger(self, event: PipeLineEvent) -> bool:
        transition = self._transitions.get((self._state, event))
        if transition is None:
            return False

        next_state, action = transition
        if action is not None:
            action()

        self._state = next_state
        if next_state in self._on_enter.keys():
            return False
        action = self._on_enter[next_state]
        if action is not None:
            action()
        return True


class AbstractPipeLineManager(abc.ABC):

    def __init__(self) -> None:
        self._ui_console = UIConsole()

    @abc.abstractmethod
    def run(self) -> None:
        raise NotImplementedError
