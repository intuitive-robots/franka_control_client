from __future__ import annotations
from typing import TypeVar, Generic, Optional, Union, Any, Dict
import pyzlc
import time

MessageT = TypeVar("MessageT", bound=Union[Dict[str, Any], str])


class LatestMsgSubscriber(Generic[MessageT]):
    """A ZMQ SUB socket that keeps only the latest received message."""

    def __init__(self, topic_name: str) -> None:
        self.topic_name: str = topic_name
        pyzlc.register_subscriber_handler(
            self.topic_name, self._handle_message
        )
        self.last_message: Optional[MessageT] = None
        while self.last_message is None:
            time.sleep(1)
        pyzlc.info(
            f"Waiting for first message on topic '{self.topic_name}'..."
        )

    def _handle_message(self, msg: MessageT) -> None:
        self.last_message = msg

    def get_latest(self) -> Optional[MessageT]:
        return self.last_message

    def stop(self) -> None:
        """Stop the subscriber and cancel its running task."""
        raise NotImplementedError
