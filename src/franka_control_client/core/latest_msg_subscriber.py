from __future__ import annotations
from typing import TypeVar, Generic, Optional, Union, Any, Dict
import pyzlc
import time

MessageT = TypeVar("MessageT", bound=Union[Dict[str, Any], str])


class LatestMsgSubscriber(Generic[MessageT]):
    """A ZMQ SUB socket that keeps only the latest received message."""

    def __init__(
        self,
        topic_name: str,
        wait_for_first_message: bool = True,
        initial_message: Optional[MessageT] = None,
        wait_log_interval_s: float = 1.0,
    ) -> None:
        self.topic_name: str = topic_name
        self.last_message: Optional[MessageT] = initial_message
        pyzlc.register_subscriber_handler(
            self.topic_name, self._handle_message
        )
        if wait_for_first_message and self.last_message is None:
            last_log_time = 0.0
            while self.last_message is None:
                time.sleep(0.05)
                now = time.time()
                if (now - last_log_time) >= wait_log_interval_s:
                    pyzlc.info(
                        f"Waiting for first message on topic '{self.topic_name}'..."
                    )
                    last_log_time = now

    def _handle_message(self, msg: MessageT) -> None:
        self.last_message = msg

    def get_latest(self) -> Optional[MessageT]:
        return self.last_message

    def stop(self) -> None:
        """Stop the subscriber and cancel its running task."""
        raise NotImplementedError
