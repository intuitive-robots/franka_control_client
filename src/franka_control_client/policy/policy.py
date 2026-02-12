from __future__ import annotations

from typing import TypedDict, Optional
import pyzlc

from ..core.remote_device import RemoteDevice
from ..core.latest_msg_subscriber import LatestMsgSubscriber

#build connection for inference node with policy node
class PolicyActionMsg(TypedDict, total=True):
    timesamp: float
    action: list[float]
    shape: list[int]


class PolicyObservationMsg(TypedDict, total=True):
    state: list[float]
    #for each camera
    width: int
    height: int
    channels: int
    rgb_data: bytes
    task: str | None

class RemotePolicy(RemoteDevice):
    """Remote client for a gripper device."""

    def __init__(self, device_name: str) -> None:
        super().__init__(device_name)
        self.obs_publisher = pyzlc.Publisher(
            f"{self._name}/policy_observation",
        )
        self.action_subscriber = LatestMsgSubscriber(
            f"{self._name}/policy_action"
        )

    @property
    def current_action(self) -> Optional[PolicyActionMsg]:
        """Return the latest action."""
        msg = self.action_subscriber.last_message
        if msg is None:
            return None
        return PolicyActionMsg(
            timestamp=msg["timestamp"],
            action=msg["action"],
            shape=msg["shape"],
        )
    #maybe put this in policy node, directly get observations 
    def send_observation(self, ) -> None:
        """Send observation."""
        self.obs_publisher.publish(PolicyObservationMsg())
