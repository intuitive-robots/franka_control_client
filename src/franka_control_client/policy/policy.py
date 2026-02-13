from __future__ import annotations

from typing import TypedDict, Optional, Dict, Any
import pyzlc

from ..core.remote_device import RemoteDevice
from ..core.latest_msg_subscriber import LatestMsgSubscriber

#build connection for inference node with policy node
class PolicyActionMsg(TypedDict, total=True):
    timestamp: float
    action: list[float]
    shape: list[int]


class PolicyObservationMsg(TypedDict, total=True):
    state: list[float]
    images: Dict[str, Any]
    task: str | None

class RemotePolicy(RemoteDevice):
    """Remote client for a policy node."""

    def __init__(
        self,
        device_name: str,
        obs_topic: Optional[str] = None,
        action_topic: Optional[str] = None,
    ) -> None:
        super().__init__(device_name)
        if obs_topic is None:
            obs_topic = f"{self._name}/policy_observation"
        if action_topic is None:
            action_topic = f"{self._name}/policy_action"
        self.obs_publisher = pyzlc.Publisher(
            obs_topic,
        )
        self.action_subscriber = LatestMsgSubscriber(
            action_topic
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
    #if put this in policy node, directly get observations,policy part need few of subscriber, if put here just need one subscriber，maybe save time fore policy_node
    def send_observation(self, obs: PolicyObservationMsg) -> None:
        """Send observation."""
        self.obs_publisher.publish(obs)
