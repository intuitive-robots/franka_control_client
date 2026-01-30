from __future__ import annotations

import numpy as np
from typing import TypedDict

from ..core.remote_device import RemoteDevice
from ..core.latest_msg_subscriber import LatestMsgSubscriber


class CameraFrame(TypedDict):
    timestamp: float
    width: int
    height: int
    channels: int
    image_data: bytes


class CameraDevice(RemoteDevice):

    def __init__(self, camera_name: str) -> None:
        super().__init__(camera_name)
        self.image_subscriber = LatestMsgSubscriber(
            f"{self._name}/camera_frame"
        )

    def get_image(self) -> np.ndarray:
        """Get the latest RGB image from the camera."""
        msg = self.image_subscriber.get_latest()
        frame: CameraFrame = msg  # type: ignore
        image_array = np.frombuffer(frame["image_data"], dtype=np.uint8)
        image_array = image_array.reshape(
            (frame["height"], frame["width"], frame["channels"])
        )
        return image_array
