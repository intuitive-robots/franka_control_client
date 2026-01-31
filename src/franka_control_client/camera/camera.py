from __future__ import annotations

import numpy as np
from typing import TypedDict, Optional
import cv2
import time

from ..core.remote_device import RemoteDevice
from ..core.latest_msg_subscriber import LatestMsgSubscriber


class CameraFrame(TypedDict):
    timestamp: float
    width: int
    height: int
    channels: int
    rgb_data: bytes
    depth_data: Optional[bytes]


class CameraDevice(RemoteDevice):

    def __init__(self, camera_name: str, preview: bool) -> None:
        super().__init__(camera_name)
        self.image_subscriber = LatestMsgSubscriber(f"{self._name}")
        self.preview = preview

    def get_image(self) -> Optional[np.ndarray]:
        """Get the latest RGB image from the camera."""
        frame: Optional[CameraFrame] = self.image_subscriber.get_latest()
        if frame is None:
            return
        image_array = np.frombuffer(frame["rgb_data"], dtype=np.uint8)
        image_array = image_array.reshape(
            (frame["height"], frame["width"], frame["channels"])
        )
        if self.preview:
            self.show_preview_rgb(image_array)
        return image_array

    def show_preview_rgb(self, img_mat: np.ndarray) -> None:
        """Show a preview of the RGB image frame using OpenCV.

        Args:
            frame (CameraFrame): The captured image frame.
        """
        cv2.imshow(f"{self._name} Preview", img_mat)
        cv2.waitKey(1)

    # TODO: implement depth preview
    def show_preview_rgbd(self, frame: CameraFrame) -> None:
        raise NotImplementedError(
            "Subclasses must implement show_preview_rgbd method."
        )
