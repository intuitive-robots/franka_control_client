from __future__ import annotations

import numpy as np
from typing import TypedDict, Optional, Tuple
import cv2

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

    def __init__(
        self,
        camera_name: str,
        preview: bool,
        final_size: Tuple[int, int] = None,
    ) -> None:
        super().__init__(camera_name)
        self.image_subscriber = LatestMsgSubscriber(f"{self._name}")
        self.preview = preview
        self.final_size = final_size
        latest_msg = self.image_subscriber.get_latest()
        assert (
            latest_msg is not None
        ), f"No image data received from camera device '{self._name}'."
        self.size: Tuple[int, int] = (
            latest_msg["height"],
            latest_msg["width"],
        )

    def get_image(self) -> Optional[np.ndarray]:
        """Get the latest RGB image from the camera."""
        frame: Optional[CameraFrame] = self.image_subscriber.get_latest()
        if frame is None:
            raise ValueError("No image data received from camera device.")
        image_array = np.frombuffer(frame["rgb_data"], dtype=np.uint8)
        image_array = image_array.reshape(
            (frame["height"], frame["width"], frame["channels"])
        )
        if self.preview:
            self.show_preview_rgb(image_array)
        if self.final_size is not None:
            image_array = cv2.resize(image_array, self.final_size)
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
