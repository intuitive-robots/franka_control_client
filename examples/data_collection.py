import pyzlc

import time

from franka_control_client.camera.camera import CameraDevice
from franka_control_client.data_collection import DataCollectionManager
from franka_control_client.data_collection.wrapper import ImageDataWrapper


if __name__ == "__main__":
    pyzlc.init(
        "test_receive_camera_data",
        "192.168.0.134",
        group_name="hardware_collection",
    )
    camera = ImageDataWrapper(CameraDevice("depthai_camera", preview=True))
    name = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    data_collection_manager = DataCollectionManager(
        [camera], f"/home/xinkai/datasets/{name}", task="pick_and_place"
    )
    data_collection_manager.run()
    pyzlc.shutdown()
