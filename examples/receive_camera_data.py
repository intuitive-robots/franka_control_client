import pyzlc
from franka_control_client.camera.camera import CameraDevice


if __name__ == "__main__":
    pyzlc.init(
        "test_receive_camera_data",
        "127.0.0.1",
        group_name="hardware_collection",
    )
    camera = CameraDevice("depthai_camera", preview=True)
    try:
        while True:
            img = camera.get_image()
    except KeyboardInterrupt:
        pass
