import time

import pyzlc

from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)


if __name__ == "__main__":
    # Update these to match your ZeroLanCom node and IP.
    pyzlc.init("robotiq_control_client", "192.168.0.117", group_name="RobotControlGroup")
    gripper = RemoteRobotiqGripper("FrankaPanda")
    gripper.connect()

    # Simple open/close demo.
    # gripper.open(speed=0.1, force=0.1)
    # time.sleep(1.0)
    gripper.close(speed=0.1, force=0.1)
    print("Gripper closing...")
    time.sleep(5.0)

    # Print the latest state if available.
    try:
        state = gripper.current_state
    except AssertionError:
        state = None
    if state is not None:
        print("Robotiq gripper state:", state)

    for _ in range(100):
        time.sleep(0.1)
