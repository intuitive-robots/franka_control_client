import time

import pyzlc

from franka_control_client.franka_robot.franka_arm import (
    ControlMode,
    RemoteFranka,
)

if __name__ == "__main__":
    pyzlc.init("MetaQuest3Controller", "127.0.0.1")
    robot = RemoteFranka("FrankaPanda")
    robot.connect()
    robot.set_franka_arm_control_mode(ControlMode.CARTESIAN_VELOCITY)
    for _ in range(100):
        time.sleep(0.1)
