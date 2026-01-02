import time

import numpy as np
from simpub.xr_devices import MetaQuest3
import pyzlc

from franka_control_client.franka_robot.franka_arm import (
    ControlMode,
    RemoteFranka,
)
from franka_control_client.franka_robot.franka_gripper import (
    RemoteFrankaGripper,
)

if __name__ == "__main__":
    pyzlc.init("MetaQuest3Controller", "192.168.0.134")
    mq3 = MetaQuest3("IRL-MQ3-1")  # You can change the name by using simpubweb
    robot = RemoteFranka("FrankaRobot")
    gripper = RemoteFrankaGripper("192.168.0.232", 5557)
    robot.connect()
    gripper.start_gripper_control()
    robot.set_franka_arm_control_mode(ControlMode.CARTESIAN_VELOCITY)
    try:
        while True:
            input_data = mq3.get_controller_data()
            if input_data is None:
                continue
            if input_data["right"]["hand_trigger"]:
                vel = np.array(input_data["right"]["vel"])
                angular_vel = np.array(input_data["right"]["ang_vel"])
                angular_vel = 0.5 * angular_vel
                robot.send_cartesian_velocity_command(
                    vel.tolist() + angular_vel.tolist()
                )
                width = 0.08 * (1 - input_data["right"]["index_trigger"])
                gripper.send_gripper_command(width)
                print(f"Gripper width command: {width:.3f}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    for _ in range(100):
        time.sleep(0.1)
