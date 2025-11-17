import time

import numpy as np
from simpub.core.node_manager import init_xr_node_manager
from simpub.xr_device.meta_quest3 import MetaQuest3

from franka_control_client.franka_robot.franka_arm import (
    ControlMode,
    FrankaArmState,
    RemoteFranka,
)

if __name__ == "__main__":
    net_manager = init_xr_node_manager("192.168.0.134")
    net_manager.start_discover_node_loop()
    mq3 = MetaQuest3("IRL-MQ3-1")  # You can change the name by using simpubweb
    robot = RemoteFranka("192.168.0.52", 5556)
    robot.connect()
    robot.set_franka_arm_control_mode(ControlMode.CARTESIAN_VELOCITY)
    try:
        while True:
            # if mq3.device_info is not None:
            #     print(f"Device Info: {mq3.device_info}")
            input_data = mq3.get_controller_data()
            if input_data and input_data["right"]["hand_trigger"]:
                vel = np.array(input_data["right"]["vel"])
                # if True:
                # vel = [0.005, 0.0, 0.0]
                # vel[0] = 0.5 * vel[0]
                # vel[1] = 0.5 * vel[1]
                # vel[2] = 0.5 * vel[2]
                angular_vel = np.array(input_data["right"]["ang_vel"])
                angular_vel = 0.5 * angular_vel
                robot.send_cartesian_velocity_command(
                    vel.tolist() + angular_vel.tolist()
                )
                print(f"Received input data: {vel}")
            else:
                robot.send_cartesian_velocity_command([0, 0, 0, 0, 0, 0])
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    for _ in range(100):
        time.sleep(0.1)
