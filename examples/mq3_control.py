import time

import numpy as np
from simpub.core.node_manager import init_xr_node_manager
from simpub.xr_device.meta_quest3 import MetaQuest3

from franka_control_client.franka_robot.franka_arm import (
    ControlMode,
    RemoteFranka,
)
from franka_control_client.franka_robot.franka_gripper import (
    RemoteFrankaGripper,
)

if __name__ == "__main__":
    net_manager = init_xr_node_manager("192.168.0.134")
    net_manager.start_discover_node_loop()
    mq3 = MetaQuest3("IRL-MQ3-1")  # You can change the name by using simpubweb
    # robot = RemoteFranka("192.168.0.52", 5555)
    # gripper = RemoteFrankaGripper("192.168.0.52", 5557)
    robot = RemoteFranka("127.0.0.1", 5555)
    gripper = RemoteFrankaGripper("127.0.0.1", 5557)
    robot.connect()
    gripper.connect()
    gripper.start_gripper_control()
    robot.set_franka_arm_control_mode(ControlMode.CARTESIAN_VELOCITY)
    try:
        while True:
            # if mq3.device_info is not None:
            #     print(f"Device Info: {mq3.device_info}")
            input_data = mq3.get_controller_data()
            if input_data is None:
                continue
                # print(f"Input Data: {input_data}")
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
