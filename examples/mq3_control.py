import time

import numpy as np
from simpub.core import init_xr_node_manager
from simpub.xr_device import MetaQuest3
import pyzlc

from franka_control_client.franka_robot.panda_arm import (
    ControlMode,
    RemotePandaArm,
)
from franka_control_client.franka_robot.panda_gripper import (
    RemotePandaGripper,
)

if __name__ == "__main__":
    init_xr_node_manager("192.168.0.134")
    mq3 = MetaQuest3("IRL-MQ3-1")  # You can change the name by using simpubweb
    pyzlc.init("mq3_control_client", "127.0.0.1")
    robot = RemotePandaArm("FrankaPanda")
    # gripper = RemotePandaGripper("192.168.0.", 5557)
    robot.connect()
    # gripper.start_gripper_control()
    print(robot.get_franka_arm_control_mode())
    print(robot.get_franka_arm_state())
    robot.set_franka_arm_control_mode(ControlMode.HybridJointImpedance)
    current_joint_pose = robot.current_state["q"]
    current_joint_pose[-1] += 1
    print("Moving to new joint position:", current_joint_pose)
    try:
        while True:
            input_data = mq3.get_controller_data()
            robot.send_joint_position_command(current_joint_pose)
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
                print(f"Gripper width command: {width:.3f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    for _ in range(100):
        time.sleep(0.1)
