import time

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
    robot = RemoteFranka("127.0.0.1", 5555)
    robot.connect()
    robot.set_franka_arm_control_mode(ControlMode.CARTESIAN_VELOCITY)
    try:
        while True:
            if mq3.device_info is not None:
                print(f"Device Info: {mq3.device_info}")
            input_data = mq3.get_controller_data()
            if input_data:
                vel = input_data["left"]["vel"]
                angular_vel = input_data["left"]["ang_vel"]
                robot.send_cartesian_velocity_command(vel + angular_vel)
                print(f"Received input data: {input_data}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    for _ in range(100):
        time.sleep(0.1)
