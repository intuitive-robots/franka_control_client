import pyzlc

from franka_control_client.franka_robot.panda_arm import (
    ControlMode,
    RemotePandaArm,
)

if __name__ == "__main__":
    pyzlc.init(
        "policy_inference",
        "141.3.53.25",
        group_name="robot_lab_robotiq_202",
        group_port=7725,
    )
    robot = RemotePandaArm("FrankaPanda")
    robot.connect()
    robot.set_franka_arm_control_mode(ControlMode.IDLE)
