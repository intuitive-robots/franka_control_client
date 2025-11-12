import time

# test robot server interactions for Franka arm
from franka_control_client.franka_robot.franka_arm import (
    ControlMode,
    FrankaArmState,
    RemoteFranka,
)


def test_get_franka_arm_state():
    robot = RemoteFranka("127.0.0.1", 5555)
    print("Connecting to Franka robot...")
    robot.connect()
    print("Fetching Franka arm state...")
    state = robot.get_franka_arm_state()
    assert isinstance(state, FrankaArmState)
    robot.disconnect()


def test_get_franka_arm_state_pub_port():
    robot = RemoteFranka("127.0.0.1", 5555)
    robot.connect()
    pub_port = robot.get_franka_arm_state_pub_port()
    print(f"Franka arm state pub port: {pub_port}")
    robot.disconnect()


def test_get_franka_arm_control_mode():
    robot = RemoteFranka("127.0.0.1", 5555)
    robot.connect()
    control_mode = robot.get_franka_arm_control_mode()
    assert control_mode == ControlMode.IDLE
    robot.disconnect()


def test_set_control_mode_success():
    robot = RemoteFranka("127.0.0.1", 5555)
    robot.connect()
    robot.set_franka_arm_control_mode(ControlMode.CARTESIAN_VELOCITY)
    for _ in range(100):
        robot.send_cartesian_velocity_command([0.1] * 6)
        time.sleep(0.1)
    robot.disconnect()


# def test_move_to_position_success(fake_franka_server):
#     robot = RemoteFranka("127.0.0.1", 5555)
#     robot.connect()
#     pose = tuple(float(i) for i in range(16))
#     robot.move_to_position(pose)
#     robot.disconnect()

if __name__ == "__main__":
    # test_get_franka_arm_state()
    # test_get_franka_arm_state_pub_port()
    # test_get_franka_arm_control_mode()
    test_set_control_mode_success()
    # test_move_to_position_success()
