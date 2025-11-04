from franka_control_client.franka_robot.franka_arm import (
    ControlMode,
    FrankaArmState,
    RemoteFranka,
)


def test_get_franka_arm_state(fake_franka_server):
    robot = RemoteFranka("127.0.0.1", 5555)
    robot.connect()
    state = robot.get_franka_arm_state()
    assert isinstance(state, FrankaArmState)
    robot.disconnect()


def test_get_franka_arm_state_pub_port(fake_franka_server):
    robot = RemoteFranka("127.0.0.1", 5555)
    robot.connect()
    pub_port = robot.get_franka_arm_state_pub_port()
    assert pub_port == 5556
    robot.disconnect()


def test_get_franka_arm_control_mode(fake_franka_server):
    robot = RemoteFranka("127.0.0.1", 5555)
    robot.connect()
    control_mode = robot.get_franka_arm_control_mode()
    assert control_mode == ControlMode.HUMAN_MODE
    robot.disconnect()


# def test_set_control_mode_success(fake_franka_server):
#     robot = RemoteFranka("127.0.0.1", 5555)
#     robot.connect()
#     robot.set_franka_arm_control_mode(
#         ControlMode.JOINT_POSITION, controller_ip="127.0.0.1", controller_port=9999
#     )
#     robot.disconnect()


# def test_move_to_position_success(fake_franka_server):
#     robot = RemoteFranka("127.0.0.1", 5555)
#     robot.connect()
#     pose = tuple(float(i) for i in range(16))
#     robot.move_to_position(pose)
#     robot.disconnect()
