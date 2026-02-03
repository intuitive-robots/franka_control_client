import sys
import os
import time

import numpy as np
import pyzlc

# Add hardware directory to path to import GelloAgent
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from hardware.gello_zlc import GelloAgent

from franka_control_client.franka_robot.franka_arm import (
    ControlMode,
    RemoteFranka,
)
from franka_control_client.franka_robot.franka_gripper import (
    RemoteFrankaGripper,
)
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)

if __name__ == "__main__":
    # Initialize Gello agent
    gello_port = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT94EVRT-if00-port0"
    gello = GelloAgent(port=gello_port)
    
    pyzlc.init("gello_control_client", "192.168.0.117", group_name="RobotControlGroup")
    # pyzlc.init("gello_control_client", "192.168.0.117")
    robot = RemoteFranka("FrankaPanda")
    robotiq = RemoteRobotiqGripper("FrankaPanda")
    # gripper = RemoteFrankaGripper("192.168.0.", 5557)
    robot.connect()
    robotiq.connect()
    # gripper.start_gripper_control()
    print(robot.get_franka_arm_control_mode())
    # print(robot.get_franka_arm_state())
    gello_joint_state = gello._robot.get_joint_state()
    arm_joints = np.array(gello_joint_state[:-1], dtype=np.float64)
    print("Aligning Franka to Gello joints:", arm_joints.round(3).tolist())
    robot.move_franka_arm_to_joint_position((arm_joints))
    print("before set mode arm joints now:", robot.get_franka_arm_state()["q"])
    time.sleep(2)
    print("Setting Franka to Hybrid Joint Impedance mode")
    robot.set_franka_arm_control_mode(ControlMode.HybridJointImpedance)
    print("after set mode arm joints now:", robot.get_franka_arm_state()["q"])
    last_gripper_cmd = None
    log_path = os.path.join(os.path.dirname(__file__), "robotiq_state_log.txt")
    log_file = open(log_path, "a", encoding="utf-8")
    last_log_time = 0.0
    try:
        while True:
            # Get Gello joint state (8 joints: 7 arm + 1 gripper)
            gello_joint_state = gello._robot.get_joint_state()
            # print("Gello joint state:", gello_joint_state)
            
            # Extract arm joints (first 7) and gripper (last joint)
            arm_joints = np.array(gello_joint_state[:-1], dtype=np.float64)
            # print("arm joints now:", robot.get_franka_arm_state()["q"])
            gripper_value = float(gello_joint_state[-1])
            
            # Send arm joint positions to robot
            robot.send_joint_position_command(arm_joints)

            # Send gripper command to Robotiq (expects 0.0=open, 1.0=close)
            gripper_cmd = max(0.0, min(1.0, gripper_value))
            if last_gripper_cmd is None or abs(gripper_cmd - last_gripper_cmd) > 1e-3:
                robotiq.send_grasp_command(
                    position=gripper_cmd, speed=1, force=0.3, blocking=False #todo: add into cofig
                )
                last_gripper_cmd = gripper_cmd

            try:
                state = robotiq.current_state
                # print("Robotiq gripper state:", state)
            except AssertionError:
                state = None
            now = time.time()
            if state is not None and (now - last_log_time) >= 0.5:
                log_file.write(f"{time.time():.6f}\t{state}\n")
                last_log_time = now
            
            # print(f"Gello joints: {arm_joints.round(3).tolist()} | Gripper: {gripper_value:.3f}")
            
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        log_file.close()
    for _ in range(100):
        time.sleep(0.1)
