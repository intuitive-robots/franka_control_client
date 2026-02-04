import time
import os
import sys
# Add hardware directory to path to import GelloAgent
# sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
# from hardware.gello_zlc import GelloAgent

# # Allow running this script directly without installing the package.
# sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))
# import pyzlc
import pyzlc
from franka_control_client.camera.camera import CameraDevice
from franka_control_client.data_collection import DataCollectionManager
from franka_control_client.data_collection.wrapper import (
    HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    RobotiqGripperDataWrapper,
    GelloArmDataWrapper,
    GelloGripperDataWrapper,
)
from franka_control_client.franka_robot.franka_arm import (
    ControlMode,
    RemoteFranka,
)
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)

class EpisodeResetWrapper(HardwareDataWrapper):
    def __init__(
        self,
        franka: RemoteFranka,
        robotiq: RemoteRobotiqGripper,
        gello_arm: GelloArmDataWrapper,
        gello_gripper: GelloGripperDataWrapper,
        gripper_speed: float = 1.0,
        gripper_force: float = 0.3,
    ) -> None:
        self._franka = franka
        self._robotiq = robotiq
        self._gello_arm = gello_arm
        self._gello_gripper = gello_gripper
        self._gripper_speed = gripper_speed
        self._gripper_force = gripper_force
        super().__init__({})

    def capture_step(self):
        return {}

    def discard(self) -> None:
        pass

    def reset(self) -> None:
        try:
            arm_payload = self._gello_arm.capture_step()
            arm_joints = arm_payload[self._gello_arm.key]
            gripper_payload = self._gello_gripper.capture_step()
            gripper_value = float(gripper_payload[self._gello_gripper.key][0])
        except Exception as exc:
            pyzlc.warn(f"Reset skipped: failed to read Gello state ({exc})")
            return
        self._franka.move_franka_arm_to_joint_position(
            tuple(arm_joints.tolist())
        )
        self._franka.set_franka_arm_control_mode(
            ControlMode.HybridJointImpedance
        )
        if getattr(self._robotiq, "_enable_publishers", True):
            gripper_cmd = max(0.0, min(1.0, gripper_value))
            self._robotiq.send_grasp_command(
                position=gripper_cmd,
                speed=self._gripper_speed,
                force=self._gripper_force,
                blocking=False,
            )
        else:
            pyzlc.info(
                "Robotiq publishers disabled; skipping gripper reset."
            )

    def close(self) -> None:
        pass


if __name__ == "__main__":
    pyzlc.init(
        "data_collection_droid",
        "192.168.0.117",
        group_name="DroidGroup",
    )
    try:
        # if not pyzlc.check_node_info("gello"):
        #     pyzlc.info(
        #         "Gello node 'gello' not found. "
        #         "Make sure hardware/gello_zlc.py is running "
        #         "or set GelloArmDataWrapper(name=...) to match."
        #     )
        #todo: seperate connection from wrapper creation
        # camera_left = ImageDataWrapper(
        #     CameraDevice("zed_left", preview=False)
        # )
        # print("Left camera wrapper created for data collection.")   
        # camera_right = ImageDataWrapper(
        #     CameraDevice("zed_right", previe00w=False)
        # )
        # camera_wrist = ImageDataWrapper(
        #     CameraDevice("zed_wrist", preview=False)
        # )

        franka = RemoteFranka("FrankaPanda", enable_publishers=False)
        franka.connect()
        franka_arm = PandaArmDataWrapper(franka)
        print("Franka arm connected for data collection.")
        robotiq = RemoteRobotiqGripper("FrankaPanda", enable_publishers=False)
        robotiq.connect()
        robotiq_gripper = RobotiqGripperDataWrapper(robotiq)
        print("Robotiq gripper connected for data collection.")
        
        gello_arm = GelloArmDataWrapper(name="gello")
        print("Gello arm wrapper created for data collection.")
        gello_gripper = GelloGripperDataWrapper(name="gello")
        print("Gello arm and gripper wrappers created for data collection.")
        # resetter = EpisodeResetWrapper(
        #     franka,
        #     robotiq,
        #     gello_arm,
        #     gello_gripper,
        # )
        # print("Resetter wrapper created for data collection.")
        name = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        data_dir = f"./datasets/{name}"
        data_collection_manager = DataCollectionManager(
            [
                # camera_left,
                # camera_right,
                # camera_wrist,
                franka_arm,
                robotiq_gripper,
                gello_arm,
                gello_gripper,
                # resetter,
            ],
            data_dir,
            task="droid",
            fps=30,
        )
        data_collection_manager.run()
    finally:
        pyzlc.shutdown()
