from __future__ import annotations

from enum import Enum
from typing import NotRequired, TypedDict, Sequence, List, Optional
import pyzlc
import numpy as np

from ..core.latest_msg_subscriber import LatestMsgSubscriber
from ..core.exception import CommandError
from ..core.message import FrankaResponseCode
from ..core.remote_device import RemoteDevice


class ControlMode(str, Enum):
    IDLE = "Idle"
    HybridJointImpedance = "HybridJointImpedance"
    # same controller, soft gains for contact-rich tasks (hybrid_joint_impedance_compliant_controller.yaml)
    HybridJointImpedanceCompliant = "HybridJointImpedanceCompliant"
    HybridJointExtTorqueImpedance = "HybridJointExtTorqueImpedance"
    OSC = "OSC"
    CartesianImpedance = "CartesianImpedance"
    HumanControl = "HumanControl"
    # JOINT_POSITION = "JointPosition"
    # JOINT_VELOCITY = "JointVelocity"
    # CARTESIAN_VELOCITY = "CartesianVelocity"
    # JOINT_TORQUE = "JointTorque"
    # GRAVITY_COMP = "GravityComp"


class PandaArmState(TypedDict):
    """
    Franka arm state structure.
    """

    time_ms: int
    EE_pos: List[float]
    EE_quat: List[float]
    O_T_EE: List[float]
    O_T_EE_d: List[float]
    q: List[float]
    q_d: List[float]
    dq: List[float]
    dq_d: List[float]
    tau_ext_hat_filtered: List[float]
    O_F_ext_hat_K: List[float]
    K_F_ext_hat_K: List[float]


class JointPositionCommand(TypedDict):
    """
    Joint position command structure.
    """

    pos: List[float]  # 7 joint angles in radians
    vel: NotRequired[List[float]]  # optional feed-forward joint velocities in rad/s


class JointTorqueCommand(TypedDict):
    """
    External joint torque command structure.
    """

    tau: List[float]  # 7 joint torques in Nm


class EEWrenchCommand(TypedDict):
    """
    Feed-forward wrench at the flange, added by the HybridJointImpedance modes.
    """

    wrench: List[float]  # fx, fy, fz [N], tx, ty, tz [Nm] in the base frame


class CartesianPoseCommand(TypedDict):
    """
    Cartesian pose command structure.
    """

    pos: List[float]  # x, y, z and quaternion x, y, z, w
    rot: List[float]  # Optional rotation matrix (9 values) if needed
    pos_vel: List[float]  # Optional Cartesian velocity (vx, vy, vz) if needed
    rot_vel: List[float]  # Optional rotational velocity (wx, wy, wz


class CartesianVelocityCommand(TypedDict):
    """
    Cartesian velocity command structure.
    """

    vel: List[float]  # vx, vy, vz, wx, wy, wz


class RemotePandaArm(RemoteDevice):
    """
    RemotePandaArm class for controlling a Franka robot.

    This class extends the RemoteDevice class and provides
    specific functionality for interacting with a Franka robot.
    Attributes:
        robot_name (str): Name of the Franka robot.
    """

    def __init__(
        self, robot_name: str, enable_publishers: bool = True
    ) -> None:
        """
        Initialize the RemotePandaArm instance.

        Args:
            robot_name (str): The name of the Franka robot.
        """
        super().__init__(robot_name)
        self.default_pose: Sequence[float] = (-1,)
        self.arm_state_sub = LatestMsgSubscriber(
            f"{robot_name}/franka_arm_state"
        )
        self._enable_publishers = enable_publishers
        self.joint_position_publisher = pyzlc.Publisher(
            f"{robot_name}/joint_command"
        )
        self.cartesian_pose_publisher = pyzlc.Publisher(
            f"{robot_name}/cartesian_pose_command"
        )
        self.joint_torque_publisher = pyzlc.Publisher(
            f"{robot_name}/joint_torque_command"
        )
        self.ee_wrench_publisher = pyzlc.Publisher(
            f"{robot_name}/ee_wrench_command"
        )

    def connect(self) -> None:
        """
        Connect to the Franka robot.

        Raises:
            DeviceConnectionError: If the connection fails.
        """
        super().connect()
        for _ in range(5):
            if self.arm_state_sub.last_message is not None:
                pyzlc.info("Franka arm state subscriber connected.")
                return
            pyzlc.info("Waiting for Franka arm state...")
            pyzlc.sleep(1)

    @property
    def current_state(self) -> Optional[PandaArmState]:
        """Return the latest Franka arm state."""
        return self.arm_state_sub.get_latest()

    @property
    def current_ee_position(self) -> Optional[List[float]]:
        """Return the current end-effector position (x, y, z)."""
        state = self.arm_state_sub.get_latest()
        if state is None:
            return None
        return state["EE_pos"]

    @property
    def current_ee_rotation(self) -> Optional[List[float]]:
        """Return the current end-effector rotation (quaternion x, y, z, w)."""
        state = self.arm_state_sub.get_latest()
        if state is None:
            return None
        return state["EE_quat"]

    def get_franka_arm_state(self) -> PandaArmState:
        """Return a single state sample"""
        return pyzlc.call(f"{self._name}/get_franka_arm_state", pyzlc.empty)

    def get_franka_arm_control_mode(self) -> str:
        """Return the currently active control mode."""
        return pyzlc.call(
            f"{self._name}/get_franka_arm_control_mode", pyzlc.empty
        )

    def set_franka_arm_control_mode(self, mode: ControlMode) -> None:
        """Set the control mode of the Franka arm."""
        pyzlc.call(f"{self._name}/set_franka_arm_control_mode", mode.value)
        pyzlc.info(f"Set Franka arm control mode to {mode.value}")

    def move_franka_arm_to_joint_position(
        self, joint_positions: Sequence[float]
    ) -> None:
        """
        Move the Franka arm to the specified joint position.

        Args:
            joint_positions (tuple of 7 floats): Target joint angles (radians).
        Raises:
            CommandError: If packing or response fails.
        """
        if len(joint_positions) != 7:
            raise CommandError(
                f"Expected 7 joint values, got {len(joint_positions)}"
            )
        header, _ = pyzlc.call(
            f"{self._name}/move_franka_arm_to_joint_position",
            list(joint_positions),
            10.0,
        )

        if header is None or header != FrankaResponseCode.SUCCESS.value:
            raise CommandError(
                f"MOVE_FRANKA_ARM_TO_JOINT_POSITION failed (status={header})"
            )

    def move_franka_arm_to_cartesian_position(
        self, pose_matrix: Sequence[float]
    ) -> None:
        """
        Move the Franka arm to the specified Cartesian pose.

        Args:
            pose_matrix (tuple of 16 floats): 4x4 transformation matrix (row-major).
        Raises:
            CommandError: If packing or command execution fails.
        """
        if len(pose_matrix) != 16:
            raise CommandError(
                f"Expected 16 pose values, got {len(pose_matrix)}"
            )
        raise NotImplementedError

    def send_joint_position_command(
        self,
        joint_positions: Sequence[float],
        joint_velocities: Optional[Sequence[float]] = None,
    ) -> None:
        """
        Send a joint position command to the Franka arm.
        Accepts tuple, list, numpy array, or torch tensor (no type checking).
        joint_velocities is an optional feed-forward velocity [rad/s]; the hybrid joint
        controllers use it as damping reference and to extrapolate between commands.
        """
        if not self._enable_publishers:
            raise RuntimeError(
                "Publishers disabled for this RemotePandaArm instance."
            )
        arr = np.asarray(joint_positions, dtype=np.float64).reshape(-1)
        # print(f"Sending joint position command: {arr}")
        if arr.size != 7:
            raise ValueError(f"Expected 7 joint angles, got {arr.size}")
        cmd = JointPositionCommand(pos=arr.tolist())
        if joint_velocities is not None:
            vel = np.asarray(joint_velocities, dtype=np.float64).reshape(-1)
            if vel.size != 7:
                raise ValueError(f"Expected 7 joint velocities, got {vel.size}")
            cmd["vel"] = vel.tolist()
        self.joint_position_publisher.publish(cmd)

    def send_cartesian_pose_command(
        self,
        pos: Sequence[float],
        rot: Sequence[float],
        pos_vel: Optional[Sequence[float]] = None,
        rot_vel: Optional[Sequence[float]] = None,
    ) -> None:
        """
        Send a Cartesian pose command to the Franka arm.

        Args:
            pos (tuple of 3 floats): translation (x, y, z).
            rot (tuple of 4 floats): orientation quaternion (x, y, z, w).
            pos_vel (tuple of 3 floats): optional feed-forward linear velocity [m/s], base frame.
            rot_vel (tuple of 3 floats): optional feed-forward angular velocity [rad/s], base frame.
        Raises:
            CommandError: If packing or command execution fails.
        """
        if not self._enable_publishers:
            raise RuntimeError(
                "Publishers disabled for this RemotePandaArm instance."
            )
        pos_arr = np.asarray(pos, dtype=np.float64).reshape(-1)
        rot_arr = np.asarray(rot, dtype=np.float64).reshape(-1)
        if pos_arr.size != 3 or rot_arr.size != 4:
            raise ValueError(
                f"Expected 3 position values and 4 orientation values, got {pos_arr.size} and {rot_arr.size}"
            )
        pos_vel_arr = np.zeros(3) if pos_vel is None else np.asarray(pos_vel, dtype=np.float64).reshape(-1)
        rot_vel_arr = np.zeros(3) if rot_vel is None else np.asarray(rot_vel, dtype=np.float64).reshape(-1)
        if pos_vel_arr.size != 3 or rot_vel_arr.size != 3:
            raise ValueError(
                f"Expected 3 linear and 3 angular velocity values, got {pos_vel_arr.size} and {rot_vel_arr.size}"
            )
        self.cartesian_pose_publisher.publish(
            CartesianPoseCommand(
                pos=pos_arr.tolist(),
                rot=rot_arr.tolist(),
                pos_vel=pos_vel_arr.tolist(),
                rot_vel=rot_vel_arr.tolist(),
            )
        )

    def send_joint_velocity_command(
        self, joint_velocities: Sequence[float]
    ) -> None:
        """
        Send a joint velocity command to the Franka arm.
        Accepts tuple, list, numpy array, or torch tensor (no type checking).
        """
        if not self._enable_publishers:
            raise RuntimeError(
                "Publishers disabled for this RemotePandaArm instance."
            )
        arr = np.asarray(joint_velocities, dtype=np.float64).reshape(-1)
        if arr.size != 7:
            raise ValueError(f"Expected 7 joint velocities, got {arr.size}")
        raise NotImplementedError

    def send_cartesian_velocity_command(
        self, cartesian_velocities: Sequence[float]
    ) -> None:
        raise NotImplementedError

    def send_joint_torque_command(
        self, joint_torques: Sequence[float]
    ) -> None:
        """
        Send a joint torque command to the Franka arm.
        Accepts tuple, list, numpy array, or torch tensor (no type checking).
        """
        if not self._enable_publishers:
            raise RuntimeError(
                "Publishers disabled for this RemotePandaArm instance."
            )
        arr = np.asarray(joint_torques, dtype=np.float64).reshape(-1)
        if arr.size != 7:
            raise ValueError(f"Expected 7 joint torques, got {arr.size}")
        self.joint_torque_publisher.publish(
            JointTorqueCommand(tau=arr.tolist())
        )

    def send_ee_wrench_command(self, wrench: Sequence[float]) -> None:
        """
        Send a feed-forward wrench at the flange (fx, fy, fz [N], tx, ty, tz [Nm], base frame).
        The HybridJointImpedance modes add it as J^T * wrench on top of the impedance, limited to
        max_ee_force / max_ee_torque of the controller config. It is dropped when no new command
        arrives within ee_wrench_timeout (0.1 s), so keep sending it while it should act.
        """
        if not self._enable_publishers:
            raise RuntimeError(
                "Publishers disabled for this RemotePandaArm instance."
            )
        arr = np.asarray(wrench, dtype=np.float64).reshape(-1)
        if arr.size != 6:
            raise ValueError(f"Expected 6 wrench values, got {arr.size}")
        self.ee_wrench_publisher.publish(EEWrenchCommand(wrench=arr.tolist()))
