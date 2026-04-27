from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import pyzlc
import torch

from .control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode
from ..franka_robot.panda_robotiq import PandaRobotiq


CONTROL_MODE: ControlMode = ControlMode.HybridJointImpedance
GRIPPER_SPEED = 0.7
GRIPPER_FORCE = 0.3
GRIPPER_DEADBAND = 1e-3


class TrajectoryPandaControlPair(ControlPair):
    def __init__(
        self,
        trajectory_dir: Path,
        follower: PandaRobotiq,
        control_hz: float = 100,
    ) -> None:
        super().__init__()
        self.trajectory_dir = Path(trajectory_dir)
        self.follower = follower
        self.control_hz = float(control_hz)
        self.joint_pos: Optional[np.ndarray] = None
        self.gripper_pos: Optional[np.ndarray] = None
        self._step_idx = 0
        self._last_gripper_cmd: Optional[float] = None

    def start_control_pair(self) -> None:
        if self.is_running:
            return
        self.control_reset()
        super().start_control_pair()

    def load_trajectory(self) -> None:
        joint_pos_path = self.trajectory_dir / "joint_pos.pt"
        gripper_state_path = self.trajectory_dir / "gripper_state.pt"

        if not joint_pos_path.exists():
            raise FileNotFoundError(f"Missing replay joint trajectory: {joint_pos_path}")
        if not gripper_state_path.exists():
            raise FileNotFoundError(f"Missing replay gripper trajectory: {gripper_state_path}")

        joint_pos = torch.load(joint_pos_path, map_location="cpu")
        gripper_pos = torch.load(gripper_state_path, map_location="cpu")
        self.joint_pos = np.asarray(joint_pos, dtype=np.float64).reshape(-1, 7)
        self.gripper_pos = np.asarray(gripper_pos, dtype=np.float64).reshape(-1)

        if self.joint_pos.shape[0] == 0:
            raise ValueError(f"Empty replay joint trajectory: {joint_pos_path}")
        if self.gripper_pos.shape[0] == 0:
            raise ValueError(f"Empty replay gripper trajectory: {gripper_state_path}")

        replay_len = min(self.joint_pos.shape[0], self.gripper_pos.shape[0])
        self.joint_pos = self.joint_pos[:replay_len]
        self.gripper_pos = self.gripper_pos[:replay_len]
        self._step_idx = 0
        self._last_gripper_cmd = None
        pyzlc.info(f"Loaded replay trajectory with {replay_len} steps from {self.trajectory_dir}")

    def save_trajectory(self, output_dir: Path, target_len: Optional[int] = None) -> None:
        if self.joint_pos is None or self.gripper_pos is None:
            self.load_trajectory()

        assert self.joint_pos is not None
        assert self.gripper_pos is not None
        joint_pos = self.joint_pos
        gripper_pos = self.gripper_pos

        if target_len is not None:
            target_len = int(target_len)
            if target_len < 0:
                raise ValueError(f"target_len must be non-negative, got {target_len}")
            joint_pos, gripper_pos = self._resize_trajectory(
                joint_pos, gripper_pos, target_len
            )

        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        torch.save(
            torch.tensor(joint_pos, dtype=torch.float64),
            output_dir / "joint_pos.pt",
        )
        torch.save(
            torch.tensor(gripper_pos, dtype=torch.float64),
            output_dir / "gripper_state.pt",
        )
        pyzlc.info(f"Saved replay trajectory with {len(joint_pos)} steps to {output_dir}")

    @staticmethod
    def _resize_trajectory(
        joint_pos: np.ndarray, gripper_pos: np.ndarray, target_len: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        if target_len <= joint_pos.shape[0]:
            return joint_pos[:target_len], gripper_pos[:target_len]

        pad_len = target_len - joint_pos.shape[0]
        joint_padding = np.repeat(joint_pos[-1:], pad_len, axis=0)
        gripper_padding = np.repeat(gripper_pos[-1:], pad_len, axis=0)
        return (
            np.concatenate([joint_pos, joint_padding], axis=0),
            np.concatenate([gripper_pos, gripper_padding], axis=0),
        )

    def control_reset(self) -> None:
        self.load_trajectory()
        assert self.joint_pos is not None
        assert self.gripper_pos is not None
        self.follower.panda_arm.move_franka_arm_to_joint_position(self.joint_pos[0])
        self.follower.robotiq_gripper.send_grasp_command(
            position=float(np.clip(self.gripper_pos[0], 0.0, 1.0)),
            speed=GRIPPER_SPEED,
            force=GRIPPER_FORCE,
            blocking=True,
        )

    def control_step(self) -> None:
        if self.joint_pos is None or self.gripper_pos is None:
            self.load_trajectory()

        assert self.joint_pos is not None
        assert self.gripper_pos is not None
        idx = min(self._step_idx, self.joint_pos.shape[0] - 1)

        self.follower.panda_arm.send_joint_position_command(self.joint_pos[idx])
        gripper_cmd = float(np.clip(self.gripper_pos[idx], 0.0, 1.0))
        if (
            self._last_gripper_cmd is None
            or abs(gripper_cmd - self._last_gripper_cmd) > GRIPPER_DEADBAND
        ):
            self.follower.robotiq_gripper.send_grasp_command(
                position=gripper_cmd,
                speed=GRIPPER_SPEED,
                force=GRIPPER_FORCE,
                blocking=False,
            )
            self._last_gripper_cmd = gripper_cmd

        self._step_idx += 1
        pyzlc.sleep(1 / self.control_hz)

    def control_end(self) -> None:
        self.follower.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)

    def _control_task(self) -> None:
        try:
            self.load_trajectory()
            self.follower.panda_arm.set_franka_arm_control_mode(CONTROL_MODE)
            while self.is_running:
                self.control_step()
            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
