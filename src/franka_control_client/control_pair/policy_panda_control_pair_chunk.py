from __future__ import annotations

import threading
import time
import traceback
from pathlib import Path
from typing import Optional, Union

import matplotlib
import numpy as np
import torch
import pyzlc
from collections import deque

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode, RemotePandaArm
from ..franka_robot.panda_gripper import RemotePandaGripper
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper

# todo refine the executed action count
DEFAULT_CONTROL_HZ: float = 50
GRIPPER_DEADBAND: float = 1e-3
GRIPPER_SPEED = 0.7
GRIPPER_FORCE = 0.3
GRIPPER_TOGGLE_WARN_WINDOW_S: float = 3.0
GRIPPER_TOGGLE_WARN_COUNT: int = 6
DEFAULT_POSITION = (0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0)

# Calculate velocity limits using the standard approach from training
# VELOCITY_LIMITS = np.array([[-4 * np.pi / 2, 4 * np.pi / 2]] * 7).T / 16
# VELOCITY_LIMITS_NORM = np.linalg.norm(VELOCITY_LIMITS)
# Franka joint velocity limits in rad/s for joints 1..7.
VELOCITY_LIMITS = np.array(
    [
        [-2.62, -2.62, -2.62, -2.62, -5.26, -4.18, -5.26],
        [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26],
        # [-2.62, -2.62, -2.62, -2.62, -0.5, -2, -2],
        # [2.62, 2.62, 2.62, 2.62, 0.5, 2, 2],
    ],
    dtype=np.float32,
)
VELOCITY_LIMIT_SCALE = 0.8
VELOCITY_LIMITS_MAX = VELOCITY_LIMITS[1] * VELOCITY_LIMIT_SCALE

class PolicyPandaControlPair(ControlPair):
    """
    Apply policy actions to a Panda arm with a gripper.

    Action semantics: [q0..q6, gripper] (joint positions + gripper).
    Gripper value is normalized in [0, 1]. It is scaled to device range.
    Includes velocity and acceleration limiting for safety.
    """

    def __init__(
        self,
        panda_arm: RemotePandaArm,
        gripper: Union[RemotePandaGripper, RemoteRobotiqGripper],
        control_hz: float = DEFAULT_CONTROL_HZ,
    ) -> None:
        super().__init__()
        self.panda_arm = panda_arm
        self.gripper = gripper
        self.control_hz = float(control_hz)
        self._action_lock = threading.Lock() #only one of the update_action and control_step visit latest_action at the same time 
        self._latest_action: Optional[np.ndarray] = None
        self._latest_action_chunk: deque[np.ndarray] = deque()
        self._current_action_chunk: deque[np.ndarray] = deque()
        self._latest_action_chunk_version: int = 0
        self._current_action_chunk_version: int = 0
        self._last_gripper_cmd: Optional[float] = None
        self._last_gripper_binary: Optional[int] = None
        self._gripper_toggle_window_start_ts: float = time.time()
        self._gripper_toggle_count: int = 0
        
        # Velocity limiting state
        self._last_joint_pos: Optional[np.ndarray] = None
        self._last_control_time: Optional[float] = None
        self._dt = 1.0 / self.control_hz  # time delta between control steps
        self._plot_dir = Path("debug/control_pair_joint_plots")
        self._action_chunk_joint_history: list[np.ndarray] = []
        self._expanded_action_chunk_joint_history: list[np.ndarray] = []
        self._sent_waypoint_joint_history: list[np.ndarray] = []
        self._plot_current_pos_joint_history: list[np.ndarray] = []
        self._chunk_end_action_indices: set[int] = set()
        self._chunk_end_plot_indices: list[int] = []
        self._executed_action_count: int = 0
        self._received_chunk_count: int = 0

    def _get_current_joint_pos(self) -> Optional[np.ndarray]:
        current_state = self.panda_arm.current_state
        if current_state is None or "q" not in current_state:
            return None
        joint_pos = np.asarray(current_state["q"], dtype=np.float32).reshape(-1)
        if joint_pos.size != 7:
            pyzlc.error(
                f"Unexpected current arm state size during control init: {joint_pos.size}"
            )
            return None
        return joint_pos

    #using by policy side to update the latest action_chunk, and control loop will read the latest action and execute it
    def update_action_chunk(self, action_chunk: np.ndarray) -> None:
        """Update the latest action chunk used by the control loop."""
        chunk = np.asarray(action_chunk, dtype=np.float64)
        if chunk.ndim == 1:
            chunk = chunk.reshape(1, 1, -1)
        elif chunk.ndim == 2:
            chunk = chunk.reshape(1, *chunk.shape)
        elif chunk.ndim != 3:
            raise ValueError(
                f"Expected action chunk shape (B, T, D), (T, D), or (D,), got {chunk.shape}"
            )

        if chunk.shape[-1] < 8:
            raise ValueError(f"Expected action size >= 8, got {chunk.shape[-1]}")
        if chunk.shape[0] < 1 or chunk.shape[1] < 1:
            raise ValueError(f"Action chunk must contain at least one action, got {chunk.shape}")

        action_queue = deque(np.array(action, copy=True) for action in chunk[0])
        with self._action_lock:
            self._latest_action = action_queue[-1].copy()
            self._latest_action_chunk = action_queue
            self._latest_action_chunk_version += 1
        self._action_chunk_joint_history.extend(
            np.asarray(action[:7], dtype=np.float32).copy() for action in chunk[0]
        )
        # Mark chunk arrival at the latest waypoint index available at receive time.
        self._chunk_end_plot_indices.append(max(len(self._sent_waypoint_joint_history) - 1, 0))
        self._received_chunk_count += 1

    def _copy_action_queue(
        self, action_queue: deque[np.ndarray]
    ) -> deque[np.ndarray]:
        return deque(np.array(action, copy=True) for action in action_queue)

    def _blend_action(
        self, current_action: np.ndarray, new_action: np.ndarray, alpha: float
    ) -> np.ndarray:
        blended = ((1.0 - alpha) * current_action) + (alpha * new_action)
        if blended.size > 7:
            blended[7] = current_action[7] if alpha < 0.5 else new_action[7]
        return blended

    def _blend_action_chunks(
        self,
        current_chunk: deque[np.ndarray],
        latest_chunk: deque[np.ndarray],
    ) -> tuple[deque[np.ndarray], int]:
        current_actions = [np.array(action, copy=True) for action in current_chunk]
        latest_actions = [np.array(action, copy=True) for action in latest_chunk]
        if not current_actions:
            return deque(latest_actions), 0
        if not latest_actions:
            return deque(current_actions), 0

        overlap = min(len(current_actions), len(latest_actions))
        blended_actions: list[np.ndarray] = []
        for idx in range(overlap):
            alpha = float(idx + 1) / float(overlap)
            blended_actions.append(
                self._blend_action(current_actions[idx], latest_actions[idx], alpha)
            )

        if len(latest_actions) > overlap:
            blended_actions.extend(latest_actions[overlap:])
        elif len(current_actions) > overlap:
            blended_actions.extend(current_actions[overlap:])

        # pyzlc.info(
        #     "Blended action chunks "
        #     f"(remaining_current={len(current_actions)}, "
        #     f"latest={len(latest_actions)}, overlap={overlap})"
        # )
        return deque(blended_actions), overlap

    def _promote_latest_chunk_locked(self) -> None:
        if not self._latest_action_chunk:
            return
        self._current_action_chunk = self._copy_action_queue(self._latest_action_chunk)
        self._current_action_chunk_version = self._latest_action_chunk_version
        self._latest_action = self._current_action_chunk[-1].copy()

    def _get_latest_action_from_chunk(self) -> Optional[np.ndarray]:
        with self._action_lock:
            if not self._current_action_chunk:
                self._promote_latest_chunk_locked()
            elif (
                self._latest_action_chunk
                and self._latest_action_chunk_version > self._current_action_chunk_version
            ):
                self._current_action_chunk, overlap = self._blend_action_chunks(
                    self._current_action_chunk, self._latest_action_chunk
                )
                self._current_action_chunk_version = self._latest_action_chunk_version

            if self._current_action_chunk:
                action = self._current_action_chunk.popleft()
                if self._current_action_chunk:
                    self._latest_action = self._current_action_chunk[-1].copy()
                else:
                    self._latest_action = action.copy()
                return action.copy()

            if self._latest_action is None:
                return None
            return self._latest_action.copy()
    
    def reset_action(self) -> None:
        """Reset the latest action state when starting a new episode."""
        with self._action_lock:
            self._latest_action = None
            self._latest_action_chunk.clear()
            self._current_action_chunk.clear()
            self._latest_action_chunk_version = 0
            self._current_action_chunk_version = 0
        self._action_chunk_joint_history = []
        self._expanded_action_chunk_joint_history = []
        self._sent_waypoint_joint_history = []
        self._plot_current_pos_joint_history = []
        self._chunk_end_action_indices = set()
        self._chunk_end_plot_indices = []
        self._executed_action_count = 0
        self._received_chunk_count = 0
        self._last_gripper_cmd = None
        self._last_gripper_binary = None
        self._gripper_toggle_count = 0
        self._gripper_toggle_window_start_ts = time.time()
        self._last_joint_pos = self._get_current_joint_pos()
        pyzlc.info("Action state reset for new episode")

    def _save_joint_comparison_plots(self) -> None:
        """Save one plot per joint comparing target chunk values and sent waypoints."""
        if not self._action_chunk_joint_history:
            pyzlc.info("Skipping joint plots: no action chunk history recorded.")
            return
        if not self._sent_waypoint_joint_history:
            pyzlc.info("Skipping joint plots: no waypoint commands were sent.")
            return

        action_chunk = np.asarray(self._expanded_action_chunk_joint_history, dtype=np.float32)
        sent_waypoints = np.asarray(self._sent_waypoint_joint_history, dtype=np.float32)
        current_pos = np.asarray(self._plot_current_pos_joint_history, dtype=np.float32)
        self._plot_dir.mkdir(parents=True, exist_ok=True)

        for joint_idx in range(7):
            fig, ax = plt.subplots(figsize=(12, 5))
            ax.plot(
                np.arange(action_chunk.shape[0]),
                action_chunk[:, joint_idx],
                label="action_chunk",
                linewidth=2,
            )
            ax.plot(
                np.arange(sent_waypoints.shape[0]),
                sent_waypoints[:, joint_idx],
                label="sent_waypoint",
                linewidth=1.5,
            )
            if len(current_pos) > 0:
                ax.plot(
                    np.arange(current_pos.shape[0]),
                    current_pos[:, joint_idx],
                    label="plot_current_pos",
                    linewidth=1.2,
                )
            ##chunk end marker
            # for marker_idx in self._chunk_end_plot_indices:
            #     if 0 <= marker_idx < action_chunk.shape[0]:
            #         ax.axvline(
            #             x=marker_idx,
            #             color="black",
            #             linestyle="--",
            #             linewidth=1.0,
            #             alpha=0.5,
            #         )
            #         ax.scatter(
            #             marker_idx,
            #             action_chunk[marker_idx, joint_idx],
            #             color="red",
            #             s=18,
            #             zorder=5,
            #         )
            ax.set_title(f"Joint {joint_idx} Action Chunk vs Sent Waypoints")
            ax.set_xlabel("Step")
            ax.set_ylabel("Joint Position")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best")
            fig.tight_layout()
            save_path = self._plot_dir / f"joint_{joint_idx}.png"
            fig.savefig(save_path)
            plt.close(fig)

        pyzlc.info(
            "Saved joint comparison plots to "
            f"{self._plot_dir.resolve()} "
            f"(received_chunks={self._received_chunk_count}, "
            f"raw_action_points={len(self._action_chunk_joint_history)}, "
            f"executed_actions={self._executed_action_count}, "
            # f"expanded_action_points={len(self._expanded_action_chunk_joint_history)}, "
            f"sent_waypoints={len(self._sent_waypoint_joint_history)}, "
            # f"chunk_end_markers={len(self._chunk_end_plot_indices)}, "
            # f"plot_current_pos={len(self._plot_current_pos_joint_history)})"
        )

    def _generate_waypoints_within_limits(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        hz: float,
        max_joint_vel: Optional[np.ndarray] = None,
    ) -> tuple[torch.Tensor, np.ndarray]:
        """
        Generate waypoints that respect velocity limits.
        
        Args:
            start: Current joint positions (7,)
            goal: Target joint positions (7,)
            hz: Control frequency
            max_joint_vel: Maximum per-joint velocities in rad/s (7,)
        
        Returns:
            waypoints: Tensor of shape (n_steps, 7)
            feasible_vel: Feasible velocity (7,)
        """
        start = torch.as_tensor(start, dtype=torch.float32)
        goal = torch.as_tensor(goal, dtype=torch.float32)
        step_duration = 1.0 / hz
        delta = goal - start

        if max_joint_vel is None:
            feasible_vel = delta / step_duration
            n_steps = 1
        else:
            max_joint_vel_tensor = torch.as_tensor(
                max_joint_vel, dtype=torch.float32
            ).reshape(-1)
            if max_joint_vel_tensor.numel() != 7:
                raise ValueError(
                    f"Expected 7 per-joint velocity limits, got {max_joint_vel_tensor.numel()}"
                )
            if torch.any(max_joint_vel_tensor <= 0):
                raise ValueError("Per-joint velocity limits must be positive")

            required_time = torch.abs(delta) / max_joint_vel_tensor
            n_steps = max(1, int(torch.ceil(torch.max(required_time) * hz).item()))
            feasible_vel = delta / (n_steps * step_duration)

        if torch.max(torch.abs(delta)).item() < 1e-6:
            # No movement needed
            return torch.stack([goal]), feasible_vel.numpy()

        t = torch.linspace(0, 1, n_steps + 1)[1:]
        waypoints = (1 - t[:, None]) * start + t[:, None] * goal
        
        return waypoints, feasible_vel.numpy()

    def _send_waypoint_command(
        self, goal_joint_pos: np.ndarray, max_vel_norm_factor: float = 1.0
    ) -> np.ndarray:
        """
        Send one velocity-limited waypoint toward the target joint position.

        This helper is meant to be called once per control loop iteration.
        Sending the entire waypoint sequence in a single iteration would
        collapse the trajectory into a command burst and cause jerky motion.
        
        Args:
            goal_joint_pos: Target joint positions (7,)
            max_vel_norm_factor: Factor to scale max velocity (0.0 to 1.0)

        Returns:
            The joint position command that was sent.
        """
        goal_joint_pos = np.asarray(goal_joint_pos, dtype=np.float32).reshape(-1)
        if goal_joint_pos.size != 7:
            raise ValueError(f"Expected 7 joint targets, got {goal_joint_pos.size}")

        if self._last_joint_pos is None:
            current_joint_pos = self._get_current_joint_pos()
            if current_joint_pos is None:
                pyzlc.error("Current arm state not available, cannot generate waypoint command")
                return goal_joint_pos
            self._last_joint_pos = current_joint_pos

        max_joint_vel = VELOCITY_LIMITS_MAX * max_vel_norm_factor
        # print("debug:last_joint",self._last_joint_pos)
        # print("debug:goal_joint",goal_joint_pos)
        waypoints, _ = self._generate_waypoints_within_limits(
            self._last_joint_pos, goal_joint_pos, self.control_hz, max_joint_vel
        )
        # print(
        #     f"debug:Generated {len(waypoints)} waypoints with max joint velocity "
        #     f"{max_joint_vel} rad/s"
        # )
        expanded_goal = goal_joint_pos.copy()
        #too jerky to actuate the entire waypoint sequence in one control step, so we send one waypoint at a time in each control step. The next waypoint will be generated in the next control step based on the latest joint position, which ensures smoother motion and better adherence to velocity limits.
        for i in range(min(20, len(waypoints))):
            plot_current_pos = self._get_current_joint_pos()
            joint_cmd = (waypoints[i].numpy())
            self.panda_arm.send_joint_position_command(joint_cmd)

            self._last_joint_pos = np.asarray(joint_cmd, dtype=np.float32)
            self._expanded_action_chunk_joint_history.append(expanded_goal.copy())
            self._sent_waypoint_joint_history.append(self._last_joint_pos.copy())
            if plot_current_pos is None:
                self._plot_current_pos_joint_history.append(
                    np.full(7, np.nan, dtype=np.float32)
                )
            else:
                self._plot_current_pos_joint_history.append(
                    np.asarray(plot_current_pos, dtype=np.float32).copy()
                )
            #need to refine
            pyzlc.sleep(1.0 / self.control_hz)
        # self._last_joint_pos = self._get_current_joint_pos()
        ###
        ##only execute action once
        # joint_cmd = (
        #     waypoints[0].numpy() if len(waypoints) > 0 else goal_joint_pos.copy()
        # )
        # self.panda_arm.send_joint_position_command(joint_cmd)
        # self._last_joint_pos = np.asarray(joint_cmd, dtype=np.float32)
        ###
        return self._last_joint_pos.copy()


    def control_reset(self) -> None:
        self.panda_arm.set_franka_arm_control_mode(
            ControlMode.HybridJointImpedance
        )
        current_joint_pos = self._get_current_joint_pos()
        if current_joint_pos is None:
            pyzlc.error("Unable to seed control from current arm state during startup")
            return
        self._last_joint_pos = current_joint_pos.copy()
        self.panda_arm.send_joint_position_command(current_joint_pos)


    def go_home(self) -> None:
        self.panda_arm.move_franka_arm_to_joint_position(DEFAULT_POSITION)
        # Open the gripper
        if isinstance(self.gripper, RemoteRobotiqGripper):
            self.gripper.send_grasp_command(
                position=0.0,
                speed=GRIPPER_SPEED,
                force=GRIPPER_FORCE,
                blocking=True,
            )
        else:
            self.gripper.send_gripper_command(width=0.0, speed=0.1)

    def control_step(self) -> None:
        # start_time = time.perf_counter()
        # action = self._get_latest_action()
        action = self._get_latest_action_from_chunk()
        if action is None:
            pyzlc.sleep(1.0 / self.control_hz)
            return

        joint_pos = np.asarray(action[:7], dtype=np.float32)
        # print(f"Received action: joint_pos={joint_pos}, gripper_cmd={action[7]:.3f}")
        joint_pos = self._send_waypoint_command(joint_pos)
        self._executed_action_count += 1
        
        # Gripper command
        gripper_cmd = float(action[7])
        gripper_cmd = 1 if gripper_cmd >= 0.6 else 0

        if isinstance(self.gripper, RemoteRobotiqGripper):
            if (
                self._last_gripper_cmd is None
                or abs(gripper_cmd - self._last_gripper_cmd)
                > GRIPPER_DEADBAND
            ):
                self.gripper.send_grasp_command(
                    position=gripper_cmd,
                    speed=GRIPPER_SPEED,
                    force=GRIPPER_FORCE,
                    blocking=False,
                )
                self._last_gripper_cmd = gripper_cmd
        else:
            max_width = None
            state = self.gripper.current_state
            if state is not None:
                max_width = float(state.get("max_width", 0.0))
            if max_width is None or max_width <= 0.0:
                width = gripper_cmd
            else:
                width = gripper_cmd * max_width
            self.gripper.send_gripper_command(width=width, speed=0.1)
        # End_time = time.perf_counter()
        # print(f"command took {End_time - start_time:.3f} seconds")

   
    def control_end(self) -> None:
        ###plot
        self._save_joint_comparison_plots()
        self.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)

    def _control_task(self) -> None:
        try:
            self.control_reset()
            print("debug:velocity limit", VELOCITY_LIMITS_MAX)
            while self.is_running:
                start = time.perf_counter()
                self.control_step()
                end_time = time.perf_counter()
                # print(f"Control step took {end_time - start:.3f} seconds")
                elapsed = end_time - start
                sleep_time = (1.0 / self.control_hz)-elapsed
                if elapsed < (1.0 / self.control_hz) and elapsed >= 0.001 :
                    pyzlc.sleep(sleep_time)
               
            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
            traceback.print_exc()
