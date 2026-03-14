from __future__ import annotations

import threading
import time
from typing import Optional, Union

import numpy as np
import torch
import pyzlc
from collections import deque

from .control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode, RemotePandaArm
from ..franka_robot.panda_gripper import RemotePandaGripper
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper


DEFAULT_CONTROL_HZ: float = 1000
GRIPPER_DEADBAND: float = 1e-3
GRIPPER_SPEED = 0.7
GRIPPER_FORCE = 0.3
ACTION_LOG_INTERVAL_S: float = 0.5
GRIPPER_TOGGLE_WARN_WINDOW_S: float = 3.0
GRIPPER_TOGGLE_WARN_COUNT: int = 6
DEFAULT_POSITION = (0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0)

# Calculate velocity limits using the standard approach from training
VELOCITY_LIMITS = np.array([[-4 * np.pi / 2, 4 * np.pi / 2]] * 7).T / 32
VELOCITY_LIMITS_NORM = np.linalg.norm(VELOCITY_LIMITS)

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
        self._last_gripper_cmd: Optional[float] = None
        self._last_action_log_ts: float = 0.0
        self._last_gripper_binary: Optional[int] = None
        self._gripper_toggle_window_start_ts: float = time.time()
        self._gripper_toggle_count: int = 0
        
        # Velocity limiting state
        self._last_joint_pos: Optional[np.ndarray] = None
        self._last_control_time: Optional[float] = None
        self._dt = 1.0 / self.control_hz  # time delta between control steps

    def _get_current_joint_pos(self) -> Optional[np.ndarray]:
        current_state = self.panda_arm.current_state
        if current_state is None or "q" not in current_state:
            return None
        joint_pos = np.asarray(current_state["q"], dtype=np.float32).reshape(-1)
        if joint_pos.size != 7:
            pyzlc.warn(
                f"Unexpected current arm state size during control init: {joint_pos.size}"
            )
            return None
        return joint_pos
        
    #using by policy side to update the latest action, and control loop will read the latest action and execute it
    def update_action(self, action: np.ndarray) -> None:
        """Update the latest action used by the control loop."""
        arr = np.asarray(action, dtype=np.float64).reshape(-1)
        if arr.size < 8:
            raise ValueError(f"Expected action size >= 8, got {arr.size}")
        with self._action_lock:
            self._latest_action = arr

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

    
    def _get_latest_action(self) -> Optional[np.ndarray]:
        with self._action_lock:
            if self._latest_action is None:
                return None
            return self._latest_action.copy()
        
    def _get_latest_action_from_chunk(self) -> Optional[np.ndarray]:
        with self._action_lock:
            if self._latest_action_chunk:
                if len(self._latest_action_chunk) > 1:
                    action = self._latest_action_chunk.popleft()
                    # print("len of action chunk:", len(self._latest_action_chunk))
                    self._latest_action = self._latest_action_chunk[-1].copy()
                    return action.copy()
                #keep the latest action in the chunk as the current action until the next chunk comes in, to ensure smoother control when policy inference is faster than control loop
                # print("only one action in the chunk")
                self._latest_action = self._latest_action_chunk[0].copy()
                return self._latest_action.copy()

            if self._latest_action is None:
                return None
            return self._latest_action.copy()
    
    def reset_action(self) -> None:
        """Reset the latest action state when starting a new episode."""
        with self._action_lock:
            self._latest_action = None
            self._latest_action_chunk.clear()
        self._last_gripper_cmd = None
        self._last_gripper_binary = None
        self._gripper_toggle_count = 0
        self._gripper_toggle_window_start_ts = time.time()
        self._last_joint_pos = self._get_current_joint_pos()
        pyzlc.info("Action state reset for new episode")

    def _generate_waypoints_within_limits(
        self, start: np.ndarray, goal: np.ndarray, hz: float, max_vel_norm: float = float("inf")
    ) -> tuple[torch.Tensor, np.ndarray]:
        """
        Generate waypoints that respect velocity limits.
        
        Args:
            start: Current joint positions (7,)
            goal: Target joint positions (7,)
            hz: Control frequency
            max_vel_norm: Maximum velocity norm (default: infinity, no limit)
        
        Returns:
            waypoints: Tensor of shape (n_steps, 7)
            feasible_vel: Feasible velocity (7,)
        """
        start = torch.as_tensor(start, dtype=torch.float32)
        goal = torch.as_tensor(goal, dtype=torch.float32)
        
        step_duration = 1.0 / hz
        vel = (goal - start) / step_duration
        vel_norm = torch.norm(vel).item()
        
        if vel_norm > max_vel_norm:
            feasible_vel = (vel / vel_norm) * max_vel_norm
        else:
            feasible_vel = vel
        
        feasible_norm = torch.norm(feasible_vel).item()
        
        if feasible_norm < 1e-6:
            # No movement needed
            return torch.stack([goal]), feasible_vel.numpy()
        
        n_steps = int(np.ceil(vel_norm / feasible_norm))
        
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

        max_vel = VELOCITY_LIMITS_NORM * max_vel_norm_factor
        waypoints, _ = self._generate_waypoints_within_limits(
            self._last_joint_pos, goal_joint_pos, self.control_hz, max_vel
        )
        #too jerky to actuate the entire waypoint sequence in one control step, so we send one waypoint at a time in each control step. The next waypoint will be generated in the next control step based on the latest joint position, which ensures smoother motion and better adherence to velocity limits.
        # for i in range(len(waypoints)):
        #     joint_cmd = (waypoints[i].numpy())
        #     self.panda_arm.send_joint_position_command(joint_cmd)
        #     self._last_joint_pos = np.asarray(joint_cmd, dtype=np.float32)
        # print(f"Generated {len(waypoints)} waypoints with max velocity {max_vel:.3f} rad/s")
        joint_cmd = (
            waypoints[0].numpy() if len(waypoints) > 0 else goal_joint_pos.copy()
        )
        self.panda_arm.send_joint_position_command(joint_cmd)
        self._last_joint_pos = np.asarray(joint_cmd, dtype=np.float32)
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
        
        # Gripper command
        gripper_cmd = float(action[7])
        gripper_cmd = 1 if gripper_cmd >= 0.5 else 0

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

    def _log_action_debug(self, joint_pos: np.ndarray, gripper_cmd: float) -> None:
        now = time.time()
        if (now - self._last_action_log_ts) >= ACTION_LOG_INTERVAL_S:
            pyzlc.info(
                "Policy action: "
                f"q=[{', '.join(f'{x:.3f}' for x in joint_pos)}], "
                f"gripper={gripper_cmd:.3f}"
            )
            self._last_action_log_ts = now

        gripper_binary = 1 if gripper_cmd >= 0.5 else 0
        if self._last_gripper_binary is None:
            self._last_gripper_binary = gripper_binary
            self._gripper_toggle_window_start_ts = now
            self._gripper_toggle_count = 0
            return

        if gripper_binary != self._last_gripper_binary:
            self._gripper_toggle_count += 1
            self._last_gripper_binary = gripper_binary

        window_elapsed = now - self._gripper_toggle_window_start_ts
        if window_elapsed >= GRIPPER_TOGGLE_WARN_WINDOW_S:
            if self._gripper_toggle_count >= GRIPPER_TOGGLE_WARN_COUNT:
                pyzlc.warn(
                    "Gripper action toggling frequently: "
                    f"{self._gripper_toggle_count} toggles in "
                    f"{window_elapsed:.2f}s (threshold={GRIPPER_TOGGLE_WARN_COUNT}/"
                    f"{GRIPPER_TOGGLE_WARN_WINDOW_S:.1f}s)."
                )
            self._gripper_toggle_window_start_ts = now
            self._gripper_toggle_count = 0

    def control_end(self) -> None:
        self.panda_arm.set_franka_arm_control_mode(ControlMode.IDLE)

    def _control_task(self) -> None:
        try:
            self.control_reset()
            while self.is_running:
                start = time.perf_counter()
                self.control_step()
                # end_time = time.perf_counter()
                # print(f"Control step took {end_time - start:.3f} seconds")
                if time.perf_counter() - start < (1.0 / self.control_hz):
                    pyzlc.sleep((1.0 / self.control_hz) - (time.perf_counter() - start))
               
            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
