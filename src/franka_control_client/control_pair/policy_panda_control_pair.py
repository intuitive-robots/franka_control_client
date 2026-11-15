from __future__ import annotations

import threading
import time
import traceback
from typing import Optional, Union

import numpy as np
import pyzlc
from collections import deque

from .control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode, RemotePandaArm
from ..franka_robot.panda_gripper import RemotePandaGripper
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper


DEFAULT_CONTROL_HZ: float = 100
GRIPPER_DEADBAND: float = 1e-3
GRIPPER_SPEED = 0.7
GRIPPER_FORCE = 0.3
ACTION_LOG_INTERVAL_S: float = 0.5
GRIPPER_TOGGLE_WARN_WINDOW_S: float = 3.0
GRIPPER_TOGGLE_WARN_COUNT: int = 6
DEFAULT_POSITION = (0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0)
# Velocity-limited waypoints advanced per control step (so the effective joint
# speed cap is WAYPOINTS_PER_STEP * VELOCITY_LIMITS_MAX).
WAYPOINTS_PER_STEP = 3

# Calculate velocity limits using the standard approach from training
# VELOCITY_LIMITS = np.array([[-4 * np.pi / 2, 4 * np.pi / 2]] * 7).T / 16
# VELOCITY_LIMITS_NORM = np.linalg.norm(VELOCITY_LIMITS)
# Franka joint velocity limits in rad/s for joints 1..7.
VELOCITY_LIMITS = np.array(
    [
        # [-2.62, -2.62, -2.62, -2.62, -5.26, -4.18, -5.26],
        # [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26],
        [-2.62, -2.62, -2.62, -2.62, -0.5, -2, -2],
        [2.62, 2.62, 2.62, 2.62, 0.5, 2, 2],
    ],
    dtype=np.float32,
)
VELOCITY_LIMIT_SCALE = 0.5
VELOCITY_LIMITS_MAX = VELOCITY_LIMITS[1] * VELOCITY_LIMIT_SCALE

class PolicyPandaControlPair(ControlPair):
    """
    Apply policy actions to a Panda arm with a gripper.

    Action semantics: [q0..q6, gripper] (joint positions + gripper).
    Gripper value is normalized in [0, 1]. It is scaled to device range.
    Includes velocity and acceleration limiting for safety.
    """

    # Joint pose for go_home(); the inference GUI overrides it per instance.
    home_position = DEFAULT_POSITION

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
            pyzlc.error(
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
        
    def  _get_latest_action_from_chunk(self) -> Optional[np.ndarray]:
        with self._action_lock:
            if self._latest_action_chunk:
                if len(self._latest_action_chunk) > 1:
                    action = self._latest_action_chunk.popleft()
                    # print("len of action chunk:", len(self._latest_action_chunk))
                    self._latest_action = self._latest_action_chunk[-1].copy()
                    # print("current action", action)
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
        self,
        start: np.ndarray,
        goal: np.ndarray,
        hz: float,
        max_joint_vel: Optional[np.ndarray] = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Generate waypoints that respect velocity limits.

        Pure numpy on purpose: this runs in the control thread every step, and
        each torch call releases and re-acquires the GIL. With another busy
        Python thread in the process every re-acquire can wait a full switch
        interval (5 ms), which stretched control steps to 100-300 ms.

        Args:
            start: Current joint positions (7,)
            goal: Target joint positions (7,)
            hz: Control frequency
            max_joint_vel: Maximum per-joint velocities in rad/s (7,)

        Returns:
            waypoints: Array of shape (n_steps, 7)
            feasible_vel: Feasible velocity (7,)
        """
        start = np.asarray(start, dtype=np.float32).reshape(-1)
        goal = np.asarray(goal, dtype=np.float32).reshape(-1)
        step_duration = 1.0 / hz
        delta = goal - start

        if max_joint_vel is None:
            feasible_vel = delta / step_duration
            n_steps = 1
        else:
            max_vel = np.asarray(max_joint_vel, dtype=np.float32).reshape(-1)
            if max_vel.size != 7:
                raise ValueError(f"Expected 7 per-joint velocity limits, got {max_vel.size}")
            if np.any(max_vel <= 0):
                raise ValueError("Per-joint velocity limits must be positive")

            required_time = np.abs(delta) / max_vel
            n_steps = max(1, int(np.ceil(float(np.max(required_time)) * hz)))
            feasible_vel = delta / (n_steps * step_duration)

        if float(np.max(np.abs(delta))) < 1e-6:
            # No movement needed
            return goal[None, :].copy(), feasible_vel

        t = np.linspace(0.0, 1.0, n_steps + 1, dtype=np.float32)[1:]
        waypoints = (1 - t[:, None]) * start + t[:, None] * goal

        return waypoints, feasible_vel

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
            # f"debug:Generated {len(waypoints)} waypoints with max joint velocity "
            # f"{max_joint_vel} rad/s"s
        # )
        # Advance at most WAYPOINTS_PER_STEP waypoints per control step. This used
        # to publish waypoints 0..2 back to back with a 1 ms sleep in between,
        # which ends at the same target; publishing only that target once gives
        # the same motion without the extra sleeps / GIL hand-offs.
        joint_cmd = np.asarray(
            waypoints[min(WAYPOINTS_PER_STEP, len(waypoints)) - 1], dtype=np.float32
        )
        self.panda_arm.send_joint_position_command(joint_cmd)
        self._last_joint_pos = joint_cmd.copy()
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
        self.panda_arm.move_franka_arm_to_joint_position(tuple(self.home_position))
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
            print("debug:velocity limit", VELOCITY_LIMITS_MAX)
            period = 1.0 / self.control_hz
            next_t = time.perf_counter()
            while self.is_running:
                self.control_step()
                # Fixed-rate schedule: sleep until the next slot. After an overrun,
                # restart the grid from now instead of bursting to catch up.
                next_t += period
                delay = next_t - time.perf_counter()
                if delay > 0:
                    pyzlc.sleep(delay)
                else:
                    next_t = time.perf_counter()

            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
            traceback.print_exc()
