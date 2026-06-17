from __future__ import annotations

import threading
import time
import traceback
from typing import Optional, Sequence, Union

import numpy as np
import torch
import pyzlc
from collections import deque
from scipy.spatial.transform import Rotation as R

from .control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode, RemotePandaArm
from ..franka_robot.panda_gripper import RemotePandaGripper
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper


DEFAULT_CONTROL_HZ: float = 500
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


class CartesianPolicyPandaControlPair(ControlPair):
    """
    Apply policy actions to a Panda arm with a gripper.

    Action semantics are selected by action_rotation_mode:
    - "quat": [x, y, z, qx, qy, qz, qw, gripper]
    - "euler": [x, y, z, roll, pitch, yaw, gripper]

    action_pose_mode selects whether the pose values are absolute targets or
    deltas from the current end-effector pose.
    Gripper value is normalized in [0, 1]. It is scaled to device range.
    Includes velocity and acceleration limiting for safety.
    """

    def __init__(
        self,
        panda_arm: RemotePandaArm,
        gripper: Union[RemotePandaGripper, RemoteRobotiqGripper],
        control_hz: float = DEFAULT_CONTROL_HZ,
        action_rotation_mode: str = "quat",
        action_pose_mode: str = "absolute",
        action_gripper_mode: str = "absolute",
        home_joint_position: Optional[Sequence[float]] = None,
        home_gripper_position: float = 0.0,
    ) -> None:
        super().__init__()
        self.panda_arm = panda_arm
        self.gripper = gripper
        self.control_hz = float(control_hz)
        if home_joint_position is None:
            self.home_joint_position = tuple(DEFAULT_POSITION)
        else:
            home = tuple(float(x) for x in home_joint_position)
            if len(home) != 7:
                raise ValueError(
                    f"home_joint_position must have 7 joint values, got {len(home)}"
                )
            self.home_joint_position = home
        # Gripper closedness used by go_home/reset: 0.0 = fully open, 1.0 = closed.
        self.home_gripper_position = float(np.clip(home_gripper_position, 0.0, 1.0))
        self.action_rotation_mode = self._normalize_action_rotation_mode(
            action_rotation_mode
        )
        self.action_pose_mode = self._normalize_action_pose_mode(action_pose_mode)
        self.action_gripper_mode = self._normalize_action_gripper_mode(
            action_gripper_mode
        )
        self._action_lock = (
            threading.Lock()
        )  # only one of the update_action and control_step visit latest_action at the same time
        self._command_lock = threading.Lock()  # ensure thread-safe command sending
        self._lastest_command = None  # store the latest command for debugging or visualization
        self._latest_action: Optional[np.ndarray] = None
        self._latest_action_chunk: deque[np.ndarray] = deque()
        self._last_gripper_cmd: Optional[float] = None
        self._last_action_log_ts: float = 0.0
        self._last_gripper_binary: Optional[int] = None
        self._gripper_toggle_window_start_ts: float = time.time()
        self._gripper_toggle_count: int = 0
        self._active_delta_target_pose: Optional[np.ndarray] = None
        self._active_delta_gripper_cmd: Optional[float] = None
        # Full cartesian target [x, y, z, qx, qy, qz, qw] of the action the
        # control loop is currently executing. Reset to None by update_action
        # when a new action arrives and re-set by the control loop once it picks
        # that action up, so callers can tell when the arm has a fresh target to
        # chase and whether it has reached it.
        self._active_target_pose: Optional[np.ndarray] = None

        # Velocity limiting state
        self._last_cartesian_pos: Optional[np.ndarray] = None
        self._last_control_time: Optional[float] = None
        self._dt = 1.0 / self.control_hz  # time delta between control steps

        self.last_command = None

    def get_lastest_command(self) -> Optional[np.ndarray]:
        with self._command_lock:
            if self._lastest_command is not None:
                return np.append(self._lastest_command.copy(), self._last_gripper_cmd)
            return None

    def get_active_target_pose(self) -> Optional[np.ndarray]:
        """Return the full cartesian target [x, y, z, qx, qy, qz, qw] of the
        action the control loop is currently executing, or None if no action has
        been picked up since the last update_action (the loop hasn't processed it
        yet)."""
        with self._command_lock:
            if self._active_target_pose is None:
                return None
            return self._active_target_pose.copy()

    def get_current_cartesian_pose(self) -> Optional[np.ndarray]:
        """Return the live measured end-effector pose [x, y, z, qx, qy, qz, qw]."""
        return self._get_current_cartesian_pose()

    def clear_lastest_command(self) -> None:
        with self._command_lock:
            self._lastest_command = None

    def _get_current_cartesian_pose(self) -> Optional[np.ndarray]:
        current_state = self.panda_arm.current_state
        if current_state is None or "EE_pos" not in current_state:
            return None
        cartesian_pos = np.asarray(
            current_state["EE_pos"], dtype=np.float32
        ).reshape(-1)
        cartesian_rot = np.asarray(
            current_state["EE_quat"], dtype=np.float32
        ).reshape(-1)
        cartesian_pose = np.concatenate([cartesian_pos, cartesian_rot])
        if cartesian_pose.size != 7:
            pyzlc.error(
                f"Unexpected current arm state size during control init: {cartesian_pose.size}"
            )
            return None
        return cartesian_pose

    @staticmethod
    def _normalize_cartesian_quat(cartesian_pose: np.ndarray) -> np.ndarray:
        pose = np.asarray(cartesian_pose, dtype=np.float32).reshape(-1).copy()
        if pose.size != 7:
            raise ValueError(f"Expected cartesian pose size 7, got {pose.size}")
        quat_norm = float(np.linalg.norm(pose[3:7]))
        if quat_norm < 1e-8:
            raise ValueError("Cannot normalize zero-length cartesian quaternion.")
        pose[3:7] = pose[3:7] / quat_norm
        return pose

    @staticmethod
    def _normalize_action_rotation_mode(action_rotation_mode: str) -> str:
        mode = action_rotation_mode.lower()
        if mode in ("quat", "quaternion"):
            return "quat"
        if mode in ("euler", "ee_euler", "ee_euler_gripper"):
            return "euler"
        raise ValueError(
            "action_rotation_mode must be 'quat'/'quaternion' or "
            f"'euler'/'ee_euler_gripper', got {action_rotation_mode!r}"
        )

    @staticmethod
    def _normalize_action_pose_mode(action_pose_mode: str) -> str:
        mode = action_pose_mode.lower()
        if mode in ("absolute", "abs"):
            return "absolute"
        if mode in ("delta", "relative"):
            return "delta"
        raise ValueError(
            "action_pose_mode must be 'absolute'/'abs' or 'delta'/'relative', "
            f"got {action_pose_mode!r}"
        )

    @staticmethod
    def _normalize_action_gripper_mode(action_gripper_mode: str) -> str:
        mode = action_gripper_mode.lower()
        if mode in ("absolute", "abs", "binary"):
            return "absolute"
        if mode in ("delta", "relative"):
            return "delta"
        raise ValueError(
            "action_gripper_mode must be 'absolute'/'abs' or 'delta'/'relative', "
            f"got {action_gripper_mode!r}"
        )

    def _action_to_cartesian_pose(self, action: np.ndarray) -> np.ndarray:
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if self.action_rotation_mode == "quat":
            if action.size < 7:
                raise ValueError(
                    f"Expected quaternion action size >= 7, got {action.size}"
                )
            return action[:7].copy()

        if action.size < 6:
            raise ValueError(f"Expected euler action size >= 6, got {action.size}")
        pos = action[:3]
        euler = action[3:6]
        quat = R.from_euler("xyz", euler).as_quat().astype(np.float32)
        return np.concatenate([pos, quat]).astype(np.float32, copy=False)

    def _delta_action_to_cartesian_pose(self, action: np.ndarray) -> np.ndarray:
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.size < 6:
            raise ValueError(f"Expected delta action size >= 6, got {action.size}")

        # The policy delta is relative to the pose that was sent in the
        # observation, i.e. the current *measured* end-effector pose. Base the
        # delta on the live arm state (not the previously commanded target) so
        # deltas don't accumulate command drift. Fall back to the last command
        # only if the live state is momentarily unavailable.
        base_pose = self._get_current_cartesian_pose()
        if base_pose is None:
            base_pose = self._active_delta_target_pose
        if base_pose is None:
            base_pose = self._last_cartesian_pos
        if base_pose is None:
            raise ValueError("Current cartesian pose unavailable for delta action.")
        base_pose = self._normalize_cartesian_quat(base_pose)

        delta_pos = action[:3]
        delta_rot = action[3:6]
        target_pos = base_pose[:3] + delta_pos

        if self.action_rotation_mode == "euler":
            # Compose the euler delta in the base frame: R_new = R_delta @ R_current.
            # This matches the model's training convention (compute_new_pose),
            # which builds rotation matrices from euler "xyz" and pre-multiplies.
            target_quat = (
                R.from_euler("xyz", delta_rot) * R.from_quat(base_pose[3:7])
            ).as_quat()
        else:
            target_quat = (
                R.from_rotvec(delta_rot) * R.from_quat(base_pose[3:7])
            ).as_quat()

        return np.concatenate([target_pos, target_quat]).astype(np.float32, copy=False)

    def _get_gripper_action(self, action: np.ndarray) -> Optional[float]:
        min_size = 7 if self.action_rotation_mode == "euler" else 8
        if action.size < min_size:
            return None
        return float(action[-1])

    def _get_gripper_target(self, action_value: float) -> float:
        if self.action_gripper_mode == "absolute":
            return float(np.clip(action_value, 0.0, 1.0))

        current = 0.0
        if isinstance(self.gripper, RemoteRobotiqGripper):
            state = self.gripper.current_state
            if state is not None:
                current = float(state.get("position", 0.0))
            elif self._last_gripper_cmd is not None:
                current = float(self._last_gripper_cmd)
        else:
            state = self.gripper.current_state
            max_width = 0.0
            if state is not None:
                current = float(state.get("width", 0.0))
                max_width = float(state.get("max_width", 0.0))
            if max_width > 0.0:
                current = current / max_width

        return float(np.clip(current + action_value, 0.0, 1.0))

    def _send_gripper_command(self, gripper_cmd: float) -> None:
        gripper_cmd = float(np.clip(gripper_cmd, 0.0, 1.0))
        if isinstance(self.gripper, RemoteRobotiqGripper):
            if (
                self._last_gripper_cmd is None
                or abs(gripper_cmd - self._last_gripper_cmd) > GRIPPER_DEADBAND
            ):
                self.gripper.send_grasp_command(
                    position=gripper_cmd,
                    speed=GRIPPER_SPEED,
                    force=GRIPPER_FORCE,
                    blocking=False,
                )
                self._last_gripper_cmd = gripper_cmd
            return

        max_width = None
        state = self.gripper.current_state
        if state is not None:
            max_width = float(state.get("max_width", 0.0))
        if max_width is None or max_width <= 0.0:
            width = gripper_cmd
        else:
            width = gripper_cmd * max_width
        self.gripper.send_gripper_command(width=width, speed=0.1)

    def _get_latest_action_once(self) -> Optional[np.ndarray]:
        with self._action_lock:
            if self._latest_action is None:
                return None
            action = self._latest_action.copy()
            self._latest_action = None
            return action

    # using by policy side to update the latest action, and control loop will read the latest action and execute it
    def update_action(self, action: np.ndarray) -> None:
        """Update the latest action used by the control loop."""
        arr = np.asarray(action, dtype=np.float64).reshape(-1)
        if arr.size < 7:
            raise ValueError(f"Expected action size >= 7, got {arr.size}")
        with self._action_lock:
            self._latest_action = arr
        # Invalidate the published target: the control loop will re-publish it
        # once it picks up this new action, so a settle-wait can tell when the
        # arm is chasing the *new* target rather than the previous one.
        with self._command_lock:
            self._active_target_pose = None

    # using by policy side to update the latest action_chunk, and control loop will read the latest action and execute it
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

        if chunk.shape[-1] < 7:
            raise ValueError(
                f"Expected action size >= 7, got {chunk.shape[-1]}"
            )
        if chunk.shape[0] < 1 or chunk.shape[1] < 1:
            raise ValueError(
                f"Action chunk must contain at least one action, got {chunk.shape}"
            )

        action_queue = deque(
            np.array(action, copy=True) for action in chunk[0]
        )
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
                    # print("current action", action)
                    return action.copy()
                # keep the latest action in the chunk as the current action until the next chunk comes in, to ensure smoother control when policy inference is faster than control loop
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
        self._last_cartesian_pos = self._get_current_cartesian_pose()
        self._active_delta_target_pose = None
        self._active_delta_gripper_cmd = None
        with self._command_lock:
            self._active_target_pose = None
        pyzlc.info("Action state reset for new episode")

    def _generate_waypoints_within_limits(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        hz: float,
        max_vel_norm: float = float("inf"),
    ) -> tuple[torch.Tensor, np.ndarray]:
        """
        Generate waypoints that respect velocity limits.

        Args:
            start: Current cartesian positions (7,)
            goal: Target cartesian positions (7,)
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
        self, cartesian_waypoints: np.ndarray, max_vel_norm_factor: float = 1.0
    ) -> np.ndarray:
        """
        Send one velocity-limited waypoint toward the target cartesian position.

        This helper is meant to be called once per control loop iteration.
        Sending the entire waypoint sequence in a single iteration would
        collapse the trajectory into a command burst and cause jerky motion.

        Args:
            cartesian_waypoints: Target cartesian positions (7,)
            max_vel_norm_factor: Factor to scale max velocity (0.0 to 1.0)

        Returns:
            The cartesian position command that was sent.
        """
        cartesian_waypoints = np.asarray(
            cartesian_waypoints, dtype=np.float32
        ).reshape(-1)
        if cartesian_waypoints.size != 7:
            raise ValueError(
                f"Expected 7 cartesian targets, got {cartesian_waypoints.size}"
            )
        cartesian_waypoints = self._normalize_cartesian_quat(cartesian_waypoints)

        if self._last_cartesian_pos is None:
            current_cartesian_pos = self._get_current_cartesian_pose()
            if current_cartesian_pos is None:
                pyzlc.error(
                    "Current arm state not available, cannot generate waypoint command"
                )
                return cartesian_waypoints
            self._last_cartesian_pos = current_cartesian_pos

        max_vel = VELOCITY_LIMITS_NORM * max_vel_norm_factor
        waypoints, _ = self._generate_waypoints_within_limits(
            self._last_cartesian_pos,
            cartesian_waypoints,
            self.control_hz,
            max_vel,
        )
        # too jerky to actuate the entire waypoint sequence in one control step,
        # so we send one waypoint at a time in each control step.
        # The next waypoint will be generated in the next control step based on the latest cartesian position,
        # which ensures smoother motion and better adherence to velocity limits.
        # for i in range(len(waypoints)):
        #     cartesian_cmd = (waypoints[i].numpy())
        #     self.panda_arm.send_cartesian_position_command(cartesian_cmd)
        #     self._last_cartesian_pos = np.asarray(cartesian_cmd, dtype=np.float32)
        # print(f"Generated {len(waypoints)} waypoints with max velocity {max_vel:.3f} rad/s")
        cartesian_cmd = (
            waypoints[0].numpy()
            if len(waypoints) > 0
            else cartesian_waypoints.copy()
        )
        cartesian_cmd = self._normalize_cartesian_quat(cartesian_cmd)
        self.panda_arm.send_cartesian_pose_command(
            cartesian_cmd[:3], cartesian_cmd[3:7]
        )
        # print(f"Sent cartesian command: {cartesian_cmd}")
        self._last_cartesian_pos = np.asarray(cartesian_cmd, dtype=np.float32)
        with self._command_lock:
            self._lastest_command = cartesian_cmd.copy()
        return self._last_cartesian_pos.copy()

    def control_reset(self) -> None:
        self.panda_arm.set_franka_arm_control_mode(
            ControlMode.CartesianImpedance
        )
        current_cartesian_pos = self._get_current_cartesian_pose()
        if current_cartesian_pos is None:
            pyzlc.error(
                "Unable to seed control from current arm state during startup"
            )
            return
        self._last_cartesian_pos = current_cartesian_pos.copy()
        self.panda_arm.send_cartesian_pose_command(
            current_cartesian_pos[:3], current_cartesian_pos[3:7]
        )

    def go_home(self) -> None:
        self.panda_arm.move_franka_arm_to_joint_position(self.home_joint_position)
        # Drive the gripper to the configured home closedness
        # (0.0 = open, 1.0 = closed).
        gripper_cmd = float(np.clip(self.home_gripper_position, 0.0, 1.0))
        if isinstance(self.gripper, RemoteRobotiqGripper):
            self.gripper.send_grasp_command(
                position=gripper_cmd,
                speed=GRIPPER_SPEED,
                force=GRIPPER_FORCE,
                blocking=True,
            )
        else:
            max_width = 0.0
            state = self.gripper.current_state
            if state is not None:
                max_width = float(state.get("max_width", 0.0))
            width = (1.0 - gripper_cmd) * max_width if max_width > 0.0 else 0.0
            self.gripper.send_gripper_command(width=width, speed=0.1)
        self._last_gripper_cmd = gripper_cmd

    def control_step(self) -> None:
        if self.action_pose_mode == "delta":
            self._control_step_delta()
            return

        # start_time = time.perf_counter()
        # action = self._get_latest_action()
        action = self._get_latest_action_from_chunk()
        if action is None:
            pyzlc.sleep(1.0 / self.control_hz)
            return

        cartesian_pos = self._action_to_cartesian_pose(action)
        # print(f"Received action: cartesian_pos={cartesian_pos}, gripper_cmd={action[-1]:.3f}")
        with self._command_lock:
            self._active_target_pose = cartesian_pos.copy()
        cartesian_pos = self._send_waypoint_command(cartesian_pos)

        gripper_action = self._get_gripper_action(action)
        if gripper_action is None:
            return

        # Gripper command
        gripper_cmd = self._get_gripper_target(gripper_action)
        action[-1] = gripper_cmd

        self._send_gripper_command(gripper_cmd)
        # End_time = time.perf_counter()
        # print(f"command took {End_time - start_time:.3f} seconds")

    def _control_step_delta(self) -> None:
        action = self._get_latest_action_once()
        if action is not None:
            self._active_delta_target_pose = self._delta_action_to_cartesian_pose(action)
            with self._command_lock:
                self._active_target_pose = self._active_delta_target_pose.copy()
            gripper_action = self._get_gripper_action(action)
            if gripper_action is not None:
                self._active_delta_gripper_cmd = self._get_gripper_target(
                    gripper_action
                )
            now = time.time()
            if now - self._last_action_log_ts >= ACTION_LOG_INTERVAL_S:
                pyzlc.info(
                    "Cartesian delta action: "
                    f"dpos=[{', '.join(f'{x:.4f}' for x in action[:3])}], "
                    f"drot=[{', '.join(f'{x:.4f}' for x in action[3:6])}], "
                    f"target_pos=[{', '.join(f'{x:.4f}' for x in self._active_delta_target_pose[:3])}], "
                    f"gripper={self._active_delta_gripper_cmd}"
                )
                self._last_action_log_ts = now

        if self._active_delta_target_pose is None:
            pyzlc.sleep(1.0 / self.control_hz)
            return

        self._send_waypoint_command(self._active_delta_target_pose)
        if self._active_delta_gripper_cmd is not None:
            self._send_gripper_command(self._active_delta_gripper_cmd)

    def _log_action_debug(
        self, joint_pos: np.ndarray, gripper_cmd: float
    ) -> None:
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
                    pyzlc.sleep(
                        (1.0 / self.control_hz) - (time.perf_counter() - start)
                    )

            self.control_end()
        except Exception as e:
            print(f"Control task encountered an error: {e}")
            traceback.print_exc()
