from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import pyzlc
from scipy.spatial.transform import Rotation as R

from franka_control_client.control_pair.cartesian_policy_panda_control_pair import (
    CartesianPolicyPandaControlPair,
)

from .irl_wrapper import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    PandaGripperDataWrapper,
    RobotiqGripperDataWrapper,
)
from .policy_inference_manager import (
    PolicyInferenceEvent,
    PolicyInferenceManager,
    PolicyInferenceState,
)
from .policy_server_client import PolicyServerClient


class RemotePolicyInference(PolicyInferenceManager):
    def __init__(
        self,
        data_collectors: List[IRL_HardwareDataWrapper],
        control_pair: CartesianPolicyPandaControlPair,
        task: str,
        fps: int = 4,
        server_host: str = "127.0.0.1",
        server_port: int = 8765,
        request_timeout_s: float = 30.0,
        state_mode: str = "ee_euler_gripper",
        include_force_torque: bool = True,
        expected_image_shape: Optional[tuple[int, int, int]] = None,
        camera_key_map: Optional[Dict[str, str]] = None,
        goal_dir: Optional[str] = None,
        goal_left_filename: str = "left.png",
        goal_wrist_filename: str = "wrist.png",
        goal_state_filename: str = "state.npy",
        goal_instruction: str = "goal",
        goal_pos_tol: float = 0.02,
        goal_rot_tol: float = 0.1,
        goal_gripper_tol: float = 0.1,
    ) -> None:
        super().__init__(task=task, fps=fps)
        self.data_collectors = data_collectors
        self.control_pair = control_pair
        self.client = PolicyServerClient(
            host=server_host, port=server_port, timeout_s=request_timeout_s
        )
        self.state_mode = state_mode
        self.include_force_torque = include_force_torque
        self.expected_image_shape = expected_image_shape
        self.camera_key_map = camera_key_map
        self.goal_dir = goal_dir
        self.goal_left_filename = goal_left_filename
        self.goal_wrist_filename = goal_wrist_filename
        self.goal_state_filename = goal_state_filename
        self.goal_instruction = goal_instruction
        self.goal_pos_tol = goal_pos_tol
        self.goal_rot_tol = goal_rot_tol
        self.goal_gripper_tol = goal_gripper_tol
        self._goal_state: Optional[np.ndarray] = None
        self._last_state_vector: Optional[np.ndarray] = None
        self._last_action_log_ts = 0.0

        self.cameras: List[ImageDataWrapper] = []
        self.arm_wrapper: Optional[PandaArmDataWrapper] = None
        self.gripper_wrapper: Optional[IRL_HardwareDataWrapper] = None
        for hw in data_collectors:
            if isinstance(hw, ImageDataWrapper) or hw.hw_type == "camera":
                self.cameras.append(hw)  # type: ignore[arg-type]
            elif isinstance(hw, PandaArmDataWrapper) or hw.hw_type == "follower_arm":
                self.arm_wrapper = hw  # type: ignore[assignment]
            elif (
                isinstance(hw, (PandaGripperDataWrapper, RobotiqGripperDataWrapper))
                or hw.hw_type == "follower_gripper"
            ):
                self.gripper_wrapper = hw

        if self.arm_wrapper is None:
            raise ValueError("Missing PandaArmDataWrapper for inference.")
        if self.gripper_wrapper is None:
            raise ValueError("Missing gripper wrapper for inference.")

        health = self.client.health()
        pyzlc.info(f"Connected to policy server: {health.get('policy', health)}")

        self.register_start_infering_event(self.control_pair.start_control_pair)
        self.register_stop_infering_event(self.control_pair.stop_control_pair)

    def _infer_step(self) -> None:
        start_time = time.perf_counter()
        observation = self._build_observation_payload()

        if self._goal_reached(self._last_state_vector):
            pyzlc.info("Goal state reached; stopping episode.")
            self._state_machine.trigger(PolicyInferenceEvent.SAVE)
            return

        response = self._infer_interruptible(observation)
        # The user pressed 'd'/'s'/'q' while we were waiting for the server, so
        # we already left the INFERING state. Drop the (now stale) action.
        if response is None:
            return

        action = self._extract_action(response)
        self.control_pair.update_action(action)

        elapsed = time.perf_counter() - start_time
        now = time.time()
        if now - self._last_action_log_ts >= max(0.0, (1.0 / self.fps) - 0.001):
            action_source = response.get("source", "policy")
            pyzlc.info(
                f"Remote {action_source} action: "
                f"[{', '.join(f'{x:.4f}' for x in action.reshape(-1))}] "
                f"in {elapsed:.3f}s"
            )
            self._last_action_log_ts = now
        sleep_time = max(0.0, (1.0 / self.fps) - elapsed)
        if sleep_time > 0.001:
            time.sleep(sleep_time)

    def _infer_interruptible(self, observation: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Send the inference request in a background thread so key presses
        (e.g. 'd' to discard) are handled immediately instead of only after the
        server returns its action.

        Returns the server response, or ``None`` if a key press moved us out of
        the INFERING state while we were waiting (the request is left to drain
        in the background and its action is discarded).
        """
        result: Dict[str, Any] = {}

        def _worker() -> None:
            try:
                result["response"] = self.client.infer(observation)
            except Exception as exc:  # noqa: BLE001
                result["error"] = exc

        worker = threading.Thread(target=_worker, daemon=True)
        worker.start()

        while worker.is_alive():
            if self._kp is not None:
                key = self._kp.get_data()
                if key:
                    self._handle_keypress(key)
                    if self._state_machine.state != PolicyInferenceState.INFERING:
                        return None
            time.sleep(0.001)

        worker.join()
        if "error" in result:
            raise result["error"]
        return result.get("response")

    def _build_observation_payload(self) -> Dict[str, Any]:
        arm_state = self.arm_wrapper.capture_step()
        grip_state = self.gripper_wrapper.capture_step()

        state_vector = self._build_state_vector(arm_state, grip_state)
        self._last_state_vector = state_vector

        observation: Dict[str, Any] = {
            "observation.state": state_vector,
            "task": self.task,
        }

        if self.include_force_torque:
            observation["observation.force_torque"] = self._build_force_torque_vector(
                arm_state, grip_state
            )

        for cam in self.cameras:
            frame = cam.capture_step()
            if frame is None:
                continue
            if (
                self.expected_image_shape is not None
                and tuple(frame.shape) != self.expected_image_shape
            ):
                raise ValueError(
                    f"Expected {cam.hw_name} image shape "
                    f"{self.expected_image_shape}, got {tuple(frame.shape)}."
                )
            observation[self._camera_observation_key(cam.hw_name)] = np.ascontiguousarray(
                frame
            )

        return observation

    def _extract_action(self, response: Dict[str, Any]) -> np.ndarray:
        action = np.asarray(response["action"], dtype=np.float32)
        if action.ndim == 3:
            if action.shape[0] != 1:
                raise ValueError(
                    f"Expected action batch size 1, got action shape {action.shape}."
                )
            action = action[0]
        if action.ndim == 2:
            if action.shape[0] > 1:
                pyzlc.warn(
                    "Policy returned an action chunk; using the first action for "
                    f"the current control step. action_shape={action.shape}"
                )
            action = action[0]
        if action.ndim != 1:
            raise ValueError(f"Expected 1D policy action, got shape {action.shape}.")
        return action.astype(np.float32, copy=False)

    def _build_state_vector(self, arm_state: Any, grip_state: Any) -> np.ndarray:
        if not isinstance(arm_state, dict):
            raise ValueError("Arm state must be a dict.")

        mode = self.state_mode.lower()
        if mode in ("ee", "ee_euler", "ee_euler_gripper"):
            ee_pos = arm_state.get("EE_pos")
            ee_quat = arm_state.get("EE_quat")
            if ee_pos is None or ee_quat is None:
                raise ValueError("Arm state missing EE_pos or EE_quat for ee state_mode.")
            pos = np.asarray(ee_pos, dtype=np.float32).reshape(-1)
            quat = np.asarray(ee_quat, dtype=np.float32).reshape(-1)
            euler = R.from_quat(quat).as_euler("xyz").astype(np.float32)
            gripper_val = self._get_gripper_value(grip_state)
            state = np.concatenate([pos, euler, np.asarray([gripper_val], dtype=np.float32)])
            if state.size != 7:
                raise ValueError(f"Expected EE/euler/gripper state size 7, got {state.size}.")
            return state.astype(np.float32, copy=False)

        if mode == "joint":
            q = arm_state.get("q")
            if q is None:
                raise ValueError("Arm state missing q for joint state_mode.")
            state = np.asarray(q, dtype=np.float32).reshape(-1)
            if state.size != 7:
                raise ValueError(f"Expected joint state size 7, got {state.size}.")
            return state.astype(np.float32, copy=False)

        raise ValueError(f"Unknown state_mode {self.state_mode!r}; expected 'ee' or 'joint'.")

    def _get_gripper_value(self, grip_state: Any) -> float:
        if not isinstance(grip_state, dict):
            raise ValueError("Gripper state must be a dict.")
        for key in ("position", "width", "gripper"):
            if key not in grip_state:
                continue
            value = grip_state[key]
            arr = np.asarray(value, dtype=np.float32).reshape(-1)
            if arr.size > 0:
                return float(arr[0])
        raise ValueError("Gripper state missing position/width value.")

    def _build_force_torque_vector(self, arm_state: Any, grip_state: Any) -> np.ndarray:
        if not isinstance(arm_state, dict) or not isinstance(grip_state, dict):
            raise ValueError("Arm and gripper states must be dicts for force_torque.")

        joint_torque = arm_state.get("tau_ext_hat_filtered")
        external_wrench = arm_state.get("O_F_ext_hat_K")
        gripper_current = grip_state.get("current")
        if joint_torque is None or external_wrench is None or gripper_current is None:
            raise ValueError(
                "Missing force_torque source: expected tau_ext_hat_filtered, "
                "O_F_ext_hat_K, and gripper current."
            )

        force_torque = np.concatenate(
            [
                np.asarray(joint_torque, dtype=np.float32).reshape(-1),
                np.asarray(external_wrench, dtype=np.float32).reshape(-1),
                np.asarray([gripper_current], dtype=np.float32),
            ]
        )
        if force_torque.size != 14:
            raise ValueError(f"Expected force_torque size 14, got {force_torque.size}.")
        return force_torque.astype(np.float32, copy=False)

    def _camera_observation_key(self, hw_name: str) -> str:
        if self.camera_key_map is not None:
            if hw_name not in self.camera_key_map:
                raise ValueError(
                    f"Camera '{hw_name}' has no entry in camera_key_map "
                    f"{sorted(self.camera_key_map)}; cannot map it to a server "
                    "observation key."
                )
            return self.camera_key_map[hw_name]
        camera_key_map = {
            "zed_left": "observation.images.left",
            "left": "observation.images.left",
            "left_cam": "observation.images.left",
            "zed_wrist": "observation.images.wrist",
            "wrist": "observation.images.wrist",
            "wrist_cam": "observation.images.wrist",
        }
        return camera_key_map.get(hw_name, f"observation.images.{hw_name}")

    def _start_infering(self) -> None:
        self.client.reset()
        self._load_goal()
        self.control_pair.reset_action()
        super()._start_infering()

    def _load_goal(self) -> None:
        """Load goal images (and optional goal state) from disk and send them
        to the server via set_goal. The VALPA server is goal-conditioned and
        rejects infer requests until a goal has been set."""
        self._goal_state = None
        if self.goal_dir is None:
            pyzlc.warn(
                "No goal_dir configured; skipping set_goal. The goal-conditioned "
                "server will reject inference until a goal is set."
            )
            return

        goal_path = Path(self.goal_dir)
        left = self._read_goal_image(goal_path / self.goal_left_filename)
        wrist = self._read_goal_image(goal_path / self.goal_wrist_filename)
        self.client.set_goal(left, wrist, instruction=self.goal_instruction)
        pyzlc.info(f"Goal images set from {goal_path}")

        state_file = goal_path / self.goal_state_filename
        if state_file.exists():
            goal_state = np.asarray(np.load(state_file), dtype=np.float32).reshape(-1)
            if goal_state.size < 7:
                raise ValueError(
                    f"Goal state {state_file} must have at least 7 values "
                    f"[x, y, z, roll, pitch, yaw, gripper], got {goal_state.size}."
                )
            self._goal_state = goal_state[:7]
            pyzlc.info(
                "Goal state loaded for reached-check: "
                f"[{', '.join(f'{x:.4f}' for x in self._goal_state)}]"
            )
        else:
            pyzlc.warn(
                f"No goal state file at {state_file}; goal-reached auto-stop "
                "is disabled (will run until you press 's'/'d')."
            )

    def _read_goal_image(self, path: Path) -> np.ndarray:
        if not path.exists():
            raise FileNotFoundError(f"Goal image not found: {path}")
        img_bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if img_bgr is None:
            raise ValueError(f"Failed to read goal image: {path}")
        img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        if self.expected_image_shape is not None:
            h, w = self.expected_image_shape[0], self.expected_image_shape[1]
            if img.shape[0] != h or img.shape[1] != w:
                img = cv2.resize(img, (w, h))
        return np.ascontiguousarray(img.astype(np.uint8))

    def _goal_reached(self, state: Optional[np.ndarray]) -> bool:
        if self._goal_state is None or state is None:
            return False
        state = np.asarray(state, dtype=np.float32).reshape(-1)
        if state.size < 7:
            return False
        pos_err = float(np.linalg.norm(state[:3] - self._goal_state[:3]))
        rot_err = float(
            (
                R.from_euler("xyz", state[3:6]).inv()
                * R.from_euler("xyz", self._goal_state[3:6])
            ).magnitude()
        )
        grip_err = abs(float(state[6]) - float(self._goal_state[6]))
        return (
            pos_err <= self.goal_pos_tol
            and rot_err <= self.goal_rot_tol
            and grip_err <= self.goal_gripper_tol
        )

    def _save_episode(self) -> None:
        self._stop_infering()
        self._ui_console.log("Episode stopped.")

    def _discard_infering(self) -> None:
        super()._discard_infering()

    def _stop_infering(self) -> None:
        super()._stop_infering()

    def _reset_arm(self) -> None:
        if hasattr(self.control_pair, "go_home"):
            self.control_pair.go_home()

    def _close(self) -> None:
        self.client.close()
        super()._close()
