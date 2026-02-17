from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Any

import numpy as np
import pyzlc

from .policy_inference_manager import PolicyInferenceManager
from .irl_wrapper import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    PandaGripperDataWrapper,
    RobotiqGripperDataWrapper,
)
from ..policy.policy import RemotePolicy
from ..control_pair.policy_panda_control_pair import PolicyPandaControlPair


@dataclass
class LeRobotPolicyInferenceConfig:
    policy_name: str
    task: str
    fps: int = 30
    obs_topic: Optional[str] = None
    action_topic: Optional[str] = None


class LeRobotPolicyInference(PolicyInferenceManager):
    """
    Policy inference loop that publishes observations and applies actions.

    Observation format matches src/franka_control_client/policy/lerobot_node.py.
    Action semantics: [q0..q6, gripper].
    """

    def __init__(
        self,
        data_collectors: List[IRL_HardwareDataWrapper],
        control_pair: PolicyPandaControlPair,
        cfg: LeRobotPolicyInferenceConfig,
    ) -> None:
        super().__init__(task=cfg.task, fps=cfg.fps)
        self.data_collectors = data_collectors
        self.control_pair = control_pair
        self.cfg = cfg
        self.policy = RemotePolicy(
            cfg.policy_name, obs_topic=cfg.obs_topic, action_topic=cfg.action_topic
        )

        self.cameras: List[ImageDataWrapper] = []
        self.arm_wrapper: Optional[PandaArmDataWrapper] = None
        self.gripper_wrapper: Optional[
            IRL_HardwareDataWrapper
        ] = None
        for hw in data_collectors:
            if isinstance(hw, ImageDataWrapper) or hw.hw_type == "camera":
                self.cameras.append(hw)  # type: ignore[arg-type]
            elif isinstance(hw, PandaArmDataWrapper) or hw.hw_type == "follower_arm":
                self.arm_wrapper = hw  # type: ignore[assignment]
            elif isinstance(hw, (PandaGripperDataWrapper, RobotiqGripperDataWrapper)) or hw.hw_type == "follower_gripper":
                self.gripper_wrapper = hw

        if self.arm_wrapper is None:
            raise ValueError("Missing PandaArmDataWrapper for inference.")
        if self.gripper_wrapper is None:
            raise ValueError("Missing gripper wrapper for inference.")

        # Auto-hook control start/stop to inference events.
        self.register_start_infering_event(self.control_pair.start_control_pair)
        self.register_stop_infering_event(self.control_pair.stop_control_pair)

    def _start_infering(self) -> None:
        super()._start_infering()
        self.last_timestamp = None

    def _infer_step(self) -> None:
        if self.last_timestamp is None:
            self.last_timestamp = time.perf_counter()

        obs = self._build_observation()
        self.policy.send_observation(obs)

        action_msg = self.policy.current_action
        if action_msg is not None:
            action = np.asarray(action_msg["action"], dtype=np.float64).reshape(-1)
            try:
                self.control_pair.update_action(action)
            except Exception as exc:
                pyzlc.error(f"Failed to apply policy action: {exc}")

        elapsed = time.perf_counter() - self.last_timestamp
        sleep_time = max(0.0, (1.0 / self.fps) - elapsed)
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        self.last_timestamp = time.perf_counter()

    def _save_episode(self) -> None:
        self._stop_infering()
        self._ui_console.log("Episode saved.")

    def _discard_infering(self) -> None:
        self._stop_infering()
        self._ui_console.log("Episode discarded.")

    def _stop_infering(self) -> None:
        super()._stop_infering()

    def _build_observation(self) -> Dict[str, Any]:
        state_vec = self._build_state_vector()
        images = self._build_images()
        return {
            "state": state_vec.tolist(),
            "images": images,
            "task": self.task,
        }

    def _build_state_vector(self) -> np.ndarray:
        arm_state = self.arm_wrapper.capture_step()
        q = None
        if isinstance(arm_state, dict):
            if "q" in arm_state:
                q = np.asarray(arm_state["q"], dtype=np.float32).reshape(-1)
            elif "joint_state" in arm_state:
                q = np.asarray(arm_state["joint_state"], dtype=np.float32).reshape(-1)
        if q is None or q.size != 7:
            raise ValueError("Arm state missing valid joint positions.")

        grip_state = self.gripper_wrapper.capture_step()
        gripper_val = None
        if isinstance(grip_state, dict):
            if "width" in grip_state:
                gripper_val = float(grip_state["width"])
            elif "position" in grip_state:
                gripper_val = float(grip_state["position"])
            elif "gripper" in grip_state:
                gripper_arr = np.asarray(grip_state["gripper"], dtype=np.float32).reshape(-1)
                if gripper_arr.size > 0:
                    gripper_val = float(gripper_arr[0])
        if gripper_val is None:
            raise ValueError("Gripper state missing value.")

        return np.concatenate([q, np.asarray([gripper_val], dtype=np.float32)])

    def _build_images(self) -> Dict[str, Any]:
        images: Dict[str, Any] = {}
        for cam in self.cameras:
            frame = cam.capture_step()
            if frame is None:
                continue
            if isinstance(frame, np.ndarray):
                h, w, c = frame.shape
                images[cam.hw_name] = {
                    "height": int(h),
                    "width": int(w),
                    "channels": int(c),
                    "rgb_data": frame.tobytes(),
                }
            else:
                images[cam.hw_name] = frame
        return images
