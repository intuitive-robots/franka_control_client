from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

import numpy as np
import pyzlc

from franka_control_client.control_pair.policy_panda_control_pair import (
    PolicyPandaControlPair,
)

from .irl_wrapper import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    PandaGripperDataWrapper,
    RobotiqGripperDataWrapper,
)
from .policy_inference_manager import PolicyInferenceEvent, PolicyInferenceManager
from .policy_server_client import PolicyServerClient


JOINT_ACTION_SPACE = "joint_pos_abs_gripper_closedness"


class RemoteJointPolicyInference(PolicyInferenceManager):
    """
    Policy inference loop against a remote joint-space policy server
    (e.g. wam_ssm_droid_policy_server.py).

    Observation sent: {"observation.state": [q0..q6, gripper_closedness],
                       "observation.images.<...>": HxWx3 uint8 RGB, "task": str}
    Action received:  chunk [T, 8] of ABSOLUTE joint targets + gripper closedness
                      (0 = open, 1 = closed), one row per 1/fps s. Only the first
                      exec_steps rows are executed before the next request.

    The loop is synchronous: observe -> infer -> execute exec_steps rows ->
    observe ... The server feeds the executed rows back into the model's memory,
    so every requested chunk prefix is executed completely. While the server is
    computing, the arm holds the last target of the previous chunk.
    """

    def __init__(
        self,
        data_collectors: List[IRL_HardwareDataWrapper],
        control_pair: PolicyPandaControlPair,
        task: str,
        fps: int = 15,
        server_host: str = "127.0.0.1",
        server_port: int = 8767,
        request_timeout_s: Optional[float] = None,
        camera_key_map: Optional[Dict[str, str]] = None,
        expected_image_shape: Optional[tuple[int, int, int]] = None,
        max_first_step_joint_delta: float = 0.35,
    ) -> None:
        super().__init__(task=task, fps=fps)
        self.data_collectors = data_collectors
        self.control_pair = control_pair
        self.client = PolicyServerClient(
            host=server_host, port=server_port, timeout_s=request_timeout_s
        )
        self.camera_key_map = camera_key_map
        self.expected_image_shape = expected_image_shape
        # Safety: refuse a chunk whose first row jumps further than this (rad,
        # any joint) from the measured joints, e.g. a wrong state convention.
        self.max_first_step_joint_delta = max_first_step_joint_delta

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
        pyzlc.info(f"Connected to policy server: {health}")
        action_space = health.get("action_space")
        if action_space != JOINT_ACTION_SPACE:
            raise ValueError(
                f"Policy server action space is {action_space!r}, expected "
                f"{JOINT_ACTION_SPACE!r} (absolute joints + gripper closedness)."
            )
        server_fps = health.get("fps")
        if server_fps is not None and int(server_fps) != int(self.fps):
            pyzlc.warn(
                f"Server action rate is {server_fps} Hz but fps={self.fps}; "
                f"using the server's {server_fps} Hz."
            )
            self.fps = int(server_fps)
        self.exec_steps = int(health.get("exec_steps", 8))
        if abs(float(self.control_pair.control_hz) - float(self.fps)) > 1e-6:
            pyzlc.info(
                f"Resampling each {self.fps} Hz chunk to the control pair's "
                f"{self.control_pair.control_hz:g} Hz."
            )

        self.register_start_infering_event(self.control_pair.start_control_pair)
        self.register_stop_infering_event(self.control_pair.stop_control_pair)

    def _infer_step(self) -> None:
        start_time = time.perf_counter()
        observation = self._build_observation_payload()
        q_now = np.asarray(observation["observation.state"][:7], dtype=np.float64)

        response = self.client.infer(observation)
        chunk = self._extract_chunk(response)
        exec_steps = int(response.get("exec_steps", self.exec_steps))
        chunk = chunk[:exec_steps]

        jump = float(np.max(np.abs(chunk[0, :7] - q_now)))
        if jump > self.max_first_step_joint_delta:
            pyzlc.error(
                f"First action is {jump:.3f} rad away from the measured joints "
                f"(limit {self.max_first_step_joint_delta:.3f}); discarding episode."
            )
            self._state_machine.trigger(PolicyInferenceEvent.DISCARD)
            return

        self.control_pair.update_action_chunk(self._resample_chunk(chunk, q_now))
        infer_s = time.perf_counter() - start_time
        pyzlc.info(
            f"Remote action chunk ({exec_steps} @ {self.fps} Hz) in {infer_s:.3f}s: "
            f"q_end=[{', '.join(f'{x:.3f}' for x in chunk[-1, :7])}], "
            f"gripper={chunk[-1, 7]:.2f}"
        )

        # Let the control loop play the chunk before the next observation, so the
        # model sees the result of exactly the rows it fed into its memory.
        time.sleep(exec_steps / float(self.fps))

    def _resample_chunk(self, chunk: np.ndarray, q_now: np.ndarray) -> np.ndarray:
        """[T, 8] rows at self.fps -> rows at the control pair's rate.

        Joints are interpolated linearly from the measured joints (t = 0) through
        the chunk rows (t = (k + 1) / fps); the gripper is held per row."""
        control_hz = float(self.control_pair.control_hz)
        if abs(control_hz - float(self.fps)) < 1e-6:
            return chunk
        n = chunk.shape[0]
        t_src = np.arange(n + 1) / float(self.fps)
        q_src = np.vstack([q_now[None, :7], chunk[:, :7]])
        n_out = max(1, int(round(n * control_hz / float(self.fps))))
        # Spread the rows evenly over the chunk so the last one lands exactly on
        # the last target (n_out / control_hz alone may fall short of n / fps).
        t_out = np.arange(1, n_out + 1) * (t_src[-1] / n_out)
        out = np.empty((n_out, chunk.shape[1]), dtype=np.float64)
        for j in range(7):
            out[:, j] = np.interp(t_out, t_src, q_src[:, j])
        row = np.clip(np.ceil(t_out * self.fps - 1e-9).astype(int) - 1, 0, n - 1)
        out[:, 7] = chunk[row, 7]
        return out

    def _extract_chunk(self, response: Dict[str, Any]) -> np.ndarray:
        if response.get("action_space", JOINT_ACTION_SPACE) != JOINT_ACTION_SPACE:
            raise ValueError(f"Unexpected action space {response.get('action_space')!r}.")
        chunk = np.asarray(response["action"], dtype=np.float64)
        if chunk.ndim == 3:
            if chunk.shape[0] != 1:
                raise ValueError(f"Expected action batch size 1, got shape {chunk.shape}.")
            chunk = chunk[0]
        if chunk.ndim == 1:
            chunk = chunk[None]
        if chunk.ndim != 2 or chunk.shape[1] != 8:
            raise ValueError(f"Expected action chunk [T, 8], got shape {chunk.shape}.")
        if not np.all(np.isfinite(chunk)):
            raise ValueError("Policy returned non-finite actions.")
        chunk[:, 7] = np.clip(chunk[:, 7], 0.0, 1.0)
        return chunk

    def _build_observation_payload(self) -> Dict[str, Any]:
        arm_state = self.arm_wrapper.capture_step()
        grip_state = self.gripper_wrapper.capture_step()

        q = arm_state.get("q") if isinstance(arm_state, dict) else None
        if q is None:
            raise ValueError("Arm state missing q.")
        q = np.asarray(q, dtype=np.float32).reshape(-1)
        if q.size != 7:
            raise ValueError(f"Expected joint state size 7, got {q.size}.")
        state = np.concatenate(
            [q, np.asarray([self._get_gripper_closedness(grip_state)], dtype=np.float32)]
        )

        observation: Dict[str, Any] = {
            "observation.state": state,
            "task": self.task,
        }
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
            observation[self._camera_observation_key(cam.hw_name)] = np.ascontiguousarray(frame)
        return observation

    def _get_gripper_closedness(self, grip_state: Any) -> float:
        """0 = open, 1 = closed (Robotiq position; Franka hand 1 - width/max_width)."""
        if not isinstance(grip_state, dict):
            raise ValueError("Gripper state must be a dict.")
        if "position" in grip_state:
            return float(np.clip(float(grip_state["position"]), 0.0, 1.0))
        if "width" in grip_state:
            max_width = float(grip_state.get("max_width", 0.0) or 0.0)
            if max_width <= 0.0:
                raise ValueError("Panda gripper state has no max_width.")
            return float(np.clip(1.0 - float(grip_state["width"]) / max_width, 0.0, 1.0))
        raise ValueError("Gripper state missing position/width value.")

    def _camera_observation_key(self, hw_name: str) -> str:
        if self.camera_key_map is None:
            return f"observation.images.{hw_name}"
        if hw_name not in self.camera_key_map:
            raise ValueError(
                f"Camera '{hw_name}' has no entry in camera_key_map "
                f"{sorted(self.camera_key_map)}."
            )
        return self.camera_key_map[hw_name]

    def _start_infering(self) -> None:
        self.client.reset()
        self.control_pair.reset_action()
        super()._start_infering()

    def _save_episode(self) -> None:
        self._stop_infering()
        self._ui_console.log("Episode stopped.")

    def _discard_infering(self) -> None:
        super()._discard_infering()

    def _stop_infering(self) -> None:
        super()._stop_infering()

    def _reset_arm(self) -> None:
        self.control_pair.go_home()

    def _close(self) -> None:
        self.client.close()
        super()._close()
