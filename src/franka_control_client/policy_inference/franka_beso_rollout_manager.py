from __future__ import annotations

import queue
import time
from concurrent.futures import Future
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import pyzlc

from .irl_wrapper import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    PandaGripperDataWrapper,
)
from .utils import NonBlockingKeyPress, UIConsole, VoidEvent
from ..control_pair.rollout_single_franka_control_pair import (
    RolloutSingleFrankaControlPair,
)
from ..policy.policy import RemotePolicy

RECORD_DATA_ROOT = str(
    (Path(__file__).resolve().parents[3] / "data" / "beso_rollout_records").resolve()
)


class RolloutState(str, Enum):
    WAITING = "waiting"
    ROLLING = "rolling"
    STOPPED = "stopped"
    EXITING = "exiting"


class RolloutEvent(str, Enum):
    NEW_ROLLOUT = "new_rollout"
    END = "end"
    QUIT = "quit"


Transition = Tuple[RolloutState, Optional[Callable[[], None]]]
StateEventPair = Tuple[RolloutState, RolloutEvent]


class RolloutStateMachine:
    def __init__(
        self,
        initial_state: RolloutState,
        on_enter: Optional[Callable[[RolloutState], None]] = None,
    ) -> None:
        self._state = initial_state
        self._transitions: Dict[StateEventPair, Transition] = {}
        self._on_enter = on_enter

    @property
    def state(self) -> RolloutState:
        return self._state

    def register_transition(
        self,
        from_state: RolloutState,
        event: RolloutEvent,
        to_state: RolloutState,
        action: Optional[Callable[[], None]] = None,
    ) -> None:
        self._transitions[(from_state, event)] = (to_state, action)

    def trigger(self, event: RolloutEvent) -> bool:
        transition = self._transitions.get((self._state, event))
        if transition is None:
            return False

        next_state, action = transition
        if action is not None:
            action()
        self._state = next_state
        if self._on_enter is not None:
            self._on_enter(next_state)
        return True


@dataclass
class FrankaBesoRolloutConfig:
    policy_name: str
    task: str
    fps: int = 30
    obs_topic: Optional[str] = None
    action_topic: Optional[str] = None
    record_enable: bool = False


class FrankaBesoRolloutManager:
    def __init__(
        self,
        obs_sources: List[IRL_HardwareDataWrapper],
        control_pair: RolloutSingleFrankaControlPair,
        cfg: FrankaBesoRolloutConfig,
    ) -> None:
        self.obs_sources = obs_sources
        self.control_pair = control_pair
        self.cfg = cfg
        self.fps = int(cfg.fps)

        self.policy = RemotePolicy(
            cfg.policy_name, obs_topic=cfg.obs_topic, action_topic=cfg.action_topic
        )

        self._ui_console = UIConsole()
        self._start_rollout_event = VoidEvent()
        self._stop_rollout_event = VoidEvent()
        self._state_machine = RolloutStateMachine(
            initial_state=RolloutState.WAITING,
            on_enter=self._on_state_enter,
        )
        self._state_machine.register_transition(
            RolloutState.WAITING,
            RolloutEvent.NEW_ROLLOUT,
            RolloutState.ROLLING,
            action=self._start_rollout,
        )
        self._state_machine.register_transition(
            RolloutState.ROLLING,
            RolloutEvent.END,
            RolloutState.STOPPED,
            action=self._end_rollout,
        )
        self._state_machine.register_transition(
            RolloutState.STOPPED,
            RolloutEvent.QUIT,
            RolloutState.EXITING,
            action=self._close,
        )

        self.cameras: List[ImageDataWrapper] = []
        self.arm_wrapper: Optional[PandaArmDataWrapper] = None
        self.gripper_wrapper: Optional[PandaGripperDataWrapper] = None
        for hw in obs_sources:
            if isinstance(hw, ImageDataWrapper) or hw.hw_type == "camera":
                self.cameras.append(hw)  # type: ignore[arg-type]
            elif isinstance(hw, PandaArmDataWrapper) or hw.hw_type == "follower_arm":
                self.arm_wrapper = hw  # type: ignore[assignment]
            elif isinstance(hw, PandaGripperDataWrapper) or hw.hw_type == "follower_gripper":
                self.gripper_wrapper = hw

        if self.arm_wrapper is None:
            raise ValueError("Missing PandaArmDataWrapper for rollout.")
        if self.gripper_wrapper is None:
            raise ValueError("Missing PandaGripperDataWrapper for rollout.")

        self._need_policy_reset = False
        self._rollout_id: Optional[int] = None
        self._next_obs_seq = 0
        self._pending_obs_seq: Optional[int] = None
        self._pending_state_vec: Optional[np.ndarray] = None
        self._pending_image_arrays: Optional[Dict[str, np.ndarray]] = None
        self._closed = False

        self._record_enable = bool(cfg.record_enable)
        self._dataset = None
        self._save_queue: queue.Queue[Optional[Dict[str, Any]]] = queue.Queue()
        self._save_future: Optional[Future] = None
        self._record_run_dir: Optional[str] = None

    def register_start_rollout_event(self, handler: Callable[[], None]) -> None:
        self._start_rollout_event.subscribe(handler)

    def register_stop_rollout_event(self, handler: Callable[[], None]) -> None:
        self._stop_rollout_event.subscribe(handler)

    def run(self) -> None:
        self._on_state_enter(self._state_machine.state)
        try:
            with NonBlockingKeyPress() as kp:
                while self._state_machine.state != RolloutState.EXITING:
                    key = kp.get_data()
                    if key:
                        self._handle_keypress(key)
                    if self._state_machine.state == RolloutState.ROLLING:
                        try:
                            self._rollout_step()
                        except Exception as exc:
                            pyzlc.error(f"Error during rollout step: {exc}")
        finally:
            self._close()

    def _handle_keypress(self, key: str) -> None:
        if key == "n" and self._state_machine.state == RolloutState.WAITING:
            self._state_machine.trigger(RolloutEvent.NEW_ROLLOUT)
        elif key == "e" and self._state_machine.state == RolloutState.ROLLING:
            self._state_machine.trigger(RolloutEvent.END)
        elif key == "q" and self._state_machine.state == RolloutState.STOPPED:
            self._state_machine.trigger(RolloutEvent.QUIT)

    def _on_state_enter(self, state: RolloutState) -> None:
        if state == RolloutState.WAITING:
            self._ui_console.update_hint("Press 'n' to start rollout")
        elif state == RolloutState.ROLLING:
            self._ui_console.update_hint("Rolling... Press 'e' to end rollout")
        elif state == RolloutState.STOPPED:
            self._ui_console.update_hint("Rollout ended. Press 'q' to quit")
        elif state == RolloutState.EXITING:
            self._ui_console.update_hint("Exiting rollout")

    def _start_rollout(self) -> None:
        self._ui_console.update_hint("Starting rollout...")
        self._need_policy_reset = True
        self._rollout_id = time.time_ns()
        self._next_obs_seq = 0
        self._pending_obs_seq = None
        self._pending_state_vec = None
        self._pending_image_arrays = None
        self._start_recording_episode()
        self._start_rollout_event.emit()

    def _end_rollout(self) -> None:
        self._ui_console.update_hint("Stopping rollout...")
        self._stop_rollout_event.emit()
        self._stop_recording_episode(save_episode=True)
        self._ui_console.log("Rollout ended.")

    def _close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self.control_pair.is_running:
            self._stop_rollout_event.emit() 
        self._stop_recording_episode(save_episode=False)
        self._finalize_recorder()
        self._ui_console.update_hint("Rollout manager closed.")

    def _try_consume_pending_action(self) -> bool:
        if self._pending_obs_seq is None:
            return False

        action_msg = self.policy.current_action
        if action_msg is None:
            return False
        if action_msg["rollout_id"] != self._rollout_id:
            return False
        if int(action_msg["source_obs_seq"]) != self._pending_obs_seq:
            return False

        action = np.asarray(action_msg["action"], dtype=np.float64).reshape(-1)
        self.control_pair.update_action(action)
        if self._pending_state_vec is not None and self._pending_image_arrays is not None:
            self._record_step(
                self._pending_state_vec,
                self._pending_image_arrays,
                action,
            )

        self._pending_obs_seq = None
        self._pending_state_vec = None
        self._pending_image_arrays = None
        return True

    def _send_next_observation(self) -> None:
        rollout_id = self._rollout_id
        if rollout_id is None:
            raise RuntimeError("rollout_id is not initialized before sending observations.")

        state_vec = self._build_state_vector()
        image_arrays = self._capture_image_arrays()
        obs_seq = self._next_obs_seq
        obs = self._build_observation(
            rollout_id=rollout_id,
            obs_seq=obs_seq,
            obs_timestamp=time.time(),
            state_vec=state_vec,
            image_arrays=image_arrays,
            reset_policy=self._need_policy_reset,
        )
        self.policy.send_observation(obs)
        self._need_policy_reset = False
        self._pending_obs_seq = obs_seq
        self._pending_state_vec = state_vec
        self._pending_image_arrays = image_arrays
        self._next_obs_seq += 1

    def _rollout_step(self) -> None:
        cycle_start = time.perf_counter()

        self._try_consume_pending_action()
        if self._pending_obs_seq is None:
            self._send_next_observation()

        elapsed = time.perf_counter() - cycle_start
        sleep_time = max(0.0, (1.0 / self.fps) - elapsed)
        if sleep_time > 0.0:
            pyzlc.sleep(sleep_time)

    def _build_observation(
        self,
        rollout_id: int,
        obs_seq: int,
        obs_timestamp: float,
        state_vec: np.ndarray,
        image_arrays: Dict[str, np.ndarray],
        reset_policy: bool,
    ) -> Dict[str, Any]:
        return {
            "rollout_id": rollout_id,
            "obs_seq": obs_seq,
            "obs_timestamp": obs_timestamp,
            "state": state_vec.tolist(),
            "images": self._encode_images_for_policy(image_arrays),
            "task": self.cfg.task,
            "reset_policy": reset_policy,
        }

    def _build_state_vector(self) -> np.ndarray:
        arm_state = self.arm_wrapper.capture_step()
        q = np.asarray(arm_state["q"], dtype=np.float32).reshape(-1)

        grip_state = self.gripper_wrapper.capture_step()
        gripper_width = float(grip_state["width"])

        return np.concatenate([q, np.asarray([gripper_width], dtype=np.float32)])

    def _capture_image_arrays(self) -> Dict[str, np.ndarray]:
        images: Dict[str, np.ndarray] = {}
        for cam in self.cameras:
            frame = cam.capture_step()
            images[cam.hw_name] = frame
        return images

    def _encode_images_for_policy(self, image_arrays: Dict[str, np.ndarray]) -> Dict[str, Any]:
        images: Dict[str, Any] = {}
        for cam_name, frame in image_arrays.items():
            h, w, c = frame.shape
            images[cam_name] = {
                "height": int(h),
                "width": int(w),
                "channels": int(c),
                "rgb_data": frame.tobytes(),
            }
        return images

    def _build_record_features(self) -> Dict[str, Dict[str, Any]]:
        features: Dict[str, Dict[str, Any]] = {
            "observation.state": {"dtype": "float32", "shape": (8,)},
            "action": {"dtype": "float32", "shape": (8,)},
        }
        for cam in self.cameras:
            h, w = cam.camera_device.size
            features[f"observation.image.{cam.hw_name}"] = {
                "dtype": "video",
                "shape": (h, w, 3),
            }
        return features

    def _init_recorder_if_needed(self) -> None:
        if not self._record_enable or self._dataset is not None:
            return

        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        record_root = Path(RECORD_DATA_ROOT)
        record_root.mkdir(parents=True, exist_ok=True)

        record_dir = record_root / str(time.time_ns())
        features = self._build_record_features()

        self._dataset = LeRobotDataset.create(
            repo_id=str(record_dir),
            features=features,
            fps=self.fps,
        )
        self._dataset.meta.metadata_buffer_size = 1
        pyzlc.info(f"Recording dataset dir: {record_dir}")


    def _start_recording_episode(self) -> None:
        if not self._record_enable:
            return
        self._init_recorder_if_needed()
        if self._save_future is not None:
            return
        while not self._save_queue.empty():
            self._save_queue.get()
        self._save_future = pyzlc.submit_thread_pool_task(self._save_data_task)

    def _stop_recording_episode(self, save_episode: bool) -> None:
        if not self._record_enable or self._save_future is None:
            return
        self._save_queue.put(None)
        self._save_future.result()
        self._save_future = None
        if save_episode and self._dataset is not None:
            self._dataset.save_episode()

    def _finalize_recorder(self) -> None:
        if not self._record_enable or self._dataset is None:
            return
        self._dataset.finalize()

    def _save_data_task(self) -> None:
        while True:
            frame = self._save_queue.get()
            if frame is None:
                break
            self._dataset.add_frame(frame)

    def _build_record_frame(
        self,
        state_vec: np.ndarray,
        image_arrays: Dict[str, np.ndarray],
        action: np.ndarray,
    ) -> Dict[str, Any]:
        frame: Dict[str, Any] = {
            "observation.state": np.asarray(state_vec, dtype=np.float32).reshape(-1),
            "action": np.asarray(action, dtype=np.float32).reshape(-1),
            "task": self.cfg.task,
        }
        for cam_name, image in image_arrays.items():
            frame[f"observation.image.{cam_name}"] = image
        return frame

    def _record_step(
        self,
        state_vec: np.ndarray,
        image_arrays: Dict[str, np.ndarray],
        action: np.ndarray,
    ) -> None:
        if not self._record_enable or self._save_future is None:
            return
        if action.size != 8:
            return
        frame = self._build_record_frame(state_vec, image_arrays, action)
        self._save_queue.put(frame)
