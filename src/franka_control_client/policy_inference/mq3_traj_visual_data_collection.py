import traceback
from collections import deque
from typing import List, Optional, Sequence
import time
import sys
from anyio import Path
import torch
import pyzlc
import numpy as np
import threading

from digital_twin.models import RobotModelId
from digital_twin.simulation.mirror import RobotMirror
from simpub.core import XRTrajectory

from ..control_pair.pil_panda_control_pair import PILMode, PILPandaControlPair
from ..data_collection.utils import NonBlockingKeyPress
from .policy_inference_manager import PolicyInferenceState

from ..data_collection.irl_wrapper import IRLDataWrapper

from ..data_collection.pil_irl_vr_data_collection import PILIRLDataCollection

from .lerobot_policy_inference import (
    LeRobotPolicyInference,
    LeRobotPolicyInferenceConfig,
)
from .finger_waypoints import (
    align_gripper_trajectory_start,
    build_finger_waypoint_positions,
)
from ..data_collection.data_collection_manager import DataCollectionState


class MQ3TrajVisualDataCollectionInference(LeRobotPolicyInference):
    def __init__(
        self,
        data_collectors: List[IRLDataWrapper],
        control_pair: PILPandaControlPair,
        task: str,
        cfg: LeRobotPolicyInferenceConfig,
        save_path: str = None,
        mirror: RobotMirror = None,
        visualize_history: bool = False,
        visualization_hz: float = 30.0,
        action_buffer_refill_threshold: int = 0,
    ) -> None:
        super().__init__(data_collectors, control_pair, cfg)
        self.control_pair: PILPandaControlPair = control_pair
        self.mirror = mirror if mirror is not None else RobotMirror.from_model_id(
            RobotModelId.FRANKA_PANDA_ROBOTIQ
        )
        self.history_way_points = []
        self.history_traj: Optional[XRTrajectory] = None
        self.reset_history_event = threading.Event()
        self.visualize_history = visualize_history
        self.visualization_hz = float(visualization_hz)
        self.action_buffer_refill_threshold = max(
            0, int(action_buffer_refill_threshold)
        )
        self._action_buffer: deque[np.ndarray] = deque()
        self._finger_waypoint_sphere_names: dict[str, list[str]] = {}
        self._mirror_lock = threading.Lock()
        self.running = True
        self._closed = False

        # Chunk-pair recording state. `_active_policy_chunk` is the most
        # recently inferred policy chunk that the policy buffer is draining
        # from; `_chunk_size` is its length. During an interrupt window we
        # track which policy chunk was paired with the current correction
        # window, how many leader corrections we've collected, and the
        # offset `k` where corrections begin inside the chunk (only > 0
        # for the first partial window after trigger press).
        #
        # `_latest_viz_policy_chunk` is the chunk the user is currently
        # being shown — re-inferred every interrupt step, used both as
        # the finger-waypoint visualization source and as the payload
        # that fills the leftover correction slots on release (and that
        # the policy then executes when control returns to it).
        self._chunk_size: Optional[int] = None
        self._active_policy_chunk: Optional[np.ndarray] = None
        self._correction_window_policy_chunk: Optional[np.ndarray] = None
        self._correction_window_buffer: list[np.ndarray] = []
        self._correction_window_k: int = 0
        self._latest_viz_policy_chunk: Optional[np.ndarray] = None
        self._prev_pil_state: PILMode = PILMode.POLICY
        self._visualization_thread = threading.Thread(
            target=self._visualization_loop,
            daemon=True,
            name="robot-mirror-visualization",
        )

        self._data_colection: PILIRLDataCollection = PILIRLDataCollection(
            data_collectors,
            str(Path(save_path)/task) if save_path else None,
            task,
            fps=40,
            control_pair=control_pair,
        )
        # self.data_collection_thread = threading.Thread(target=self.run_data_collection, daemon=True)
        # self.data_collection_thread.start()
        self.control_pair.register_history(self._data_colection)
        self._visualization_thread.start()
        pyzlc.info(
            f"Started robot mirror visualization thread at {self.visualization_hz:.1f} Hz."
        )

    def _update_mirror_arm_state(self) -> None:
        arm_state = self.arm_wrapper.arm.current_state
        if arm_state is not None:
            with self._mirror_lock:
                self.mirror.apply_arm_state(np.array(arm_state["q"]))

    def _visualization_loop(self) -> None:
        period = 1.0 / self.visualization_hz if self.visualization_hz > 0 else 0.0
        while self.running:
            start_time = time.perf_counter()
            try:
                self._update_mirror_arm_state()
            except Exception as exc:
                pyzlc.error(f"Robot mirror visualization update failed: {exc}")
            if period <= 0:
                continue
            elapsed = time.perf_counter() - start_time
            sleep_time = max(0.0, period - elapsed)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    def _check_pil_state_transition(self) -> None:
        """Run interrupt-start / interrupt-end hooks on the tick where the
        PIL mode flips. Called both at the top of `_infer_step` (so that
        `_on_interrupt_end` clears+loads the buffer *before* we pop from
        it) and at the top of `_collect_step` (as a safety net for state
        changes that race between the two halves of the loop iteration).
        """
        current_state = self.control_pair.current_state
        if current_state == self._prev_pil_state:
            return
        if current_state == PILMode.INTERRUPT:
            self._on_interrupt_start()
        elif self._prev_pil_state == PILMode.INTERRUPT:
            self._on_interrupt_end()
        self._prev_pil_state = current_state

    def _collect_step(self) -> None:
        self._check_pil_state_transition()
        current_state = self.control_pair.current_state

        if current_state == PILMode.INTERRUPT:
            self._data_colection._collect_step(command_source=1.0)
            # Re-infer at every interrupt step so the user sees a live
            # policy projection on the finger waypoints; the same chunk
            # also serves as the storage source on release.
            self._refresh_viz_policy_chunk()
            self._record_correction_sample()
        elif current_state == PILMode.POLICY:
            # During policy control, we can also collect data but mark it differently
            self._data_colection._collect_step(
                self.control_pair.get_lastest_command(),
                command_source=0.0,
            )
        elif current_state == PILMode.REPLAY:
            return

    def _refresh_viz_policy_chunk(self) -> None:
        """Run one fresh policy inference and update the finger-waypoint
        visualization. Called on every leader-sample tick (~40 Hz) during
        interrupt. The result is also the snapshot used by storage at
        window boundaries and by release to fill the correction tail.
        """
        try:
            chunk = self._infer_fresh_policy_chunk()
        except Exception as exc:
            pyzlc.error(f"Per-step viz inference failed: {exc}")
            return
        self._latest_viz_policy_chunk = chunk
        if self._chunk_size is None:
            self._chunk_size = int(chunk.shape[0])
        self._update_finger_waypoint_visualization(chunk)

    def _on_interrupt_start(self) -> None:
        """Begin the first correction window after the trigger is pressed."""
        if self._active_policy_chunk is None or self._chunk_size is None:
            # No chunk has been inferred yet this episode — produce one
            # immediately and treat k=0 (correction will fill all slots).
            self._active_policy_chunk = self._infer_fresh_policy_chunk()
            self._chunk_size = int(self._active_policy_chunk.shape[0])

        self._correction_window_policy_chunk = np.array(
            self._active_policy_chunk, copy=True
        )
        # Seed the live viz with the active chunk; it will be refreshed on
        # the very next `_collect_step` tick.
        self._latest_viz_policy_chunk = np.array(
            self._active_policy_chunk, copy=True
        )
        consumed = self._chunk_size - len(self._action_buffer)
        self._correction_window_k = int(max(0, min(consumed, self._chunk_size)))
        self._correction_window_buffer = []

    def _on_interrupt_end(self) -> None:
        """Finalize the in-flight correction window on trigger release.

        The last visualization chunk the user was looking at becomes the
        next executed trajectory — it's loaded straight into the action
        buffer (no extra inference latency on resume) and its leading
        slots also fill the tail of the just-finalized correction chunk
        via `_finalize_correction_window`.
        """
        if self._correction_window_policy_chunk is not None:
            self._finalize_correction_window()
        self._correction_window_policy_chunk = None
        self._correction_window_buffer = []
        self._correction_window_k = 0

        self._action_buffer.clear()
        if (
            self._latest_viz_policy_chunk is not None
            and self._chunk_size is not None
        ):
            for action in self._latest_viz_policy_chunk:
                self._action_buffer.append(np.array(action, copy=True))
            self._active_policy_chunk = np.array(
                self._latest_viz_policy_chunk, copy=True
            )

    def _record_correction_sample(self) -> None:
        """Capture one leader correction action into the current window."""
        if (
            self._correction_window_policy_chunk is None
            or self._chunk_size is None
        ):
            return

        leader_action = self._get_last_leader_action()
        if leader_action is None:
            return

        self._correction_window_buffer.append(leader_action)

        slots_filled = self._correction_window_k + len(
            self._correction_window_buffer
        )
        if slots_filled >= self._chunk_size:
            self._finalize_correction_window()
            # Start a fresh window. Reuse the most recent viz chunk
            # (re-inferred in `_refresh_viz_policy_chunk` on this same
            # tick) so storage and visualization stay in sync without an
            # extra inference call.
            if self._latest_viz_policy_chunk is not None:
                self._correction_window_policy_chunk = np.array(
                    self._latest_viz_policy_chunk, copy=True
                )
            else:
                self._correction_window_policy_chunk = (
                    self._infer_fresh_policy_chunk()
                )
            self._correction_window_k = 0
            self._correction_window_buffer = []

    def _finalize_correction_window(self) -> None:
        """Emit the (policy_chunk, correction_chunk, slot_source) tuple.

        Layout of the correction chunk:
          - slots `[0:k]`            = window-start policy_chunk[0:k]
            (executed by policy before interrupt — only k>0 for the
            first partial window after trigger press)
          - slots `[k:k+j]`          = collected leader corrections
          - slots `[k+j:chunk_size]` = latest viz chunk[0:chunk_size-k-j]
            (only happens when the user releases mid-window; the same
            chunk is loaded into the action buffer for execution by
            `_on_interrupt_end`)

        Per-slot source: 1.0 for the leader sample slots `[k:k+j]`, 0.0
        for both policy-padding regions.
        """
        policy_chunk = self._correction_window_policy_chunk
        if policy_chunk is None or self._chunk_size is None:
            return
        chunk_size = self._chunk_size
        k = self._correction_window_k
        buffer = self._correction_window_buffer
        j = len(buffer)

        correction_chunk = np.zeros_like(policy_chunk)
        if k > 0:
            correction_chunk[0:k] = policy_chunk[0:k]
        if j > 0:
            correction_chunk[k : k + j] = np.stack(buffer, axis=0)
        if k + j < chunk_size:
            n_remaining = chunk_size - k - j
            viz_chunk = self._latest_viz_policy_chunk
            if viz_chunk is not None:
                correction_chunk[k + j : chunk_size] = viz_chunk[0:n_remaining]
            else:
                correction_chunk[k + j : chunk_size] = policy_chunk[
                    k + j : chunk_size
                ]

        slot_source = np.zeros(chunk_size, dtype=np.float32)
        if j > 0:
            slot_source[k : k + j] = 1.0

        self._data_colection.emit_chunk_pair(
            policy_chunk=policy_chunk,
            correction_chunk=correction_chunk,
            source=slot_source,
        )

    def _get_last_leader_action(self) -> Optional[np.ndarray]:
        """Read the leader EE pose just appended by `_data_colection._collect_step`."""
        data = self._data_colection
        if data is None:
            return None
        with data.data_lock:
            leader = data.leader_robot_data
            if (
                not leader.EE_pos
                or not leader.EE_quat
                or not leader.gripper_width_list
            ):
                return None
            pos = leader.EE_pos[-1].detach().cpu().numpy().reshape(-1)
            quat = leader.EE_quat[-1].detach().cpu().numpy().reshape(-1)
            gripper = float(leader.gripper_width_list[-1].detach().cpu().numpy())
        return np.concatenate([pos, quat, [gripper]]).astype(np.float64)

    def _visualize_step(self) -> None:
        if self.reset_history_event.is_set():
            self.history_traj = None
            self.history_way_points = []
            self.reset_history_event.clear()
            return
        self._update_mirror_arm_state()
        if not self.visualize_history:
            return
        if not hasattr(self._data_colection, "command_state_data"):
            return
        if self.control_pair.current_state == PILMode.POLICY:
            color = [0.0, 0.0, 1.0, 1.0]
        elif self.control_pair.current_state == PILMode.INTERRUPT:
            color = [0.0, 1.0, 0.0, 1.0]
        else:
            self.history_way_points = []
            with self._data_colection.data_lock:
                command_data = self._data_colection.command_state_data
                for pos, source in zip(command_data.EE_pos, command_data.source):
                    self.history_way_points.append(
                        {
                            "pos": pos.tolist(),
                            "color": (
                                [0.0, 0.0, 1.0, 1.0]
                                if float(source) < 0.5
                                else [0.0, 1.0, 0.0, 1.0]
                            ),
                        }
                    )
            if len(self.history_way_points) != 0:
                with self._mirror_lock:
                    self.history_traj.update(waypoints=self.history_way_points)
            return
        lastest_action = self._data_colection.command_state_data
        if lastest_action is not None and len(lastest_action.EE_pos) != 0:
            self.history_way_points.append(
                {
                    "pos": lastest_action.EE_pos[-1].tolist(),
                    "color": color,
                }
            )
            if self.history_traj is None:
                with self._mirror_lock:
                    self.history_traj = self.mirror._cavns.create_trajectory(
                        name="history_traj", waypoints=self.history_way_points
                    )
            else:
                with self._mirror_lock:
                    self.history_traj.update(waypoints=self.history_way_points)

    def _infer_fresh_policy_chunk(self) -> np.ndarray:
        """Run policy inference once and return the postprocessed `(T, 8)` chunk.

        Does NOT touch the action buffer or the active-chunk tracking; use
        this for the contrastive policy chunk paired with a correction
        window during interrupt.
        """
        observation = self._build_observation()
        try:
            observation = self.preprocessor(observation)
        except Exception as exc:
            image_shapes = {
                k: tuple(v.shape)
                for k, v in observation.items()
                if str(k).startswith("observation.images.")
                and hasattr(v, "shape")
            }
            raise RuntimeError(
                f"Preprocessor failed. image_shapes={image_shapes}, state_shape={tuple(observation['observation.state'].shape)}"
            ) from exc

        with torch.inference_mode():
            action_chunk = self.policy.predict_action_chunk(batch=observation)

        post_action_chunk = self._postprocess_action_chunk(action_chunk)
        if post_action_chunk.shape[0] < 1:
            raise RuntimeError(
                f"Expected at least one batch in action chunk, got {post_action_chunk.shape}"
            )
        return np.array(post_action_chunk[0], copy=True)

    def _refill_action_buffer(self) -> None:
        chunk = self._infer_fresh_policy_chunk()
        print(f"Refilled action buffer with chunk shape: {chunk.shape}")

        self._action_buffer.clear()
        for action in chunk:
            self._action_buffer.append(np.array(action, copy=True))

        self._active_policy_chunk = chunk
        self._chunk_size = int(chunk.shape[0])

        # Pure policy rollout: pair this fresh policy chunk with a
        # zero-padded correction chunk. During interrupt the natural
        # inference cadence still drains the buffer, but we suppress
        # emission here because the correction window owns chunk-pair
        # bookkeeping.
        if self.control_pair.current_state == PILMode.POLICY:
            zeros = np.zeros_like(chunk)
            slot_source = np.zeros(self._chunk_size, dtype=np.float32)
            self._data_colection.emit_chunk_pair(
                policy_chunk=chunk,
                correction_chunk=zeros,
                source=slot_source,
            )

    def _postprocess_action_chunk(self, action_chunk: torch.Tensor) -> np.ndarray:
        if action_chunk.ndim == 2:
            action_chunk = action_chunk.unsqueeze(1)
        elif action_chunk.ndim != 3:
            raise RuntimeError(
                f"Expected action_chunk to have shape (B, T, D) or (B, D), got {tuple(action_chunk.shape)}"
            )

        _, chunk_size, _ = action_chunk.shape
        if chunk_size < 1:
            raise RuntimeError(
                f"Action chunk must contain at least one action, got {tuple(action_chunk.shape)}"
            )

        processed_actions = []
        for chunk_idx in range(chunk_size):
            single_action = action_chunk[:, chunk_idx, :]
            single_action = single_action[:, :8]
            processed_action = self.postprocessor(single_action)
            if processed_action.ndim == 1:
                processed_action = processed_action.unsqueeze(0)
            processed_actions.append(processed_action[:, :8])

        return torch.stack(processed_actions, dim=1).float().cpu().numpy()

    def _get_buffered_action_chunk(self) -> np.ndarray:
        if not self._action_buffer:
            return np.empty((0, 8), dtype=np.float32)
        return np.array(list(self._action_buffer), dtype=np.float32, copy=True)

    def _update_finger_waypoint_visualization(
        self, action_chunk: np.ndarray
    ) -> None:
        visual_action_chunk = np.array(action_chunk, copy=True)
        arm_state = self.arm_wrapper.arm.current_state
        if (
            arm_state is not None
            and "EE_pos" in arm_state
            and len(visual_action_chunk) > 0
        ):
            current_ee_pos = np.asarray(
                arm_state["EE_pos"], dtype=np.float32
            ).reshape(-1)
            if current_ee_pos.size >= 3:
                visual_action_chunk[:, :3] += (
                    current_ee_pos[:3] - visual_action_chunk[0, :3]
                )
        current_gripper = self._get_current_gripper_command()
        if current_gripper is not None and len(visual_action_chunk) > 0:
            visual_action_chunk = align_gripper_trajectory_start(
                visual_action_chunk, current_gripper
            )
        left_finger_positions, right_finger_positions = (
            build_finger_waypoint_positions(visual_action_chunk)
        )
        self._update_waypoint_sphere_group(
            left_finger_positions,
            name_prefix="left_finger_waypoint_sphere",
            start_color=(0.0, 0.9, 1.0, 1.0),
            end_color=(0.0, 0.2, 1.0, 1.0),
        )
        self._update_waypoint_sphere_group(
            right_finger_positions,
            name_prefix="right_finger_waypoint_sphere",
            start_color=(1.0, 0.7, 0.0, 1.0),
            end_color=(1.0, 0.0, 0.0, 1.0),
        )

    def _infer_step(self) -> None:
        # Handle a pending interrupt-end *before* we look at the buffer —
        # otherwise we'd pop the action that was sitting there from
        # before interrupt and push it to the arm one tick later, causing
        # a ~200 ms snap to a pose that ignores the human's corrections.
        self._check_pil_state_transition()

        # During interrupt the human is driving — pause the policy buffer
        # so we don't double-infer (correction windows own their own fresh
        # inference) and so leftover stale actions don't snap the arm back
        # to a pre-correction pose when the trigger releases.
        if self.control_pair.current_state == PILMode.INTERRUPT:
            return

        if len(self._action_buffer) <= self.action_buffer_refill_threshold:
            self._refill_action_buffer()

        visual_action_chunk = self._get_buffered_action_chunk()
        self._update_finger_waypoint_visualization(visual_action_chunk)

        if not self._action_buffer:
            raise RuntimeError("Policy action buffer is empty after refill.")
        action_vec = self._action_buffer.popleft()
        try:
            self.control_pair.update_action(action_vec)
        except Exception as exc:
            pyzlc.error(f"Failed to apply policy action: {exc}")

    def _sleep_for_policy_fps(self, loop_start_time: float) -> None:
        if self.control_pair.current_state != PILMode.POLICY:
            return
        sleep_time = max(
            0.0, (1.0 / self.fps) - (time.perf_counter() - loop_start_time)
        )
        if sleep_time > 0.001:
            time.sleep(sleep_time)

    def _close(self):
        if self._closed:
            return
        self._closed = True
        self.running = False
        try:
            if (
                self._visualization_thread.is_alive()
                and threading.current_thread() is not self._visualization_thread
            ):
                self._visualization_thread.join(timeout=1.0)
            self.control_pair.stop_control_pair()
        finally:
            self._data_colection._close()
        return super()._close()

    def _reset_arm(self):
        self.control_pair.reset_action()
        self._action_buffer.clear()
        self._reset_chunk_pair_state()
        self.reset_history_event.set()
        self._clear_finger_waypoint_spheres()
        return super()._reset_arm()

    def _reset_chunk_pair_state(self) -> None:
        """Drop any in-flight chunk-pair bookkeeping between episodes."""
        self._active_policy_chunk = None
        self._chunk_size = None
        self._correction_window_policy_chunk = None
        self._correction_window_buffer = []
        self._correction_window_k = 0
        self._latest_viz_policy_chunk = None
        self._prev_pil_state = self.control_pair.current_state

    def _get_current_gripper_command(self) -> Optional[float]:
        if self.gripper_wrapper is None:
            return None
        try:
            grip_state = self.gripper_wrapper.capture_step()
        except Exception as exc:
            pyzlc.error(f"Failed to read gripper state for visualization: {exc}")
            return None

        if not isinstance(grip_state, dict):
            return None
        if "position" in grip_state:
            return float(np.clip(grip_state["position"], 0.0, 1.0))
        if "commanded_position" in grip_state:
            return float(np.clip(grip_state["commanded_position"], 0.0, 1.0))
        if "gripper" in grip_state:
            gripper = np.asarray(grip_state["gripper"], dtype=np.float32).reshape(-1)
            if gripper.size > 0:
                return float(np.clip(gripper[0], 0.0, 1.0))
        if "width" in grip_state:
            width = np.asarray(grip_state["width"], dtype=np.float32).reshape(-1)
            if width.size > 0:
                return float(np.clip(1.0 - width[0] / 0.08, 0.0, 1.0))
        return None

    def _update_waypoint_sphere_group(
        self,
        positions: np.ndarray,
        *,
        name_prefix: str,
        radius: float = 0.005,
        start_color: Sequence[float] = (0.0, 1.0, 0.0, 1.0),
        end_color: Sequence[float] = (1.0, 0.0, 0.0, 1.0),
    ) -> None:
        positions_array = np.asarray(positions, dtype=float)
        marker_names = self._finger_waypoint_sphere_names.setdefault(
            name_prefix, []
        )
        identity_quat = np.array([1.0, 0.0, 0.0, 0.0])

        if positions_array.size == 0:
            self._hide_waypoint_spheres(marker_names, identity_quat)
            return
        if positions_array.ndim != 2 or positions_array.shape[1] != 3:
            raise ValueError("positions must have shape (N, 3)")
        if not np.all(np.isfinite(positions_array)):
            raise ValueError("positions must contain finite numeric values")

        num_waypoints = positions_array.shape[0]
        with self._mirror_lock:
            for idx, pos in enumerate(positions_array):
                if idx >= len(marker_names):
                    marker_name = f"{name_prefix}_{idx}"
                    color = self.mirror._interpolate_rgba(
                        idx, num_waypoints, start_color, end_color
                    )
                    self.mirror._create_future_waypoint_sphere(
                        marker_name, pos, radius, color
                    )
                    marker_names.append(marker_name)
                marker_name = marker_names[idx]
                self.mirror._publisher.tracked_obj_trans[marker_name] = (
                    pos.copy(),
                    identity_quat,
                )

            self._hide_waypoint_spheres(
                marker_names[num_waypoints:], identity_quat, already_locked=True
            )

    def _clear_finger_waypoint_spheres(self) -> None:
        identity_quat = np.array([1.0, 0.0, 0.0, 0.0])
        with self._mirror_lock:
            for marker_names in self._finger_waypoint_sphere_names.values():
                self._hide_waypoint_spheres(
                    marker_names, identity_quat, already_locked=True
                )

    def _hide_waypoint_spheres(
        self,
        marker_names: Sequence[str],
        identity_quat: np.ndarray,
        already_locked: bool = False,
    ) -> None:
        def hide() -> None:
            for marker_name in marker_names:
                self.mirror._publisher.tracked_obj_trans[marker_name] = (
                    np.array([0.0, 0.0, -10.0]),
                    identity_quat,
                )

        if already_locked:
            hide()
        else:
            with self._mirror_lock:
                hide()

    def run(self) -> None:
        self._on_state_enter(self._state_machine.state)
        try:
            with NonBlockingKeyPress() as kp:
                while (
                    self._state_machine.state != PolicyInferenceState.EXITING
                ):
                    key = kp.get_data()
                    if key:
                        self._handle_keypress(key)
                    if (
                        self._state_machine.state
                        == PolicyInferenceState.INFERING
                    ):
                        loop_start_time = time.perf_counter()
                        self._infer_step()
                        self._collect_step()
                        self._visualize_step()
                        self._sleep_for_policy_fps(loop_start_time)
                    if (
                        self._state_machine.state
                        == PolicyInferenceState.STOPPED
                    ):
                        self._reset_to_waiting()
                    # time.sleep(0.001)
        finally:
            if sys.exc_info()[0] is not None:
                traceback.print_exc()
            self._close()

    def _reset_to_waiting(self) -> None:
        self._data_colection._reset_to_waiting()
        return super()._reset_to_waiting()

    def _discard_infering(self) -> None:
        self._data_colection._discard_collecting()
        result = super()._discard_infering()
        self.control_pair.reset_action()
        self._action_buffer.clear()
        self._reset_chunk_pair_state()
        return result

    def _save_episode(self) -> None:
        # Flush a partial correction window so its chunk pair makes it
        # into the saved file before the data collector serializes.
        if self._correction_window_policy_chunk is not None:
            self._finalize_correction_window()
            self._correction_window_policy_chunk = None
            self._correction_window_buffer = []
            self._correction_window_k = 0
        self._data_colection._save_episode()
        result = super()._save_episode()
        self.control_pair.reset_action()
        self._action_buffer.clear()
        self._reset_chunk_pair_state()
        return result

    def _start_infering(self):
        self.control_pair.reset_action()
        self._action_buffer.clear()
        self._reset_chunk_pair_state()
        self._data_colection._start_collecting()
        return super()._start_infering()
