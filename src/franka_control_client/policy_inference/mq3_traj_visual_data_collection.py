import traceback
from typing import List, Optional
import time
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
    ) -> None:
        super().__init__(data_collectors, control_pair, cfg)
        self.control_pair: PILPandaControlPair = control_pair
        self.mirror = mirror if mirror is not None else RobotMirror.from_model_id(
            RobotModelId.FRANKA_PANDA_ROBOTIQ
        )
        self.last_chunk_traj: Optional[XRTrajectory] = None
        self.history_way_points = []
        self.history_traj: Optional[XRTrajectory] = None
        self.reset_history_event = threading.Event()
        self.running = True

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

    def _collect_step(self) -> None:
        # only collect data when in interrupt mode
        if self.control_pair.current_state == PILMode.INTERRUPT:
            self._data_colection._collect_step(
                self.control_pair.get_lastest_command()
            )
        elif self.control_pair.current_state == PILMode.REPLAY or \
            self.control_pair.current_state == PILMode.POLICY:
            return

    def _visualize_step(self) -> None:
        if self.reset_history_event.is_set():
            self.history_traj = None
            self.history_way_points = []
            self.reset_history_event.clear()
            return
        arm_state = self.arm_wrapper.arm.current_state
        if arm_state is not None:
            self.mirror.apply_arm_state(np.array(arm_state["q"]))
        if self.control_pair.current_state == PILMode.POLICY:
            color = [0.0, 0.0, 1.0, 1.0]
        elif self.control_pair.current_state == PILMode.INTERRUPT:
            color = [0.0, 1.0, 0.0, 1.0]
        else:
            self.history_way_points = []
            with self._data_colection.data_lock:
                leader_data = self._data_colection.leader_robot_data
                for pos, source in zip(leader_data.EE_pos, leader_data.source):
                    self.history_way_points.append(
                        {
                            "pos": pos.tolist(),
                            "color": (
                                [0.0, 0.0, 1.0, 1.0]
                                if float(source) > 0.5
                                else [0.0, 1.0, 0.0, 1.0]
                            ),
                        }
                    )
            if len(self.history_way_points) != 0:
                self.history_traj.update(waypoints=self.history_way_points)
            return
        if not hasattr(self._data_colection, "leader_robot_data"):
            return
        lastest_action = self._data_colection.leader_robot_data
        if lastest_action is not None and len(lastest_action.EE_pos) != 0:
            self.history_way_points.append(
                {
                    "pos": lastest_action.EE_pos[-1].tolist(),
                    "color": color,
                }
            )
            if self.history_traj is None:
                self.history_traj = self.mirror._cavns.create_trajectory(
                    name="history_traj", waypoints=self.history_way_points
                )
            else:
                self.history_traj.update(waypoints=self.history_way_points)

    def _infer_step(self) -> None:
        # if self.last_timestamp is None:
        #     self.last_timestamp = time.perf_counter()
        start_time = time.perf_counter()
        # Build observation from hardware
        observation = self._build_observation()

        try:
            # Preprocess observation
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

        # Evaluate policy and postprocess each action in the predicted chunk.
        with torch.inference_mode():
            ###single action
            #       action = self.policy.select_action(observation)
            # action = action[:, :8]

            # # Postprocess action
            # action = self.postprocessor(action).float().cpu().numpy()
            # # print("post action:",action)

            # action_vec = action[0] if action.ndim == 2 else action

            ###action chunk
            print(f"type of policy: {type(self.policy)}, observation keys: {list(observation.keys())}")
            action_chunk = self.policy.predict_action_chunk(batch = observation)

        if action_chunk.ndim == 2:
            action_chunk = action_chunk.unsqueeze(1)
        elif action_chunk.ndim != 3:
            raise RuntimeError(
                f"Expected action_chunk to have shape (B, T, D) or (B, D), got {tuple(action_chunk.shape)}"
            )

        batch_size, chunk_size, action_dim = action_chunk.shape
        action_dim_expected = 8  # 7 joints + 1 gripper
        post_action_chunk = torch.zeros(
            (batch_size, chunk_size, action_dim_expected), dtype=torch.float32
        )
        for chunk_idx in range(chunk_size):
            single_action = action_chunk[:, chunk_idx, :]
            single_action = single_action[:, :8]
            processed_action = self.postprocessor(single_action)
            # pyzlc.info(f"Processed action chunk {chunk_idx}: {processed_action.float().cpu().numpy()}")
            post_action_chunk[:, chunk_idx, :] = processed_action

        post_action_chunk = post_action_chunk.float().cpu().numpy()
        way_points = []
        for idx in range(len(post_action_chunk[0])):
            # pyzlc.info(f"Postprocessed action chunk for batch {idx}: {post_action_chunk[0][idx]}")
            way_points.append(
                {
                    "pos": post_action_chunk[0][idx][:3].tolist(),
                    "color": [1.0, 0.0, 0.0, 1.0],
                }
            )
        if self.last_chunk_traj is None:
            self.last_chunk_traj = self.mirror._cavns.create_trajectory(
                name="ee_trajectory", waypoints=way_points
            )
        else:
            self.last_chunk_traj.update(waypoints=way_points)
        try:
            # single_action
            # self.control_pair.update_action(action_vec)
            # action chunk
            self.control_pair.update_action_chunk(post_action_chunk)
        except Exception as exc:
            pyzlc.error(f"Failed to apply policy action: {exc}")
        end_time = time.perf_counter()
        elapsed = end_time - start_time
        # print(f"Inference step took {elapsed:.4f} seconds.")

        sleep_time = max(0.0, (1.0 / self.fps) - elapsed)
        if sleep_time > 0.001:
            time.sleep(sleep_time)
            # print(f"Inference step took {elapsed:.5f} seconds, slept for {sleep_time:.5f} seconds to maintain {self.fps} FPS.")

    def _close(self):
        self.running = False
        return super()._close()

    def _reset_arm(self):
        self.control_pair.clear_lastest_command()
        self.reset_history_event.set()
        return super()._reset_arm()

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
                        # curr_time = time.perf_counter()
                        self._infer_step()
                        self._collect_step()
                        # self._visualize_step()
                        # end_time = time.perf_counter()
                        # elapsed = end_time - curr_time
                        # print(f"Inference step took {elapsed:.3f} seconds")
                    if (
                        self._state_machine.state
                        == PolicyInferenceState.STOPPED
                    ):
                        self._reset_to_waiting()
                    # time.sleep(0.001)
        finally:
            traceback.print_exc()
            self._close()

    def _reset_to_waiting(self) -> None:
        self._data_colection._reset_to_waiting()
        return super()._reset_to_waiting()

    def _discard_infering(self) -> None:
        self._data_colection._discard_collecting()
        return super()._discard_infering()

    def _save_episode(self) -> None:
        self._data_colection._save_episode()
        return super()._save_episode()

    def _start_infering(self):
        self._data_colection._start_collecting()
        return super()._start_infering()
