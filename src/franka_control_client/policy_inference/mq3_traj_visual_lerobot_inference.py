from typing import List, Optional
import time
import torch
import pyzlc
import numpy as np
import threading

from digital_twin.models import RobotModelId
from digital_twin.simulation.mirror import RobotMirror
from simpub.core import XRTrajectory

from franka_control_client.control_pair.cartesian_policy_panda_control_pair import (
    CartesianPolicyPandaControlPair,
)

from ..control_pair.pil_panda_control_pair import PILPandaControlPair

from ..data_collection.irl_wrapper import IRLDataWrapper

from .lerobot_policy_inference import (
    LeRobotPolicyInference,
    LeRobotPolicyInferenceConfig,
)


class MQ3TrajVisualLeRobotInference(LeRobotPolicyInference):
    def __init__(
        self,
        data_collectors: List[IRLDataWrapper],
        control_pair: PILPandaControlPair,
        cfg: LeRobotPolicyInferenceConfig,
    ) -> None:
        super().__init__(data_collectors, control_pair, cfg)
        self.mirror = RobotMirror.from_model_id(
            RobotModelId.FRANKA_PANDA_ROBOTIQ
        )
        self.last_chunk_traj: Optional[XRTrajectory] = None
        self.history_way_points = []
        self.history_traj: Optional[XRTrajectory] = None
        self.reset_history_event = threading.Event()
        self.running = True
        self.visualize_thread = threading.Thread(
            target=self._visualize_loop, daemon=True
        )
        self.visualize_thread.start()

    def _visualize_loop(self) -> None:
        while self.running:
            if self.arm_wrapper is None:
                pyzlc.sleep(0.1)
                continue
            arm_state = self.arm_wrapper.arm.current_state
            if arm_state is not None:
                self.mirror.apply_arm_state(np.array(arm_state["q"]))
            if hasattr(
                self.control_pair.current_control_pair, "get_lastest_command"
            ):
                lastest_action = (
                    self.control_pair.current_control_pair.get_lastest_command()
                )
                if self.reset_history_event.is_set():
                    self.history_way_points = []
                    self.history_traj = None
                    self.reset_history_event.clear()
                if lastest_action is not None:
                    self.history_way_points.append(
                        {
                            "pos": lastest_action[:3].tolist(),
                            "color": [1.0, 0.0, 0.0, 1.0],
                        }
                    )
                    if self.history_traj is None:
                        self.history_traj = (
                            self.mirror._cavns.create_trajectory(
                                name="history_traj",
                                waypoints=self.history_way_points,
                            )
                        )
                    else:
                        self.history_traj.update(
                            waypoints=self.history_way_points
                        )
            # elif hasattr(self.control_pair.current_control_pair, "leader"):
            else:
                # print("Using leader control signal for visualization")
                control_signal = (
                    self.control_pair.current_control_pair.leader.current_control_signal
                )
                if control_signal is None:
                    continue
                # print(f"Current control signal: {control_signal}")
                self.history_way_points.append(
                    {
                        "pos": control_signal["pos"],
                        "color": [0.0, 0.0, 1.0, 1.0],
                    }
                )
                if self.history_traj is None:
                    self.history_traj = self.mirror._cavns.create_trajectory(
                        name="history_traj", waypoints=self.history_way_points
                    )
                else:
                    self.history_traj.update(waypoints=self.history_way_points)
            time.sleep(0.05)

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
            action_chunk = self.policy.predict_action_chunk(observation)

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
                    "color": [0.0, 1.0, 0.0, 1.0],
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
        if self.visualize_thread.is_alive():
            self.visualize_thread.join(timeout=1.0)
        return super()._close()

    def _reset_arm(self):
        self.reset_history_event.set()
        self.control_pair.clear_lastest_command()
        return super()._reset_arm()
