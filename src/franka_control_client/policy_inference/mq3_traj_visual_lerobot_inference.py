from typing import List, Optional
import time
import torch
import pyzlc
import numpy as np
import threading

# from digital_twin.models import RobotModelId
# from digital_twin.simulation.mirror import RobotMirror
from simpub.core import XRTrajectory
from simpub.core.xrcavns import TrajectoryWaypointDict
from enum import Enum

from ..franka_robot.panda_robotiq import PandaRobotiq
from ..control_pair.cartesian_policy_panda_control_pair import (
    PolicyPandaRobotiqDeltaCartesianControlPair,
)
from ..data_collection.irl_wrapper import IRLDataWrapper
from .lerobot_policy_inference import (
    LeRobotPolicyInference,
    LeRobotPolicyInferenceConfig,
)


class WayPointColor(Enum):
    RED = [1.0, 0.0, 0.0, 1.0]
    GREEN = [0.0, 1.0, 0.0, 1.0]
    BLUE = [0.0, 0.0, 1.0, 1.0]
    YELLOW = [1.0, 1.0, 0.0, 1.0]
    CYAN = [0.0, 1.0, 1.0, 1.0]
    MAGENTA = [1.0, 0.0, 1.0, 1.0]


class PILDigitalTwin:

    def __init__(
        self, control_pair: PolicyPandaRobotiqDeltaCartesianControlPair
    ):
        self.mirror = RobotMirror.from_model_id(
            RobotModelId.FRANKA_PANDA_ROBOTIQ
        )
        self.control_pair = control_pair
        self.panda_arm = control_pair.panda_arm
        self.lastest_action: Optional[XRTrajectory] = None
        self.history_traj: Optional[XRTrajectory] = None
        self.running = True
        self.visualize_thread = threading.Thread(
            target=self._visualize_loop, daemon=True
        )
        self.visualize_thread.start()

    def apply_arm_state(self, joint_positions: np.ndarray):
        self.mirror.apply_arm_state(joint_positions)

    def update_action(self, action: np.ndarray):
        way_points: List[TrajectoryWaypointDict] = [
            {
                "pos": action[:3].tolist(),
                "color": WayPointColor.RED.value,
            }
        ]
        if self.lastest_action is None:
            self.lastest_action = self.mirror._cavns.create_trajectory(
                name="latest_action_traj", waypoints=way_points
            )
        else:
            self.lastest_action.update(waypoints=way_points)

        if self.history_traj is None:
            self.history_traj = self.mirror._cavns.create_trajectory(
                name="history_traj", waypoints=way_points
            )
        else:
            self.history_traj.update(waypoints=way_points)

    def add_traj_point(self, pos: np.ndarray, color: WayPointColor):
        way_point: TrajectoryWaypointDict = {
            "pos": pos[:3].tolist(),
            "color": color.value,
        }
        if self.history_traj is None:
            self.history_traj = self.mirror._cavns.create_trajectory(
                name="history_traj", waypoints=[way_point]
            )
        else:
            current_waypoints = self.history_traj.get_waypoints()
            current_waypoints.append(way_point)
            self.history_traj.update(waypoints=current_waypoints)

    def _visualize_loop(self) -> None:
        while self.running:
            arm_state = self.panda_arm.current_state
            if arm_state is None:
                continue
            self.apply_arm_state(np.array(arm_state["q"]))
            self.add_traj_point(
                np.array(arm_state["EE_pos"]), WayPointColor.BLUE
            )
            time.sleep(0.05)


class MQ3TrajVisualLeRobotInference(LeRobotPolicyInference):
    def __init__(
        self,
        data_collectors: List[IRLDataWrapper],
        control_pair: PolicyPandaRobotiqDeltaCartesianControlPair,
        cfg: LeRobotPolicyInferenceConfig,
    ) -> None:
        super().__init__(data_collectors, control_pair, cfg)
        self.running = True

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
            # Populate observation queues and stack images (mirroring select_action preprocessing)
            from lerobot.utils.constants import ACTION, OBS_IMAGES
            from lerobot.policies.utils import populate_queues
            obs = dict(observation)
            if ACTION in obs:
                obs.pop(ACTION)
            if self.policy.config.image_features:
                obs[OBS_IMAGES] = torch.stack([obs[key] for key in self.policy.config.image_features], dim=-4)
            self.policy._queues = populate_queues(self.policy._queues, obs)
            action_chunk = self.policy.predict_action_chunk(obs)

        if action_chunk.ndim == 2:
            action_chunk = action_chunk.unsqueeze(1)
        elif action_chunk.ndim != 3:
            raise RuntimeError(
                f"Expected action_chunk to have shape (B, T, D) or (B, D), got {tuple(action_chunk.shape)}"
            )

        batch_size, chunk_size, action_dim = action_chunk.shape
        post_action_chunk = torch.zeros(
            (batch_size, chunk_size, action_dim), dtype=torch.float32
        )
        for chunk_idx in range(chunk_size):
            single_action = action_chunk[:, chunk_idx, :]
            processed_action = self.postprocessor(single_action)
            post_action_chunk[:, chunk_idx, :] = processed_action

        # Remove batch dim: (1, T, D) -> (T, D) for the action buffer
        post_action_chunk = post_action_chunk[0].float().cpu().numpy()

        try:
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
        self.control_pair.reset_action()
        return super()._reset_arm()
