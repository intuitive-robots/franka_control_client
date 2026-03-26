#!/usr/bin/env python

from __future__ import annotations

import logging
import queue
import random
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import hydra
import numpy as np
import pyzlc
import torch
from omegaconf import DictConfig


log = logging.getLogger(__name__)


def set_seed_everywhere(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


@dataclass
class BesoPolicyNodeConfig:
    checkpoint_path: str
    stats_data_dir: str
    dataset_repo_id: str
    device: str
    use_non_ema: bool
    sampling_steps: int | None
    obs_topic: str
    action_topic: str
    pyzlc_name: str
    pyzlc_host: str
    pyzlc_group: str
    pyzlc_group_name: str
    pyzlc_group_port: int
    seed: int


class BesoPolicyNode:
    def __init__(self, cfg: BesoPolicyNodeConfig) -> None:
        self.cfg = cfg
        self._running = False
        self._obs_queue: queue.Queue[dict[str, Any] | None] = queue.Queue()
        self._obs_seq_lock = threading.Lock()
        self._active_rollout_id: int | None = None
        self._last_enqueued_obs_seq: int | None = None

        pyzlc.init(
            self.cfg.pyzlc_name,
            self.cfg.pyzlc_host,
            group=self.cfg.pyzlc_group,
            group_name=self.cfg.pyzlc_group_name,
            group_port=self.cfg.pyzlc_group_port,
        )
        pyzlc.register_subscriber_handler(self.cfg.obs_topic, self._on_observation)
        self._action_pub = pyzlc.Publisher(self.cfg.action_topic)

        self.policy, self.device = self._load_policy()
        self.policy.reset()

    def _clear_pending_observations(self) -> None:
        saw_stop_sentinel = False
        while True:
            try:
                item = self._obs_queue.get_nowait()
            except queue.Empty:
                break
            if item is None:
                saw_stop_sentinel = True
        if saw_stop_sentinel:
            self._obs_queue.put(None)

    def _on_observation(self, msg: dict[str, Any]) -> None:
        rollout_id = int(msg["rollout_id"])
        obs_seq = int(msg["obs_seq"])
        with self._obs_seq_lock:
            if self._active_rollout_id is None or rollout_id > self._active_rollout_id:
                self._active_rollout_id = rollout_id
                self._last_enqueued_obs_seq = None
                self._clear_pending_observations()
            elif rollout_id < self._active_rollout_id:
                return
            if self._last_enqueued_obs_seq is not None and obs_seq <= self._last_enqueued_obs_seq:
                return
            self._last_enqueued_obs_seq = obs_seq
        self._obs_queue.put(msg)

    def _decode_image(self, img: Any) -> np.ndarray:
        if isinstance(img, np.ndarray):
            return img
        h = int(img["height"])
        w = int(img["width"])
        c = int(img.get("channels", 3))
        arr = np.frombuffer(img["rgb_data"], dtype=np.uint8)
        return arr.reshape((h, w, c)).copy()

    def _load_policy(self):
        from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
        from policies.beso.beso_config import BesoConfig
        from policies.beso.modelling_beso import BesoPolicy

        ckpt = Path(self.cfg.checkpoint_path).expanduser().resolve()
        stats_root = Path(self.cfg.stats_data_dir).expanduser().resolve()

        device = torch.device(self.cfg.device)

        model_cfg = BesoConfig.from_pretrained(ckpt)
        model_cfg.device = str(device)
        model_cfg.load_non_ema = bool(self.cfg.use_non_ema)

        ds_meta = LeRobotDatasetMetadata(
            repo_id=self.cfg.dataset_repo_id,
            root=stats_root,
        )
        policy = BesoPolicy.from_pretrained(
            ckpt,
            config=model_cfg,
            dataset_meta=ds_meta,
            strict=False,
        )
        policy = policy.to(device)
        policy.eval()

        if self.cfg.sampling_steps is not None:
            policy.config.sampling_steps = int(self.cfg.sampling_steps)

        return policy, device

    def _build_policy_input(self, obs_msg: dict[str, Any]) -> dict[str, torch.Tensor]:
        state = np.asarray(obs_msg["state"], dtype=np.float32).reshape(1, -1)

        batch: dict[str, torch.Tensor] = {
            "observation.state": torch.from_numpy(state).to(self.device),
        }

        images = obs_msg["images"]
        for feat_key in self.policy.config.input_features.keys():
            key = str(feat_key)

            if key == "observation.state":
                continue

            if key.startswith("observation.goal.") and not bool(getattr(self.policy.config, "goal_conditioned", False)):
                continue

            if key.startswith("observation.image."):
                cam_name = key.replace("observation.image.", "", 1)
                rgb = self._decode_image(images[cam_name]).astype(np.float32) / 255.0
                chw = np.transpose(rgb, (2, 0, 1)).copy()
                batch[key] = torch.from_numpy(chw).unsqueeze(0).to(self.device)
                continue

            if key.startswith("observation.goal."):
                goal = np.asarray(obs_msg[key], dtype=np.float32)
                if goal.ndim == 1:
                    goal = goal[None, None, :]
                elif goal.ndim == 2:
                    goal = goal[None, :, :]
                batch[key] = torch.from_numpy(goal).to(self.device)

        return batch

    def _process_observation(self, obs_msg: dict[str, Any]) -> None:
        rollout_id = int(obs_msg["rollout_id"])
        obs_seq = int(obs_msg["obs_seq"])
        if bool(obs_msg.get("reset_policy", False)):
            self.policy.reset()

        policy_input = self._build_policy_input(obs_msg)

        with torch.inference_mode():
            action = self.policy.select_action(policy_input)

        action_vec = action[0] if action.ndim == 2 else action
        payload = {
            "rollout_id": rollout_id,
            "source_obs_seq": obs_seq,
            "timestamp": time.time(),
            "action": action_vec.detach().cpu().tolist(),
            "shape": list(action_vec.shape),
        }
        self._action_pub.publish(payload)

    def run(self) -> None:
        self._running = True

        while self._running:
            try:
                obs_msg = self._obs_queue.get()
                if obs_msg is None:
                    break
                self._process_observation(obs_msg)
            except Exception as exc:
                pyzlc.error(f"beso_policy_node step error: {exc}")

    def stop(self) -> None:
        self._running = False
        self._obs_queue.put(None)


def _cfg_get(cfg: DictConfig, key: str, default: Any = None) -> Any:
    return cfg.get(key, default) if isinstance(cfg, DictConfig) else default


def _build_node_cfg(cfg: DictConfig) -> BesoPolicyNodeConfig:
    return BesoPolicyNodeConfig(
        checkpoint_path=str(_cfg_get(cfg, "checkpoint_path")),
        stats_data_dir=str(_cfg_get(cfg, "stats_data_dir")),
        dataset_repo_id=str(_cfg_get(cfg, "dataset_repo_id")),
        device=str(_cfg_get(cfg, "device", "cuda" if torch.cuda.is_available() else "cpu")),
        use_non_ema=bool(_cfg_get(cfg, "use_non_ema", False)),
        sampling_steps=_cfg_get(cfg, "sampling_steps", None),
        obs_topic=str(_cfg_get(cfg, "obs_topic", "beso/observation")),
        action_topic=str(_cfg_get(cfg, "action_topic", "beso/action")),
        pyzlc_name=str(_cfg_get(cfg, "pyzlc_name", "beso_policy_node")),
        pyzlc_host=str(_cfg_get(cfg, "pyzlc_host")),
        pyzlc_group=str(_cfg_get(cfg, "pyzlc_group", "224.0.0.1")),
        pyzlc_group_name=str(_cfg_get(cfg, "pyzlc_group_name", "realrobot_rollout")),
        pyzlc_group_port=int(_cfg_get(cfg, "pyzlc_group_port", 7730)),
        seed=int(_cfg_get(cfg, "seed", 1000)),
    )


@hydra.main(config_path="../../../configs", config_name="beso_rollout_franka.yaml", version_base="1.3")
def main(cfg: DictConfig) -> None:
    logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s: %(message)s")

    policy_cfg = cfg.policy_node

    lerobot_repo_root = str(_cfg_get(policy_cfg, "lerobot_repo_root"))
    src_root = Path(lerobot_repo_root).expanduser().resolve() / "src"
    sys.path.insert(0, str(src_root))

    node_cfg = _build_node_cfg(policy_cfg)

    set_seed_everywhere(node_cfg.seed)

    node = BesoPolicyNode(node_cfg)
    try:
        node.run()
    except KeyboardInterrupt:
        log.info("KeyboardInterrupt received, exiting.")
    finally:
        node.stop()
        pyzlc.shutdown()


if __name__ == "__main__":
    main()
