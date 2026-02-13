#!/usr/bin/env python

"""
Policy node using pyzlc transport with eval_xvla-style LeRobot policy loading.

This node subscribes to an observation topic and publishes actions.
Observation/action pub-sub behavior stays unchanged; only policy/config loading
matches eval_xvla.py.
"""

from __future__ import annotations

import importlib
import logging
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import hydra
import numpy as np
import pyzlc
import torch
from lerobot.configs.train import TrainPipelineConfig
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.utils.random_utils import set_seed
from lerobot.utils.utils import get_safe_torch_device
from omegaconf import DictConfig

log = logging.getLogger(__name__)

INIT_ACTION = [0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0, 0.0]
INIT_ACTION_PUBLISH_INTERVAL_S = 0.5


@dataclass
class EvalNodeConfig:
    policy_type: str
    checkpoint_path: str
    dataset_path: Optional[str]
    device: str
    policy_dtype: Optional[str]
    obs_topic: str
    action_topic: str
    fps: float
    default_task: str
    pyzlc_name: str
    pyzlc_host: str
    pyzlc_group_name: str
    pyzlc_group_port: int


class EvalNode:
    def __init__(self, cfg: EvalNodeConfig) -> None:
        self.cfg = cfg
        self._latest_obs: Optional[Dict[str, Any]] = None
        self._running = False
        self._last_init_publish_ts = 0.0

        pyzlc.init(
            self.cfg.pyzlc_name,
            self.cfg.pyzlc_host,
            group_name=self.cfg.pyzlc_group_name,
            group_port=self.cfg.pyzlc_group_port,
        )
        pyzlc.register_subscriber_handler(self.cfg.obs_topic, self._on_observation)
        self._action_pub = pyzlc.Publisher(self.cfg.action_topic)
        self._publish_init_action(force=True)

        self.train_cfg = self._load_train_cfg()
        self.policy, self.preprocessor, self.postprocessor = self._load_policy_stack()

        self._expected_image_shapes = self._get_expected_image_shapes()
        self._expected_state_dim = self._get_expected_state_dim()

    def _publish_init_action(self, force: bool = False) -> None:
        now = time.time()
        if not force and (now - self._last_init_publish_ts) < INIT_ACTION_PUBLISH_INTERVAL_S:
            return
        payload = {
            "timestamp": now,
            "action": INIT_ACTION,
            "shape": [len(INIT_ACTION)],
        }
        self._action_pub.publish(payload)
        self._last_init_publish_ts = now
        if force:
            pyzlc.info(f"Published init action on '{self.cfg.action_topic}': {INIT_ACTION}")

    def _load_train_cfg(self) -> TrainPipelineConfig:
        cli_args = [
            f"--policy.pretrained_path={self.cfg.checkpoint_path}",
            f"--policy.device={self.cfg.device}",
            "--dataset.image_transforms.enable=false",
        ]
        if self.cfg.dataset_path:
            cli_args.append(f"--dataset.root={self.cfg.dataset_path}")
        if self.cfg.policy_dtype:
            cli_args.append(f"--policy.dtype={self.cfg.policy_dtype}")

        train_cfg = TrainPipelineConfig.from_pretrained(
            pretrained_name_or_path=self.cfg.checkpoint_path,
            cli_args=cli_args,
        )

        if any("empty_camera" in key for key in train_cfg.policy.input_features):
            train_cfg.policy.input_features = {
                "observation.images.image": PolicyFeature(
                    type=FeatureType.VISUAL, shape=(3, 256, 256)
                ),
                "observation.images.image2": PolicyFeature(
                    type=FeatureType.VISUAL, shape=(3, 256, 256)
                ),
                "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(8,)),
            }
            train_cfg.policy.num_views = 2
            train_cfg.policy.empty_camera = 1

        return train_cfg

    def _load_dataset_meta(self) -> Any:
        # Prefer the same helper used in eval_xvla.py when available.
        try:
            eval_utils = importlib.import_module("src.utils.eval_utils")
            return eval_utils.load_dataset_meta(self.train_cfg)
        except Exception as exc:
            pyzlc.info(f"load_dataset_meta helper unavailable, proceeding without ds_meta: {exc}")
            return None

    def _load_policy_stack(self):
        ds_meta = self._load_dataset_meta()
        device = get_safe_torch_device(self.train_cfg.policy.device, log=True)

        policy = make_policy(
            cfg=self.train_cfg.policy,
            env_cfg=None,
            ds_meta=ds_meta,
            rename_map=getattr(self.train_cfg, "rename_map", None),
        )
        policy.eval()
        policy.to(device)
        pyzlc.info(f"Policy running on {device}")

        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=self.train_cfg.policy,
            pretrained_path=self.train_cfg.policy.pretrained_path,
            preprocessor_overrides={"device_processor": {"device": device.type}},
        )

        return policy, preprocessor, postprocessor

    def _on_observation(self, msg: Dict[str, Any]) -> None:
        self._latest_obs = msg

    def _decode_image(self, img: Any) -> np.ndarray:
        if isinstance(img, np.ndarray):
            return np.ascontiguousarray(img)
        if isinstance(img, dict) and "rgb_data" in img:
            h = int(img["height"])
            w = int(img["width"])
            c = int(img.get("channels", 3))
            image_array = np.frombuffer(img["rgb_data"], dtype=np.uint8)
            # np.frombuffer may create a read-only view; copy to avoid non-writable tensor warnings.
            return image_array.reshape((h, w, c)).copy()
        if isinstance(img, list):
            return np.ascontiguousarray(np.asarray(img, dtype=np.uint8))
        raise ValueError("Unsupported image format in observation.")

    def _get_expected_image_shapes(self) -> dict[str, tuple[int, int, int]]:
        shapes: dict[str, tuple[int, int, int]] = {}
        input_feats = getattr(self.train_cfg.policy, "input_features", None)
        if isinstance(input_feats, dict):
            for key, feat in input_feats.items():
                if not str(key).startswith("observation.images."):
                    continue
                try:
                    shape = tuple(feat.shape)
                except Exception:
                    continue
                if len(shape) == 3:
                    shapes[key] = (shape[0], shape[1], shape[2])
        if shapes:
            return shapes

        # Fallback for policies exposing image feature map in config.
        cfg = getattr(self.policy, "config", None)
        image_feats = getattr(cfg, "image_features", None)
        if isinstance(image_feats, dict):
            for key, feat in image_feats.items():
                try:
                    shape = tuple(feat.shape)
                except Exception:
                    continue
                if len(shape) == 3:
                    shapes[key] = (shape[0], shape[1], shape[2])
        return shapes

    def _get_expected_state_dim(self) -> Optional[int]:
        input_feats = getattr(self.train_cfg.policy, "input_features", None)
        if isinstance(input_feats, dict) and "observation.state" in input_feats:
            try:
                shape = input_feats["observation.state"].shape
                if len(shape) >= 1:
                    return int(shape[-1])
            except Exception:
                return None
        return None

    def _resize_image(self, img: np.ndarray, shape: tuple[int, int, int]) -> np.ndarray:
        c, h, w = shape
        img = np.ascontiguousarray(img)
        if img.shape[:2] == (h, w):
            return img
        img_t = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).float()
        resized = torch.nn.functional.interpolate(
            img_t, size=(h, w), mode="bilinear", align_corners=False
        )
        out = resized.squeeze(0).permute(1, 2, 0).byte().cpu().numpy()
        if out.shape[2] != c:
            raise ValueError(f"Image channels mismatch after resize: expected {c}, got {out.shape[2]}")
        return out

    def _build_observation(self, obs_msg: Dict[str, Any]) -> Dict[str, Any]:
        if "state" not in obs_msg:
            raise ValueError("Observation is missing 'state'.")
        if "images" not in obs_msg:
            raise ValueError("Observation is missing 'images'.")

        state = np.asarray(obs_msg["state"], dtype=np.float32)
        if state.ndim == 1:
            state = state[None, :]
        if self._expected_state_dim is not None and state.shape[-1] != self._expected_state_dim:
            if state.shape[-1] > self._expected_state_dim:
                state = state[..., : self._expected_state_dim]
            else:
                pad = self._expected_state_dim - state.shape[-1]
                state = np.pad(state, ((0, 0), (0, pad)), mode="constant")
            pyzlc.info(f"Adjusted state dim to {self._expected_state_dim} (now {state.shape[-1]}).")

        observation: Dict[str, Any] = {
            "observation.state": torch.from_numpy(state),
        }

        images = obs_msg["images"]
        if not isinstance(images, dict):
            raise ValueError("'images' must be a dict keyed by camera name.")

        expected_image_keys = list(self._expected_image_shapes.keys())
        if expected_image_keys:
            if set(expected_image_keys).issubset({f"observation.images.{k}" for k in images.keys()}):
                mapped = {
                    k: images[k.replace("observation.images.", "", 1)]
                    for k in expected_image_keys
                }
            elif len(images) == len(expected_image_keys):
                mapped = dict(zip(expected_image_keys, images.values()))
            elif len(images) == 1 and len(expected_image_keys) == 1:
                mapped = {expected_image_keys[0]: next(iter(images.values()))}
            else:
                raise ValueError(
                    f"Image keys mismatch. Expected {expected_image_keys}, got {list(images.keys())}"
                )
        else:
            mapped = {f"observation.images.{k}": v for k, v in images.items()}

        for obs_key, cam_img in mapped.items():
            rgb = self._decode_image(cam_img)
            if obs_key in self._expected_image_shapes:
                rgb = self._resize_image(rgb, self._expected_image_shapes[obs_key])
            if rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError(f"Expected HWC image with 3 channels for {obs_key}, got shape {rgb.shape}")
            # Preprocessor expects image channels first (C,H,W), not HWC.
            rgb = np.ascontiguousarray(rgb)
            observation[obs_key] = torch.from_numpy(rgb).permute(2, 0, 1).contiguous()

        task = obs_msg.get("task") or self.cfg.default_task
        if task:
            observation["task"] = task

        return observation

    def step(self) -> None:
        if self._latest_obs is None:
            self._publish_init_action()
            return

        obs_msg = self._latest_obs
        observation = self._build_observation(obs_msg)
        try:
            observation = self.preprocessor(observation)
        except Exception as exc:
            image_shapes = {
                k: tuple(v.shape)
                for k, v in observation.items()
                if str(k).startswith("observation.images.") and hasattr(v, "shape")
            }
            raise RuntimeError(
                f"Preprocessor failed. image_shapes={image_shapes}, state_shape={tuple(observation['observation.state'].shape)}"
            ) from exc
        with torch.inference_mode():
            action = self.policy.select_action(observation)
        action = self.postprocessor(action)

        action_vec = action[0] if action.ndim == 2 else action

        payload = {
            "timestamp": time.time(),
            "action": action_vec.detach().cpu().tolist(),
            "shape": list(action_vec.shape),
        }
        self._action_pub.publish(payload)

    def run(self) -> None:
        self._running = True
        dt = 1.0 / self.cfg.fps if self.cfg.fps > 0 else 0.0
        while self._running:
            start = time.perf_counter()
            try:
                self.step()
            except Exception as exc:
                pyzlc.error(f"xvla_node step error: {exc}")
            if dt > 0:
                elapsed = time.perf_counter() - start
                if elapsed < dt:
                    pyzlc.sleep(dt - elapsed)

    def stop(self) -> None:
        self._running = False


def _cfg_get(cfg: DictConfig, key: str, default: Any = None) -> Any:
    return cfg.get(key, default) if isinstance(cfg, DictConfig) else default


def _build_node_cfg(cfg: DictConfig) -> EvalNodeConfig:
    checkpoint_path = _cfg_get(cfg, "checkpoint_path", _cfg_get(cfg, "policy_path", None))
    if not checkpoint_path:
        raise ValueError("Missing checkpoint path. Set `checkpoint_path` (or legacy `policy_path`).")

    policy_dtype = _cfg_get(cfg, "policy_dtype", None)
    if policy_dtype is None:
        policy_block = _cfg_get(cfg, "policy", None)
        if policy_block is not None:
            policy_dtype = policy_block.get("dtype", None)

    default_task = _cfg_get(cfg, "default_task", "")
    task_instruction = _cfg_get(cfg, "task_instruction", None)
    if task_instruction:
        default_task = task_instruction

    return EvalNodeConfig(
        policy_type=str(_cfg_get(cfg, "policy_type", "xvla")),
        checkpoint_path=str(checkpoint_path),
        dataset_path=_cfg_get(cfg, "dataset_path", None),
        device=str(_cfg_get(cfg, "device", "cpu")),
        policy_dtype=policy_dtype,
        obs_topic=str(_cfg_get(cfg, "obs_topic", "xvla/observation")),
        action_topic=str(_cfg_get(cfg, "action_topic", "xvla/action")),
        fps=float(_cfg_get(cfg, "fps", 30.0)),
        default_task=str(default_task),
        pyzlc_name=str(_cfg_get(cfg, "pyzlc_name", "xvla")),
        pyzlc_host=str(_cfg_get(cfg, "pyzlc_host", "192.168.0.109")),
        pyzlc_group_name=str(_cfg_get(cfg, "pyzlc_group_name", "DroidGroup")),
        pyzlc_group_port=int(_cfg_get(cfg, "pyzlc_group_port", 7730)),
    )


@hydra.main(config_path="../../../configs", config_name="eval_config.yaml", version_base="1.3")
def main(cfg: DictConfig) -> None:
    torch.backends.cudnn.benchmark = True
    torch.backends.cuda.matmul.allow_tf32 = True
    set_seed(int(_cfg_get(cfg, "seed", 0)))

    node_cfg = _build_node_cfg(cfg)
    node = EvalNode(node_cfg)
    node.run()


if __name__ == "__main__":
    main()
