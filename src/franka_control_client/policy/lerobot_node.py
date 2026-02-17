#!/usr/bin/env python

"""
Policy node using pyzlc (lerobot_eval-style inference wrapper).

This node subscribes to an observation topic and publishes actions.
It is a minimal "EvalNode" wrapper around LeRobot XVLA inference.

Expected observation message format (Python dict):
{
  "state": list[float] | np.ndarray,                     # required
  "images": {                                            # required
    "<cam_name>": np.ndarray(H,W,3) | {                  # uint8 RGB
        "height": int, "width": int, "channels": int,
        "rgb_data": bytes
    }
  },
  "task": str | None                                     # optional
}

Published action message format:
{
  "timestamp": float,
  "action": list[float],
  "shape": list[int],
}
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import numpy as np
import pyzlc
import torch

from lerobot.policies.factory import get_policy_class, make_pre_post_processors


@dataclass
class EvalNodeConfig:
    policy_type: str
    policy_path: str
    device: str
    obs_topic: str
    action_topic: str
    fps: float
    default_task: str


class EvalNode:
    def __init__(self, cfg: EvalNodeConfig) -> None:
        self.cfg = cfg
        self._latest_obs: Optional[Dict[str, Any]] = None
        self._running = False

        # Subscribe to observations
        pyzlc.register_subscriber_handler(self.cfg.obs_topic, self._on_observation)

        # Publisher for actions
        self._action_pub = pyzlc.Publisher(self.cfg.action_topic)

        # Load policy + processors
        policy_class = get_policy_class(self.cfg.policy_type)
        self.policy = policy_class.from_pretrained(self.cfg.policy_path)
        self.policy.to(self.cfg.device)

        device_override = {"device": self.cfg.device}
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            self.policy.config,
            pretrained_path=self.cfg.policy_path,
            preprocessor_overrides={"device_processor": device_override},
            postprocessor_overrides={"device_processor": device_override},
        )
        self._expected_image_shapes = self._get_expected_image_shapes()
        self._expected_state_dim = self._get_expected_state_dim()

    def _on_observation(self, msg: Dict[str, Any]) -> None:
        self._latest_obs = msg

    def _decode_image(self, img: Any) -> np.ndarray:
        if isinstance(img, np.ndarray):
            return img
        if isinstance(img, dict) and "rgb_data" in img:
            h = int(img["height"])
            w = int(img["width"])
            c = int(img.get("channels", 3))
            image_array = np.frombuffer(img["rgb_data"], dtype=np.uint8)
            return image_array.reshape((h, w, c))
        if isinstance(img, list):
            return np.asarray(img, dtype=np.uint8)
        raise ValueError("Unsupported image format in observation.")

    def _get_expected_image_shapes(self) -> dict[str, tuple[int, int, int]]:
        shapes: dict[str, tuple[int, int, int]] = {}
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
        cfg = getattr(self.policy, "config", None)
        input_feats = getattr(cfg, "input_features", None)
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
        if img.shape[:2] == (h, w):
            return img
        img_t = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).float()
        resized = torch.nn.functional.interpolate(img_t, size=(h, w), mode="bilinear", align_corners=False)
        out = resized.squeeze(0).permute(1, 2, 0).byte().cpu().numpy()
        if out.shape[2] != c:
            raise ValueError(f"Image channels mismatch after resize: expected {c}, got {out.shape[2]}")
        return out

    def _build_observation(self, obs_msg: Dict[str, Any]) -> Dict[str, torch.Tensor]:
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
            pyzlc.info(
                f"Adjusted state dim to {self._expected_state_dim} (now {state.shape[-1]})."
            )

        observation: Dict[str, torch.Tensor] = {
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
            observation[obs_key] = torch.from_numpy(rgb)

        task = obs_msg.get("task") or self.cfg.default_task
        if task:
            observation["task"] = task

        return observation

    def step(self) -> None:
        if self._latest_obs is None:
            return

        obs_msg = self._latest_obs
        observation = self._build_observation(obs_msg)

        observation = self.preprocessor(observation)
        with torch.inference_mode():
            action = self.policy.select_action(observation)
        action = self.postprocessor(action)

        if action.ndim == 2:
            action_vec = action[0]
        else:
            action_vec = action

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


def _parse_args() -> EvalNodeConfig:
    parser = argparse.ArgumentParser(description="LeRobot policy node (pyzlc).")
    parser.add_argument("--policy_type", default="xvla", help="Policy type (e.g., xvla/act/smolvla).")
    parser.add_argument("--policy_path", required=True, help="HF repo or local path to policy.")
    parser.add_argument("--device", default="cpu", help="Torch device (cpu/cuda/mps).")
    parser.add_argument("--obs_topic", default="xvla/observation", help="pyzlc topic for observations.")
    parser.add_argument("--action_topic", default="xvla/action", help="pyzlc topic for actions.")
    parser.add_argument("--fps", type=float, default=30.0, help="Max inference rate.")
    parser.add_argument("--default_task", default="", help="Fallback task string if not in obs.")
    args = parser.parse_args()
    return EvalNodeConfig(
        policy_type=args.policy_type,
        policy_path=args.policy_path,
        device=args.device,
        obs_topic=args.obs_topic,
        action_topic=args.action_topic,
        fps=args.fps,
        default_task=args.default_task,
    )


def main() -> None:
    cfg = _parse_args()
    node = EvalNode(cfg)
    node.run()


if __name__ == "__main__":
    main()
