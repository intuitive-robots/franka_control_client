#!/usr/bin/env python

"""
XVLA policy node using pyzlc.

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
        policy_class = get_policy_class("xvla")
        self.policy = policy_class.from_pretrained(self.cfg.policy_path)
        self.policy.to(self.cfg.device)

        device_override = {"device": self.cfg.device}
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            self.policy.config,
            pretrained_path=self.cfg.policy_path,
            preprocessor_overrides={"device_processor": device_override},
            postprocessor_overrides={"device_processor": device_override},
        )

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

    def _build_observation(self, obs_msg: Dict[str, Any]) -> Dict[str, torch.Tensor]:
        if "state" not in obs_msg:
            raise ValueError("Observation is missing 'state'.")
        if "images" not in obs_msg:
            raise ValueError("Observation is missing 'images'.")

        state = np.asarray(obs_msg["state"], dtype=np.float32)
        if state.ndim == 1:
            state = state[None, :]

        observation: Dict[str, torch.Tensor] = {
            "observation.state": torch.from_numpy(state),
        }

        images = obs_msg["images"]
        if not isinstance(images, dict):
            raise ValueError("'images' must be a dict keyed by camera name.")

        for cam_name, cam_img in images.items():
            rgb = self._decode_image(cam_img)
            observation[f"observation.images.{cam_name}"] = torch.from_numpy(rgb)

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
    parser = argparse.ArgumentParser(description="XVLA policy node (pyzlc).")
    parser.add_argument("--policy_path", required=True, help="HF repo or local path to XVLA policy.")
    parser.add_argument("--device", default="cpu", help="Torch device (cpu/cuda/mps).")
    parser.add_argument("--obs_topic", default="xvla/observation", help="pyzlc topic for observations.")
    parser.add_argument("--action_topic", default="xvla/action", help="pyzlc topic for actions.")
    parser.add_argument("--fps", type=float, default=30.0, help="Max inference rate.")
    parser.add_argument("--default_task", default="", help="Fallback task string if not in obs.")
    args = parser.parse_args()
    return EvalNodeConfig(
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
