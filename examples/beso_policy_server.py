"""Serve a LeRobot BESO checkpoint to the inference GUI (run on the policy PC).

    conda activate franka_control_client_interrupt      # LeRobot fork with the beso policy
    PYTHONPATH=src python examples/beso_policy_server.py --port 8767 \
        --ckpt /home/jjiang/xinkai/ckpt/seed-42/42-beso/lemon_in_drawer_10/checkpoints/last/pretrained_model

BESO checkpoints predict absolute EE poses: select the cartesian control pair in the GUI.
Inputs are taken from the checkpoint config: observation.state (xyz + quat xyzw + gripper)
and the cameras listed there (e.g. static_cam / wrist_cam), resized to the trained size.
The GUI camera obs_key must be observation.images.<name> with the checkpoint's names, or
use --camera-map (e.g. external=static_cam,wrist=wrist_cam).

Checkpoints written by the PIL/correction BESO fork carry config keys and preprocessor
steps that the plain BESO runtime does not know; they are training-only, so they are
dropped from a temporary copy of the checkpoint (the checkpoint itself is not changed).
"""

from __future__ import annotations

import argparse
import json
import logging
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import torch

from franka_control_client.inference_gui.policy_server import (
    CARTESIAN_ACTION_SPACE,
    BasePolicy,
    PolicyServer,
)

LOG = logging.getLogger("beso_server")

# Written by the PIL/correction/residual BESO fork, unknown to BESOConfig (training only).
LEGACY_CONFIG_KEYS = {
    "training_data_mode", "correction_negative_action_key", "correction_mask_key",
    "correction_beta", "correction_bc_loss_weight", "correction_preference_loss_weight",
    "use_residual", "residual_horizon", "residual_n_obs_steps", "residual_hidden_dim",
    "residual_n_layers", "residual_dropout", "residual_vit_model", "freeze_residual_vit",
    "residual_vision_feature_dim", "freeze_base_policy", "residual_base_action_key",
    "residual_target_key", "residual_quaternion_order", "residual_loss_type",
}
LEGACY_PROCESSOR_STEPS = {"beso_correction_action_normalizer", "beso_residual_target_normalizer"}


def sanitized_checkpoint(ckpt: Path, out: Path) -> Path:
    """Symlink the checkpoint into ``out`` with the fork-only config keys / steps removed."""
    config = json.loads((ckpt / "config.json").read_text())
    if config.get("use_residual"):
        raise RuntimeError("Residual BESO checkpoints are not supported by the plain BESO runtime.")
    for f in ckpt.iterdir():
        if f.name not in ("config.json", "policy_preprocessor.json"):
            (out / f.name).symlink_to(f.resolve())
    dropped = sorted(LEGACY_CONFIG_KEYS & config.keys())
    for key in dropped:
        del config[key]
    (out / "config.json").write_text(json.dumps(config, indent=2))
    pre = json.loads((ckpt / "policy_preprocessor.json").read_text())
    pre["steps"] = [s for s in pre["steps"] if s.get("registry_name") not in LEGACY_PROCESSOR_STEPS]
    (out / "policy_preprocessor.json").write_text(json.dumps(pre, indent=2))
    if dropped:
        LOG.info("Ignoring %d fork-only config keys", len(dropped))
    return out


def load_weights(policy: torch.nn.Module, model_file: Path) -> None:
    """Load model.safetensors as a plain state dict.

    BESO checkpoints contain tied CLIP text-encoder tensors, which the safetensors model
    loader refuses. CLIP keys are saved with or without the ``text_model.`` level depending
    on the transformers version; remap them so the fine-tuned text encoder is really loaded.
    """
    from safetensors.torch import load_file

    model_keys = set(policy.state_dict())
    state_dict = {}
    for key, value in load_file(str(model_file)).items():
        if key not in model_keys and ".clip_text." in key:
            head, tail = key.split(".clip_text.", 1)
            alt = tail[len("text_model."):] if tail.startswith("text_model.") else f"text_model.{tail}"
            if f"{head}.clip_text.{alt}" in model_keys:
                key = f"{head}.clip_text.{alt}"
        state_dict[key] = value
    missing, unexpected = policy.load_state_dict(state_dict, strict=False)
    missing = [k for k in missing if not k.endswith("position_ids")]
    unexpected = [k for k in unexpected if not k.endswith("position_ids")]
    if missing or unexpected:
        raise RuntimeError(f"checkpoint does not match the model: {len(missing)} missing keys "
                           f"(e.g. {missing[:3]}), {len(unexpected)} unexpected (e.g. {unexpected[:3]})")


class BesoPolicy(BasePolicy):
    action_space = CARTESIAN_ACTION_SPACE

    def __init__(self, ckpt: str, device: str = "cuda", fps: float = 20.0, exec_steps: int = 25,
                 task: str = "", camera_map: Optional[Dict[str, str]] = None) -> None:
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors

        self.ckpt = Path(ckpt).resolve()
        self.device = device
        self.fps = fps
        self.exec_steps = exec_steps
        self.fixed_task = task
        self.camera_map = camera_map or {}  # GUI key suffix -> checkpoint camera name
        self.task: Optional[str] = None

        t0 = time.time()
        self._tmp = tempfile.TemporaryDirectory(prefix="beso_ckpt_")
        path = sanitized_checkpoint(self.ckpt, Path(self._tmp.name))
        cfg = PreTrainedConfig.from_pretrained(path)
        cfg.pretrained_path = str(path)
        cfg.device = device
        self.policy = get_policy_class(cfg.type)(cfg)
        load_weights(self.policy, path / "model.safetensors")
        self.policy.to(device).eval()
        self.pre, self.post = make_pre_post_processors(
            cfg, pretrained_path=str(path),
            preprocessor_overrides={
                "device_processor": {"device": device},
                "rename_observations_processor": {"rename_map": {}},
            },
        )
        self.image_shapes = {k: tuple(f.shape) for k, f in cfg.image_features.items()}  # (C, H, W)
        self.horizon = int(cfg.horizon)
        LOG.info("Loaded %s in %.1f s: cameras %s, horizon %d", self.ckpt, time.time() - t0,
                 list(self.image_shapes), self.horizon)

    def metadata(self) -> Dict[str, Any]:
        return {"name": "beso", "ckpt": str(self.ckpt), "cameras": list(self.image_shapes),
                "horizon": self.horizon, "task": self.fixed_task or None}

    def reset(self, task: Optional[str]) -> None:
        self.task = self.fixed_task or task
        self.policy.reset()

    def _image(self, observation: Dict[str, Any], key: str) -> torch.Tensor:
        name = key[len("observation.images."):]
        sources = [key] + [f"observation.images.{gui}" for gui, ck in self.camera_map.items() if ck == name]
        img = next((observation[k] for k in sources if k in observation), None)
        if img is None:
            raise KeyError(f"camera '{name}' not in observation (got {sorted(observation)}); "
                           f"set its obs_key to {key} in the GUI or use --camera-map")
        _, h, w = self.image_shapes[key]
        t = torch.from_numpy(np.ascontiguousarray(img)).permute(2, 0, 1)[None].float()
        if t.shape[-2:] != (h, w):
            t = torch.nn.functional.interpolate(t, size=(h, w), mode="bilinear", align_corners=False)
        return t / 255.0  # [1, 3, H, W]

    @torch.no_grad()
    def infer(self, observation: Dict[str, Any]) -> np.ndarray:
        state = np.asarray(observation["observation.state"], dtype=np.float32)[:8]
        batch: Dict[str, Any] = {"observation.state": torch.from_numpy(state)[None]}
        for key in self.image_shapes:
            batch[key] = self._image(observation, key)
        batch["task"] = [self.task or observation.get("task") or ""]
        batch = self.pre(batch)
        actions = self.policy.predict_action_chunk(batch)  # [1, T, 8], normalized
        actions = self.post(actions)
        chunk = actions[0].float().cpu().numpy()
        chunk[:, 3:7] /= np.linalg.norm(chunk[:, 3:7], axis=1, keepdims=True)
        chunk[:, 7] = np.clip(chunk[:, 7], 0.0, 1.0)
        return chunk


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--ckpt", required=True, help="pretrained_model folder of the checkpoint")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--fps", type=float, default=20.0, help="training data rate (rows per second)")
    parser.add_argument("--exec-steps", type=int, default=25, help="rows executed before the next request")
    parser.add_argument("--task", default="", help="override the GUI task (use the exact training string)")
    parser.add_argument("--camera-map", default="",
                        help="GUI key -> checkpoint camera, e.g. external=static_cam,wrist=wrist_cam")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    camera_map = dict(p.split("=", 1) for p in args.camera_map.split(",") if p)
    policy = BesoPolicy(args.ckpt, args.device, args.fps, args.exec_steps, args.task, camera_map)
    PolicyServer(policy, host=args.host, port=args.port).serve_forever()


if __name__ == "__main__":
    main()
