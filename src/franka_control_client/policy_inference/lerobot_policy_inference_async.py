from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import pyzlc
import torch
import torchvision.transforms.functional as TF
from lerobot.configs.policies import PreTrainedConfig
from safetensors.torch import load_file
from transformers import AutoTokenizer

from lerobot.policies.xvla.modeling_xvla_asyncmulti_v2 import (
    XVLAPolicyAsyncMultiV2 as XVLAPolicy,
)

from ..control_pair.policy_panda_control_pair_chunk import PolicyPandaControlPair
from .irl_wrapper_async import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    PandaGripperDataWrapper,
    RobotiqGripperDataWrapper,
)
from .policy_inference_manager import PolicyInferenceManager

IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]


@dataclass
class LeRobotPolicyInferenceConfig:
    checkpoint_path: str
    task: str
    fps: int = 30
    device: str = "cuda"
    policy_dtype: Optional[str] = None
    dataset_path: Optional[str] = None
    stats_path: Optional[str] = None
    save_startup_images: bool = False
    observation_history_lengths: Optional[Dict[str, int]] = None
    observation_buffer_capacities: Optional[Dict[str, int]] = None
    pad_history_with_oldest: bool = True


def _parse_torch_dtype(dtype_name: Optional[str]) -> Optional[torch.dtype]:
    if dtype_name is None:
        return None
    normalized = dtype_name.lower()
    mapping = {
        "float32": torch.float32,
        "fp32": torch.float32,
        "float": torch.float32,
        "float16": torch.float16,
        "fp16": torch.float16,
        "half": torch.float16,
        "bfloat16": torch.bfloat16,
        "bf16": torch.bfloat16,
    }
    if normalized not in mapping:
        raise ValueError(f"Unsupported policy dtype: {dtype_name}")
    return mapping[normalized]


class LeRobotPolicyInference(PolicyInferenceManager):
    """
    Real-robot XVLA inference using the open-loop eval preprocessing path.

    This keeps the live hardware pieces:
    - real robot observation gathering
    - observation histories for multi-step inputs
    - action chunk updates to the control pair
    - episode lifecycle / UI controls
    """

    def __init__(
        self,
        data_collectors: List[IRL_HardwareDataWrapper],
        control_pair: PolicyPandaControlPair,
        cfg: LeRobotPolicyInferenceConfig,
    ) -> None:
        super().__init__(task=cfg.task, fps=cfg.fps)
        self.data_collectors = data_collectors
        self.control_pair = control_pair
        self.cfg = cfg

        self._checkpoint_path = self._resolve_checkpoint_path(cfg.checkpoint_path)
        self._pretrained_dir = self._resolve_pretrained_dir(self._checkpoint_path)
        self._weights_path = self._resolve_weights_path(self._checkpoint_path)
        self._device = torch.device(cfg.device)
        self._requested_policy_dtype = _parse_torch_dtype(cfg.policy_dtype)

        self.cameras: List[ImageDataWrapper] = []
        self.arm_wrapper: Optional[PandaArmDataWrapper] = None
        self.gripper_wrapper: Optional[IRL_HardwareDataWrapper] = None
        for hw in data_collectors:
            if isinstance(hw, ImageDataWrapper) or hw.hw_type == "camera":
                self.cameras.append(hw)  # type: ignore[arg-type]
            elif isinstance(hw, PandaArmDataWrapper) or hw.hw_type == "follower_arm":
                self.arm_wrapper = hw  # type: ignore[assignment]
            elif isinstance(hw, (PandaGripperDataWrapper, RobotiqGripperDataWrapper)) or hw.hw_type == "follower_gripper":
                self.gripper_wrapper = hw

        if self.arm_wrapper is None:
            raise ValueError("Missing PandaArmDataWrapper for inference.")
        if self.gripper_wrapper is None:
            raise ValueError("Missing gripper wrapper for inference.")

        self._dataset_stats = self._load_dataset_stats()
        self.policy = self._load_policy()
        self._model_float_dtype = self._get_model_float_dtype()
        self._action_mean, self._action_std = self._load_action_mean_std()
        (
            self._expected_state_dim,
            self._expected_state_history_length,
        ) = self._get_expected_state_spec()
        self._expected_image_keys = self._get_expected_image_keys()
        self._n_obs_steps = int(getattr(self.policy.config, "n_obs_steps", 1))
        self._n_action_steps = int(getattr(self.policy.config, "n_action_steps", 1))
        self._state_history_length = self._resolve_state_history_length()
        self._image_history_lengths = self._resolve_image_history_lengths()
        self._configure_observation_buffers()

        tokenizer_name = getattr(self.policy.config, "tokenizer_name", None) or "facebook/bart-large"
        self._tokenizer = AutoTokenizer.from_pretrained(tokenizer_name, use_fast=True)
        self._tokenizer_max_length = 50

        self.register_start_infering_event(self.control_pair.start_control_pair)
        self.register_stop_infering_event(self.control_pair.stop_control_pair)

        self._debug_image_dir = Path("debug/inference_start_images")
        self._startup_images_saved = False
        self._action_observation_log_path = Path("action_observation.txt")
        self._episode_action_observation_log_lines: list[str] = []
        self._last_buffer_debug_print_ts: float = 0.0

    def _resolve_checkpoint_path(self, checkpoint_path: str) -> Path:
        candidate = Path(checkpoint_path).expanduser()
        if candidate.exists():
            return candidate.resolve()
        raise FileNotFoundError(f"Checkpoint path does not exist: {candidate}")

    def _resolve_pretrained_dir(self, checkpoint_path: Path) -> Path:
        return checkpoint_path if checkpoint_path.is_dir() else checkpoint_path.parent

    def _resolve_weights_path(self, checkpoint_path: Path) -> Path:
        if checkpoint_path.is_file():
            return checkpoint_path

        default_path = checkpoint_path / "model.safetensors"
        if default_path.is_file():
            return default_path

        candidates = sorted(checkpoint_path.glob("*.safetensors"))
        if candidates:
            return candidates[0]

        raise FileNotFoundError(f"No .safetensors weights found in {checkpoint_path}")

    def _load_dataset_stats(self) -> Optional[dict[str, dict[str, torch.Tensor]]]:
        stats_candidates: list[Path] = []
        if self.cfg.stats_path:
            stats_candidates.append(Path(self.cfg.stats_path).expanduser())
        if self.cfg.dataset_path:
            dataset_root = Path(self.cfg.dataset_path).expanduser()
            stats_candidates.append(dataset_root / "meta" / "stats.json")
            stats_candidates.append(dataset_root / "stats.json")

        for stats_path in stats_candidates:
            if not stats_path.is_file():
                continue
            with stats_path.open("r", encoding="utf-8") as f:
                stats_json = json.load(f)

            dataset_stats: dict[str, dict[str, torch.Tensor]] = {}
            for key, value in stats_json.items():
                if not isinstance(value, dict) or "mean" not in value or "std" not in value:
                    continue
                dataset_stats[key] = {
                    "mean": torch.tensor(value["mean"], dtype=torch.float32),
                    "std": torch.tensor(value["std"], dtype=torch.float32),
                    "min": torch.tensor(value.get("min", value["mean"]), dtype=torch.float32),
                    "max": torch.tensor(value.get("max", value["mean"]), dtype=torch.float32),
                }

            pyzlc.info(f"Loaded dataset stats from {stats_path}")
            return dataset_stats

        pyzlc.info("No dataset stats file found; action stats will fall back to checkpoint state.")
        return None

    def _load_policy(self) -> XVLAPolicy:
        config = PreTrainedConfig.from_pretrained(pretrained_name_or_path=str(self._pretrained_dir))
        config._dataset_stats = self._dataset_stats

        policy = XVLAPolicy(config, dataset_stats=self._dataset_stats)

        state_dict = load_file(str(self._weights_path), device="cpu")
        remapped_state_dict: dict[str, torch.Tensor] = {}
        for key, value in state_dict.items():
            new_key = key
            if key.startswith("agent."):
                new_key = "model." + key[6:]
            elif key.startswith("policy."):
                new_key = "model." + key[7:]
            elif not key.startswith("model."):
                new_key = "model." + key

            new_key = new_key.replace(".mlp.c_fc1.", ".mlp.fc1.")
            new_key = new_key.replace(".mlp.c_fc2.", ".mlp.fc2.")
            new_key = new_key.replace(".mlp.c_proj.", ".mlp.proj.")
            remapped_state_dict[new_key] = value

        missing, unexpected = policy.load_state_dict(remapped_state_dict, strict=False)
        if missing:
            pyzlc.error(f"Missing keys while loading policy ({len(missing)}): {missing[:10]}")
        if unexpected:
            pyzlc.error(f"Unexpected keys while loading policy ({len(unexpected)}): {unexpected[:10]}")

        policy = policy.to(self._device)
        if self._requested_policy_dtype is not None:
            policy = policy.to(dtype=self._requested_policy_dtype)
        policy.eval()
        if hasattr(policy, "full_forward_at_inference"):
            policy.full_forward_at_inference = False

        pyzlc.info(
            f"Loaded XVLA policy from {self._weights_path} on {self._device} "
            f"(full_forward_at_inference={getattr(policy, 'full_forward_at_inference', None)})"
        )
        return policy

    def _get_model_float_dtype(self) -> torch.dtype:
        for param in self.policy.parameters():
            if param.is_floating_point():
                return param.dtype
        return torch.float32

    def _load_action_mean_std(self) -> tuple[torch.Tensor, torch.Tensor]:
        if self._dataset_stats is not None and "action" in self._dataset_stats:
            action_stats = self._dataset_stats["action"]
            return action_stats["mean"].float(), action_stats["std"].float()

        state_dict = load_file(str(self._weights_path), device="cpu")
        keys = list(state_dict.keys())

        def find_key(required_parts: list[str]) -> Optional[str]:
            for key in keys:
                lower_key = key.lower()
                if all(part in lower_key for part in required_parts):
                    return key
            return None

        mean_key = find_key(["action", "mean"])
        std_key = find_key(["action", "std"])
        if mean_key is None or std_key is None:
            raise KeyError(
                "Could not find action mean/std in checkpoint or dataset stats. "
                f"Sample checkpoint keys: {keys[:20]}"
            )

        return state_dict[mean_key].float(), state_dict[std_key].float()

    def _get_expected_state_spec(self) -> tuple[Optional[int], int]:
        state_feature = getattr(self.policy.config, "input_features", {}).get("observation.state")
        if state_feature is None:
            return None, 1
        try:
            shape = tuple(state_feature.shape)
        except Exception:
            return None, 1
        if not shape:
            return None, 1
        if len(shape) == 1:
            return int(shape[-1]), 1
        return int(shape[-1]), int(shape[0])

    def _get_expected_image_keys(self) -> list[str]:
        image_keys: list[str] = []
        input_features = getattr(self.policy.config, "input_features", {})
        if isinstance(input_features, dict):
            for key in input_features:
                if str(key).startswith("observation.images."):
                    image_keys.append(str(key))
        if image_keys:
            return image_keys
        return ["observation.images.image", "observation.images.image2"]

    def _resolve_state_history_length(self) -> int:
        config_lengths = self.cfg.observation_history_lengths or {}
        if "observation.state" in config_lengths:
            return max(1, int(config_lengths["observation.state"]))
        if "state" in config_lengths:
            return max(1, int(config_lengths["state"]))
        return max(1, self._expected_state_history_length)

    def _resolve_image_history_lengths(self) -> Dict[str, int]:
        config_lengths = self.cfg.observation_history_lengths or {}
        if "camera" in config_lengths:
            default_history = max(1, int(config_lengths["camera"]))
        else:
            default_history = max(1, self._n_obs_steps)

        resolved: Dict[str, int] = {}
        for cam in self.cameras:
            resolved[cam.hw_name] = max(
                1, int(config_lengths.get(cam.hw_name, default_history))
            )
        return resolved

    def _configure_observation_buffers(self) -> None:
        configured_capacities = self.cfg.observation_buffer_capacities or {}

        state_capacity = max(
            self._state_history_length,
            int(
                configured_capacities.get(
                    "observation.state",
                    configured_capacities.get("state", 1),
                )
            ),
        )
        self.arm_wrapper.set_buffer_capacity(
            max(self.arm_wrapper.buffer_capacity, state_capacity)
        )
        self.gripper_wrapper.set_buffer_capacity(
            max(self.gripper_wrapper.buffer_capacity, state_capacity)
        )

        if "camera" in configured_capacities:
            default_image_capacity = max(1, int(configured_capacities["camera"]))
        else:
            default_image_capacity = max(
                [1, *self._image_history_lengths.values()]
            )

        for cam in self.cameras:
            requested_capacity = max(
                self._image_history_lengths.get(cam.hw_name, 1),
                int(configured_capacities.get(cam.hw_name, default_image_capacity)),
            )
            cam.set_buffer_capacity(max(cam.buffer_capacity, requested_capacity))

    def _decode_image(self, img: Any) -> np.ndarray:
        if isinstance(img, np.ndarray):
            return np.ascontiguousarray(img)
        if isinstance(img, dict) and "rgb_data" in img:
            h = int(img["height"])
            w = int(img["width"])
            c = int(img.get("channels", 3))
            image_array = np.frombuffer(img["rgb_data"], dtype=np.uint8)
            return image_array.reshape((h, w, c)).copy()
        if isinstance(img, list):
            return np.ascontiguousarray(np.asarray(img, dtype=np.uint8))
        raise ValueError("Unsupported image format in observation.")

    def _prepare_xvla_image(self, image: np.ndarray) -> torch.Tensor:
        img = torch.from_numpy(np.ascontiguousarray(image)).permute(2, 0, 1)
        if img.dtype != torch.float32:
            img = img.float()
        if img.max() > 1.0:
            img = img / 255.0
        img = TF.resize(img, [256, 256], antialias=True)
        img = TF.normalize(img, mean=IMAGENET_MEAN, std=IMAGENET_STD)
        return img

    def _tokenize_task(self) -> dict[str, torch.Tensor]:
        tokens = self._tokenizer(
            self.task,
            max_length=self._tokenizer_max_length,
            padding="max_length",
            truncation=True,
            return_tensors="pt",
        )
        return {
            "observation.language.tokens": tokens["input_ids"].to(self._device),
            "observation.language.attention_mask": tokens["attention_mask"].to(self._device),
        }

    def _cast_obs_floats_inplace(self, obs: dict[str, Any]) -> None:
        for key, value in list(obs.items()):
            if torch.is_tensor(value) and torch.is_floating_point(value):
                obs[key] = value.to(device=self._device, dtype=self._model_float_dtype)

    def _images_to_tensor(self, images: List[np.ndarray]) -> torch.Tensor:
        stacked = torch.stack(
            [self._prepare_xvla_image(image) for image in images],
            dim=0,
        )
        return stacked.unsqueeze(0).to(self._device)

    def _get_recent_samples(
        self, wrapper: IRL_HardwareDataWrapper, count: int
    ) -> List[Any]:
        if wrapper.buffered_length == 0:
            wrapper.update_buffer()
        return wrapper.get_recent(
            count,
            pad_with_oldest=self.cfg.pad_history_with_oldest,
        )

    def _map_images_to_observation_keys(self, images: Dict[str, Any]) -> Dict[str, Any]:
        alias_groups: list[list[str]] = [
            ["right_cam", "zed_right", "right"],
            ["wrist_cam", "zed_wrist", "wrist"],
        ]
        preferred_by_key = {
            "observation.images.image": alias_groups[0],
            "observation.images.image2": alias_groups[1],
        }

        mapped: Dict[str, Any] = {}
        used_sources: set[str] = set()
        for obs_key in self._expected_image_keys:
            source = None
            for candidate in preferred_by_key.get(obs_key, []):
                if candidate in images and candidate not in used_sources:
                    source = candidate
                    break

            if source is None and obs_key.startswith("observation.images."):
                raw_key = obs_key.replace("observation.images.", "", 1)
                if raw_key in images and raw_key not in used_sources:
                    source = raw_key

            if source is None:
                for candidate in images:
                    if candidate not in used_sources:
                        source = candidate
                        break

            if source is None:
                raise ValueError(
                    f"Missing image source for {obs_key}. Available images: {list(images.keys())}"
                )

            mapped[obs_key] = images[source]
            used_sources.add(source)

        return mapped

    def _build_state_vector(self) -> np.ndarray:
        arm_states = self._get_recent_samples(self.arm_wrapper, self._state_history_length)
        grip_states = self._get_recent_samples(
            self.gripper_wrapper, self._state_history_length
        )

        state_history: list[np.ndarray] = []
        for arm_state, grip_state in zip(arm_states, grip_states):
            q = None
            if isinstance(arm_state, dict):
                if "q" in arm_state:
                    q = np.asarray(arm_state["q"], dtype=np.float32).reshape(-1)
                elif "joint_state" in arm_state:
                    q = np.asarray(arm_state["joint_state"], dtype=np.float32).reshape(-1)
            if q is None or q.size != 7:
                raise ValueError("Arm state missing valid joint positions.")

            gripper_val = None
            if isinstance(grip_state, dict):
                if "width" in grip_state:
                    gripper_val = float(grip_state["width"])
                elif "position" in grip_state:
                    gripper_val = float(grip_state["position"])
                elif "gripper" in grip_state:
                    gripper_arr = np.asarray(grip_state["gripper"], dtype=np.float32).reshape(-1)
                    if gripper_arr.size > 0:
                        gripper_val = float(gripper_arr[0])
            if gripper_val is None:
                raise ValueError("Gripper state missing value.")

            state_history.append(
                np.concatenate([q, np.asarray([gripper_val], dtype=np.float32)])
            )

        state = np.stack(state_history, axis=0)
        if self._expected_state_dim is not None and state.shape[-1] != self._expected_state_dim:
            if state.shape[-1] > self._expected_state_dim:
                pyzlc.error(
                    f"State dim {state.shape[-1]} larger than expected {self._expected_state_dim}; truncating."
                )
                state = state[..., : self._expected_state_dim]
            else:
                pad = self._expected_state_dim - state.shape[-1]
                pyzlc.error(
                    f"State dim {state.shape[-1]} smaller than expected {self._expected_state_dim}; padding zeros."
                )
                state = np.pad(state, ((0, 0), (0, pad)), mode="constant")

        if self._state_history_length == 1 and self._expected_state_history_length <= 1:
            return state[0]
        return state

    def _build_force_torque_vector(self) -> Optional[np.ndarray]:
        arm_samples = self._get_recent_samples(self.arm_wrapper, 1)
        grip_samples = self._get_recent_samples(self.gripper_wrapper, 1)
        if not arm_samples or not grip_samples:
            return None
        arm_state = arm_samples[-1]
        grip_state = grip_samples[-1]
        if not isinstance(arm_state, dict) or not isinstance(grip_state, dict):
            return None

        joint_torque = arm_state.get("tau_ext_hat_filtered")
        external_wrench = arm_state.get("O_F_ext_hat_K")
        gripper_current = grip_state.get("current")
        if joint_torque is None or external_wrench is None or gripper_current is None:
            return None

        tau = np.asarray(joint_torque, dtype=np.float32).reshape(-1)
        wrench = np.asarray(external_wrench, dtype=np.float32).reshape(-1)
        current = np.asarray([gripper_current], dtype=np.float32).reshape(-1)
        if tau.size != 7 or wrench.size != 6 or current.size != 1:
            raise ValueError(
                "Invalid force_torque source sizes: "
                f"tau_ext_hat_filtered={tau.size}, O_F_ext_hat_K={wrench.size}, current={current.size}"
            )
        return np.concatenate([tau, wrench, current]).astype(np.float32, copy=False)

    def _build_images(self) -> Dict[str, Any]:
        images: Dict[str, Any] = {}
        for cam in self.cameras:
            frames = self._get_recent_samples(
                cam, self._image_history_lengths.get(cam.hw_name, 1)
            )
            if not frames:
                continue
            images[cam.hw_name] = []
            for frame in frames:
                if isinstance(frame, np.ndarray):
                    h, w, c = frame.shape
                    images[cam.hw_name].append(
                        {
                            "height": int(h),
                            "width": int(w),
                            "channels": int(c),
                            "rgb_data": frame.tobytes(),
                        }
                    )
                else:
                    images[cam.hw_name].append(frame)
        return images

    def _build_observation(self) -> Dict[str, Any]:
        state_vec = self._build_state_vector()
        state = torch.from_numpy(state_vec)
        if state.ndim == 1:
            state = state.unsqueeze(0)
        else:
            state = state.unsqueeze(0)
        state = state.to(self._device)
        images = self._build_images()
        if not isinstance(images, dict) or not images:
            raise ValueError("No camera frames available for inference.")

        observation: Dict[str, Any] = {
            "observation.state": state,
            "observation.domain_id": torch.tensor([0], device=self._device, dtype=torch.long),
            **self._tokenize_task(),
        }

        force_torque = self._build_force_torque_vector()
        if force_torque is not None:
            observation["observation.force_torque"] = (
                torch.from_numpy(force_torque).unsqueeze(0).to(self._device)
            )

        mapped_images = self._map_images_to_observation_keys(images)
        for obs_key, cam_img in mapped_images.items():
            frame_history = cam_img if isinstance(cam_img, list) else [cam_img]
            rgb_history = []
            for frame in frame_history:
                rgb = self._decode_image(frame)
                if rgb.ndim != 3 or rgb.shape[2] != 3:
                    raise ValueError(
                        f"Expected HWC image with 3 channels for {obs_key}, got {rgb.shape}"
                    )
                rgb_history.append(rgb)
            observation[obs_key] = self._images_to_tensor(rgb_history)

        self._cast_obs_floats_inplace(observation)
        return observation

    def _predict_action_chunk(self, observation: Dict[str, Any]) -> torch.Tensor:
        # if hasattr(self.policy, "predict_action_chunk"):
        #     action_chunk = self.policy.predict_action_chunk(observation)
        # else:
        #     action_chunk = self.policy.select_action(observation)
        # curr_time = time.perf_counter()
        action_chunk = self.policy.select_action(observation)
        # end_time = time.perf_counter()
        # elapsed = end_time - curr_time
        # print(f"select action step took {elapsed:.3f} seconds")
        
        if action_chunk.ndim == 1:
            action_chunk = action_chunk.unsqueeze(0).unsqueeze(0)
        elif action_chunk.ndim == 2:
            action_chunk = action_chunk.unsqueeze(1)
        elif action_chunk.ndim != 3:
            raise RuntimeError(
                "Expected action chunk with shape (B, T, D), (B, D), or (D,), "
                f"got {tuple(action_chunk.shape)}"
            )
        return action_chunk

    def _postprocess_action_chunk(self, action_chunk: torch.Tensor) -> np.ndarray:
        action_chunk = action_chunk.float()
        if action_chunk.shape[-1] < 8:
            raise ValueError(f"Expected action dim >= 8, got {action_chunk.shape[-1]}")

        action_mean = self._action_mean[:8].to(action_chunk.device)
        action_std = self._action_std[:8].to(action_chunk.device)
        post_action_chunk = action_chunk[..., :8] * action_std + action_mean
        return post_action_chunk.detach().cpu().numpy()

    def _check_startup_image(self) -> None:
        if not self.cfg.save_startup_images or self._startup_images_saved:
            return

        images = self._build_images()
        if not images:
            pyzlc.error("Startup image check skipped: no camera frames available.")
            return

        try:
            mapped_images = self._map_images_to_observation_keys(images)
        except Exception as exc:
            pyzlc.error(f"Startup image check failed during image mapping: {exc}")
            return

        self._debug_image_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        saved_images: list[Path] = []
        for obs_key in sorted(mapped_images.keys()):
            debug_name = obs_key.replace("observation.images.", "", 1)
            try:
                frame_data = mapped_images[obs_key]
                frame = frame_data[-1] if isinstance(frame_data, list) else frame_data
                rgb = self._decode_image(frame)
            except Exception as exc:
                pyzlc.error(f"Startup image check failed for {obs_key}: {exc}")
                continue

            image_path = (self._debug_image_dir / f"{timestamp}_{debug_name}.png").resolve()
            image_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            if not cv2.imwrite(str(image_path), image_bgr):
                pyzlc.error(f"Failed to save startup image check to {image_path}")
                continue
            saved_images.append(image_path)

        if not saved_images:
            pyzlc.error("Startup image check failed: no camera images were saved.")
            return

        pyzlc.info(
            "Inspect startup images to verify the inference camera inputs: "
            + ", ".join(str(path) for path in saved_images)
        )
        self._startup_images_saved = True

    def _append_action_observation_log(self, lines: list[str]) -> None:
        with self._action_observation_log_path.open("a", encoding="utf-8") as log_file:
            for line in lines:
                log_file.write(f"{line}\n")

    def _print_buffer_lengths(self, prefix: str) -> None:
        details = [
            (
                f"{collector.hw_name} ({collector.hw_type}): "
                f"{collector.buffered_length}/{collector.buffer_capacity}"
            )
            for collector in self.data_collectors
        ]
        print(f"{prefix} | " + " | ".join(details), flush=True)

    def _start_infering(self) -> None:
        self.control_pair.reset_action()
        for collector in self.data_collectors:
            collector.start_buffering(clear_existing=True)
        self._print_buffer_lengths("Buffer lengths after start_buffering")
        self._last_buffer_debug_print_ts = time.perf_counter()
        if hasattr(self.policy, "reset"):
            self.policy.reset()
        self._check_startup_image()
        self._episode_action_observation_log_lines = []
        self._append_action_observation_log(
            [
                "",
                f"=== Episode started at {time.strftime('%Y-%m-%d %H:%M:%S')} ===",
            ]
        )
        super()._start_infering()

    def _infer_step(self) -> None:
        start_time = time.perf_counter()
        # if start_time - self._last_buffer_debug_print_ts >= 1.0:
        #     self._print_buffer_lengths("Buffer lengths")
        #     self._last_buffer_debug_print_ts = start_time

        try:
            observation = self._build_observation()
            with torch.inference_mode():
                raw_action_chunk = self._predict_action_chunk(observation)
            post_action_chunk = self._postprocess_action_chunk(raw_action_chunk)

            self._append_action_observation_log(
                [
                    "Running policy inference step...",
                    f"observation keys: {sorted(observation.keys())}",
                    f"raw action chunk shape: {tuple(raw_action_chunk.shape)}",
                    f"post action chunk shape: {tuple(post_action_chunk.shape)}",
                    f"post action chunk: {post_action_chunk}",
                ]
            )
            self.control_pair.update_action_chunk(post_action_chunk)
        except Exception as exc:
            pyzlc.error(f"Policy inference step failed: {exc}")

        elapsed = time.perf_counter() - start_time
        sleep_time = max(0.0, (1.0 / self.fps) - elapsed)
        if sleep_time > 0.001:
            time.sleep(sleep_time)

    def _save_episode(self) -> None:
        self._episode_action_observation_log_lines = []
        self._stop_infering()
        self._ui_console.log("Episode saved.")

    def _discard_infering(self) -> None:
        self._episode_action_observation_log_lines = []
        self._stop_infering()
        self._ui_console.log("Episode discarded.")

    def _stop_infering(self) -> None:
        for collector in self.data_collectors:
            collector.stop_buffering()
        super()._stop_infering()

    def _reset_arm(self) -> None:
        self._ui_console.log("Resetting robot arm position...")
        try:
            self.control_pair.go_home()
            time.sleep(3)
            self._ui_console.log("Robot arm reset to home position.")
        except Exception as exc:
            self._ui_console.log(f"Failed to reset arm: {exc}")
