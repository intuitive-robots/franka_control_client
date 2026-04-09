from __future__ import annotations

import json
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Any

import cv2
import draccus
import lerobot
import numpy as np
import pyzlc
import torch
from lerobot.configs.train import TrainPipelineConfig
from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.utils.utils import get_safe_torch_device
print(lerobot.__file__)
from .policy_inference_manager import PolicyInferenceManager
from .irl_wrapper import (
    IRL_HardwareDataWrapper,
    ImageDataWrapper,
    PandaArmDataWrapper,
    PandaGripperDataWrapper,
    RobotiqGripperDataWrapper,
)
from ..control_pair.policy_panda_control_pair_chunk import PolicyPandaControlPair

# Try to import load_dataset_meta helper function
try:
    from utils.eval_utils import load_dataset_meta
except ImportError:
    load_dataset_meta = None
# change denoising step in train_config

@dataclass
class LeRobotPolicyInferenceConfig:
    checkpoint_path: str
    task: str
    fps: int = 30
    device: str = "cuda"
    policy_dtype: Optional[str] = None
    dataset_path: Optional[str] = None
    save_startup_images: bool = False


class LeRobotPolicyInference(PolicyInferenceManager):
    """
    Policy inference loop that directly loads and evaluates a LeRobot policy.

    Observation format: {"state": [...], "images": {...}, "task": "..."}
    Action semantics: [q0..q6, gripper].
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
        self._resolved_checkpoint_path = self._resolve_checkpoint_path(cfg.checkpoint_path)

        self.cameras: List[ImageDataWrapper] = []
        self.arm_wrapper: Optional[PandaArmDataWrapper] = None
        self.gripper_wrapper: Optional[
            IRL_HardwareDataWrapper
        ] = None
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

        # Load policy stack directly
        self.train_cfg = self._load_train_cfg()
        pyzlc.info(f"Loaded train config: {self.train_cfg}")
        self.policy, self.preprocessor, self.postprocessor = self._load_policy_stack()

        self._expected_image_shapes = self._get_expected_image_shapes()
        self._expected_state_dim = self._get_expected_state_dim()

        # Auto-hook control start/stop to inference events.
        self.register_start_infering_event(self.control_pair.start_control_pair)
        self.register_stop_infering_event(self.control_pair.stop_control_pair)
        self._debug_image_dir = Path("debug/inference_start_images")
        self._startup_images_saved = False
        self._action_observation_log_path = Path("action_observation.txt")
        self._episode_action_observation_log_lines: list[str] = []

    def _load_train_cfg(self) -> TrainPipelineConfig:
        """Load training config from checkpoint."""
        cli_args = [
            f"--policy.pretrained_path={self._resolved_checkpoint_path}",
            f"--policy.device={self.cfg.device}",
            "--dataset.image_transforms.enable=false",
        ]
        if self.cfg.dataset_path:
            cli_args.append(f"--dataset.root={self.cfg.dataset_path}")
        if self.cfg.policy_dtype:
            cli_args.append(f"--policy.dtype={self.cfg.policy_dtype}")

        try:
            train_cfg = TrainPipelineConfig.from_pretrained(
                pretrained_name_or_path=self._resolved_checkpoint_path,
                cli_args=cli_args,
            )
        except Exception as exc:
            if not self._is_legacy_policy_type_error(exc):
                raise
            pyzlc.info(
                "Legacy train_config.json detected without policy.type; applying compatibility shim."
            )
            train_cfg = self._load_train_cfg_with_policy_type_fallback(cli_args)

        if any("empty_camera" in key for key in train_cfg.policy.input_features):
            train_cfg.policy.input_features = {
                "observation.images.image": PolicyFeature(
                    type=FeatureType.VISUAL, shape=(3, 256, 256)
                ),
                "observation.images.image2": PolicyFeature(
                    type=FeatureType.VISUAL, shape=(3, 256, 256)
                ),
                "observation.images.image3": PolicyFeature(
                    type=FeatureType.VISUAL, shape=(3, 256, 256)
                ),

                "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(8,)),
            }
            train_cfg.policy.num_image_views = 3
            train_cfg.policy.empty_cameras = 0

        return train_cfg

    def _resolve_checkpoint_path(self, checkpoint_path: str) -> str:
        """Resolve local checkpoint paths early so invalid filesystem paths don't look like Hub ids."""
        candidate = Path(checkpoint_path).expanduser()

        if candidate.exists():
            return str(candidate.resolve())

        if candidate.is_absolute():
            suggestions = self._find_nearby_pretrained_model_dirs(candidate)
            suggestion_text = ""
            if suggestions:
                suggestion_text = " Nearby checkpoint candidates:\n" + "\n".join(
                    f" - {path}" for path in suggestions
                )
            raise FileNotFoundError(
                f"Checkpoint path does not exist: {candidate}{suggestion_text}"
            )

        return checkpoint_path

    def _find_nearby_pretrained_model_dirs(self, checkpoint_path: Path) -> list[str]:
        """Search a few nearby levels for local pretrained_model directories to aid debugging."""
        search_roots: list[Path] = []
        for parent in checkpoint_path.parents:
            if parent == parent.parent:
                break
            if parent.exists():
                search_roots.append(parent)
            if len(search_roots) == 3:
                break

        matches: list[str] = []
        seen: set[Path] = set()
        for root in search_roots:
            try:
                for path in sorted(root.glob("**/pretrained_model")):
                    if path in seen or not path.is_dir():
                        continue
                    seen.add(path)
                    matches.append(str(path.resolve()))
                    if len(matches) == 5:
                        return matches
            except OSError:
                continue
        return matches

    def _load_train_cfg_with_policy_type_fallback(
        self, cli_args: list[str]
    ) -> TrainPipelineConfig:
        """Patch legacy train configs that predate draccus ChoiceRegistry type fields."""
        checkpoint_path = Path(self._resolved_checkpoint_path)
        train_config_path = checkpoint_path / "train_config.json"

        with train_config_path.open() as f:
            train_config = json.load(f)

        policy_config = train_config.get("policy")
        if not isinstance(policy_config, dict):
            raise ValueError(f"Expected dict policy config in {train_config_path}, got {type(policy_config)}")

        policy_config.setdefault(
            "type",
            self._infer_policy_type_from_checkpoint(checkpoint_path, policy_config),
        )
        if train_config.get("use_policy_training_preset", True):
            train_config["optimizer"] = None
            train_config["scheduler"] = None

        with tempfile.NamedTemporaryFile("w+", suffix=".json", delete=False) as tmp:
            json.dump(train_config, tmp)
            tmp.flush()
            tmp_path = tmp.name

        try:
            with draccus.config_type("json"):
                return draccus.parse(TrainPipelineConfig, tmp_path, args=cli_args)
        finally:
            Path(tmp_path).unlink(missing_ok=True)

    def _infer_policy_type_from_checkpoint(
        self, checkpoint_path: Path, train_policy_config: dict[str, Any]
    ) -> str:
        """Infer missing policy.type from checkpoint metadata."""
        config_json_path = checkpoint_path / "config.json"
        if config_json_path.is_file():
            with config_json_path.open() as f:
                policy_config = json.load(f)
            policy_type = policy_config.get("type")
            if isinstance(policy_type, str) and policy_type:
                return policy_type

        repo_id = train_policy_config.get("repo_id")
        if isinstance(repo_id, str) and "xvla" in repo_id.lower():
            return "xvla"

        if "florence_config" in train_policy_config:
            return "xvla"

        raise ValueError(
            "Could not infer missing policy.type from checkpoint metadata. "
            f"Checked {config_json_path} and known XVLA markers."
        )

    def _is_legacy_policy_type_error(self, exc: Exception) -> bool:
        message = str(exc)
        return "Expected a dict with a 'type' key" in message and "`policy`" in message

    def _load_dataset_meta(self) -> Any:
        """Load dataset metadata helper."""
        try:
            return load_dataset_meta(self.train_cfg)
        except Exception as exc:
            pyzlc.info(f"load_dataset_meta helper unavailable, proceeding without ds_meta: {exc}")
            return None

    def _load_policy_stack(self):
        """Load policy, preprocessor, and postprocessor."""
        ds_meta = self._load_dataset_meta()
        pyzlc.info(f"Loaded dataset meta: {ds_meta}")
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
        # pyzlc.info(f"stats json: {getattr(ds_meta, 'stats', None)}")
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=self.train_cfg.policy,
            pretrained_path=self.train_cfg.policy.pretrained_path,
            preprocessor_overrides={"device_processor": {"device": device.type}},
        )
        print("debug: preprocessor and postprocessor loaded:", preprocessor, postprocessor)
        return policy, preprocessor, postprocessor

    def _decode_image(self, img: Any) -> np.ndarray:
        """Decode image from various formats."""
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

    def _get_expected_image_shapes(self) -> dict[str, tuple[int, int, int]]:
        """Get expected image shapes from policy config."""
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
        """Get expected state dimension from policy config."""
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
        """Resize image to expected shape."""
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

    def _image_to_tensor(self, image: np.ndarray) -> torch.Tensor:
        """Build a single observation image as [B, C, H, W] for XVLA _prepare_images."""
        rgb = torch.from_numpy(image.copy()).float().permute(2, 0, 1) / 255.0
        return rgb.unsqueeze(0)

    def _map_images_to_observation_keys(self, images: Dict[str, Any]) -> Dict[str, Any]:
        """Map raw camera names onto the observation image keys expected by the policy."""
        expected_image_keys = list(getattr(self.train_cfg.policy, "image_features", {}).keys())
        if not expected_image_keys:
            expected_image_keys = list(self._expected_image_shapes.keys())

        alias_groups: list[list[str]] = [
            ["right_cam", "zed_right"],
            ["wrist_cam", "zed_wrist"],
            ["left_cam", "zed_left"],
        ]
        preferred_by_key = {
            "observation.images.image": alias_groups[0],
            "observation.images.image2": alias_groups[1],
            "observation.images.image3": alias_groups[2],
        }

        mapped: Dict[str, Any] = {}
        used_sources: set[str] = set()

        for obs_key in expected_image_keys:
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
                for alias_group in alias_groups:
                    for candidate in alias_group:
                        if candidate in images and candidate not in used_sources:
                            source = candidate
                            break
                    if source is not None:
                        break

            if source is None:
                raise ValueError(
                    f"Missing image source for {obs_key}. Available images: {list(images.keys())}"
                )

            mapped[obs_key] = images[source]
            used_sources.add(source)

        return mapped

    def _build_observation(self) -> Dict[str, Any]:
        """Build observation dict from hardware data."""
        state_vec = self._build_state_vector()
        images = self._build_images()
        
        state = np.asarray(state_vec, dtype=np.float32)
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

        force_torque = self._build_force_torque_vector()
        if force_torque is not None:
            observation["observation.force_torque"] = torch.from_numpy(force_torque[None, :])

        if not isinstance(images, dict):
            raise ValueError("'images' must be a dict keyed by camera name.")

        mapped = self._map_images_to_observation_keys(images)

        for obs_key, cam_img in mapped.items():
            rgb = self._decode_image(cam_img)
            if rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError(f"Expected HWC image with 3 channels for {obs_key}, got shape {rgb.shape}")
            expected_shape = self._expected_image_shapes.get(obs_key)
            if expected_shape is not None:
                rgb = self._resize_image(rgb, expected_shape)
            observation[obs_key] = self._image_to_tensor(np.ascontiguousarray(rgb))

        observation["task"] = self.task

        return observation

    def _build_state_vector(self) -> np.ndarray:
        arm_state = self.arm_wrapper.capture_step()
        q = None
        if isinstance(arm_state, dict):
            if "q" in arm_state:
                q = np.asarray(arm_state["q"], dtype=np.float32).reshape(-1)
            elif "joint_state" in arm_state:
                q = np.asarray(arm_state["joint_state"], dtype=np.float32).reshape(-1)
        if q is None or q.size != 7:
            raise ValueError("Arm state missing valid joint positions.")

        grip_state = self.gripper_wrapper.capture_step()
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

        return np.concatenate([q, np.asarray([gripper_val], dtype=np.float32)])

    def _build_force_torque_vector(self) -> Optional[np.ndarray]:
        """Build XVLA force input as [tau_ext_hat_filtered(7), O_F_ext_hat_K(6), gripper_current(1)]."""
        arm_state = self.arm_wrapper.capture_step()
        grip_state = self.gripper_wrapper.capture_step()

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
            frame = cam.capture_step()
            if frame is None:
                continue
            if isinstance(frame, np.ndarray):
                h, w, c = frame.shape
                images[cam.hw_name] = {
                    "height": int(h),
                    "width": int(w),
                    "channels": int(c),
                    "rgb_data": frame.tobytes(),
                }
            else:
                images[cam.hw_name] = frame
        return images

    def _check_startup_image(self) -> None:
        """Capture and persist one startup image per camera for inspection."""
        if not self.cfg.save_startup_images:
            return
        if self._startup_images_saved:
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
                rgb = self._decode_image(mapped_images[obs_key])
            except Exception as exc:
                pyzlc.error(f"Startup image check failed for {obs_key}: {exc}")
                continue

            if rgb.ndim != 3 or rgb.shape[2] != 3:
                pyzlc.error(
                    f"Startup image check skipped for {obs_key}: unexpected shape {rgb.shape}"
                )
                continue

            image_path = (self._debug_image_dir / f"{timestamp}_{debug_name}.png").resolve()
            image_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            if not cv2.imwrite(str(image_path), image_bgr):
                pyzlc.error(f"Failed to save startup image check to {image_path}")
                continue

            saved_images.append(image_path)
            pyzlc.info(
                f"Startup image check saved {obs_key} frame to {image_path} with shape {rgb.shape}"
            )

        if not saved_images:
            pyzlc.error("Startup image check failed: no camera images were saved.")
            return

        pyzlc.info(
            "Inspect startup images to verify the inference camera inputs: "
            + ", ".join(str(path) for path in saved_images)
        )
        self._startup_images_saved = True

    def _start_infering(self) -> None:
        # Reset action state for new episode
        self.control_pair.reset_action()
        self._check_startup_image()
        self._episode_action_observation_log_lines = []
        self._append_action_observation_log(
            [
                "",
                f"=== Episode started at {time.strftime('%Y-%m-%d %H:%M:%S')} ===",
            ]
        )
        # self.last_timestamp = None
        super()._start_infering()

    def _append_action_observation_log(self, lines: list[str]) -> None:
        with self._action_observation_log_path.open("a", encoding="utf-8") as log_file:
            for line in lines:
                log_file.write(f"{line}\n")

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
                if str(k).startswith("observation.images.") and hasattr(v, "shape")
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
            # action_chunk = self.policy.predict_action_chunk(observation)
            action_observation_log_lines = [
                "Running policy inference step...",
                f"observation: {observation}",
            ]
            action_chunk = self.policy.select_action(observation)
            action_observation_log_lines.append(
                f"Raw action chunk from policy: {action_chunk}"
            )
            print(len(action_chunk), action_chunk.shape)

        if action_chunk.ndim == 2:
            action_chunk = action_chunk.unsqueeze(1)
        elif action_chunk.ndim != 3:
            raise RuntimeError(
                f"Expected action_chunk to have shape (B, T, D) or (B, D), got {tuple(action_chunk.shape)}"
            )

        batch_size, chunk_size, action_dim = action_chunk.shape
        action_dim_expected = 8  # 7 joints + 1 gripper
        post_action_chunk = torch.zeros((batch_size, chunk_size, action_dim_expected), dtype=torch.float32)
        for chunk_idx in range(chunk_size):
            single_action = action_chunk[:, chunk_idx, :]
            single_action = single_action[:, :8]
            processed_action = self.postprocessor(single_action)
            pyzlc.info(f"raw action chunk {chunk_idx}: {single_action.float().cpu().numpy()}")
            pyzlc.info(f"Processed action chunk {chunk_idx}: {processed_action.float().cpu().numpy()}")
            post_action_chunk[:, chunk_idx, :] = processed_action
        post_action_chunk = post_action_chunk.float().cpu().numpy()
        action_observation_log_lines.append(
            f"Postprocessed action chunk: {post_action_chunk}"
        )
        self._append_action_observation_log(action_observation_log_lines)

        try:
            #single_action
            # self.control_pair.update_action(action_vec)
            #action chunk
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

    def _save_episode(self) -> None:
        self._episode_action_observation_log_lines = []
        self._stop_infering()
        self._ui_console.log("Episode saved.")

    def _discard_infering(self) -> None:
        self._episode_action_observation_log_lines = []
        self._stop_infering()
        self._ui_console.log("Episode discarded.")

    def _stop_infering(self) -> None:
        super()._stop_infering()

    def _reset_arm(self) -> None:
        """Reset the robot arm to a safe/home position.
        
        Called only when in WAITING state (control pair is not running).
        """
        self._ui_console.log("Resetting robot arm position...")
        try:
            # Reset the arm to home position
            self.control_pair.go_home()
            time.sleep(3)  # Wait for the arm to reach the home position
            self._ui_console.log("Robot arm reset to home position.")
        except Exception as exc:
            self._ui_console.log(f"Failed to reset arm: {exc}")
