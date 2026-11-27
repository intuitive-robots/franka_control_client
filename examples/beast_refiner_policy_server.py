"""Serve a BEAST Refiner v2 checkpoint to the Franka inference GUI.

This is a streaming policy: BEAST's fast System 1 refines the current System-2
plan once per observation. The server therefore returns one absolute
end-effector target per request and advertises ``query_every_step`` so the GUI calls it at
the checkpoint's training FPS.

Example (policy PC)::

    conda activate beast_refiner
    cd /home/jjiang/jing/franka_control_client
    PYTHONPATH=src python examples/beast_refiner_policy_server.py \\
      --ckpt /home/jjiang/utphd/models/conveyor-kit-R5-v2-joint-warm-lam0-c24 \\
      --xvla-checkpoint /home/jjiang/.cache/huggingface/hub/models--lerobot--xvla-base/snapshots/cdb7964e4fe842935d671bfab5a5ebe00a96648c \\
      --task "put red cube in bowl" \\
      --port 8767

The shipping ``conveyor_cube`` checkpoint is a 20 Hz Cartesian end-effector
policy with two camera inputs. Its actions are absolute
``[x, y, z, qx, qy, qz, qw, gripper_closedness]`` targets. Its saved
preprocessor maps ``observation.images.static_cam`` and
``observation.images.wrist_cam`` to the internal xVLA image names.
"""

from __future__ import annotations

import argparse
import importlib
import json
import logging
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import torch

_HERE = Path(__file__).resolve()
_CLIENT_SRC = _HERE.parents[1] / "src"
if str(_CLIENT_SRC) not in sys.path:
    sys.path.insert(0, str(_CLIENT_SRC))


def _alias_numpy2_pickles() -> None:
    """Let a NumPy 1.x model env read observations pickled by NumPy 2.x."""
    if int(np.__version__.split(".")[0]) >= 2:
        return
    import numpy.core

    for sub in (
        "",
        ".multiarray",
        ".numeric",
        ".umath",
        "._multiarray_umath",
        ".fromnumeric",
        ".records",
        "._internal",
        "._dtype",
        "._methods",
    ):
        try:
            sys.modules.setdefault("numpy._core" + sub, importlib.import_module("numpy.core" + sub))
        except ImportError:
            pass


_alias_numpy2_pickles()

from franka_control_client.inference_gui.policy_server import (  # noqa: E402
    CARTESIAN_ACTION_SPACE,
    BasePolicy,
    PolicyServer,
)

LOG = logging.getLogger("beast_refiner_server")
TIMING_LOG_EVERY = 100  # steps between per-stage latency summaries

# These fields originated on the checkpoint-producing branch but are
# training-loss switches only. Dropping them keeps the server compatible with
# Beast Refiner installations that predate the backward-compatible config
# aliases; none affects the inference module graph or state dict.
COMPAT_TRAINING_ONLY_CONFIG_KEYS = {
    "sys1_l1_loss",
    "sys1_cp_loss_weighting",
    "plan_guidance_native_residual",
}


def _compatible_checkpoint(ckpt: Path, out: Path) -> Path:
    """Make a symlink view with branch-only training config fields removed."""
    config = json.loads((ckpt / "config.json").read_text(encoding="utf-8"))
    dropped = sorted(COMPAT_TRAINING_ONLY_CONFIG_KEYS.intersection(config))
    for key in dropped:
        del config[key]
    for source in ckpt.iterdir():
        if source.name != "config.json":
            (out / source.name).symlink_to(source.resolve())
    (out / "config.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    if dropped:
        LOG.info("Ignoring branch-only training config fields: %s", ", ".join(dropped))
    return out


def _checkpoint_camera_keys(ckpt: Path, config: Any) -> list[str]:
    """Return the raw camera keys expected before the saved rename processor."""
    path = ckpt / "policy_preprocessor.json"
    if path.exists():
        data = json.loads(path.read_text(encoding="utf-8"))
        for step in data.get("steps", []):
            if step.get("registry_name") == "rename_observations_processor":
                rename_map = step.get("config", {}).get("rename_map", {})
                keys = [key for key in rename_map if key.startswith("observation.images.")]
                if keys:
                    return keys
    return sorted(key for key in config.image_features if key.startswith("observation.images."))


def _resize_like_training(image: np.ndarray, hw: tuple[int, int]) -> np.ndarray:
    """Squash an HxWx3 frame to the trained size exactly as the dataset converter did."""
    if image.shape[:2] == tuple(hw):
        return image
    import cv2

    return cv2.resize(np.ascontiguousarray(image), (int(hw[1]), int(hw[0])), interpolation=cv2.INTER_AREA)


class BeastRefinerPolicy(BasePolicy):
    """Adapter from the GUI's pickle protocol to BEAST Refiner's LeRobot API."""

    action_space = CARTESIAN_ACTION_SPACE
    exec_steps = 1
    query_every_step = True

    def __init__(
        self,
        ckpt: str,
        device: str = "cuda",
        xvla_checkpoint: Optional[str] = None,
        task: str = "",
        camera_map: Optional[Dict[str, str]] = None,
        direct_spline_projection: bool = False,
        direct_spline_basis: int = 4,
        sys2_trigger_margin_ticks: int = 4,
        warmup: bool = True,
        action_diagnostics: bool = False,
        swap_rgb: bool = False,
        inference_mode: Optional[str] = None,
    ) -> None:
        # Importing this package registers the custom policy and processor
        # classes with LeRobot before config/model deserialization.
        import beast_refiner
        import beast_refiner.policies  # noqa: F401
        from beast_refiner.async_inference import AsyncInferenceConfig, AsyncPolicyRunner
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors

        self.ckpt = Path(ckpt).expanduser().resolve()
        if not (self.ckpt / "config.json").is_file():
            raise FileNotFoundError(f"{self.ckpt}/config.json not found; --ckpt must be a pretrained_model folder")
        self.device = device
        self.fixed_task = task.strip()
        self.task: Optional[str] = None
        self.camera_map = camera_map or {}
        self._step = 0
        self._timings: list[tuple[float, float, float, float]] = []
        self._closed = False
        self.action_diagnostics = bool(action_diagnostics)

        t0 = time.time()
        self._tmp = tempfile.TemporaryDirectory(prefix="beast_refiner_ckpt_")
        load_path = _compatible_checkpoint(self.ckpt, Path(self._tmp.name))
        cfg = PreTrainedConfig.from_pretrained(load_path)
        if cfg.type != "beast_xvla_refiner_v2":
            raise ValueError(f"expected a beast_xvla_refiner_v2 checkpoint, got {cfg.type!r}")
        cfg.device = device
        # Keep the checkpoint's own mode: a System-2-only run (train_mode=sys2)
        # has an untrained System 1, so forcing "both" would act on noise.
        cfg.inference_mode = inference_mode or cfg.inference_mode
        if cfg.inference_mode not in ("both", "sys2"):
            raise ValueError(f"inference_mode must be 'both' or 'sys2', got {cfg.inference_mode!r}")
        if cfg.inference_mode == "both" and getattr(cfg, "train_mode", "both") == "sys2":
            raise ValueError("this checkpoint trained System 2 only; run it with --inference-mode sys2")
        self.sys2_only = cfg.inference_mode == "sys2"
        if self.sys2_only:
            # System 2's coarse plan is an ordinary action chunk: the GUI
            # executes n_action_steps rows, then asks again (training replan rate).
            self.query_every_step = False
            self.exec_steps = int(cfg.n_action_steps)
        cfg.return_act_chunk = False
        if xvla_checkpoint:
            cfg.xvla_checkpoint = str(Path(xvla_checkpoint).expanduser().resolve()) if Path(
                xvla_checkpoint
            ).expanduser().exists() else xvla_checkpoint
        if direct_spline_projection:
            cfg.direct_spline_projection = True
            cfg.direct_spline_projection_num_basis = int(direct_spline_basis)

        self.fps = float(cfg.fps)
        if self.fps <= 0:
            raise ValueError(f"checkpoint has invalid fps={cfg.fps!r}")
        self.camera_keys = _checkpoint_camera_keys(self.ckpt, cfg)
        self.swap_rgb = bool(swap_rgb)
        # The training frames were squashed to this size with cv2 INTER_AREA
        # (data_collection_gui/irl_to_lerobot.py); nothing in the model resizes
        # for System 1, so live frames must arrive at exactly this resolution.
        image_shapes = {tuple(f.shape[-2:]) for f in cfg.image_features.values()}
        if len(image_shapes) != 1:
            raise ValueError(f"cameras have different trained resolutions: {image_shapes}")
        self.image_hw: tuple[int, int] = image_shapes.pop()

        policy_class = get_policy_class(cfg.type)
        LOG.info("Loading BEAST Refiner from %s ...", self.ckpt)
        self.policy = policy_class.from_pretrained(load_path, config=cfg)
        self.policy.to(device).eval()
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            self.policy.config,
            pretrained_path=str(load_path),
            preprocessor_overrides={"device_processor": {"device": device}},
        )

        self.runner: Any = None
        if self.sys2_only:
            self.policy.reset()
            if warmup:
                from beast_refiner.async_inference.benchmarking import build_dummy_batch

                LOG.info("Warming up System 2 on synthetic data ...")
                with torch.inference_mode():
                    dummy = build_dummy_batch(self.policy.config, torch.device(device))
                    for _ in range(2):
                        self.policy.predict_action_chunk(dummy)
                self.policy.reset()
        else:
            async_cfg = AsyncInferenceConfig(
                sys2_trigger_margin_ticks=int(sys2_trigger_margin_ticks),
                warmup_on_load=bool(warmup),
            )
            self.runner = AsyncPolicyRunner(self.policy, async_cfg)
            self.runner.start()
            self.runner.reset()

        # A few model imports configure logging globally.  Restore the two
        # loggers users need in order to see readiness and request failures.
        for name in ("beast_refiner_server", "policy_server"):
            logging.getLogger(name).disabled = False
        LOG.info(
            "BEAST Refiner ready in %.1f s: %.1f Hz, mode=%s (%s), cameras=%s, xvla=%s, "
            "spline_projection=%s, beast_refiner code=%s",
            time.time() - t0,
            self.fps,
            cfg.inference_mode,
            f"chunks of {self.exec_steps} rows" if self.sys2_only else "one action per observation",
            self.camera_keys,
            self.policy.config.xvla_checkpoint,
            bool(self.policy.config.direct_spline_projection),
            Path(beast_refiner.__file__).parent,
        )

    def metadata(self) -> Dict[str, Any]:
        cfg = self.policy.config
        return {
            "name": "beast_xvla_refiner_v2",
            "ckpt": str(self.ckpt),
            "xvla_checkpoint": str(cfg.xvla_checkpoint),
            "inference_mode": cfg.inference_mode,
            "cameras": self.camera_keys,
            "task": self.fixed_task or None,
            "chunk_size": int(cfg.chunk_size),
            "n_action_steps": int(cfg.n_action_steps),
            "history_len": int(cfg.history_len),
            "direct_spline_projection": bool(cfg.direct_spline_projection),
            "action_diagnostics": self.action_diagnostics,
        }

    def _postprocessed_plan(self, value: Any) -> Optional[np.ndarray]:
        if value is None:
            return None
        tensor = value.detach().clone()
        plan = self.postprocessor(tensor).detach().cpu().float().numpy()
        if plan.ndim == 3 and plan.shape[0] == 1:
            plan = plan[0]
        elif plan.ndim == 1:
            plan = plan[None, :]
        if plan.ndim != 2 or plan.shape[1] != 8:
            LOG.warning("Ignoring diagnostic plan with unexpected shape %s", plan.shape)
            return None
        plan[:, 7] = np.clip(plan[:, 7], 0.0, 1.0)
        return plan

    def inference_diagnostics(self) -> Dict[str, Any]:
        """Expose the active System-2 and System-1 plans for saved-rollout plots."""
        if not self.action_diagnostics:
            return {}
        sys2 = self._postprocessed_plan(getattr(self.policy, "_last_sys2_plan", None))
        sys1 = self._postprocessed_plan(getattr(self.policy, "_last_sys1_window", None))
        if self.runner is None:  # System-2-only: every call is a fresh plan
            sys1 = None
            source = self._step - 1
        else:
            active = getattr(self.runner.orchestrator, "_active", None)
            source = int(active.source_timestep) if active is not None else -1
        result: Dict[str, Any] = {"obs_timestep": self._step - 1, "sys2_source_timestep": source}
        if sys2 is not None:
            result["sys2_plan"] = sys2
        if sys1 is not None:
            result["sys1_plan"] = sys1
        return result

    def reset(self, task: Optional[str]) -> None:
        self.task = self.fixed_task or (task or "").strip()
        self._step = 0
        self._timings.clear()
        if self.runner is None:
            self.policy.reset()
        else:
            self.runner.reset()
        if not self.task:
            LOG.warning("Task prompt is empty; BEAST was trained with language conditioning.")
        LOG.info("Reset BEAST rollout: task=%r", self.task)

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            if self.runner is not None:
                self.runner.stop()
            self._tmp.cleanup()

    def _image(self, observation: Dict[str, Any], checkpoint_key: str) -> torch.Tensor:
        prefix = "observation.images."
        checkpoint_name = checkpoint_key[len(prefix) :] if checkpoint_key.startswith(prefix) else checkpoint_key
        sources = [checkpoint_key]
        sources.extend(
            f"observation.images.{gui_name}"
            for gui_name, ckpt_name in self.camera_map.items()
            if ckpt_name == checkpoint_name
        )
        image = next((observation[key] for key in sources if key in observation), None)
        if image is None:
            got = sorted(key for key in observation if key.startswith("observation.images."))
            raise KeyError(
                f"camera {checkpoint_key!r} is missing (received {got}); fix the GUI obs_key "
                "or pass --camera-map gui_name=checkpoint_name"
            )
        array = np.asarray(image)
        if array.ndim != 3 or array.shape[-1] != 3:
            raise ValueError(f"{checkpoint_key} must be an HxWx3 RGB image, got {array.shape}")
        if array.dtype != np.uint8:
            scale = 255.0 if np.issubdtype(array.dtype, np.floating) and array.size and array.max() <= 1 else 1.0
            array = np.clip(array * scale, 0, 255).astype(np.uint8)
        if self.swap_rgb:
            array = array[..., ::-1]
        array = _resize_like_training(array, self.image_hw)
        # Pickled camera frames may be backed by a read-only buffer.  PyTorch
        # tensors require writable NumPy storage even though inference only
        # reads the image, so make an owned writable copy here.
        return torch.from_numpy(np.array(array, copy=True, order="C")).permute(2, 0, 1).contiguous()

    def _batch(self, observation: Dict[str, Any]) -> Dict[str, Any]:
        state = np.asarray(observation["observation.state"], dtype=np.float32).reshape(-1)
        if state.shape != (8,):
            raise ValueError(
                "observation.state must be x,y,z,qx,qy,qz,qw + gripper [8], "
                f"got {state.shape}"
            )
        raw: Dict[str, Any] = {
            "observation.state": torch.from_numpy(state.copy()),
            "task": self.task or str(observation.get("task") or ""),
        }
        for key in self.camera_keys:
            raw[key] = self._image(observation, key)
        return self.preprocessor(raw)

    def _sync(self) -> None:
        if self.device.startswith("cuda"):
            torch.cuda.synchronize()

    def _log_timing(self, observation: Dict[str, Any]) -> None:
        """Every TIMING_LOG_EVERY steps, log where the per-step model time goes."""
        if self._step == 1:
            shapes = {
                key: tuple(np.asarray(observation[key]).shape)
                for key in observation
                if key.startswith("observation.images.")
            }
            LOG.info(
                "Incoming camera frames: %s -> resized to %s for the model%s",
                shapes,
                self.image_hw,
                " (red/blue swapped)" if self.swap_rgb else "",
            )
        # A System-2-only server is called once per chunk, so summarise sooner.
        every = max(1, TIMING_LOG_EVERY // self.exec_steps)
        if len(self._timings) < every:
            return
        t = np.asarray(self._timings) * 1000.0
        self._timings.clear()
        p50, p95 = np.percentile(t, 50, axis=0), np.percentile(t, 95, axis=0)
        LOG.info(
            "Call timing ms (p50/p95) over %d calls: preprocess %.1f/%.1f, %s %.1f/%.1f, "
            "postprocess %.1f/%.1f, total %.1f/%.1f (budget %.1f)",
            every,
            p50[0], p95[0],
            "system2 plan" if self.sys2_only else "system1",
            p50[1], p95[1], p50[2], p95[2], p50[3], p95[3],
            1000.0 * self.exec_steps / self.fps,
        )

    @torch.inference_mode()
    def infer(self, observation: Dict[str, Any]) -> np.ndarray:
        t0 = time.perf_counter()
        batch = self._batch(observation)
        self._sync()
        t1 = time.perf_counter()
        if self.runner is None:
            action = self.policy.predict_action_chunk(batch)  # [1, chunk_size, 8]
        else:
            action = self.runner.select_action(batch, obs_timestep=self._step)
        self._sync()
        t2 = time.perf_counter()
        self._step += 1
        action = self.postprocessor(action)
        output = action.detach().cpu().float().numpy().reshape(-1, 8)
        output[:, 7] = np.clip(output[:, 7], 0.0, 1.0)
        t3 = time.perf_counter()
        self._timings.append((t1 - t0, t2 - t1, t3 - t2, t3 - t0))
        self._log_timing(observation)
        return output


def _camera_map(value: str) -> Dict[str, str]:
    try:
        return dict(item.split("=", 1) for item in value.split(",") if item)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected gui_name=checkpoint_name pairs separated by commas") from exc


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--ckpt", required=True, help="BEAST pretrained_model/checkpoint directory")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--xvla-checkpoint", default=None, help="local xVLA base path or Hub id")
    parser.add_argument("--task", default="", help="fixed prompt; use the exact task from the training dataset")
    parser.add_argument(
        "--camera-map",
        type=_camera_map,
        default={},
        help="GUI suffix -> checkpoint suffix, e.g. external=static_cam,wrist=wrist_cam",
    )
    parser.add_argument("--direct-spline-projection", action="store_true", help="project the raw refiner head to a C0/C1 spline")
    parser.add_argument("--direct-spline-basis", type=int, default=4)
    parser.add_argument("--sys2-trigger-margin-ticks", type=int, default=4)
    parser.add_argument(
        "--action-diagnostics",
        action="store_true",
        help="send System-1/System-2 plan traces to the GUI rollout recorder",
    )
    parser.add_argument(
        "--inference-mode",
        choices=("both", "sys2"),
        default=None,
        help="default: the checkpoint's own (R4 is sys2: System-2 plans served as action chunks)",
    )
    parser.add_argument(
        "--beast-src",
        default=None,
        help="use this beast_refiner source tree (its src/ folder) instead of the installed one, "
        "e.g. a checkout of the commit a checkpoint was trained on",
    )
    parser.add_argument(
        "--swap-rgb",
        action="store_true",
        help="swap red/blue of incoming frames (use only if the live camera stream is BGR)",
    )
    parser.add_argument("--no-warmup", action="store_true", help="skip CUDA warmup (first rollout will be slower)")
    args = parser.parse_args()

    if args.beast_src:
        src = Path(args.beast_src).expanduser().resolve()
        if not (src / "beast_refiner" / "__init__.py").is_file():
            parser.error(f"--beast-src {src} has no beast_refiner/ package; pass the repo's src/ folder")
        sys.path.insert(0, str(src))  # ahead of the installed (editable) copy
    os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    policy = BeastRefinerPolicy(
        ckpt=args.ckpt,
        device=args.device,
        xvla_checkpoint=args.xvla_checkpoint,
        task=args.task,
        camera_map=args.camera_map,
        direct_spline_projection=args.direct_spline_projection,
        direct_spline_basis=args.direct_spline_basis,
        sys2_trigger_margin_ticks=args.sys2_trigger_margin_ticks,
        warmup=not args.no_warmup,
        action_diagnostics=args.action_diagnostics,
        swap_rgb=args.swap_rgb,
        inference_mode=args.inference_mode,
    )
    try:
        PolicyServer(policy, host=args.host, port=args.port).serve_forever()
    except KeyboardInterrupt:
        policy.close()


if __name__ == "__main__":
    main()
