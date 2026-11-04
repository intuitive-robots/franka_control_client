"""MemoryVLA policy server for the inference GUI (run on the policy PC).

    conda activate memvla
    python examples/memvla_policy_server.py --port 8767 \
        --ckpt /path/to/<run>/checkpoints/step-020000.pt

Then in the GUI (robot PC): point the policy server at <policy-pc-ip>:8767 and add
the front camera (zed_right, obs_key observation.images.external -- the dataset's ``right`` view) to the camera list. MemoryVLA is
single-camera: only ``--camera_key`` is fed to the model; other images are ignored.

Model contract (our fine-tune, see MemoryVLA/jsc/README.md):
  * input : one 224x224 RGB front image + task string (no proprio)
  * output: chunk [16, 8] of ABSOLUTE joint targets q0..q6 + gripper closedness
            (0 open, 1 closed), 15 Hz -- exactly the GUI's joint action space,
            so no conversion is needed.

Memory: the cognitive/perceptual memory banks persist across calls and are reset on
the first ``infer`` after every GUI ``reset`` (= start of a rollout). In training each
memory entry is one 15 Hz frame; here a call happens only every ``exec_steps`` frames,
so by default the timestep counter is advanced by ``exec_steps`` per call to keep the
timestep encoding on the real frame index (``--timestep_stride 1`` disables this).
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np

_HERE = Path(__file__).resolve()
_CLIENT_SRC = _HERE.parents[1] / "src"
_DEFAULT_MEMVLA_ROOT = _HERE.parents[2] / "MemoryVLA"
if _CLIENT_SRC.is_dir() and str(_CLIENT_SRC) not in sys.path:
    sys.path.insert(0, str(_CLIENT_SRC))

# Base weights (Llama-2 config+tokenizer, DINOv2 / SigLIP towers) live next to the
# checkpoints, not in ~/.cache. Must be set before huggingface_hub is imported.
_DEFAULT_HF_CACHE = _HERE.parents[2] / "ckpt" / "memvla" / "hf_cache"
os.environ.setdefault("HF_HUB_CACHE", str(_DEFAULT_HF_CACHE))
# meta-llama/Llama-2-7b-hf is gated; in inference mode only its config + tokenizer are
# read (every LLM weight comes from the checkpoint), so the verbatim public mirror is used.
os.environ.setdefault("MEMVLA_LLAMA2_REPO", "NousResearch/Llama-2-7b-hf")


def _alias_numpy2_pickles() -> None:
    """Let numpy 1.x unpickle arrays sent by a numpy 2.x robot PC.

    numpy 2 pickles reference ``numpy._core.*``; numpy 1.x only has ``numpy.core.*``
    (same objects). The memvla env must stay on numpy<2 (tensorflow 2.15), so map the
    names instead. No-op on numpy 2.
    """
    if int(np.__version__.split(".")[0]) >= 2:
        return
    import importlib
    import numpy.core

    for sub in ("", ".multiarray", ".numeric", ".umath", "._multiarray_umath",
                ".fromnumeric", ".records", "._internal", "._dtype", "._methods"):
        try:
            sys.modules.setdefault("numpy._core" + sub, importlib.import_module("numpy.core" + sub))
        except ImportError:
            pass


_alias_numpy2_pickles()

from franka_control_client.inference_gui.policy_server import (  # noqa: E402
    JOINT_ACTION_SPACE,
    BasePolicy,
    PolicyServer,
)

LOG = logging.getLogger("memvla_server")

# Architecture kwargs that must match training; read back from <run>/config.json.
# Same list as MemoryVLA/jsc/openloop_memvla.py: from_pretrained defaults action_dim
# to 7 with strict=False, so a missing kwarg silently randomises the action head.
ARCH_KEYS = (
    "action_dim", "future_action_window_size", "action_model_type", "use_ema",
    "dataloader_type", "group_size", "per_token_size", "mem_length",
    "retrieval_layers", "use_timestep_pe", "fusion_type", "consolidate_type",
    "update_fused",
)


def _resolve_hf_token(token_or_path: Optional[str]) -> Optional[str]:
    token_or_path = token_or_path or os.environ.get("HF_TOKEN") or os.environ.get("HUGGING_FACE_HUB_TOKEN")
    if token_or_path:
        p = os.path.expanduser(str(token_or_path))
        return open(p).read().strip() if os.path.isfile(p) else token_or_path
    default = os.path.expanduser("~/.cache/huggingface/token")
    return open(default).read().strip() if os.path.isfile(default) else None


class MemVLAPolicy(BasePolicy):
    action_space = JOINT_ACTION_SPACE
    fps = 15.0  # training data is 15 Hz; one chunk row per frame

    def __init__(
        self,
        ckpt: str,
        memvla_root: str = str(_DEFAULT_MEMVLA_ROOT),
        unnorm_key: str = "franka_lerobot",
        camera_key: str = "observation.images.external",
        default_task: Optional[str] = None,
        use_gui_task: bool = False,
        exec_steps: int = 8,
        cfg_scale: float = 1.5,
        num_ddim_steps: int = 10,
        use_ddpm: bool = False,
        use_bf16: bool = True,
        timestep_stride: Optional[int] = None,
        image_size: int = 224,
        hf_token: Optional[str] = None,
        save_debug_image: Optional[str] = None,
    ) -> None:
        os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")
        # flash-attn is not installed in the memvla env; eval forces it off anyway.
        os.environ.setdefault("MEMVLA_NO_FLASH", "1")
        if str(memvla_root) not in sys.path:
            sys.path.insert(0, str(memvla_root))

        import torch
        from PIL import Image
        from vla import load_vla

        # prismatic's overwatch runs dictConfig(disable_existing_loggers=True) on import,
        # which silences every logger created before it (ours and the PolicyServer's).
        for name in ("memvla_server", "policy_server"):
            logging.getLogger(name).disabled = False

        self._torch = torch
        self._Image = Image
        self.ckpt = str(ckpt)
        self.unnorm_key = unnorm_key
        self.camera_key = camera_key
        self.default_task = default_task
        self.use_gui_task = bool(use_gui_task)
        self._warned_task = False
        self.exec_steps = int(exec_steps)
        self.cfg_scale = float(cfg_scale)
        self.num_ddim_steps = int(num_ddim_steps)
        self.use_ddim = not use_ddpm
        self.use_bf16 = bool(use_bf16)
        self.timestep_stride = int(timestep_stride) if timestep_stride else self.exec_steps
        self.image_size = int(image_size)
        self.save_debug_image = save_debug_image

        self.task: Optional[str] = None
        self._first_frame = True
        self._n_calls = 0
        self._lock = threading.Lock()

        # ---- architecture from the run's own config.json ------------------
        ckpt_path = Path(self.ckpt).resolve()
        cfg_json = ckpt_path.parents[1] / "config.json"
        if not cfg_json.exists():
            raise FileNotFoundError(
                f"{cfg_json} not found; expected <run>/checkpoints/<step>.pt next to <run>/config.json"
            )
        # Training instruction of this checkpoint: --task, else <run>/task.txt. The model
        # is single-task and sensitive to wording, so this wins over the GUI's task
        # unless --use_gui_task is given.
        task_txt = ckpt_path.parents[1] / "task.txt"
        if not self.default_task and task_txt.exists():
            self.default_task = task_txt.read_text().strip() or None
        LOG.info("checkpoint task: %r (%s)", self.default_task,
                 "GUI task overrides" if self.use_gui_task else "overrides GUI task")
        tcfg = json.load(open(cfg_json))
        arch = {k: tcfg[k] for k in ARCH_KEYS if k in tcfg}
        LOG.info("arch from config.json: %s", arch)

        with torch.inference_mode():
            self.vla = load_vla(
                str(ckpt_path),
                hf_token=_resolve_hf_token(hf_token),
                load_for_training=False,
                **arch,
            )
            self._to_device(torch.bfloat16 if self.use_bf16 else torch.float32)

        act_dim = self.vla.get_action_dim(self.unnorm_key)
        if act_dim != 8 or self.vla.action_model.in_channels != 8:
            raise ValueError(
                f"expected an 8-dim joint+gripper model, got norm-stats dim {act_dim} / head "
                f"{self.vla.action_model.in_channels} (wrong --unnorm_key or checkpoint?)"
            )
        self.chunk_len = int(self.vla.future_action_window_size) + 1
        if not 1 <= self.exec_steps <= self.chunk_len:
            raise ValueError(f"exec_steps must be in [1, {self.chunk_len}]")
        LOG.info(
            "MemoryVLA ready: chunk=%d exec_steps=%d timestep_stride=%d sampler=%s dtype=%s",
            self.chunk_len, self.exec_steps, self.timestep_stride,
            f"ddim-{self.num_ddim_steps}" if self.use_ddim else "ddpm",
            "bf16" if self.use_bf16 else "fp32",
        )

    def _to_device(self, dtype) -> None:
        vla = self.vla
        if getattr(vla, "offload_memory_banks", False):  # MEMVLA_OFFLOAD_MEMORY_BANKS=1
            for name in ("vlm", "per_compr", "action_model", "ema_diffusion"):
                if hasattr(vla, name):
                    getattr(vla, name).to(device="cuda", dtype=dtype)
            vla.cog_mem_bank.to(device="cpu")
            vla.per_mem_bank.to(device="cpu")
            vla.eval()
        else:
            self.vla = vla.to(device="cuda", dtype=dtype).eval()

    # ------------------------------------------------------------- BasePolicy
    def metadata(self) -> Dict[str, Any]:
        return {
            "name": "memoryvla",
            "ckpt": self.ckpt,
            "unnorm_key": self.unnorm_key,
            "camera_key": self.camera_key,
            "task": self.default_task,
            "task_source": "gui" if self.use_gui_task or not self.default_task else "checkpoint",
            "chunk_len": self.chunk_len,
            "sampler": f"ddim-{self.num_ddim_steps}" if self.use_ddim else "ddpm",
            "cfg_scale": self.cfg_scale,
            "timestep_stride": self.timestep_stride,
        }

    def reset(self, task: Optional[str]) -> None:
        with self._lock:
            self.task = task
            self._first_frame = True  # memory banks are cleared on the next infer
            self._n_calls = 0
        LOG.info("reset: task=%r", task)

    def _get_image(self, observation: Dict[str, Any]) -> np.ndarray:
        img = observation.get(self.camera_key)
        if img is None:
            keys = [k for k in observation if k.startswith("observation.images.")]
            if len(keys) == 1:
                img = observation[keys[0]]
            else:
                raise KeyError(
                    f"camera '{self.camera_key}' not in observation (got {keys}); "
                    f"set --camera_key or fix the GUI camera list"
                )
        img = np.asarray(img)
        if img.ndim != 3 or img.shape[2] != 3:
            raise ValueError(f"expected HxWx3 RGB image, got {img.shape}")
        if img.dtype != np.uint8:
            img = np.clip(img * (255.0 if img.max() <= 1.0 else 1.0), 0, 255).astype(np.uint8)
        # Training frames are 224x224, produced by a plain (non-aspect-preserving)
        # resize of the camera image -- same as CameraDevice(final_size=...).
        if img.shape[:2] != (self.image_size, self.image_size):
            img = cv2.resize(img, (self.image_size, self.image_size), interpolation=cv2.INTER_AREA)
        return img

    def infer(self, observation: Dict[str, Any]) -> np.ndarray:
        gui_task = observation.get("task") or self.task
        if self.default_task and not self.use_gui_task:
            task = self.default_task
            if gui_task and gui_task.strip() != task and not self._warned_task:
                LOG.warning("GUI task %r ignored; using checkpoint task %r "
                            "(pass --use_gui_task to change)", gui_task, task)
                self._warned_task = True
        else:
            task = gui_task or self.default_task
        if not task:
            raise ValueError("no task instruction: set it in the GUI or pass --task")
        image = self._Image.fromarray(self._get_image(observation))
        if self.save_debug_image:
            image.save(self.save_debug_image)

        with self._lock, self._torch.inference_mode():
            first = self._first_frame
            t0 = time.time()
            actions, _ = self.vla.predict_action(
                image=image,
                instruction=task,
                unnorm_key=self.unnorm_key,
                cfg_scale=self.cfg_scale,
                use_ddim=self.use_ddim,
                num_ddim_steps=self.num_ddim_steps,
                episode_first_frame="True" if first else "False",
            )
            # predict_action advanced cur_timestep by 1; the robot will execute
            # exec_steps frames before the next call.
            self.vla.cur_timestep += self.timestep_stride - 1
            self._first_frame = False
            self._n_calls += 1
            dt = time.time() - t0

        chunk = np.asarray(actions, dtype=np.float32).reshape(-1, 8)
        chunk[:, 7] = np.clip(chunk[:, 7], 0.0, 1.0)
        if not np.all(np.isfinite(chunk)):
            raise ValueError("model produced non-finite actions")
        LOG.info(
            "infer #%d%s %.0f ms | task=%r | grip=%.2f",
            self._n_calls, " (memory reset)" if first else "", dt * 1e3, task, chunk[0, 7],
        )
        return chunk


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8767)
    p.add_argument("--ckpt", required=True, help="<run>/checkpoints/step-XXXX.pt")
    p.add_argument("--memvla_root", default=str(_DEFAULT_MEMVLA_ROOT))
    p.add_argument("--unnorm_key", default="franka_lerobot")
    p.add_argument("--camera_key", default="observation.images.external",
                   help="observation key of the front camera fed to the model")
    p.add_argument("--task", default=None,
                   help="instruction to use (default: <run>/task.txt); overrides the GUI task")
    p.add_argument("--use_gui_task", action="store_true",
                   help="use the task typed in the GUI instead of the checkpoint task")
    p.add_argument("--exec_steps", type=int, default=8, help="chunk rows executed per request")
    p.add_argument("--timestep_stride", type=int, default=0,
                   help="memory timestep advance per call (0 -> exec_steps)")
    p.add_argument("--cfg_scale", type=float, default=1.5)
    p.add_argument("--num_ddim_steps", type=int, default=10)
    p.add_argument("--ddpm", action="store_true", help="100-step DDPM instead of DDIM")
    p.add_argument("--fp32", action="store_true", help="float32 instead of bf16 (needs >32 GB VRAM)")
    p.add_argument("--hf_token", default=None, help="token or path to token file")
    p.add_argument("--save_debug_image", default=None, help="write the model input image here")
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    policy = MemVLAPolicy(
        ckpt=args.ckpt,
        memvla_root=args.memvla_root,
        unnorm_key=args.unnorm_key,
        camera_key=args.camera_key,
        default_task=args.task,
        use_gui_task=args.use_gui_task,
        exec_steps=args.exec_steps,
        cfg_scale=args.cfg_scale,
        num_ddim_steps=args.num_ddim_steps,
        use_ddpm=args.ddpm,
        use_bf16=not args.fp32,
        timestep_stride=args.timestep_stride or None,
        hf_token=args.hf_token,
        save_debug_image=args.save_debug_image,
    )
    PolicyServer(policy, host=args.host, port=args.port).serve_forever()


if __name__ == "__main__":
    main()
