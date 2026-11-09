"""MME-VLA ("memevla", pi0.5 + perceptual memory) policy server for the inference GUI.

Runs on the policy PC inside the memevla venv (JAX):

    cd /data/zhuoyue/Nov_Evaluation/memevla && source env_local.sh
    $PY ../franka_control_client/examples/meme_policy_server.py --port 8768 \
        --ckpt ../ckpt/meme/franka-mem-pi05-seed42-10k/franka-mem-seed42/9999 \
        --compressor runs/token_compressor/franka/best.pt   # the compressor the run was trained with

The model is multitask: type the task's prompt in the GUI verbatim (table in the checkpoint's
README.md), or pin it with --task.

GUI (robot PC): control pair **joint**, cameras zed_right -> observation.images.external
(front, also the memory view) and zed_wrist -> observation.images.wrist, both send_to_policy.
Recommended: inference_mode "async" (see "Memory clock" below).

Model contract (memevla/realworld/HANDOVER.md, our LeRobot data converted to its HDF5 format):
  * input : front + wrist RGB (collected at 224x224, packed to 256x256), state [8] =
            q0..q6 + gripper closedness, task prompt (verbatim training prompt)
  * output: chunk [20, 8] = joint targets q0..q6 + gripper closedness at 15 Hz. The model may be
            trained on joint DELTAS w.r.t. the current joints (gripper absolute), as
            franka-mem-pi05-seed42-10k is. memevla >= a4b9d9f restores absolute targets itself
            (franka_mem_pi05: delta_joint_actions=True -> AbsoluteActions); with an older checkout
            ``--action_mode auto`` detects deltas from the norm stats and adds the joints back here.
    The memevla config ``franka_mem_pi05`` slices the model output to 7 dims
    (``RoboMemArenaOutputs``, LIBERO delta actions); this server re-slices it to the
    dimension of the checkpoint's action norm stats, so an 8-D training run is served
    without touching the memevla repo. Anything other than 8 dims is refused.

Memory clock: the model was trained on every 15 Hz frame and its time code counts frames
(128 uniformly kept frames x 4 tokens). Preferred: set the GUI's **env frequency** to 15 Hz;
the GUI then streams every frame ("observe" requests, mapped to frames by timestamp) and the
memory is fed only from that stream. Without the stream the GUI only sends a frame when it
asks for a chunk, so on every call the server advances the memory by the number of frames that elapsed since
the previous call and fills that gap with the current image (state linearly interpolated):
  * ``--frame_clock wall`` (default): frames = elapsed wall time x fps. Matches real time
    (what POT TIMER needs). In GUI sync mode the arm holds still during inference, so the
    memory also sees those pauses; async mode avoids them.
  * ``--frame_clock exec``: frames = exec_steps per call (frames the arm actually executed).
The SigLIP encoding of the current frame is computed once and reused for the gap frames;
only the frame-dependent position code and the learned compressor are recomputed per frame.
The memory position table holds 4096 frames (273 s at 15 Hz): set the GUI's max_episode_s
below that.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

_HERE = Path(__file__).resolve()
_CLIENT_SRC = _HERE.parents[1] / "src"
_DEFAULT_MEME_ROOT = _HERE.parents[2] / "memevla"
if _CLIENT_SRC.is_dir() and str(_CLIENT_SRC) not in sys.path:
    sys.path.insert(0, str(_CLIENT_SRC))


def _setup_env(meme_root: Path) -> None:
    """What memevla/env_local.sh exports; must run before jax is imported."""
    for p in (meme_root, meme_root / "packages" / "openpi-client" / "src", meme_root / "src"):
        if str(p) not in sys.path:
            sys.path.insert(0, str(p))
    os.environ.setdefault("OPENPI_DATA_HOME", "/data/zhuoyue/openpi_data")
    os.environ.setdefault("JAX_COMPILATION_CACHE_DIR", "/data/zhuoyue/jax_cache")
    os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")
    # jaxlib 0.5.3 segfaults compiling GEMMs on the RTX 5090 (sm_120) with Triton GEMM / autotuning.
    flags = os.environ.get("XLA_FLAGS", "")
    for f in ("--xla_gpu_enable_triton_gemm=false", "--xla_gpu_autotune_level=0"):
        if f.split("=")[0] not in flags:
            flags += " " + f
    os.environ["XLA_FLAGS"] = flags.strip()


def _alias_numpy2_pickles() -> None:
    """Let numpy 1.x (memevla venv) unpickle arrays sent by a numpy 2.x robot PC."""
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

LOG = logging.getLogger("meme_server")
MAX_MEMORY_FRAMES = 4096  # MemoryBuffer position table


class _SliceActions:
    """Replacement for RoboMemArenaOutputs ([:, :7]): keep the dataset's action dimension."""

    def __init__(self, dim: int) -> None:
        self.dim = int(dim)

    def __call__(self, data: dict) -> dict:
        return {"actions": np.asarray(data["actions"][:, : self.dim], dtype=np.float32)}


class MemePolicy(BasePolicy):
    action_space = JOINT_ACTION_SPACE
    fps = 15.0  # dataset rate; the memory time code counts 15 Hz frames

    def __init__(
        self,
        ckpt: str,
        config: str = "franka_mem_pi05",
        meme_root: str = str(_DEFAULT_MEME_ROOT),
        front_key: str = "observation.images.external",
        wrist_key: str = "observation.images.wrist",
        task: Optional[str] = None,
        exec_steps: int = 10,
        frame_clock: str = "wall",
        seed: int = 7,
        collect_size: int = 224,
        pack_size: int = 256,
        save_debug_image: Optional[str] = None,
        warmup: bool = True,
        action_mode: str = "auto",
        compressor: Optional[str] = None,
    ) -> None:
        meme_root_p = Path(meme_root).resolve()
        ckpt_p = Path(ckpt).resolve()
        if not (ckpt_p / "params").is_dir():
            raise FileNotFoundError(f"{ckpt_p}/params not found; --ckpt is <ckpts>/<config>/<exp>/<step>")
        if not any((ckpt_p.parent / n).exists() for n in ("history_config.txt", "history_config.yaml")):
            LOG.warning("no history_config.txt/.yaml next to %s: the config's own memory settings are used",
                        ckpt_p)
        _setup_env(meme_root_p)
        # Relative paths in the memory YAML (compressor_checkpoint: runs/token_compressor/...) are
        # resolved against the cwd, as in the memevla scripts.
        os.chdir(meme_root_p)

        import jax  # noqa: F401  (after _setup_env)
        from mme_vla_suite.policies import policy_config as _policy_config
        from mme_vla_suite.policies.robomme_policy import RoboMemArenaOutputs
        from mme_vla_suite.training import config as _config

        for name in ("meme_server", "policy_server"):  # in case an import silenced existing loggers
            logging.getLogger(name).disabled = False

        self.ckpt = str(ckpt_p)
        self.config_name = config
        self.front_key, self.wrist_key = front_key, wrist_key
        self.default_task = task
        self.exec_steps = int(exec_steps)
        if frame_clock not in ("wall", "exec"):
            raise ValueError("frame_clock must be 'wall' or 'exec'")
        self.frame_clock = frame_clock
        self.collect_size, self.pack_size = int(collect_size), int(pack_size)
        self.save_debug_image = save_debug_image
        self._lock = threading.Lock()

        override = None
        if compressor:
            override = self._history_config_with_compressor(ckpt_p, Path(compressor).resolve(strict=True))
        else:
            self._check_compressor(ckpt_p)

        t0 = time.time()
        LOG.info("loading %s (%s) ...", ckpt_p, config)
        self.policy = _policy_config.create_trained_policy(
            _config.get_config(config), ckpt_p, seed=seed, history_config_override=override)

        stats = self.policy._output_transform  # CompositeTransform
        norm = None
        for t in stats.transforms:
            if hasattr(t, "norm_stats") and t.norm_stats and "actions" in t.norm_stats:
                norm = t.norm_stats["actions"]
        if norm is None:
            raise RuntimeError("no action norm stats in the policy's output transforms")
        self.action_dim = int(np.asarray(norm.mean).shape[-1])
        if self.action_dim != 8:
            raise ValueError(
                f"checkpoint actions are {self.action_dim}-D; this server serves 8-D absolute joint targets "
                f"(q0..q6, gripper closedness). Train on the dataset's 8-D 'action' column.")
        from openpi import transforms as _transforms

        ts = list(stats.transforms)
        if any(isinstance(t, _transforms.AbsoluteActions) for t in ts):
            # memevla >= a4b9d9f: franka_mem_pi05 (delta_joint_actions=True) adds the state back itself.
            if action_mode == "delta":
                raise ValueError("the memevla config already restores absolute actions (AbsoluteActions); "
                                 "--action_mode delta would add the state twice")
            self.action_mode = "delta (restored by memevla AbsoluteActions)"
            LOG.info("action conversion: memevla output transforms (AbsoluteActions) -> absolute targets")
        else:  # older memevla checkout: convert here
            self.action_mode = self._resolve_action_mode(action_mode, ts_norm=stats)
        idx = [i for i, t in enumerate(ts) if isinstance(t, RoboMemArenaOutputs)]
        if not idx:
            ts.append(_SliceActions(self.action_dim))
        elif getattr(ts[idx[-1]], "action_dim", 7) != self.action_dim:
            ts[idx[-1]] = _SliceActions(self.action_dim)  # older memevla: hard-coded [:, :7]
        self.policy._output_transform = type(stats)(ts)  # CompositeTransform is frozen

        hc = self.policy.config
        self.has_memory = hc is not None and self.policy.mem_buffer is not None
        self._fast_fill = bool(
            self.has_memory and hc.representation_type == "perceptual"
            and not self.policy.mem_buffer.compute_token_drop_score
            and self.policy._stem_encode is None and not self.policy._memory_geometry
            and self.policy._fact_runtime is None)
        self.memory_desc = (
            f"{hc.perceptual_memory.type}, {hc.perceptual_memory.history_frames}x"
            f"{hc.perceptual_memory.history_tokens_per_image} tokens, "
            f"{hc.perceptual_memory.get('time_encoding', 'absolute')}"
            if self.has_memory and hc.representation_type == "perceptual" else str(hc and hc.representation_type))
        LOG.info("loaded in %.0f s | memory: %s | action_dim %d | actions: %s", time.time() - t0,
                 self.memory_desc, self.action_dim, self.action_mode)

        self.task: Optional[str] = None
        self._warned_task = False
        self._n_calls = 0
        self._t_last: Optional[float] = None
        self._state_last: Optional[np.ndarray] = None
        self.chunk_len = 0
        if warmup:
            self._warmup()
        self.reset(None)

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _yaml_compressor(path: Path) -> Optional[str]:
        import re
        m = re.search(r"^\s*compressor_checkpoint:\s*(\S+)", path.read_text(), re.M)
        return m.group(1) if m else None

    def _check_compressor(self, ckpt_p: Path) -> None:
        y = ckpt_p.parent / "history_config.yaml"
        c = self._yaml_compressor(y) if y.exists() else None
        if c and not Path(c).exists():  # relative to the memevla root (cwd)
            raise FileNotFoundError(
                f"token compressor {c} (from {y}) not found under {Path.cwd()}; it is not part of the "
                f"checkpoint. Get the compressor the run was trained with and pass --compressor <best.pt>")

    @staticmethod
    def _history_config_with_compressor(ckpt_p: Path, compressor: Path) -> str:
        """Copy of the run's history_config.yaml pointing at ``compressor`` (checkpoint files untouched)."""
        import re
        import tempfile
        src = ckpt_p.parent / "history_config.yaml"
        text = re.sub(r"^(\s*compressor_checkpoint:).*$", rf"\1 {compressor}", src.read_text(), flags=re.M)
        out = Path(tempfile.mkdtemp(prefix="meme_hc_")) / "history_config.yaml"
        out.write_text(text)
        LOG.info("memory config %s with compressor %s", src, compressor)
        return str(out)

    def _resolve_action_mode(self, mode: str, ts_norm) -> str:
        """'delta': joints are offsets from the current joints (openpi DeltaActions), gripper absolute.
        auto: absolute joint targets have the state's mean (q3 ~ -2 rad, q5 ~ +2 rad); deltas are ~0."""
        if mode in ("delta", "abs"):
            return mode
        if mode != "auto":
            raise ValueError("action_mode must be auto, delta or abs")
        stats = {}
        for t in ts_norm.transforms:
            if hasattr(t, "norm_stats") and t.norm_stats:
                stats = t.norm_stats
        a = np.asarray(stats["actions"].mean)[:7]
        s = np.asarray(stats["state"].mean)[:7]
        mode = "delta" if np.abs(a).max() < 0.2 and np.abs(s).max() > 1.0 else "abs"
        LOG.info("action_mode auto -> %s (action mean %s, state mean %s)", mode,
                 np.round(a, 3).tolist(), np.round(s, 3).tolist())
        return mode

    def _prep_image(self, img: Any, name: str) -> np.ndarray:
        """Camera frame -> training pixels: plain resize to the collection size (as the
        recorder did), then PIL bilinear to the packed size (as realworld/build_packed.py)."""
        import cv2
        from PIL import Image

        img = np.asarray(img)
        if img.ndim != 3 or img.shape[2] != 3:
            raise ValueError(f"{name}: expected HxWx3 RGB, got {img.shape}")
        if img.dtype != np.uint8:
            img = np.clip(img * (255.0 if img.max() <= 1.0 else 1.0), 0, 255).astype(np.uint8)
        if self.collect_size and img.shape[:2] != (self.collect_size, self.collect_size):
            img = cv2.resize(img, (self.collect_size, self.collect_size), interpolation=cv2.INTER_AREA)
        if img.shape[:2] != (self.pack_size, self.pack_size):
            img = np.asarray(Image.fromarray(img).resize((self.pack_size, self.pack_size), Image.BILINEAR))
        return np.ascontiguousarray(img)

    def _get(self, observation: Dict[str, Any], key: str) -> Any:
        if key not in observation:
            keys = [k for k in observation if k.startswith("observation.images.")]
            raise KeyError(f"camera '{key}' not in observation (got {keys}); fix the GUI camera list "
                           f"or pass --front_key/--wrist_key")
        return observation[key]

    def _add_frames(self, image: np.ndarray, states: np.ndarray) -> None:
        """Append len(states) consecutive memory frames, all showing ``image``."""
        pol = self.policy
        n = len(states)
        first = pol.step_idx + 1
        if first + n > MAX_MEMORY_FRAMES:
            raise RuntimeError(f"episode longer than the memory's {MAX_MEMORY_FRAMES} frames "
                               f"({MAX_MEMORY_FRAMES / self.fps:.0f} s); lower the GUI max_episode_s")
        if not self._fast_fill:
            for k in range(n):
                pol.add_buffer({"images": image[None, None], "state": states[k:k + 1]})
            return
        pol.add_buffer({"images": image[None, None], "state": states[:1]})  # one SigLIP pass
        buf = pol.mem_buffer
        base = buf._history_feats[first]
        nv = buf.num_views
        for k in range(1, n):
            s = first + k
            e = dict(base)
            e["state_emb"] = states[k]
            for g in ("8x8", "4x4", "2x2"):
                e[f"pos_emb_{g}"] = buf.pos_emb_dict[g][s * nv:(s + 1) * nv]
            if buf.learned_compress_fn is not None:
                e["image_emb_learned"], e["pos_emb_learned"] = buf.learned_compress_fn(
                    e["image_emb_8x8"], e["pos_emb_8x8"])
            buf._history_feats[s] = e
        pol.step_idx += n - 1

    def _warmup(self) -> None:
        """Compile SigLIP + the sampler once so the GUI's first request does not time out."""
        t0 = time.time()
        img = np.zeros((self.pack_size, self.pack_size, 3), np.uint8)
        state = np.zeros(8, np.float32)
        self.policy.reset()
        self._add_frames(img, np.stack([state] * 3))
        out = self.policy.infer({"observation/image": img, "observation/wrist_image": img,
                                 "observation/state": state, "prompt": "warm up"})
        self.chunk_len = int(np.asarray(out["actions"]).shape[0])
        LOG.info("warm-up (JIT compile) %.0f s, chunk %s", time.time() - t0, np.asarray(out["actions"]).shape)

    # --------------------------------------------------------------- BasePolicy
    def metadata(self) -> Dict[str, Any]:
        return {
            "name": "mme-vla",
            "ckpt": self.ckpt,
            "config": self.config_name,
            "memory": self.memory_desc,
            "frame_clock": self.frame_clock,
            "cameras": {"front(memory)": self.front_key, "wrist": self.wrist_key},
            "task": self.default_task,
            "task_source": "server --task" if self.default_task else "gui",
            "chunk_len": self.chunk_len,
            "action_mode": self.action_mode,
        }

    def reset(self, task: Optional[str]) -> None:
        with self._lock:
            self.task = task
            self.policy.reset()  # new memory buffer, step_idx = -1, rng reseeded
            self._n_calls = 0
            self._t_last = None
            self._state_last = None
            self._stream_t0: Optional[float] = None  # timestamp of env frame 0 (GUI env stream)
            self._n_observed = 0
        LOG.info("reset: task=%r", task)

    def observe(self, observation: Dict[str, Any]) -> None:
        """One frame of the GUI's env stream (env frequency > 0). Its timestamp is
        mapped onto the model's 15 Hz frame index: frames landing on an index that is
        already filled are dropped, missing indices are filled with this frame (state
        interpolated), so any env frequency keeps the memory clock on real time."""
        if not self.has_memory:
            return
        state = np.asarray(observation["observation.state"], dtype=np.float32).reshape(-1)
        if state.shape != (8,):
            raise ValueError(f"expected joint state [8] (q0..q6, gripper), got {state.shape}")
        ts = float(observation.get("timestamp", time.time()))
        with self._lock:
            if self._stream_t0 is None:
                self._stream_t0 = ts
            target = int(round((ts - self._stream_t0) * self.fps))  # model frame index of this obs
            n_new = target - self.policy.step_idx
            if n_new < 1:
                return  # this slot already has a frame (env frequency above the model's fps)
            front = self._prep_image(self._get(observation, self.front_key), self.front_key)
            if self._state_last is None or n_new == 1:
                states = state[None]
            else:
                w = (np.arange(1, n_new + 1, dtype=np.float32) / n_new)[:, None]
                states = (1 - w) * self._state_last[None] + w * state[None]
            self._add_frames(front, states.astype(np.float32))
            self._state_last = state
            self._n_observed += 1

    def _resolve_task(self, observation: Dict[str, Any]) -> str:
        gui_task = (observation.get("task") or self.task or "").strip()
        if self.default_task:
            if gui_task and gui_task != self.default_task and not self._warned_task:
                LOG.warning("GUI task %r ignored; using --task %r", gui_task, self.default_task)
                self._warned_task = True
            return self.default_task
        if not gui_task:
            raise ValueError("no task: type the exact training prompt in the GUI or pass --task")
        return gui_task

    def infer(self, observation: Dict[str, Any]) -> np.ndarray:
        task = self._resolve_task(observation)
        state = np.asarray(observation["observation.state"], dtype=np.float32).reshape(-1)
        if state.shape != (8,):
            raise ValueError(f"expected joint state [8] (q0..q6, gripper), got {state.shape}")
        front = self._prep_image(self._get(observation, self.front_key), self.front_key)
        wrist = self._prep_image(self._get(observation, self.wrist_key), self.wrist_key)
        if self.save_debug_image:
            from PIL import Image
            Image.fromarray(np.concatenate([front, wrist], axis=1)).save(self.save_debug_image)

        with self._lock:
            now = time.time()
            first = self._t_last is None
            streamed = bool(observation.get("env_stream")) and self._stream_t0 is not None
            if streamed:  # memory is fed by observe(); the model just reads it
                n_new = 0 if self.policy.step_idx >= 0 else 1
            elif first:
                n_new = 1
            elif self.frame_clock == "wall":
                n_new = max(1, int(round((now - self._t_last) * self.fps)))
            else:
                n_new = self.exec_steps
            if streamed or first or n_new == 1:
                states = state[None]
            else:  # linear from the previous observed state to this one
                w = (np.arange(1, n_new + 1, dtype=np.float32) / n_new)[:, None]
                states = (1 - w) * self._state_last[None] + w * state[None]
            t0 = time.perf_counter()
            if self.has_memory and n_new > 0:
                self._add_frames(front, states.astype(np.float32))
            t1 = time.perf_counter()
            out = self.policy.infer({"observation/image": front, "observation/wrist_image": wrist,
                                     "observation/state": state, "prompt": task})
            t2 = time.perf_counter()
            self._t_last = now
            if not streamed:
                self._state_last = state
            self._n_calls += 1
            step = self.policy.step_idx

        chunk = np.asarray(out["actions"], dtype=np.float32).reshape(-1, self.action_dim)
        if self.action_mode == "delta":
            chunk[:, :7] += state[None, :7]
        if not np.all(np.isfinite(chunk)):
            raise ValueError("model produced non-finite actions")
        chunk[:, 7] = np.clip(chunk[:, 7], 0.0, 1.0)
        LOG.info("infer #%d frame %d (+%d%s) | mem %.0f ms model %.0f ms | jump %.3f rad | grip %.2f | %r",
                 self._n_calls, step, n_new, ", env stream" if streamed else "", (t1 - t0) * 1e3, (t2 - t1) * 1e3,
                 float(np.abs(chunk[0, :7] - state[:7]).max()), chunk[0, 7], task)
        return chunk


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8768)
    p.add_argument("--ckpt", required=True, help="<ckpts>/<config>/<exp>/<step> (contains params/ and assets/)")
    p.add_argument("--config", default="franka_mem_pi05", help="memevla training config name")
    p.add_argument("--meme_root", default=str(_DEFAULT_MEME_ROOT))
    p.add_argument("--front_key", default="observation.images.external",
                   help="GUI obs key of the front camera (dataset 'right' view; also the memory view)")
    p.add_argument("--wrist_key", default="observation.images.wrist")
    p.add_argument("--task", default=None, help="prompt to use instead of the GUI task (verbatim training prompt)")
    p.add_argument("--exec_steps", type=int, default=10, help="chunk rows executed per request")
    p.add_argument("--frame_clock", choices=("wall", "exec"), default="wall",
                   help="how many memory frames elapse between requests (see module doc)")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--collect_size", type=int, default=224,
                   help="resolution the demonstrations were recorded at (0: skip this resize)")
    p.add_argument("--save_debug_image", default=None, help="write the front|wrist model input here")
    p.add_argument("--no_warmup", action="store_true")
    p.add_argument("--action_mode", choices=("auto", "delta", "abs"), default="auto",
                   help="model joint outputs: deltas from the current joints or absolute (auto: from norm stats)")
    p.add_argument("--compressor", default=None,
                   help="token compressor best.pt the run was trained with (overrides the memory YAML path)")
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s", force=True)
    policy = MemePolicy(
        ckpt=args.ckpt, config=args.config, meme_root=args.meme_root, front_key=args.front_key,
        wrist_key=args.wrist_key, task=args.task, exec_steps=args.exec_steps, frame_clock=args.frame_clock,
        seed=args.seed, collect_size=args.collect_size, save_debug_image=args.save_debug_image,
        warmup=not args.no_warmup, action_mode=args.action_mode, compressor=args.compressor,
    )
    PolicyServer(policy, host=args.host, port=args.port).serve_forever()


if __name__ == "__main__":
    main()
