"""Records one rollout and saves it under ``<save_root>/<batch>/<NNN>_<name>/``.

Layout of a saved rollout::

    meta.json        name, description, outcome, task, config, policy info, stats
    episode.npz      all numeric streams (schema in meta.json["arrays"])
    cam_<name>.mp4   frames sent to the policy (if record_images), + timestamps in npz

and ``<save_root>/<batch>/index.jsonl`` gets one summary line per saved rollout,
``<save_root>/<batch>/batch.json`` holds the batch name/description/config.
"""

from __future__ import annotations

import datetime as _dt
import json
import logging
import queue
import re
import shutil
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np

try:  # cv2 is only needed to write camera videos
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

LOG = logging.getLogger(__name__)

ARRAY_SCHEMA = {
    "state/t": "wall time [s] of each robot state sample (state_record_hz)",
    "state/q": "measured joint positions [N,7] rad",
    "state/dq": "measured joint velocities [N,7] rad/s",
    "state/q_d": "controller desired joint positions reported by the robot [N,7]",
    "state/ee_pos": "measured EE position [N,3] m, base frame",
    "state/ee_quat": "measured EE orientation [N,4] (x,y,z,w)",
    "state/gripper": "gripper closedness [N] (0 open, 1 closed)",
    "state/tau_ext": "estimated external joint torques [N,7] Nm",
    "policy/t_obs": "time the observation was captured [M]",
    "policy/t_recv": "time the action chunk arrived [M]",
    "policy/rtt_s": "robot-side round-trip time of the infer call [M]",
    "policy/server_infer_s": "server-reported model time [M] (NaN if not reported)",
    "policy/obs_state": "observation.state sent to the policy [M,S]",
    "policy/action_raw": "action chunk as returned by the policy, NaN-padded [M,T,D]",
    "policy/action_exec": "rows queued to the control pair after slicing/resampling, NaN-padded [M,T',D]",
    "policy/skipped_rows": "stale rows dropped from the chunk (async mode) [M]",
    "control/t": "wall time of every control-loop step",
    "command/arm_t": "wall time of each arm command sent by the controller",
    "command/arm": "arm commands: joint targets [K,7] or EE pose xyz+quat [K,7]",
    "command/gripper_t": "wall time of each gripper command",
    "command/gripper": "gripper command [G] (closedness for Robotiq, width for Franka hand)",
    "camera/<name>/t": "capture time of each recorded frame (matches the mp4 frame order)",
}


def slugify(text: str) -> str:
    text = re.sub(r"[^\w\-]+", "_", text.strip(), flags=re.UNICODE).strip("_")
    return text[:60] or "rollout"


def _pad_stack(chunks: List[np.ndarray]) -> np.ndarray:
    if not chunks:
        return np.zeros((0, 0, 0))
    t_max = max(c.shape[0] for c in chunks)
    d_max = max(c.shape[1] for c in chunks)
    out = np.full((len(chunks), t_max, d_max), np.nan)
    for i, c in enumerate(chunks):
        out[i, : c.shape[0], : c.shape[1]] = c
    return out


def _stack(rows: List[Sequence[float]], width: int) -> np.ndarray:
    if not rows:
        return np.zeros((0, width))
    return np.asarray(rows, dtype=np.float64)


class _VideoSink:
    """Background mp4 writer so encoding never blocks the inference loop."""

    def __init__(self, directory: Path, fps: float) -> None:
        self.directory = directory
        self.fps = max(1.0, float(fps))
        self._queue: "queue.Queue[Optional[tuple]]" = queue.Queue(maxsize=256)
        self._writers: Dict[str, Any] = {}
        self.timestamps: Dict[str, List[float]] = {}
        self.dropped = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def put(self, t: float, frames: Dict[str, np.ndarray]) -> None:
        try:
            self._queue.put_nowait((t, frames))
        except queue.Full:
            self.dropped += 1

    def close(self) -> None:
        self._queue.put(None)
        self._thread.join(timeout=30)
        for writer in self._writers.values():
            writer.release()
        self._writers.clear()

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            if item is None:
                return
            t, frames = item
            for name, frame in frames.items():
                try:
                    writer = self._writers.get(name)
                    if writer is None:
                        h, w = frame.shape[:2]
                        path = str(self.directory / f"cam_{name}.mp4")
                        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
                        self._writers[name] = writer
                        self.timestamps[name] = []
                    writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                    self.timestamps[name].append(t)
                except Exception:  # never kill the writer thread
                    LOG.exception("Failed to write frame of camera %s", name)


class EpisodeRecorder:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.active = False
        self._tmp_dir: Optional[Path] = None
        self._video: Optional[_VideoSink] = None
        self.dataset_writer: Any = None  # lerobot_export.RolloutDatasetWriter, if enabled
        self._clear()

    @property
    def tmp_dir(self) -> Optional[Path]:
        return self._tmp_dir

    # ------------------------------------------------------------ lifecycle
    def start(
        self,
        batch_dir: Path,
        episode_index: int,
        record_images: bool,
        video_fps: float,
        context: Dict[str, Any],
    ) -> None:
        with self._lock:
            self._clear()
            self.batch_dir = Path(batch_dir)
            self.episode_index = episode_index
            self.context = context
            self.t_start = time.time()
            self._tmp_dir = self.batch_dir / f".tmp_{uuid.uuid4().hex[:8]}"
            self._tmp_dir.mkdir(parents=True, exist_ok=True)
            if record_images and cv2 is not None:
                self._video = _VideoSink(self._tmp_dir, video_fps)
            elif record_images:
                LOG.warning("cv2 not available: camera frames will not be recorded")
            self.active = True

    def stop(self, reason: str) -> None:
        with self._lock:
            if not self.active:
                return
            self.active = False
            self.t_end = time.time()
            self.stop_reason = reason
        if self._video is not None:
            self._video.close()

    def discard(self) -> None:
        self.stop("discarded")
        if self.dataset_writer is not None:
            try:
                self.dataset_writer.abort()
            except Exception:
                LOG.exception("Aborting the rollout dataset failed")
            self.dataset_writer = None
        if self._tmp_dir is not None:
            shutil.rmtree(self._tmp_dir, ignore_errors=True)
        self._tmp_dir = None
        self._video = None
        with self._lock:
            self._clear()

    @property
    def has_data(self) -> bool:
        return self._tmp_dir is not None

    # --------------------------------------------------------------- record
    def add_state(self, t: float, row: Dict[str, Sequence[float]]) -> None:
        with self._lock:
            if not self.active:
                return
            self._state_t.append(t)
            for key, store in self._state.items():
                store.append(row[key])

    def add_control_step(self, t: float) -> None:
        if self.active:
            self._control_t.append(t)

    def add_command(self, t: float, kind: str, values: Sequence[float]) -> None:
        with self._lock:
            if not self.active:
                return
            if kind == "gripper":
                self._grip_t.append(t)
                self._grip.append(float(values[0]))
            else:
                self._arm_t.append(t)
                self._arm.append(list(values)[:7])

    def add_inference(self, rec: Dict[str, Any]) -> None:
        with self._lock:
            if not self.active:
                return
            self._policy.append(rec)

    def add_frames(self, t: float, frames: Dict[str, np.ndarray]) -> None:
        if self.active and self._video is not None and frames:
            self._video.put(t, frames)

    # ----------------------------------------------------------------- save
    def summary(self) -> Dict[str, Any]:
        with self._lock:
            rtt = np.asarray([p["rtt_s"] for p in self._policy], dtype=np.float64)
            model = np.asarray([p["server_infer_s"] for p in self._policy], dtype=np.float64)
            control_t = np.asarray(self._control_t, dtype=np.float64)
            end = self.t_end if self.t_end is not None else time.time()
            duration = end - self.t_start if self.t_start else 0.0

            def stat(arr: np.ndarray, fn) -> Optional[float]:
                arr = arr[np.isfinite(arr)]
                return None if arr.size == 0 else round(float(fn(arr)) * 1000.0, 2)

            dt = np.diff(control_t)
            return {
                "duration_s": round(duration, 3),
                "n_inference_calls": len(self._policy),
                "n_state_samples": len(self._state_t),
                "n_control_steps": int(control_t.size),
                "n_arm_commands": len(self._arm_t),
                "n_gripper_commands": len(self._grip_t),
                "latency_ms_p50": stat(rtt, np.median),
                "latency_ms_p95": stat(rtt, lambda a: np.percentile(a, 95)),
                "model_ms_p50": stat(model, np.median),
                "inference_hz_mean": round(len(self._policy) / duration, 3) if duration > 0 else None,
                "control_hz_mean": round(1.0 / float(np.mean(dt)), 3) if dt.size else None,
                "control_dt_ms_p95": round(float(np.percentile(dt, 95)) * 1000, 2) if dt.size else None,
                "dropped_video_frames": self._video.dropped if self._video is not None else 0,
                "stop_reason": self.stop_reason,
            }

    def save(
        self,
        name: str,
        description: str,
        outcome: str,
        notes: Dict[str, Any],
        progress: Callable[[str], None] = lambda msg: None,
    ) -> Path:
        if self._tmp_dir is None:
            raise RuntimeError("Nothing recorded to save.")
        self.stop("saved")
        summary = self.summary()
        dataset_info: Optional[Dict[str, Any]] = None
        if self.dataset_writer is not None:
            writer, self.dataset_writer = self.dataset_writer, None
            progress("Writing the LeRobot episode (encoding videos)...")
            frames = writer.finish()
            dataset_info = {
                "path": "lerobot" if frames else None,
                "repo_id": writer.repo_id,
                "fps": writer.fps,
                "frames": frames,
                "duplicated_frames": writer.duplicated,
                "features": sorted(writer.features),
                "warning": writer.error,
            }
        folder = self.batch_dir / f"{self.episode_index:03d}_{slugify(name)}"
        if folder.exists():
            folder = self.batch_dir / f"{self.episode_index:03d}_{slugify(name)}_{uuid.uuid4().hex[:4]}"

        with self._lock:
            arrays: Dict[str, np.ndarray] = {
                "state/t": np.asarray(self._state_t, dtype=np.float64),
                "control/t": np.asarray(self._control_t, dtype=np.float64),
                "command/arm_t": np.asarray(self._arm_t, dtype=np.float64),
                "command/arm": _stack(self._arm, 7),
                "command/gripper_t": np.asarray(self._grip_t, dtype=np.float64),
                "command/gripper": np.asarray(self._grip, dtype=np.float64),
            }
            widths = {"q": 7, "dq": 7, "q_d": 7, "ee_pos": 3, "ee_quat": 4, "tau_ext": 7}
            for key, store in self._state.items():
                if key == "gripper":
                    arrays["state/gripper"] = np.asarray(store, dtype=np.float64)
                else:
                    arrays[f"state/{key}"] = _stack(store, widths[key])
            pol = self._policy
            arrays.update(
                {
                    "policy/t_obs": np.asarray([p["t_obs"] for p in pol], dtype=np.float64),
                    "policy/t_recv": np.asarray([p["t_recv"] for p in pol], dtype=np.float64),
                    "policy/rtt_s": np.asarray([p["rtt_s"] for p in pol], dtype=np.float64),
                    "policy/server_infer_s": np.asarray([p["server_infer_s"] for p in pol], dtype=np.float64),
                    "policy/skipped_rows": np.asarray([p["skipped_rows"] for p in pol], dtype=np.int64),
                    "policy/obs_state": _stack([p["obs_state"] for p in pol], 8),
                    "policy/action_raw": _pad_stack([p["action_raw"] for p in pol]),
                    "policy/action_exec": _pad_stack([p["action_exec"] for p in pol]),
                }
            )
            if self._video is not None:
                for cam, ts in self._video.timestamps.items():
                    arrays[f"camera/{cam}/t"] = np.asarray(ts, dtype=np.float64)

        np.savez_compressed(self._tmp_dir / "episode.npz", **arrays)
        meta = {
            "name": name,
            "description": description,
            "outcome": outcome,
            "episode_index": self.episode_index,
            "start_time": _dt.datetime.fromtimestamp(self.t_start).isoformat(timespec="seconds"),
            "start_unix": self.t_start,
            "end_unix": self.t_end,
            "stats": summary,
            "notes": notes,
            "lerobot": dataset_info,
            **self.context,
            "arrays": {k: v for k, v in ARRAY_SCHEMA.items()},
            "files": sorted(p.name for p in self._tmp_dir.iterdir()) + ["meta.json"],
        }
        with open(self._tmp_dir / "meta.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2, default=_json_default)
        self._tmp_dir.rename(folder)
        self._tmp_dir = None
        self._video = None

        index_line = {
            "folder": folder.name,
            "name": name,
            "outcome": outcome,
            "description": description,
            "start_time": meta["start_time"],
            "task": self.context.get("task"),
            "lerobot_frames": dataset_info["frames"] if dataset_info else None,
            **{k: summary[k] for k in ("duration_s", "n_inference_calls", "latency_ms_p50", "control_hz_mean", "stop_reason")},
        }
        with open(self.batch_dir / "index.jsonl", "a", encoding="utf-8") as f:
            f.write(json.dumps(index_line, default=_json_default) + "\n")
        with self._lock:
            self._clear()
        return folder

    # ------------------------------------------------------------ internals
    def _clear(self) -> None:
        self.batch_dir: Path = Path(".")
        self.episode_index = 0
        self.context: Dict[str, Any] = {}
        self.t_start: float = 0.0
        self.t_end: Optional[float] = None
        self.stop_reason: str = ""
        self._state_t: List[float] = []
        self._state: Dict[str, List[Any]] = {k: [] for k in ("q", "dq", "q_d", "ee_pos", "ee_quat", "gripper", "tau_ext")}
        self._control_t: List[float] = []
        self._arm_t: List[float] = []
        self._arm: List[List[float]] = []
        self._grip_t: List[float] = []
        self._grip: List[float] = []
        self._policy: List[Dict[str, Any]] = []


def _json_default(obj: Any) -> Any:
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    if isinstance(obj, Path):
        return str(obj)
    return str(obj)


def read_batch_index(batch_dir: Path) -> List[Dict[str, Any]]:
    path = Path(batch_dir) / "index.jsonl"
    if not path.exists():
        return []
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    return rows


def update_saved_rollout(batch_dir: Path, folder: str, fields: Dict[str, Any]) -> Dict[str, Any]:
    """Edit description / outcome of an already saved rollout (meta.json + index.jsonl)."""
    batch_dir = Path(batch_dir)
    target = (batch_dir / folder).resolve()
    if target.parent != batch_dir.resolve() or not (target / "meta.json").exists():
        raise ValueError(f"No saved rollout {folder!r} in this batch.")
    allowed = {k: v for k, v in fields.items() if k in ("description", "outcome")}
    meta_path = target / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    meta.update(allowed)
    meta.setdefault("edits", []).append({"time": _dt.datetime.now().isoformat(timespec="seconds"), **allowed})
    meta_path.write_text(json.dumps(meta, indent=2, default=_json_default), encoding="utf-8")
    rows = read_batch_index(batch_dir)
    for row in rows:
        if row.get("folder") == folder:
            row.update(allowed)
    _write_batch_index(batch_dir, rows)
    return allowed


TRASH_DIR = ".trash"


def _write_batch_index(batch_dir: Path, rows: List[Dict[str, Any]]) -> None:
    tmp = batch_dir / "index.jsonl.tmp"
    tmp.write_text("".join(json.dumps(r, default=_json_default) + "\n" for r in rows), encoding="utf-8")
    tmp.replace(batch_dir / "index.jsonl")


def delete_saved_rollout(batch_dir: Path, folder: str) -> Path:
    """Remove a saved rollout from the batch: its folder is moved to ``<batch>/.trash/``
    (recoverable by moving it back and re-adding its index line) and its line is
    dropped from index.jsonl, so it no longer shows in the GUI or in merged datasets."""
    batch_dir = Path(batch_dir)
    target = (batch_dir / folder).resolve()
    if not folder or target.parent != batch_dir.resolve() or not target.is_dir() or folder.startswith("."):
        raise ValueError(f"No saved rollout {folder!r} in this batch.")
    trash = batch_dir / TRASH_DIR
    trash.mkdir(exist_ok=True)
    dest = trash / f"{folder}__deleted_{_dt.datetime.now().strftime('%Y%m%d-%H%M%S')}"
    target.rename(dest)
    rows = read_batch_index(batch_dir)
    _write_batch_index(batch_dir, [r for r in rows if r.get("folder") != folder])
    return dest


def next_episode_index(batch_dir: Path) -> int:
    batch_dir = Path(batch_dir)
    best = -1
    # Deleted rollouts (in .trash) still count, so episode numbers are never reused.
    for d in (batch_dir, batch_dir / TRASH_DIR):
        if not d.exists():
            continue
        for p in d.iterdir():
            m = re.match(r"^(\d+)_", p.name)
            if p.is_dir() and m:
                best = max(best, int(m.group(1)))
    return best + 1
