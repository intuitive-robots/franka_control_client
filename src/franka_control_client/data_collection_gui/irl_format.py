"""The IRL raw recording format: writing (live collection) and reading (conversion, GUI).

One episode is one folder (same layout as ``data_collection/irl_data_collection.py``, so the
existing tools in ``dataset_utils/`` and ``teach_replay_collection.Demo.load`` keep working)::

    <episode>/
      timestamps.pt                 [N] time.perf_counter() of every robot sample
      FrankaPanda/                  follower arm + Robotiq gripper, [N, ...] float64
        joint_pos.pt  joint_vel.pt  ee_pos.pt (O_T_EE, 16, column-major)
        external_joint_torque.pt  external_wrench.pt  gripper_state.pt  gripper_current.pt
        EE_pos.pt  EE_quat.pt       (when the arm reports them)
        gripper_command.pt          (kinesthetic / teach & replay: commanded gripper position)
      Gello/                        GELLO leader: joint_pos [N,7], gripper_state [N], gripper_command [N]
      MQ3/                          VR leader: EE_pos [N,3], EE_quat [N,4], gripper_state [N]
      Keyboard/                     keyboard leader (target): EE_pos, EE_quat, gripper_state, EE_vel [N,3]
      Traj/                         teach & replay, replay episodes: commands sent to the arm
      sensors/<camera>/000000.png   BGR PNG frames
      camera_timestamps.pt          {camera: [F] perf_counter of every frame}   (written by the GUI)
      metadata.json                 mode, task, rates, ...                       (written by the GUI)

Older recordings have no camera_timestamps.pt; their camera frames are assumed evenly spread
over the episode (rate = frames / duration), or at a rate given by the user.
"""

from __future__ import annotations

import json
import logging
import os
import re
import shutil
import sys
import threading
from concurrent.futures import Future, ThreadPoolExecutor, wait
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

LOG = logging.getLogger(__name__)

TIMESTAMPS = "timestamps.pt"
CAMERA_TIMESTAMPS = "camera_timestamps.pt"
METADATA = "metadata.json"
SENSORS = "sensors"
FOLLOWER_FILES = ("joint_pos.pt", "external_joint_torque.pt")
SKIP_DIRS = {SENSORS, "lerobot", "images", "videos", "data", "meta", "__pycache__"}


def _torch():
    import torch

    return torch


def _numeric_key(p: Path) -> Tuple[int, str]:
    digits = re.sub(r"\D", "", p.stem)
    return (int(digits) if digits else sys.maxsize, p.name)


def list_frames(cam_dir: Path) -> List[Path]:
    return sorted(cam_dir.glob("*.png"), key=_numeric_key)


# ===================================================================== writer
class IRLEpisodeWriter:
    """Collects one episode in memory (numeric) / on disk (PNG frames, written by a thread
    pool so the sampling loop never waits on disk) and saves it in the IRL format."""

    def __init__(
        self,
        record_dir: Path,
        follower_name: str,
        leader_name: Optional[str],
        cameras: Sequence[str],
        max_workers: Optional[int] = None,
        max_pending: int = 4096,
    ) -> None:
        self.record_dir = Path(record_dir)
        self.follower_name = follower_name
        self.leader_name = leader_name
        self.cameras = list(cameras)
        self.max_pending = max_pending
        self.timestamps: List[float] = []
        self.columns: Dict[str, List[Any]] = {}
        self.camera_times: Dict[str, List[float]] = {c: [] for c in self.cameras}
        self.dropped_frames: Dict[str, int] = {c: 0 for c in self.cameras}
        self.write_errors = 0
        self._next_idx: Dict[str, int] = {c: 0 for c in self.cameras}
        self._futures: List[Future] = []
        self._lock = threading.Lock()
        self._pool = ThreadPoolExecutor(max_workers=max_workers or max(2, min(8, os.cpu_count() or 4)))
        self.record_dir.mkdir(parents=True, exist_ok=False)
        for cam in self.cameras:
            (self.record_dir / SENSORS / cam).mkdir(parents=True)

    # ---------------------------------------------------------------- samples
    @property
    def n_samples(self) -> int:
        return len(self.timestamps)

    def add_sample(self, t: float, values: Dict[str, Any]) -> None:
        """``values``: "<device>/<file>.pt" -> row (all rows of one sample at once, so every
        column stays as long as timestamps)."""
        with self._lock:
            if not self.timestamps:
                for key in values:
                    self.columns.setdefault(key, [])
            elif set(values) != set(self.columns):
                raise ValueError(f"sample keys changed: {sorted(set(values) ^ set(self.columns))}")
            self.timestamps.append(float(t))
            for key, row in values.items():
                self.columns[key].append(row)

    def add_frame(self, camera: str, t: float, image_rgb: np.ndarray) -> bool:
        """Queue one RGB frame; False if dropped because the disk cannot keep up."""
        self._futures = [f for f in self._futures if not f.done()]
        if len(self._futures) >= self.max_pending:
            self.dropped_frames[camera] += 1
            return False
        idx = self._next_idx[camera]
        self._next_idx[camera] = idx + 1
        path = self.record_dir / SENSORS / camera / f"{idx:06d}.png"
        self.camera_times[camera].append(float(t))
        self._futures.append(self._pool.submit(self._write_png, path, image_rgb))
        return True

    def _write_png(self, path: Path, image_rgb: np.ndarray) -> None:
        import cv2

        try:
            if not cv2.imwrite(str(path), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)):
                raise IOError("cv2.imwrite returned False")
        except Exception as exc:
            self.write_errors += 1
            LOG.error("Failed to write %s: %s", path, exc)

    @property
    def pending_writes(self) -> int:
        return sum(1 for f in self._futures if not f.done())

    def frame_counts(self) -> Dict[str, int]:
        return {c: len(ts) for c, ts in self.camera_times.items()}

    def flush(self) -> None:
        if self._futures:
            wait(self._futures)
            self._futures = []

    # ------------------------------------------------------------------- save
    def arrays(self) -> Dict[str, np.ndarray]:
        with self._lock:
            return {k: np.asarray(v, dtype=np.float64) for k, v in self.columns.items()}

    def save(self, metadata: Dict[str, Any], extra: Optional[Dict[str, np.ndarray]] = None) -> Path:
        """Write everything; ``extra``: more "<dir>/<file>.pt" arrays (e.g. Traj/ of a replay)."""
        torch = _torch()
        self.flush()
        self._pool.shutdown(wait=True)
        root = self.record_dir
        torch.save(torch.tensor(self.timestamps, dtype=torch.float64), root / TIMESTAMPS)
        for key, arr in {**self.arrays(), **(extra or {})}.items():
            path = root / key
            path.parent.mkdir(parents=True, exist_ok=True)
            torch.save(torch.tensor(np.asarray(arr), dtype=torch.float64), path)
        if self.cameras:
            torch.save(
                {c: torch.tensor(ts, dtype=torch.float64) for c, ts in self.camera_times.items()},
                root / CAMERA_TIMESTAMPS,
            )
        (root / METADATA).write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        return root

    def discard(self) -> None:
        self.flush()
        self._pool.shutdown(wait=True)
        shutil.rmtree(self.record_dir, ignore_errors=True)


# ---------------------------------------------------------- state -> columns
def follower_columns(follower: str, arm: Dict[str, Any], gripper: Dict[str, Any], gripper_position: float) -> Dict[str, Any]:
    cols = {
        f"{follower}/joint_pos.pt": arm["q"],
        f"{follower}/joint_vel.pt": arm["dq"],
        f"{follower}/ee_pos.pt": arm["O_T_EE"],
        f"{follower}/external_joint_torque.pt": arm["tau_ext_hat_filtered"],
        f"{follower}/external_wrench.pt": arm["O_F_ext_hat_K"],
        f"{follower}/gripper_state.pt": gripper_position,
        f"{follower}/gripper_current.pt": float(gripper.get("current", 0.0)),
    }
    if "EE_pos" in arm and "EE_quat" in arm:
        cols[f"{follower}/EE_pos.pt"] = arm["EE_pos"]
        cols[f"{follower}/EE_quat.pt"] = arm["EE_quat"]
    return cols


def gello_columns(leader: str, state: Dict[str, Any], gripper: Dict[str, Any]) -> Dict[str, Any]:
    arm, grip = state["gello_arm_state"], state["gello_gripper_state"]
    return {
        f"{leader}/joint_pos.pt": np.asarray(arm["joint_state"], dtype=np.float64).reshape(-1)[:7],
        f"{leader}/gripper_state.pt": float(np.asarray(grip["gripper"], dtype=np.float64).reshape(-1)[0]),
        f"{leader}/gripper_command.pt": float(gripper.get("commanded_position", np.nan)),
    }


def mq3_columns(leader: str, signal: Dict[str, Any]) -> Dict[str, Any]:
    return {
        f"{leader}/EE_pos.pt": np.asarray(signal["pos"], dtype=np.float64).reshape(-1)[:3],
        f"{leader}/EE_quat.pt": np.asarray(signal["rot"], dtype=np.float64).reshape(-1)[:4],
        f"{leader}/gripper_state.pt": float(np.asarray(signal["gripper_width"], dtype=np.float64).reshape(-1)[0]),
    }


def keyboard_columns(leader: str, signal: Dict[str, Any]) -> Dict[str, Any]:
    cols = mq3_columns(leader, signal)
    cols[f"{leader}/EE_vel.pt"] = np.asarray(signal["pos_vel"], dtype=np.float64).reshape(-1)[:3]
    return cols


# ===================================================================== reader
def is_episode_dir(path: Path) -> bool:
    return (Path(path) / TIMESTAMPS).is_file()


def find_episodes(root: Path, max_depth: int = 4) -> List[Path]:
    """Episode folders at or below ``root`` (teach & replay: <task>/demo/*, <task>/replay/*)."""
    root = Path(root).expanduser()
    if is_episode_dir(root):
        return [root]
    found: List[Path] = []

    def walk(d: Path, depth: int) -> None:
        try:
            children = sorted(c for c in d.iterdir() if c.is_dir())
        except OSError:
            return
        for c in children:
            if c.name.startswith(".") or c.name in SKIP_DIRS:
                continue
            if is_episode_dir(c):
                found.append(c)
            elif depth < max_depth:
                walk(c, depth + 1)

    walk(root, 1)
    return found


@dataclass
class EpisodeLayout:
    path: Path
    follower: str
    leader: Optional[str]  # sub folder name
    leader_space: Optional[str]  # "joint" | "cartesian"
    cameras: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)


def _files(d: Path) -> set:
    return {c.name for c in d.iterdir() if c.is_file()}


def detect_layout(ep: Path) -> EpisodeLayout:
    ep = Path(ep)
    subdirs = sorted(d for d in ep.iterdir() if d.is_dir() and d.name != SENSORS)
    follower = None
    for d in subdirs:
        if set(FOLLOWER_FILES) <= _files(d):
            if follower is None or d.name == "FrankaPanda":
                follower = d.name
    if follower is None:
        raise ValueError(f"{ep}: no follower folder with {', '.join(FOLLOWER_FILES)}")
    leader, leader_space = None, None
    # a real leader device first, the replayed command trajectory (Traj/) otherwise
    for d in sorted((d for d in subdirs if d.name != follower), key=lambda d: d.name == "Traj"):
        files = _files(d)
        if "joint_pos.pt" in files and "gripper_state.pt" in files:
            leader, leader_space = d.name, "joint"
            break
        if {"EE_pos.pt", "EE_quat.pt", "gripper_state.pt"} <= files:
            leader, leader_space = d.name, "cartesian"
            break
    sensors = ep / SENSORS
    cameras = sorted(d.name for d in sensors.iterdir() if d.is_dir() and any(d.glob("*.png"))) if sensors.is_dir() else []
    meta: Dict[str, Any] = {}
    if (ep / METADATA).is_file():
        try:
            meta = json.loads((ep / METADATA).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            pass
    return EpisodeLayout(ep, follower, leader, leader_space, cameras, meta)


def load_tensor(path: Path) -> np.ndarray:
    torch = _torch()
    value = torch.load(path, map_location="cpu", weights_only=False)
    return value.numpy() if hasattr(value, "numpy") else np.asarray(value)


def load_camera_timestamps(ep: Path) -> Dict[str, np.ndarray]:
    path = Path(ep) / CAMERA_TIMESTAMPS
    if not path.is_file():
        return {}
    torch = _torch()
    data = torch.load(path, map_location="cpu", weights_only=False)
    return {k: np.asarray(v, dtype=np.float64).reshape(-1) for k, v in data.items()}


def summarize_episode(ep: Path, root: Optional[Path] = None) -> Dict[str, Any]:
    """Cheap overview for the GUI table (loads only the timestamps)."""
    ep = Path(ep)
    info: Dict[str, Any] = {"path": str(ep), "name": str(ep.relative_to(root)) if root and ep != root else ep.name}
    try:
        layout = detect_layout(ep)
        t = load_tensor(ep / TIMESTAMPS).reshape(-1)
        n = int(len(t))
        duration = float(t[-1] - t[0]) if n > 1 else 0.0
        cam_ts = load_camera_timestamps(ep)
        cams = {}
        for cam in layout.cameras:
            frames = len(list((ep / SENSORS / cam).glob("*.png")))
            ts = cam_ts.get(cam)
            if ts is not None and len(ts) > 1:
                hz = (len(ts) - 1) / max(ts[-1] - ts[0], 1e-9)
            else:
                hz = frames / duration if duration > 0 else None
            cams[cam] = {"frames": frames, "hz": hz, "timestamps": ts is not None}
        info.update(
            ok=True,
            samples=n,
            duration_s=duration,
            robot_hz=(n - 1) / duration if duration > 0 else None,
            follower=layout.follower,
            leader=layout.leader,
            leader_space=layout.leader_space,
            cameras=cams,
            mode=layout.metadata.get("mode"),
            kind=layout.metadata.get("kind"),
            task=layout.metadata.get("task"),
            task_instruction=layout.metadata.get("task_instruction"),
        )
    except Exception as exc:
        info.update(ok=False, error=f"{type(exc).__name__}: {exc}")
    return info
