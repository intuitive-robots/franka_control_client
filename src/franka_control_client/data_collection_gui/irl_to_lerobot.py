"""IRL raw recordings -> LeRobot dataset (lerobot 0.4, v3 format).

Used by the GUI (live, after every saved episode, and the Convert tab) and as a CLI::

    python -m franka_control_client.data_collection_gui.irl_to_lerobot \\
        /home/irl-admin/new_data_collection/cup_40hz --output /path/lerobot/cup --fps 15 \\
        [--state-space joint|cartesian] [--action leader|next_state] [--size 256 256] [--task "..."]

Timeline: the dataset frames are a grid ``t0 + k / fps`` over the robot timestamps. Frame k
uses the latest robot sample at or before its grid time, and from every camera the latest
frame captured up to that sample (frames are read in the same loop step as the robot state,
just after it). Camera times come from ``camera_timestamps.pt``; older recordings without
it are assumed to spread their frames evenly over the episode (or use ``--camera-hz``).
The episode ends with the stream that ends first.

Frame layout (the convention of ``dataset_utils/convert_data_to_lerobot_*.py``):

    observation.images.<camera>  "zed_" prefix removed: zed_right -> observation.images.right
    observation.state     (8)    joint: q0..q6, gripper | cartesian: x, y, z, qx, qy, qz, qw, gripper
    observation.force_torque (14) external joint torque (7), external wrench (6), gripper current
    action                (8)    leader:     the leader (GELLO joints / VR pose / replayed command)
                                             at the same sample, its gripper
                                 next_state: the follower state of the next frame, gripper command
                                             if recorded (kinesthetic), else the next gripper state

Gripper values are Robotiq closedness (0 open, 1 closed).
"""

from __future__ import annotations

import argparse
import copy
import json
import logging
import math
import shutil
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Iterator, List, Optional, Sequence, Tuple

import numpy as np

from .irl_format import (
    SENSORS,
    TIMESTAMPS,
    EpisodeLayout,
    detect_layout,
    find_episodes,
    list_frames,
    load_camera_timestamps,
    load_tensor,
)

LOG = logging.getLogger(__name__)

SOURCES_FILE = "irl_sources.json"  # meta/irl_sources.json: episode index -> IRL folder
DEFAULT_KEYS = {"timestamp", "frame_index", "episode_index", "index", "task_index"}
Progress = Callable[[str, Optional[float]], None]


class ConversionCancelled(Exception):
    pass


@dataclass
class ConvertOptions:
    fps: float = 15.0
    state_space: str = "auto"  # auto: cartesian if the leader is cartesian (VR), else joint
    action_source: str = "auto"  # auto: leader if it matches the state space, else next_state
    image_size: Optional[Tuple[int, int]] = (256, 256)  # (width, height); None = native
    use_videos: bool = True
    force_torque: bool = True
    cameras: Optional[List[str]] = None  # None: every camera of the first episode
    camera_hz: Dict[str, float] = field(default_factory=dict)  # for recordings without camera timestamps
    task: str = ""  # empty: metadata task_instruction > metadata task > folder name
    robot_type: str = "franka_panda"
    strip_zed_prefix: bool = True


# ================================================================== helpers
def image_key(camera: str, strip_zed_prefix: bool = True) -> str:
    name = camera[4:] if strip_zed_prefix and camera.startswith("zed_") else camera
    return f"observation.images.{name}"


def state_names(space: str) -> List[str]:
    if space == "joint":
        return [f"joint_{i}" for i in range(7)] + ["gripper"]
    return ["x", "y", "z", "qx", "qy", "qz", "qw", "gripper"]


def build_features(
    image_shapes: Dict[str, Tuple[int, int, int]], state_space: str, force_torque: bool, use_videos: bool
) -> Dict[str, dict]:
    """``image_shapes``: feature key -> (H, W, C)."""
    kind = "video" if use_videos else "image"
    features: Dict[str, dict] = {}
    for key, (h, w, c) in image_shapes.items():
        features[key] = {"dtype": kind, "shape": (c, h, w), "names": ["channel", "height", "width"]}
    names = state_names(state_space)
    features["observation.state"] = {"dtype": "float32", "shape": (8,), "names": names}
    if force_torque:
        features["observation.force_torque"] = {
            "dtype": "float32",
            "shape": (14,),
            "names": [f"external_joint_torque_{i}" for i in range(7)]
            + [f"external_wrench_{i}" for i in range(6)]
            + ["gripper_current"],
        }
    features["action"] = {"dtype": "float32", "shape": (8,), "names": names}
    return features


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrices [..., 3, 3] -> quaternions [..., 4] (x, y, z, w)."""
    from scipy.spatial.transform import Rotation

    return Rotation.from_matrix(R).as_quat()


def continuous_quat(q: np.ndarray, reference: Optional[np.ndarray] = None) -> np.ndarray:
    """Flip signs so consecutive quaternions (or each row vs ``reference``) stay in one hemisphere."""
    q = np.array(q, dtype=np.float64, copy=True)
    if reference is not None:
        q[np.sum(q * reference, axis=1) < 0] *= -1
        return q
    if len(q) and q[0, 3] < 0:
        q[0] *= -1
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] *= -1
    return q


def default_task(layout: EpisodeLayout) -> str:
    meta = layout.metadata
    if str(meta.get("task_instruction") or "").strip():
        return str(meta["task_instruction"]).strip()
    if str(meta.get("task") or "").strip():
        return str(meta["task"]).replace("_", " ").strip()
    parent = layout.path.parent
    if parent.name in ("demo", "replay"):
        parent = parent.parent
    return parent.name.replace("_", " ")


def resolve_spaces(layout: EpisodeLayout, opts: ConvertOptions) -> Tuple[str, str]:
    state = opts.state_space
    if state == "auto":
        state = "cartesian" if layout.leader_space == "cartesian" else "joint"
    action = opts.action_source
    if action == "auto":
        action = "leader" if layout.leader is not None and layout.leader_space == state else "next_state"
    if action == "leader":
        if layout.leader is None:
            raise ValueError("no leader device recorded (kinesthetic demo): use next_state actions")
        if layout.leader_space != state:
            raise ValueError(
                f"the leader '{layout.leader}' is {layout.leader_space}-space but the state is {state}: "
                f"choose the {layout.leader_space} state space or next_state actions"
            )
    return state, action


def _read_image(path: Path, size: Optional[Tuple[int, int]]) -> np.ndarray:
    import cv2

    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        raise IOError(f"cannot read {path}")
    if size is not None and (img.shape[1], img.shape[0]) != tuple(size):
        img = cv2.resize(img, tuple(size), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def _first_column(arr: np.ndarray) -> np.ndarray:
    """[N] or [N, k] -> [N] (gripper values are saved as [N] or [N, 1])."""
    arr = np.asarray(arr)
    return arr.reshape(len(arr), -1)[:, 0]


def latest_at(times: np.ndarray, query: np.ndarray) -> np.ndarray:
    """Index of the latest ``times`` entry <= every query (0 if none)."""
    return np.clip(np.searchsorted(times, query, side="right") - 1, 0, len(times) - 1)


# ================================================================== episode
class EpisodePlan:
    """One IRL episode mapped onto the dataset grid; iterate ``frames()`` to get LeRobot frames."""

    def __init__(self, ep: Path, opts: ConvertOptions, cameras: Optional[Sequence[str]] = None) -> None:
        self.layout = layout = detect_layout(ep)
        self.opts = opts
        self.state_space, self.action_source = resolve_spaces(layout, opts)
        self.task = opts.task.strip() or default_task(layout)
        self.cameras = list(cameras if cameras is not None else (opts.cameras or layout.cameras))
        missing = [c for c in self.cameras if c not in layout.cameras]
        if missing:
            raise ValueError(f"camera(s) {missing} not recorded in this episode (has {layout.cameras})")
        self.notes: List[str] = []

        ep = layout.path
        fol = ep / layout.follower
        t = load_tensor(ep / TIMESTAMPS).astype(np.float64).reshape(-1)
        cols: Dict[str, np.ndarray] = {
            "q": load_tensor(fol / "joint_pos.pt"),
            "gripper": _first_column(load_tensor(fol / "gripper_state.pt")),
        }
        if self.opts.force_torque:
            cols["tau"] = load_tensor(fol / "external_joint_torque.pt")
            cols["wrench"] = load_tensor(fol / "external_wrench.pt")
            cols["current"] = (
                _first_column(load_tensor(fol / "gripper_current.pt"))
                if (fol / "gripper_current.pt").is_file() else None
            )
        if self.state_space == "cartesian":
            if (fol / "EE_pos.pt").is_file() and (fol / "EE_quat.pt").is_file():
                cols["pos"] = load_tensor(fol / "EE_pos.pt").reshape(-1, 3)
                cols["quat"] = load_tensor(fol / "EE_quat.pt").reshape(-1, 4)
            else:  # O_T_EE, column-major 4x4
                T = load_tensor(fol / "ee_pos.pt").reshape(-1, 4, 4).transpose(0, 2, 1)
                cols["pos"] = T[:, :3, 3]
                cols["quat"] = rot_to_quat(T[:, :3, :3])
        if self.action_source == "next_state" and (fol / "gripper_command.pt").is_file():
            cols["gripper_cmd"] = _first_column(load_tensor(fol / "gripper_command.pt"))
        if self.action_source == "leader":
            lead = ep / layout.leader
            if layout.leader_space == "joint":
                cols["lead_arm"] = load_tensor(lead / "joint_pos.pt").reshape(-1, 7)
            else:
                cols["lead_pos"] = load_tensor(lead / "EE_pos.pt").reshape(-1, 3)
                cols["lead_quat"] = load_tensor(lead / "EE_quat.pt").reshape(-1, 4)
            cols["lead_grip"] = _first_column(load_tensor(lead / "gripper_state.pt"))

        # all robot columns are sampled with the timestamps; trim to the shortest
        n = min([len(t)] + [len(v) for v in cols.values() if v is not None])
        if n < len(t):
            self.notes.append(f"robot streams trimmed to {n} of {len(t)} samples")
        self.t = t[:n]
        self.cols = {k: (None if v is None else np.asarray(v, dtype=np.float64)[:n]) for k, v in cols.items()}
        if n < 2:
            raise ValueError(f"only {n} robot samples")
        if np.any(np.diff(self.t) < 0):
            raise ValueError("robot timestamps are not monotonic")
        if "quat" in self.cols:
            self.cols["quat"] = continuous_quat(self.cols["quat"])

        robot_dt = float(np.median(np.diff(self.t)))
        if robot_dt <= 0:
            raise ValueError("robot timestamps do not advance")
        self.robot_hz = 1.0 / robot_dt
        t0, ends = self.t[0], [self.t[-1] + robot_dt]

        # camera timelines
        cam_ts = load_camera_timestamps(ep)
        self.frame_files: Dict[str, List[Path]] = {}
        self.cam_times: Dict[str, np.ndarray] = {}
        self.camera_hz: Dict[str, float] = {}
        for cam in self.cameras:
            files = list_frames(ep / SENSORS / cam)
            ts = cam_ts.get(cam)
            if ts is not None and len(ts) >= 1:
                if len(ts) != len(files):
                    self.notes.append(f"{cam}: {len(files)} PNGs but {len(ts)} timestamps, using {min(len(ts), len(files))}")
                m = min(len(ts), len(files))
                files, ts = files[:m], np.asarray(ts[:m], dtype=np.float64)
                hz = (m - 1) / max(ts[-1] - ts[0], 1e-9) if m > 1 else self.robot_hz
            else:
                hz = float(opts.camera_hz.get(cam) or 0.0)
                if hz <= 0:  # evenly spread over the robot recording
                    hz = len(files) / (self.t[-1] - t0 + robot_dt)
                    self.notes.append(f"{cam}: no camera timestamps, assuming {hz:.2f} Hz")
                ts = t0 + np.arange(len(files)) / hz
            if len(files) == 0:
                raise ValueError(f"camera {cam} has no frames")
            self.frame_files[cam] = files
            self.cam_times[cam] = ts
            self.camera_hz[cam] = hz
            ends.append(ts[-1] + 1.0 / hz)

        # dataset grid: slot k -> robot sample, cameras: latest frame up to that sample
        fps = float(opts.fps)
        self.duration = float(min(ends) - t0)
        n_slots = int(math.floor(self.duration * fps + 1e-6))
        if n_slots < 3:
            raise ValueError(f"episode too short ({self.duration:.2f} s at {fps:g} fps)")
        slots = t0 + np.arange(n_slots) / fps
        self.robot_idx = latest_at(self.t, slots + 1e-9)
        tol = 0.5 * robot_dt  # frames read in the same loop step, just after the robot state
        self.cam_idx = {cam: latest_at(ts, self.t[self.robot_idx] + tol) for cam, ts in self.cam_times.items()}
        self.n_frames = n_slots - 1  # next_state actions need the following slot
        if fps > self.robot_hz * 1.05:
            self.notes.append(f"dataset fps {fps:g} > robot rate {self.robot_hz:.1f} Hz: samples repeat")
        self.repeated_robot = int(np.sum(np.diff(self.robot_idx[: self.n_frames]) == 0))
        self.repeated_cam = {c: int(np.sum(np.diff(i[: self.n_frames]) == 0)) for c, i in self.cam_idx.items()}

        self._states = self._state_rows(self.robot_idx)
        self._actions = self._action_rows()

    # ----------------------------------------------------------------- rows
    def _state_rows(self, idx: np.ndarray) -> np.ndarray:
        c = self.cols
        grip = c["gripper"][idx, None]
        if self.state_space == "joint":
            return np.concatenate([c["q"][idx][:, :7], grip], axis=1)
        return np.concatenate([c["pos"][idx], c["quat"][idx], grip], axis=1)

    def _action_rows(self) -> np.ndarray:
        c = self.cols
        if self.action_source == "leader":
            idx = self.robot_idx
            grip = c["lead_grip"][idx, None]
            if "lead_arm" in c:
                rows = np.concatenate([c["lead_arm"][idx], grip], axis=1)
            else:
                quat = continuous_quat(c["lead_quat"][idx], reference=self._states[:, 3:7])
                rows = np.concatenate([c["lead_pos"][idx], quat, grip], axis=1)
            return rows
        nxt = self.robot_idx[1:]
        rows = self._state_rows(nxt)
        if "gripper_cmd" in c:
            rows[:, 7] = c["gripper_cmd"][nxt]
        return np.concatenate([rows, rows[-1:]], axis=0)  # pad: same length as the slots

    def image_shapes(self) -> Dict[str, Tuple[int, int, int]]:
        shapes = {}
        for cam in self.cameras:
            img = _read_image(self.frame_files[cam][0], self.opts.image_size)
            shapes[image_key(cam, self.opts.strip_zed_prefix)] = tuple(img.shape)
        return shapes

    def features(self) -> Dict[str, dict]:
        return build_features(self.image_shapes(), self.state_space, self.opts.force_torque, self.opts.use_videos)

    def frames(self) -> Iterator[Dict[str, Any]]:
        c = self.cols
        cache: Dict[str, Tuple[int, np.ndarray]] = {}
        for k in range(self.n_frames):
            i = self.robot_idx[k]
            frame: Dict[str, Any] = {
                "observation.state": self._states[k].astype(np.float32),
                "action": self._actions[k].astype(np.float32),
                "task": self.task,
            }
            if self.opts.force_torque:
                current = c["current"][i] if c.get("current") is not None else 0.0
                frame["observation.force_torque"] = np.concatenate(
                    [c["tau"][i][:7], c["wrench"][i][:6], [current]]
                ).astype(np.float32)
            for cam in self.cameras:
                j = int(self.cam_idx[cam][k])
                if cam not in cache or cache[cam][0] != j:
                    cache[cam] = (j, _read_image(self.frame_files[cam][j], self.opts.image_size))
                frame[image_key(cam, self.opts.strip_zed_prefix)] = cache[cam][1]
            yield frame

    def summary(self) -> Dict[str, Any]:
        return {
            "source": str(self.layout.path),
            "frames": self.n_frames,
            "duration_s": round(self.duration, 3),
            "robot_hz": round(self.robot_hz, 2),
            "camera_hz": {k: round(v, 2) for k, v in self.camera_hz.items()},
            "repeated_robot_frames": self.repeated_robot,
            "repeated_camera_frames": self.repeated_cam,
            "state_space": self.state_space,
            "action_source": self.action_source,
            "task": self.task,
            "notes": self.notes,
        }


# ===================================================================== sink
def _feature_signature(features: Dict[str, dict]) -> Dict[str, Any]:
    return {
        k: [v.get("dtype"), list(v.get("shape") or []), v.get("names") if v.get("dtype") not in ("image", "video") else None]
        for k, v in sorted(features.items())
        if k not in DEFAULT_KEYS
    }


def dataset_exists(root: Path) -> bool:
    return (Path(root) / "meta" / "info.json").is_file()


class LeRobotSink:
    """Creates a dataset or appends to an existing one (lerobot's resume path); every episode
    is written and the parquet files closed in ``close()``, so call open/close around a batch.
    """

    def __init__(self, root: Path, fps: float, features: Dict[str, dict], robot_type: str, use_videos: bool,
                 repo_id: Optional[str] = None) -> None:
        self.root = Path(root).expanduser()
        self.repo_id = repo_id or f"local/{self.root.name}"
        self.fps = int(round(fps))
        self.features = copy.deepcopy(features)
        self.robot_type = robot_type
        self.use_videos = use_videos
        self.dataset: Any = None
        self.sources: List[Dict[str, Any]] = []

    def check_compatible(self) -> None:
        if not dataset_exists(self.root):
            return
        info = json.loads((self.root / "meta" / "info.json").read_text(encoding="utf-8"))
        problems = []
        if int(info.get("fps", -1)) != self.fps:
            problems.append(f"fps {info.get('fps')} vs {self.fps}")
        have, want = _feature_signature(info.get("features", {})), _feature_signature(self.features)
        for key in sorted(set(have) | set(want)):
            if have.get(key) != want.get(key):
                problems.append(f"{key}: dataset {have.get(key)} vs new {want.get(key)}")
        if problems:
            raise ValueError(
                f"{self.root} exists with different settings, cannot append ("
                + "; ".join(problems[:6]) + "). Choose another output folder or overwrite."
            )

    def open(self) -> None:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        if dataset_exists(self.root):
            self.check_compatible()
            self.dataset = LeRobotDataset(self.repo_id, root=self.root)
            if any(f["dtype"] in ("image", "video") for f in self.features.values()):
                self.dataset.start_image_writer(num_processes=0, num_threads=4)
        else:
            if self.root.exists() and any(self.root.iterdir()):
                raise ValueError(f"{self.root} exists and is not a LeRobot dataset")
            self.root.parent.mkdir(parents=True, exist_ok=True)
            if self.root.exists():
                self.root.rmdir()  # create() wants to make it
            self.dataset = LeRobotDataset.create(
                repo_id=self.repo_id,
                fps=self.fps,
                features=copy.deepcopy(self.features),
                root=self.root,
                robot_type=self.robot_type,
                use_videos=self.use_videos,
                image_writer_threads=4,
            )
        self.dataset.meta.metadata_buffer_size = 1

    @property
    def num_episodes(self) -> int:
        return int(self.dataset.meta.total_episodes) if self.dataset is not None else 0

    def add_episode(self, plan: EpisodePlan, progress: Optional[Progress] = None,
                    cancel: Optional[threading.Event] = None) -> int:
        """Write one episode; returns its index in the dataset."""
        ds = self.dataset
        index = self.num_episodes
        n = plan.n_frames
        try:
            for k, frame in enumerate(plan.frames()):
                if cancel is not None and cancel.is_set():
                    raise ConversionCancelled()
                ds.add_frame(frame)
                if progress and k % 25 == 0:
                    progress(f"{plan.layout.path.name}: frame {k}/{n}", k / max(n, 1))
            if progress:
                progress(f"{plan.layout.path.name}: encoding {n} frames...", 1.0)
            ds.save_episode()
        except BaseException:
            try:
                ds.clear_episode_buffer()
            except Exception:
                pass
            raise
        self.sources.append({"episode_index": index, **plan.summary()})
        return index

    def close(self) -> None:
        if self.dataset is None:
            return
        try:
            self.dataset.finalize()
        finally:
            try:
                if getattr(self.dataset, "image_writer", None) is not None:
                    self.dataset.stop_image_writer()
            except Exception:
                pass
            self.dataset = None
        if self.sources:
            path = self.root / "meta" / SOURCES_FILE
            old = json.loads(path.read_text(encoding="utf-8")) if path.is_file() else []
            path.write_text(json.dumps(old + self.sources, indent=1), encoding="utf-8")
            self.sources = []
        images = self.root / "images"
        if images.exists() and not any(p.is_file() for p in images.rglob("*")):
            shutil.rmtree(images, ignore_errors=True)  # empty scratch folders of the image writer


# ============================================================ batch convert
def convert(
    episodes: Sequence[Path],
    output: Path,
    opts: ConvertOptions,
    existing: str = "fail",  # fail | append | overwrite
    progress: Optional[Progress] = None,
    cancel: Optional[threading.Event] = None,
    repo_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Convert IRL episodes into one LeRobot dataset; bad episodes are skipped and reported."""
    progress = progress or (lambda msg, frac: None)
    output = Path(output).expanduser().resolve()
    episodes = [Path(e) for e in episodes]
    if not episodes:
        raise ValueError("no episodes selected")
    for ep in episodes:
        if output == ep.resolve() or output in ep.resolve().parents or ep.resolve() in output.parents:
            raise ValueError("the output must not be inside (or contain) a source episode")
    if dataset_exists(output) or (output.exists() and any(output.iterdir())):
        if existing == "overwrite":
            shutil.rmtree(output)
        elif existing != "append":
            raise ValueError(f"{output} already exists: choose append or overwrite")

    # the first episode that plans fine defines the features
    first, skipped = None, []
    for ep in episodes:
        try:
            first = EpisodePlan(ep, opts)
            break
        except Exception as exc:
            skipped.append({"source": str(ep), "error": f"{type(exc).__name__}: {exc}"})
    if first is None:
        raise ValueError(f"no convertible episode: {skipped[0]['error'] if skipped else ''}")
    cameras = first.cameras
    features = first.features()
    signature = _feature_signature(features)  # create() adds keys to the dict it gets
    sink = LeRobotSink(output, opts.fps, copy.deepcopy(features), opts.robot_type, opts.use_videos, repo_id)
    sink.open()
    written: List[Dict[str, Any]] = []
    total = len(episodes)
    try:
        for n, ep in enumerate(episodes):
            if cancel is not None and cancel.is_set():
                raise ConversionCancelled()
            if any(s["source"] == str(ep) for s in skipped):
                continue
            try:
                plan = first if ep == first.layout.path else EpisodePlan(ep, opts, cameras=cameras)
                if _feature_signature(plan.features()) != signature:
                    raise ValueError("different image sizes / features than the first episode")
                frac0 = n / total

                def episode_progress(msg: str, frac: Optional[float], n=n, frac0=frac0) -> None:
                    progress(f"[{n + 1}/{total}] {msg}", frac0 + (frac or 0.0) / total)

                index = sink.add_episode(plan, episode_progress, cancel)
                written.append({"episode_index": index, **plan.summary()})
                LOG.info("[OK] %s -> episode %d (%d frames)", ep, index, plan.n_frames)
            except ConversionCancelled:
                raise
            except Exception as exc:
                LOG.warning("[SKIP] %s: %s", ep, exc)
                skipped.append({"source": str(ep), "error": f"{type(exc).__name__}: {exc}"})
    finally:
        sink.close()
    progress(f"Done: {len(written)} episodes written, {len(skipped)} skipped", 1.0)
    return {
        "output": str(output),
        "repo_id": sink.repo_id,
        "written": written,
        "skipped": skipped,
        "frames": int(sum(w["frames"] for w in written)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", type=Path, help="IRL task folder (or one episode)")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--state-space", choices=("auto", "joint", "cartesian"), default="auto")
    parser.add_argument("--action", choices=("auto", "leader", "next_state"), default="auto")
    parser.add_argument("--size", type=int, nargs=2, metavar=("W", "H"), default=(256, 256),
                        help="image size; 0 0 = native")
    parser.add_argument("--images", action="store_true", help="PNG images instead of videos")
    parser.add_argument("--no-force-torque", action="store_true")
    parser.add_argument("--camera", action="append", help="repeatable; default: all")
    parser.add_argument("--camera-hz", action="append", default=[], metavar="NAME=HZ",
                        help="rate of a camera in recordings without camera timestamps")
    parser.add_argument("--task", default="")
    parser.add_argument("--robot-type", default="franka_panda")
    parser.add_argument("--append", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    opts = ConvertOptions(
        fps=args.fps,
        state_space=args.state_space,
        action_source=args.action,
        image_size=tuple(args.size) if all(v > 0 for v in args.size) else None,
        use_videos=not args.images,
        force_torque=not args.no_force_torque,
        cameras=args.camera,
        camera_hz={k: float(v) for k, v in (s.split("=", 1) for s in args.camera_hz)},
        task=args.task,
        robot_type=args.robot_type,
    )
    episodes = find_episodes(args.source)
    print(f"{len(episodes)} episodes in {args.source}")
    result = convert(
        episodes, args.output, opts,
        existing="overwrite" if args.overwrite else "append" if args.append else "fail",
        progress=lambda msg, frac: print(f"\r{msg:<80}", end="", flush=True),
    )
    print()
    print(json.dumps({k: v for k, v in result.items() if k != "written"}, indent=1))


if __name__ == "__main__":
    main()
