"""Rollouts as LeRobot datasets.

Per rollout (``save_lerobot`` on): a sampler in the session captures, at the
dataset rate, every camera, the robot state and what the controller is playing
into a memory buffer; when the rollout is saved, :class:`RolloutDatasetWriter`
resamples it onto an exact fps grid and writes a one-episode LeRobot dataset
in ``<rollout folder>/lerobot/``. One rollout = one folder, so deleting a
rollout is deleting its folder and re-labelling never touches the dataset.

Frame layout (the convention of ``dataset_utils/convert_data_to_lerobot_*.py``):

    observation.images.<key>   the camera under the same key the policy gets
    observation.state          joint pair: q0..q6, gripper  | cartesian: xyz, quat xyzw, gripper
    observation.force_torque   tau_ext(7), O_F_ext_hat_K(6), gripper current  (optional)
    action                     what the controller played at the NEXT frame:
                               joint targets / EE pose + gripper command (1 = closed)

Merging (GUI button, or ``python -m franka_control_client.inference_gui.lerobot_export``)
aggregates the rollouts of a batch, filtered by outcome, into one dataset whose
``meta/rollouts.json`` maps every episode index to its rollout folder, outcome
and description.
"""

from __future__ import annotations

import argparse
import json
import logging
import shutil
import threading
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

LOG = logging.getLogger(__name__)

DATASET_DIR = "lerobot"
OUTCOMES = ("success", "partial", "failure", "unlabeled")


def lerobot_available() -> Optional[str]:
    """None if lerobot imports, else the reason it does not."""
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: F401
    except Exception as exc:  # pragma: no cover - depends on the env
        return f"{type(exc).__name__}: {exc}"
    return None


def state_names(control_pair: str) -> List[str]:
    if control_pair == "joint":
        return [f"joint_{i}" for i in range(7)] + ["gripper"]
    return ["x", "y", "z", "qx", "qy", "qz", "qw", "gripper"]


def build_features(
    control_pair: str,
    image_shapes: Dict[str, Tuple[int, int, int]],
    use_videos: bool,
    force_torque: bool,
) -> Dict[str, dict]:
    """``image_shapes``: feature key -> (H, W, C) of the stored frames."""
    kind = "video" if use_videos else "image"
    features: Dict[str, dict] = {}
    for key, (h, w, c) in image_shapes.items():
        features[key] = {"dtype": kind, "shape": (c, h, w), "names": ["channel", "height", "width"]}
    names = state_names(control_pair)
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


def resize_frame(frame: np.ndarray, size: Optional[Tuple[int, int]]) -> np.ndarray:
    """``size`` = (width, height); None keeps the native resolution."""
    if size is None or (frame.shape[1], frame.shape[0]) == size:
        return frame
    import cv2

    return cv2.resize(frame, size, interpolation=cv2.INTER_AREA)


def mem_available_bytes() -> Optional[int]:
    """MemAvailable from /proc/meminfo (Linux); None elsewhere."""
    try:
        with open("/proc/meminfo", "r", encoding="ascii") as f:
            for line in f:
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) * 1024
    except OSError:
        pass
    return None


class RolloutDatasetWriter:
    """Buffers one rollout in memory and writes it as a one-episode LeRobot dataset
    when the rollout is saved.

    During the rollout ``add(t, observation, command)`` only stores references
    (images HWC, as captured), so recording costs ~nothing in real time; all the
    LeRobot work (add_frame, video encoding) runs in ``finish()``. There the
    samples are resampled onto an exact grid t0 + k / fps from the first sample to
    ``t_end``, so the episode has the real duration (frames = duration * fps)
    whatever the sampling jitter. Slot k holds the latest sample at or before its
    time; its action is the command at slot k + 1 (as in the converters, where the
    action is the leader one step later)."""

    def __init__(
        self,
        root: Path,
        repo_id: str,
        fps: float,
        features: Dict[str, dict],
        task: str,
        robot_type: str,
        use_videos: bool,
    ) -> None:
        self.root = Path(root)
        self.repo_id = repo_id
        self.fps = float(fps)
        self.task = task or "rollout"
        self.features = features
        self.robot_type = robot_type
        self.use_videos = use_videos
        self.n_frames = 0  # frames written by finish()
        self.duplicated = 0  # grid slots that repeat the previous sample
        self.error: Optional[str] = None
        self.t_end: Optional[float] = None
        self.nbytes = 0
        self._lock = threading.Lock()
        self._t: List[float] = []
        self._obs: List[Dict[str, Any]] = []
        self._cmd: List[np.ndarray] = []

    @property
    def n_samples(self) -> int:
        return len(self._t)

    def add(self, t: float, observation: Dict[str, Any], command: np.ndarray) -> None:
        size = sum(v.nbytes for v in observation.values() if isinstance(v, np.ndarray))
        with self._lock:
            self._t.append(float(t))
            self._obs.append(observation)
            self._cmd.append(np.asarray(command, dtype=np.float32))
            self.nbytes += size

    def grid(self, times: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
        """Sample index of every grid slot, and of the slot after it (the action)."""
        ts = np.asarray(times, dtype=np.float64)
        t_end = max(self.t_end if self.t_end is not None else ts[-1], ts[-1])
        n = max(int(round((t_end - ts[0]) * self.fps)), 1)
        slots = ts[0] + np.arange(n + 1) / self.fps
        idx = np.clip(np.searchsorted(ts, slots + 1e-9, side="right") - 1, 0, len(ts) - 1)
        return idx[:-1], idx[1:]

    def finish(self, progress: Callable[[str], None] = lambda msg: None) -> int:
        """Write the episode; returns the number of frames (0: nothing written)."""
        with self._lock:
            ts, obs, cmd = self._t, self._obs, self._cmd
            self._t, self._obs, self._cmd = [], [], []
        if len(ts) < 2:
            LOG.warning("Rollout dataset has %d samples; not writing it.", len(ts))
            return 0
        frame_idx, action_idx = self.grid(ts)
        self.duplicated = int(np.sum(frame_idx[1:] == frame_idx[:-1]))
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        dataset = LeRobotDataset.create(
            repo_id=self.repo_id,
            fps=int(round(self.fps)),
            features=self.features,
            root=self.root,
            robot_type=self.robot_type,
            use_videos=self.use_videos,
            image_writer_threads=4,
        )
        image_keys = [k for k, f in self.features.items() if f["dtype"] in ("image", "video")]
        n = len(frame_idx)
        try:
            for j, (i, a) in enumerate(zip(frame_idx, action_idx)):
                if j % 50 == 0:
                    progress(f"Writing the LeRobot episode: frame {j}/{n}...")
                src = obs[i]
                frame = {k: v for k, v in src.items() if k not in image_keys}
                for k in image_keys:
                    frame[k] = np.ascontiguousarray(np.transpose(src[k], (2, 0, 1)))
                frame["action"] = cmd[a]
                frame["task"] = self.task
                dataset.add_frame(frame)
                self.n_frames += 1
            del obs  # free the image buffer before encoding
            progress(f"Encoding the LeRobot videos ({n} frames)...")
            dataset.save_episode()
            dataset.finalize()
        except Exception as exc:
            self.error = f"writing the LeRobot episode failed: {exc}"
            try:
                if getattr(dataset, "image_writer", None) is not None:
                    dataset.stop_image_writer()
            except Exception:
                pass
            raise
        images = self.root / "images"
        if images.exists() and not any(p.is_file() for p in images.rglob("*")):
            shutil.rmtree(images, ignore_errors=True)  # the image writer's empty scratch folders
        return self.n_frames

    def abort(self) -> None:
        with self._lock:
            self._t, self._obs, self._cmd = [], [], []
            self.nbytes = 0


# ---------------------------------------------------------------- merging
def rollout_datasets(batch_dir: Path, outcomes: Sequence[str]) -> List[Tuple[Path, Dict[str, Any]]]:
    """Saved rollouts of the batch that have a dataset, in recording order."""
    found = []
    for folder in sorted(Path(batch_dir).iterdir()):
        meta_file = folder / "meta.json"
        if not folder.is_dir() or folder.name.startswith(".") or not meta_file.exists():
            continue
        if not (folder / DATASET_DIR / "meta" / "info.json").exists():
            continue
        meta = json.loads(meta_file.read_text(encoding="utf-8"))
        if meta.get("outcome", "unlabeled") in outcomes:
            found.append((folder, meta))
    return found


def merge_rollouts(
    batch_dir: Path,
    outcomes: Sequence[str],
    output: Optional[Path] = None,
    overwrite: bool = False,
    progress: Callable[[str], None] = lambda msg: None,
) -> Dict[str, Any]:
    """Aggregate the rollout datasets of a batch into one LeRobot dataset."""
    batch_dir = Path(batch_dir).expanduser().resolve()
    outcomes = [o for o in outcomes if o in OUTCOMES]
    if not outcomes:
        raise ValueError("Select at least one outcome.")
    rollouts = rollout_datasets(batch_dir, outcomes)
    if not rollouts:
        raise ValueError(f"No rollouts with a dataset and outcome in {outcomes} in {batch_dir}.")

    # aggregate_datasets needs identical features and fps
    signatures: Dict[str, List[str]] = {}
    for folder, _ in rollouts:
        info = json.loads((folder / DATASET_DIR / "meta" / "info.json").read_text(encoding="utf-8"))
        sig = json.dumps({"fps": info.get("fps"), "features": {
            k: [v.get("dtype"), v.get("shape")] for k, v in sorted(info.get("features", {}).items())
            if k not in ("timestamp", "frame_index", "episode_index", "index", "task_index")}}, sort_keys=True)
        signatures.setdefault(sig, []).append(folder.name)
    if len(signatures) > 1:
        groups = "; ".join(", ".join(names[:4]) + ("…" if len(names) > 4 else "") for names in signatures.values())
        raise ValueError(f"Rollouts have different features/fps (changed settings?) and cannot be merged: {groups}")

    tag = "all" if set(outcomes) == set(OUTCOMES) else "+".join(outcomes)
    output = Path(output).expanduser().resolve() if output else batch_dir.with_name(f"{batch_dir.name}_dataset_{tag}")
    if output == batch_dir or batch_dir in output.parents:
        raise ValueError("The merged dataset must not be inside the batch folder.")
    if output.exists():
        if not overwrite:
            raise ValueError(f"{output} exists; tick 'overwrite' to replace it.")
        shutil.rmtree(output)

    from lerobot.datasets.aggregate import aggregate_datasets

    progress(f"Merging {len(rollouts)} rollouts into {output}...")
    repo_id = f"local/{output.name}"
    aggregate_datasets(
        repo_ids=[f"local/{batch_dir.name}_{folder.name}" for folder, _ in rollouts],
        aggr_repo_id=repo_id,
        roots=[folder / DATASET_DIR for folder, _ in rollouts],
        aggr_root=output,
    )
    index = [
        {
            "episode": i,
            "folder": folder.name,
            "name": meta.get("name"),
            "outcome": meta.get("outcome"),
            "description": meta.get("description"),
            "task": meta.get("task"),
            "start_time": meta.get("start_time"),
            "stop_reason": meta.get("stats", {}).get("stop_reason"),
        }
        for i, (folder, meta) in enumerate(rollouts)
    ]
    (output / "meta" / "rollouts.json").write_text(json.dumps(index, indent=1), encoding="utf-8")
    counts = {o: sum(1 for _, m in rollouts if m.get("outcome") == o) for o in OUTCOMES}
    return {"output": str(output), "repo_id": repo_id, "episodes": len(rollouts), "counts": counts}


def main() -> None:
    parser = argparse.ArgumentParser(description="Merge the rollout datasets of a batch into one LeRobot dataset.")
    parser.add_argument("batch_dir", type=Path, help="<save_root>/<batch_name>")
    parser.add_argument("--outcome", action="append", choices=OUTCOMES,
                        help="repeatable; default: success")
    parser.add_argument("--output", type=Path, help="default <batch_dir>_dataset_<outcomes>")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    result = merge_rollouts(args.batch_dir, args.outcome or ["success"], args.output, args.overwrite, print)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
