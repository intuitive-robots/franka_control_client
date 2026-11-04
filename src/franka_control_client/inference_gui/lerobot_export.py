"""Rollouts as LeRobot datasets.

Per rollout (``save_lerobot`` on): a sampler in the session captures, at the
dataset rate, every camera, the robot state and what the controller is playing,
and :class:`RolloutDatasetWriter` streams it into a one-episode LeRobot dataset
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
import queue
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


class RolloutDatasetWriter:
    """Streams one rollout into a one-episode LeRobot dataset.

    ``add_sample(obs, action_now, repeat)`` is called on the dataset clock;
    ``repeat`` > 1 fills slots the sampler missed with copies of this sample, so
    the episode keeps the real duration. A frame's action is the command playing
    at the next sample (as in the converters, where the action is the leader one
    step later), so each frame is held back one sample, and the last one is
    dropped at ``finish``. Frames are never dropped: if the writer falls behind,
    ``add_sample`` blocks and the sampler fills the gap with repeats."""

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
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        self.root = Path(root)
        self.repo_id = repo_id
        self.fps = fps
        self.task = task or "rollout"
        self.features = features
        self.n_frames = 0
        self.duplicated = 0  # frames that repeat the previous sample (sampler too slow)
        self.error: Optional[str] = None
        self._pending: Optional[Dict[str, Any]] = None
        # Bounded in time, not 30 s: queued frames hold full images (2K: ~8 MB per camera).
        self._queue: "queue.Queue[Optional[Dict[str, Any]]]" = queue.Queue(maxsize=int(fps * 5) + 5)
        self._dataset = LeRobotDataset.create(
            repo_id=repo_id,
            fps=int(round(fps)),
            features=features,
            root=self.root,
            robot_type=robot_type,
            use_videos=use_videos,
            image_writer_threads=4,
        )
        self._thread = threading.Thread(target=self._run, name="lerobot-writer", daemon=True)
        self._thread.start()

    def add_sample(self, observation: Dict[str, Any], action_now: np.ndarray, repeat: int = 1) -> None:
        action_now = np.asarray(action_now, dtype=np.float32)
        if self._pending is not None:
            self._pending["action"] = action_now
            self._put(self._pending)
        # Repeats share the arrays (shallow copies); the copy in a missed slot is
        # followed by the same capture, so its action is this sample's command.
        for _ in range(max(1, int(repeat)) - 1):
            self._put({**observation, "action": action_now})
            self.duplicated += 1
        self._pending = dict(observation)

    def _put(self, frame: Dict[str, Any]) -> None:
        while self._thread.is_alive():
            try:
                self._queue.put(frame, timeout=1.0)
                return
            except queue.Full:
                continue
        self.error = "dataset writer thread stopped; frames were lost"

    def finish(self) -> int:
        """Write the episode; returns the number of frames (0: nothing written)."""
        self._pending = None
        self._queue.put(None)
        self._thread.join()
        if self.error and self.n_frames == 0:
            raise RuntimeError(self.error)
        if self.n_frames < 2:
            LOG.warning("Rollout dataset has %d frames; not writing it.", self.n_frames)
            self._close_quietly()
            shutil.rmtree(self.root, ignore_errors=True)
            return 0
        self._dataset.save_episode()
        self._dataset.finalize()
        images = self.root / "images"
        if images.exists() and not any(p.is_file() for p in images.rglob("*")):
            shutil.rmtree(images, ignore_errors=True)  # the image writer's empty scratch folders
        return self.n_frames

    def abort(self) -> None:
        self._pending = None
        self._queue.put(None)
        self._thread.join(timeout=10)
        self._close_quietly()

    def _close_quietly(self) -> None:
        try:
            if getattr(self._dataset, "image_writer", None) is not None:
                self._dataset.stop_image_writer()
        except Exception:
            pass

    def _run(self) -> None:
        while True:
            frame = self._queue.get()
            if frame is None:
                return
            try:
                frame["task"] = self.task
                self._dataset.add_frame(frame)
                self.n_frames += 1
            except Exception as exc:  # keep draining so the episode stays consistent
                self.error = f"add_frame failed: {exc}"
                LOG.exception("add_frame failed")


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
