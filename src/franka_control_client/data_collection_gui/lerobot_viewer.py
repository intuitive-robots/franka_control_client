"""Read a local LeRobot v3 dataset for the GUI viewer (no torch DataLoader, no hub access).

The browser plays the dataset's own mp4 files (one file holds several episodes; an episode is
the span from_timestamp..to_timestamp) and plots the numeric features of the episode. For
datasets stored as images, and as a fallback when the browser cannot decode a video, single
frames are served as JPEG. ``launch_rerun`` opens the episode in the Rerun viewer
(``dataset_utils/lerobot_viz.py``, like ``lerobot-dataset-viz``).
"""

from __future__ import annotations

import glob
import io
import json
import subprocess
import sys
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

SKIP_SERIES = {"timestamp", "frame_index", "episode_index", "index", "task_index"}
REPO_ROOT = Path(__file__).resolve().parents[3]


class DatasetView:
    def __init__(self, root: Path) -> None:
        import pandas as pd

        self.root = Path(root).expanduser().resolve()
        info_path = self.root / "meta" / "info.json"
        if not info_path.is_file():
            raise ValueError(f"{self.root} is not a LeRobot dataset (no meta/info.json)")
        self.info = json.loads(info_path.read_text(encoding="utf-8"))
        if not str(self.info.get("codebase_version", "")).startswith("v3"):
            raise ValueError(f"only LeRobot v3 datasets are supported (this is {self.info.get('codebase_version')})")
        self.fps = float(self.info["fps"])
        self.features: Dict[str, dict] = self.info["features"]
        self.camera_keys = [k for k, f in self.features.items() if f["dtype"] in ("video", "image")]
        files = sorted(glob.glob(str(self.root / "meta" / "episodes" / "*" / "*.parquet")))
        if not files:
            raise ValueError(f"{self.root} has no episodes yet")
        frames = [pd.read_parquet(f) for f in files]
        df = pd.concat(frames, ignore_index=True).sort_values("episode_index")
        self.episodes = df[[c for c in df.columns if not c.startswith("stats/")]].reset_index(drop=True)
        self._by_index = {int(r["episode_index"]): i for i, r in self.episodes.iterrows()}
        sources = self.root / "meta" / "irl_sources.json"
        self.sources: Dict[int, Dict[str, Any]] = {}
        if sources.is_file():
            try:
                self.sources = {int(s["episode_index"]): s for s in json.loads(sources.read_text(encoding="utf-8"))}
            except (ValueError, KeyError):
                pass
        self._lock = threading.Lock()
        self._cache: Tuple[Optional[int], Any] = (None, None)

    # ---------------------------------------------------------------- summary
    def summary(self) -> Dict[str, Any]:
        eps = []
        for _, r in self.episodes.iterrows():
            idx = int(r["episode_index"])
            tasks = r.get("tasks")
            tasks = list(tasks) if tasks is not None and not isinstance(tasks, str) else [tasks]
            src = self.sources.get(idx, {})
            eps.append({
                "index": idx,
                "length": int(r["length"]),
                "duration_s": int(r["length"]) / self.fps,
                "tasks": [str(t) for t in tasks if t is not None],
                "source": src.get("source"),
            })
        return {
            "root": str(self.root),
            "fps": self.fps,
            "robot_type": self.info.get("robot_type"),
            "codebase_version": self.info.get("codebase_version"),
            "total_episodes": int(self.info.get("total_episodes", len(eps))),
            "total_frames": int(self.info.get("total_frames", 0)),
            "features": {
                k: {"dtype": f["dtype"], "shape": f.get("shape"), "names": f.get("names")}
                for k, f in self.features.items() if k not in SKIP_SERIES
            },
            "cameras": self.camera_keys,
            "episodes": eps,
        }

    def _row(self, episode: int):
        if episode not in self._by_index:
            raise KeyError(f"episode {episode} not in the dataset")
        return self.episodes.iloc[self._by_index[episode]]

    def _episode_table(self, episode: int):
        """Data rows of one episode (cached: the viewer asks for frames of the same episode)."""
        import pandas as pd

        with self._lock:
            if self._cache[0] == episode:
                return self._cache[1]
            r = self._row(episode)
            path = self.root / self.info["data_path"].format(
                chunk_index=int(r["data/chunk_index"]), file_index=int(r["data/file_index"])
            )
            df = pd.read_parquet(path)
            df = df[df["episode_index"] == episode].sort_values("frame_index").reset_index(drop=True)
            self._cache = (episode, df)
            return df

    # ---------------------------------------------------------------- episode
    def episode(self, episode: int) -> Dict[str, Any]:
        r = self._row(episode)
        df = self._episode_table(episode)
        series = {}
        for key, f in self.features.items():
            if key in SKIP_SERIES or f["dtype"] in ("video", "image", "string") or key not in df.columns:
                continue
            try:
                values = np.stack([np.asarray(v, dtype=np.float64).reshape(-1) for v in df[key].to_numpy()])
            except (TypeError, ValueError):
                continue
            names = f.get("names")
            if isinstance(names, dict):  # older style {"motors": [...]}
                names = next(iter(names.values()), None)
            if not isinstance(names, list) or len(names) != values.shape[1]:
                names = [f"{key.split('.')[-1]}_{i}" for i in range(values.shape[1])]
            series[key] = {"names": names, "values": values.T.tolist()}
        videos = {}
        for key in self.camera_keys:
            if self.features[key]["dtype"] != "video":
                continue
            videos[key] = {
                "chunk": int(r[f"videos/{key}/chunk_index"]),
                "file": int(r[f"videos/{key}/file_index"]),
                "from": float(r[f"videos/{key}/from_timestamp"]),
                "to": float(r[f"videos/{key}/to_timestamp"]),
            }
        tasks = r.get("tasks")
        return {
            "index": episode,
            "length": int(len(df)),
            "fps": self.fps,
            "timestamps": df["timestamp"].astype(float).tolist() if "timestamp" in df else [],
            "series": series,
            "videos": videos,
            "images": [k for k in self.camera_keys if self.features[k]["dtype"] == "image"],
            "tasks": [str(t) for t in (list(tasks) if tasks is not None and not isinstance(tasks, str) else [tasks])],
            "source": self.sources.get(episode),
        }

    # ------------------------------------------------------------------ media
    def video_path(self, key: str, chunk: int, file: int) -> Path:
        if key not in self.camera_keys or self.features[key]["dtype"] != "video":
            raise KeyError(f"{key} is not a video feature")
        path = self.root / self.info["video_path"].format(video_key=key, chunk_index=int(chunk), file_index=int(file))
        if not path.is_file():
            raise FileNotFoundError(str(path))
        return path

    def frame_jpeg(self, episode: int, key: str, frame: int, max_width: int = 640) -> bytes:
        import cv2

        if key not in self.camera_keys:
            raise KeyError(f"{key} is not a camera feature")
        if self.features[key]["dtype"] == "image":
            df = self._episode_table(episode)
            frame = int(np.clip(frame, 0, len(df) - 1))
            cell = df[key].iloc[frame]
            data = cell.get("bytes") if isinstance(cell, dict) else cell
            if data is None and isinstance(cell, dict) and cell.get("path"):
                data = (self.root / cell["path"]).read_bytes()
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        else:
            from lerobot.datasets.video_utils import decode_video_frames

            r = self._row(episode)
            path = self.video_path(key, int(r[f"videos/{key}/chunk_index"]), int(r[f"videos/{key}/file_index"]))
            frame = int(np.clip(frame, 0, int(r["length"]) - 1))
            ts = float(r[f"videos/{key}/from_timestamp"]) + frame / self.fps
            t = decode_video_frames(path, [ts], tolerance_s=0.5 / self.fps, backend="pyav")[0]
            rgb = (t.permute(1, 2, 0).numpy() * 255).clip(0, 255).astype(np.uint8)
            img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        h, w = img.shape[:2]
        if w > max_width > 0:
            img = cv2.resize(img, (max_width, int(h * max_width / w)), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        return buf.tobytes()


def is_lerobot_dataset(path: Path) -> bool:
    return (Path(path) / "meta" / "info.json").is_file()


def launch_rerun(root: Path, episode: int) -> subprocess.Popen:
    """Open the episode in a local Rerun viewer (needs a display on this machine)."""
    root = Path(root).resolve()
    script = REPO_ROOT / "dataset_utils" / "lerobot_viz.py"
    args = ["--repo-id", f"local/{root.name}", "--root", str(root), "--episode-index", str(int(episode)),
            "--num-workers", "0"]
    cmd = [sys.executable, str(script), *args] if script.is_file() else \
        [sys.executable, "-m", "lerobot.scripts.lerobot_dataset_viz", *args]
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
