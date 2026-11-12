#!/usr/bin/env python
"""Convert WebXR teleop recordings (data_collection_droid_webxr_irl.py) into a LeRobotDataset.

Episode layout (one dir per episode under --raw-dir):
    webxr/{EE_pos,EE_quat,gripper_state}.pt          leader: commanded EE target, xyzw quat, gripper 1=open 0=closed
    FrankaPanda/{joint_pos,EE_pos,EE_quat,gripper_state,
                 external_joint_torque,external_wrench,gripper_current}.pt   follower
    sensors/zed_{left,wrist,right}/*.png

Features:
    observation.images.{left,wrist,right}  video
    observation.state         (8)  follower joints + gripper (same as the GELLO converter)
    observation.ee_pose       (7)  follower EE pos + quat xyzw
    observation.force_torque (14)  external joint torque + external wrench + gripper current
    action                    (8)  WebXR EE target pos + quat xyzw + gripper, taken at t + action_offset

Episodes missing any stream (aborted recordings) are skipped.
"""

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset

LEADER = "webxr"
FOLLOWER = "FrankaPanda"
LEADER_FILES = ["EE_pos.pt", "EE_quat.pt", "gripper_state.pt"]
FOLLOWER_FILES = [
    "joint_pos.pt",
    "EE_pos.pt",
    "EE_quat.pt",
    "gripper_state.pt",
    "external_joint_torque.pt",
    "external_wrench.pt",
    "gripper_current.pt",
]
QUAT = ["qx", "qy", "qz", "qw"]


def _load(path: Path) -> np.ndarray:
    a = torch.load(path).detach().cpu().numpy().astype(np.float32)
    return a[:, None] if a.ndim == 1 else a


def _is_complete(ep: Path, cams: list[str]) -> bool:
    return (
        all((ep / LEADER / f).exists() for f in LEADER_FILES)
        and all((ep / FOLLOWER / f).exists() for f in FOLLOWER_FILES)
        and all(any((ep / "sensors" / c).glob("*.png")) for c in cams)
    )


def _frames(folder: Path) -> list[Path]:
    return sorted(folder.glob("*.png"), key=lambda p: int(p.stem))


def _read_rgb(p: Path, size: tuple[int, int] | None) -> np.ndarray:
    img = cv2.imread(str(p), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"Unreadable image {p}")
    if size is not None:
        img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def _features(cams: list[str], h: int, w: int) -> dict:
    feats = {
        f"observation.images.{c.replace('zed_', '')}": {
            "dtype": "video",
            "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        }
        for c in cams
    }
    feats["observation.state"] = {
        "dtype": "float32",
        "shape": (8,),
        "names": [f"joint_{i}" for i in range(7)] + ["gripper"],
    }
    feats["observation.ee_pose"] = {
        "dtype": "float32",
        "shape": (7,),
        "names": ["x", "y", "z"] + QUAT,
    }
    feats["observation.force_torque"] = {
        "dtype": "float32",
        "shape": (14,),
        "names": [f"external_joint_torque_{i}" for i in range(7)]
        + [f"external_wrench_{i}" for i in range(6)]
        + ["gripper_current"],
    }
    feats["action"] = {
        "dtype": "float32",
        "shape": (8,),
        "names": ["x", "y", "z"] + QUAT + ["gripper"],
    }
    return feats


def convert_episode(ds: LeRobotDataset, ep: Path, cams, size, task: str, offset: int) -> int:
    ld = {f: _load(ep / LEADER / f) for f in LEADER_FILES}
    fd = {f: _load(ep / FOLLOWER / f) for f in FOLLOWER_FILES}
    imgs = {c: _frames(ep / "sensors" / c) for c in cams}

    T = min([len(v) for v in (*ld.values(), *fd.values(), *imgs.values())])
    n = T - offset
    if n < 1:
        raise ValueError(f"too short (T={T})")

    state = np.concatenate([fd["joint_pos.pt"], fd["gripper_state.pt"][:, :1]], axis=1)
    ee_pose = np.concatenate([fd["EE_pos.pt"], fd["EE_quat.pt"]], axis=1)
    ft = np.concatenate(
        [fd["external_joint_torque.pt"], fd["external_wrench.pt"], fd["gripper_current.pt"][:, :1]],
        axis=1,
    )
    action = np.concatenate([ld["EE_pos.pt"], ld["EE_quat.pt"], ld["gripper_state.pt"][:, :1]], axis=1)

    for i in range(n):
        frame = {
            f"observation.images.{c.replace('zed_', '')}": _read_rgb(imgs[c][i], size) for c in cams
        }
        frame.update(
            {
                "observation.state": state[i],
                "observation.ee_pose": ee_pose[i],
                "observation.force_torque": ft[i],
                "action": action[i + offset],
                "task": task,
            }
        )
        ds.add_frame(frame)
    ds.save_episode()
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw-dir", type=Path, required=True, help="Task folder holding the episode dirs")
    ap.add_argument("--out-dir", type=Path, required=True, help="LeRobot dataset root (overwritten)")
    ap.add_argument("--repo-id", default=None, help="Defaults to the raw dir name")
    ap.add_argument("--task", required=True, help="Language instruction stored with every frame")
    ap.add_argument("--fps", type=int, default=4, help="Recording rate (IRLDataCollection fps)")
    ap.add_argument("--cams", nargs="+", default=["zed_left", "zed_wrist", "zed_right"])
    ap.add_argument("--size", type=int, nargs=2, default=[256, 256], metavar=("W", "H"), help="0 0 keeps raw size")
    ap.add_argument("--action-offset", type=int, default=1, help="action[t] = leader[t + offset]")
    ap.add_argument("--robot-type", default="franka_panda")
    ap.add_argument("--push-to-hub", action="store_true")
    args = ap.parse_args()

    raw = args.raw_dir.resolve()
    repo_id = args.repo_id or raw.name
    size = tuple(args.size) if all(args.size) else None

    eps = sorted(p for p in raw.iterdir() if p.is_dir())
    good = [p for p in eps if _is_complete(p, args.cams)]
    for p in sorted(set(eps) - set(good)):
        print(f"[SKIP] incomplete episode {p.name}")
    if not good:
        raise SystemExit(f"No complete episodes in {raw}")

    h, w = _read_rgb(_frames(good[0] / "sensors" / args.cams[0])[0], size).shape[:2]
    if args.out_dir.exists():
        shutil.rmtree(args.out_dir)
    ds = LeRobotDataset.create(
        repo_id=repo_id,
        root=args.out_dir,
        fps=args.fps,
        robot_type=args.robot_type,
        features=_features(args.cams, h, w),
        use_videos=True,
        image_writer_threads=8,
    )

    frames = 0
    for ep in good:
        try:
            n = convert_episode(ds, ep, args.cams, size, args.task, args.action_offset)
            frames += n
            print(f"[OK]   {ep.name}: {n} frames")
        except Exception as e:  # keep going, report at the end
            print(f"[WARN] {ep.name}: {e}")
    ds.finalize()
    print(f"Wrote {ds.meta.total_episodes} episodes / {frames} frames to {args.out_dir}")

    if args.push_to_hub:
        ds.push_to_hub(tags=["LeRobot", repo_id, "webxr"], private=True)


if __name__ == "__main__":
    main()
