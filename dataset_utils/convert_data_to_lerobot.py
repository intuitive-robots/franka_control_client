#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Convert your current "Robocasa-like" folder structure into a LeRobotDataset.

Assumptions (override via CLI flags):
- Episodes are directories that contain:
    <EPISODE_DIR>/
      "201 leader"/joint_pos.pt
      "201 leader"/gripper_state.pt
      sensors/right_cam/*.png
      sensors/wrist_cam/*.png
      sensors/<tactile_name>/*.png   (optional, multiple)
- Camera images are RGB .png in BGR on disk (cv2), converted to RGB uint8.
- We store raw images (HWC, uint8). No normalization here; training can handle that.
# CHANGED: Action and State are perfectly aligned at t=0...T. 
- Action = [7 joint positions at t, 1 gripper] taken from t= 0, ... T of leader
  State  = [7 joint positions at t, 1 gripper] taken from t= 0, ... T of follower

Output:
- LeRobotDataset with keys:
    observation.images.<cam_name>     (image/video)
    observation.images.<tactile_name> (image/video)  [optional]
    observation.state                 (float32, 8)
    observation.force_torque          (float32, 14)
    action                            (float32, 8)
- One episode per discovered trajectory.
"""

import argparse
import re
import shutil
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.constants import HF_LEROBOT_HOME
from concurrent.futures import ThreadPoolExecutor  # ADDED: For parallel image loading

# ADDED: Prevent CV2 deadlocks when using Python multiprocessing in LeRobot
cv2.setNumThreads(0)
cv2.ocl.setUseOpenCL(False)

# ---------- Utils ----------


def _numeric_sort_key(p: Path) -> Tuple[int, str]:
    """Sort by numeric stem if possible, else by name."""
    try:
        return (int(re.sub(r"\D", "", p.stem)), p.name)
    except Exception:
        return (sys.maxsize, p.name)


# ADDED: Helper function for ThreadPoolExecutor
def _process_single_image(p: Path, resize: Optional[Tuple[int, int]]) -> Optional[np.ndarray]:
    img_bgr = cv2.imread(str(p), cv2.IMREAD_COLOR)
    if img_bgr is None:
        return None
    if resize is not None:
        w, h = resize
        img_bgr = cv2.resize(img_bgr, (w, h), interpolation=cv2.INTER_AREA)
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    # NOTE: Currently the collect_data.py script saves the image observations with red and blue channels switched
    # -> remove the following line as soon as this has been fixed in the data collection
    # img_rgb = img_rgb[..., [2, 1, 0]]
    return img_rgb


def _read_png_folder(folder: Path, resize: Optional[Tuple[int, int]]) -> np.ndarray:
    """
    Read a folder of PNGs to an array [T, H, W, 3], dtype=uint8, RGB.
    If resize is provided, it is (width, height).
    """
    if not folder.exists():
        raise FileNotFoundError(f"Missing image folder: {folder}")
    files = sorted([p for p in folder.glob("*.png")], key=_numeric_sort_key)
    if not files:
        raise FileNotFoundError(f"No .png files in {folder}")

    # CHANGED: Replaced sequential loop with ThreadPoolExecutor for massive speedup
    with ThreadPoolExecutor(max_workers=8) as executor:
        frames = list(executor.map(lambda p: _process_single_image(p, resize), files))

    # Filter out None values just in case
    frames = [f for f in frames if f is not None]

    if not frames:
        raise RuntimeError(f"Failed to load any images from {folder}")
    arr = np.stack(frames, axis=0)  # [T, H, W, 3]
    return arr.astype(np.uint8)


def _find_episode_dirs(root: Path, leader_subdir: str, follower_subdir) -> List[Path]:
    """
    Find all episode directories containing required files.
    We consider a directory an episode if it contains:
      <episode>/<leader_subdir>/joint_pos.pt and gripper_state.pt
    """
    episodes = []
    for ep in root.rglob("*"):
        if not ep.is_dir():
            continue
        leader_dir = ep / leader_subdir
        follower_dir = ep / follower_subdir
        if (
            (leader_dir / "joint_pos.pt").exists()
            and (leader_dir / "gripper_state.pt").exists()
            and (follower_dir / "joint_pos.pt").exists()
            and (follower_dir / "gripper_state.pt").exists()
            ##add t/f information
            and (follower_dir / "external_joint_torque.pt").exists()
            and (follower_dir / "external_wrench.pt").exists()
            and (follower_dir / "gripper_current.pt").exists()
        ):
            episodes.append(ep)
    episodes = sorted(episodes)
    return episodes


def _compute_lengths_align(
    leader_joint_pos: torch.Tensor,
    leader_gripper: torch.Tensor,
    follower_joint_pos: torch.Tensor,
    follower_gripper: torch.Tensor,
    cam_frames: Dict[str, np.ndarray],
    tactile_frames: Dict[str, np.ndarray],
    ##add t/f information
    follower_external_joint_torque: torch.Tensor,
    follower_external_wrench: torch.Tensor,
    follower_gripper_current: torch.Tensor,
) -> int:
    """
    Compute aligned length T such that we can form T action/state pairs. # CHANGED: T instead of T-1
    All streams will be trimmed to T.
    """
    lengths = [leader_joint_pos.shape[0], leader_gripper.shape[0]]
    lengths.extend([follower_joint_pos.shape[0], follower_gripper.shape[0]])
    lengths.extend([v.shape[0] for v in cam_frames.values()])
    lengths.extend([v.shape[0] for v in tactile_frames.values()])
    ##add t/f information
    lengths.extend([
        follower_external_joint_torque.shape[0],
        follower_external_wrench.shape[0],
        follower_gripper_current.shape[0],
    ])
    T = min(lengths)
    if T < 2:
        raise ValueError(f"Sequence too short after alignment: T={T}")
    return T


# ---------- Feature spec ----------


def _build_features_spec(
    image_shapes: Dict[str, Tuple[int, int, int]],
    tactile_shapes: Dict[str, Tuple[int, int, int]],
    use_videos: bool,
) -> Dict[str, dict]:
    """
    Build the LeRobot feature spec dictionary.
    image_shapes/tactile_shapes: dict cam_name -> (H, W, C) but LeRobot expects (C, H, W)
    """
    dtype_choice = "video" if use_videos else "image"

    features = {}
    for k, (h, w, c) in image_shapes.items():
        features[f"observation.images.{k.replace('zed_', '')}"] = {
            "dtype": dtype_choice,
            "shape": (c, h, w),
            "names": ["channel", "height", "width"],
        }
    for k, (h, w, c) in tactile_shapes.items():
        features[f"observation.images.{k}"] = {
            "dtype": dtype_choice,
            "shape": (c, h, w),
            "names": ["channel", "height", "width"],
        }

    # State: 7 joints + 1 gripper
    features["observation.state"] = {
        "dtype": "float32",
        "shape": (8,),
        "names": [f"joint_{i}" for i in range(7)] + ["gripper"],
    }
    # Force / torque and gripper current: 7 joint torque + 6 external wrench + 1 gripper current
    features["observation.force_torque"] = {
        "dtype": "float32",
        "shape": (14,),
        "names": (
            [f"external_joint_torque_{i}" for i in range(7)]
            + [f"external_wrench_{i}" for i in range(6)]
            + ["gripper_current"]
        ),
    }

    # Action: 7 joints + 1 gripper
    features["action"] = {
        "dtype": "float32",
        "shape": (8,),
        "names": [f"joint_{i}" for i in range(7)] + ["gripper"],
    }
    return features


# ---------- Core conversion ----------


def probe_first_valid_episode(
    episodes: List[Path],
    cams: List[str],
    tactile_names: List[str],
    leader_subdir: str,
    sensors_dirname: str,
    resize: Optional[Tuple[int, int]],
) -> Tuple[Dict[str, Tuple[int, int, int]], Dict[str, Tuple[int, int, int]]]:
    """
    Load the first episode just to discover image shapes for features.
    Returns dicts cam->(H,W,3), tactile->(H,W,3)
    """
    for ep in episodes:
        try:
            cam_shapes = {}
            for cam in cams:
                arr = _read_png_folder(ep / sensors_dirname / cam, resize)
                cam_shapes[cam] = tuple(arr.shape[1:4])  # (H,W,3)
            tact_shapes = {}
            for tname in tactile_names:
                arr = _read_png_folder(ep / sensors_dirname / tname, resize)
                tact_shapes[tname] = tuple(arr.shape[1:4])
            return cam_shapes, tact_shapes
        except Exception:
            continue
    raise RuntimeError(
        "Could not probe any episode for image shapes. Check paths/flags."
    )


def save_episode_to_lerobot(
    lerobot_ds: LeRobotDataset,
    ep_dir: Path,
    cams: List[str],
    tactile_names: List[str],
    leader_subdir: str,
    follower_subdir: str,
    sensors_dirname: str,
    resize: Optional[Tuple[int, int]],
    task_instruction_mapping: Dict[str, str],  # str = "parent",  # "parent" or "name"
):
    """
    Read one episode and write it into the LeRobot dataset.
    """
    leader = ep_dir / leader_subdir
    follower = ep_dir / follower_subdir
    sensors = ep_dir / sensors_dirname

    leader_joint_pos = torch.load(leader / "joint_pos.pt")
    leader_gripper = torch.load(leader / "gripper_state.pt")
    follower_joint_pos = torch.load(follower / "joint_pos.pt")
    follower_gripper = torch.load(follower / "gripper_state.pt")
    ##add t/f information
    follower_external_joint_torque = torch.load(follower / "external_joint_torque.pt")
    follower_external_wrench = torch.load(follower / "external_wrench.pt")
    follower_gripper_current = torch.load(follower / "gripper_current.pt")

    if leader_joint_pos.ndim != 2 or leader_joint_pos.shape[1] != 7:
        raise ValueError(
            f"Expected joint_pos shape [T,7], got {tuple(leader_joint_pos.shape)} in {leader}"
        )
    if follower_joint_pos.ndim != 2 or follower_joint_pos.shape[1] != 7:
        raise ValueError(
            # CHANGED: Fixed error message typo pointing to leader instead of follower
            f"Expected joint_pos shape [T,7], got {tuple(follower_joint_pos.shape)} in {follower}"
        )
    if leader_gripper.ndim == 1:
        leader_gripper = leader_gripper[:, None]
    if follower_gripper.ndim == 1:
        follower_gripper = follower_gripper[:, None]
        
    # ADDED: Gripper binarization to clean up -1 artifacts
    leader_gripper[leader_gripper == -1] = 0
    follower_gripper[follower_gripper == -1] = 0
    
    ## add t/f information
    if follower_external_joint_torque.ndim != 2 or follower_external_joint_torque.shape[1] != 7:
        raise ValueError(
            f"Expected external_joint_torque shape [T,7], got {tuple(follower_external_joint_torque.shape)} in {follower}"
        )
    if follower_external_wrench.ndim != 2 or follower_external_wrench.shape[1] != 6:
        raise ValueError(
            f"Expected external_wrench shape [T,6], got {tuple(follower_external_wrench.shape)} in {follower}"
        )
    if follower_gripper_current.ndim == 1:
        follower_gripper_current = follower_gripper_current[:, None]
        
    # Load image streams
    cam_frames: Dict[str, np.ndarray] = {}
    for cam in cams:
        cam_frames[cam] = _read_png_folder(sensors / cam, resize)

    tactile_frames: Dict[str, np.ndarray] = {}
    for tname in tactile_names:
        tdir = sensors / tname
        if tdir.exists():
            tactile_frames[tname] = _read_png_folder(tdir, resize)

    # Align all streams to common T
    T = _compute_lengths_align(
        leader_joint_pos,
        leader_gripper,
        follower_joint_pos,
        follower_gripper,
        cam_frames,
        tactile_frames,
        follower_external_joint_torque,
        follower_external_wrench,
        follower_gripper_current,
    )
    
    # Build state based on follower and actions based on leader with exactly T steps
    leader_joint_np = (
        leader_joint_pos.detach().cpu().numpy().astype(np.float32)[:T]
    )  # [T,7]
    leader_grip_np = (
        leader_gripper.detach().cpu().numpy().astype(np.float32)[:T, :1]
    )  # [T,1]
    follower_joint_np = (
        follower_joint_pos.detach().cpu().numpy().astype(np.float32)[:T]
    )  # [T,7]
    follower_grip_np = (
        follower_gripper.detach().cpu().numpy().astype(np.float32)[:T, :1]
    )  # [T,1]
    ## add t/f information
    follower_external_joint_torque_np = (
        follower_external_joint_torque.detach().cpu().numpy().astype(np.float32)[:T]
    )  # [T,7]
    follower_external_wrench_np = (
        follower_external_wrench.detach().cpu().numpy().astype(np.float32)[:T]
    )  # [T,6]
    follower_gripper_current_np = (
        follower_gripper_current.detach().cpu().numpy().astype(np.float32)[:T, :1]
    )  # [T,1]
    
    # CHANGED: Removed the [:-1] and [1:] slicing to enforce a perfectly aligned mapping 
    # where GELLO teleop assumes the leader command at `t` is the action at `t`.
    state_np = np.concatenate([follower_joint_np, follower_grip_np], axis=1)  # [T,8]
    force_torque_np = np.concatenate(
        [
            follower_external_joint_torque_np,
            follower_external_wrench_np,
            follower_gripper_current_np,
        ],
        axis=1,
    )  # [T,14]
    action_np = np.concatenate(
        [leader_joint_np, leader_grip_np], axis=1
    )  # [T,8]

    # CHANGED: Trimming limits removed the -1 offset.
    cam_trimmed = {k: v[:T] for k, v in cam_frames.items()}  # [T,H,W,3]
    tactile_trimmed = {k: v[:T] for k, v in tactile_frames.items()}  # [T,H,W,3]

    task_instr = task_instruction_mapping[ep_dir.parent.name]

    # CHANGED: Iterate over T directly instead of T-1
    # Stream frames
    for i in range(T):
        image_dict = {}
        for cam, arr in cam_trimmed.items():
            # Convert from (H, W, C) to (C, H, W) for LeRobot
            image_dict[f"observation.images.{cam.replace('zed_','')}"] = np.transpose(
                arr[i], (2, 0, 1)
            )
        for tname, arr in tactile_trimmed.items():
            # Convert from (H, W, C) to (C, H, W) for LeRobot
            image_dict[f"observation.images.{tname}"] = np.transpose(arr[i], (2, 0, 1))

        lerobot_ds.add_frame(
            {
                **image_dict,
                "observation.state": state_np[i],
                "observation.force_torque": force_torque_np[i],
                "action": action_np[i],
                "task": task_instr,
            },
        )
    lerobot_ds.save_episode()


def create_lerobot_dataset(
    raw_dir: Path,
    local_dir: Optional[Path],
    repo_id: Optional[str],
    push_to_hub: bool,
    robot_type: Optional[str],
    fps: Optional[int],
    use_videos: bool,
    image_writer_process: int,
    image_writer_threads: int,
    keep_images: bool,  # kept for parity; LeRobot handles storage internally
    cams: List[str],
    tactile_names: List[str],
    leader_subdir: str,
    follower_subdir: str,
    sensors_dirname: str,
    resize_w: Optional[int],
    resize_h: Optional[int],
    task_instruction_mapping: Dict[str, str],
):
    raw_dir = raw_dir.resolve()
    if local_dir is None:
        local_dir = Path(HF_LEROBOT_HOME)
    dataset_name = raw_dir.name
    out_root = local_dir.resolve()
    if out_root.exists():
        shutil.rmtree(out_root)

    episodes = _find_episode_dirs(raw_dir, leader_subdir, follower_subdir)
    if not episodes:
        raise RuntimeError(
            f"No episodes found in {raw_dir} with leader subdir '{leader_subdir}'"
        )

    resize = None
    if resize_w is not None and resize_h is not None:
        resize = (resize_w, resize_h)

    # Probe shapes for features
    cam_shapes, tactile_shapes = probe_first_valid_episode(
        episodes, cams, tactile_names, leader_subdir, sensors_dirname, resize
    )

    # FPS & robot type defaults
    if fps is None:
        fps = 10
    if robot_type is None:
        robot_type = "unknown"

    # Build feature spec
    features = _build_features_spec(cam_shapes, tactile_shapes, use_videos)

    # Create target LeRobot dataset
    ds = LeRobotDataset.create(
        repo_id=repo_id,
        robot_type=robot_type.lower().replace(" ", "_").replace("-", "_"),
        root=out_root,
        fps=int(fps),
        use_videos=use_videos,
        video_backend="pyav",
        features=features,
        image_writer_threads=image_writer_threads,
        image_writer_processes=image_writer_process,
    )

    # Convert each episode
    failures = 0
    for ep in episodes:
        try:
            save_episode_to_lerobot(
                ds,
                ep_dir=ep,
                cams=cams,
                tactile_names=tactile_names,
                leader_subdir=leader_subdir,
                follower_subdir=follower_subdir,
                sensors_dirname=sensors_dirname,
                resize=resize,
                task_instruction_mapping=task_instruction_mapping,
            )
            print(f"[OK]   episode {ep}")
        except Exception as e:
            failures += 1
            print(f"[WARN] skipping episode {ep}: {e}")

    ds.finalize() #necessary for lerobot 0.5.2 compatibility

    print(f"Done. Episodes written: {len(episodes)-failures} / {len(episodes)}")
    print(f"LeRobot dataset root: {out_root}")

    if push_to_hub:
        assert repo_id is not None, "repo_id required to push_to_hub"
        tags = ["LeRobot", dataset_name]
        if tactile_names:
            tags.append("tactile")
        if robot_type != "unknown":
            tags.append(robot_type)
        ds.push_to_hub(
            tags=tags,
            private=False,
            push_videos=True,
            license="apache-2.0",
        )
        print("Pushed to Hub.")


# ---------- CLI ----------

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Convert Robocasa-like folder structure into a LeRobotDataset.")
    parser.add_argument("--raw_dir", type=str, required=True, help="Path to the raw dataset directory")
    parser.add_argument("--local_dir", type=str, required=True, help="Path to the output LeRobot dataset directory")
    parser.add_argument("--task_instr", type=str, required=True, help="put the sponge on the plate")
    parser.add_argument("--fps", type=int, default=15, help="Target FPS for the dataset")
    parser.add_argument("--resize_w", type=int, default=256, help="Resize image width")
    parser.add_argument("--resize_h", type=int, default=256, help="Resize image height")
    parser.add_argument("--push_to_hub", type=bool, default=False, help="Push the resulting dataset to HF hub")
    args = parser.parse_args()

    # Cast string arguments to Path objects
    raw_dir = Path(args.raw_dir)
    local_dir = Path(args.local_dir)
    repo_id = raw_dir.name

    task_instruction_mapping = {
        repo_id: args.task_instr
    }   # NOTE: currently supports only a single task

    # Structure configuration
    follower_subdir = robot_type = "FrankaPanda"
    leader_subdir = "Gello"
    sensors_dirname = "sensors"
    cams = ["zed_right", "zed_wrist"]  # ["zed_right", "zed_left", "zed_wrist"]
    tactile_names = []

    # Image and video handling
    use_videos = True
    image_writer_process = 5
    image_writer_threads = 10
    keep_images = False

    create_lerobot_dataset(
        raw_dir=raw_dir,
        local_dir=local_dir,
        repo_id=repo_id,
        push_to_hub=args.push_to_hub,
        robot_type=robot_type,
        fps=args.fps,  
        use_videos=use_videos,
        image_writer_process=image_writer_process,
        image_writer_threads=image_writer_threads,
        keep_images=keep_images,
        cams=cams,
        tactile_names=tactile_names,
        leader_subdir=leader_subdir,
        follower_subdir=follower_subdir,
        sensors_dirname=sensors_dirname,
        resize_w=args.resize_w,    
        resize_h=args.resize_h,
        task_instruction_mapping=task_instruction_mapping,
    )


if __name__ == "__main__":
    main()