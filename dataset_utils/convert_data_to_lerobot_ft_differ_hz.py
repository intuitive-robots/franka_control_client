#!/usr/bin/env python
# -*- coding: utf-8 -*-
###version: match with the index(not timestamp), possible patch:1.allign with timestamp 2.add offset
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
- Action = [7 joint positions at t, 1 gripper] taken from t=1..T-1 of leader
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

import re
import shutil
import sys
from math import floor
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import DEFAULT_FEATURES
from lerobot.utils.constants import HF_LEROBOT_HOME

# ---------- Utils ----------


def _numeric_sort_key(p: Path) -> Tuple[int, str]:
    """Sort by numeric stem if possible, else by name."""
    try:
        return (int(re.sub(r"\D", "", p.stem)), p.name)
    except Exception:
        return (sys.maxsize, p.name)


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

    frames = []
    for p in files:
        img_bgr = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img_bgr is None:
            continue
        if resize is not None:
            w, h = resize
            img_bgr = cv2.resize(img_bgr, (w, h), interpolation=cv2.INTER_AREA)
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        frames.append(img_rgb)
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


def _compute_episode_duration_s(
    leader_joint_pos: torch.Tensor,
    leader_gripper: torch.Tensor,
    follower_joint_pos: torch.Tensor,
    follower_gripper: torch.Tensor,
    cam_frames: Dict[str, np.ndarray],
    tactile_frames: Dict[str, np.ndarray],
    follower_external_joint_torque: torch.Tensor,
    follower_external_wrench: torch.Tensor,
    follower_gripper_current: torch.Tensor,
    proprio_fps: int,
    image_fps_by_stream: Dict[str, int],
) -> float:
    """
    Compute the duration shared by all streams in seconds.
    Numeric streams are assumed to run at proprio_fps and image streams use their
    per-stream fps values from image_fps_by_stream.
    """
    durations = [
        leader_joint_pos.shape[0] / proprio_fps,
        leader_gripper.shape[0] / proprio_fps,
        follower_joint_pos.shape[0] / proprio_fps,
        follower_gripper.shape[0] / proprio_fps,
        follower_external_joint_torque.shape[0] / proprio_fps,
        follower_external_wrench.shape[0] / proprio_fps,
        follower_gripper_current.shape[0] / proprio_fps,
    ]
    durations.extend(
        frames.shape[0] / image_fps_by_stream[stream_name]
        for stream_name, frames in cam_frames.items()
    )
    durations.extend(
        frames.shape[0] / image_fps_by_stream[stream_name]
        for stream_name, frames in tactile_frames.items()
    )

    duration_s = min(durations)
    if duration_s <= 0:
        raise ValueError(f"Sequence too short after alignment: duration_s={duration_s}")
    return duration_s


def _compute_num_master_steps(duration_s: float, dataset_fps: int) -> int:
    """Convert a shared duration to the number of frames on the dataset timeline."""
    num_master_steps = floor(duration_s * dataset_fps)
    if num_master_steps < 2:
        raise ValueError(
            f"Sequence too short after alignment: num_master_steps={num_master_steps}"
        )
    return num_master_steps


def _map_master_step_to_stream_index(
    master_step: int,
    stream_fps: int,
    dataset_fps: int,
    stream_length: int,
) -> int:
    """Map one step on the master timeline to a source stream index."""
    if stream_length <= 0:
        raise ValueError("Cannot map into an empty stream.")
    # use round may leak future frame
    # source_idx = round(master_step * stream_fps / dataset_fps)
    source_idx = floor(master_step * stream_fps / dataset_fps)
    return min(source_idx, stream_length - 1)#use min to avoid overflow when master_step is the last step and stream_fps/dataset_fps ratio is slightly larger than 1, which would cause source_idx == stream_length


def _build_mapped_indices(
    num_master_steps: int,
    stream_fps: int,
    dataset_fps: int,
    stream_length: int,
) -> List[int]:
    """Return the source indices selected for each master-timeline step."""
    return [
        _map_master_step_to_stream_index(
            master_step=master_step,
            stream_fps=stream_fps,
            dataset_fps=dataset_fps,
            stream_length=stream_length,
        )
        for master_step in range(num_master_steps)
    ]


def _print_episode_alignment_debug(
    ep_dir: Path,
    duration_s: float,
    num_master_steps: int,
    dataset_fps: int,
    proprio_length: int,
    image_lengths: Dict[str, int],
    tactile_lengths: Dict[str, int],
    image_fps_by_stream: Dict[str, int],
) -> None:
    """Print a compact summary of how streams were aligned for one episode."""
    print(
        f"[DEBUG] {ep_dir.name}: duration={duration_s:.3f}s, "
        f"master_steps={num_master_steps}, dataset_fps={dataset_fps}, "
        f"proprio_frames={proprio_length}"
    )

    for stream_name, stream_length in sorted(image_lengths.items()):
        mapped_indices = _build_mapped_indices(
            num_master_steps=num_master_steps - 1,
            stream_fps=image_fps_by_stream[stream_name],
            dataset_fps=dataset_fps,
            stream_length=stream_length,
        )
        unique_indices = len(set(mapped_indices))
        repeats = len(mapped_indices) - unique_indices
        print(
            f"[DEBUG]   image {stream_name}: fps={image_fps_by_stream[stream_name]}, "
            f"raw_frames={stream_length}, "
            f"used_frames={unique_indices}, repeated_steps={repeats}"
        )

    for stream_name, stream_length in sorted(tactile_lengths.items()):
        mapped_indices = _build_mapped_indices(
            num_master_steps=num_master_steps - 1,
            stream_fps=image_fps_by_stream[stream_name],
            dataset_fps=dataset_fps,
            stream_length=stream_length,
        )
        unique_indices = len(set(mapped_indices))
        repeats = len(mapped_indices) - unique_indices
        print(
            f"[DEBUG]   tactile {stream_name}: fps={image_fps_by_stream[stream_name]}, "
            f"raw_frames={stream_length}, "
            f"used_frames={unique_indices}, repeated_steps={repeats}"
        )


def _validate_frame_keys(frame: Dict[str, object], features: Dict[str, dict], ep_dir: Path) -> None:
    """Raise a detailed error when a frame is missing expected feature keys."""
    expected_keys = set(features.keys()) - set(DEFAULT_FEATURES)
    actual_keys = set(frame.keys()) - {"task", "timestamp"}
    missing_keys = sorted(expected_keys - actual_keys)
    extra_keys = sorted(actual_keys - expected_keys)

    if missing_keys or extra_keys:
        message_lines = [f"Frame key mismatch for episode {ep_dir}:"]
        if missing_keys:
            message_lines.append(f"  Missing features: {missing_keys}")
        if extra_keys:
            message_lines.append(f"  Extra features: {extra_keys}")
        raise ValueError("\n".join(message_lines))


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
    dataset_fps: int,
    proprio_fps: int,
    image_fps_by_stream: Dict[str, int],
    debug_alignment: bool,
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
            f"Expected joint_pos shape [T,7], got {tuple(follower_joint_pos.shape)} in {leader}"
        )
    if leader_gripper.ndim == 1:
        leader_gripper = leader_gripper[:, None]
    if follower_gripper.ndim == 1:
        follower_gripper = follower_gripper[:, None]
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

    for stream_name in [*cam_frames.keys(), *tactile_frames.keys()]:
        if stream_name not in image_fps_by_stream:
            raise ValueError(
                f"Missing fps config for image stream '{stream_name}'. "
                f"Configured streams: {sorted(image_fps_by_stream.keys())}"
            )

    # Compute shared duration and map every modality onto the dataset timeline.
    duration_s = _compute_episode_duration_s(
        leader_joint_pos,
        leader_gripper,
        follower_joint_pos,
        follower_gripper,
        cam_frames,
        tactile_frames,
        follower_external_joint_torque,
        follower_external_wrench,
        follower_gripper_current,
        proprio_fps=proprio_fps,
        image_fps_by_stream=image_fps_by_stream,
    )
    num_master_steps = _compute_num_master_steps(duration_s, dataset_fps)

    if debug_alignment:
        _print_episode_alignment_debug(
            ep_dir=ep_dir,
            duration_s=duration_s,
            num_master_steps=num_master_steps,
            dataset_fps=dataset_fps,
            proprio_length=follower_joint_pos.shape[0],
            image_lengths={cam: arr.shape[0] for cam, arr in cam_frames.items()},
            tactile_lengths={tname: arr.shape[0] for tname, arr in tactile_frames.items()},
            image_fps_by_stream=image_fps_by_stream,
        )

    leader_joint_np = leader_joint_pos.detach().cpu().numpy().astype(np.float32)
    leader_grip_np = leader_gripper.detach().cpu().numpy().astype(np.float32)[:, :1]
    follower_joint_np = follower_joint_pos.detach().cpu().numpy().astype(np.float32)
    follower_grip_np = follower_gripper.detach().cpu().numpy().astype(np.float32)[:, :1]
    follower_external_joint_torque_np = (
        follower_external_joint_torque.detach().cpu().numpy().astype(np.float32)
    )
    follower_external_wrench_np = (
        follower_external_wrench.detach().cpu().numpy().astype(np.float32)
    )
    follower_gripper_current_np = (
        follower_gripper_current.detach().cpu().numpy().astype(np.float32)[:, :1]
    )

    task_instr = task_instruction_mapping[ep_dir.parent.name]

    # Stream frames on the dataset timeline. Images are held with nearest-neighbor
    # sampling when they run slower than the control and proprio streams.
    for master_step in range(num_master_steps - 1):
        obs_idx = _map_master_step_to_stream_index(
            master_step=master_step,
            stream_fps=proprio_fps,
            dataset_fps=dataset_fps,
            stream_length=follower_joint_np.shape[0],
        )
        action_idx = _map_master_step_to_stream_index(
            master_step=master_step,
            stream_fps=proprio_fps,
            dataset_fps=dataset_fps,
            stream_length=leader_joint_np.shape[0],
        )

        state = np.concatenate(
            [follower_joint_np[obs_idx], follower_grip_np[obs_idx]], axis=0
        )
        force_torque = np.concatenate(
            [
                follower_external_joint_torque_np[obs_idx],
                follower_external_wrench_np[obs_idx],
                follower_gripper_current_np[obs_idx],
            ],
            axis=0,
        )
        action = np.concatenate(
            [leader_joint_np[action_idx], leader_grip_np[action_idx]], axis=0
        )

        image_dict = {}
        for cam, arr in cam_frames.items():
            image_idx = _map_master_step_to_stream_index(
                master_step=master_step,
                stream_fps=image_fps_by_stream[cam],
                dataset_fps=dataset_fps,
                stream_length=arr.shape[0],
            )
            # Convert from (H, W, C) to (C, H, W) for LeRobot
            image_dict[f"observation.images.{cam.replace('zed_','')}"] = np.transpose(
                arr[image_idx], (2, 0, 1)
            )
        for tname, arr in tactile_frames.items():
            image_idx = _map_master_step_to_stream_index(
                master_step=master_step,
                stream_fps=image_fps_by_stream[tname],
                dataset_fps=dataset_fps,
                stream_length=arr.shape[0],
            )
            # Convert from (H, W, C) to (C, H, W) for LeRobot
            image_dict[f"observation.images.{tname}"] = np.transpose(
                arr[image_idx], (2, 0, 1)
            )

        frame = {
            **image_dict,
            "observation.state": state,
            "observation.force_torque": force_torque,
            "action": action,
            "task": task_instr,
        }
        _validate_frame_keys(frame, lerobot_ds.features, ep_dir)
        lerobot_ds.add_frame(frame)
    lerobot_ds.save_episode()


def create_lerobot_dataset(
    raw_dir: Path,
    local_dir: Optional[Path],
    repo_id: Optional[str],
    push_to_hub: bool,
    robot_type: Optional[str],
    dataset_fps: Optional[int],
    proprio_fps: int,
    image_fps_by_stream: Dict[str, int],
    debug_alignment: bool,
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
    if dataset_fps is None:
        dataset_fps = proprio_fps
    if robot_type is None:
        robot_type = "unknown"

    # Build feature spec
    features = _build_features_spec(cam_shapes, tactile_shapes, use_videos)

    # Create target LeRobot dataset
    ds = LeRobotDataset.create(
        repo_id=repo_id,
        robot_type=robot_type.lower().replace(" ", "_").replace("-", "_"),
        root=out_root,
        fps=int(dataset_fps),
        use_videos=use_videos,
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
                dataset_fps=dataset_fps,
                proprio_fps=proprio_fps,
                image_fps_by_stream=image_fps_by_stream,
                debug_alignment=debug_alignment,
                task_instruction_mapping=task_instruction_mapping,
            )
            print(f"[OK]   episode {ep}")
        except Exception as e:
            failures += 1
            print(f"[WARN] skipping episode {ep}: {e}")

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
    # Configuration - modify these variables as needed
    repo_id = (
        "ZhuoyueLLL/new_scarf_100hz_cam_25hz"  # HF repo id (e.g. user/dataset). Required if push_to_hub=True
    )
    # raw_dir = Path(f"/home/irl-admin/new_data_collection/{repo_id}")
    raw_dir = Path(f"/home/irl-admin/new_data_collection/new_scarf_100hz_cam_25hz")
    local_dir = Path(
        f"/home/irl-admin/new_data_collection/lerobot_test/{repo_id}"
    )
    push_to_hub = True
    follower_subdir = (
        robot_type
    ) = "FrankaPanda"  # for FLOWER this should be e.g. JOINT_POS
    control_mode = "position"  # Change this if necessary
    num_arms = 1
    dataset_fps = 100 #the final fps of whole dataset
    proprio_fps = 100
    image_fps_by_stream = {
        "zed_right": 25,
        "zed_left": 25,
        "zed_wrist": 25,
    }
    debug_alignment = True
    use_videos = True
    image_writer_process = 5
    image_writer_threads = 10
    keep_images = False

    # Structure configuration
    leader_subdir = "Gello"
    sensors_dirname = "sensors"
    cams = ["zed_right", "zed_left", "zed_wrist"]
    tactile_names = []  # Optional tactile folder names
    resize_w = 256
    resize_h = 256

    # The script looks up the parent dir name of an episode and matches the key
    # in the following directory to identify the correct task instruction
    task_instruction_mapping = {
        "new_scarf_100hz_cam_25hz": "fold the scarf on the table."
    }

    create_lerobot_dataset(
        raw_dir=raw_dir,
        local_dir=local_dir,
        repo_id=repo_id,
        push_to_hub=push_to_hub,
        robot_type=robot_type,
        dataset_fps=dataset_fps,
        proprio_fps=proprio_fps,
        image_fps_by_stream=image_fps_by_stream,
        debug_alignment=debug_alignment,
        use_videos=use_videos,
        image_writer_process=image_writer_process,
        image_writer_threads=image_writer_threads,
        keep_images=keep_images,
        cams=cams,
        tactile_names=tactile_names,
        leader_subdir=leader_subdir,
        follower_subdir=follower_subdir,
        sensors_dirname=sensors_dirname,
        resize_w=resize_w,
        resize_h=resize_h,
        task_instruction_mapping=task_instruction_mapping,
    )


if __name__ == "__main__":
    main()
