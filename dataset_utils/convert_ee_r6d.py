#!/usr/bin/env python
"""Rewrite a WebXR LeRobot dataset (convert_webxr_to_lerobot.py / collect-webxr) to an EE-pose-only R6D layout.

    observation.state (10)  follower EE pos + R6D + gripper        (from observation.ee_pose + state gripper)
    action            (10)  commanded EE pos + R6D + gripper, absolute
    observation.images.*    unchanged (videos are hardlinked, not re-encoded)

observation.ee_pose and observation.force_torque are dropped. The actions stay absolute in the dataset;
chunk-relative positions are applied at training time (train_act_ee_r6d.py), since they depend on the chunk start.

    pixi run python dataset_utils/convert_ee_r6d.py \
        ../../recordings/lerobot/insert_pcie_8 ../../recordings/lerobot/insert_pcie_8_ee_r6d

--interventions-only (rollout recordings with observation.intervention, droid_rollout/recorder.py): every
operator segment becomes an episode, preceded by the --history frames before the takeover (intervention 0
there), so the policy's observation history at the takeover is the one it really saw. Only rows with
intervention 1 carry operator actions; the trainer samples just those. Episodes without an intervention are
dropped. The videos stay whole; each episode's from_timestamp points at its first kept frame.

    pixi run python dataset_utils/convert_ee_r6d.py --interventions-only --history 14 \
        ../../recordings/rollouts/ode_pcie8_v1 ../../recordings/lerobot/insert_pcie_8_v1_dagger_iv_ee_r6d
"""

import argparse
import json
import os
import shutil
from pathlib import Path

import numpy as np
import pandas as pd

from franka_control_client.policy.ee_r6d import POSE_DIM, POSE_NAMES, ee_pose_to_vec

DROP = ["observation.ee_pose", "observation.force_torque"]
QS = {"q01": 1, "q10": 10, "q50": 50, "q90": 90, "q99": 99}
IV = "observation.intervention"
SCALARS = ["timestamp", "frame_index", "episode_index", "index", "task_index", IV]


def _stats(x: np.ndarray) -> dict[str, np.ndarray]:
    s = {"min": x.min(0), "max": x.max(0), "mean": x.mean(0), "std": x.std(0), "count": np.array([len(x)])}
    s.update({k: np.percentile(x, q, axis=0) for k, q in QS.items()})
    return s


def _convert_frames(df: pd.DataFrame) -> pd.DataFrame:
    ee = np.stack(df["observation.ee_pose"].values)
    state = np.stack(df["observation.state"].values)
    act = np.stack(df["action"].values)
    df = df.drop(columns=DROP)
    df["observation.state"] = list(ee_pose_to_vec(ee[:, :3], ee[:, 3:7], state[:, 7]))
    df["action"] = list(ee_pose_to_vec(act[:, :3], act[:, 3:7], act[:, 7]))
    return df


def _segments(flag: np.ndarray) -> list[tuple[int, int]]:
    d = np.diff(np.r_[0, flag.astype(np.int8), 0])
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def _cut_interventions(frames: pd.DataFrame, eps: pd.DataFrame, history: int, fps: int,
                       video_keys: list[str]) -> tuple[pd.DataFrame, pd.DataFrame]:
    """One episode per operator segment, with `history` frames before it; all frames in one data file."""
    out_frames, out_eps, n = [], [], 0
    for _, e in eps.iterrows():
        g = frames.iloc[int(e["dataset_from_index"]):int(e["dataset_to_index"])]
        assert (g["episode_index"].values == e["episode_index"]).all()
        flag = np.asarray(g[IV].tolist()).reshape(-1) > 0.5
        for x, y in _segments(flag):
            s = max(0, x - history)
            c = g.iloc[s:y].copy()
            t0 = float(c["timestamp"].iloc[0])
            k = len(out_eps)
            c["episode_index"] = k
            c["frame_index"] = np.arange(len(c))
            c["timestamp"] = (c["timestamp"].values - t0).astype(np.float32)
            c["index"] = np.arange(n, n + len(c))
            row = e.copy()
            row["episode_index"], row["length"] = k, len(c)
            row["dataset_from_index"], row["dataset_to_index"] = n, n + len(c)
            row["data/chunk_index"] = row["data/file_index"] = 0
            row["meta/episodes/chunk_index"] = row["meta/episodes/file_index"] = 0
            for v in video_keys:
                row[f"videos/{v}/from_timestamp"] = e[f"videos/{v}/from_timestamp"] + t0
                row[f"videos/{v}/to_timestamp"] = row[f"videos/{v}/from_timestamp"] + len(c) / fps
            out_frames.append(c)
            out_eps.append(row)
            n += len(c)
    return pd.concat(out_frames, ignore_index=True), pd.DataFrame(out_eps).reset_index(drop=True)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("src", type=Path)
    p.add_argument("dst", type=Path)
    p.add_argument("--overwrite", action="store_true")
    p.add_argument("--interventions-only", action="store_true", help="keep only the operator segments")
    p.add_argument("--history", type=int, default=14,
                   help="frames kept before every takeover (ODE: (n_obs_steps - 1) * stride = 14)")
    args = p.parse_args()

    if args.dst.exists():
        if not args.overwrite:
            raise SystemExit(f"{args.dst} exists (use --overwrite)")
        shutil.rmtree(args.dst)

    info = json.loads((args.src / "meta/info.json").read_text())
    video_keys = [k for k, v in info["features"].items() if v["dtype"] == "video"]

    # videos: hardlink (same filesystem) or copy
    for f in (args.src / "videos").rglob("*.mp4"):
        out = args.dst / f.relative_to(args.src)
        out.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.link(f, out)
        except OSError:
            shutil.copy2(f, out)
    (args.dst / "meta").mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.src / "meta/tasks.parquet", args.dst / "meta/tasks.parquet")

    # frames
    files = {f.relative_to(args.src): _convert_frames(pd.read_parquet(f))
             for f in sorted((args.src / "data").rglob("*.parquet"))}
    ep_files = {f.relative_to(args.src): pd.read_parquet(f)
                for f in sorted((args.src / "meta/episodes").rglob("*.parquet"))}
    if args.interventions_only:
        assert IV in info["features"], f"{args.src} has no {IV}"
        frames = pd.concat(files.values()).sort_values("index").reset_index(drop=True)
        eps = pd.concat(ep_files.values()).sort_values("episode_index").reset_index(drop=True)
        frames, eps = _cut_interventions(frames, eps, args.history, int(info["fps"]), video_keys)
        files = {Path("data/chunk-000/file-000.parquet"): frames}
        ep_files = {Path("meta/episodes/chunk-000/file-000.parquet"): eps}
    for rel, df in files.items():
        out = args.dst / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        df.to_parquet(out, index=False)
    frames = pd.concat(files.values())

    # per-episode stats in meta/episodes
    by_ep = {i: g for i, g in frames.groupby("episode_index")}
    keys = ["observation.state", "action"]
    if args.interventions_only:
        keys += [k for k in SCALARS if k in frames.columns]
    for rel, ep in ep_files.items():
        ep = ep.drop(columns=[c for c in ep.columns if any(c.startswith(f"stats/{k}/") for k in DROP)])
        for key in keys:
            rows = [_stats(np.stack(by_ep[i][key].values).reshape(len(by_ep[i]), -1)) for i in ep["episode_index"]]
            for stat in rows[0]:
                ep[f"stats/{key}/{stat}"] = [r[stat] for r in rows]
        out = args.dst / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        ep.to_parquet(out, index=False)

    # global stats
    stats = json.loads((args.src / "meta/stats.json").read_text())
    for k in DROP:
        stats.pop(k, None)
    for key in keys:
        x = np.stack(frames[key].values).reshape(len(frames), -1)
        stats[key] = {k: v.tolist() for k, v in _stats(x).items()}
    (args.dst / "meta/stats.json").write_text(json.dumps(stats, indent=4))

    # info
    for k in DROP:
        info["features"].pop(k)
    for key in ["observation.state", "action"]:
        info["features"][key] = {"dtype": "float32", "shape": [POSE_DIM], "names": POSE_NAMES}
    info["total_episodes"], info["total_frames"] = len(by_ep), len(frames)
    info["splits"] = {"train": f"0:{len(by_ep)}"}
    (args.dst / "meta/info.json").write_text(json.dumps(info, indent=4))

    print(f"wrote {len(frames)} frames, {len(by_ep)} episodes -> {args.dst}")


if __name__ == "__main__":
    main()
