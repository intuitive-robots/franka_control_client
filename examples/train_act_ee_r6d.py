#!/usr/bin/env python
"""Train an ACT policy on an EE-pose R6D dataset (dataset_utils/convert_ee_r6d.py) with chunk-relative positions.

    observation.state (10)  EE pos + R6D + gripper, absolute
    action       (T, 10)    EE pos relative to observation.state pos at the chunk start, R6D + gripper absolute

The relative transform happens on the batch before normalization, and the action normalization stats are
computed on the relative chunks. At inference, add the observed EE position back to the predicted chunk
(franka_control_client.policy.ee_r6d.from_chunk_relative); the checkpoint's ee_r6d.json records this.

    pixi run train-act --dataset ../../recordings/lerobot/insert_pcie_8_ee_r6d --output outputs/act_pcie

Several --dataset (same features, plus observation.intervention in rollout data) train together; the last
N episodes of the last one are held out. In datasets with observation.intervention
(convert_ee_r6d.py --interventions-only) only the operator's rows are sampled; chunks running past an
episode end are padded and masked as usual.
"""

import argparse
import json
import logging
import math
import time
from pathlib import Path

import numpy as np
import pandas as pd
import torch
from lerobot.configs.types import FeatureType
from lerobot.datasets.compute_stats import aggregate_stats
from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.datasets.transforms import ImageTransforms, ImageTransformsConfig
from lerobot.datasets.utils import dataset_to_policy_features
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.act.processor_act import make_act_pre_post_processors

from franka_control_client.policy.ee_r6d import POS, from_chunk_relative, to_chunk_relative

log = logging.getLogger("train_act")
IV = "observation.intervention"


def relative_action_stats(sources: list[tuple[Path, list[int]]], chunk: int) -> dict[str, np.ndarray]:
    """Mean/std/min/max of the chunk-relative actions over all (t, k) pairs inside each episode, t sampled."""
    rel = []
    for root, episodes in sources:
        df = pd.concat(pd.read_parquet(f) for f in sorted((root / "data").rglob("*.parquet")))
        df = df[df["episode_index"].isin(episodes)]
        for _, g in df.groupby("episode_index"):
            g = g.sort_values("frame_index")
            a = np.stack(g["action"].values)
            s = np.stack(g["observation.state"].values)
            n = len(a)
            t = np.arange(n)
            if IV in g.columns:
                t = t[np.asarray(g[IV].tolist()).reshape(-1) > 0.5]
            idx = t[:, None] + np.arange(chunk)[None]
            valid = idx < n
            r = a[np.minimum(idx, n - 1)]  # (len(t), chunk, D)
            r[..., POS] -= s[t][:, None, POS]
            rel.append(r[valid])
    rel = np.concatenate(rel)
    return {
        "mean": rel.mean(0),
        "std": rel.std(0),
        "min": rel.min(0),
        "max": rel.max(0),
        "count": np.array([len(rel)]),
    }


def make_dataset(root: Path, episodes: list[int], chunk: int, fps: int, augment: bool) -> LeRobotDataset:
    return LeRobotDataset(
        root.name,
        root=root,
        episodes=episodes,
        delta_timestamps={"action": [i / fps for i in range(chunk)]},
        image_transforms=ImageTransforms(ImageTransformsConfig(enable=True)) if augment else None,
    )


class Sampled(torch.utils.data.Dataset):
    """The anchor rows of a LeRobotDataset (operator rows if it has observation.intervention), keys limited
    to `keys` so that datasets with and without the flag collate together."""

    def __init__(self, ds: LeRobotDataset, keys: set[str]):
        self.ds, self.keys = ds, keys
        if IV in ds.meta.features:
            flag = np.asarray(ds.hf_dataset.with_format(None)[IV]).reshape(-1)
            self.rows = np.flatnonzero(flag > 0.5)
        else:
            self.rows = np.arange(len(ds))

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, i: int) -> dict:
        item = self.ds[int(self.rows[i])]
        return {k: v for k, v in item.items() if k in self.keys}


def to_device(batch: dict, device: str) -> dict:
    return {k: v.to(device, non_blocking=True) if isinstance(v, torch.Tensor) else v for k, v in batch.items()}


@torch.no_grad()
def validate(policy, preprocessor, postprocessor, loader, device, max_batches: int) -> dict[str, float]:
    """L1 loss (normalized) and absolute EE position error of the predicted chunk (no VAE latent)."""
    policy.eval()
    l1, pos_err, pos_err_last, grip_err, n = 0.0, 0.0, 0.0, 0.0, 0
    for i, batch in enumerate(loader):
        if i >= max_batches:
            break
        batch = to_device(batch, device)
        state = batch["observation.state"].clone()
        target = batch["action"]  # absolute
        mask = ~batch["action_is_pad"]
        batch["action"] = to_chunk_relative(target, state)
        nb = preprocessor(batch)
        with torch.autocast("cuda", dtype=torch.bfloat16):
            pred_norm = policy.predict_action_chunk(nb).float()
        l1 += ((pred_norm - nb["action"]).abs().mean(-1)[mask]).mean().item()
        pred = from_chunk_relative(postprocessor(pred_norm).to(device), state)
        err = (pred[..., POS] - target[..., POS]).norm(dim=-1)
        pos_err += err[mask].mean().item()
        pos_err_last += err[:, -1].mean().item()
        grip_err += (pred[..., -1] - target[..., -1]).abs()[mask].mean().item()
        n += 1
    policy.train()
    return {
        "val/l1": l1 / n,
        "val/pos_err_mm": 1e3 * pos_err / n,
        "val/pos_err_last_mm": 1e3 * pos_err_last / n,
        "val/gripper_err": grip_err / n,
    }


def save(output: Path, name: str, policy, preprocessor, postprocessor, meta: dict) -> None:
    d = output / "checkpoints" / name / "pretrained_model"
    policy.save_pretrained(d)
    preprocessor.save_pretrained(d)
    postprocessor.save_pretrained(d)
    (d / "ee_r6d.json").write_text(json.dumps(meta, indent=2))
    log.info("saved %s", d)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dataset", type=Path, nargs="+", required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--steps", type=int, default=100_000)
    p.add_argument("--batch-size", type=int, default=16)
    p.add_argument("--lr", type=float, default=2e-5)
    p.add_argument("--chunk-size", type=int, default=100)
    p.add_argument("--kl-weight", type=float, default=10.0)
    p.add_argument("--val-episodes", type=int, default=4, help="last N episodes held out for validation")
    p.add_argument("--no-augment", action="store_true", help="disable image augmentation")
    p.add_argument("--workers", type=int, default=12)
    p.add_argument("--log-every", type=int, default=100)
    p.add_argument("--val-every", type=int, default=2_000)
    p.add_argument("--save-every", type=int, default=10_000)
    p.add_argument("--wandb", type=str, default=None, help="wandb project (off if unset)")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S", force=True)
    torch.manual_seed(args.seed)
    torch.backends.cudnn.benchmark = True
    device = "cuda"
    roots = [d.resolve() for d in args.dataset]
    root = roots[-1]
    args.output.mkdir(parents=True, exist_ok=True)

    metas = [LeRobotDatasetMetadata(r.name, root=r) for r in roots]
    meta = metas[-1]
    common = {k: v for k, v in meta.features.items() if k != IV}
    for r, m in zip(roots, metas):
        assert m.fps == meta.fps, f"{r}: fps {m.fps} != {meta.fps}"
        missing = [k for k, v in common.items() if k not in m.features or m.features[k]["shape"] != v["shape"]]
        assert not missing, f"{r}: features {missing} differ from {root}"
    all_eps = list(range(meta.total_episodes))
    train_eps, val_eps = all_eps[: -args.val_episodes or None], all_eps[len(all_eps) - args.val_episodes :]
    sources = [(r, list(range(m.total_episodes))) for r, m in zip(roots[:-1], metas[:-1])] + [(root, train_eps)]
    log.info("%d train / %d val episodes, fps %d", sum(len(e) for _, e in sources), len(val_eps), meta.fps)

    features = dataset_to_policy_features(common)
    cfg = ACTConfig(
        input_features={k: f for k, f in features.items() if f.type is not FeatureType.ACTION},
        output_features={k: f for k, f in features.items() if f.type is FeatureType.ACTION},
        chunk_size=args.chunk_size,
        n_action_steps=args.chunk_size,
        kl_weight=args.kl_weight,
        optimizer_lr=args.lr,
        optimizer_lr_backbone=args.lr,
        device=device,
    )

    stats = aggregate_stats([{k: {s: np.asarray(v) for s, v in d.items()} for k, d in m.stats.items() if k in common}
                             for m in metas])
    stats["action"] = relative_action_stats(sources, args.chunk_size)
    log.info("relative action mean %s\n std %s", stats["action"]["mean"].round(4), stats["action"]["std"].round(4))

    policy = ACTPolicy(cfg).to(device)
    preprocessor, postprocessor = make_act_pre_post_processors(cfg, dataset_stats=stats)
    log.info("policy params: %.1fM", sum(p.numel() for p in policy.parameters()) / 1e6)

    keys = set(common) | {"action_is_pad", "task"}
    parts = [Sampled(make_dataset(r, e, args.chunk_size, meta.fps, augment=not args.no_augment), keys)
             for r, e in sources]
    for (r, _), d in zip(sources, parts):
        log.info("%s: %d samples", r.name, len(d))
    train_ds = torch.utils.data.ConcatDataset(parts)
    val_ds = Sampled(make_dataset(root, val_eps, args.chunk_size, meta.fps, augment=False), keys)
    loader = torch.utils.data.DataLoader(
        train_ds, batch_size=args.batch_size, shuffle=True, num_workers=args.workers, pin_memory=True,
        drop_last=True, persistent_workers=True, prefetch_factor=4,
    )
    val_loader = torch.utils.data.DataLoader(
        val_ds, batch_size=args.batch_size, shuffle=True, num_workers=4, pin_memory=True,
        persistent_workers=True, generator=torch.Generator().manual_seed(0),
    )

    optim = torch.optim.AdamW(policy.get_optim_params(), lr=args.lr, weight_decay=cfg.optimizer_weight_decay)
    warmup = 1_000
    sched = torch.optim.lr_scheduler.LambdaLR(
        optim,
        lambda s: min(1.0, (s + 1) / warmup) * (0.1 + 0.9 * 0.5 * (1 + math.cos(math.pi * min(s, args.steps) / args.steps))),
    )

    ckpt_meta = {
        "state": "observation.state = [x, y, z, r6d(6), gripper], absolute",
        "action": "action = [x, y, z, r6d(6), gripper]; xyz relative to observation.state xyz at chunk start",
        "r6d": "first two rotation-matrix columns, [R00, R10, R20, R01, R11, R21]",
        "dataset": str(root),
        "datasets": [str(r) for r in roots],
        "train_episodes": train_eps,
        "val_episodes": val_eps,
        "args": {k: str(v) for k, v in vars(args).items()},
    }
    run = None
    if args.wandb:
        import wandb

        run = wandb.init(project=args.wandb, name=args.output.name, config=ckpt_meta, dir=str(args.output))
    metrics_f = (args.output / "metrics.jsonl").open("a")

    policy.train()
    step, t0, best = 0, time.time(), float("inf")
    data_iter = iter(loader)
    while step < args.steps:
        try:
            batch = next(data_iter)
        except StopIteration:
            data_iter = iter(loader)
            batch = next(data_iter)
        batch = to_device(batch, device)
        batch["action"] = to_chunk_relative(batch["action"], batch["observation.state"])
        batch = preprocessor(batch)
        with torch.autocast("cuda", dtype=torch.bfloat16):
            loss, out = policy.forward(batch)
        loss.backward()
        grad_norm = torch.nn.utils.clip_grad_norm_(policy.parameters(), 10.0)
        optim.step()
        optim.zero_grad(set_to_none=True)
        sched.step()
        step += 1

        rec = None
        if step % args.log_every == 0:
            dt = time.time() - t0
            t0 = time.time()
            rec = {"step": step, "loss": loss.item(), "grad_norm": grad_norm.item(), "lr": sched.get_last_lr()[0],
                   "steps_per_s": args.log_every / dt, **{k: float(v) for k, v in out.items()}}
        if step % args.val_every == 0 or step == args.steps:
            v = validate(policy, preprocessor, postprocessor, val_loader, device, max_batches=50)
            rec = {**(rec or {"step": step}), **v}
            if v["val/l1"] < best:
                best = v["val/l1"]
                save(args.output, "best", policy, preprocessor, postprocessor, {**ckpt_meta, "step": step, **v})
        if rec:
            log.info(" ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in rec.items()))
            metrics_f.write(json.dumps(rec) + "\n")
            metrics_f.flush()
            if run:
                run.log(rec, step=step)
        if step % args.save_every == 0 or step == args.steps:
            save(args.output, f"{step:06d}", policy, preprocessor, postprocessor, {**ckpt_meta, "step": step})
            save(args.output, "last", policy, preprocessor, postprocessor, {**ckpt_meta, "step": step})


if __name__ == "__main__":
    main()
