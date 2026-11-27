"""Plot one saved BEAST rollout from model plans through physical tracking.

The policy server must have been started with ``--action-diagnostics`` and the
rollout must be saved (not discarded) in the inference GUI.  The resulting
figure aligns, for every action dimension:

* active System-2 coarse plans;
* per-step System-1 refined futures;
* the action returned by the policy;
* the post-limiter command published by the controller; and
* the measured end-effector pose.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


NAMES = ("x", "y", "z", "qx", "qy", "qz", "qw", "gripper")
COLORS = {
    "measured": "#0072B2",
    "command": "#E69F00",
    "generated": "#009E73",
    "sys1": "#D55E00",
    "sys2": "#6A3D9A",
}


def _episode_path(value: Path) -> tuple[Path, Path]:
    npz = value if value.name == "episode.npz" else value / "episode.npz"
    if not npz.is_file():
        raise FileNotFoundError(f"No episode.npz at {npz}")
    return npz, npz.parent


def _first_rows(chunks: np.ndarray) -> np.ndarray:
    if chunks.ndim != 3 or chunks.shape[2] < 8:
        raise ValueError(f"Expected [calls, rows, 8] actions, got {chunks.shape}")
    return chunks[:, 0, :8]


def _valid_plan(plan: np.ndarray) -> Optional[np.ndarray]:
    if plan.ndim != 2 or plan.shape[1] < 8:
        return None
    keep = np.all(np.isfinite(plan[:, :8]), axis=1)
    return plan[keep, :8] if np.any(keep) else None


def _interp(t: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    if not len(t):
        return np.full((len(query), values.shape[1]), np.nan)
    return np.column_stack([np.interp(query, t, values[:, j]) for j in range(values.shape[1])])


def _hold(t: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    if not len(t):
        return np.full(len(query), np.nan)
    idx = np.clip(np.searchsorted(t, query, side="right") - 1, 0, len(t) - 1)
    return values[idx]


def _percentiles(values: np.ndarray) -> dict[str, Optional[float]]:
    values = values[np.isfinite(values)]
    if not values.size:
        return {"p50": None, "p95": None, "max": None}
    return {
        "p50": round(float(np.percentile(values, 50)), 3),
        "p95": round(float(np.percentile(values, 95)), 3),
        "max": round(float(np.max(values)), 3),
    }


def plot_rollout(npz_path: Path, output: Path, fps: float) -> dict[str, object]:
    data = np.load(npz_path)
    required = (
        "policy/t_obs",
        "policy/t_recv",
        "policy/action_raw",
        "policy/sys1_plan",
        "policy/sys2_plan",
        "policy/obs_timestep",
        "policy/sys2_source_timestep",
        "state/t",
        "state/ee_pos",
        "state/ee_quat",
        "command/arm_t",
        "command/arm",
    )
    missing = [key for key in required if key not in data]
    if missing:
        raise KeyError(
            "Rollout lacks BEAST diagnostics: " + ", ".join(missing)
            + ". Restart the server with --action-diagnostics and save a new rollout."
        )

    t_obs = data["policy/t_obs"]
    t_recv = data["policy/t_recv"]
    generated = _first_rows(data["policy/action_raw"])
    sys1 = data["policy/sys1_plan"]
    sys2 = data["policy/sys2_plan"]
    obs_steps = data["policy/obs_timestep"].astype(int)
    source_steps = data["policy/sys2_source_timestep"].astype(int)

    state_t = data["state/t"]
    measured = np.column_stack(
        [data["state/ee_pos"], data["state/ee_quat"], data["state/gripper"]]
    )
    cmd_t = data["command/arm_t"]
    cmd_arm = data["command/arm"]
    grip_t = data.get("command/gripper_t", np.zeros(0))
    grip = data.get("command/gripper", np.zeros(0))

    all_starts = [a[0] for a in (t_obs, state_t, cmd_t) if len(a)]
    t0 = min(all_starts)
    measured_at_infer = _interp(state_t, measured, t_recv)
    command_at_infer = np.column_stack(
        [_interp(cmd_t, cmd_arm, t_recv), _hold(grip_t, grip, t_recv)]
    )

    step_to_time = {int(step): float(t) for step, t in zip(obs_steps, t_obs)}
    fig, axes = plt.subplots(4, 2, figsize=(17, 14), sharex=True)
    axes = axes.ravel()
    sys1_stride = max(1, len(t_recv) // 80)
    last_source = None
    for i in range(len(t_recv)):
        plan2 = _valid_plan(sys2[i])
        source = int(source_steps[i])
        if plan2 is not None and source >= 0 and source != last_source:
            anchor_t = step_to_time.get(source, float(t_obs[i]))
            x2 = anchor_t - t0 + np.arange(len(plan2)) / fps
            for j, ax in enumerate(axes):
                ax.plot(x2, plan2[:, j], color=COLORS["sys2"], alpha=0.38, lw=1.1)
            last_source = source

        if i % sys1_stride == 0 or i == len(t_recv) - 1:
            plan1 = _valid_plan(sys1[i])
            if plan1 is not None:
                x1 = t_recv[i] - t0 + np.arange(len(plan1)) / fps
                for j, ax in enumerate(axes):
                    ax.plot(x1, plan1[:, j], color=COLORS["sys1"], alpha=0.12, lw=0.8)

    for j, ax in enumerate(axes):
        ax.plot(state_t - t0, measured[:, j], color=COLORS["measured"], lw=2.0, label="measured")
        if j < 7:
            ax.plot(cmd_t - t0, cmd_arm[:, j], color=COLORS["command"], lw=1.0, label="command sent")
        elif len(grip_t):
            ax.step(grip_t - t0, grip, where="post", color=COLORS["command"], lw=1.0, label="command sent")
        ax.plot(t_recv - t0, generated[:, j], color=COLORS["generated"], ls="--", lw=1.4, label="policy output")
        valid = np.isfinite(sys1[:, 0, j])
        ax.scatter(
            t_recv[valid] - t0,
            sys1[valid, 0, j],
            color=COLORS["sys1"],
            s=7,
            alpha=0.65,
            label="System 1 t=0",
            zorder=5,
        )
        ax.set_title(NAMES[j])
        ax.grid(alpha=0.2)
        if j >= 6:
            ax.set_xlabel("rollout time (s)")
    handles = [
        plt.Line2D([], [], color=COLORS["measured"], lw=2, label="measured robot pose"),
        plt.Line2D([], [], color=COLORS["command"], lw=1.2, label="published command"),
        plt.Line2D([], [], color=COLORS["generated"], ls="--", label="policy output"),
        plt.Line2D([], [], color=COLORS["sys1"], lw=1.2, label="System 1 refined plan"),
        plt.Line2D([], [], color=COLORS["sys2"], lw=1.2, label="System 2 coarse plan"),
    ]
    fig.legend(handles=handles, loc="upper center", ncol=5)
    fig.suptitle("BEAST action pipeline: plans → policy → limiter/controller → robot", y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    plt.close(fig)

    tracking_mm = 1e3 * np.linalg.norm(command_at_infer[:, :3] - measured_at_infer[:, :3], axis=1)
    policy_command_mm = 1e3 * np.linalg.norm(generated[:, :3] - command_at_infer[:, :3], axis=1)
    displacement_mm = 1e3 * float(np.linalg.norm(measured[-1, :3] - measured[0, :3]))
    summary: dict[str, object] = {
        "calls": int(len(t_recv)),
        "duration_s": round(float(max(state_t[-1], t_recv[-1]) - t0), 3),
        "measured_translation_displacement_mm": round(displacement_mm, 3),
        "command_tracking_error_mm": _percentiles(tracking_mm),
        "policy_to_command_translation_gap_mm": _percentiles(policy_command_mm),
        "figure": str(output),
    }
    summary_path = output.with_suffix(".json")
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    csv_path = output.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        fields = ["step", "time_s"]
        for prefix in ("measured", "command", "generated", "sys1_t0"):
            fields.extend(f"{prefix}_{name}" for name in NAMES)
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for i in range(len(t_recv)):
            row: dict[str, object] = {"step": int(obs_steps[i]), "time_s": float(t_recv[i] - t0)}
            values = {
                "measured": measured_at_infer[i],
                "command": command_at_infer[i],
                "generated": generated[i],
                "sys1_t0": sys1[i, 0],
            }
            for prefix, vector in values.items():
                row.update({f"{prefix}_{name}": float(vector[j]) for j, name in enumerate(NAMES)})
            writer.writerow(row)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rollout", type=Path, help="saved rollout folder or its episode.npz")
    parser.add_argument("--fps", type=float, default=20.0, help="policy plan rate")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    npz, folder = _episode_path(args.rollout.expanduser().resolve())
    output = args.output or folder / "beast_action_diagnostics.png"
    summary = plot_rollout(npz, output, args.fps)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
