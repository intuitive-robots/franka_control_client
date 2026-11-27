from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("matplotlib")
from examples.plot_beast_action_diagnostics import plot_rollout


def test_plot_beast_action_diagnostics_writes_bundle(tmp_path: Path):
    calls, plan_len = 4, 3
    t = 100.0 + np.arange(calls) * 0.1
    state_t = 100.0 + np.arange(20) * 0.02
    measured = np.column_stack(
        [
            0.5 + 0.001 * np.arange(20),
            np.zeros(20),
            np.full(20, 0.3),
        ]
    )
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (20, 1))
    plans = np.tile(np.array([0.52, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0, 0.0]), (calls, plan_len, 1))
    raw = plans[:, :1].copy()
    command_t = state_t.copy()
    command = np.column_stack([measured, quat])
    npz = tmp_path / "episode.npz"
    np.savez_compressed(
        npz,
        **{
            "policy/t_obs": t,
            "policy/t_recv": t + 0.09,
            "policy/action_raw": raw,
            "policy/sys1_plan": plans,
            "policy/sys2_plan": plans,
            "policy/obs_timestep": np.arange(calls),
            "policy/sys2_source_timestep": np.array([0, 0, 2, 2]),
            "state/t": state_t,
            "state/ee_pos": measured,
            "state/ee_quat": quat,
            "state/gripper": np.zeros(20),
            "command/arm_t": command_t,
            "command/arm": command,
            "command/gripper_t": np.array([100.0]),
            "command/gripper": np.array([0.0]),
        },
    )
    output = tmp_path / "diagnostics.png"

    summary = plot_rollout(npz, output, fps=20.0)

    assert summary["calls"] == calls
    assert output.is_file()
    assert output.with_suffix(".json").is_file()
    assert output.with_suffix(".csv").is_file()
