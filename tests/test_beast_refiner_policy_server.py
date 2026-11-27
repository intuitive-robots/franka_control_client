import json
from pathlib import Path

import numpy as np
import torch

from examples.beast_refiner_policy_server import (
    BeastRefinerPolicy,
    _compatible_checkpoint,
)
from franka_control_client.inference_gui.policy_server import CARTESIAN_ACTION_SPACE


def test_beast_refiner_advertises_cartesian_action_space():
    assert BeastRefinerPolicy.action_space == CARTESIAN_ACTION_SPACE
    assert BeastRefinerPolicy.exec_steps == 1
    assert BeastRefinerPolicy.query_every_step is True


def test_checkpoint_compatibility_view_drops_training_only_fields(tmp_path: Path):
    checkpoint = tmp_path / "checkpoint"
    checkpoint.mkdir()
    (checkpoint / "config.json").write_text(
        json.dumps(
            {
                "type": "beast_xvla_refiner_v2",
                "sys1_l1_loss": False,
                "sys1_cp_loss_weighting": None,
                "plan_guidance_native_residual": False,
                "history_early_fusion": False,
            }
        ),
        encoding="utf-8",
    )
    (checkpoint / "model.safetensors").write_bytes(b"weights")

    compatible_dir = tmp_path / "compatible"
    compatible_dir.mkdir()
    compatible = _compatible_checkpoint(checkpoint, compatible_dir)
    config = json.loads((compatible / "config.json").read_text(encoding="utf-8"))

    assert "sys1_l1_loss" not in config
    assert "sys1_cp_loss_weighting" not in config
    assert "plan_guidance_native_residual" not in config
    assert config["history_early_fusion"] is False
    assert (compatible / "model.safetensors").resolve() == (
        checkpoint / "model.safetensors"
    ).resolve()


def test_beast_diagnostic_plans_are_postprocessed_and_clipped():
    adapter = BeastRefinerPolicy.__new__(BeastRefinerPolicy)
    adapter.postprocessor = lambda value: value * 2
    raw = torch.full((1, 2, 8), 0.25)
    raw[..., 7] = 0.75

    plan = adapter._postprocessed_plan(raw)

    assert plan.shape == (2, 8)
    assert np.allclose(plan[:, :7], 0.5)
    assert np.allclose(plan[:, 7], 1.0)


def test_live_frames_are_resized_like_the_training_dataset(tmp_path: Path):
    import cv2
    import numpy as np

    from examples.beast_refiner_policy_server import _resize_like_training
    from franka_control_client.data_collection_gui.irl_to_lerobot import _read_image

    rng = np.random.default_rng(0)
    live_rgb = rng.integers(0, 256, size=(640, 1280, 3), dtype=np.uint8)
    # The data-collection path: RGB frame -> BGR PNG -> converter resize -> RGB.
    png = tmp_path / "frame.png"
    cv2.imwrite(str(png), cv2.cvtColor(live_rgb, cv2.COLOR_RGB2BGR))
    trained = _read_image(png, (256, 256))

    served = _resize_like_training(live_rgb, (256, 256))
    assert served.shape == (256, 256, 3)
    assert np.array_equal(served, trained)
    same = live_rgb[:256, :256]
    assert _resize_like_training(same, (256, 256)) is same
