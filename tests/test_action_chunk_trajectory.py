import time
from pathlib import Path
from typing import List, Optional, Tuple, Union

import numpy as np
import pyzlc
import pytest
import torch

from franka_control_client.control_pair.pil_panda_control_pair import (
    DEFAULT_CONTROL_HZ,
    GRIPPER_DEADBAND,
    GRIPPER_FORCE,
    GRIPPER_SPEED,
    PILMode,
    PILPandaControlPair,
)
from franka_control_client.data_collection.irl_data_collection import (
    IRLDataCollection,
)
from franka_control_client.data_collection.irl_wrapper import IRLDataWrapper
from franka_control_client.franka_robot.panda_arm import RemotePandaArm
from franka_control_client.franka_robot.panda_gripper import RemotePandaGripper
from franka_control_client.franka_robot.panda_robotiq import PandaRobotiq
from franka_control_client.robotiq_gripper.robotiq_gripper import (
    RemoteRobotiqGripper,
)
from franka_control_client.vr.meta_quest3 import MQ3Controller


SourceTag = float

POLICY_SOURCE: SourceTag = 0.0
INTERRUPT_SOURCE: SourceTag = 1.0


class ActionChunkLoader:
    """Flattens the saved `(N, chunk_size, 8)` chunks into a per-step stream.

    The recorder writes one row per chunk pair:
      - `policy_action_chunks.pt`     `(N, chunk_size, 8)`
      - `correction_action_chunks.pt` `(N, chunk_size, 8)` (zero-padded for rows with no leader samples)
      - `source.pt`                   `(N, chunk_size)` per-slot origin tag
        (0.0 = policy action, 1.0 = leader correction sample)

    `series="policy"` replays the policy chunk on every row.
    `series="correction"` replays the correction chunk where one was
    captured, and falls back to the policy chunk for rows that have no
    leader samples (their correction chunk is zero-padded and would
    otherwise crash the arm into the origin).
    """

    def __init__(
        self,
        record_dir: Union[str, Path],
        series: str = "policy",
    ):
        if series not in ("policy", "correction"):
            raise ValueError(
                f"series must be 'policy' or 'correction', got {series!r}"
            )
        self.record_dir = Path(record_dir)
        self.chunks_dir = self.record_dir / "action_chunks"
        if not self.chunks_dir.exists():
            raise FileNotFoundError(
                f"No action_chunks directory at {self.chunks_dir}. "
                "This episode was collected before chunk-pair recording was supported."
            )

        self.series = series
        self.policy_chunks, self.correction_chunks, self.source = (
            self._load_tensors()
        )
        self._validate_shapes()

        self.num_rows, self.chunk_size, self.action_dim = (
            self.policy_chunks.shape
        )
        if self.action_dim < 8:
            raise ValueError(
                f"Expected action_dim >= 8 (pos3 + quat4 + gripper1), got {self.action_dim}"
            )

        self._cursor_row = 0
        self._cursor_slot = 0

    def _load_tensors(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        policy_path = self.chunks_dir / "policy_action_chunks.pt"
        correction_path = self.chunks_dir / "correction_action_chunks.pt"
        source_path = self.chunks_dir / "source.pt"

        for path in (policy_path, correction_path, source_path):
            if not path.exists():
                raise FileNotFoundError(f"Missing {path}")

        policy = (
            torch.load(policy_path, weights_only=True).cpu().numpy()
        )
        correction = (
            torch.load(correction_path, weights_only=True).cpu().numpy()
        )
        source = torch.load(source_path, weights_only=True).cpu().numpy()
        return policy, correction, source

    def _validate_shapes(self) -> None:
        if self.policy_chunks.shape != self.correction_chunks.shape:
            raise ValueError(
                "policy and correction chunk arrays must have identical shape; "
                f"got {self.policy_chunks.shape} vs {self.correction_chunks.shape}"
            )
        if self.policy_chunks.ndim != 3:
            raise ValueError(
                f"Expected (N, T, D) for action chunks, got {self.policy_chunks.shape}"
            )
        n_rows, chunk_size, _ = self.policy_chunks.shape
        if self.source.ndim != 2 or self.source.shape != (n_rows, chunk_size):
            raise ValueError(
                f"Expected source shape ({n_rows}, {chunk_size}) — one flag per slot — "
                f"got {self.source.shape}. Episodes recorded with the old per-row "
                "source format are not supported by this loader."
            )

    def _row_has_leader_samples(self, row_idx: int) -> bool:
        return bool(np.any(self.source[row_idx] >= 0.5))

    def _current_chunk(self) -> np.ndarray:
        if self.series == "policy":
            return self.policy_chunks[self._cursor_row]
        # Correction series: replay the captured correction chunk when
        # the row has at least one leader sample; otherwise the
        # correction tensor is the zero-padded sentinel for a pure
        # policy-rollout row, so fall back to the policy chunk instead
        # (so the arm follows what was actually executed during that
        # window rather than crashing into the origin).
        if self._row_has_leader_samples(self._cursor_row):
            return self.correction_chunks[self._cursor_row]
        return self.policy_chunks[self._cursor_row]

    def pop(
        self,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float, float, int, int]]:
        """Return `(pos, quat, gripper, per_slot_source, row_idx, slot_idx)` or None when done.

        `per_slot_source` is read directly from `source.pt[row, slot]`:
        0.0 for policy actions, 1.0 for leader correction samples. In the
        policy series the result is forced to 0.0 because every slot of a
        policy chunk is, by construction, a policy prediction. In the
        correction series, fallback rows (no leader samples; chunk taken
        from policy_chunks) naturally return 0.0 since their stored
        source row is all zeros.
        """
        if self._cursor_row >= self.num_rows:
            return None

        chunk = self._current_chunk()
        action = chunk[self._cursor_slot]
        pos = np.asarray(action[0:3], dtype=np.float64)
        quat = np.asarray(action[3:7], dtype=np.float64)
        gripper = float(action[7])
        row_idx = self._cursor_row
        slot_idx = self._cursor_slot
        if self.series == "policy":
            per_slot_source = 0.0
        else:
            per_slot_source = float(self.source[row_idx, slot_idx])

        self._cursor_slot += 1
        if self._cursor_slot >= self.chunk_size:
            self._cursor_row += 1
            self._cursor_slot = 0

        return pos, quat, gripper, per_slot_source, row_idx, slot_idx

    @staticmethod
    def _format_source(source: float) -> str:
        return "policy" if source < 0.5 else "human_interrupt"


def test_action_chunk_loader_reads_policy_series_in_order(tmp_path):
    chunks_dir = tmp_path / "action_chunks"
    chunks_dir.mkdir()

    policy = torch.tensor(
        [
            [
                [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0, 0.25],
                [0.4, 0.5, 0.6, 0.0, 0.0, 1.0, 0.0, 0.75],
            ],
        ],
        dtype=torch.float64,
    )
    correction = torch.zeros_like(policy)
    source = torch.tensor([[0.0, 0.0]], dtype=torch.float32)

    torch.save(policy, chunks_dir / "policy_action_chunks.pt")
    torch.save(correction, chunks_dir / "correction_action_chunks.pt")
    torch.save(source, chunks_dir / "source.pt")

    loader = ActionChunkLoader(tmp_path, series="policy")

    pos, quat, gripper, src, row, slot = loader.pop()
    np.testing.assert_allclose(pos, [0.1, 0.2, 0.3])
    np.testing.assert_allclose(quat, [0.0, 0.0, 0.0, 1.0])
    assert gripper == pytest.approx(0.25)
    assert src == pytest.approx(0.0)
    assert (row, slot) == (0, 0)

    pos, quat, gripper, src, row, slot = loader.pop()
    np.testing.assert_allclose(pos, [0.4, 0.5, 0.6])
    np.testing.assert_allclose(quat, [0.0, 0.0, 1.0, 0.0])
    assert gripper == pytest.approx(0.75)
    assert (row, slot) == (0, 1)

    assert loader.pop() is None


def test_action_chunk_loader_falls_back_to_policy_for_zero_correction_rows(tmp_path):
    """In the correction series, rows whose correction chunk is the
    zero-padded sentinel (no leader samples) should replay the policy
    chunk for that row instead of being skipped — the user wants the
    full trajectory, not just the corrected segments.
    """
    chunks_dir = tmp_path / "action_chunks"
    chunks_dir.mkdir()

    policy = torch.tensor(
        [
            [[0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]],
            [[0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]],
        ],
        dtype=torch.float64,
    )
    correction = torch.tensor(
        [
            [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],  # all zeros → fall back to policy
            [[0.9, 0.8, 0.7, 1.0, 0.0, 0.0, 0.0, 0.5]],
        ],
        dtype=torch.float64,
    )
    source = torch.tensor([[0.0], [1.0]], dtype=torch.float32)

    torch.save(policy, chunks_dir / "policy_action_chunks.pt")
    torch.save(correction, chunks_dir / "correction_action_chunks.pt")
    torch.save(source, chunks_dir / "source.pt")

    loader = ActionChunkLoader(tmp_path, series="correction")

    # Row 0 has no leader samples → replays the policy chunk for that row.
    pos, _quat, _gripper, src, row, _slot = loader.pop()
    np.testing.assert_allclose(pos, [0.1, 0.0, 0.0])
    assert src == pytest.approx(0.0)
    assert row == 0

    # Row 1 has a leader sample → replays the correction chunk.
    pos, quat, gripper, src, row, _slot = loader.pop()
    np.testing.assert_allclose(pos, [0.9, 0.8, 0.7])
    np.testing.assert_allclose(quat, [1.0, 0.0, 0.0, 0.0])
    assert gripper == pytest.approx(0.5)
    assert src == pytest.approx(1.0)
    assert row == 1

    assert loader.pop() is None


def test_action_chunk_loader_rejects_episode_without_chunks(tmp_path):
    with pytest.raises(FileNotFoundError, match="No action_chunks directory"):
        ActionChunkLoader(tmp_path)


def test_action_chunk_loader_reports_per_slot_source_for_mixed_row(tmp_path):
    """Within a single interrupt row, slots [0:k] and [k+j:T] are policy
    padding (source=0) and slots [k:k+j] are actual leader samples
    (source=1). The loader must report each slot's true origin even
    though they share a row.
    """
    chunks_dir = tmp_path / "action_chunks"
    chunks_dir.mkdir()

    chunk_size = 4
    policy = torch.tensor(
        [
            [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            ],
        ],
        dtype=torch.float64,
    )
    correction = torch.tensor(
        [
            [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],  # policy padding (k=2 prefix)
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],  # policy padding
                [0.7, 0.7, 0.7, 1.0, 0.0, 0.0, 0.0, 1.0],  # leader sample
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],  # policy padding (release)
            ],
        ],
        dtype=torch.float64,
    )
    source = torch.tensor([[0.0, 0.0, 1.0, 0.0]], dtype=torch.float32)

    torch.save(policy, chunks_dir / "policy_action_chunks.pt")
    torch.save(correction, chunks_dir / "correction_action_chunks.pt")
    torch.save(source, chunks_dir / "source.pt")

    loader = ActionChunkLoader(tmp_path, series="correction")
    observed = []
    while True:
        result = loader.pop()
        if result is None:
            break
        _pos, _quat, _gripper, src, _row, slot = result
        observed.append((slot, src))

    assert observed == [
        (0, 0.0),  # policy padding
        (1, 0.0),  # policy padding
        (2, 1.0),  # leader correction
        (3, 0.0),  # policy padding on release
    ]

    # And the policy series for the same row reports source=0 everywhere
    # since the policy chunk is pure policy by construction.
    loader_policy = ActionChunkLoader(tmp_path, series="policy")
    policy_sources = [loader_policy.pop()[3] for _ in range(chunk_size)]
    assert policy_sources == [0.0, 0.0, 0.0, 0.0]


def test_action_chunk_loader_rejects_legacy_per_row_source(tmp_path):
    chunks_dir = tmp_path / "action_chunks"
    chunks_dir.mkdir()
    policy = torch.zeros((1, 2, 8), dtype=torch.float64)
    torch.save(policy, chunks_dir / "policy_action_chunks.pt")
    torch.save(policy.clone(), chunks_dir / "correction_action_chunks.pt")
    torch.save(
        torch.tensor([1.0], dtype=torch.float32),  # legacy per-row scalar
        chunks_dir / "source.pt",
    )

    with pytest.raises(ValueError, match="one flag per slot"):
        ActionChunkLoader(tmp_path)


class ActionChunkReplayControlPair(PILPandaControlPair):
    def __init__(
        self,
        panda_arm: RemotePandaArm,
        gripper: Union[RemotePandaGripper, RemoteRobotiqGripper],
        mq3_controller: MQ3Controller,
        replay_path: str,
        series: str = "policy",
        per_action_dt: float = 0.1,
        control_hz: float = DEFAULT_CONTROL_HZ,
    ):
        super().__init__(panda_arm, gripper, mq3_controller, control_hz)
        self._last_gripper_cmd = None
        self.current_state = PILMode.REPLAY
        self.per_action_dt = float(per_action_dt)
        self.data_loader = ActionChunkLoader(replay_path, series=series)

    def _replay(self):
        previous_state = self.current_state
        last_row = -1
        while True:
            self.current_state = PILMode.REPLAY
            result = self.data_loader.pop()
            if result is None:
                break
            pos, quat, gripper_width, slot_source, row_idx, slot_idx = result

            if row_idx != last_row:
                row_slot_sources = self.data_loader.source[row_idx]
                num_corrections = int(np.sum(row_slot_sources >= 0.5))
                row_label = (
                    "human_interrupt"
                    if num_corrections > 0
                    else "policy"
                )
                print(
                    f"--- Chunk row {row_idx} "
                    f"(row={row_label}, "
                    f"correction_slots={num_corrections}/{self.data_loader.chunk_size}, "
                    f"series={self.data_loader.series}) ---"
                )
                last_row = row_idx

            self.panda_arm.send_cartesian_pose_command(pos=pos, rot=quat)

            gripper_cmd = 1.0 if gripper_width >= 0.5 else 0.0
            if (
                self._last_gripper_cmd is None
                or abs(gripper_cmd - self._last_gripper_cmd) > GRIPPER_DEADBAND
            ):
                self.gripper.send_grasp_command(
                    position=gripper_cmd,
                    speed=GRIPPER_SPEED,
                    force=GRIPPER_FORCE,
                    blocking=False,
                )
                self._last_gripper_cmd = gripper_cmd

            print(
                f"  slot {slot_idx:02d}: "
                f"source={self.data_loader._format_source(slot_source)} ({slot_source:.1f}), "
                f"pos={pos.round(3).tolist()}, "
                f"gripper={gripper_width:.2f}"
            )
            self.reset_action()
            pyzlc.sleep(self.per_action_dt)

        self.current_state = previous_state


if __name__ == "__main__":
    pyzlc.init(
        "data_collection",
        "141.3.53.25",
        group="224.0.0.1",
        group_name="robot_lab_robotiq_202",
        group_port=7725,
    )
    follower = PandaRobotiq(
        "PandaRobotiq",
        RemotePandaArm("FrankaPanda"),
        RemoteRobotiqGripper("FrankaPanda"),
    )
    leader = MQ3Controller("IRL-MQ3-2", "192.168.0.117", follower.panda_arm)

    # Switch `series` to "correction" to replay the human-correction chunks
    # and switch to "policy" to replay the policy chunks. Set per_action_dt to 1/5 = 0.2 to match the original policy
    # cadence (5 Hz), or 1/40 = 0.025 to match the correction-sampling
    # cadence; the default 0.1 is a safe compromise for visual inspection.
    replay_path = (
        "/home/jjiang/ahmad/dataset/green_on_yellow/2026_05_29-12_14_59/"
    )
    control_pair = ActionChunkReplayControlPair(
        panda_arm=follower.panda_arm,
        gripper=follower.robotiq_gripper,
        mq3_controller=leader,
        replay_path=replay_path,
        series="correction",
        per_action_dt=0.1,
        control_hz=50,
    )

    task = "pick_up_cylinder_on_the_top_of_cube"
    data_collectors: List[IRLDataWrapper] = []
    data_collection_manager = IRLDataCollection(
        data_collectors,
        f"/home/jjiang/ahmad/dataset/lerobot/{task}",
        task,
        fps=20,
        control_pair=control_pair,
    )
    control_pair.control_reset()
    data_collection_manager.run()
    pyzlc.shutdown()
