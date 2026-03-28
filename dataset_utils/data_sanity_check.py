import argparse
import re
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, List, Sequence

import torch
# usage 
# python data_sanity_check.py dataset_root
# python data_sanity_check.py /home/irl-admin/new_data_collection/test_proprio_100hz_cam_30hz --delete-failed

EPISODE_DIR_PATTERN = re.compile(r"^\d{4}_\d{2}_\d{2}-\d{2}_\d{2}_\d{2}$")

LEADER_REQUIRED_FILES = (
    "joint_pos.pt",
    "gripper_state.pt",
    "gripper_command.pt",
)

FOLLOWER_REQUIRED_FILES = (
    "ee_pos.pt",
    "joint_pos.pt",
    "joint_vel.pt",
    "external_joint_torque.pt",
    "gripper_state.pt",
    "external_wrench.pt",
    "gripper_current.pt",
)


@dataclass
class CheckResult:
    ok: bool
    message: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sanity-check a saved data collection episode. "
            "You can pass either a task root containing timestamped episode folders "
            "or a specific timestamped episode folder."
        )
    )
    parser.add_argument(
        "path",
        type=Path,
        help="Task root or episode folder to check.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=None,
        help=(
            "Optional path for the detailed text report. "
            "Defaults to '<input>/sanity_check_report.txt' for a task root or "
            "'<episode_parent>/sanity_check_report.txt' for a single episode."
        ),
    )
    parser.add_argument(
        "--delete-failed",
        action="store_true",
        help=(
            "Delete failed episode folders after the checks finish. "
            "Useful for CI cleanup. This is disabled by default."
        ),
    )
    return parser.parse_args()


def is_episode_dir(path: Path) -> bool:
    return path.is_dir() and EPISODE_DIR_PATTERN.fullmatch(path.name) is not None


def parse_episode_time(path: Path) -> datetime:
    return datetime.strptime(path.name, "%Y_%m_%d-%H_%M_%S")


def resolve_episode_dir(path: Path) -> Path:
    path = path.expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Path does not exist: {path}")

    if is_episode_dir(path):
        return path

    candidates = [child for child in path.iterdir() if is_episode_dir(child)]
    if not candidates:
        raise FileNotFoundError(
            f"No timestamped episode folders found under: {path}"
        )

    return max(candidates, key=parse_episode_time)


def resolve_episode_dirs(path: Path) -> list[Path]:
    path = path.expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Path does not exist: {path}")

    if is_episode_dir(path):
        return [path]

    candidates = sorted(
        (child for child in path.iterdir() if is_episode_dir(child)),
        key=parse_episode_time,
    )
    if not candidates:
        raise FileNotFoundError(
            f"No timestamped episode folders found under: {path}"
        )

    return candidates


def load_tensor_file(path: Path) -> CheckResult:
    if not path.is_file():
        return CheckResult(False, f"Missing file: {path}")

    try:
        data = torch.load(path, map_location="cpu")
    except Exception as exc:
        return CheckResult(False, f"Failed to load {path}: {exc}")

    numel = getattr(data, "numel", None)
    if callable(numel):
        try:
            if data.numel() == 0:
                return CheckResult(False, f"Empty tensor data in {path}")
        except Exception as exc:
            return CheckResult(False, f"Could not inspect tensor size for {path}: {exc}")

    return CheckResult(True, f"OK: {path.name}")


def check_required_files(directory: Path, required_files: Sequence[str], label: str) -> List[CheckResult]:
    if not directory.is_dir():
        return [CheckResult(False, f"Missing {label} directory: {directory}")]

    results: List[CheckResult] = []
    for filename in required_files:
        results.append(load_tensor_file(directory / filename))
    return results


def find_role_dirs(episode_dir: Path) -> tuple[Path | None, Path | None]:
    candidate_dirs = [
        child
        for child in episode_dir.iterdir()
        if child.is_dir() and child.name != "sensors"
    ]

    leader_dir = None
    follower_dir = None

    for directory in candidate_dirs:
        files = {child.name for child in directory.iterdir() if child.is_file()}
        if set(LEADER_REQUIRED_FILES).issubset(files):
            leader_dir = directory
        if set(FOLLOWER_REQUIRED_FILES).issubset(files):
            follower_dir = directory

    return leader_dir, follower_dir


def check_root_files(episode_dir: Path) -> List[CheckResult]:
    return [load_tensor_file(episode_dir / "timestamps.pt")]


def check_sensor_dirs(sensors_dir: Path) -> List[CheckResult]:
    if not sensors_dir.is_dir():
        return [CheckResult(False, f"Missing sensors directory: {sensors_dir}")]

    sensor_subdirs = [child for child in sensors_dir.iterdir() if child.is_dir()]
    if not sensor_subdirs:
        return [CheckResult(False, f"No sensor folders found in: {sensors_dir}")]

    results: List[CheckResult] = []
    for sensor_dir in sorted(sensor_subdirs):
        contents = [child for child in sensor_dir.iterdir() if child.is_file()]
        if contents:
            results.append(
                CheckResult(
                    True,
                    f"Sensor folder has content: {sensor_dir.name} ({len(contents)} files)",
                )
            )
        else:
            results.append(
                CheckResult(False, f"Sensor folder is empty: {sensor_dir}")
            )
    return results


def summarize(results: Iterable[CheckResult]) -> tuple[list[str], list[str]]:
    passed = []
    failed = []
    for result in results:
        if result.ok:
            passed.append(result.message)
        else:
            failed.append(result.message)
    return passed, failed


def default_report_path(input_path: Path, episode_dirs: Sequence[Path]) -> Path:
    resolved_input = input_path.expanduser().resolve()
    if is_episode_dir(resolved_input):
        report_name = f"{resolved_input.parent.name}_sanity_check_report.txt"
        return Path.cwd() / report_name
    if resolved_input.is_dir():
        report_name = f"{resolved_input.name}_sanity_check_report.txt"
        return Path.cwd() / report_name
    return Path.cwd() / "sanity_check_report.txt"


def delete_failed_episodes(failed_episodes: Sequence[tuple[Path, list[str]]]) -> list[str]:
    deleted_messages: list[str] = []
    for episode_dir, _ in failed_episodes:
        if not episode_dir.exists():
            deleted_messages.append(f"Skipped missing failed episode: {episode_dir}")
            continue
        shutil.rmtree(episode_dir)
        deleted_messages.append(f"Deleted failed episode: {episode_dir}")
    return deleted_messages


def check_episode(episode_dir: Path) -> tuple[list[str], list[str]]:
    all_results: List[CheckResult] = []
    all_results.extend(check_root_files(episode_dir))

    leader_dir, follower_dir = find_role_dirs(episode_dir)
    if leader_dir is None:
        all_results.append(
            CheckResult(False, "Could not find leader directory with expected .pt files")
        )
    else:
        all_results.extend(
            check_required_files(leader_dir, LEADER_REQUIRED_FILES, "leader")
        )

    if follower_dir is None:
        all_results.append(
            CheckResult(False, "Could not find follower directory with expected .pt files")
        )
    else:
        all_results.extend(
            check_required_files(follower_dir, FOLLOWER_REQUIRED_FILES, "follower")
        )

    all_results.extend(check_sensor_dirs(episode_dir / "sensors"))
    return summarize(all_results)


def main() -> int:
    args = parse_args()

    try:
        episode_dirs = resolve_episode_dirs(args.path)
    except Exception as exc:
        print(f"[FAIL] {exc}")
        return 1

    report_path = (
        args.report.expanduser().resolve()
        if args.report is not None
        else default_report_path(args.path, episode_dirs)
    )

    total_failures = 0
    total_episodes = len(episode_dirs)
    passed_episodes: List[Path] = []
    failed_episodes: List[tuple[Path, list[str]]] = []
    report_lines: List[str] = []

    for episode_dir in episode_dirs:
        passed, failed = check_episode(episode_dir)
        report_lines.append(f"Checking episode folder: {episode_dir}")
        report_lines.append("Passed checks:")
        if passed:
            for message in passed:
                report_lines.append(f"  [OK] {message}")
        else:
            report_lines.append("  None")

        report_lines.append("Failed checks:")
        if failed:
            for message in failed:
                report_lines.append(f"  [FAIL] {message}")
        else:
            report_lines.append("  None")

        if failed:
            total_failures += len(failed)
            failed_episodes.append((episode_dir, failed))
            report_lines.append(f"Episode result: FAILED with {len(failed)} issue(s).")
        else:
            passed_episodes.append(episode_dir)
            report_lines.append("Episode result: PASSED.")
        report_lines.append("")

    report_lines.append(
        f"Checked {total_episodes} episode(s). Total issues found: {total_failures}."
    )

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(report_lines) + "\n", encoding="utf-8")

    print(f"Detailed report saved to: {report_path}")
    print(f"Passed episodes: {len(passed_episodes)}")
    print(f"Failed episodes: {len(failed_episodes)}")
    if failed_episodes:
        print("Failed folder names:")
        for episode_dir, _ in failed_episodes:
            print(f"  {episode_dir}")

    if args.delete_failed and failed_episodes:
        deleted_messages = delete_failed_episodes(failed_episodes)
        with report_path.open("a", encoding="utf-8") as report_file:
            report_file.write("\nDeletion summary:\n")
            for message in deleted_messages:
                report_file.write(f"{message}\n")
        print(f"Deleted failed episodes: {len(deleted_messages)}")

    return 1 if total_failures else 0


if __name__ == "__main__":
    sys.exit(main())
