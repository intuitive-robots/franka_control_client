import argparse
import shutil
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import torch


EPISODE_TIME_FORMAT = "%Y_%m_%d-%H_%M_%S"
EPISODE_NAME_LEN = len("YYYY_MM_DD-HH_MM_SS")
TRAJ_FILES = ("joint_pos.pt", "gripper_state.pt")


@dataclass
class FixResult:
    episode_dir: Path
    ok: bool
    changed: bool
    lines: List[str]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fix replay Traj length mismatches by matching Traj/*.pt to "
            "FrankaPanda/joint_pos.pt. Short Traj tensors are padded with their "
            "last sample; long Traj tensors are truncated."
        )
    )
    parser.add_argument(
        "path",
        type=Path,
        help="Dataset/task root containing timestamped episodes, or one episode folder.",
    )
    parser.add_argument(
        "--robot-dir",
        default="FrankaPanda",
        help="Robot state directory name used as the target length source.",
    )
    parser.add_argument(
        "--traj-dir",
        default="Traj",
        help="Trajectory directory name to repair.",
    )
    parser.add_argument(
        "--target-file",
        default="joint_pos.pt",
        help="File inside --robot-dir whose first dimension is the target length.",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Actually overwrite Traj files. Without this flag the script only reports.",
    )
    parser.add_argument(
        "--backup",
        action="store_true",
        help="Before overwriting, copy original files to '<name>.bak'.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=None,
        help="Optional text report path. Defaults to '<dataset>_fix_traj_lengths_report.txt'.",
    )
    return parser.parse_args()


def is_episode_dir(path: Path) -> bool:
    if not path.is_dir() or len(path.name) != EPISODE_NAME_LEN:
        return False
    try:
        datetime.strptime(path.name, EPISODE_TIME_FORMAT)
    except ValueError:
        return False
    return True


def resolve_episode_dirs(path: Path) -> List[Path]:
    path = path.expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Path does not exist: {path}")
    if is_episode_dir(path):
        return [path]

    episodes = sorted(
        (child for child in path.iterdir() if is_episode_dir(child)),
        key=lambda episode: datetime.strptime(episode.name, EPISODE_TIME_FORMAT),
    )
    if not episodes:
        raise FileNotFoundError(f"No timestamped episode folders found under: {path}")
    return episodes


def default_report_path(input_path: Path) -> Path:
    resolved = input_path.expanduser().resolve()
    if is_episode_dir(resolved):
        name = f"{resolved.parent.name}_fix_traj_lengths_report.txt"
    else:
        name = f"{resolved.name}_fix_traj_lengths_report.txt"
    return Path.cwd() / name


def load_first_dim(path: Path) -> Tuple[object, int]:
    data = torch.load(path, map_location="cpu")
    shape = getattr(data, "shape", None)
    if shape is not None:
        if len(shape) == 0:
            raise ValueError(f"{path} has no first dimension")
        return data, int(shape[0])
    return data, len(data)


def resize_sequence(data: object, target_len: int) -> Tuple[object, int, str]:
    shape = getattr(data, "shape", None)
    if shape is not None:
        old_len = int(shape[0])
        if old_len == target_len:
            return data, old_len, "unchanged"
        if old_len > target_len:
            return data[:target_len], old_len, "truncated"
        if old_len == 0:
            raise ValueError("cannot pad an empty tensor")
        pad_shape = (target_len - old_len,) + tuple(shape[1:])
        padding = data[-1:].expand(pad_shape).clone()
        return torch.cat([data, padding], dim=0), old_len, "padded"

    old_len = len(data)
    if old_len == target_len:
        return data, old_len, "unchanged"
    if old_len > target_len:
        return data[:target_len], old_len, "truncated"
    if old_len == 0:
        raise ValueError("cannot pad an empty sequence")
    return data + [data[-1]] * (target_len - old_len), old_len, "padded"


def maybe_backup(path: Path) -> None:
    backup_path = path.with_name(f"{path.name}.bak")
    if backup_path.exists():
        return
    shutil.copy2(path, backup_path)


def fix_episode(
    episode_dir: Path,
    robot_dir_name: str,
    traj_dir_name: str,
    target_file: str,
    apply_changes: bool,
    backup: bool,
) -> FixResult:
    lines = [f"Episode: {episode_dir}"]
    changed = False
    issues: List[str] = []

    target_path = episode_dir / robot_dir_name / target_file
    if not target_path.is_file():
        issues.append(f"missing target file: {target_path}")
    else:
        try:
            _, target_len = load_first_dim(target_path)
            lines.append(f"  target length from {robot_dir_name}/{target_file}: {target_len}")
        except Exception as exc:
            issues.append(f"failed to read target length from {target_path}: {exc}")
            target_len = -1

    traj_dir = episode_dir / traj_dir_name
    if not traj_dir.is_dir():
        issues.append(f"missing trajectory directory: {traj_dir}")

    if issues:
        lines.extend(f"  [FAIL] {issue}" for issue in issues)
        return FixResult(episode_dir=episode_dir, ok=False, changed=False, lines=lines)

    for filename in TRAJ_FILES:
        traj_path = traj_dir / filename
        if not traj_path.is_file():
            issues.append(f"missing trajectory file: {traj_path}")
            continue

        try:
            data, _ = load_first_dim(traj_path)
            fixed_data, old_len, action = resize_sequence(data, target_len)
        except Exception as exc:
            issues.append(f"failed to resize {traj_path}: {exc}")
            continue

        new_len = target_len
        lines.append(f"  {traj_dir_name}/{filename}: {old_len} -> {new_len} ({action})")
        if action != "unchanged":
            changed = True
            if apply_changes:
                if backup:
                    maybe_backup(traj_path)
                torch.save(fixed_data, traj_path)

    if issues:
        lines.extend(f"  [FAIL] {issue}" for issue in issues)
        return FixResult(episode_dir=episode_dir, ok=False, changed=changed, lines=lines)

    if changed and not apply_changes:
        lines.append("  result: DRY RUN, changes needed")
    elif changed:
        lines.append("  result: FIXED")
    else:
        lines.append("  result: OK, already matched")
    return FixResult(episode_dir=episode_dir, ok=True, changed=changed, lines=lines)


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
        else default_report_path(args.path)
    )

    results = [
        fix_episode(
            episode_dir=episode_dir,
            robot_dir_name=args.robot_dir,
            traj_dir_name=args.traj_dir,
            target_file=args.target_file,
            apply_changes=args.apply,
            backup=args.backup,
        )
        for episode_dir in episode_dirs
    ]

    lines: List[str] = []
    for result in results:
        lines.extend(result.lines)
        lines.append("")

    failed = [result for result in results if not result.ok]
    changed = [result for result in results if result.changed]
    mode = "APPLY" if args.apply else "DRY RUN"
    lines.append(
        f"Mode: {mode}. Checked {len(results)} episode(s). "
        f"Changed/needs change: {len(changed)}. Failed: {len(failed)}."
    )

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"Detailed report saved to: {report_path}")
    print(f"Mode: {mode}")
    print(f"Checked episodes: {len(results)}")
    print(f"Changed/needs change: {len(changed)}")
    print(f"Failed episodes: {len(failed)}")
    for result in failed:
        print(f"  {result.episode_dir}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
