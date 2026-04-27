import argparse
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import torch


EPISODE_TIME_FORMAT = "%Y_%m_%d-%H_%M_%S"
EPISODE_NAME_LEN = len("YYYY_MM_DD-HH_MM_SS")
IGNORED_DIRS = {"sensors", "__pycache__"}
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


@dataclass
class LengthItem:
    name: str
    length: Optional[int]
    ok: bool
    message: str = ""


@dataclass
class EpisodeReport:
    episode_dir: Path
    ok: bool
    lines: List[str]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check that every saved tensor stream in each episode has the expected "
            "length. By default this checks timestamps, robot directories, and Traj."
        )
    )
    parser.add_argument(
        "path",
        type=Path,
        help="Dataset/task root containing timestamped episodes, or one episode folder.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=None,
        help="Optional text report path. Defaults to '<dataset>_length_check_report.txt'.",
    )
    parser.add_argument(
        "--check-cameras",
        action="store_true",
        help=(
            "Also require camera image folders under sensors/ to have the same length "
            "as the robot state. Use only for same-Hz camera datasets."
        ),
    )
    parser.add_argument(
        "--allow-missing-timestamps",
        action="store_true",
        help="Do not fail an episode when timestamps.pt is missing.",
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
        name = f"{resolved.parent.name}_length_check_report.txt"
    else:
        name = f"{resolved.name}_length_check_report.txt"
    return Path.cwd() / name


def first_dim_length(path: Path) -> LengthItem:
    try:
        data = torch.load(path, map_location="cpu")
    except Exception as exc:
        return LengthItem(path.name, None, False, f"failed to load: {exc}")

    shape = getattr(data, "shape", None)
    if shape is not None:
        try:
            if len(shape) == 0:
                return LengthItem(path.name, None, False, "scalar tensor has no time dimension")
            return LengthItem(path.name, int(shape[0]), True)
        except Exception as exc:
            return LengthItem(path.name, None, False, f"failed to inspect shape: {exc}")

    try:
        return LengthItem(path.name, len(data), True)
    except TypeError:
        return LengthItem(path.name, None, False, "object has no length")


def tensor_files(directory: Path) -> List[Path]:
    if not directory.is_dir():
        return []
    files: List[Path] = []
    for child in sorted(directory.iterdir()):
        if not child.is_file():
            continue
        if child.suffix.lower() in IMAGE_SUFFIXES:
            continue
        files.append(child)
    return files


def collect_dir_lengths(directory: Path) -> List[LengthItem]:
    return [first_dim_length(path) for path in tensor_files(directory)]


def length_counts(items: Iterable[LengthItem]) -> Dict[int, int]:
    counts: Dict[int, int] = {}
    for item in items:
        if item.ok and item.length is not None:
            counts[item.length] = counts.get(item.length, 0) + 1
    return counts


def expected_robot_length(streams: Dict[str, List[LengthItem]]) -> Optional[int]:
    candidates: List[LengthItem] = []
    for name, items in streams.items():
        if name == "Traj":
            continue
        candidates.extend(items)

    counts = length_counts(candidates)
    if not counts:
        return None
    return max(counts.items(), key=lambda pair: pair[1])[0]


def format_items(label: str, items: Sequence[LengthItem]) -> List[str]:
    if not items:
        return [f"  {label}: no tensor-like files found"]

    lines = [f"  {label}:"]
    for item in items:
        if item.ok:
            lines.append(f"    {item.name}: {item.length}")
        else:
            lines.append(f"    {item.name}: FAIL ({item.message})")
    return lines


def sensor_lengths(sensors_dir: Path) -> List[LengthItem]:
    if not sensors_dir.is_dir():
        return []

    items: List[LengthItem] = []
    for sensor_dir in sorted(child for child in sensors_dir.iterdir() if child.is_dir()):
        count = sum(
            1
            for child in sensor_dir.iterdir()
            if child.is_file() and child.suffix.lower() in IMAGE_SUFFIXES
        )
        items.append(LengthItem(sensor_dir.name, count, True))
    return items


def check_episode(
    episode_dir: Path,
    check_cameras: bool,
    allow_missing_timestamps: bool,
) -> EpisodeReport:
    lines = [f"Episode: {episode_dir}"]
    issues: List[str] = []
    streams: Dict[str, List[LengthItem]] = {}

    timestamps_path = episode_dir / "timestamps.pt"
    if timestamps_path.is_file():
        streams["timestamps.pt"] = [first_dim_length(timestamps_path)]
    elif not allow_missing_timestamps:
        issues.append("missing timestamps.pt")

    for child in sorted(episode_dir.iterdir()):
        if not child.is_dir() or child.name in IGNORED_DIRS:
            continue
        items = collect_dir_lengths(child)
        if items:
            streams[child.name] = items

    for label, items in streams.items():
        lines.extend(format_items(label, items))
        for item in items:
            if not item.ok:
                issues.append(f"{label}/{item.name}: {item.message}")

    robot_len = expected_robot_length(streams)
    if robot_len is None:
        issues.append("could not determine robot-state length")
    else:
        lines.append(f"  expected robot-state length: {robot_len}")

        for label, items in streams.items():
            for item in items:
                if not item.ok or item.length is None:
                    continue
                if item.length != robot_len:
                    issues.append(
                        f"{label}/{item.name} length {item.length} != {robot_len}"
                    )

    sensors = sensor_lengths(episode_dir / "sensors")
    if sensors:
        lines.append("  sensors:")
        for item in sensors:
            lines.append(f"    {item.name}: {item.length} image(s)")
            if check_cameras and robot_len is not None and item.length != robot_len:
                issues.append(
                    f"sensors/{item.name} length {item.length} != {robot_len}"
                )

    if issues:
        lines.append("  result: FAIL")
        for issue in issues:
            lines.append(f"    [FAIL] {issue}")
    else:
        lines.append("  result: OK")

    return EpisodeReport(episode_dir=episode_dir, ok=not issues, lines=lines)


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

    reports = [
        check_episode(
            episode_dir,
            check_cameras=args.check_cameras,
            allow_missing_timestamps=args.allow_missing_timestamps,
        )
        for episode_dir in episode_dirs
    ]

    lines: List[str] = []
    for report in reports:
        lines.extend(report.lines)
        lines.append("")

    failed = [report for report in reports if not report.ok]
    lines.append(
        f"Checked {len(reports)} episode(s). Failed episodes: {len(failed)}."
    )

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"Detailed report saved to: {report_path}")
    print(f"Checked episodes: {len(reports)}")
    print(f"Failed episodes: {len(failed)}")
    for report in failed:
        print(f"  {report.episode_dir}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
