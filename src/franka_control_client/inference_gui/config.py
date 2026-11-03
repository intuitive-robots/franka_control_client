"""Session configuration for the inference GUI (robot PC side)."""

from __future__ import annotations

import dataclasses
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

CONTROL_PAIRS = ("joint", "cartesian")
GRIPPER_TYPES = ("robotiq", "panda")
INFERENCE_MODES = ("sync", "async")

# Fields that may only change while no robot/policy connection exists, because
# they decide which devices / sockets / control pair get built.
CONNECTION_FIELDS = {
    "sim",
    "node_name",
    "node_ip",
    "group_name",
    "group_port",
    "robot_name",
    "gripper_type",
    "cameras",
}


@dataclass
class CameraConfig:
    # pyzlc camera device / topic name (e.g. "zed_right").
    name: str
    # Observation key sent to the policy. Empty -> "observation.images.<name>".
    obs_key: str = ""
    # False: only shown on the dashboard / recorded, never sent to the policy.
    send_to_policy: bool = True

    @property
    def policy_key(self) -> str:
        return self.obs_key or f"observation.images.{self.name}"


@dataclass
class SessionConfig:
    # ---- hardware / network (robot PC) ------------------------------------
    sim: bool = False  # simulated arm/gripper/cameras, no pyzlc network needed
    node_name: str = "inference_gui"
    node_ip: str = "192.168.1.1"
    group_name: str = "DroidGroup"
    group_port: int = 7730
    robot_name: str = "FrankaPanda"
    gripper_type: str = "robotiq"
    cameras: List[CameraConfig] = field(default_factory=list)

    # ---- control pair ------------------------------------------------------
    control_pair: str = "joint"  # "joint" | "cartesian"
    control_hz: float = 15.0

    # ---- policy server -----------------------------------------------------
    policy_host: str = "127.0.0.1"
    policy_port: int = 8767
    # Seconds to wait for one server reply; 0 waits forever (first call warms up).
    request_timeout_s: float = 0.0
    # Rate of the rows in an action chunk. The server's health() overrides it.
    policy_fps: float = 15.0
    # Rows executed per chunk; 0 -> use the server's exec_steps (or the full chunk).
    exec_steps: int = 0
    # sync : observe -> infer -> play exec_steps rows -> observe ...
    # async: request the next chunk while the current one is still playing and
    #        drop the rows that are already stale when it arrives.
    inference_mode: str = "sync"
    task: str = ""

    # ---- safety ------------------------------------------------------------
    # Refuse (and stop on) a chunk whose first row jumps further than this from
    # the measured state: rad on any joint / metres of EE translation.
    max_first_step_jump_joint: float = 0.35
    max_first_step_jump_cartesian: float = 0.10
    clip_joint_limits: bool = True
    max_episode_s: float = 120.0  # 0 disables the auto-stop

    # ---- evaluation / recording -------------------------------------------
    batch_name: str = "eval"
    batch_description: str = ""
    save_root: str = "eval_rollouts"
    state_record_hz: float = 50.0
    record_images: bool = True  # the frames sent to the policy, as mp4 per camera

    # ---- LeRobot dataset (one episode per rollout, in <rollout>/lerobot/) ---
    save_lerobot: bool = False
    dataset_fps: float = 0.0  # 0 -> the policy's chunk rate
    dataset_image_width: int = 0  # 0 -> camera resolution
    dataset_image_height: int = 0
    dataset_force_torque: bool = True  # observation.force_torque (14)
    dataset_use_videos: bool = True  # False stores PNG frames
    robot_type: str = "franka_panda"

    # ------------------------------------------------------------------------
    def validate(self) -> None:
        if self.control_pair not in CONTROL_PAIRS:
            raise ValueError(f"control_pair must be one of {CONTROL_PAIRS}")
        if self.gripper_type not in GRIPPER_TYPES:
            raise ValueError(f"gripper_type must be one of {GRIPPER_TYPES}")
        if self.inference_mode not in INFERENCE_MODES:
            raise ValueError(f"inference_mode must be one of {INFERENCE_MODES}")
        if not 1.0 <= float(self.control_hz) <= 1000.0:
            raise ValueError("control_hz must be within [1, 1000]")
        if not 0.5 <= float(self.policy_fps) <= 200.0:
            raise ValueError("policy_fps must be within [0.5, 200]")
        if not 1.0 <= float(self.state_record_hz) <= 500.0:
            raise ValueError("state_record_hz must be within [1, 500]")
        if int(self.exec_steps) < 0:
            raise ValueError("exec_steps must be >= 0")
        if not 0.0 <= float(self.dataset_fps) <= 100.0:
            raise ValueError("dataset_fps must be within [0, 100] (0 = policy rate)")
        if (int(self.dataset_image_width) > 0) != (int(self.dataset_image_height) > 0):
            raise ValueError("set both dataset image width and height, or neither")
        if not self.batch_name.strip():
            raise ValueError("batch_name must not be empty")
        if any(c in self.batch_name for c in '/\\:*?"<>|'):
            raise ValueError("batch_name must be a valid folder name")
        names = [c.name for c in self.cameras]
        if len(names) != len(set(names)):
            raise ValueError("camera names must be unique")

    @property
    def dataset_image_size(self) -> Optional[tuple]:
        if int(self.dataset_image_width) > 0 and int(self.dataset_image_height) > 0:
            return (int(self.dataset_image_width), int(self.dataset_image_height))
        return None

    @property
    def timeout(self) -> Optional[float]:
        return None if float(self.request_timeout_s) <= 0 else float(self.request_timeout_s)

    def to_dict(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SessionConfig":
        known = {f.name: f for f in dataclasses.fields(cls)}
        kwargs: Dict[str, Any] = {}
        for key, value in data.items():
            if key not in known:
                continue  # ignore unknown keys (older/newer config files)
            if key == "cameras":
                value = [c if isinstance(c, CameraConfig) else CameraConfig(**c) for c in value]
            kwargs[key] = value
        cfg = cls(**kwargs)
        cfg._coerce_types()
        cfg.validate()
        return cfg

    def merged(self, update: Dict[str, Any]) -> "SessionConfig":
        data = self.to_dict()
        data.update(update)
        return SessionConfig.from_dict(data)

    def _coerce_types(self) -> None:
        # Values coming from HTML forms may be strings.
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if f.type in ("float",) and not isinstance(value, float):
                setattr(self, f.name, float(value))
            elif f.type in ("int",) and not isinstance(value, int):
                setattr(self, f.name, int(float(value)))
            elif f.type in ("bool",) and not isinstance(value, bool):
                setattr(self, f.name, str(value).lower() in ("1", "true", "yes", "on"))

    @classmethod
    def load(cls, path: str | Path) -> "SessionConfig":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)
