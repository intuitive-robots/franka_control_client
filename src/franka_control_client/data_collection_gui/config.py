"""Settings of the data collection GUI (one JSON file, GUI changes are written back)."""

from __future__ import annotations

import dataclasses
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# gello        GELLO leader arm -> Panda (joint teleop), data_collection_droid_irl.py
# vr           Meta Quest 3 controller -> Panda (Cartesian teleop), data_collection_droid_vr_irl.py
# kinesthetic  hand guiding (HumanControl), demo_collection_droid_irl.py
# teach_replay kinesthetic demo, then the robot replays it, both recorded, teach_and_replay_droid_irl.py
# keyboard     browser keys -> Cartesian impedance (arrows: xy, q/e: z), for planar pushing (push-T)
MODES = ("gello", "vr", "kinesthetic", "teach_replay", "keyboard")
ARM_GAINS = ("stiff", "compliant")
STATE_SPACES = ("auto", "joint", "cartesian")
ACTION_SOURCES = ("auto", "leader", "next_state")

JOINT_LOWER = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
JOINT_UPPER = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
DEFAULT_HOME_Q = (0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0)

# Only changeable while disconnected: they decide which devices get built.
CONNECTION_FIELDS = {
    "sim", "node_name", "node_ip", "group_name", "group_port", "robot_name",
    "gello_name", "mq3_name", "mq3_ip", "cameras", "mode", "gello_gains",
}


@dataclass
class CameraConfig:
    name: str  # pyzlc camera topic, e.g. "zed_right"
    hz: float = 25.0  # recording rate; 0 = every robot sample

    def __post_init__(self) -> None:
        self.hz = float(self.hz)


@dataclass
class CollectionConfig:
    # ---- hardware / network --------------------------------------------------
    sim: bool = False
    node_name: str = "data_collection_gui"
    node_ip: str = "192.168.1.1"
    group_name: str = "DroidGroup"
    group_port: int = 7730
    robot_name: str = "FrankaPanda"
    gello_name: str = "gello"
    mq3_name: str = "IRL-MQ3-2"
    mq3_ip: str = "192.168.0.117"
    cameras: List[CameraConfig] = field(default_factory=list)

    # ---- collection ----------------------------------------------------------
    mode: str = "gello"
    gello_gains: str = "stiff"  # stiff: HybridJointImpedance, compliant: ...Compliant
    data_dir: str = "/home/irl-admin/new_data_collection"
    task: str = "my_task"  # folder name: <data_dir>/<task>/<timestamp>/
    task_instruction: str = ""  # language instruction for LeRobot; empty -> task with spaces
    record_hz: float = 50.0  # robot state sampling rate of the IRL recording
    max_episode_s: float = 0.0  # auto-save after this long, 0 = off

    # ---- what to save --------------------------------------------------------
    save_irl: bool = True  # raw IRL format (timestamps.pt, <device>/*.pt, sensors/<cam>/*.png)
    save_lerobot: bool = False  # append every saved episode to a LeRobot dataset
    lerobot_root: str = ""  # empty -> <data_dir>/lerobot_data/<task>
    lerobot_fps: float = 15.0
    lerobot_state_space: str = "auto"  # auto: cartesian for vr, else joint
    lerobot_action_source: str = "auto"  # auto: leader if it matches the state space, else next state
    lerobot_image_width: int = 256  # 0 = native resolution (set both or neither)
    lerobot_image_height: int = 256
    lerobot_use_videos: bool = True
    lerobot_force_torque: bool = True
    robot_type: str = "franka_panda"

    # ---- teach & replay ------------------------------------------------------
    replay_gains: str = "stiff"
    replay_hz: float = 200.0
    replay_feedforward: bool = True
    replay_settle_s: float = 1.0

    # ---- keyboard teleop (live-updated while connected) -----------------------
    kb_speed: float = 0.05  # m/s, arrow keys (xy)
    kb_z_speed: float = 0.03  # m/s, q / e
    kb_slow_factor: float = 0.25  # shift held
    kb_yaw_deg: float = 0.0  # rotates the arrow keys about z (operator position), 0: up = base +x
    kb_control_hz: float = 100.0
    kb_max_lead: float = 0.02  # m the target may run ahead of the measured EE (per axis)
    kb_workspace_min: List[float] = field(default_factory=lambda: [0.25, -0.45, 0.0])
    kb_workspace_max: List[float] = field(default_factory=lambda: [0.85, 0.45, 0.7])

    home_q: List[float] = field(default_factory=lambda: list(DEFAULT_HOME_Q))

    # --------------------------------------------------------------------------
    def validate(self) -> None:
        if self.mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}")
        if self.gello_gains not in ARM_GAINS or self.replay_gains not in ARM_GAINS:
            raise ValueError(f"gains must be one of {ARM_GAINS}")
        if self.lerobot_state_space not in STATE_SPACES:
            raise ValueError(f"lerobot_state_space must be one of {STATE_SPACES}")
        if self.lerobot_action_source not in ACTION_SOURCES:
            raise ValueError(f"lerobot_action_source must be one of {ACTION_SOURCES}")
        if not 1.0 <= float(self.record_hz) <= 1000.0:
            raise ValueError("record_hz must be within [1, 1000]")
        if not 1.0 <= float(self.lerobot_fps) <= 200.0:
            raise ValueError("lerobot_fps must be within [1, 200]")
        if float(self.lerobot_fps) > float(self.record_hz) + 1e-9:
            raise ValueError("lerobot_fps cannot be higher than record_hz (frames would only repeat)")
        if (int(self.lerobot_image_width) > 0) != (int(self.lerobot_image_height) > 0):
            raise ValueError("set both LeRobot image width and height, or neither")
        if not 1.0 <= float(self.replay_hz) <= 1000.0:
            raise ValueError("replay_hz must be within [1, 1000]")
        if float(self.max_episode_s) < 0 or float(self.replay_settle_s) < 0:
            raise ValueError("durations must be >= 0")
        if not self.task.strip() or any(c in self.task for c in '/\\:*?"<>|'):
            raise ValueError("task must be a valid, non-empty folder name")
        if not str(self.data_dir).strip():
            raise ValueError("data_dir must not be empty")
        names = [c.name for c in self.cameras]
        if len(names) != len(set(names)) or any(not n.strip() for n in names):
            raise ValueError("camera names must be unique and non-empty")
        for cam in self.cameras:
            if not 0.0 <= cam.hz <= 200.0:
                raise ValueError(f"camera {cam.name}: hz must be within [0, 200]")
        if not (0 < float(self.kb_speed) <= 0.5 and 0 < float(self.kb_z_speed) <= 0.5):
            raise ValueError("keyboard speeds must be within (0, 0.5] m/s")
        if not 0 < float(self.kb_slow_factor) <= 1:
            raise ValueError("kb_slow_factor must be within (0, 1]")
        if not 10 <= float(self.kb_control_hz) <= 1000:
            raise ValueError("kb_control_hz must be within [10, 1000]")
        if not 0 <= float(self.kb_max_lead) <= 0.2:
            raise ValueError("kb_max_lead must be within [0, 0.2] m (0 = off)")
        if len(self.kb_workspace_min) != 3 or len(self.kb_workspace_max) != 3:
            raise ValueError("keyboard workspace needs 3 values (x, y, z) for min and max")
        if any(not lo < hi for lo, hi in zip(self.kb_workspace_min, self.kb_workspace_max)):
            raise ValueError("keyboard workspace: every min must be below its max")
        if len(self.home_q) != 7:
            raise ValueError(f"home_q needs 7 values, got {len(self.home_q)}")
        for i, (q, lo, hi) in enumerate(zip(self.home_q, JOINT_LOWER, JOINT_UPPER)):
            if not (math.isfinite(q) and lo + 0.02 <= q <= hi - 0.02):
                raise ValueError(f"home_q[{i}] = {q} is outside the joint limits")

    # ---- derived -------------------------------------------------------------
    @property
    def task_dir(self) -> Path:
        return Path(self.data_dir).expanduser() / self.task

    @property
    def lerobot_dir(self) -> Path:
        if str(self.lerobot_root).strip():
            return Path(self.lerobot_root).expanduser()
        return Path(self.data_dir).expanduser() / "lerobot_data" / self.task

    @property
    def lerobot_image_size(self) -> Optional[Tuple[int, int]]:
        if int(self.lerobot_image_width) > 0 and int(self.lerobot_image_height) > 0:
            return (int(self.lerobot_image_width), int(self.lerobot_image_height))
        return None

    @property
    def instruction(self) -> str:
        return self.task_instruction.strip() or self.task.replace("_", " ")

    # ---- (de)serialisation ---------------------------------------------------
    def to_dict(self) -> Dict[str, Any]:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CollectionConfig":
        known = {f.name for f in dataclasses.fields(cls)}
        kwargs: Dict[str, Any] = {}
        for key, value in data.items():
            if key not in known:
                continue
            if key == "cameras":
                value = [c if isinstance(c, CameraConfig) else CameraConfig(**c) for c in value]
            kwargs[key] = value
        cfg = cls(**kwargs)
        cfg._coerce_types()
        cfg.validate()
        return cfg

    def merged(self, update: Dict[str, Any]) -> "CollectionConfig":
        data = self.to_dict()
        data.update(update)
        return CollectionConfig.from_dict(data)

    def _coerce_types(self) -> None:
        # HTML forms may send strings
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if f.type == "float" and not isinstance(value, float):
                setattr(self, f.name, float(value))
            elif f.type == "int" and not isinstance(value, int):
                setattr(self, f.name, int(float(value)))
            elif f.type == "bool" and not isinstance(value, bool):
                setattr(self, f.name, str(value).lower() in ("1", "true", "yes", "on"))
            elif f.type == "str" and not isinstance(value, str):
                setattr(self, f.name, str(value))
        home = self.home_q
        if isinstance(home, str):
            home = [v for v in home.replace(",", " ").split() if v]
        self.home_q = [float(v) for v in home]
        for name in ("kb_workspace_min", "kb_workspace_max"):
            value = getattr(self, name)
            if isinstance(value, str):
                value = [v for v in value.replace(",", " ").split() if v]
            setattr(self, name, [float(v) for v in value])

    @classmethod
    def load(cls, path: str | Path) -> "CollectionConfig":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)
