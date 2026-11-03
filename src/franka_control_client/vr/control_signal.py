"""Leader-side interface shared by the Quest teleop controllers (simpub-free)."""

from typing import List, Optional, Protocol, TypedDict


class MQ3ControlSignal(TypedDict):
    pos: List[float]
    rot: List[float]  # xyzw
    pos_vel: List[float]
    rot_vel: List[float]
    gripper_width: float  # Robotiq position: 0.0 open .. 1.0 closed


class EEControlSource(Protocol):
    """Anything that yields a target EE pose, e.g. MQ3Controller or WebXRController."""

    @property
    def current_control_signal(self) -> Optional[MQ3ControlSignal]: ...

    def reset(self) -> None: ...
