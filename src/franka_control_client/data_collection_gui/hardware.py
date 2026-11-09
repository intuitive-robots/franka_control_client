"""Devices + control pair of each collection mode (real, or simulated with ``"sim": true``).

The existing control pairs are used unchanged:

    gello         GelloPandControlPair   (GELLO -> joint impedance, stiff or compliant gains)
    vr            MQ3PandaControlPair    (Meta Quest 3 -> Cartesian impedance)
    keyboard      KeyboardPandaControlPair (browser keys -> Cartesian impedance, gripper untouched)
    kinesthetic   HumanPandaControlPair  (HumanControl: gravity-compensated hand guiding)
    teach_replay  HumanPandaControlPair  for the demo, teach_replay_collection.DemoReplayer for the replay

The simulated devices reuse ``inference_gui.hardware`` (arm, Robotiq, cameras) plus a GELLO
and a VR controller that move by themselves, and an arm that drifts while "hand guided".
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from ..control_pair.control_pair import ControlPair
from ..franka_robot.panda_arm import ControlMode, RemotePandaArm
from ..franka_robot.panda_robotiq import PandaRobotiq
from ..gello.gello import RemoteGello
from ..inference_gui.hardware import (
    HOME_Q,
    Q_LOWER,
    Q_UPPER,
    SimCamera,
    SimPandaArm,
    SimRobotiqGripper,
    panda_fk,
    rot_to_quat,
)
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper
from .config import CollectionConfig

LOG = logging.getLogger(__name__)

GAINS = {"stiff": ControlMode.HybridJointImpedance, "compliant": ControlMode.HybridJointImpedanceCompliant}
LEADER_DIRS = {"gello": "Gello", "vr": "MQ3", "keyboard": "Keyboard"}  # leader folder names of the IRL format
GRIPPER_SPEED = 0.7
GRIPPER_FORCE = 0.3


@dataclass
class Rig:
    mode: str
    arm: RemotePandaArm
    gripper: RemoteRobotiqGripper
    robot: PandaRobotiq
    pair: ControlPair
    cameras: Dict[str, Any] = field(default_factory=dict)
    leader: Any = None
    sim: bool = False
    _loops: List[Any] = field(default_factory=list)

    @property
    def leader_dir(self) -> Optional[str]:
        return LEADER_DIRS.get(self.mode)

    def send_gripper(self, position: float) -> None:
        self.gripper.send_grasp_command(position=float(position), speed=GRIPPER_SPEED, force=GRIPPER_FORCE, blocking=False)

    def set_idle(self) -> None:
        try:
            self.arm.set_franka_arm_control_mode(ControlMode.IDLE)
        except Exception:
            LOG.exception("Failed to set the arm to Idle")

    def close(self) -> None:
        if self.pair is not None and self.pair.is_running:
            self.pair.stop_control_pair()
        for loop in self._loops:
            loop.stop()


def keyboard_settings(cfg: CollectionConfig) -> Dict[str, Any]:
    return dict(
        speed=cfg.kb_speed, z_speed=cfg.kb_z_speed, slow_factor=cfg.kb_slow_factor, yaw_deg=cfg.kb_yaw_deg,
        bounds=(list(cfg.kb_workspace_min), list(cfg.kb_workspace_max)), max_lead=cfg.kb_max_lead,
    )


def apply_keyboard_settings(leader: Any, cfg: CollectionConfig) -> None:
    """Live update of a connected KeyboardController (speeds, yaw, workspace, lead)."""
    s = keyboard_settings(cfg)
    leader.speed, leader.z_speed = float(s["speed"]), float(s["z_speed"])
    leader.slow_factor, leader.yaw_deg, leader.max_lead = float(s["slow_factor"]), float(s["yaw_deg"]), float(s["max_lead"])
    leader.lo, leader.hi = np.asarray(s["bounds"][0], dtype=float), np.asarray(s["bounds"][1], dtype=float)


def build_keyboard(cfg: CollectionConfig, robot: PandaRobotiq):
    from ..control_pair.keyboard_panda_control_pair import KeyboardPandaControlPair
    from ..keyboard.keyboard_controller import KeyboardController

    leader = KeyboardController(**keyboard_settings(cfg))
    return leader, KeyboardPandaControlPair(leader, robot, cfg.kb_control_hz, cfg.home_q)


# ======================================================================= real
def build_real_rig(cfg: CollectionConfig, log: Callable[[str], None]) -> Rig:
    log(f"Connecting to arm + Robotiq '{cfg.robot_name}'...")
    arm = RemotePandaArm(cfg.robot_name)
    arm.connect()
    gripper = RemoteRobotiqGripper(cfg.robot_name)
    gripper.connect()
    robot = PandaRobotiq("PandaRobotiq", arm, gripper)
    leader: Any = None
    if cfg.mode == "gello":
        from ..control_pair.gello_panda_control_pair import GelloPandControlPair

        log(f"Connecting to GELLO '{cfg.gello_name}'...")
        leader = RemoteGello(cfg.gello_name)
        leader.connect()
        pair: ControlPair = GelloPandControlPair(leader, robot, control_mode=GAINS[cfg.gello_gains])
    elif cfg.mode == "vr":
        from ..control_pair.mq3_panda_control_pair import MQ3PandaControlPair
        from ..vr.meta_quest3 import MQ3Controller

        log(f"Waiting for Meta Quest 3 '{cfg.mq3_name}' ({cfg.mq3_ip})...")
        leader = MQ3Controller(cfg.mq3_name, cfg.mq3_ip, arm)
        leader.mq3.wait_for_connection()
        pair = MQ3PandaControlPair(leader, robot)
    elif cfg.mode == "keyboard":
        leader, pair = build_keyboard(cfg, robot)
    else:
        from ..control_pair.human_control import HumanPandaControlPair

        pair = HumanPandaControlPair(robot)
    cameras: Dict[str, Any] = {}
    if cfg.cameras:
        from ..camera.camera import CameraDevice

        for cam in cfg.cameras:
            log(f"Waiting for camera '{cam.name}'...")
            cameras[cam.name] = CameraDevice(cam.name, preview=False)
    return Rig(cfg.mode, arm, gripper, robot, pair, cameras, leader)


# ================================================================= simulation
class SimGuidedPandaArm(SimPandaArm):
    """SimPandaArm that wanders slowly in HumanControl, as if somebody guided it."""

    def _integrate(self, dt: float) -> None:
        if self._mode != ControlMode.HumanControl:
            return super()._integrate(dt)
        with self._lock:
            self._phase = getattr(self, "_phase", 0.0) + dt
            dq = 0.12 * np.sin(0.6 * self._phase + np.arange(7)) * np.array([1, 1, 1, 1, 0.5, 1, 1.5])
            self._q = np.clip(self._q + dq * dt, Q_LOWER + 0.05, Q_UPPER - 0.05)
            self._dq = dq
            self._q_cmd = self._q.copy()


class SimGello(RemoteGello):
    """A GELLO whose joints swing around the home pose (no network)."""

    def __init__(self, name: str = "gello") -> None:  # no super().__init__
        self._name = name
        self._t0 = time.time()

    def connect(self) -> None:
        return None

    @property
    def current_state(self):
        t = time.time() - self._t0
        q = HOME_Q + 0.25 * np.sin(0.4 * t + np.arange(7) * 0.7) * np.array([1, 0.6, 1, 0.5, 1, 0.5, 1])
        return {
            "gello_arm_state": {"joint_state": q.tolist(), "joint_velocity": [0.0] * 7, "seq": int(t * 200)},
            "gello_gripper_state": {"gripper": [float(0.5 + 0.5 * np.sin(0.3 * t))]},
        }


class SimMQ3:
    """Stand-in for MQ3Controller: the target circles around the EE pose at reset()."""

    def __init__(self, arm: RemotePandaArm) -> None:
        self.panda_arm = arm
        self._t0 = time.time()
        self.reset()

    def reset(self) -> None:
        T = panda_fk(HOME_Q)
        self._base_pos, self._base_quat = T[:3, 3].copy(), rot_to_quat(T[:3, :3])
        self._t0 = time.time()

    def stop_control(self) -> None:
        pass

    @property
    def current_control_signal(self):
        t = time.time() - self._t0
        pos = self._base_pos + 0.06 * np.array([np.sin(0.5 * t), np.cos(0.5 * t) - 1, 0.5 * np.sin(0.25 * t)])
        return {
            "pos": pos.tolist(),
            "rot": self._base_quat.tolist(),
            "pos_vel": [0.0] * 3,
            "rot_vel": [0.0] * 3,
            "gripper_width": float(0.5 + 0.5 * np.sin(0.3 * t)),
        }


class _SimRobot(PandaRobotiq):
    def __init__(self, arm: RemotePandaArm, gripper: RemoteRobotiqGripper) -> None:  # no super().__init__
        self._name = "SimPandaRobotiq"
        self.panda_arm = arm
        self.robotiq_gripper = gripper


def build_sim_rig(cfg: CollectionConfig, log: Callable[[str], None]) -> Rig:
    arm = SimGuidedPandaArm(cfg.robot_name)
    gripper = SimRobotiqGripper(cfg.robot_name)
    robot = _SimRobot(arm, gripper)
    leader: Any = None
    if cfg.mode == "gello":
        from ..control_pair.gello_panda_control_pair import GelloPandControlPair

        leader = SimGello(cfg.gello_name)
        pair: ControlPair = GelloPandControlPair(leader, robot, control_mode=GAINS[cfg.gello_gains])
    elif cfg.mode == "vr":
        from ..control_pair.mq3_panda_control_pair import MQ3PandaControlPair

        leader = SimMQ3(arm)
        pair = MQ3PandaControlPair(leader, robot)
    elif cfg.mode == "keyboard":
        leader, pair = build_keyboard(cfg, robot)
    else:
        from ..control_pair.human_control import HumanPandaControlPair

        pair = HumanPandaControlPair(robot)
    cameras = {cam.name: SimCamera(cam.name) for cam in cfg.cameras}
    log("Simulated hardware ready.")
    return Rig(cfg.mode, arm, gripper, robot, pair, cameras, leader, sim=True, _loops=[arm.loop, gripper.loop])


def rate_limit_control_step(pair: ControlPair, hz: float) -> None:
    """MQ3PandaControlPair.control_step has no sleep (its CONTROL_HZ is unused): the control
    thread spins, floods the network with commands and starves the recording thread of the
    GIL. Pace it on this instance only (the terminal scripts keep the original behaviour)."""
    step = pair.control_step
    period = 1.0 / hz
    state = {"next": time.perf_counter()}

    def paced_step() -> None:
        step()
        state["next"] = max(state["next"] + period, time.perf_counter() - period)
        delay = state["next"] - time.perf_counter()
        if delay > 0:
            time.sleep(delay)

    pair.control_step = paced_step  # type: ignore[method-assign]


def build_rig(cfg: CollectionConfig, log: Callable[[str], None]) -> Rig:
    rig = build_sim_rig(cfg, log) if cfg.sim else build_real_rig(cfg, log)
    if cfg.mode == "vr":
        from ..control_pair import mq3_panda_control_pair

        rate_limit_control_step(rig.pair, mq3_panda_control_pair.CONTROL_HZ)
    return rig
