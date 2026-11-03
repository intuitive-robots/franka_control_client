"""Robot-PC devices: the real pyzlc devices, or a simulated stand-in.

The simulated devices subclass the real ones (without calling their
``__init__``), so the existing control pairs' ``isinstance`` checks and method
calls work unchanged. Use ``"sim": true`` to try the whole GUI pipeline on any
machine, together with the dummy policy server.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Union

import numpy as np

from ..camera.camera import CameraDevice
from ..franka_robot.panda_arm import ControlMode, RemotePandaArm
from ..franka_robot.panda_gripper import RemotePandaGripper
from ..robotiq_gripper.robotiq_gripper import RemoteRobotiqGripper
from .config import SessionConfig

LOG = logging.getLogger(__name__)

HOME_Q = np.array([0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0])
Q_LOWER = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_UPPER = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

# Panda modified-DH (Craig) parameters: a, d, alpha per joint, then flange + hand.
_DH = [
    (0.0, 0.333, 0.0),
    (0.0, 0.0, -np.pi / 2),
    (0.0, 0.316, np.pi / 2),
    (0.0825, 0.0, np.pi / 2),
    (-0.0825, 0.384, -np.pi / 2),
    (0.0, 0.0, np.pi / 2),
    (0.088, 0.0, np.pi / 2),
]
_FLANGE_D = 0.107
_HAND_D = 0.1034


def _mdh(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta)
    return np.array(
        [
            [ct, -st, 0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0, 0, 0, 1],
        ]
    )


def panda_fk(q: Sequence[float]) -> np.ndarray:
    """4x4 base->EE (hand TCP, rotated -45deg about z like the Franka EE frame)."""
    T = np.eye(4)
    for (a, d, alpha), qi in zip(_DH, q):
        T = T @ _mdh(a, d, alpha, qi)
    T = T @ _mdh(0.0, _FLANGE_D, 0.0, 0.0)
    T = T @ _mdh(0.0, _HAND_D, 0.0, -np.pi / 4)
    return T


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> quaternion (x, y, z, w)."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


GripperT = Union[RemoteRobotiqGripper, RemotePandaGripper]
CommandTap = Callable[[float, str, Sequence[float]], None]


@dataclass
class Hardware:
    arm: RemotePandaArm
    gripper: GripperT
    cameras: Dict[str, Any] = field(default_factory=dict)
    sim: bool = False
    _threads: List[Any] = field(default_factory=list)

    def install_command_tap(self, tap: CommandTap) -> None:
        """Report every command the controller publishes, without touching the
        control pair code: wraps the device methods on these instances."""
        arm, gripper = self.arm, self.gripper

        joint_send = arm.send_joint_position_command
        cart_send = arm.send_cartesian_pose_command

        def send_joint(joint_positions, joint_velocities=None):
            joint_send(joint_positions, joint_velocities)
            tap(time.time(), "joint", np.asarray(joint_positions, dtype=np.float64).reshape(-1))

        def send_cart(pos, rot, pos_vel=None, rot_vel=None):
            cart_send(pos, rot, pos_vel, rot_vel)
            pose = np.concatenate([np.asarray(pos, dtype=np.float64).reshape(-1), np.asarray(rot, dtype=np.float64).reshape(-1)])
            tap(time.time(), "cartesian", pose)

        arm.send_joint_position_command = send_joint  # type: ignore[method-assign]
        arm.send_cartesian_pose_command = send_cart  # type: ignore[method-assign]

        if isinstance(gripper, RemoteRobotiqGripper):
            grasp = gripper.send_grasp_command

            def send_grasp(position, speed=0.1, force=0.1, blocking=False):
                grasp(position=position, speed=speed, force=force, blocking=blocking)
                tap(time.time(), "gripper", [float(position)])

            gripper.send_grasp_command = send_grasp  # type: ignore[method-assign]
        else:
            grip = gripper.send_gripper_command

            def send_grip(width, speed=0.01):
                grip(width=width, speed=speed)
                tap(time.time(), "gripper", [float(width)])

            gripper.send_gripper_command = send_grip  # type: ignore[method-assign]

    def set_idle(self) -> None:
        try:
            self.arm.set_franka_arm_control_mode(ControlMode.IDLE)
        except Exception:
            LOG.exception("Failed to set the arm to Idle")

    def close(self) -> None:
        for t in self._threads:
            t.stop()


def gripper_closedness(gripper: GripperT) -> Optional[float]:
    """0 = open, 1 = closed (Robotiq position; Franka hand 1 - width/max_width)."""
    state = gripper.current_state
    if not isinstance(state, dict):
        return None
    if "position" in state:
        return float(np.clip(float(state["position"]), 0.0, 1.0))
    if "width" in state:
        max_width = float(state.get("max_width", 0.0) or 0.0)
        if max_width <= 0.0:
            return None
        return float(np.clip(1.0 - float(state["width"]) / max_width, 0.0, 1.0))
    return None


# ---------------------------------------------------------------- real robot
def build_real_hardware(cfg: SessionConfig, log: Callable[[str], None]) -> Hardware:
    log(f"Connecting to arm '{cfg.robot_name}'...")
    arm = RemotePandaArm(cfg.robot_name)
    arm.connect()
    if cfg.gripper_type == "robotiq":
        gripper: GripperT = RemoteRobotiqGripper(cfg.robot_name)
    else:
        gripper = RemotePandaGripper(cfg.robot_name)
    gripper.connect()
    cameras: Dict[str, Any] = {}
    for cam in cfg.cameras:
        log(f"Waiting for camera '{cam.name}'...")
        cameras[cam.name] = CameraDevice(cam.name, preview=False)
    log("Robot hardware connected.")
    return Hardware(arm=arm, gripper=gripper, cameras=cameras)


# ----------------------------------------------------------------- simulation
class _SimLoop:
    def __init__(self, hz: float, step: Callable[[float], None]) -> None:
        self._dt = 1.0 / hz
        self._step = step
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        last = time.perf_counter()
        while self._running:
            time.sleep(self._dt)
            now = time.perf_counter()
            self._step(now - last)
            last = now

    def stop(self) -> None:
        self._running = False


class SimPandaArm(RemotePandaArm):
    """First-order joint tracking. Cartesian commands move the reported EE
    pose directly (no IK), which is enough to exercise the pipeline."""

    TAU_S = 0.08

    def __init__(self, name: str = "SimPanda") -> None:  # noqa: D401 - no super().__init__
        self._name = name
        self._enable_publishers = True
        self._lock = threading.Lock()
        self._q = HOME_Q.copy()
        self._dq = np.zeros(7)
        self._q_cmd = HOME_Q.copy()
        self._ee_override: Optional[np.ndarray] = None
        self._ee_cmd: Optional[np.ndarray] = None
        self._mode = ControlMode.IDLE
        self._t0 = time.time()
        self.loop = _SimLoop(500.0, self._integrate)

    def connect(self) -> None:
        return None

    def _integrate(self, dt: float) -> None:
        with self._lock:
            if self._mode == ControlMode.IDLE:
                self._dq[:] = 0.0
                return
            alpha = 1.0 - np.exp(-dt / self.TAU_S)
            if self._ee_cmd is not None:
                if self._ee_override is None:
                    T = panda_fk(self._q)
                    self._ee_override = np.concatenate([T[:3, 3], rot_to_quat(T[:3, :3])])
                self._ee_override = self._ee_override + alpha * (self._ee_cmd - self._ee_override)
                self._ee_override[3:] /= np.linalg.norm(self._ee_override[3:])
                self._dq[:] = 0.0
                return
            new_q = self._q + alpha * (self._q_cmd - self._q)
            self._dq = (new_q - self._q) / max(dt, 1e-6)
            self._q = np.clip(new_q, Q_LOWER, Q_UPPER)

    @property
    def current_state(self):
        with self._lock:
            q, dq, q_d = self._q.copy(), self._dq.copy(), self._q_cmd.copy()
            ee_override = None if self._ee_override is None else self._ee_override.copy()
        T = panda_fk(q)
        if ee_override is not None:
            pos, quat = ee_override[:3], ee_override[3:]
        else:
            pos, quat = T[:3, 3], rot_to_quat(T[:3, :3])
        return {
            "time_ms": int((time.time() - self._t0) * 1000),
            "EE_pos": pos.tolist(),
            "EE_quat": quat.tolist(),
            "O_T_EE": T.T.reshape(-1).tolist(),
            "O_T_EE_d": T.T.reshape(-1).tolist(),
            "q": q.tolist(),
            "q_d": q_d.tolist(),
            "dq": dq.tolist(),
            "dq_d": [0.0] * 7,
            "tau_ext_hat_filtered": (0.05 * np.random.randn(7)).tolist(),
            "O_F_ext_hat_K": [0.0] * 6,
            "K_F_ext_hat_K": [0.0] * 6,
        }

    def get_franka_arm_control_mode(self) -> str:
        return self._mode.value

    def set_franka_arm_control_mode(self, mode: ControlMode) -> None:
        with self._lock:
            self._mode = mode
            self._q_cmd = self._q.copy()
            if mode != ControlMode.CartesianImpedance:
                self._ee_cmd = None
                self._ee_override = None

    def move_franka_arm_to_joint_position(self, joint_positions: Sequence[float]) -> None:
        target = np.asarray(joint_positions, dtype=np.float64)
        with self._lock:
            start = self._q.copy()
            self._ee_cmd = None
            self._ee_override = None
        steps = 100
        for i in range(1, steps + 1):  # ~2 s smooth move, like the real one
            s = 0.5 - 0.5 * np.cos(np.pi * i / steps)
            with self._lock:
                self._q = start + s * (target - start)
                self._q_cmd = self._q.copy()
            time.sleep(0.02)

    def send_joint_position_command(self, joint_positions, joint_velocities=None) -> None:
        arr = np.asarray(joint_positions, dtype=np.float64).reshape(-1)
        if arr.size != 7:
            raise ValueError(f"Expected 7 joint angles, got {arr.size}")
        with self._lock:
            self._q_cmd = arr

    def send_cartesian_pose_command(self, pos, rot, pos_vel=None, rot_vel=None) -> None:
        cmd = np.concatenate([np.asarray(pos, dtype=np.float64).reshape(-1), np.asarray(rot, dtype=np.float64).reshape(-1)])
        with self._lock:
            self._ee_cmd = cmd


class SimRobotiqGripper(RemoteRobotiqGripper):
    def __init__(self, name: str = "SimPanda") -> None:  # no super().__init__
        self._name = name
        self._enable_publishers = True
        self._pos = 0.0
        self._cmd = 0.0
        self.loop = _SimLoop(100.0, self._integrate)

    def connect(self) -> None:
        return None

    def _integrate(self, dt: float) -> None:
        step = 1.5 * dt  # full stroke in ~0.7 s
        self._pos += float(np.clip(self._cmd - self._pos, -step, step))

    @property
    def current_state(self):
        return {
            "commanded_position": self._cmd,
            "commanded_speed": 0.7,
            "commanded_force": 0.3,
            "position": self._pos,
            "current": 0.0,
            "raw_commanded_position": int(self._cmd * 255),
            "raw_position": int(self._pos * 255),
        }

    def send_grasp_command(self, position: float, speed: float = 0.1, force: float = 0.1, blocking: bool = False) -> None:
        self._cmd = float(np.clip(position, 0.0, 1.0))
        if blocking:
            time.sleep(0.7)


class SimPandaGripper(RemotePandaGripper):
    MAX_WIDTH = 0.08

    def __init__(self, name: str = "SimPanda") -> None:  # no super().__init__
        self._name = name
        self._width = self.MAX_WIDTH
        self._cmd = self.MAX_WIDTH
        self.loop = _SimLoop(100.0, self._integrate)

    def connect(self) -> None:
        return None

    def _integrate(self, dt: float) -> None:
        step = 0.1 * dt
        self._width += float(np.clip(self._cmd - self._width, -step, step))

    @property
    def current_state(self):
        return {"width": self._width, "max_width": self.MAX_WIDTH, "is_grasped": False, "temperature": 30, "time": 0}

    def send_gripper_command(self, width: float, speed: float = 0.01) -> None:
        self._cmd = float(np.clip(width, 0.0, self.MAX_WIDTH))


class SimCamera:
    """Synthetic frames: a moving gradient with the camera name and time."""

    def __init__(self, name: str, height: int = 240, width: int = 320) -> None:
        self._name = name
        self.size = (height, width)
        yy, xx = np.mgrid[0:height, 0:width]
        self._xx, self._yy = xx, yy

    def get_image(self) -> np.ndarray:
        t = time.time()
        h, w = self.size
        img = np.empty((h, w, 3), dtype=np.uint8)
        img[..., 0] = ((self._xx + 40 * t) % w * 255 / w).astype(np.uint8)
        img[..., 1] = (self._yy * 255 / h).astype(np.uint8)
        img[..., 2] = 120
        cx = int((0.5 + 0.35 * np.sin(t)) * w)
        img[h // 2 - 12 : h // 2 + 12, max(0, cx - 12) : cx + 12] = 255
        try:
            import cv2

            cv2.putText(img, f"{self._name} {t % 100:5.2f}", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        except ImportError:
            pass
        return img


def build_sim_hardware(cfg: SessionConfig, log: Callable[[str], None]) -> Hardware:
    arm = SimPandaArm()
    gripper: GripperT = SimRobotiqGripper() if cfg.gripper_type == "robotiq" else SimPandaGripper()
    cameras = {cam.name: SimCamera(cam.name) for cam in cfg.cameras}
    log("Simulated hardware ready.")
    hw = Hardware(arm=arm, gripper=gripper, cameras=cameras, sim=True)
    hw._threads = [arm.loop, gripper.loop]  # type: ignore[attr-defined]
    return hw


def build_hardware(cfg: SessionConfig, log: Callable[[str], None]) -> Hardware:
    return build_sim_hardware(cfg, log) if cfg.sim else build_real_hardware(cfg, log)
