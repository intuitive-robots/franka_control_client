"""Keyboard "leader device": held keys -> end-effector velocity -> integrated Cartesian target.

Keys (made for planar pushing tasks such as push-T):

    ↑ / ↓      EE forward / backward   (base +x / -x with yaw 0)
    ← / →      EE left / right         (base +y / -y with yaw 0)
    q / e      EE up / down            (base +z / -z)
    shift      slow (fine) motion

The orientation stays the one measured when the target is anchored (``anchor``), so the tool
keeps pointing down while it moves. ``yaw_deg`` rotates the arrow keys about z to match where
the operator stands (180: facing the robot from the front).

The key state comes from any source through ``set_keys`` (the data collection GUI sends the
browser's keydown/keyup). It is a dead-man input: without a refresh for ``timeout_s`` every key
counts as released, so a lost connection stops the motion.

Safety on the target (``KeyboardPandaControlPair`` calls ``step``):
* clamped to the workspace box ``bounds`` (min xyz, max xyz),
* never more than ``max_lead`` metres ahead of the measured EE per axis, so holding a key while
  the tool is blocked (table, object) does not wind the target up into a large force.
"""

from __future__ import annotations

import threading
import time
from typing import Dict, Iterable, Optional, Sequence, Tuple

import numpy as np

KEYS = ("up", "down", "left", "right", "q", "e")
# key -> unit direction in the operator frame (x forward, y left, z up)
_DIRS = {
    "up": (1.0, 0.0, 0.0),
    "down": (-1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
    "q": (0.0, 0.0, 1.0),
    "e": (0.0, 0.0, -1.0),
}


class KeyboardController:
    def __init__(
        self,
        speed: float = 0.05,
        z_speed: Optional[float] = None,
        slow_factor: float = 0.25,
        yaw_deg: float = 0.0,
        bounds: Tuple[Sequence[float], Sequence[float]] = ((0.25, -0.45, 0.0), (0.85, 0.45, 0.7)),
        max_lead: float = 0.02,
        timeout_s: float = 0.35,
    ) -> None:
        self.speed = float(speed)
        self.z_speed = float(z_speed if z_speed is not None else speed)
        self.slow_factor = float(slow_factor)
        self.yaw_deg = float(yaw_deg)
        self.lo = np.asarray(bounds[0], dtype=np.float64)
        self.hi = np.asarray(bounds[1], dtype=np.float64)
        self.max_lead = float(max_lead)
        self.timeout_s = float(timeout_s)
        self.gripper = 0.0  # commanded gripper closedness, recorded with the target
        self._lock = threading.Lock()
        self._keys: frozenset = frozenset()
        self._slow = False
        self._t_keys = 0.0
        self._pos: Optional[np.ndarray] = None
        self._quat: Optional[np.ndarray] = None
        self._vel = np.zeros(3)

    # ------------------------------------------------------------------ input
    def set_keys(self, keys: Iterable[str], slow: bool = False) -> None:
        with self._lock:
            self._keys = frozenset(k for k in keys if k in _DIRS)
            self._slow = bool(slow)
            self._t_keys = time.monotonic()

    def release_all(self) -> None:
        self.set_keys((), False)

    def held_keys(self) -> Tuple[frozenset, bool]:
        with self._lock:
            if time.monotonic() - self._t_keys > self.timeout_s:
                return frozenset(), False
            return self._keys, self._slow

    def direction(self) -> np.ndarray:
        """Commanded velocity [m/s] in the base frame for the keys held now."""
        keys, slow = self.held_keys()
        d = np.zeros(3)
        for k in keys:
            d += _DIRS[k]
        yaw = np.deg2rad(self.yaw_deg)
        c, s = np.cos(yaw), np.sin(yaw)
        xy = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
        n = np.linalg.norm(xy)
        if n > 1e-9:
            xy /= n  # diagonal: same speed as straight
        v = np.array([xy[0] * self.speed, xy[1] * self.speed, np.sign(d[2]) * self.z_speed])
        return v * (self.slow_factor if slow else 1.0)

    # ----------------------------------------------------------------- target
    def anchor(self, pos: Sequence[float], quat: Sequence[float]) -> None:
        """Start the target at the measured EE pose (no jump when control starts)."""
        with self._lock:
            self._pos = np.clip(np.asarray(pos, dtype=np.float64).reshape(3), self.lo, self.hi)
            self._quat = np.asarray(quat, dtype=np.float64).reshape(4)
            self._vel = np.zeros(3)

    @property
    def anchored(self) -> bool:
        return self._pos is not None

    def step(self, dt: float, measured_pos: Optional[Sequence[float]] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Advance the target by ``dt``; returns (pos, quat, effective velocity)."""
        v = self.direction()
        with self._lock:
            if self._pos is None:
                raise RuntimeError("KeyboardController.step before anchor()")
            old = self._pos.copy()
            new = np.clip(old + v * dt, self.lo, self.hi)
            if measured_pos is not None and self.max_lead > 0:
                m = np.asarray(measured_pos, dtype=np.float64).reshape(3)
                # only limit motion away from the measured pose; moving back is always allowed
                lead = new - m
                over = np.abs(lead) > self.max_lead
                away = np.sign(new - old) == np.sign(lead)
                limit = over & away
                new[limit] = np.where(np.abs(old[limit] - m[limit]) > self.max_lead, old[limit],
                                      m[limit] + np.sign(lead[limit]) * self.max_lead)
            self._pos = new
            self._vel = (new - old) / dt if dt > 0 else np.zeros(3)
            return new.copy(), self._quat.copy(), self._vel.copy()

    @property
    def current_control_signal(self) -> Optional[Dict[str, object]]:
        """Same keys as MQ3Controller.current_control_signal (the recorder uses it)."""
        with self._lock:
            if self._pos is None:
                return None
            return {
                "pos": self._pos.tolist(),
                "rot": self._quat.tolist(),
                "pos_vel": self._vel.tolist(),
                "rot_vel": [0.0, 0.0, 0.0],
                "gripper_width": float(self.gripper),
            }
