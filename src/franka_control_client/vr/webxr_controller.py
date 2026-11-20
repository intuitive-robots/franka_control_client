"""Quest 3 teleop over WebXR: right-controller pose -> target EE pose (clutch on grip),
trigger toggles the gripper between open and closed.

Same interface as MQ3Controller (current_control_signal, reset). While the grip is
held, the EE target follows the controller relative to where both were at clutch time:

    pos = ee_base + k · M @ (ctrl - ctrl_base)
    rot = (M · R_ctrl R_ctrl_base⁻¹ · Mᵀ)^k * R_ee_base

Safety: the clutch drops on a stale sample (> stale_timeout_s), WS disconnect, new
connection, recenter or XR session end, and re-engages only on a fresh grip press.
The target is clamped to a workspace box and rate-limited per received sample. Its twist
(estimated from consecutive targets) is sent as feed-forward and zeroed on de-clutch.

Realign: pressing the right thumbstick takes the headset's current heading as the
operator's forward (``yaw_deg`` in the base frame), like a recenter. It also drops the
clutch. A recenter or a new XR session clears it, because WebXR forward is then the
headset heading again.

Motion scale: the controller delta since clutch (translation and rotation angle) is scaled by
``motion_scale`` in [min_motion_scale, 1]. Left thumbstick up/down raises/lowers it
(exponentially, motion_scale_rate per second at full deflection), pressing it resets to default_motion_scale.
A change while clutched rebases the clutch, so the target does not jump.

Haptics (optional): while clutched, the right controller vibrates with the external force
estimate |O_F_ext_hat_K[:3]|, zeroed at clutch time, scaled from deadband_n (off) to
max_force_n (full). Vibration only, so it cues contact but does not resist the hand.
"""

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional, Protocol, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..core.remote_device import RemoteDevice
from .control_signal import MQ3ControlSignal
from .webxr_frames import (
    clamp_position,
    heading_from_quat,
    in_bounds,
    limit_step,
    position_delta,
    rotation_delta,
    scale_rotation,
    xr_to_base_matrix,
)
from .webxr_server import WebXRServer, XRSample

log = logging.getLogger("webxr")

GRIP_PRESS = 0.9
GRIP_RELEASE = 0.1
TRIGGER_PRESS = 0.7
TRIGGER_RELEASE = 0.3
STICK_DEADZONE = 0.2
DEFAULT_YAW_DEG = -90.0  # CLI default operator heading: 90 deg clockwise (seen from above) from base +X
DEFAULT_MOTION_SCALE = 0.5
MIN_MOTION_SCALE = 0.1
MOTION_SCALE_RATE = 1.5  # per second at full stick deflection (exponential)
GRIPPER_OPEN = 0.0  # Robotiq position: 0.0 open .. 1.0 closed (droid_robotiq.yaml: alpha 1, beta 0)
GRIPPER_CLOSED = 1.0


class _ArmState(Protocol):
    @property
    def current_state(self): ...


@dataclass(frozen=True)
class WorkspaceBounds:
    lower: Tuple[float, float, float] = (0.25, -0.45, -0.05)
    upper: Tuple[float, float, float] = (0.80, 0.45, 0.75)


@dataclass(frozen=True)
class HapticConfig:
    deadband_n: float = 4.0
    max_force_n: float = 30.0
    smoothing: float = 0.3  # EMA weight of the newest force per control tick (1: none)


def force_to_intensity(force_n: float, cfg: HapticConfig) -> float:
    return float(np.clip((force_n - cfg.deadband_n) / (cfg.max_force_n - cfg.deadband_n), 0.0, 1.0))


class WebXRController(RemoteDevice):
    def __init__(
        self,
        name: str,
        panda_arm: _ArmState,
        server: Optional[WebXRServer] = None,
        yaw_deg: float = 0.0,
        bounds: WorkspaceBounds = WorkspaceBounds(),
        max_pos_step: float = 0.01,
        max_rot_step_deg: float = 5.0,
        stale_timeout_s: float = 0.1,
        haptics: Optional[HapticConfig] = None,
        default_motion_scale: float = DEFAULT_MOTION_SCALE,
        min_motion_scale: float = MIN_MOTION_SCALE,
        motion_scale_rate: float = MOTION_SCALE_RATE,
    ) -> None:
        if not 0.0 < min_motion_scale <= default_motion_scale <= 1.0:
            raise ValueError(f"need 0 < min_motion_scale ({min_motion_scale}) <= default_motion_scale "
                             f"({default_motion_scale}) <= 1")
        super().__init__(name)
        self.panda_arm = panda_arm
        self.server = server if server is not None else WebXRServer()
        self.server.start()
        self.yaw_deg = yaw_deg
        self.heading_xr = 0.0  # set by the realign button
        self.m = xr_to_base_matrix(yaw_deg)
        self.bounds = bounds
        self.max_pos_step = max_pos_step
        self.max_rot_step = np.deg2rad(max_rot_step_deg)
        self.stale_timeout_s = stale_timeout_s
        self.min_motion_scale = min_motion_scale
        self.motion_scale_rate = motion_scale_rate
        self.default_motion_scale = default_motion_scale
        self.motion_scale = default_motion_scale
        self._lstick_prev = False
        self._last_scale_t: Optional[float] = None
        self._scale_unlogged = False
        self._state_lock = threading.Lock()
        self.haptics = haptics
        self._force_bias: Optional[np.ndarray] = None
        self._force_filt = 0.0

        self.on_control = False
        self._armed = False  # grip seen released since the last de-clutch
        self._seen_conn_id = 0
        self._seen_reset_count = 0
        self._seen_frame_reset_count = 0
        self._stick_prev = True  # a stick press counts only after it was seen released
        self._last_seq: Optional[int] = None
        self._last_sample: Optional[XRSample] = None
        self.declutch_reason: Optional[str] = None

        self._base_ee_pos: Optional[np.ndarray] = None
        self._base_ee_rot: Optional[R] = None
        self._base_ctrl_pos: Optional[np.ndarray] = None
        self._base_ctrl_rot: Optional[Sequence[float]] = None
        self._target_pos: Optional[np.ndarray] = None
        self._target_rot: Optional[R] = None
        self._gripper = GRIPPER_OPEN
        self._trigger_pressed = False
        self._current_control_signal: Optional[MQ3ControlSignal] = None
        # Target twist (base frame) estimated from consecutive targets, sent as feed-forward so the
        # proxy's reference follows between samples instead of lagging behind the held pose
        self._pos_vel = np.zeros(3)
        self._rot_vel = np.zeros(3)
        self._last_step_t: Optional[float] = None
        self.reset()

    def connect(self) -> None:
        # not a pyzlc node; the server is started in __init__
        return None

    def reset(self) -> None:
        with self._state_lock:
            self._declutch("reset")
            panda_state = self.panda_arm.current_state
            if panda_state is None:
                raise ValueError("No initial state data received from the robot.")
            self._target_pos = np.asarray(panda_state["EE_pos"], dtype=float)
            self._target_rot = R.from_quat(panda_state["EE_quat"])
            self._gripper = GRIPPER_OPEN
            self._publish()

    def sync_target(self, pos: Sequence[float], quat: Sequence[float], gripper: float) -> None:
        """Start the target from the command the robot currently tracks (e.g. a policy's), without
        moving the arm or the gripper. Takes effect only while de-clutched; the next clutch walks
        from this target to the clutch base instead of from a stale one."""
        with self._state_lock:
            if self.on_control:
                return
            self._target_pos = np.asarray(pos, dtype=float).reshape(3)
            self._target_rot = R.from_quat(np.asarray(quat, dtype=float).reshape(4))
            self._gripper = float(gripper)
            self._publish()

    @property
    def last_sample(self) -> Optional[XRSample]:
        return self._last_sample

    @property
    def current_control_signal(self) -> Optional[MQ3ControlSignal]:
        with self._state_lock:
            self._update(time.monotonic())
            self._update_haptics()
            return self._current_control_signal

    # ---- internals (called with _state_lock held) ----

    def _update(self, now: float) -> None:
        snap = self.server.snapshot()
        if snap.conn_id != self._seen_conn_id or snap.reset_count != self._seen_reset_count:
            self._seen_conn_id = snap.conn_id
            self._seen_reset_count = snap.reset_count
            self._last_seq = None
            self._stick_prev = True
            self._declutch("recenter/reconnect")
        if snap.frame_reset_count != self._seen_frame_reset_count:
            self._seen_frame_reset_count = snap.frame_reset_count
            if self.heading_xr != 0.0:
                log.info("WebXR frame reset (recenter / new session): realign cleared")
            self._set_heading(0.0)
        sample = snap.sample
        if not snap.connected or sample is None:
            self._declutch("disconnected" if not snap.connected else "no sample")
            return
        if now - sample.recv_t > self.stale_timeout_s:
            self._declutch("stale")
            return
        if sample.seq == self._last_seq:
            return
        self._last_seq = sample.seq
        self._last_sample = sample
        self._step(sample)

    def _external_force(self) -> Optional[np.ndarray]:
        state = self.panda_arm.current_state
        f = state.get("O_F_ext_hat_K") if state is not None else None
        return None if f is None else np.asarray(f[:3], dtype=float)

    def _update_haptics(self) -> None:
        if self.haptics is None:
            return
        f = self._external_force() if self.on_control else None
        if f is None:
            self._force_filt = 0.0
            self.server.set_haptic(0.0)
            return
        if self._force_bias is None:
            self._force_bias = f
        mag = float(np.linalg.norm(f - self._force_bias))
        a = self.haptics.smoothing
        self._force_filt = a * mag + (1.0 - a) * self._force_filt
        self.server.set_haptic(force_to_intensity(self._force_filt, self.haptics))

    def _step(self, s: XRSample) -> None:
        if s.stick and not self._stick_prev:
            self._realign(s)
        self._stick_prev = s.stick
        self._update_scale(s)
        if not self.on_control:
            if s.grip < GRIP_RELEASE:
                self._armed = True
            if self._armed and s.grip > GRIP_PRESS:
                self._engage(s)
            return
        if s.grip < GRIP_RELEASE:
            self._declutch("grip released")
            self._armed = True
            return

        raw_pos, raw_rot = self._raw_target(s)
        raw_pos = clamp_position(raw_pos, self.bounds.lower, self.bounds.upper)
        prev_pos, prev_rot = self._target_pos, self._target_rot
        self._target_pos, self._target_rot = limit_step(
            self._target_pos, self._target_rot, raw_pos, raw_rot, self.max_pos_step, self.max_rot_step
        )
        self._update_twist(s, prev_pos, prev_rot)
        self._toggle_gripper(s.trigger)
        self._publish()

    def _raw_target(self, s: XRSample) -> Tuple[np.ndarray, R]:
        k = self.motion_scale
        pos = self._base_ee_pos + k * position_delta(self.m, s.pos, self._base_ctrl_pos)
        rot = scale_rotation(rotation_delta(self.m, s.rot, self._base_ctrl_rot), k) * self._base_ee_rot
        return pos, rot

    def _update_scale(self, s: XRSample) -> None:
        t = s.t_client_ms / 1000.0 if s.t_client_ms > 0.0 else s.recv_t
        dt = None if self._last_scale_t is None else t - self._last_scale_t
        self._last_scale_t = t
        pressed = s.left_stick and not self._lstick_prev
        self._lstick_prev = s.left_stick
        if pressed:
            k = self.default_motion_scale
        elif abs(s.left_stick_y) > STICK_DEADZONE and dt is not None and 0.0 < dt <= self.stale_timeout_s:
            k = self.motion_scale * float(np.exp(self.motion_scale_rate * s.left_stick_y * dt))
            k = float(np.clip(k, self.min_motion_scale, 1.0))
        else:
            k = self.motion_scale
        if k != self.motion_scale:
            if self.on_control:
                # new base = current target under the old scale, so the change takes effect without a jump
                self._base_ee_pos, self._base_ee_rot = self._raw_target(s)
                self._base_ctrl_pos = np.asarray(s.pos, dtype=float)
                self._base_ctrl_rot = s.rot
            self.motion_scale = k
            self._scale_unlogged = True
        if self._scale_unlogged and (pressed or abs(s.left_stick_y) <= STICK_DEADZONE):
            self._scale_unlogged = False
            log.info("WebXR motion scale %.2f", self.motion_scale)

    def _toggle_gripper(self, trigger: float) -> None:
        if not self._trigger_pressed and trigger >= TRIGGER_PRESS:
            self._trigger_pressed = True
            self._gripper = GRIPPER_OPEN if self._gripper >= 0.5 else GRIPPER_CLOSED
        elif self._trigger_pressed and trigger <= TRIGGER_RELEASE:
            self._trigger_pressed = False

    def _update_twist(self, s: XRSample, prev_pos: np.ndarray, prev_rot: R) -> None:
        # headset clock (regular frame times); arrival time if the page sent none
        t = s.t_client_ms / 1000.0 if s.t_client_ms > 0.0 else s.recv_t
        dt = None if self._last_step_t is None else t - self._last_step_t
        self._last_step_t = t
        if dt is None or not 1e-3 <= dt <= self.stale_timeout_s:
            self._pos_vel = np.zeros(3)
            self._rot_vel = np.zeros(3)
            return
        self._pos_vel = (self._target_pos - prev_pos) / dt
        self._rot_vel = (self._target_rot * prev_rot.inv()).as_rotvec() / dt

    def _realign(self, s: XRSample) -> None:
        if s.head_rot is None:
            log.warning("WebXR realign: no headset pose in the sample (reload the page)")
            return
        heading = heading_from_quat(s.head_rot)
        if heading is None:
            log.warning("WebXR realign: looking too far up/down, look toward your working direction")
            return
        # the clutch base was taken in the old mapping; keeping it would make the target jump
        self._declutch("realign")
        self._set_heading(heading)
        log.info(
            "WebXR realigned: your current facing is now base heading %.0f deg (offset %.0f deg from WebXR forward)",
            self.yaw_deg, np.rad2deg(heading),
        )

    def _set_heading(self, heading_xr: float) -> None:
        self.heading_xr = heading_xr
        self.m = xr_to_base_matrix(self.yaw_deg, heading_xr)

    def _engage(self, s: XRSample) -> None:
        arm_state = self.panda_arm.current_state
        if arm_state is None:
            log.warning("WebXR clutch: no arm state, not engaging")
            return
        ee_pos = np.asarray(arm_state["EE_pos"], dtype=float)
        if not in_bounds(ee_pos, self.bounds.lower, self.bounds.upper):
            log.warning(
                "WebXR clutch: EE %s is outside the workspace bounds %s..%s, not engaging",
                np.round(ee_pos, 3).tolist(), self.bounds.lower, self.bounds.upper,
            )
            self._armed = False
            return
        self._base_ee_pos = ee_pos
        self._base_ee_rot = R.from_quat(arm_state["EE_quat"])
        self._base_ctrl_pos = np.asarray(s.pos, dtype=float)
        self._base_ctrl_rot = s.rot
        # a trigger already down at clutch time toggles only after it has been let go
        self._trigger_pressed = s.trigger > TRIGGER_RELEASE
        self._force_bias = self._external_force()
        self._force_filt = 0.0
        # the target keeps its last commanded value; limit_step walks it to the new base
        self.on_control = True
        self.declutch_reason = None
        log.info("WebXR clutch engaged")

    def _declutch(self, reason: str) -> None:
        if self.on_control:
            log.info("WebXR clutch released: %s", reason)
            self.declutch_reason = reason
        self.on_control = False
        if reason != "grip released":
            self._armed = False
        self._base_ee_pos = None
        self._base_ee_rot = None
        self._base_ctrl_pos = None
        self._base_ctrl_rot = None
        self._force_bias = None
        # stop the feed-forward at once, otherwise the proxy keeps following the last twist
        self._pos_vel = np.zeros(3)
        self._rot_vel = np.zeros(3)
        self._last_step_t = None
        if self._current_control_signal is not None and self._target_pos is not None:
            self._publish()

    def _publish(self) -> None:
        self._current_control_signal = MQ3ControlSignal(
            pos=self._target_pos.tolist(),
            rot=self._target_rot.as_quat().tolist(),
            pos_vel=self._pos_vel.tolist(),
            rot_vel=self._rot_vel.tolist(),
            gripper_width=self._gripper,
        )
