import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from franka_control_client.vr.webxr_controller import WebXRController, WorkspaceBounds
from franka_control_client.vr.webxr_frames import (
    heading_from_quat,
    limit_step,
    position_delta,
    rotation_delta,
    xr_to_base_matrix,
)
from franka_control_client.vr.webxr_server import XRSample, XRSnapshot

EE_POS = [0.5, 0.0, 0.4]
EE_QUAT = [1.0, 0.0, 0.0, 0.0]  # gripper pointing down
IDENT = [0.0, 0.0, 0.0, 1.0]


# ---- frames ----

@pytest.mark.parametrize(
    "xr_dir, base_dir",
    [
        ([0, 0, -1], [1, 0, 0]),  # forward -> +X
        ([-1, 0, 0], [0, 1, 0]),  # left -> +Y
        ([0, 1, 0], [0, 0, 1]),  # up -> +Z
    ],
)
def test_axes_default(xr_dir, base_dir):
    m = xr_to_base_matrix(0.0)
    np.testing.assert_allclose(position_delta(m, xr_dir, [0, 0, 0]), base_dir, atol=1e-12)


def test_matrix_is_proper_rotation():
    for yaw in (0.0, 37.0, -90.0, 180.0):
        m = xr_to_base_matrix(yaw)
        np.testing.assert_allclose(m @ m.T, np.eye(3), atol=1e-12)
        assert np.linalg.det(m) == pytest.approx(1.0)


def test_yaw_sign():
    # yaw 90: the operator faces base +Y, so their forward maps to +Y and their left to -X
    m = xr_to_base_matrix(90.0)
    np.testing.assert_allclose(position_delta(m, [0, 0, -1], [0, 0, 0]), [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(position_delta(m, [-1, 0, 0], [0, 0, 0]), [-1, 0, 0], atol=1e-12)


def test_rotation_about_up_maps_to_base_z():
    m = xr_to_base_matrix(0.0)
    turn_left = R.from_rotvec([0, np.deg2rad(30), 0]).as_quat()  # CCW about XR +Y (up)
    delta = rotation_delta(m, turn_left, IDENT)
    np.testing.assert_allclose(delta.as_rotvec(), [0, 0, np.deg2rad(30)], atol=1e-12)


def test_rotation_about_forward_maps_to_base_x():
    m = xr_to_base_matrix(0.0)
    roll = R.from_rotvec([0, 0, -np.deg2rad(20)]).as_quat()  # about XR forward (-Z)
    delta = rotation_delta(m, roll, IDENT)
    np.testing.assert_allclose(delta.as_rotvec(), [np.deg2rad(20), 0, 0], atol=1e-12)


def test_rotation_delta_independent_of_base_orientation():
    m = xr_to_base_matrix(0.0)
    base = R.from_euler("xyz", [10, -40, 70], degrees=True)
    step = R.from_rotvec([0, np.deg2rad(15), 0])
    delta = rotation_delta(m, (step * base).as_quat(), base.as_quat())
    np.testing.assert_allclose(delta.as_rotvec(), [0, 0, np.deg2rad(15)], atol=1e-9)


@pytest.mark.parametrize("deg", [0.0, 30.0, -75.0, 120.0, 179.0])
def test_heading_from_quat(deg):
    # head turned CCW (left) about +Y by deg, pitched down 20 deg: pitch must not change the heading
    q = (R.from_euler("y", deg, degrees=True) * R.from_euler("x", -20, degrees=True)).as_quat()
    assert heading_from_quat(q) == pytest.approx(np.deg2rad(deg))


def test_heading_undefined_when_looking_down():
    assert heading_from_quat(R.from_euler("x", -85, degrees=True).as_quat()) is None


def test_heading_offset_maps_facing_to_forward():
    # operator turned 90 deg left in the XR frame: their forward is XR -X
    m = xr_to_base_matrix(0.0, np.deg2rad(90))
    np.testing.assert_allclose(position_delta(m, [-1, 0, 0], [0, 0, 0]), [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(position_delta(m, [0, 0, 1], [0, 0, 0]), [0, 1, 0], atol=1e-12)  # their left
    np.testing.assert_allclose(position_delta(m, [0, 1, 0], [0, 0, 0]), [0, 0, 1], atol=1e-12)
    turn_left = R.from_rotvec([0, np.deg2rad(30), 0]).as_quat()
    np.testing.assert_allclose(rotation_delta(m, turn_left, IDENT).as_rotvec(), [0, 0, np.deg2rad(30)], atol=1e-12)


def test_limit_step():
    pos, rot = limit_step(np.zeros(3), R.identity(), np.array([0.1, 0, 0]), R.from_rotvec([0, 0, 1.0]), 0.01, np.deg2rad(5))
    np.testing.assert_allclose(pos, [0.01, 0, 0])
    assert np.linalg.norm(rot.as_rotvec()) == pytest.approx(np.deg2rad(5))
    pos, rot = limit_step(np.zeros(3), R.identity(), np.array([0.005, 0, 0]), R.from_rotvec([0, 0, 0.01]), 0.01, np.deg2rad(5))
    np.testing.assert_allclose(pos, [0.005, 0, 0])
    np.testing.assert_allclose(rot.as_rotvec(), [0, 0, 0.01])


# ---- controller ----

class FakeArm:
    def __init__(self, pos=EE_POS, quat=EE_QUAT):
        self.current_state = {"EE_pos": list(pos), "EE_quat": list(quat)}


class FakeServer:
    def __init__(self):
        self.conn_id = 1
        self.connected = True
        self.reset_count = 1
        self.frame_reset_count = 0
        self.sample = None
        self.seq = 0

    def start(self):
        pass

    def push(self, pos=(0, 1, 0), rot=IDENT, grip=0.0, trigger=0.0, age=0.0, stick=False, head=IDENT):
        self.seq += 1
        self.sample = XRSample(self.seq, 0.0, tuple(pos), tuple(rot), trigger, grip, False, False,
                               time.monotonic() - age, self.conn_id, stick=stick,
                               head_rot=tuple(head) if head is not None else None)

    def snapshot(self):
        return XRSnapshot(self.sample, self.conn_id, self.connected, self.reset_count, None, self.frame_reset_count)


def make(arm=None, **kw):
    server = FakeServer()
    ctrl = WebXRController("webxr", arm or FakeArm(), server=server, **kw)
    return ctrl, server


def step(ctrl, server, **kw):
    server.push(**kw)
    return ctrl.current_control_signal


def engage(ctrl, server, pos=(0, 1, 0)):
    step(ctrl, server, pos=pos, grip=0.0)
    step(ctrl, server, pos=pos, grip=1.0)
    assert ctrl.on_control


def test_holds_initial_pose_until_clutched():
    ctrl, server = make()
    sig = step(ctrl, server, pos=(0, 1, -0.5), grip=0.5)
    np.testing.assert_allclose(sig["pos"], EE_POS)
    assert not ctrl.on_control


def test_clutch_hysteresis():
    ctrl, server = make()
    step(ctrl, server, grip=0.0)
    step(ctrl, server, grip=0.85)
    assert not ctrl.on_control  # below press threshold
    step(ctrl, server, grip=0.95)
    assert ctrl.on_control
    step(ctrl, server, grip=0.5)
    assert ctrl.on_control  # between thresholds: stays engaged
    step(ctrl, server, grip=0.05)
    assert not ctrl.on_control
    step(ctrl, server, grip=0.95)
    assert ctrl.on_control  # normal release re-arms


def test_grip_already_held_at_connect_does_not_engage():
    ctrl, server = make()
    step(ctrl, server, grip=1.0)
    step(ctrl, server, grip=1.0)
    assert not ctrl.on_control


def test_forward_motion_moves_ee_plus_x():
    ctrl, server = make()
    engage(ctrl, server)
    sig = step(ctrl, server, pos=(0, 1, -0.005), grip=1.0)
    np.testing.assert_allclose(sig["pos"], [0.505, 0.0, 0.4], atol=1e-9)
    np.testing.assert_allclose(R.from_quat(sig["rot"]).as_quat(), R.from_quat(EE_QUAT).as_quat(), atol=1e-9)


def test_trigger_drives_gripper_and_holds_on_declutch():
    ctrl, server = make()
    engage(ctrl, server)
    assert step(ctrl, server, grip=1.0, trigger=0.7)["gripper_width"] == pytest.approx(0.7)
    assert step(ctrl, server, grip=0.0, trigger=0.0)["gripper_width"] == pytest.approx(0.7)


def test_stale_sample_declutches_and_needs_fresh_press():
    ctrl, server = make()
    engage(ctrl, server)
    server.push(grip=1.0, age=0.15)
    ctrl.current_control_signal
    assert not ctrl.on_control and ctrl.declutch_reason == "stale"
    step(ctrl, server, grip=1.0)
    assert not ctrl.on_control  # still held: not a fresh press
    engage(ctrl, server)


def test_stale_without_new_sample():
    ctrl, server = make(stale_timeout_s=0.05)
    engage(ctrl, server)
    time.sleep(0.07)
    ctrl.current_control_signal
    assert not ctrl.on_control


def test_disconnect_declutches():
    ctrl, server = make()
    engage(ctrl, server)
    server.connected = False
    ctrl.current_control_signal
    assert not ctrl.on_control


def test_reset_message_and_reconnect_declutch():
    ctrl, server = make()
    engage(ctrl, server)
    server.reset_count += 1
    step(ctrl, server, grip=1.0)
    assert not ctrl.on_control
    engage(ctrl, server)
    server.conn_id += 1
    server.seq = 0  # new page: seq restarts
    step(ctrl, server, grip=1.0)
    assert not ctrl.on_control
    engage(ctrl, server)


def test_step_cap_and_workspace_clamp():
    ctrl, server = make(bounds=WorkspaceBounds((0.25, -0.45, 0.05), (0.52, 0.45, 0.75)))
    engage(ctrl, server)
    sig = step(ctrl, server, pos=(0, 1, -0.5), grip=1.0)  # 0.5 m forward in one sample
    np.testing.assert_allclose(sig["pos"], [0.51, 0.0, 0.4], atol=1e-9)
    for _ in range(10):
        sig = step(ctrl, server, pos=(0, 1, -0.5), grip=1.0)
    np.testing.assert_allclose(sig["pos"], [0.52, 0.0, 0.4], atol=1e-9)


def test_rotation_step_cap():
    ctrl, server = make()
    engage(ctrl, server)
    big = R.from_rotvec([0, np.deg2rad(40), 0]).as_quat()
    sig = step(ctrl, server, rot=big, grip=1.0)
    delta = R.from_quat(sig["rot"]) * R.from_quat(EE_QUAT).inv()
    np.testing.assert_allclose(delta.as_rotvec(), [0, 0, np.deg2rad(5)], atol=1e-9)


def test_no_engage_outside_bounds():
    ctrl, server = make(arm=FakeArm(pos=[0.1, 0.0, 0.4]))
    step(ctrl, server, grip=0.0)
    step(ctrl, server, grip=1.0)
    assert not ctrl.on_control


def test_reengage_walks_from_last_command():
    arm = FakeArm()
    ctrl, server = make(arm=arm)
    engage(ctrl, server)
    step(ctrl, server, grip=0.0)
    arm.current_state["EE_pos"] = [0.55, 0.0, 0.4]  # robot ended up 5 cm away from the held target
    engage(ctrl, server)
    sig = step(ctrl, server, grip=1.0)
    np.testing.assert_allclose(sig["pos"], [0.51, 0.0, 0.4], atol=1e-9)


TURNED_LEFT_90 = R.from_euler("y", 90, degrees=True).as_quat()  # head facing XR -X


def test_realign_maps_current_facing_to_base_x():
    ctrl, server = make()
    step(ctrl, server, head=TURNED_LEFT_90)
    step(ctrl, server, head=TURNED_LEFT_90, stick=True)
    assert ctrl.heading_xr == pytest.approx(np.pi / 2)
    step(ctrl, server, stick=False)
    engage(ctrl, server)
    sig = step(ctrl, server, pos=(-0.005, 1, 0), grip=1.0)  # 5 mm along the new forward (XR -X)
    np.testing.assert_allclose(sig["pos"], [0.505, 0.0, 0.4], atol=1e-9)


def test_realign_declutches_and_needs_fresh_press():
    ctrl, server = make()
    engage(ctrl, server)
    step(ctrl, server, grip=1.0, head=TURNED_LEFT_90, stick=True)
    assert not ctrl.on_control and ctrl.declutch_reason == "realign"
    step(ctrl, server, grip=1.0)
    assert not ctrl.on_control
    engage(ctrl, server)


def test_realign_needs_stick_release_and_ignores_held_stick_at_connect():
    ctrl, server = make()
    step(ctrl, server, head=TURNED_LEFT_90, stick=True)  # held since before connecting
    assert ctrl.heading_xr == 0.0
    step(ctrl, server, head=TURNED_LEFT_90, stick=False)
    step(ctrl, server, head=TURNED_LEFT_90, stick=True)
    assert ctrl.heading_xr == pytest.approx(np.pi / 2)
    step(ctrl, server, head=IDENT, stick=True)  # still held: no second realign
    assert ctrl.heading_xr == pytest.approx(np.pi / 2)


def test_realign_without_head_pose_or_looking_down_is_ignored():
    ctrl, server = make()
    step(ctrl, server, head=None)
    step(ctrl, server, head=None, stick=True)
    assert ctrl.heading_xr == 0.0
    step(ctrl, server, stick=False)
    step(ctrl, server, head=R.from_euler("x", -85, degrees=True).as_quat(), stick=True)
    assert ctrl.heading_xr == 0.0


def test_recenter_clears_realign_but_reconnect_keeps_it():
    ctrl, server = make()
    step(ctrl, server, head=TURNED_LEFT_90)
    step(ctrl, server, head=TURNED_LEFT_90, stick=True)
    server.conn_id += 1
    server.seq = 0
    step(ctrl, server)
    assert ctrl.heading_xr == pytest.approx(np.pi / 2)  # same XR frame, only the socket changed
    server.frame_reset_count += 1
    step(ctrl, server)
    assert ctrl.heading_xr == 0.0


def test_webxr_modules_do_not_import_simpub():
    import subprocess
    import sys

    code = (
        "import sys;"
        "import franka_control_client.vr.webxr_controller;"
        "import franka_control_client.control_pair.mq3_panda_control_pair;"
        "import franka_control_client.data_collection.irl_wrapper;"
        "assert 'simpub' not in sys.modules, 'simpub imported'"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
