"""Check whether the end-effector load configured in Franka Desk matches the mounted hardware.

libfranka compensates gravity (and computes tau_ext_hat_filtered) with the end-effector
mass / CoM configured in Desk. If that load is wrong, the robot reports a pose-dependent
"external" torque even when nothing touches it. This script moves the arm through a set
of poses, averages tau_ext_hat_filtered at rest and fits the unmodelled payload

    tau_ext(q) = Jv(q)^T (dm * g) + Jw(q)^T ((R(q) * dm*c) x g) + bias

for dm (missing mass) and c (its CoM in the flange frame). If the current Desk values
are given, the corrected Desk values are printed.

Usage (workspace must be clear, nothing may touch the gripper while measuring):
    python examples/check_ee_load.py --desk-mass 0.73 --desk-com -0.01 0 0.03
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.81])

# Craig (modified) DH parameters of the Panda: a, d, alpha. Last row is the flange.
PANDA_DH = np.array(
    [
        [0.0, 0.333, 0.0],
        [0.0, 0.0, -np.pi / 2],
        [0.0, 0.316, np.pi / 2],
        [0.0825, 0.0, np.pi / 2],
        [-0.0825, 0.384, -np.pi / 2],
        [0.0, 0.0, np.pi / 2],
        [0.088, 0.0, np.pi / 2],
        [0.0, 0.107, 0.0],
    ]
)

# DROID home, same as DEFAULT_POSITION used by the control pairs' go_home()
HOME_Q = np.array([0.0, 0.0, 0.0, -2.15, 0.0, 2.15, 0.0])

# Offsets from HOME_Q chosen to rotate the flange in different directions while keeping the
# gripper tip >= 20 cm above the robot base.
# Tilting (joints 5/6) and spinning (joint 7) make dm and all three CoM components observable.
POSE_OFFSETS = [
    [0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1.5],
    [0, 0, 0, 0, 0, 0, -1.5],
    [0, 0, 0, 0, 0, -0.55, 0],
    [0, 0, 0, 0, 0, 0.5, 0.8],
    [0, 0, 0, 0, 0.8, 0, 0],
    [0, 0, 0, 0, -0.8, 0, 0.8],
    [0, -0.3, 0, 0.35, 0, -0.35, -0.8],
    [0, -0.3, 0, 0, 0, 0.3, 0],
]


def _dh(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta)
    return np.array(
        [
            [ct, -st, 0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0, 0, 0, 1],
        ]
    )


def flange_kinematics(q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return flange rotation, flange position and the 6x7 flange Jacobian (world aligned)."""
    T = np.eye(4)
    origins, axes = [], []
    for i, (a, d, alpha) in enumerate(PANDA_DH):
        T = T @ _dh(a, d, alpha, q[i] if i < 7 else 0.0)
        if i < 7:
            origins.append(T[:3, 3].copy())
            axes.append(T[:3, 2].copy())
    p = T[:3, 3]
    J = np.zeros((6, 7))
    for i in range(7):
        J[:3, i] = np.cross(axes[i], p - origins[i])
        J[3:, i] = axes[i]
    return T[:3, :3], p, J


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def regressor(q: np.ndarray) -> np.ndarray:
    """7x4 regressor mapping [dm, dm*cx, dm*cy, dm*cz] to joint torques on the robot."""
    R, _, J = flange_kinematics(q)
    Y = np.zeros((7, 4))
    Y[:, 0] = J[:3].T @ GRAVITY
    # (R h) x g = -[g]x R h
    Y[:, 1:] = -J[3:].T @ _skew(GRAVITY) @ R
    return Y


def fit_payload(qs: np.ndarray, taus: np.ndarray, fit_bias: bool = True) -> dict:
    """Least-squares fit of the unmodelled payload from static torques acting on the robot."""
    rows = []
    for q in qs:
        Y = regressor(q)
        rows.append(np.hstack([Y, np.eye(7)]) if fit_bias else Y)
    A = np.vstack(rows)
    b = np.concatenate(taus)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    residual = (b - A @ x).reshape(-1, 7)
    dm, h = x[0], x[1:4]
    return {
        "dm": float(dm),
        "dm_com": h,
        "com": h / dm if abs(dm) > 1e-3 else np.full(3, np.nan),
        "bias": x[4:] if fit_bias else np.zeros(7),
        "residual_rms": float(np.sqrt(np.mean(residual**2))),
        "cond": float(np.linalg.cond(A)),
    }


def _move(robot, target: np.ndarray, max_step: float = 0.8) -> None:
    """Move in segments: the proxy moves at ~0.2-0.25 rad/s and the service call times out after 10 s."""
    state = robot.current_state
    start = np.asarray(state["q"]) if state is not None else target
    n = max(1, int(np.ceil(np.max(np.abs(target - start)) / max_step)))
    for k in range(1, n + 1):
        robot.move_franka_arm_to_joint_position((start + (target - start) * k / n).tolist())


def _average_state(robot, duration_s: float, rate_hz: float = 100.0) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    qs, taus, forces, max_dq = [], [], [], 0.0
    t_end = time.time() + duration_s
    while time.time() < t_end:
        s = robot.current_state
        if s is not None:
            qs.append(s["q"])
            taus.append(s["tau_ext_hat_filtered"])
            forces.append(s["O_F_ext_hat_K"])
            max_dq = max(max_dq, float(np.max(np.abs(s["dq"]))))
        time.sleep(1.0 / rate_hz)
    if not qs:
        raise RuntimeError("No arm state received")
    return np.mean(qs, axis=0), np.mean(taus, axis=0), np.mean(forces, axis=0), max_dq


def check_sign_convention(robot, settle_s: float) -> float:
    """Return +1 if tau_ext_hat_filtered is the torque acting on the robot, -1 if opposite."""
    _, _, f0, _ = _average_state(robot, settle_s)
    input("\nSign check: press DOWN on the gripper with ~15-20 N, keep pressing and hit Enter ...")
    _, _, f1, _ = _average_state(robot, 1.0)
    input("Release the gripper and hit Enter ...")
    dfz = f1[2] - f0[2]
    print(f"  O_F_ext_hat_K z changed by {dfz:+.1f} N while pushing down")
    if abs(dfz) < 3.0:
        raise RuntimeError("Push not detected clearly (< 3 N), rerun the sign check")
    # Pushing down applies -z force to the robot.
    sign = 1.0 if dfz < 0 else -1.0
    print(
        "  -> tau_ext_hat_filtered is the torque the environment applies ON the robot"
        if sign > 0
        else "  -> tau_ext_hat_filtered is the torque the robot applies on the environment (negated)"
    )
    return sign


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-name", default="FrankaPanda")
    parser.add_argument("--local-ip", default="192.168.1.1")
    parser.add_argument("--group-name", default="DroidGroup")
    parser.add_argument("--group-port", type=int, default=7730)
    parser.add_argument("--desk-mass", type=float, default=None, help="End-effector mass currently set in Desk [kg]")
    parser.add_argument("--desk-com", type=float, nargs=3, default=None, help="CoM currently set in Desk, flange frame [m]")
    parser.add_argument("--settle", type=float, default=1.5, help="Wait after each move [s]")
    parser.add_argument("--average", type=float, default=2.0, help="Averaging window per pose [s]")
    parser.add_argument("--no-bias", action="store_true", help="Do not fit per-joint torque sensor offsets")
    parser.add_argument(
        "--sign",
        type=float,
        choices=[1.0, -1.0],
        default=None,
        help="Skip the interactive push test: +1 = tau_ext is torque on the robot, -1 = negated",
    )
    parser.add_argument("--output", type=Path, default=Path("ee_load_check.json"))
    args = parser.parse_args()

    import pyzlc

    from franka_control_client.franka_robot.panda_arm import ControlMode, RemotePandaArm

    pyzlc.init("ee_load_check", args.local_ip, group_name=args.group_name, group_port=args.group_port)
    robot = RemotePandaArm(args.robot_name)
    robot.connect()
    robot.set_franka_arm_control_mode(ControlMode.IDLE)

    print("The arm will move through", len(POSE_OFFSETS), "poses around the DROID home pose.")
    input("Clear the workspace, keep the gripper free of contact and hit Enter to start ...")

    _move(robot, HOME_Q)
    time.sleep(args.settle)
    sign = args.sign if args.sign is not None else check_sign_convention(robot, args.settle)

    qs, taus, records = [], [], []
    for i, offset in enumerate(POSE_OFFSETS):
        target = HOME_Q + np.array(offset)
        _move(robot, target)
        time.sleep(args.settle)
        q, tau, force, max_dq = _average_state(robot, args.average)
        if max_dq > 0.02:
            print(f"  pose {i}: arm still moving (max |dq| = {max_dq:.3f} rad/s), result may be off")
        tau_on_robot = sign * tau
        qs.append(q)
        taus.append(tau_on_robot)
        records.append({"q": q.tolist(), "tau_ext_hat_filtered": tau.tolist(), "O_F_ext_hat_K": force.tolist()})
        print(
            f"  pose {i}: tau_ext = {np.array2string(tau, precision=2, suppress_small=True)}"
            f"  |F| = {np.linalg.norm(force[:3]):.2f} N"
        )

    _move(robot, HOME_Q)

    fit = fit_payload(np.array(qs), np.array(taus), fit_bias=not args.no_bias)
    worst = float(np.max(np.abs(np.array(taus) - fit["bias"])))

    print("\n=== Result ===")
    print(f"Largest static |tau_ext| (bias removed): {worst:.2f} Nm")
    print(f"Unmodelled mass dm: {fit['dm']:+.3f} kg  (positive = hardware heavier than Desk config)")
    print(f"dm * CoM (flange frame): {np.array2string(fit['dm_com'], precision=4)} kg m")
    if abs(fit["dm"]) > 0.05:
        print(f"CoM of the unmodelled mass: {np.array2string(fit['com'], precision=3)} m")
    print(f"Sensor bias per joint: {np.array2string(fit['bias'], precision=2)} Nm")
    print(f"Fit residual RMS: {fit['residual_rms']:.3f} Nm, regressor condition: {fit['cond']:.0f}")

    if abs(fit["dm"]) < 0.05 and np.linalg.norm(fit["dm_com"]) < 0.005:
        print("-> Desk end-effector load looks correct.")
    else:
        print("-> Desk end-effector load does NOT match the mounted hardware.")
        if args.desk_mass is not None and args.desk_com is not None:
            m_new = args.desk_mass + fit["dm"]
            com_new = (args.desk_mass * np.array(args.desk_com) + fit["dm_com"]) / m_new
            print(f"   Suggested Desk values: mass = {m_new:.3f} kg, CoM = {np.array2string(com_new, precision=4)} m")
            print("   (inertia is not observable from static poses; keep a reasonable estimate)")
        else:
            print("   Pass --desk-mass and --desk-com to get corrected Desk values.")

    result = {
        "sign": sign,
        "poses": records,
        "fit": {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in fit.items()},
    }
    args.output.write_text(json.dumps(result, indent=2))
    print(f"\nRaw data written to {args.output}")


if __name__ == "__main__":
    main()
