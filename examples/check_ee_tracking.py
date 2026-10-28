"""Follow a Cartesian test trajectory from the DROID home pose and plot desired vs. actual.

The proxy's CartesianImpedance controller is a pure spring-damper on top of libfranka's
gravity compensation (no integral term, no feed-forward). A wrong end-effector load in
Desk therefore shows up as a pose-dependent offset between desired and actual EE position,
mostly along z. Phase lag is the damper and is expected; a constant or gravity-aligned
offset points at the load.

The trajectory starts at the current EE pose, ramps its speed up and down smoothly and
keeps the start orientation.

Usage (workspace must be clear, nothing may touch the gripper while running):
    python examples/check_ee_tracking.py --shape figure8 --plane yz --size 0.08 --period 8 --laps 3
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from check_ee_load import HOME_Q, _move

PLANES = {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}


def shape_offset(shape: str, theta: float, size: float) -> tuple[float, float]:
    """In-plane offset (u, v) from the start point at phase theta; zero at theta = 0."""
    if shape == "circle":
        return size * (np.cos(theta) - 1.0), size * np.sin(theta)
    if shape == "figure8":
        return size * np.sin(theta), 0.5 * size * np.sin(2 * theta)
    if shape == "line":
        return size * np.sin(theta), 0.0
    raise ValueError(f"Unknown shape {shape}")


def shape_velocity(shape: str, theta: float, size: float, rate: float) -> tuple[float, float]:
    """In-plane velocity (du/dt, dv/dt) for phase theta advancing at `rate` rad/s."""
    if shape == "circle":
        return -size * np.sin(theta) * rate, size * np.cos(theta) * rate
    if shape == "figure8":
        return size * np.cos(theta) * rate, size * np.cos(2 * theta) * rate
    if shape == "line":
        return size * np.cos(theta) * rate, 0.0
    raise ValueError(f"Unknown shape {shape}")


def phase_rate(t: float, duration: float, ramp: float, omega: float) -> float:
    """Angular rate with cosine ramps at the start and end of the trajectory."""
    if t <= 0.0 or t >= duration:
        return 0.0
    edge = min(t, duration - t)
    if edge >= ramp:
        return omega
    return omega * 0.5 * (1.0 - np.cos(np.pi * edge / ramp))


def _pose_from_state(state) -> tuple[np.ndarray, np.ndarray]:
    """Position and rotation matrix of the EE from the column-major O_T_EE."""
    T = np.asarray(state["O_T_EE"]).reshape(4, 4).T
    return T[:3, 3].copy(), T[:3, :3].copy()


def _rot_angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def run_trajectory(robot, args) -> dict:
    from scipy.spatial.transform import Rotation

    state = robot.current_state
    if state is None:
        raise RuntimeError("No arm state received")
    p0, R0 = _pose_from_state(state)
    quat0 = Rotation.from_matrix(R0).as_quat()  # x, y, z, w as the proxy expects
    iu, iv = PLANES[args.plane]

    omega = 2 * np.pi / args.period
    ramp = args.period / 2
    # extra phase covered by the two ramps (each covers half of a full-speed ramp)
    duration = args.laps * args.period + ramp
    dt = 1.0 / args.hz

    log = {k: [] for k in ("t", "p_des", "p_act", "rot_err_deg", "tau_ext", "q", "dq")}
    theta = 0.0
    t_start = time.perf_counter()
    next_tick = t_start
    t_end = duration + args.hold
    while True:
        t = time.perf_counter() - t_start
        if t > t_end:
            break
        rate = phase_rate(t, duration, ramp, omega)
        theta += rate * dt
        du, dv = shape_offset(args.shape, theta, args.size)
        p_des = p0.copy()
        p_des[iu] += du
        p_des[iv] += dv
        v_des = np.zeros(3)
        if not args.no_feedforward:
            v_des[iu], v_des[iv] = shape_velocity(args.shape, theta, args.size, rate)
        robot.send_cartesian_pose_command(pos=p_des, rot=quat0, pos_vel=v_des, rot_vel=np.zeros(3))

        s = robot.current_state
        if s is not None:
            p_act, R_act = _pose_from_state(s)
            log["t"].append(t)
            log["p_des"].append(p_des)
            log["p_act"].append(p_act)
            log["rot_err_deg"].append(_rot_angle_deg(R0, R_act))
            log["tau_ext"].append(s["tau_ext_hat_filtered"])
            log["q"].append(s["q"])
            log["dq"].append(s["dq"])

        next_tick += dt
        sleep = next_tick - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.perf_counter()

    return {k: np.asarray(v) for k, v in log.items()}


def estimate_lag(t: np.ndarray, p_des: np.ndarray, p_act: np.ndarray, max_lag_s: float = 0.5) -> float:
    """Delay [s] that best aligns actual with desired position (least squares over shifts)."""
    dt = float(np.median(np.diff(t)))
    best, best_err = 0, np.inf
    for k in range(int(max_lag_s / dt) + 1):
        err = np.mean(np.sum((p_act[k:] - p_des[: len(p_des) - k]) ** 2, axis=1))
        if err < best_err:
            best, best_err = k, err
    return best * dt


def summarize(log: dict, lag: float) -> dict:
    t, p_des, p_act = log["t"], log["p_des"], log["p_act"]
    err = p_act - p_des
    k = int(round(lag / float(np.median(np.diff(t)))))
    err_lag = p_act[k:] - p_des[: len(p_des) - k]
    return {
        "rms_err_mm": float(1e3 * np.sqrt(np.mean(np.sum(err**2, axis=1)))),
        "max_err_mm": float(1e3 * np.max(np.linalg.norm(err, axis=1))),
        "mean_err_mm": (1e3 * err.mean(axis=0)).tolist(),
        "lag_s": lag,
        "rms_err_lag_comp_mm": float(1e3 * np.sqrt(np.mean(np.sum(err_lag**2, axis=1)))),
        "mean_err_lag_comp_mm": (1e3 * err_lag.mean(axis=0)).tolist(),
        "max_rot_err_deg": float(np.max(log["rot_err_deg"])),
    }


def plot(log: dict, summary: dict, args, out_png: Path) -> None:
    import matplotlib

    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t, p_des, p_act = log["t"], log["p_des"], log["p_act"]
    err_mm = 1e3 * (p_act - p_des)
    iu, iv = PLANES[args.plane]
    names = "xyz"

    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(
        f"{args.shape} in {args.plane}, size {args.size * 100:.0f} cm, period {args.period:.1f} s, "
        f"feed-forward {'off' if args.no_feedforward else 'on'}  |  "
        f"RMS {summary['rms_err_mm']:.1f} mm, lag {summary['lag_s'] * 1e3:.0f} ms, "
        f"RMS lag-comp. {summary['rms_err_lag_comp_mm']:.1f} mm"
    )

    a = ax[0, 0]
    a.plot(1e3 * (p_des[:, iu] - p_des[0, iu]), 1e3 * (p_des[:, iv] - p_des[0, iv]), "--", label="desired")
    a.plot(1e3 * (p_act[:, iu] - p_des[0, iu]), 1e3 * (p_act[:, iv] - p_des[0, iv]), label="actual")
    a.set_xlabel(f"{names[iu]} [mm]")
    a.set_ylabel(f"{names[iv]} [mm]")
    a.set_aspect("equal", adjustable="datalim")
    a.set_title("Path in plane (relative to start)")
    a.legend()
    a.grid(True)

    a = ax[0, 1]
    for i in range(3):
        a.plot(t, err_mm[:, i], label=f"{names[i]}  (mean {summary['mean_err_mm'][i]:+.1f})")
    a.plot(t, np.linalg.norm(err_mm, axis=1), "k", lw=0.8, label="|e|")
    a.set_xlabel("t [s]")
    a.set_ylabel("actual - desired [mm]")
    a.set_title("Position error")
    a.legend()
    a.grid(True)

    a = ax[1, 0]
    a.plot(t, log["rot_err_deg"])
    a.set_xlabel("t [s]")
    a.set_ylabel("[deg]")
    a.set_title("Orientation error (vs. start orientation)")
    a.grid(True)

    a = ax[1, 1]
    for i in range(7):
        a.plot(t, log["tau_ext"][:, i], label=f"j{i + 1}")
    a.set_xlabel("t [s]")
    a.set_ylabel("[Nm]")
    a.set_title("tau_ext_hat_filtered")
    a.legend(ncol=4, fontsize="small")
    a.grid(True)

    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    print(f"Plot written to {out_png}")
    if not args.no_show:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-name", default="FrankaPanda")
    parser.add_argument("--local-ip", default="192.168.1.1")
    parser.add_argument("--group-name", default="DroidGroup")
    parser.add_argument("--group-port", type=int, default=7730)
    parser.add_argument("--shape", choices=["circle", "figure8", "line"], default="figure8")
    parser.add_argument("--plane", choices=list(PLANES), default="yz", help="Plane of the shape in the base frame")
    parser.add_argument("--size", type=float, default=0.08, help="Radius / half-width of the shape [m]")
    parser.add_argument("--period", type=float, default=8.0, help="Time per lap at full speed [s]")
    parser.add_argument("--laps", type=float, default=3.0)
    parser.add_argument("--hz", type=float, default=100.0, help="Command rate [Hz]")
    parser.add_argument("--settle", type=float, default=1.5, help="Wait at home before starting [s]")
    parser.add_argument("--hold", type=float, default=1.5, help="Keep logging after the trajectory ends [s]")
    parser.add_argument(
        "--no-feedforward", action="store_true", help="Send zero velocity with the pose (old controller behaviour)"
    )
    parser.add_argument("--no-show", action="store_true", help="Only save the plot, do not open a window")
    parser.add_argument("--output", type=Path, default=Path("ee_tracking"), help="Output prefix (.png, .json)")
    args = parser.parse_args()

    if args.size > 0.15:
        parser.error("--size above 0.15 m is not allowed around the home pose")
    if args.size * 2 * np.pi / args.period > 0.25:
        parser.error("peak speed above 0.25 m/s, increase --period or reduce --size")

    import pyzlc

    from franka_control_client.franka_robot.panda_arm import ControlMode, RemotePandaArm

    pyzlc.init("ee_tracking_check", args.local_ip, group_name=args.group_name, group_port=args.group_port)
    robot = RemotePandaArm(args.robot_name)
    robot.connect()
    robot.set_franka_arm_control_mode(ControlMode.IDLE)

    print(f"The arm will move to the DROID home pose and follow a {args.shape} ({args.size * 100:.0f} cm) in {args.plane}.")
    input("Clear the workspace, keep the gripper free of contact and hit Enter to start ...")

    _move(robot, HOME_Q)
    time.sleep(args.settle)

    robot.set_franka_arm_control_mode(ControlMode.CartesianImpedance)
    try:
        # the controller latches the current pose on start; give it a moment before streaming
        time.sleep(0.5)
        log = run_trajectory(robot, args)
    finally:
        robot.set_franka_arm_control_mode(ControlMode.IDLE)
    _move(robot, HOME_Q)

    if len(log["t"]) < 10:
        raise RuntimeError("Too few state samples recorded")
    summary = summarize(log, estimate_lag(log["t"], log["p_des"], log["p_act"]))

    print("\n=== Result ===")
    print(f"Position error RMS: {summary['rms_err_mm']:.1f} mm, max: {summary['max_err_mm']:.1f} mm")
    print(f"Mean error x/y/z: {np.array2string(np.array(summary['mean_err_mm']), precision=1)} mm")
    print(f"Estimated lag: {summary['lag_s'] * 1e3:.0f} ms")
    print(
        f"Lag-compensated RMS: {summary['rms_err_lag_comp_mm']:.1f} mm, "
        f"mean x/y/z: {np.array2string(np.array(summary['mean_err_lag_comp_mm']), precision=1)} mm"
    )
    print(f"Max orientation drift: {summary['max_rot_err_deg']:.2f} deg")
    print("A large lag-compensated mean z error (sagging or rising) points at a wrong Desk mass.")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    out_json = args.output.with_suffix(".json")
    out_json.write_text(
        json.dumps(
            {"args": {k: str(v) for k, v in vars(args).items()}, "summary": summary}
            | {k: v.tolist() for k, v in log.items()},
            indent=2,
        )
    )
    print(f"Raw data written to {out_json}")
    plot(log, summary, args, args.output.with_suffix(".png"))


if __name__ == "__main__":
    main()
