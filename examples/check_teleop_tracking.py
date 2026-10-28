"""Measure how well the Panda follows GELLO during normal teleoperation.

Passive: this script sends nothing. Run your usual teleop (e.g. `pixi run data-collection`),
start this script, move GELLO through typical motions (slow and fast, all joints) and wait
for the recording to finish. It reports

  - GELLO sample rate: how often the leader actually delivers a new reading (bus rate),
  - lag per joint: delay that best aligns the Panda's q with the GELLO motion,
  - tracking error per joint, raw and after removing the lag (what is left is
    overshoot / steady offset, i.e. what the gains have to fix),
  - ringing: error content above 3 Hz (too little damping / too much stiffness),
  - overshoot: how far the Panda runs past GELLO's own peak when a fast motion stops.

Errors are measured against the GELLO samples interpolated linearly (the operator's hand
motion), not against the stair-stepped held command: at low GELLO rates the steps would
dominate the error. The step size is reported separately as "staircase RMS".

Compare runs while changing the controller yaml (the proxy re-reads it on mode start).

Usage:
    python examples/check_teleop_tracking.py --duration 30
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np


class Recorder:
    """Records every message of a topic with its local receive time."""

    def __init__(self, topic: str) -> None:
        import pyzlc

        self.times: list[float] = []
        self.msgs: list[dict] = []
        self.recording = False
        self._lock = threading.Lock()
        pyzlc.register_subscriber_handler(topic, self._handle)

    def _handle(self, msg: dict) -> None:
        if self.recording:
            with self._lock:
                self.times.append(time.perf_counter())
                self.msgs.append(msg)


def gello_samples(rec: Recorder) -> tuple[np.ndarray, np.ndarray, dict]:
    """Distinct GELLO samples (receive time, joints) and sample-rate statistics."""
    t = np.asarray(rec.times)
    q = np.asarray([m["joint_state"] for m in rec.msgs], dtype=float)
    if "seq" in rec.msgs[0]:
        seq = np.asarray([m["seq"] for m in rec.msgs])
        keep = np.concatenate([[True], np.diff(seq) != 0])
        stamps = np.asarray([m["stamp"] for m in rec.msgs])[keep]
        intervals = np.diff(stamps)
        missed = int(np.sum(np.diff(seq[keep]) - 1))
        source = "seq"
    else:
        # old gello node republishes in a busy loop: count value changes instead (a still
        # GELLO looks like a missing sample, so this underestimates the rate when idle)
        keep = np.concatenate([[True], np.any(np.diff(q, axis=0) != 0, axis=1)])
        intervals = np.diff(t[keep])
        missed = 0
        source = "value changes"
    stats = {
        "messages_per_s": float(len(t) / (t[-1] - t[0])),
        "samples_per_s": float(1.0 / np.median(intervals)) if len(intervals) else float("nan"),
        "interval_ms_median": float(1e3 * np.median(intervals)) if len(intervals) else float("nan"),
        "interval_ms_p95": float(1e3 * np.percentile(intervals, 95)) if len(intervals) else float("nan"),
        "samples_skipped": missed,
        "rate_source": source,
    }
    return t[keep], q[keep], stats


def resample_hold(t_src: np.ndarray, x_src: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Zero-order hold, i.e. what the controller sees between commands."""
    idx = np.clip(np.searchsorted(t_src, t, side="right") - 1, 0, len(t_src) - 1)
    return x_src[idx]


def sample_velocity(t_src: np.ndarray, x_src: np.ndarray) -> np.ndarray:
    """Max |velocity| per joint between consecutive GELLO samples (not of the held signal)."""
    if len(t_src) < 2:
        return np.zeros(x_src.shape[1])
    v = np.diff(x_src, axis=0) / np.maximum(np.diff(t_src), 1e-3)[:, None]
    return np.max(np.abs(v), axis=0)


RINGING_CUTOFF_HZ = 3.0


def highpass(x: np.ndarray, dt: float, cutoff_hz: float) -> np.ndarray:
    """Zero-phase FFT high-pass along axis 0 (mean removed)."""
    X = np.fft.rfft(x - x.mean(axis=0), axis=0)
    X[np.fft.rfftfreq(len(x), dt) < cutoff_hz] = 0
    return np.fft.irfft(X, len(x), axis=0)


def stop_overshoot(t: np.ndarray, ref: np.ndarray, q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per joint: how far the Panda runs past GELLO's own peak when a motion stops.

    A stop is the smoothed GELLO speed falling below 3 deg/s after exceeding 15 deg/s. Measured
    against GELLO's peak (not its final value) so the operator's own overshoot does not count.
    Returns number of stops, median and max overshoot [rad] (nan without stops).
    """
    from scipy.ndimage import uniform_filter1d

    dt = float(t[1] - t[0])
    vs = np.gradient(uniform_filter1d(ref, max(1, int(0.04 / dt)), axis=0), dt, axis=0)
    after = int(0.4 / dt)
    n, med, mx = np.zeros(7, int), np.full(7, np.nan), np.full(7, np.nan)
    for j in range(7):
        sp, over, i = np.abs(vs[:, j]), [], 0
        while i < len(t) - after:
            if sp[i] <= np.radians(15):
                i += 1
                continue
            slow = np.flatnonzero(sp[i:] < np.radians(3))
            if not len(slow) or i + slow[0] + after >= len(t):
                break
            k = i + slow[0]
            sgn, w = np.sign(vs[i, j]), slice(max(i, k - after // 4), k + after)
            over.append(np.max(sgn * q[w, j]) - np.max(sgn * ref[w, j]))
            i = k + after
        if over:
            n[j], med[j], mx[j] = len(over), np.median(over), np.max(over)
    return n, med, mx


def analyse(t: np.ndarray, cmd: np.ndarray, q: np.ndarray, max_lag_s: float = 0.4) -> dict:
    dt = float(t[1] - t[0])
    moving = np.std(cmd, axis=0) > 0.02  # joints that were actually moved
    err = q - cmd
    lags, err_comp = np.zeros(7), np.zeros_like(err)
    for j in range(7):
        best, best_err = 0, np.inf
        for k in range(int(max_lag_s / dt) + 1):
            e = np.mean((q[k:, j] - cmd[: len(cmd) - k, j]) ** 2)
            if e < best_err:
                best, best_err = k, e
        lags[j] = best * dt
        e = q[best:, j] - cmd[: len(cmd) - best, j]
        err_comp[: len(e), j] = e
        err_comp[len(e) :, j] = np.nan
    n_stops, over_med, over_max = stop_overshoot(t, cmd, q)
    return {
        "stops": n_stops.tolist(),
        "overshoot_median_deg": np.degrees(over_med).tolist(),
        "overshoot_max_deg": np.degrees(over_max).tolist(),
        "moving": moving.tolist(),
        "panda_moving": (np.std(q, axis=0) > 0.005).tolist(),
        "lag_ms": (1e3 * lags).tolist(),
        "rms_err_deg": np.degrees(np.sqrt(np.mean(err**2, axis=0))).tolist(),
        "max_err_deg": np.degrees(np.max(np.abs(err), axis=0)).tolist(),
        "mean_err_deg": np.degrees(np.mean(err, axis=0)).tolist(),
        "rms_err_lag_comp_deg": np.degrees(np.sqrt(np.nanmean(err_comp**2, axis=0))).tolist(),
        "ringing_rms_deg": np.degrees(np.sqrt(np.mean(highpass(err, dt, RINGING_CUTOFF_HZ) ** 2, axis=0))).tolist(),
        "_err_comp": err_comp,
    }


def plot(t, cmd, q, res, stats, out_png: Path, show: bool) -> None:
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(7, 2, figsize=(15, 16), sharex=True)
    fig.suptitle(
        f"GELLO {stats['samples_per_s']:.0f} samples/s (median interval {stats['interval_ms_median']:.1f} ms, "
        f"p95 {stats['interval_ms_p95']:.1f} ms)  |  lag per joint [ms]: "
        + ", ".join(f"{v:.0f}" for v in res["lag_ms"])
    )
    for j in range(7):
        a = axes[j, 0]
        a.plot(t, np.degrees(cmd[:, j]), "--", lw=1, label="GELLO")
        a.plot(t, np.degrees(q[:, j]), lw=1, label="Panda")
        a.set_ylabel(f"j{j + 1} [deg]")
        a.grid(True)
        a = axes[j, 1]
        a.plot(t, np.degrees(q[:, j] - cmd[:, j]), lw=0.8, label="raw")
        a.plot(t, np.degrees(res["_err_comp"][:, j]), lw=0.8, label=f"lag-comp. ({res['lag_ms'][j]:.0f} ms)")
        a.set_ylabel("err [deg]")
        a.grid(True)
        a.legend(loc="upper right", fontsize="small")
    axes[0, 0].legend(loc="upper right")
    axes[0, 0].set_title("Joint position")
    axes[0, 1].set_title("Panda - GELLO")
    axes[-1, 0].set_xlabel("t [s]")
    axes[-1, 1].set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    print(f"Plot written to {out_png}")
    if show:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-name", default="FrankaPanda")
    parser.add_argument("--gello-name", default="gello")
    parser.add_argument("--local-ip", default="192.168.1.1")
    parser.add_argument("--group-name", default="DroidGroup")
    parser.add_argument("--group-port", type=int, default=7730)
    parser.add_argument("--duration", type=float, default=30.0, help="Recording length [s]")
    parser.add_argument("--rate", type=float, default=1000.0, help="Resampling rate for the analysis [Hz]")
    parser.add_argument("--no-show", action="store_true", help="Only save the plot, do not open a window")
    parser.add_argument("--output", type=Path, default=Path("teleop_tracking"), help="Output prefix (.png, .json)")
    args = parser.parse_args()

    import pyzlc

    pyzlc.init("teleop_tracking_check", args.local_ip, group_name=args.group_name, group_port=args.group_port)
    gello = Recorder(f"{args.gello_name}/gello_arm_state")
    panda = Recorder(f"{args.robot_name}/franka_arm_state")

    input("Start teleoperation, then hit Enter and move GELLO (slow and fast, all joints) ...")
    gello.recording = panda.recording = True
    t_end = time.perf_counter() + args.duration
    while time.perf_counter() < t_end:
        print(f"\r  recording ... {t_end - time.perf_counter():4.1f} s left", end="", flush=True)
        time.sleep(0.2)
    gello.recording = panda.recording = False
    print()

    if len(gello.msgs) < 10 or len(panda.msgs) < 10:
        raise RuntimeError(f"Too few messages: GELLO {len(gello.msgs)}, Panda {len(panda.msgs)}")

    t_g, q_g, stats = gello_samples(gello)
    t_p = np.asarray(panda.times)
    q_p = np.asarray([m["q"] for m in panda.msgs], dtype=float)
    t0, t1 = max(t_g[0], t_p[0]), min(t_g[-1], t_p[-1])
    t = np.arange(t0, t1, 1.0 / args.rate)
    cmd = resample_hold(t_g, q_g, t)
    ref = np.stack([np.interp(t, t_g, q_g[:, j]) for j in range(7)], axis=1)
    q = np.stack([np.interp(t, t_p, q_p[:, j]) for j in range(7)], axis=1)
    res = analyse(t - t0, ref, q)
    res["staircase_rms_deg"] = np.degrees(np.sqrt(np.mean((cmd - ref) ** 2, axis=0))).tolist()
    in_window = (t_g >= t0) & (t_g <= t1)
    res["max_cmd_vel_deg_s"] = np.degrees(sample_velocity(t_g[in_window], q_g[in_window])).tolist()

    def row(name, vals, fmt="{:7.1f}"):
        print(f"{name:<26}" + "".join(fmt.format(v) for v in vals))

    print("\n=== GELLO stream ===")
    print(
        f"{stats['samples_per_s']:.0f} new samples/s (median interval {stats['interval_ms_median']:.1f} ms, "
        f"p95 {stats['interval_ms_p95']:.1f} ms, from {stats['rate_source']}), "
        f"{stats['messages_per_s']:.0f} messages/s, {stats['samples_skipped']} samples skipped"
    )
    print(f"Panda state: {len(t_p) / (t_p[-1] - t_p[0]):.0f} messages/s")
    if any(res["moving"]) and not any(res["panda_moving"]):
        print(
            "\n!!! GELLO moved but the Panda did not: teleop was not active during the recording.\n"
            "    In data collection the control pair only runs while an episode is recording;\n"
            "    also check the data-collection console for 'Control task encountered an error'."
        )
    print("\n=== Tracking ===")
    row("joint", range(1, 8), "{:7d}")
    row("GELLO moved", ["yes" if m else "no" for m in res["moving"]], "{:>7}")
    row("Panda moved", ["yes" if m else "no" for m in res["panda_moving"]], "{:>7}")
    row("max cmd velocity [deg/s]", res["max_cmd_vel_deg_s"])
    row("lag [ms]", res["lag_ms"], "{:7.0f}")
    row("RMS error [deg]", res["rms_err_deg"], "{:7.2f}")
    row("max error [deg]", res["max_err_deg"], "{:7.2f}")
    row("mean error [deg]", res["mean_err_deg"], "{:7.2f}")
    row("RMS lag-comp. [deg]", res["rms_err_lag_comp_deg"], "{:7.2f}")
    row(f"ringing >{RINGING_CUTOFF_HZ:.0f} Hz RMS [deg]", res["ringing_rms_deg"], "{:7.2f}")
    row("stops (>15 deg/s)", res["stops"], "{:7d}")
    row("overshoot median [deg]", res["overshoot_median_deg"], "{:7.2f}")
    row("overshoot max [deg]", res["overshoot_max_deg"], "{:7.2f}")
    row("staircase RMS [deg]", res["staircase_rms_deg"], "{:7.2f}")
    print("Lag/RMS of joints that were not moved are meaningless.")
    print(
        "Lag is measured against the GELLO samples as they arrive at the robot, interpolated. It includes "
        f"~{stats['interval_ms_median'] / 2:.0f} ms of holding each sample (half a GELLO interval); the bus "
        "read time comes on top for the operator."
    )

    out_json = args.output.with_suffix(".json")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(
        json.dumps(
            {
                "gello": stats,
                "tracking": {k: v for k, v in res.items() if not k.startswith("_")},
                "t": (t - t0).tolist(),
                "gello_q": cmd.tolist(),
                "gello_ref": ref.tolist(),
                "panda_q": q.tolist(),
            }
        )
    )
    print(f"Raw data written to {out_json}")
    plot(t - t0, ref, q, res, stats, args.output.with_suffix(".png"), show=not args.no_show)


if __name__ == "__main__":
    main()
