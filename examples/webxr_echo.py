"""WebXR teleop echo: no robot. Prints the mapped EE target, clutch state, rate and latency.

Open http://localhost:8000 in the Quest Browser (after `adb reverse tcp:8000 tcp:8000`
on the host), press "Enter XR", hold the right grip and move the controller.
"""

import argparse
import logging
import time

import numpy as np

from franka_control_client.vr.webxr_controller import WebXRController, WorkspaceBounds
from franka_control_client.vr.webxr_server import WebXRServer

# DROID-like start pose, gripper pointing down
FAKE_EE_POS = [0.45, 0.0, 0.35]
FAKE_EE_QUAT = [1.0, 0.0, 0.0, 0.0]
POLL_HZ = 500.0


class FakeArm:
    """Stands in for RemotePandaArm: the EE sits wherever it was last commanded."""

    def __init__(self) -> None:
        self.current_state = {"EE_pos": list(FAKE_EE_POS), "EE_quat": list(FAKE_EE_QUAT)}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="WebXR teleop echo (no robot)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--yaw", type=float, default=0.0, help="Operator heading in the base frame, deg CCW from +X")
    parser.add_argument("--rate", type=float, default=10.0, help="Print rate (Hz)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    arm = FakeArm()
    server = WebXRServer(args.host, args.port)
    ctrl = WebXRController("webxr", arm, server=server, yaw_deg=args.yaw, bounds=WorkspaceBounds())
    print(f"Open http://localhost:{args.port} in the Quest Browser. Ctrl-C to quit.")
    next_print = time.monotonic()
    try:
        while True:
            # poll like the control loop does: the step cap applies per processed sample
            time.sleep(1.0 / POLL_HZ)
            sig = ctrl.current_control_signal
            # emulate perfect tracking, so a re-clutch starts where the target is
            arm.current_state = {"EE_pos": sig["pos"], "EE_quat": sig["rot"]}
            if time.monotonic() < next_print:
                continue
            next_print += 1.0 / args.rate
            snap = server.snapshot()
            stats = server.stats()
            s = ctrl.last_sample
            age = f"{(time.monotonic() - s.recv_t) * 1e3:5.1f}" if s is not None else "  n/a"
            rtt = f"{stats.rtt_ms:5.1f}" if stats.rtt_ms is not None else "  n/a"
            delta = np.asarray(sig["pos"]) - FAKE_EE_POS
            clutch = "ENGAGED " if ctrl.on_control else f"free({ctrl.declutch_reason or '-'})"
            fresh = s is not None and snap.connected
            buttons = (" A" if fresh and s.a else "") + (" B" if fresh and s.b else "") + (" STICK" if fresh and s.stick else "")
            print(
                f"{'ws' if snap.connected else 'NO WS'} {clutch:<26} "
                f"target {np.round(sig['pos'], 3)} (Δ from start {np.round(delta, 3)}) "
                f"grip_cmd {sig['gripper_width']:.2f} | {stats.hz:5.1f} Hz p95 {stats.interarrival_p95_ms:5.1f} ms "
                f"age {age} ms RTT {rtt} ms align {np.rad2deg(ctrl.heading_xr):+4.0f} deg{buttons}",
                flush=True,
            )
    except KeyboardInterrupt:
        server.stop()


if __name__ == "__main__":
    main()
