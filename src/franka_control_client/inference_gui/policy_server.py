"""Policy-PC side: serve any policy over the socket protocol used by
:class:`franka_control_client.policy_inference.policy_server_client.PolicyServerClient`.

Wire format: 8-byte big-endian length + pickled dict, request/response.

Requests  (robot PC -> policy PC)
    {"type": "health"}
    {"type": "reset", "task": str | None}
    {"type": "infer", "observation": {"observation.state": np.ndarray,
                                      "observation.images.<cam>": HxWx3 uint8 RGB,
                                      "task": str, ...}}
    {"type": "close"}

Responses (policy PC -> robot PC), always {"ok": bool, ...}
    health -> action_space, fps, exec_steps, policy metadata
    infer  -> action [T, D] (absolute targets, gripper closedness last),
              action_space, exec_steps, server_infer_s (pure model time)
    error  -> {"ok": False, "error": "..."}

Usage on the policy PC:

    # smoke test without a model
    python -m franka_control_client.inference_gui.policy_server --policy dummy --action-space joint

    # your own policy: any class implementing BasePolicy
    python -m franka_control_client.inference_gui.policy_server \
        --policy my_package.my_module:MyPolicy --policy-kwargs '{"ckpt": "/path"}'
"""

from __future__ import annotations

import argparse
import importlib
import json
import logging
import socket
import threading
import time
import traceback
from typing import Any, Dict, Optional

import numpy as np

from franka_control_client.policy_inference.policy_server_client import (
    receive_message,
    send_message,
)

LOG = logging.getLogger("policy_server")

JOINT_ACTION_SPACE = "joint_pos_abs_gripper_closedness"
CARTESIAN_ACTION_SPACE = "ee_pose_abs_quat_gripper_closedness"
ACTION_SPACES = {"joint": JOINT_ACTION_SPACE, "cartesian": CARTESIAN_ACTION_SPACE}


class BasePolicy:
    """Implement this on the policy PC.

    ``infer`` gets the observation dict built by the robot PC and returns an
    action chunk ``[T, 8]`` with rows spaced ``1 / fps`` seconds apart:

    * joint     : [q0..q6, gripper_closedness]          (absolute, rad)
    * cartesian : [x, y, z, qx, qy, qz, qw, gripper]    (absolute, base frame)

    gripper_closedness: 0 = open, 1 = closed.
    """

    action_space: str = JOINT_ACTION_SPACE
    fps: float = 15.0
    exec_steps: int = 8  # rows the robot executes before the next request

    def metadata(self) -> Dict[str, Any]:
        return {}

    def reset(self, task: Optional[str]) -> None:
        """Called at the start of every rollout."""

    def infer(self, observation: Dict[str, Any]) -> np.ndarray:
        raise NotImplementedError


class DummyPolicy(BasePolicy):
    """Model-free test policy: a gentle oscillation around the pose seen at
    the first observation of each rollout. Safe-ish amplitudes, but keep a hand
    on the e-stop when trying it on the real robot."""

    def __init__(
        self,
        action_space: str = "joint",
        fps: float = 15.0,
        chunk_len: int = 16,
        exec_steps: int = 8,
        amplitude: float = 0.15,
        period_s: float = 6.0,
        latency_s: float = 0.05,
    ) -> None:
        self.action_space = ACTION_SPACES.get(action_space, action_space)
        self.fps = float(fps)
        self.chunk_len = int(chunk_len)
        self.exec_steps = int(exec_steps)
        self.amplitude = float(amplitude)
        self.period_s = float(period_s)
        self.latency_s = float(latency_s)
        self._anchor: Optional[np.ndarray] = None
        self._t0 = time.time()

    def metadata(self) -> Dict[str, Any]:
        return {"name": "dummy", "chunk_len": self.chunk_len, "amplitude": self.amplitude}

    def reset(self, task: Optional[str]) -> None:
        self._anchor = None
        self._t0 = time.time()

    def infer(self, observation: Dict[str, Any]) -> np.ndarray:
        state = np.asarray(observation["observation.state"], dtype=np.float64).reshape(-1)
        if self._anchor is None:
            self._anchor = state.copy()
        time.sleep(self.latency_s)  # emulate model compute time
        t = (time.time() - self._t0) + np.arange(1, self.chunk_len + 1) / self.fps
        phase = np.sin(2 * np.pi * t / self.period_s)
        chunk = np.repeat(self._anchor[None, :8], self.chunk_len, axis=0)
        if self.action_space == JOINT_ACTION_SPACE:
            chunk[:, 0] += self.amplitude * phase
            chunk[:, 3] += 0.5 * self.amplitude * (1 - np.cos(2 * np.pi * t / self.period_s))
        else:
            r = 0.2 * self.amplitude  # metres
            chunk[:, 0] += r * phase
            chunk[:, 1] += r * (np.cos(2 * np.pi * t / self.period_s) - 1)
        chunk[:, 7] = (np.sin(2 * np.pi * t / (2 * self.period_s)) > 0).astype(np.float64)
        return chunk


def load_policy(spec: str, kwargs: Dict[str, Any]) -> BasePolicy:
    if spec == "dummy":
        return DummyPolicy(**kwargs)
    if ":" not in spec:
        raise ValueError("--policy must be 'dummy' or 'package.module:ClassName'")
    module_name, class_name = spec.split(":", 1)
    cls = getattr(importlib.import_module(module_name), class_name)
    return cls(**kwargs)


class PolicyServer:
    def __init__(self, policy: BasePolicy, host: str = "0.0.0.0", port: int = 8767) -> None:
        self.policy = policy
        self.host = host
        self.port = int(port)
        self._policy_lock = threading.Lock()
        self._sock: Optional[socket.socket] = None
        self._running = False

    # ------------------------------------------------------------------ api
    def serve_forever(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(4)
        self._running = True
        LOG.info("Policy server listening on %s:%d (%s)", self.host, self.port, self.policy.action_space)
        try:
            while self._running:
                try:
                    conn, addr = self._sock.accept()
                except OSError:
                    break
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                threading.Thread(target=self._serve_client, args=(conn, addr), daemon=True).start()
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self._running = False
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    # ------------------------------------------------------------ internals
    def _serve_client(self, conn: socket.socket, addr: Any) -> None:
        LOG.info("Client connected: %s", addr)
        try:
            with conn:
                while True:
                    try:
                        request = receive_message(conn)
                    except (ConnectionError, OSError):
                        break
                    response = self._handle(request)
                    send_message(conn, response)
                    if request.get("type") == "close":
                        break
        finally:
            LOG.info("Client disconnected: %s", addr)

    def _handle(self, request: Dict[str, Any]) -> Dict[str, Any]:
        kind = request.get("type")
        try:
            if kind == "health":
                return {
                    "ok": True,
                    "action_space": self.policy.action_space,
                    "fps": self.policy.fps,
                    "exec_steps": self.policy.exec_steps,
                    "policy": self.policy.metadata(),
                }
            if kind == "reset":
                with self._policy_lock:
                    self.policy.reset(request.get("task"))
                return {"ok": True}
            if kind == "infer":
                t_recv = time.time()
                with self._policy_lock:
                    start = time.perf_counter()
                    action = self.policy.infer(request["observation"])
                    infer_s = time.perf_counter() - start
                action = np.asarray(action, dtype=np.float64)
                return {
                    "ok": True,
                    "action": action,
                    "action_space": self.policy.action_space,
                    "exec_steps": self.policy.exec_steps,
                    "server_infer_s": infer_s,
                    "server_recv_time": t_recv,
                }
            if kind == "close":
                return {"ok": True}
            return {"ok": False, "error": f"Unknown request type {kind!r}"}
        except Exception as exc:  # report to the robot PC instead of dying
            LOG.error("Request %r failed:\n%s", kind, traceback.format_exc())
            return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--policy", default="dummy", help="'dummy' or 'package.module:ClassName'")
    parser.add_argument("--policy-kwargs", default="{}", help="JSON dict passed to the policy constructor")
    parser.add_argument("--action-space", choices=sorted(ACTION_SPACES), default=None,
                        help="shortcut for the dummy policy's action_space")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    kwargs = json.loads(args.policy_kwargs)
    if args.action_space is not None:
        if args.policy != "dummy":
            parser.error("--action-space only applies to the dummy policy")
        kwargs["action_space"] = args.action_space
    policy = load_policy(args.policy, kwargs)
    server = PolicyServer(policy, host=args.host, port=args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
