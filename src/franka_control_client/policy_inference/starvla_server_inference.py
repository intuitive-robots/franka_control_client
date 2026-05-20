# =============================================================================
# SETUP
#
# On the robot PC (franka_control_client env):
#   pip install websockets msgpack scipy opencv-python
#
# HOW TO RUN EVAL
#
# Step 1 — start the policy server on the GPU node (starVLA repo):
#   cd starVLA/
#   python deployment/model_server/server_policy.py \
#       --ckpt_path /path/to/checkpoint.pt \
#       --port 10093 \
#       --use_bf16
#   Check logs for available_unnorm_keys and action_chunk_size.
#
# Step 2 — smoke test (no robot, just verifies server connection + action shape):
#   cd starVLA/
#   python franka_control_client/src/franka_control_client/policy_inference/starvla_server_inference.py \
#       --host <server_ip> --port 10093 \
#       --task "put red cylinder on green cube" \
#       --n_cameras 2 --n_infer 3
#   Expected output: actions shape=(24, 8) and "Smoke test PASSED".
#   Add --include_state only if model was trained with include_state: true.
#
# Step 3 — deploy on robot:
#   cd starVLA/
#   python franka_control_client/examples/policy_inference_202_robotiq_starvla_cartesian.py
#   Set SERVER_HOST in that script to the GPU node IP if on different machines.
#   Keyboard: n=start episode  s=stop  d=discard  r=reset arm  q=quit
# =============================================================================

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import pyzlc
from scipy.spatial.transform import Rotation as R

if __package__ in (None, ""):
    # Support `python src/franka_control_client/.../starvla_server_inference.py`.
    # Adding src/ lets sibling relative imports resolve through the package.
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from franka_control_client.policy_inference.websocket_policy_client import (
        WebsocketClientPolicy,
    )
else:
    from .websocket_policy_client import WebsocketClientPolicy

if __package__ in (None, ""):
    # Direct execution is only used for the smoke test below; robot hardware
    # classes are not needed until the package is imported by the robot example.
    PolicyInferenceManager = object  # type: ignore[assignment,misc]
else:
    try:
        from ..control_pair.cartesian_policy_panda_control_pair import (
            PolicyPandaRobotiqDeltaCartesianControlPair,
        )
        from ..data_collection.irl_wrapper import (
            IRLDataWrapper,
            ImageDataWrapper,
            PandaArmDataWrapper,
            RobotiqGripperDataWrapper,
        )
        from .policy_inference_manager import PolicyInferenceManager
    except ImportError:
        # Allows importing this module in lightweight smoke-test environments.
        PolicyInferenceManager = object  # type: ignore[assignment,misc]


class StarVLAServerInference(PolicyInferenceManager):
    """
    Policy inference loop that queries a remote starVLA policy server over WebSocket.

    Action format (8D absolute EEF):
      abs_eef_position(3) + abs_eef_rotation_quat(4) + gripper_close(1)

    State is optional. When include_state=True, a 7D vector is passed:
      eef_position(3) + eef_rotation_euler(3) + gripper_close(1)
    Pass include_state=False when the model was trained without state
    (include_state: false / state_dim: 0 in the training config).

    The server returns already-unnormalized absolute EEF actions.
    These are fed directly to control_pair.update_action() which sends
    absolute cartesian pose commands to the robot (no delta conversion).

    Action chunking is handled client-side: the server is queried once
    every action_chunk_size steps; intermediate steps replay the cache.
    """

    def __init__(
        self,
        data_collectors: List[IRLDataWrapper],
        control_pair: PolicyPandaRobotiqDeltaCartesianControlPair,
        task: str,
        fps: int = 10,
        host: str = "127.0.0.1",
        port: int = 10093,
        unnorm_key: Optional[str] = None,
        image_size: Tuple[int, int] = (224, 224),
        include_state: bool = False,
    ) -> None:
        super().__init__(task=task, fps=fps)
        self.control_pair = control_pair
        self.image_size = image_size
        self.unnorm_key = unnorm_key
        self.include_state = include_state

        self.cameras: List[ImageDataWrapper] = []
        self.arm_wrapper: Optional[PandaArmDataWrapper] = None
        self.gripper_wrapper: Optional[IRLDataWrapper] = None
        for hw in data_collectors:
            if isinstance(hw, ImageDataWrapper) or hw.hw_type == "camera":
                self.cameras.append(hw)
            elif isinstance(hw, PandaArmDataWrapper) or hw.hw_type == "follower_arm":
                self.arm_wrapper = hw
            elif (
                isinstance(hw, RobotiqGripperDataWrapper)
                or hw.hw_type == "follower_gripper"
            ):
                self.gripper_wrapper = hw

        if include_state and self.arm_wrapper is None:
            raise ValueError("include_state=True but no PandaArmDataWrapper found in data_collectors.")
        if include_state and self.gripper_wrapper is None:
            raise ValueError("include_state=True but no gripper wrapper found in data_collectors.")

        pyzlc.info(f"Connecting to starVLA server at ws://{host}:{port} ...")
        self.client = WebsocketClientPolicy(host=host, port=port)
        server_meta = self.client.get_server_metadata()
        self.action_chunk_size: int = int(server_meta["action_chunk_size"])
        pyzlc.info(
            "Connected to starVLA server. metadata=%s, action_chunk_size=%d",
            server_meta,
            self.action_chunk_size,
        )

        self._action_chunk: Optional[np.ndarray] = None  # (T, 8) cached chunk
        self._chunk_step: int = 0

        self.register_start_infering_event(self.control_pair.start_control_pair)
        self.register_stop_infering_event(self.control_pair.stop_control_pair)

    # ------------------------------------------------------------------
    # Observation building
    # ------------------------------------------------------------------

    def _build_images(self) -> List[np.ndarray]:
        """Capture and resize camera frames. Order: primary (static), then wrist."""
        images: List[np.ndarray] = []
        for cam in self.cameras:
            frame = cam.capture_step()
            if frame is None:
                pyzlc.warning("Camera %s returned None frame, skipping.", cam.hw_name)
                continue
            if isinstance(frame, np.ndarray):
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            elif isinstance(frame, dict) and "rgb_data" in frame:
                h, w, c = int(frame["height"]), int(frame["width"]), int(frame.get("channels", 3))
                frame = np.frombuffer(frame["rgb_data"], dtype=np.uint8).reshape(h, w, c).copy()
            frame = cv2.resize(frame, self.image_size, interpolation=cv2.INTER_AREA)
            images.append(frame)
        return images

    def _build_state(self) -> Optional[np.ndarray]:
        """Build 7D state: eef_pos(3) + eef_rot_euler(3) + gripper_close(1). None if include_state=False."""
        if not self.include_state:
            return None
        arm_state = self.arm_wrapper.capture_step()
        ee_pos = np.asarray(arm_state[0], dtype=np.float32).reshape(3)
        ee_quat = np.asarray(arm_state[1], dtype=np.float32).reshape(4)  # [qx,qy,qz,qw]
        ee_euler = R.from_quat(ee_quat).as_euler("xyz", degrees=False).astype(np.float32)
        grip_state = self.gripper_wrapper.capture_step()
        gripper_close = self._parse_gripper_close(grip_state)
        return np.concatenate([ee_pos, ee_euler, [gripper_close]], dtype=np.float32)

    @staticmethod
    def _parse_gripper_close(grip_state) -> float:
        if isinstance(grip_state, dict):
            if "position" in grip_state:
                return float(grip_state["position"])
            if "width" in grip_state:
                return float(np.clip(1.0 - grip_state["width"], 0.0, 1.0))
            if "gripper_close" in grip_state:
                return float(grip_state["gripper_close"])
        return 0.0

    # ------------------------------------------------------------------
    # Core inference step
    # ------------------------------------------------------------------

    def _infer_step(self) -> None:
        start = time.perf_counter()

        if self._action_chunk is None or self._chunk_step >= self.action_chunk_size:
            images = self._build_images()
            state = self._build_state()
            pyzlc.info("Querying server | include_state=%s", self.include_state)

            example: dict = {"image": images, "lang": self.task}
            if state is not None:
                example["state"] = state[None]  # (1, 7)
            payload: dict = {"examples": [example], "unnorm_key": self.unnorm_key}

            response = self.client.predict_action(payload)
            if not response.get("ok", False):
                pyzlc.error("Server inference failed: %s", response.get("error", {}).get("message", "unknown"))
                return

            self._action_chunk = np.asarray(response["data"]["actions"][0], dtype=np.float32)  # (T, 8)
            self._chunk_step = 0
            pyzlc.info("Received chunk shape=%s  chunk[0]=%s", self._action_chunk.shape, self._action_chunk[0])

        action = self._action_chunk[self._chunk_step]  # (8,) absolute EEF
        self._chunk_step += 1
        pyzlc.info("Applying step %d/%d: %s", self._chunk_step, self.action_chunk_size, action)

        try:
            # size-8 input → update_action treats it as absolute (no delta conversion)
            self.control_pair.update_action(action)
        except Exception as exc:
            pyzlc.error("Failed to apply action: %s", exc)

        elapsed = time.perf_counter() - start
        sleep_time = max(0.0, (1.0 / self.fps) - elapsed)
        if sleep_time > 0.001:
            time.sleep(sleep_time)

    # ------------------------------------------------------------------
    # State machine hooks
    # ------------------------------------------------------------------

    def _start_infering(self) -> None:
        self._action_chunk = None
        self._chunk_step = 0
        self.control_pair.reset_action()
        super()._start_infering()

    def _save_episode(self) -> None:
        self._stop_infering()
        self._ui_console.log("Episode saved.")

    def _discard_infering(self) -> None:
        self._stop_infering()
        self._ui_console.log("Episode discarded.")

    def _stop_infering(self) -> None:
        super()._stop_infering()

    def _reset_arm(self) -> None:
        self._ui_console.log("Resetting robot arm to home position...")
        try:
            self.control_pair.go_home()
            time.sleep(3)
            self._ui_console.log("Arm reset complete.")
        except Exception as exc:
            self._ui_console.log(f"Failed to reset arm: {exc}")

    def _close(self) -> None:
        try:
            self.client.close()
        except Exception:
            pass
        super()._close()


# ---------------------------------------------------------------------------
# Smoke test — no robot hardware required
# Run directly to verify server connection and action shape before deploying.
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    import logging

    def _build_argparser() -> argparse.ArgumentParser:
        ap = argparse.ArgumentParser(
            description="Smoke test for starVLA policy server (synthetic obs, no hardware)",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        )
        ap.add_argument("--host", default="127.0.0.1")
        ap.add_argument("--port", type=int, default=10093)
        ap.add_argument("--task", default="pick up the red block")
        ap.add_argument("--unnorm_key", default=None)
        ap.add_argument("--image_size", type=int, default=224)
        ap.add_argument("--n_cameras", type=int, default=2)
        ap.add_argument("--include_state", action="store_true")
        ap.add_argument("--n_infer", type=int, default=1)
        ap.add_argument("--log_level", default="INFO")
        return ap

    args = _build_argparser().parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(message)s",
        force=True,
    )

    logging.info("Connecting to ws://%s:%d ...", args.host, args.port)
    client = WebsocketClientPolicy(host=args.host, port=args.port)
    meta = client.get_server_metadata()
    logging.info("Server metadata: %s", meta)
    logging.info(
        "action_chunk_size=%d  available_unnorm_keys=%s  default=%s",
        meta.get("action_chunk_size", -1),
        meta.get("available_unnorm_keys"),
        meta.get("default_unnorm_key"),
    )

    latencies = []
    for i in range(args.n_infer):
        images = [
            np.random.randint(0, 256, (args.image_size, args.image_size, 3), dtype=np.uint8)
            for _ in range(args.n_cameras)
        ]
        example: dict = {"image": images, "lang": args.task}
        if args.include_state:
            example["state"] = np.zeros((1, 7), dtype=np.float32)
        payload: dict = {"examples": [example], "unnorm_key": args.unnorm_key}

        t0 = time.perf_counter()
        response = client.predict_action(payload)
        latency = time.perf_counter() - t0
        latencies.append(latency)

        if not response.get("ok", False):
            logging.error("Call %d FAILED: %s", i + 1, response.get("error"))
            client.close()
            sys.exit(1)

        actions = np.asarray(response["data"]["actions"][0])  # (T, D)
        logging.info("Call %d/%d OK — shape=%s  latency=%.3fs", i + 1, args.n_infer, actions.shape, latency)
        logging.info("  chunk[0] : %s", np.round(actions[0], 4))
        if actions.shape[-1] >= 8:
            logging.info(
                "  → abs_pos=%s  quat=%s  gripper=%.3f",
                np.round(actions[0, :3], 4),
                np.round(actions[0, 3:7], 4),
                float(actions[0, 7]),
            )

    if args.n_infer > 1:
        logging.info(
            "Timing — mean=%.3fs  min=%.3fs  max=%.3fs",
            float(np.mean(latencies)), float(np.min(latencies)), float(np.max(latencies)),
        )

    client.close()
    logging.info("Smoke test PASSED.")
