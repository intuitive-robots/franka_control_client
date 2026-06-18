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

import json
import re
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import pyzlc
from scipy.spatial.transform import Rotation as R

from .websocket_policy_client import WebsocketClientPolicy

try:
    # Relative imports only available when used as a module (not run directly).
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
    # Running as __main__ for smoke testing — hardware classes not needed.
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
        if not server_meta.get("cot_generate_at_inference", False):
            pyzlc.warning(
                "Server metadata reports cot_generate_at_inference=false; explicit CoT text may be unavailable."
            )

        self._action_chunk: Optional[np.ndarray] = None  # (T, 8) cached chunk
        self._chunk_step: int = 0
        self._latest_cot_text: Optional[str] = None
        self._latest_trace_points: List[Tuple[float, float]] = []
        self._latest_obs_images: List[np.ndarray] = []
        self._prediction_count: int = 0
        self._last_prediction_ts: Optional[float] = None
        self._viz_window_name = "StarVLA CoT Viz"

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
        self._latest_obs_images = [np.array(image, copy=True) for image in images]
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
            self._update_cot_state(response.get("data", {}))
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
        self._render_visualization()
        sleep_time = max(0.0, (1.0 / self.fps) - elapsed)
        if sleep_time > 0.001:
            time.sleep(sleep_time)

    def _update_cot_state(self, response_data: dict) -> None:
        self._prediction_count += 1
        self._last_prediction_ts = time.time()
        self._latest_trace_points = []

        cot_text = response_data.get("cot_text")
        if isinstance(cot_text, list):
            cot_text = cot_text[0] if cot_text else None
        if not isinstance(cot_text, str):
            self._latest_cot_text = None
            return

        cot_text = " ".join(cot_text.split())
        self._latest_trace_points = self._extract_trace_points(cot_text)
        if not cot_text:
            self._latest_cot_text = None
            return

        if cot_text != self._latest_cot_text:
            self._ui_console.log(f"[CoT] {cot_text}")
        self._latest_cot_text = cot_text

    @staticmethod
    def _extract_trace_points(cot_text: str) -> List[Tuple[float, float]]:
        points: List[Tuple[float, float]] = []

        for match in re.finditer(r"<\|trace\|>(.*?)<\|/trace\|>", cot_text):
            try:
                payload = json.loads(match.group(1))
            except json.JSONDecodeError:
                continue
            trace_2d = payload.get("trace_2d")
            if isinstance(trace_2d, list):
                for point in trace_2d:
                    parsed = StarVLAServerInference._coerce_xy_pair(point)
                    if parsed is not None:
                        points.append(parsed)
            if points:
                return points

        for match in re.finditer(r"<\|point\|>(.*?)<\|/point\|>", cot_text):
            try:
                payload = json.loads(match.group(1))
            except json.JSONDecodeError:
                continue
            parsed = StarVLAServerInference._coerce_xy_pair(payload.get("point_2d"))
            if parsed is not None:
                points.append(parsed)
        if points:
            return points

        for match in re.finditer(r"\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\]", cot_text):
            points.append((float(match.group(1)), float(match.group(2))))
        return points

    @staticmethod
    def _coerce_xy_pair(value: object) -> Optional[Tuple[float, float]]:
        if not isinstance(value, (list, tuple)) or len(value) < 2:
            return None
        try:
            return float(value[0]), float(value[1])
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _denormalize_point(
        point: Tuple[float, float],
        width: int,
        height: int,
    ) -> Tuple[int, int]:
        x, y = point
        scale = 1000.0
        if max(abs(x), abs(y)) <= 1.5:
            scale = 1.0
        x_px = int(round(np.clip(x / scale, 0.0, 1.0) * max(width - 1, 1)))
        y_px = int(round(np.clip(y / scale, 0.0, 1.0) * max(height - 1, 1)))
        return x_px, y_px

    @staticmethod
    def _wrap_text(text: str, max_chars: int) -> List[str]:
        if not text:
            return []
        words = text.split()
        lines: List[str] = []
        current = ""
        for word in words:
            candidate = word if not current else f"{current} {word}"
            if len(candidate) <= max_chars:
                current = candidate
            else:
                if current:
                    lines.append(current)
                current = word
        if current:
            lines.append(current)
        return lines

    def _render_visualization(self) -> None:
        if not self._latest_obs_images:
            return

        image_tiles = [cv2.cvtColor(image, cv2.COLOR_RGB2BGR) for image in self._latest_obs_images]
        annotated_tiles: List[np.ndarray] = []
        for idx, tile in enumerate(image_tiles):
            canvas = np.array(tile, copy=True)
            cv2.putText(
                canvas,
                f"cam {idx}",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                canvas,
                f"cam {idx}",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (30, 30, 30),
                1,
                cv2.LINE_AA,
            )
            annotated_tiles.append(canvas)

        if annotated_tiles and self._latest_trace_points:
            overlay = annotated_tiles[0]
            denorm_points = [
                self._denormalize_point(point, overlay.shape[1], overlay.shape[0])
                for point in self._latest_trace_points
            ]
            if len(denorm_points) >= 2:
                cv2.polylines(
                    overlay,
                    [np.asarray(denorm_points, dtype=np.int32)],
                    False,
                    (0, 220, 255),
                    2,
                    cv2.LINE_AA,
                )
            for point_idx, (x_px, y_px) in enumerate(denorm_points):
                radius = 6 if point_idx in (0, len(denorm_points) - 1) else 4
                color = (0, 255, 0) if point_idx == 0 else (0, 220, 255)
                if point_idx == len(denorm_points) - 1:
                    color = (0, 120, 255)
                cv2.circle(overlay, (x_px, y_px), radius, color, -1, lineType=cv2.LINE_AA)
                cv2.circle(overlay, (x_px, y_px), radius + 2, (20, 20, 20), 1, lineType=cv2.LINE_AA)

        image_panel = np.hstack(annotated_tiles)
        sidebar_width = 460
        sidebar = np.full((image_panel.shape[0], sidebar_width, 3), 24, dtype=np.uint8)

        y = 32
        for header in (
            "StarVLA CoT Viewer",
            f"Predictions: {self._prediction_count}",
            f"Chunk step: {self._chunk_step}/{self.action_chunk_size}",
            f"Trace points: {len(self._latest_trace_points)}",
        ):
            cv2.putText(sidebar, header, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (230, 230, 230), 2, cv2.LINE_AA)
            y += 30

        if self._last_prediction_ts is not None:
            age_s = max(0.0, time.time() - self._last_prediction_ts)
            cv2.putText(
                sidebar,
                f"Last prediction: {age_s:.1f}s ago",
                (16, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (180, 180, 180),
                1,
                cv2.LINE_AA,
            )
            y += 32

        cv2.putText(sidebar, "Task", (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (120, 200, 255), 2, cv2.LINE_AA)
        y += 28
        for line in self._wrap_text(self.task, max_chars=40):
            cv2.putText(sidebar, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (235, 235, 235), 1, cv2.LINE_AA)
            y += 24

        y += 12
        cv2.putText(sidebar, "CoT", (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (120, 200, 255), 2, cv2.LINE_AA)
        y += 28
        cot_text = self._latest_cot_text or "No explicit CoT returned yet."
        for line in self._wrap_text(cot_text, max_chars=40):
            cv2.putText(sidebar, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (235, 235, 235), 1, cv2.LINE_AA)
            y += 22
            if y > sidebar.shape[0] - 24:
                break

        canvas = np.hstack([image_panel, sidebar])
        cv2.imshow(self._viz_window_name, canvas)
        cv2.waitKey(1)

    # ------------------------------------------------------------------
    # State machine hooks
    # ------------------------------------------------------------------

    def _start_infering(self) -> None:
        self._action_chunk = None
        self._chunk_step = 0
        self._latest_cot_text = None
        self._latest_trace_points = []
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
        try:
            cv2.destroyWindow(self._viz_window_name)
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
    import sys

    # When run directly, resolve the local websocket client without relative imports.
    import importlib.util, pathlib
    _here = pathlib.Path(__file__).parent
    spec = importlib.util.spec_from_file_location("websocket_policy_client", _here / "websocket_policy_client.py")
    _wpc_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(_wpc_mod)
    WebsocketClientPolicy = _wpc_mod.WebsocketClientPolicy  # type: ignore[assignment]

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
