"""Command-line options of WebXR teleop, shared by every script that builds a WebXRController (data collection,
policy rollouts, echo), so a controller feature is configured the same way everywhere.

    add_webxr_args(parser)                                  # or an argument group
    ctrl = webxr_controller_from_args(args, arm, bounds)
"""

import argparse
from typing import Union

from .webxr_controller import (
    DEFAULT_MOTION_SCALE,
    DEFAULT_YAW_DEG,
    MIN_MOTION_SCALE,
    MOTION_SCALE_RATE,
    HapticConfig,
    WebXRController,
    WorkspaceBounds,
    _ArmState,
)
from .webxr_server import WebXRServer

_Parser = Union[argparse.ArgumentParser, argparse._ArgumentGroup]


def add_webxr_args(parser: _Parser) -> None:
    parser.add_argument("--host", default="127.0.0.1", help="WebXR server address (adb reverse targets 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--yaw", type=float, default=DEFAULT_YAW_DEG,
                        help="operator heading in the base frame, deg CCW from +X")
    parser.add_argument("--motion-scale", type=float, default=DEFAULT_MOTION_SCALE,
                        help="robot motion / hand motion at start and on a left thumbstick press "
                             "(left thumbstick up/down changes it)")
    parser.add_argument("--min-motion-scale", type=float, default=MIN_MOTION_SCALE,
                        help="lowest motion scale the left thumbstick reaches (highest: 1)")
    parser.add_argument("--motion-scale-rate", type=float, default=MOTION_SCALE_RATE,
                        help="motion scale change per second at full thumbstick deflection (factor e^rate)")
    parser.add_argument("--haptics", action="store_true",
                        help="vibrate the right controller with the EE contact force while clutched")
    parser.add_argument("--haptic-deadband", type=float, default=HapticConfig.deadband_n,
                        help="force [N] below which it is off")
    parser.add_argument("--haptic-max-force", type=float, default=HapticConfig.max_force_n,
                        help="force [N] at full vibration")


def webxr_controller_from_args(args: argparse.Namespace, arm: _ArmState,
                               bounds: WorkspaceBounds = WorkspaceBounds(), name: str = "webxr") -> WebXRController:
    return WebXRController(
        name, arm, server=WebXRServer(args.host, args.port), yaw_deg=args.yaw, bounds=bounds,
        haptics=HapticConfig(args.haptic_deadband, args.haptic_max_force) if args.haptics else None,
        default_motion_scale=args.motion_scale, min_motion_scale=args.min_motion_scale,
        motion_scale_rate=args.motion_scale_rate,
    )
