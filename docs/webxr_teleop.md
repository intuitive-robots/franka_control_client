# WebXR teleop (Quest 3 → Franka EE target)

The Quest Browser opens a WebXR page served from this machine. The page streams the right
controller pose and buttons over a WebSocket. `WebXRController` turns them into a target EE
pose for `MQ3PandaControlPair` (Cartesian impedance) and records it with
`MQ3DataWrapper(hw_name="webxr")`. There's no Unity app and no simpub, and the page uses no CDN
or build step.

## Host setup (USB)

Run these on the host, not in the container:

```bash
adb reverse tcp:8000 tcp:8000          # again after every replug/reboot
adb shell am broadcast -a com.oculus.vrpowermanager.prox_close   # keep headset awake when off-head
adb shell am start -a android.intent.action.VIEW -d http://localhost:8000 com.oculus.browser
```

The server listens on `127.0.0.1:8000` over plain HTTP. On the Quest, `localhost` counts as a
secure context, so WebXR works without TLS. Only one process can hold port 8000, so stop any
other test server first.

## Run

```bash
pixi run -m code/franka_control_client webxr-echo          # no robot: prints target, clutch, Hz, latency
pixi run collect-webxr                                     # top level: proxy + cameras + collection TUI
pixi run -m code/franka_control_client data-collection-webxr --help
```

On the page, press **Enter XR**. The page shows WebSocket state, send rate and RTT, but only
outside the immersive session. Inside it you see passthrough and nothing is rendered.

## Controls (right controller)

| Input   | Action |
|---------|--------|
| Grip    | Clutch. Hold to move the EE, release to freeze it. Engages above 0.9 and releases below 0.1. |
| Trigger | Robotiq position, 0 open … 1 closed (proportional). The gripper holds its last value while de-clutched. |
| A / B   | Streamed and shown in `webxr-echo`, not used. Episodes are controlled from the keyboard TUI. |

While the clutch is held, the EE follows the controller's motion relative to where both were
when the clutch engaged: `pos = ee_base + M·Δp` and `rot = (M·R_Δ·Mᵀ)·R_ee_base`.

## Frames and yaw

WebXR (`local-floor`) is right-handed, +Y up, −Z forward. Forward is where the headset faced
when the session started or was last recentered. The default mapping assumes the operator
stands behind the robot facing base +X:

| Operator moves | EE moves |
|----------------|----------|
| forward        | +X       |
| left           | +Y       |
| up             | +Z       |

If you stand elsewhere, pass `--yaw <deg>`: the operator's facing direction in the base frame,
counter-clockwise from +X seen from above. `90` means you face base +Y (standing on the robot's
right side), and `180` means you face the robot. Recenter (hold the Meta button) while facing
your working direction, because yaw is relative to the recentered forward direction.

## Safety behaviour

- **De-clutch** when the newest sample is older than 100 ms (headset off, browser paused, cable
  pulled), on WebSocket disconnect, on a new connection, on recenter, when the XR session loses
  visibility or ends, and on `reset()`.
- **Re-engage** needs a fresh grip press: the grip must be seen below 0.1 first. A grip still held
  through a dropout, or held while the page connects, does nothing.
- **Workspace box** (base frame, metres; override with
  `--bounds XMIN YMIN ZMIN XMAX YMAX ZMAX`): x 0.25…0.80, y −0.45…0.45, z 0.05…0.75. The target is
  clamped to the box. The clutch refuses to engage if the measured EE is outside it, and logs a
  warning. **Check `z_min` against your table height.**
- **Step cap**: the target moves at most 1 cm and 5° per received sample (≈0.7 m/s and 360°/s at
  72 Hz). On re-engage the target walks from its last commanded value to the new base, without
  jumping.

Robot tests are started by the operator only, with the e-stop at hand.
