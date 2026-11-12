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
| Thumbstick press | Realign: your current facing becomes forward (see [Frames and yaw](#frames-and-yaw)). Releases the clutch. |
| A       | Start an episode (same as `n` in the collection TUI). A short pulse confirms the press. |
| B       | Stop and save the episode (same as `s`). |
| X (left) | Delete: discards the running episode, or, while waiting, deletes the episode saved last (same as `x`). Double pulse. |
| Y (left) | Home the robot (same as `r`): turns teleop off and moves the arm home. Only while waiting. |

The buttons only act in `data-collection-webxr`, and only in the matching state: A while waiting,
B while recording. X deletes only the most recent episode, and only until the next one starts,
so a second press never reaches older recordings. A button already held when the page connects
counts only after it is released. The pulses play on the right controller.

The arm only follows the controller while the control loop runs. In the collection TUI, press
**`t`** while waiting to turn teleop on or off without recording. Pressing `n` with teleop on
keeps control running into the episode. Saving or discarding (`s`/`d`/`x`, B/X) leaves teleop
running, so the arm keeps following and keeps holding what it grasped; start
`data-collection-webxr` with `--stop-after-episode` to stop control (arm Idle, gripper opens) at
the end of each episode instead. `r` turns teleop off before moving the arm home.

## Recording

`data-collection-webxr` writes a LeRobot dataset directly to `<data-dir>/lerobot/<task>`, at
50 Hz by default (`--fps`). The features are the same as in `dataset_utils/convert_webxr_to_lerobot.py`,
so no conversion step is needed. If the folder already holds a dataset, recording resumes into
it, as long as the fps, cameras and image size match.

- Cameras are sampled on every step and resized to `--image-size` (default 256×256). A camera
  slower than `--fps` produces repeated frames, which cost almost nothing after video encoding.
- Video is encoded while recording (LeRobot streaming encoding, `--vcodec`, default
  `libsvtav1`), so saving takes well under a second.
- `action[t]` is the WebXR target `--action-offset` steps (default 1) after observation `t`.
- Episodes are written when the next one starts or on quit. That is what lets X delete the last
  one. To remove an episode that is already written, use
  `lerobot-edit-dataset --operation.type delete_episodes`.
- Quit with `q` or Ctrl-C, which finalizes the files. A hard kill or power loss leaves the files
  of that session without parquet footers, and the dataset won't load until they are repaired.
  Back up the folder between sessions if that risk matters.
- `--raw` keeps the old per-episode `.pt` + PNG layout in `<data-dir>/<task>`.

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
right side), and `180` means you face the robot. Yaw is relative to WebXR forward, so align
while facing your working direction.

**Realign (press the right thumbstick):** if the motion comes out rotated (for example after the
headset slept or the session was re-entered), face your working direction (the `--yaw`
direction) and click the thumbstick. Your current heading becomes the operator's forward, and you
feel a short double pulse. Only the heading is used, so pitch and roll don't matter, but don't
look straight down. Realigning releases the clutch, so release the grip and press it again.
A Meta recenter or a new XR session clears the realign, because WebXR forward is then your
heading anyway. `webxr-echo` shows the current offset as `align`.

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

## Taking the headset off

Taking the headset off makes it sleep, and the clutch drops (stale samples). When you put it
back on:

- **Haptics** on the right controller show the link state, because nothing is visible in XR. One
  short pulse means the WebSocket is connected. Two strong pulses mean it was lost and the page
  is reconnecting. The page drops a socket that hasn't answered a ping for 3 s.
- **XR session ended:** if the Quest ended the session during sleep, the 2D page is back. Press
  **Enter XR** again.
- **Clutch:** release the grip, then press it again to re-engage.
- **No pulse at all:** the adb tunnel is probably gone. Re-run `adb reverse tcp:8000 tcp:8000` on
  the host and reload the page.

To keep the headset from sleeping while it's off your head, run
`adb shell am broadcast -a com.oculus.vrpowermanager.prox_close`. Repeat it after every reboot.

Robot tests are started by the operator only, with the e-stop at hand.
