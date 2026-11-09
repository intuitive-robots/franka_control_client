# Data collection GUI

Collect demonstrations from a browser, save them as IRL raw data and/or a LeRobot dataset,
convert existing IRL recordings, and inspect LeRobot datasets.

```bash
pixi run data-collection-gui        # robot PC, edit configs/data_collection_gui.json first
pixi run data-collection-gui-sim    # no robot: simulated arm, GELLO, VR controller, cameras
# open http://localhost:8090   (from another PC: ssh -L 8090:localhost:8090 <robot-pc>)
```

The page can move the robot and binds to `127.0.0.1`; `--host 0.0.0.0` only on a trusted network.
Deep links: `#convert=<irl folder>`, `#view=<lerobot folder>&ep=<n>`.

## Collect

| Mode | Leader → follower | Reset (R) | Leader folder | Terminal equivalent |
|---|---|---|---|---|
| GELLO teleop | GELLO joints → joint impedance (stiff / compliant gains) | moves the arm to the GELLO pose | `Gello/` joints, gripper, Robotiq command | `examples/data_collection_droid_irl.py` |
| VR teleop | Meta Quest 3 (hold right grip, index trigger = gripper) → Cartesian impedance | home + re-anchor the controller | `MQ3/` EE pose, gripper | `examples/data_collection_droid_vr_irl.py` |
| Kinesthetic | hand guiding (HumanControl, gravity compensated) | home, then the arm floats | none; O/P are saved as `gripper_command.pt` | `examples/demo_collection_droid_irl.py` |
| Teach & replay | hand-guided demo, then the robot replays it | home, then the arm floats | replay: `Traj/` (commands sent) | `examples/teach_and_replay_droid_irl.py` |
| Keyboard (push-T) | browser keys → Cartesian impedance, orientation locked | home, gripper untouched | `Keyboard/` target EE pose, `EE_vel`, gripper | new (below) |

Keys: **N** start, **S** save, **D** discard (Space toggles start/save, Esc discards), **R** reset,
**H** home (`home_q`), **O/P** open/close the gripper, **Y** replay the demo. As in the terminal
tools, teleop / hand guiding runs while recording and the arm goes to Idle when the episode
ends. Teach & replay saves demos to `<task>/demo/` and replays to `<task>/replay/`; the
replay is saved automatically after the demo duration + settle time. A saved demo can be picked
for replay from the episode table (*use*). The insertion push force of the terminal tool is
not in the GUI.

### Keyboard teleop (push-T)

`keyboard/keyboard_controller.py` + `control_pair/keyboard_panda_control_pair.py`. While
recording, with the page focused: **↑/↓** forward/back, **←/→** left/right in the table plane,
**Q** up, **E** down, hold **Shift** for slow motion (× `kb_slow_factor`); diagonals keep the
same speed. With *arrow keys frame* 0° ↑ is base +x and ← is base +y; pick 180° when you face
the robot. Held keys integrate a target pose (orientation locked to the pose at episode start),
sent at `kb_control_hz` with the target velocity as feed-forward.

Safety: the target stays inside the workspace box (`kb_workspace_min/max`; *z floor = current
EE z* sets the floor after you lower the pusher to the table) and never more than `kb_max_lead`
(default 2 cm) ahead of the measured EE per axis, so holding E against the table does not wind
up the contact force. The browser refreshes held keys every 100 ms; after 0.35 s without a
refresh (lost focus, closed tab, network) every key counts as released. Episode start/end,
Reset and Home never open the gripper (O/P do). The page shows the key pad, measured vs
target EE position and a top view of the trajectory in the workspace.

LeRobot: *auto* gives a cartesian state and the keyboard target as action
(`x, y, z, qx, qy, qz, qw, gripper`); for a planar push-T policy use action[:2] (and state[:2]).

**Settings** (*Apply* writes them to the config file; devices, mode and cameras only while
disconnected): task folder (`<data_dir>/<task>/<timestamp>/`), task instruction, robot sampling
rate, per-camera rate (0 = every sample), optional auto-save time.

**What to save**

* *IRL raw data*: the format of `data_collection/irl_data_collection.py` (below).
* *LeRobot dataset*: every saved episode is appended (in the background, after saving) to
  `lerobot_root` (default `<data_dir>/lerobot_data/<task>`) at **LeRobot fps**, with the
  image size, state space and action source chosen. It uses the same converter as the Convert
  tab. With IRL off the raw episode is a staging copy (`<data_dir>/.staging/`) that is deleted
  after the conversion (kept if the conversion fails). Every dataset episode is listed in
  `meta/irl_sources.json`; the IRL `metadata.json` gets `lerobot.episode_index`. *→LR* in the
  table (re)appends an existing episode. Deleting an episode in the GUI does not remove it from
  the LeRobot dataset: rebuild the dataset from the IRL folder in the Convert tab.

The sampling loop runs on an absolute schedule (the tile shows the measured rate and schedule
overruns); camera frames are written to PNG by a thread pool.

### IRL format

```
<data_dir>/<task>/<YYYY_MM_DD-HH_MM_SS>/
  timestamps.pt              [N] time.perf_counter() per robot sample
  FrankaPanda/               joint_pos, joint_vel, ee_pos (O_T_EE, 16), external_joint_torque,
                             external_wrench, gripper_state, gripper_current, EE_pos, EE_quat,
                             gripper_command (kinesthetic / teach & replay)
  Gello/ | MQ3/ | Traj/      leader (see the table)
  sensors/<camera>/000000.png
  camera_timestamps.pt       {camera: [F] perf_counter per frame}       new
  metadata.json              mode, task, instruction, rates, frame counts, ...   new
```

The two new files sit at the episode root, so `dataset_utils/*` and `Demo.load` work unchanged.

## Convert IRL → LeRobot

Pick a task folder (or one episode), *Scan*, untick episodes you do not want, choose the output
folder (stop / append / overwrite if it exists), fps, image size, state space, action source,
cameras, task instruction → *Convert*. Bad episodes are skipped and listed; *Cancel* keeps the
episodes finished so far. Same from the command line:

```bash
pixi run irl-to-lerobot /home/irl-admin/new_data_collection/cup_40hz \
    --output /home/irl-admin/new_data_collection/lerobot_data/cup_15fps --fps 15 [--size 256 256] \
    [--state-space joint|cartesian] [--action leader|next_state] [--camera zed_right --camera zed_wrist] \
    [--camera-hz zed_right=25] [--task "..."] [--append | --overwrite]
```

| feature | content |
|---|---|
| `observation.images.<cam>` | `zed_` prefix removed (`zed_right` → `right`), as in `convert_data_to_lerobot_*` |
| `observation.state` (8) | joint: `joint_0..6, gripper` · cartesian: `x, y, z, qx, qy, qz, qw, gripper` (quaternion signs kept continuous) |
| `observation.force_torque` (14) | external joint torque (7), external wrench (6), gripper current |
| `action` (8) | *leader*: GELLO joints / VR pose / replay command at the same sample, leader gripper · *next_state*: follower state of the next frame, gripper command if recorded |

*auto* state space is cartesian for VR and keyboard, joint otherwise; *auto* action is the leader when it
matches the state space, else the next state (kinesthetic demos).

**Timeline.** Frame *k* is the grid time `t0 + k / fps` over the robot timestamps: the latest
robot sample at or before it, and from each camera the latest frame captured up to that sample
(cameras are read in the same loop step, just after the robot state). The episode ends with the
first stream that ends. Recordings without `camera_timestamps.pt` (older ones) are assumed to
spread their frames evenly over the episode, or use the rate you enter; with nominal rates this
gives the same frames as `convert_data_to_lerobot_ft_differ_hz.py`. Unlike that script, which
maps by sample index, the robot side follows the real timestamps, so a late sample shows up as a
repeated frame instead of shifting the rest. Choose fps ≤ the robot rate.

## View LeRobot

Open any local LeRobot v3 dataset (*Collect dataset* opens the one the Collect tab writes).
Episode list with tasks; per episode the camera videos (the dataset's own mp4 files, seeked
to the episode; frames decoded on the server if the browser cannot play them, and for image
datasets), play / pause / speed / slider (Space, ←/→), and plots of every numeric feature,
grouped (joints, gripper, EE position, orientation, torques, wrench) with `observation.state`
solid and `action` dashed in the same colour per dimension. Hover for values, click to jump.
*Open in Rerun* runs `dataset_utils/lerobot_viz.py` for the episode (Rerun window on the robot PC).

## Code

`src/franka_control_client/data_collection_gui/`: `config.py`, `hardware.py` (devices and
control pair per mode, simulation), `irl_format.py` (writer/reader), `irl_to_lerobot.py`
(converter + CLI), `lerobot_viewer.py`, `session.py` (state machine, recording loop, export
queue, convert jobs), `web_server.py`, `static/index.html`, `app.py`. The control pairs and
`teach_replay_collection.Demo/DemoReplayer` are used unchanged; in VR mode the GUI paces
`MQ3PandaControlPair.control_step` to its `CONTROL_HZ` (500 Hz) on its own instance, because the
unpaced loop spins and starved the recorder. Tests: `tests/test_data_collection_gui.py`.
