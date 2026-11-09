# Inference GUI

Run a policy on the Franka from a browser dashboard: pick the control pair and rates,
watch latency / control frequency / robot state / cameras live, and label + save every rollout.

```
 policy PC                                   robot PC
┌───────────────────────────┐   TCP 8767   ┌──────────────────────────────────────────┐
│ inference_gui.policy_server│◄────────────│ inference_gui.app                         │
│  your BasePolicy           │ obs → chunk │  InferenceSession                         │
│  (model / dummy)           │────────────►│   ├ inference loop (sync / async)         │
└───────────────────────────┘              │   ├ control pair (joint / cartesian) ─────┼─► pyzlc ─► proxy ─► robot
                                           │   ├ state monitor + recorder              │
                                           │   └ HTTP dashboard :8080  ◄── browser     │
                                           └──────────────────────────────────────────┘
```

The socket protocol is the existing one from `policy_inference/policy_server_client.py`
(length-prefixed pickle; `health` / `reset` / `infer` / `close`), so servers written for
`RemoteJointPolicyInference` keep working.

## Quick start

**Try it without hardware** (two terminals, any machine):

```bash
pixi run policy-server-dummy                 # policy side, port 8767
pixi run inference-gui-sim                   # robot side, open http://localhost:8080
```

**Real robot, MME-VLA (memevla)**

1. Policy PC: start the server in the memevla venv. Wait for
   `Policy server listening on 0.0.0.0:8768` (loading and compiling the model takes a few minutes).

   ```bash
   cd /data/zhuoyue/Nov_Evaluation/memevla && source env_local.sh
   $PY ../franka_control_client/examples/meme_policy_server.py --port 8768 \
       --ckpt <checkpoint step dir> --task "<instruction>"
   ```

   | task | `--ckpt` (under `/data/zhuoyue/Nov_Evaluation/ckpt/meme/`) | `--task` (verbatim) |
   |---|---|---|
   | all three (multitask) | `franka-mem-pi05-seed42-10k/franka-mem-seed42/9999` | one of the three below |
   | sponge | `franka-mem-pi05-sponge-seed42-6k/franka-mem-sponge-b128-seed42/5999` | `Pick up the sponge and place it on the plate. Go up, move to pick up the sponge on the plate, return it to its original position.` |
   | plant | `franka-mem-pi05-plant-seed42-8k/franka-mem-plant-b128-seed42/7999` | `Pick up the shovel, scoop two shovelfuls of soil into the flowerpot, put down the shovel, pick up the flower, and plant it in the flowerpot.` |
   | pot timer | `franka-mem-pi05-pottimer-seed42-8.7k/franka-mem-pottimer-b128-seed42/8699` | `Put the pot on the stove, wait for 30 seconds, then put the green pepper into the pot.` |

   `--task` overrides the instruction typed in the GUI. Without it, the GUI's task is used and
   must match the training instruction exactly.

2. Robot PC (franka proxy and cameras running):

   ```bash
   cd franka_control_client
   pixi run inference-gui
   # dashboard: http://localhost:8080 (from another machine: ssh -L 8080:localhost:8080 <robot-pc>)
   ```

3. In the dashboard, Setup panel (then *Apply*):
   - Policy server: host = policy PC address, *Port* `8768`, control pair `joint`,
     *Rows executed / chunk* `0` (uses the server's 10), *Inference mode* `Async`,
     ***Env frequency (Hz)* `15`** (the memory needs every frame)
   - Safety: *On a larger jump* = *Keep going, speed-limited*; a low *Max joint speed* at first
     (e.g. 0.5 rad/s)
   - LeRobot dataset (optional): on, image size e.g. 640 × 360 (native 2K needs ~250 MB/s of RAM)

4. *Connect*, check `action_space` / `fps 15` / `exec_steps 10` in the log, *Home*, *Start*.
   Keep a hand on the hardware e-stop.

**Real robot, any other policy**

```bash
# policy PC
python -m franka_control_client.inference_gui.policy_server \
    --policy my_package.my_policy:MyPolicy --policy-kwargs '{"ckpt": "/path"}'
# robot PC (franka proxy running), edit configs/inference_gui.json first
pixi run inference-gui
```

Write your policy by subclassing `BasePolicy` — see `examples/policy_server_template.py`.
`infer()` gets `observation.state` (8 values), `observation.images.<key>` (RGB uint8)
and `task`, and returns a chunk `[T, 8]` of **absolute** targets, one row per `1/fps` s,
gripper closedness last (0 open, 1 closed). Joint pair: `q0..q6, g`; cartesian pair:
`x, y, z, qx, qy, qz, qw, g`. `health()` reports `action_space`, `fps`, `exec_steps`;
the robot side refuses a server whose action space does not match the selected pair.

The dashboard binds to `127.0.0.1`. From another machine use
`ssh -L 8080:localhost:8080 <robot-pc>`, or `--host 0.0.0.0` on a trusted network only
(there is no authentication and the page can move the robot).

## Workflow

1. **Setup panel** — batch name + description, task instruction, control pair, control Hz,
   policy host/port, chunk rate, executed rows, sync/async, safety limits. *Apply* writes
   them back to the config file. Robot / network / camera settings only change while
   disconnected.
   *Export* downloads the settings shown in the form as a JSON file; *Import* loads such a
   file (or any `configs/*.json`) into the form, so each user can keep their own settings
   and reload them. An import is not applied until you review it and click *Apply*; unknown
   keys are ignored and missing ones keep their current value.
2. **Connect** — robot devices, cameras and policy server (`health()` is shown in the log).
3. Optionally type **rollout notes** (scene setup), then **Start** (`N`).
4. **Stop** (`S`), **Software stop** (`Esc`: halts the control loop and sets the arm to Idle;
   not a replacement for the hardware e-stop), or auto-stop on time limit / safety / error.
5. **Review card** — name (default `<batch>_epNNN`), description (pre-filled with the notes),
   outcome (success / partial / failure / unlabeled) → **Save** (`Ctrl+Enter`) or **Discard**.
   **Home** (`H`) is available in between. Saved rollouts can be re-labeled later from the
   table (*Edit*); changes are kept in `meta.json["edits"]`.

Sync mode (default) is observe → infer → play `exec_steps` rows → observe; the arm holds
while the server computes. Async requests the next chunk while the current one plays
(lead time = smoothed round trip) and drops the rows that are already stale when it
arrives. Chunks are interpolated from the measured state to the control rate
(joints/positions linear, quaternions normalised-linear, gripper held per row).

Safety checks on each chunk: first row may not jump more than
`max_first_step_jump_joint` rad / `max_first_step_jump_cartesian` m from the measured
state (the rollout stops otherwise); joint targets are clipped to the Panda limits;
non-finite actions stop the rollout.

## Dashboard

| Panel | Shows |
|---|---|
| Tiles | inference rate, round-trip latency p50/p95/max, model vs network time, control-loop Hz vs target, robot state stream Hz, chunks (rows / executed / skipped / rejected) |
| Charts | latency (round trip & server model time, 60 s), control-loop rate vs target, per-joint (or EE pose) measured vs commanded, gripper (10 s) |
| Robot state | 3D arm from forward kinematics (drag/scroll), commanded pose ghost and the planned EE path of the last chunk, joint positions within limits, EE pose |
| Cameras | optional live view (off by default; recording does not depend on it) |
| Saved rollouts | this batch's index with outcome, duration, latency, description; success rate in the header |

## Saved data

```
<save_root>/<batch_name>/
  batch.json            batch name/description, task, config
  index.jsonl           one summary line per saved rollout
  000_<name>/
    meta.json           name, description, outcome, notes, task, config, policy health, stats, array schema
    episode.npz         numeric streams (below)
    cam_<camera>.mp4    frames sent to the policy (record_images)
```

`episode.npz` (all times are wall clock, seconds):

| key | content |
|---|---|
| `state/t, q, dq, q_d, ee_pos, ee_quat, gripper, tau_ext` | executed trajectory sampled at `state_record_hz` |
| `policy/t_obs, t_recv, rtt_s, server_infer_s, obs_state` | each inference call |
| `policy/action_raw` | generated chunks as returned by the policy `[M, T, 8]` (NaN-padded) |
| `policy/action_exec` | rows actually queued to the control pair after slicing/resampling |
| `policy/skipped_rows` | stale rows dropped in async mode |
| `control/t` | every control-loop step |
| `command/arm_t, arm` | every command the controller published (joint targets or EE pose) |
| `command/gripper_t, gripper` | gripper commands |
| `camera/<name>/t` | timestamp of each video frame |

```python
import numpy as np, json
d = np.load("eval_rollouts/eval/000_eval_ep000/episode.npz")
meta = json.load(open("eval_rollouts/eval/000_eval_ep000/meta.json"))
```

## Rollouts as a LeRobot dataset

Tick **Save rollouts as LeRobot dataset** (Setup → LeRobot dataset, `save_lerobot`). Each
rollout then also gets a one-episode LeRobot dataset (lerobot 0.4, v3 format) in
`<rollout folder>/lerobot/`, sampled at `dataset_fps` (0 = the chunk rate) while it runs:

| feature | content |
|---|---|
| `observation.images.<key>` | every camera, under the key the policy gets; resized to `dataset_image_width × height` if set; AV1 video (or PNG with `dataset_use_videos: false`) |
| `observation.state` (8) | joint pair: `joint_0..6, gripper`; cartesian: `x, y, z, qx, qy, qz, qw, gripper` (gripper = measured closedness) |
| `observation.force_torque` (14) | `tau_ext_hat_filtered`, `O_F_ext_hat_K`, gripper current (`dataset_force_torque`) |
| `action` (8) | what the controller played at the **next** frame: its arm command (joint targets / EE pose) and the gripper command (1 = closed), as in the converters where the action is the leader one step later |

The task string is the task instruction. Like the rollout station, a rollout is its own folder,
so discarding or deleting one removes its episode, and changing a label never rewrites the
dataset.

**Buffered, written at save.** During the rollout the samples (state, command, camera frames)
only go into a RAM buffer, so recording never slows down or drops time. *Save* resamples
the buffer onto an exact `dataset_fps` grid from the start to the stop of the rollout
(frames = duration × fps), then writes and encodes the episode in the background (the
header shows the progress). `meta.json["lerobot"]` holds `frames`, `duplicated_frames` (grid
slots without a new sample) and `buffered_samples_mb`.

RAM use is width × height × 3 bytes per camera per frame: two native 2K ZED cameras at 15 Hz
need ~250 MB/s (15 GB per minute) and take a long time to encode; 640 × 360 needs ~21 MB/s.
Set `dataset_image_width/height` accordingly. The log shows the estimate at Start, and the
rollout is stopped (reason "LeRobot buffer: low memory") if free RAM drops below 2 GB.

**Merge for training.** *Build merged LeRobot dataset* under the rollout table (or the CLI)
aggregates the batch's rollouts with the chosen outcomes, using the labels as they are now:

```bash
python -m franka_control_client.inference_gui.lerobot_export eval_rollouts/<batch> \
    --outcome success [--outcome partial] [--output <dir>] [--overwrite]
# default output: eval_rollouts/<batch>_dataset_<outcomes>, repo id local/<folder name>
```

`meta/rollouts.json` of the merged dataset maps every episode index to its rollout folder,
name, outcome and description. Rollouts recorded with different settings (cameras, image
size, rate, control pair) have different features and are refused with the folders listed.

## Code

`src/franka_control_client/inference_gui/`: `config.py` (settings), `policy_server.py`
(policy PC), `session.py` (state machine, inference loop, safety), `hardware.py` (real +
simulated devices, command taps), `telemetry.py`, `recorder.py`, `lerobot_export.py`
(per-rollout LeRobot episodes and merging), `web_server.py`,
`static/index.html`, `app.py`. The existing control pairs are used unchanged; the session
wraps `control_step` and the device `send_*` methods on the instances to measure the loop
rate and record commands. Tests: `tests/test_inference_gui.py`.
