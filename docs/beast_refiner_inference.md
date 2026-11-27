# BEAST Refiner inference on the Franka

This repository now has a BEAST-specific policy server:
[`examples/beast_refiner_policy_server.py`](../examples/beast_refiner_policy_server.py).
It connects the local `beast_refiner` LeRobot policy to the existing inference GUI.

## What BEAST Refiner does

BEAST Refiner is a two-speed policy:

- **System 2 (xVLA)** looks at the two cameras, end-effector state, and task text and makes a
  slower 24-step coarse plan.
- **System 1 (Refiner)** runs on every 20 Hz observation. It uses a five-frame image
  history to correct the current plan and emits the next end-effector target.
- The server runs System 2 in a background worker. System 1 continues at the control
  rate and adopts a new plan when it is ready.

This is different from a normal action-chunk policy. The model must receive one new
observation for every action. The server reports `query_every_step: true`, and the GUI
automatically switches to this timing mode.

## The installed checkpoint

The checkpoint currently on this machine is:

```text
/home/jjiang/utphd/models/conveyor-kit-R5-v2-joint-warm-lam0-c24
```

Its saved configuration says:

| Setting | Required value |
|---|---|
| action | absolute end-effector `x,y,z,qx,qy,qz,qw` + gripper closedness |
| policy rate | 20 Hz |
| camera 1 | `observation.images.static_cam` |
| camera 2 | `observation.images.wrist_cam` |
| state | 8 values: `x,y,z,qx,qy,qz,qw, gripper` |
| System-2 horizon | 24 actions; normally replans after 12 |
| Refiner history | 5 frames |

The checkpoint README says it has only been evaluated open-loop on three held-out
episodes and **has not yet been evaluated on hardware**. Treat the first runs as a new,
unvalidated controller.

The run name's `joint-warm` means joint training of System 1 and System 2 after a
warm-start; it does **not** mean joint-space robot actions.

## 1. Start the policy server

On the policy PC:

```bash
conda activate beast_refiner
cd /home/jjiang/jing/franka_control_client

PYTHONPATH=src python examples/beast_refiner_policy_server.py \
  --ckpt /home/jjiang/utphd/models/conveyor-kit-R5-v2-joint-warm-lam0-c24 \
  --xvla-checkpoint /home/jjiang/.cache/huggingface/hub/models--lerobot--xvla-base/snapshots/cdb7964e4fe842935d671bfab5a5ebe00a96648c \
  --task "put red cube in bowl" \
  --action-diagnostics \
  --port 8767
```

Wait for both lines before connecting the GUI:

```text
BEAST Refiner ready ...
Policy server listening on 0.0.0.0:8767 ...
```

The first start loads the xVLA and BEAST weights and runs CUDA warmup, so it can take a
while. The required xVLA, BART tokenizer, and ResNet18 files are already cached on this
machine.

`--action-diagnostics` attaches the active System-2 coarse plan and System-1 refined
future to every inference response. This adds a small amount of serialization overhead
and is intended for debugging rollouts. Saved GUI rollouts then contain the plans on the
same clock as the measured pose, published commands, post-limiter rows, and policy output.

The local training dataset is
`/home/jjiang/utphd/dataset/lerobot/conveyor_cube`. Its `meta/tasks.parquet` records the
exact task prompt `put red cube in bowl`; use it verbatim. The dataset is only needed for
offline replay/evaluation, not for live inference.

The checkpoint's saved inference configuration already enables C0/C1 spline projection
with four basis functions. No extra flag is required; the readiness log should report
`spline_projection=True`.

### Optional offline dataset inference

The local dataset is useful for replaying a recorded episode before connecting the robot:

```bash
conda activate beast_refiner
cd /home/jjiang/Zhuoyue/beast_refiner

MPLCONFIGDIR=/tmp/matplotlib-beast python scripts/openloop_eval.py \
  --episode_index=21 \
  --policy.path=/home/jjiang/utphd/models/conveyor-kit-R5-v2-joint-warm-lam0-c24 \
  --policy.xvla_checkpoint=/home/jjiang/.cache/huggingface/hub/models--lerobot--xvla-base/snapshots/cdb7964e4fe842935d671bfab5a5ebe00a96648c \
  --dataset.repo_id=conveyor_cube \
  --dataset.root=/home/jjiang/utphd/dataset/lerobot/conveyor_cube \
  --dataset.video_backend=pyav \
  --output_dir=/home/jjiang/utphd/models/conveyor-kit-R5-v2-joint-warm-lam0-c24/offline_eval \
  --plot_dof=0
```

This reads the dataset but does not communicate with or move the robot.

## 2. Start the GUI

On the robot PC:

```bash
cd /home/jjiang/jing/franka_control_client
python -m franka_control_client.inference_gui.app \
  --config configs/inference_gui_beast_refiner.json
```

Open `http://localhost:8080`. If the GUI is on a remote robot PC:

```bash
ssh -L 8080:localhost:8080 <robot-pc>
```

The current streaming controller continues each limited ramp from the previous
commanded setpoint. To compare it with the old behavior, which restarted every
ramp from the measured robot pose, add this GUI-only option:

```bash
python -m franka_control_client.inference_gui.app \
  --config configs/inference_gui_beast_refiner.json \
  --stream-anchor measured-pose
```

Omit the option (or pass `--stream-anchor persistent-setpoint`) for the current
default. This comparison changes only the streaming ramp anchor; it preserves
the current 20 Hz scheduling and all policy-server preprocessing.

Review the setup before applying it:

1. Change `policy_host` from `127.0.0.1` to the policy PC's address if the server is on
   another machine.
2. Keep the exact training task `put red cube in bowl` (also pass the same string with
   server `--task`, which pins it against accidental GUI edits).
3. Keep **control pair = cartesian**. Joint control is incompatible with this checkpoint.
4. Keep both `static_cam` and `wrist_cam` enabled for the policy, with the exact observation
   keys above.
5. Keep the policy rate at 20 Hz. The server is authoritative and will report 20 Hz.

Then use **Apply → Connect**. The log should show:

```text
action_space: ee_pose_abs_quat_gripper_closedness
fps: 20.0
exec_steps: 1
query_every_step: true
```

## 3. Validate before real motion

First use the simulation GUI to validate only the protocol and model inputs:

```bash
python -m franka_control_client.inference_gui.app \
  --config configs/inference_gui_sim.json
```

The generic sim config must be changed to the Cartesian pair and the two BEAST camera keys. It
cannot validate task success or camera appearance; it only checks Connect/Start/Stop and
that the server returns finite `[1, 8]` actions.

For the first hardware rollout:

- use the exact training scene, camera poses, and task setup;
- keep `first_step_jump_mode: "limit"` and `max_ee_speed: 0.15`;
- Home before Start;
- use a short time limit (the supplied config uses 60 seconds);
- have a second person at the hardware emergency stop;
- stop immediately if inference does not stay close to 20 Hz or the first-step warning is
  large/repeated.

The saved post-hoc spline projection supplies C0/C1 continuity to the raw-action head.
Do not disable it or raise speed limits to hide chatter during the first hardware tests.

## Troubleshooting

| Symptom | Meaning / fix |
|---|---|
| GUI refuses the server because action spaces differ | Select the **cartesian** control pair. |
| `camera ... is missing` | Use `static_cam` and `wrist_cam`, or server `--camera-map external=static_cam,wrist=wrist_cam`. |
| model tries to download `lerobot/xvla-base` | Point `--xvla-checkpoint` to the local snapshot under `~/.cache/huggingface/hub/models--lerobot--xvla-base/snapshots/...`. |
| first action is far from the measured end-effector pose | Stop and check task text, home pose, gripper convention, camera identity/pose, and RGB images. |
| inference misses 20 Hz | Check policy-PC GPU load; keep the asynchronous System-2 runner and CUDA warmup enabled. |
| inference is slower than 20 Hz and limited motion is barely visible | Streaming actions that miss their 50 ms deadline are ramped over their measured inference latency, capped at two policy periods. This preserves `max_ee_speed` while avoiding a one-control-tick displacement; do not raise the speed limit to compensate. |
| live camera colours differ from the training videos (red bowl looks blue) | The camera stream is BGR. Confirm against the real scene, then restart the server with `--swap-rgb`. |
| camera frames are not 256×256 | Handled: the server squashes every frame to the checkpoint's trained size with `cv2.INTER_AREA`, exactly like the dataset converter. The first rollout logs the incoming and model sizes. |
| System-2-only checkpoint (R4, `train_mode: sys2`) | The server keeps the checkpoint's `inference_mode`. R4 is served as action chunks: 24-step System-2 plans, 12 rows executed per call. Use GUI `inference_mode: async` so the arm does not pause for each plan. |
| `unsupported enabled fields: history_early_fusion, history_keypoints` (R9), `mlp requires use_vlm_context=false` (R10), `no choice class for 'xvla_dcdp'` (R11) | These checkpoints need the code they were trained on (`theogruner/beast_refiner` `main @ 1fd8462`), newer than the installed checkout. Clone it separately and pass its `src/` folder with `--beast-src`. R11 (DCDP) is a different policy type and additionally needs server support. |
| first rollout pauses on its first action | Server was started with `--no-warmup`; restart without that flag. |

### Plot the complete action pipeline

Save the rollout in the GUI rather than discarding it, then run:

```bash
conda activate beast_refiner
cd /home/jjiang/jing/franka_control_client

PYTHONPATH=src python examples/plot_beast_action_diagnostics.py \
  eval_rollouts/beast_refiner_conveyor_cube/<saved-rollout-folder>
```

This writes `beast_action_diagnostics.png`, `.csv`, and `.json` into the rollout folder.
The figure overlays the System-2 plan, System-1 future, emitted policy action, command sent
after limiting, and measured end-effector pose for all eight action dimensions. The JSON
summarizes Cartesian command-tracking error and the policy-to-command gap.

