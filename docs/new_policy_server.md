# Serving a new policy to the inference GUI

How to run a new model on the Franka through the inference GUI
(see [inference_gui.md](inference_gui.md)). The robot PC and the GUI stay unchanged:
you write one small server on the policy PC.

Reference implementations:

- [`examples/policy_server_template.py`](../examples/policy_server_template.py): minimal skeleton
- [`examples/memvla_policy_server.py`](../examples/memvla_policy_server.py): a complete real
  example (MemoryVLA), including the workarounds listed under [Pitfalls](#pitfalls)

```
 policy PC                                     robot PC
┌──────────────────────────────┐  TCP 8767  ┌──────────────────────────────┐
│ my_policy_server.py          │◄───────────│ inference_gui.app            │
│   MyPolicy(BasePolicy)       │ obs → chunk│   control pair → robot       │
│   PolicyServer(...).serve()  │───────────►│   dashboard :8080            │
└──────────────────────────────┘            └──────────────────────────────┘
```

## 1. Implement a `BasePolicy`

```python
from franka_control_client.inference_gui.policy_server import (
    JOINT_ACTION_SPACE, CARTESIAN_ACTION_SPACE, BasePolicy, PolicyServer,
)

class MyPolicy(BasePolicy):
    action_space = JOINT_ACTION_SPACE  # must match the control pair selected in the GUI
    fps = 15.0                         # rows in a chunk are 1/fps s apart; use the training rate
    exec_steps = 8                     # rows the robot executes before the next request

    def __init__(self, ckpt: str): ...          # load the model once
    def metadata(self) -> dict: ...             # shown in the GUI log on Connect
    def reset(self, task) -> None: ...          # every Start: clear history / memory / queues
    def infer(self, observation) -> np.ndarray: # return the action chunk [T, 8]
        ...

PolicyServer(MyPolicy(ckpt), host="0.0.0.0", port=8767).serve_forever()
```

`health()` reports `action_space`, `fps` and `exec_steps` to the GUI. The GUI uses the
server's `fps`, uses its `exec_steps` when the GUI's own setting is 0, and refuses a server
whose action space does not match the selected control pair.

### The contract

| | Content |
|---|---|
| in: `observation.state` | float32 `[8]`. Joint pair: `q0..q6, gripper`. Cartesian pair: `x, y, z, qx, qy, qz, qw, gripper` (base frame) |
| in: `observation.images.<key>` | uint8 `H×W×3` **RGB** at camera resolution. `<key>` is the camera's `obs_key` in the GUI config (currently `external` = `zed_right`, `wrist` = `zed_wrist`) |
| in: `task` | the instruction typed in the GUI |
| out | `[T, 8]` of **absolute** targets, same layout as the state, one row every `1/fps` s, gripper last |

The gripper value is **closedness**: 0 = open, 1 = closed (the GUI clips it to [0, 1]).

## 2. Match training exactly

Most bad rollouts come from a mismatch here, not from the model.

- **Action type**: if the model predicts deltas or normalized values, convert them to
  absolute, un-normalized targets inside `infer`.
- **Gripper**: if the model predicts a width, convert it to closedness.
- **Images**: same camera(s), same resize / crop / aspect handling, RGB vs BGR, same value
  range. If training used random-crop augmentation, check whether the upstream
  deploy code applies a matching center crop at inference.
- **Task string**: use the exact training instruction. A good pattern is a `task.txt` next to
  the checkpoint that the server uses, overriding the GUI's task (see the MemoryVLA server).
- **Rate and chunk**: `fps` = training data rate. Pick `exec_steps` from the measured
  inference latency (one call should be shorter than `exec_steps / fps`, or use async mode).
- **Stateful models** (memory, history): reset in `reset()`. If you call the model only every
  `exec_steps` frames, the model sees a subsampled stream, which differs from training.
- **Architecture settings**: load them from the checkpoint's own config rather than hard-coding
  them; some loaders silently re-initialize layers when, for example, `action_dim` is wrong.

## 3. Test before touching the robot

1. **Offline check.** Feed a recorded frame (state + image + task) from the training dataset
   into `policy.infer()` directly. Check:
   - the output shape is `(T, 8)`;
   - the values are close to that frame's recorded action;
   - the first row is close to the current state. In our demos, |action − state| is
     below about 0.2 rad, so a much larger first-step jump points to an input mismatch.

   Also save the exact image the model receives and compare it with a training image.
2. **Protocol check.** Start the server and connect the GUI in simulation mode
   (any machine):
   ```bash
   python -m franka_control_client.inference_gui.app --config configs/inference_gui_sim.json
   ```
   Set the policy host/port to the policy PC, then run Connect → Start → Stop. On Connect,
   check `action_space`, `fps`, `exec_steps` and your `metadata()` in the log.
3. **Real robot**, conservatively:
   - Safety → *On a larger jump* = **Keep going, speed-limited** (`first_step_jump_mode: "limit"`)
   - a low `max_joint_speed` at first (e.g. 0.5 rad/s), raised later
   - **Home** before every Start, with the scene set up as in the demos
   - one person at the GUI, one person with a hand on the **hardware e-stop**

## 4. Run it

Policy PC:

```bash
conda activate <model env>
cd franka_control_client
python examples/my_policy_server.py --port 8767 --ckpt /path/to/checkpoint
# wait for "Policy server listening on 0.0.0.0:8767" before connecting
```

Robot PC (no pixi needed; this is what `pixi run inference-gui` runs):

```bash
python -m franka_control_client.inference_gui.app --config configs/inference_gui.json
# open http://localhost:8080 (remote: ssh -L 8080:localhost:8080 <robot-pc>)
```

In the GUI set the policy host/port, control pair and cameras (`send_to_policy`), then
Apply → Connect → Home → Start. Several policies can run at the same time on different ports.

## Pitfalls

Each of these broke a real deployment once.

| Symptom | Cause | Fix |
|---|---|---|
| Server crashes on the first request: `No module named 'numpy._core.numeric'` | Robot PC pickles with numpy 2, model env has numpy 1.x | Copy `_alias_numpy2_pickles()` from `memvla_policy_server.py` (maps `numpy._core.*` to `numpy.core.*`). Do not upgrade numpy if the env pins numpy<2 (e.g. TensorFlow 2.15) |
| Server seems stuck after model loading, no "listening" line | A framework called `logging.config.dictConfig(disable_existing_loggers=True)` on import and silenced your loggers | After importing the model code: `logging.getLogger(name).disabled = False` for your loggers and `"policy_server"` |
| CUDA errors / "no kernel image" on an RTX 50-series GPU | Old PyTorch build without sm_120 | PyTorch ≥ 2.7 built for CUDA 12.8 (`--index-url https://download.pytorch.org/whl/cu128`) |
| `camera '...' not in observation` | Server's camera key ≠ GUI `obs_key` | Match the server's key to the GUI config (or make it a CLI flag) |
| Rollout stops: `first action is X rad from the measured state` | Prediction far from the current state, often an input mismatch | Use `first_step_jump_mode: "limit"` to continue speed-limited, **and** investigate the inputs (section 2) |
| Model loading takes minutes the first time | Large checkpoint read from disk, not yet in the page cache | Normal; wait for the "listening" line before Connect |
| Gated Hugging Face base model (e.g. Llama-2) | No access to the official repo | If only config/tokenizer are needed at inference, use a verbatim public mirror. Keep base weights next to the checkpoints (`HF_HUB_CACHE`) |
| Disk full while installing | Big wheels / caches on a small `/home` | Keep weights and caches on the data disk; `pip install --no-cache-dir` |

## Checklist

- [ ] `action_space`, `fps`, `exec_steps` set and matching the training data
- [ ] `infer` returns absolute targets `[T, 8]`, gripper closedness last
- [ ] image preprocessing and camera choice identical to training
- [ ] exact training task string (`task.txt`)
- [ ] `reset()` clears all per-episode state
- [ ] offline check on a dataset frame passes; first-step jump is small
- [ ] numpy-2 pickle shim present if the env uses numpy 1.x
- [ ] "listening" line visible in the server log
- [ ] sim GUI run passes; real run with speed limit and a hand on the e-stop
