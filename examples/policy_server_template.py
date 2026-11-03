"""Template: serve your own model to the inference GUI (run on the policy PC).

    python examples/policy_server_template.py --port 8767 --ckpt /path/to/model

The robot PC sends, per request, a dict with
    "observation.state"          float32 [8]  joint: q0..q6 + gripper closedness
                                              cartesian: xyz + quat(xyzw) + gripper
    "observation.images.<key>"   uint8 HxWx3 RGB (keys set in the GUI camera list)
    "task"                       str
and expects an action chunk [T, 8] of ABSOLUTE targets (same layout as the
state), one row every 1/fps seconds, gripper last (0 open, 1 closed).
"""

from __future__ import annotations

import argparse
import logging
from typing import Any, Dict, Optional

import numpy as np

from franka_control_client.inference_gui.policy_server import (
    JOINT_ACTION_SPACE,
    BasePolicy,
    PolicyServer,
)


class MyPolicy(BasePolicy):
    action_space = JOINT_ACTION_SPACE  # or CARTESIAN_ACTION_SPACE
    fps = 15.0  # rate of the rows in a chunk
    exec_steps = 8  # rows the robot executes before asking again

    def __init__(self, ckpt: str, device: str = "cuda") -> None:
        self.ckpt = ckpt
        self.device = device
        self.task: Optional[str] = None
        # self.model = load_my_model(ckpt).to(device).eval()

    def metadata(self) -> Dict[str, Any]:
        return {"name": "my_policy", "ckpt": self.ckpt}

    def reset(self, task: Optional[str]) -> None:
        self.task = task
        # clear model memory / action queue here

    def infer(self, observation: Dict[str, Any]) -> np.ndarray:
        state = np.asarray(observation["observation.state"], dtype=np.float32)
        # images = {k: v for k, v in observation.items() if k.startswith("observation.images.")}
        # chunk = self.model.predict(state, images, observation.get("task"))
        chunk = np.repeat(state[None, :8], 16, axis=0)  # placeholder: hold still
        return chunk


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--ckpt", required=True)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    PolicyServer(MyPolicy(args.ckpt), host=args.host, port=args.port).serve_forever()


if __name__ == "__main__":
    main()
