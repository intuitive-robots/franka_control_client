"""Record IRL teleop episodes straight into a LeRobotDataset (no raw .pt/.png round trip).

Same features as dataset_utils/convert_webxr_to_lerobot.py, so training code does not change:

    observation.images.<cam>  video  every camera, <cam> = hw_name without "zed_"
    observation.state         (8)    follower joints + gripper position
    observation.ee_pose       (7)    follower EE pos + quat xyzw
    observation.force_torque (14)    external joint torque + external wrench + gripper current
    action                    (8)    leader EE target pos + quat xyzw + gripper, taken at t + action_offset

Performance follows lerobot_record's record loop:
  * streaming video encoding: frames go to one PyAV encoder thread per camera while recording, so
    no PNGs are written and save_episode() only has to finalize the mp4s and the parquet rows;
  * images are resized on the loop thread (cv2 releases the GIL) before they reach the encoder,
    which keeps encoding cheap enough for 50 Hz with three cameras;
  * add_frame() runs inline, as in lerobot_record: it only validates and enqueues.
On top of that the loop runs on a fixed deadline (no drift from per-step sleep math) and reports
the achieved rate and overruns per episode.

Cameras are sampled every step, so a camera slower than ``fps`` contributes repeated frames; the
video encoder compresses those to almost nothing.

Saving is deferred: 's' keeps the episode in memory and it is written when the next episode starts
or on quit. Until then 'x' drops it, like deleting the last raw episode directory did. A saved
LeRobot episode can't be removed in place, its rows and video are appended to shared files.

Crash safety is LeRobot's: finalize() runs on 'q', Ctrl-C and exceptions. A hard kill (SIGKILL,
power loss) leaves this session's parquet files without a footer, and the dataset won't load
until they are repaired; earlier sessions' files (resume always starts new ones) stay intact.
"""

import time
from collections import deque
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple

import cv2
import numpy as np
import pyzlc
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from ..control_pair.control_pair import ControlPair
from .data_collection_manager import DataCollectionManager, DataCollectionState
from .irl_wrapper import IRL_HardwareDataWrapper, ImageDataWrapper

QUAT = ["qx", "qy", "qz", "qw"]
# the Robotiq state sometimes reports 0.0 before the first real reading; padded with the last value
_GRIPPER_FALLBACK = 0.0018


def camera_key(hw_name: str) -> str:
    return f"observation.images.{hw_name.replace('zed_', '')}"


def irl_features(cameras: Dict[str, Tuple[int, int]]) -> dict:
    """LeRobot features for ``cameras`` = {hw_name: (height, width)}."""
    feats = {
        camera_key(name): {
            "dtype": "video",
            "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        }
        for name, (h, w) in cameras.items()
    }
    feats["observation.state"] = {
        "dtype": "float32",
        "shape": (8,),
        "names": [f"joint_{i}" for i in range(7)] + ["gripper"],
    }
    feats["observation.ee_pose"] = {
        "dtype": "float32",
        "shape": (7,),
        "names": ["x", "y", "z"] + QUAT,
    }
    feats["observation.force_torque"] = {
        "dtype": "float32",
        "shape": (14,),
        "names": [f"external_joint_torque_{i}" for i in range(7)]
        + [f"external_wrench_{i}" for i in range(6)]
        + ["gripper_current"],
    }
    feats["action"] = {
        "dtype": "float32",
        "shape": (8,),
        "names": ["x", "y", "z"] + QUAT + ["gripper"],
    }
    return feats


def _vec(x, n: int) -> np.ndarray:
    return np.asarray(x, dtype=np.float32).reshape(-1)[:n]


class IRLLeRobotDataCollection(DataCollectionManager):
    def __init__(
        self,
        data_collectors: List[IRL_HardwareDataWrapper],
        root: Path,
        task: str,
        control_pair: Optional[ControlPair] = None,
        fps: int = 50,
        repo_id: Optional[str] = None,
        image_size: Optional[Tuple[int, int]] = (256, 256),  # (W, H); None keeps the camera size
        action_offset: int = 1,
        vcodec: str = "libsvtav1",
        encoder_threads: Optional[int] = None,
        encoder_queue_maxsize: Optional[int] = None,  # frames per camera; default 2 s worth
        robot_type: str = "franka_panda",
        # False: saving/discarding an episode leaves the control pair (teleop) running
        stop_control_on_episode_end: bool = True,
    ) -> None:
        super().__init__(data_collectors, task, fps)
        if action_offset < 0:
            raise ValueError("action_offset must be >= 0")
        self.control_pair = control_pair
        if control_pair is not None:
            self.register_start_collecting_event(control_pair.start_control_pair)
            if stop_control_on_episode_end:
                self.register_stop_collecting_event(control_pair.stop_control_pair)

        self.leader = self.follower_arm = self.follower_gripper = None
        self.cameras: List[ImageDataWrapper] = []
        for hw in data_collectors:
            if hw.hw_type == "leader_robot":
                self.leader = hw
            elif hw.hw_type == "follower_arm":
                self.follower_arm = hw
            elif hw.hw_type == "follower_gripper":
                self.follower_gripper = hw
            elif hw.hw_type == "camera":
                self.cameras.append(hw)
        missing = [n for n, hw in [("leader_robot", self.leader), ("follower_arm", self.follower_arm),
                                   ("follower_gripper", self.follower_gripper)] if hw is None]
        if missing:
            raise ValueError(f"Missing data collectors: {missing}")

        self.image_size = tuple(image_size) if image_size else None
        cam_shapes = {
            cam.hw_name: (self.image_size[1], self.image_size[0]) if self.image_size
            else tuple(cam.camera_device.size)
            for cam in self.cameras
        }
        self._cam_keys = [camera_key(cam.hw_name) for cam in self.cameras]
        self.action_offset = action_offset
        self.dataset = self._open_dataset(
            Path(root),
            repo_id or Path(root).name,
            irl_features(cam_shapes),
            robot_type,
            vcodec,
            encoder_threads,
            encoder_queue_maxsize or 2 * fps,
        )

        self._period = 1.0 / fps
        self._next_tick: Optional[float] = None
        self._pending_obs: Deque[dict] = deque()
        self._last_gripper = _GRIPPER_FALLBACK
        self._step_times: List[float] = []
        self._overruns = 0
        self._has_unsaved = False  # 's' pressed, written on the next 'n' or on quit
        self._closed = False

    # -- dataset ---------------------------------------------------------------------------------

    def _open_dataset(self, root, repo_id, features, robot_type, vcodec, encoder_threads, queue_maxsize):
        encoder = dict(
            vcodec=vcodec,
            streaming_encoding=True,
            encoder_queue_maxsize=queue_maxsize,
            encoder_threads=encoder_threads,
            batch_encoding_size=1,
        )
        if (root / "meta" / "info.json").exists():
            ds = LeRobotDataset(repo_id, root=root, **encoder)
            if ds.fps != self.fps:
                raise ValueError(f"{root} was recorded at {ds.fps} fps, not {self.fps}")
            have = {k: (v["dtype"], tuple(v["shape"])) for k, v in ds.features.items() if k in features}
            want = {k: (v["dtype"], tuple(v["shape"])) for k, v in features.items()}
            if have != want:
                raise ValueError(f"{root} has different features (image size or cameras?):\n{have}\nvs\n{want}")
            ds.meta.metadata_buffer_size = 1
            pyzlc.info(f"Resuming {root}: {ds.meta.total_episodes} episodes")
            return ds
        if root.exists() and any(root.iterdir()):
            raise FileExistsError(f"{root} exists and is not a LeRobot dataset")
        if root.exists():
            root.rmdir()  # LeRobotDataset.create insists on creating it
        return LeRobotDataset.create(
            repo_id,
            self.fps,
            features,
            root=root,
            robot_type=robot_type,
            use_videos=True,
            metadata_buffer_size=1,  # episode metadata reaches disk with every episode
            **encoder,
        )

    def _commit_unsaved(self) -> None:
        if not self._has_unsaved:
            return
        self._has_unsaved = False
        t0 = time.perf_counter()
        self.dataset.save_episode()
        self._ui_console.log(
            f"Episode {self.dataset.meta.total_episodes - 1} written ({time.perf_counter() - t0:.2f} s)."
        )

    # -- recording ---------------------------------------------------------------------------------

    def _start_collecting(self) -> None:
        try:
            self._commit_unsaved()
        except Exception as exc:  # don't lose the session over one bad episode
            pyzlc.error(f"Writing the previous episode failed: {exc}")
            self.dataset.clear_episode_buffer(delete_images=False)
        super()._start_collecting()
        self._pending_obs.clear()
        self._next_tick = None
        self._step_times = []
        self._overruns = 0

    def _collect_step(self) -> None:
        now = time.perf_counter()
        if self._next_tick is None:
            self._next_tick = now
        elif self._next_tick > now:
            time.sleep(self._next_tick - now)
        t0 = time.perf_counter()

        leader = self.leader.capture_step()
        arm = self.follower_arm.capture_step()
        gripper = self.follower_gripper.capture_step()
        images = [cam.capture_step() for cam in self.cameras]

        pos = float(gripper["position"])
        if pos == 0.0:
            pos = self._last_gripper
        else:
            self._last_gripper = pos

        obs = {
            "observation.state": np.concatenate([_vec(arm["q"], 7), [pos]]).astype(np.float32),
            "observation.ee_pose": np.concatenate([_vec(arm["EE_pos"], 3), _vec(arm["EE_quat"], 4)]),
            "observation.force_torque": np.concatenate([
                _vec(arm["tau_ext_hat_filtered"], 7),
                _vec(arm["O_F_ext_hat_K"], 6),
                _vec(gripper["current"], 1),
            ]),
        }
        for key, img in zip(self._cam_keys, images):
            if self.image_size is not None and (img.shape[1], img.shape[0]) != self.image_size:
                img = cv2.resize(img, self.image_size, interpolation=cv2.INTER_AREA)
            obs[key] = img
        self._pending_obs.append(obs)

        # action[t] is the leader command action_offset steps after observation t
        if len(self._pending_obs) > self.action_offset:
            frame = self._pending_obs.popleft()
            frame["action"] = np.concatenate([
                _vec(leader["EE_pos"], 3), _vec(leader["EE_quat"], 4), _vec(leader["gripper_width"], 1)
            ])
            frame["task"] = self.task
            self.dataset.add_frame(frame)

        self._step_times.append(time.perf_counter() - t0)
        self._next_tick += self._period
        if time.perf_counter() > self._next_tick:
            # late: start the next step right away, but don't burst to catch up on missed ones
            self._overruns += 1
            self._next_tick = time.perf_counter()

    def _episode_report(self) -> str:
        n = self.dataset.episode_buffer["size"] if self.dataset.episode_buffer else 0
        if not self._step_times:
            return "0 frames"
        st = np.asarray(self._step_times) * 1e3
        return (
            f"{n} frames, step {np.median(st):.1f} ms median / {st.max():.1f} ms max, "
            f"{self._overruns} late steps at {self.fps} Hz"
        )

    def _save_episode(self) -> None:
        self._stop_collecting()
        report = self._episode_report()
        if not self.dataset.episode_buffer or self.dataset.episode_buffer["size"] == 0:
            self.dataset.clear_episode_buffer(delete_images=False)
            self._ui_console.log(f"Episode too short, nothing saved ({report}).")
            return
        self._has_unsaved = True
        self._ui_console.log(f"Episode kept: {report}. Written when the next one starts; 'x' drops it.")

    def _discard_collecting(self) -> None:
        self._stop_collecting()
        self.dataset.clear_episode_buffer(delete_images=False)
        for collector in self.data_collectors:
            collector.discard()
        self._ui_console.log("Episode discarded.")

    def _delete_last_episode(self) -> None:
        if not self._has_unsaved:
            self._ui_console.log(
                "No episode to drop: only the last one, before the next starts. Remove written episodes "
                "with lerobot-edit-dataset --operation.type delete_episodes."
            )
            return
        self._has_unsaved = False
        self.dataset.clear_episode_buffer(delete_images=False)
        self._ui_console.log("Last episode dropped.")

    def _stop_collecting(self) -> None:
        super()._stop_collecting()
        self._pending_obs.clear()  # the last action_offset observations have no action

    def _reset_arm(self) -> None:
        if self.control_pair is None:
            self._ui_console.log("No control pair configured for reset.")
            return
        self._ui_console.log("Resetting robot arm position...")
        try:
            self.control_pair.control_reset()
            time.sleep(3)  # wait for the arm to reach the home position
            self._ui_console.log("Robot arm reset to home position.")
        except Exception as exc:
            self._ui_console.log(f"Failed to reset arm: {exc}")

    def _close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            if self._state_machine.state == DataCollectionState.COLLECTING:
                self._ui_console.log("Quit while recording: running episode discarded.")
                self.dataset.clear_episode_buffer(delete_images=False)
            self._commit_unsaved()
        except Exception as exc:
            pyzlc.error(f"Writing the last episode failed: {exc}")
        finally:
            # closes the parquet writers (footers) and the encoder threads
            self.dataset.finalize()
            pyzlc.info(f"Dataset {self.dataset.root}: {self.dataset.meta.total_episodes} episodes")
            super()._close()
