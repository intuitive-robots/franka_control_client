"""Plot the GELLO and Franka joint values in real time with Rerun.

Passive: subscribes to the gello node (`pixi run gello`) and the franka_control_proxy and
streams both to a Rerun gRPC server on this host. Every joint gets its own plot with GELLO
and Franka overlaid (tabs: position, tracking error Franka - GELLO), next to a 3D view with
two Panda URDFs in the same scene: the real arm (solid) and the GELLO configuration
(transparent). Connect a viewer of the same Rerun version (0.26.2) from another machine,
e.g. a MacBook:

    mac$ pip install rerun-sdk==0.26.2
    mac$ ssh -p 2222 -L 9876:localhost:9876 dev@<this-host>
    mac$ rerun rerun+http://localhost:9876/proxy

The server keeps a buffer (--memory-limit), so a viewer connecting late still sees the history.

Usage:
    python examples/gello_rerun_viz.py --window 10
"""

from __future__ import annotations

import argparse
import time
import xml.etree.ElementTree as ET
from datetime import timedelta
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from scipy.spatial.transform import Rotation

JOINTS = [f"j{i + 1}" for i in range(7)]
GELLO_COLOR = [255, 140, 0]
FRANKA_COLOR = [30, 144, 255]
ERROR_COLOR = [220, 20, 60]

PANDA_MODEL_DIR = Path(__file__).resolve().parents[2] / "franka_control_proxy/models/franka_emika_panda"


def load_obj(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Vertices and triangles of an OBJ file (polygons fan-triangulated, materials ignored)."""
    verts, tris = [], []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(tok.split("/")[0]) for tok in line.split()[1:]]
            idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
            tris += [[idx[0], idx[k], idx[k + 1]] for k in range(1, len(idx) - 1)]
    return np.asarray(verts, dtype=np.float32), np.asarray(tris, dtype=np.uint32)


def vertex_normals(verts: np.ndarray, tris: np.ndarray) -> np.ndarray:
    face_n = np.cross(verts[tris[:, 1]] - verts[tris[:, 0]], verts[tris[:, 2]] - verts[tris[:, 0]])
    normals = np.zeros_like(verts)
    for k in range(3):
        np.add.at(normals, tris[:, k], face_n)
    return normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)


class UrdfArm:
    """Kinematic chain from a URDF, drawn with the visual meshes of the matching MuJoCo model.

    The URDF in franka_control_proxy/models references meshes that are not shipped; the MJCF
    (MuJoCo Menagerie) next to it has OBJ visuals in the same link frames, so those are used.
    Every link is its own entity below `root`, so posing the arm only logs one transform per joint.
    """

    def __init__(self, root: str, urdf: Path, mjcf: Path, link_prefix: str = "panda_") -> None:
        self._root = root
        self._link_prefix = link_prefix
        robot = ET.parse(urdf).getroot()
        joints = robot.findall("joint")
        children = {j.find("child").get("link") for j in joints}
        base = next(link.get("name") for link in robot.findall("link") if link.get("name") not in children)
        self.paths = {base: f"{root}/{base}"}
        self._joints = []  # (entity path, origin translation, origin rotation, axis, revolute)
        pending = list(joints)
        while pending:  # parents first, independent of the order in the file
            for j in list(pending):
                parent, child = j.find("parent").get("link"), j.find("child").get("link")
                if parent not in self.paths:
                    continue
                pending.remove(j)
                self.paths[child] = f"{self.paths[parent]}/{child}"
                origin = j.find("origin")
                xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()] if origin is not None else [0, 0, 0]
                rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()] if origin is not None else [0, 0, 0]
                axis = j.find("axis")
                self._joints.append(
                    (
                        self.paths[child],
                        np.asarray(xyz),
                        Rotation.from_euler("xyz", rpy),
                        np.asarray([float(x) for x in axis.get("xyz").split()]) if axis is not None else np.zeros(3),
                        j.get("type") in ("revolute", "continuous"),
                    )
                )
        self.num_dof = sum(revolute for *_, revolute in self._joints)
        self._mjcf = mjcf

    def log_static(self, alpha: float = 1.0) -> None:
        """Link meshes (alpha < 1 draws them transparent); call once."""
        mjcf = ET.parse(self._mjcf).getroot()
        mesh_dir = self._mjcf.parent / mjcf.find("compiler").get("meshdir", ".")
        materials = {
            m.get("name"): [float(x) for x in m.get("rgba", "0.8 0.8 0.8 1").split()]
            for m in mjcf.find("asset").iter("material")
        }
        mesh_files = {m.get("name", Path(m.get("file")).stem): m.get("file") for m in mjcf.find("asset").iter("mesh")}
        for body in mjcf.find("worldbody").iter("body"):
            path = self.paths.get(self._link_prefix + body.get("name"))
            if path is None:
                continue
            visuals = [g for g in body.findall("geom") if g.get("class", "").endswith("visual")]
            for k, geom in enumerate(visuals):
                verts, tris = load_obj(mesh_dir / mesh_files[geom.get("mesh")])
                rgba = list(materials.get(geom.get("material"), [0.8, 0.8, 0.8, 1.0]))
                rgba[3] *= alpha
                rr.log(
                    f"{path}/visual_{k}",
                    rr.Mesh3D(
                        vertex_positions=verts,
                        triangle_indices=tris,
                        vertex_normals=vertex_normals(verts, tris),
                        albedo_factor=rgba,
                    ),
                    static=True,
                )

    def log_configuration(self, q: np.ndarray) -> None:
        """Pose the arm; q holds one value per revolute joint in chain order [rad]."""
        i = 0
        for path, xyz, rot, axis, revolute in self._joints:
            r = rot
            if revolute:
                r = rot * Rotation.from_rotvec(axis * q[i])
                i += 1
            rr.log(path, rr.Transform3D(translation=xyz, mat3x3=r.as_matrix()))


class RateLimiter:
    """True at most `rate_hz` times per second; the samples in between are dropped."""

    def __init__(self, rate_hz: float) -> None:
        self._period = 1.0 / rate_hz
        self._last = 0.0

    def ready(self, now: float) -> bool:
        if now - self._last < self._period:
            return False
        self._last = now
        return True


def blueprint(window_s: float) -> rrb.Blueprint:
    """3D view of both arms next to one plot per joint (GELLO and Franka overlaid)."""
    scroll = rrb.VisibleTimeRange(
        "time",
        start=rrb.TimeRangeBoundary.cursor_relative(seconds=-window_s),
        end=rrb.TimeRangeBoundary.cursor_relative(),
    )

    def grid(group: str, unit: str, extra: list | None = None) -> rrb.Grid:
        views = [
            rrb.TimeSeriesView(origin=f"/{group}/{j}", name=f"{j} [{unit}]", time_ranges=scroll) for j in JOINTS
        ]
        return rrb.Grid(*views, *(extra or []), grid_columns=2, name=group.replace("_", " ").capitalize())

    gripper = rrb.TimeSeriesView(origin="/gripper", name="GELLO gripper", time_ranges=scroll)
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                origin="/", contents=["/franka_arm/**", "/gello_arm/**"], name="Franka (solid) / GELLO (transparent)"
            ),
            rrb.Tabs(
                grid("position", "deg", [gripper]),
                grid("tracking_error", "deg"),
            ),
            column_shares=[2, 3],
        ),
        collapse_panels=True,
    )


def log_series_styles() -> None:
    for j in JOINTS:
        rr.log(f"position/{j}/gello", rr.SeriesLines(names="GELLO", colors=GELLO_COLOR), static=True)
        rr.log(f"position/{j}/franka", rr.SeriesLines(names="Franka", colors=FRANKA_COLOR), static=True)
        rr.log(f"tracking_error/{j}", rr.SeriesLines(names="Franka - GELLO", colors=ERROR_COLOR), static=True)
    rr.log("gripper", rr.SeriesLines(names="gripper", colors=GELLO_COLOR), static=True)


class Visualizer:
    def __init__(self, gello_arm: UrdfArm, franka_arm: UrdfArm, plot_rate_hz: float, arm_rate_hz: float) -> None:
        self._gello_arm = gello_arm
        self._franka_arm = franka_arm
        self._gello_q: np.ndarray | None = None  # latest GELLO position [rad] for the tracking error
        # GELLO publishes at its bus rate (up to ~1 kHz); logging every sample makes the viewer lag
        self._plot_limit = {name: RateLimiter(plot_rate_hz) for name in ("gello", "gripper", "franka")}
        # posing an arm costs ~0.5 ms and 3D needs far less than the plot rate
        self._arm_limit = {name: RateLimiter(arm_rate_hz) for name in ("gello", "franka")}

    def on_gello_arm_state(self, msg: dict) -> None:
        q = np.asarray(msg["joint_state"], dtype=float)
        self._gello_q = q
        # local receive time, so GELLO and Franka share one timeline
        now = time.time()
        rr.set_time("time", timestamp=now)
        if self._arm_limit["gello"].ready(now):
            self._gello_arm.log_configuration(q)
        if self._plot_limit["gello"].ready(now):
            q = np.degrees(q)
            for i, j in enumerate(JOINTS):
                rr.log(f"position/{j}/gello", rr.Scalars(q[i]))

    def on_gello_gripper_state(self, msg: dict) -> None:
        now = time.time()
        if self._plot_limit["gripper"].ready(now):
            rr.set_time("time", timestamp=now)
            rr.log("gripper", rr.Scalars(msg["gripper"]))

    def on_franka_arm_state(self, msg: dict) -> None:
        now = time.time()
        rr.set_time("time", timestamp=now)
        q_rad = np.asarray(msg["q"], dtype=float)
        if self._arm_limit["franka"].ready(now):
            self._franka_arm.log_configuration(q_rad)
        if not self._plot_limit["franka"].ready(now):
            return
        q = np.degrees(q_rad)
        err = None if self._gello_q is None else np.degrees(q_rad - self._gello_q)
        for i, j in enumerate(JOINTS):
            rr.log(f"position/{j}/franka", rr.Scalars(q[i]))
            if err is not None:
                rr.log(f"tracking_error/{j}", rr.Scalars(err[i]))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-name", default="FrankaPanda")
    parser.add_argument("--gello-name", default="gello")
    parser.add_argument("--local-ip", default="192.168.1.1")
    parser.add_argument("--group-name", default="DroidGroup")
    parser.add_argument("--group-port", type=int, default=7730)
    parser.add_argument("--grpc-port", type=int, default=9876, help="Port the Rerun viewer connects to")
    parser.add_argument("--memory-limit", default="25%", help="Rerun server buffer, e.g. '2GB' or '25%%'")
    parser.add_argument("--window", type=float, default=10.0, help="Visible time window of the plots [s]")
    parser.add_argument("--urdf", type=Path, default=PANDA_MODEL_DIR / "panda_arm.urdf", help="Kinematics")
    parser.add_argument("--mjcf", type=Path, default=PANDA_MODEL_DIR / "panda_nohand.xml", help="Visual meshes")
    parser.add_argument("--plot-rate", type=float, default=100.0, help="Max. sample rate of the plots [Hz]")
    parser.add_argument("--arm-rate", type=float, default=30.0, help="Update rate of the 3D arms [Hz]")
    parser.add_argument("--flush-ms", type=float, default=20.0, help="Rerun send interval (SDK default 200 ms)")
    parser.add_argument("--gello-alpha", type=float, default=0.4, help="Opacity of the GELLO arm (0..1)")
    args = parser.parse_args()

    import pyzlc

    # the default batcher only sends every 200 ms, which is most of the visible latency
    rr.set_global_data_recording(
        rr.RecordingStream(
            "gello_franka_joints",
            batcher_config=rr.ChunkBatcherConfig(flush_tick=timedelta(milliseconds=args.flush_ms)),
        )
    )
    url = rr.serve_grpc(grpc_port=args.grpc_port, server_memory_limit=args.memory_limit)
    # active, not only default: the viewer otherwise restores the layout it stored for this app id
    rr.send_blueprint(blueprint(args.window), make_active=True)
    log_series_styles()
    # both arms share the base frame, so they overlap exactly when Franka tracks GELLO
    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    franka_arm = UrdfArm("franka_arm", args.urdf, args.mjcf)
    franka_arm.log_static()
    gello_arm = UrdfArm("gello_arm", args.urdf, args.mjcf)
    gello_arm.log_static(alpha=args.gello_alpha)
    for arm in (franka_arm, gello_arm):
        arm.log_configuration(np.zeros(arm.num_dof))  # until the first state message

    viz = Visualizer(gello_arm, franka_arm, args.plot_rate, args.arm_rate)
    pyzlc.init("gello_rerun_viz", args.local_ip, group_name=args.group_name, group_port=args.group_port)
    pyzlc.register_subscriber_handler(f"{args.gello_name}/gello_arm_state", viz.on_gello_arm_state)
    pyzlc.register_subscriber_handler(f"{args.gello_name}/gello_gripper_state", viz.on_gello_gripper_state)
    pyzlc.register_subscriber_handler(f"{args.robot_name}/franka_arm_state", viz.on_franka_arm_state)

    print(f"Rerun server at {url}")
    print(f"  remote: ssh -p 2222 -L {args.grpc_port}:localhost:{args.grpc_port} dev@<this-host>")
    print(f"          rerun rerun+http://localhost:{args.grpc_port}/proxy")
    print("Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        pyzlc.shutdown()


if __name__ == "__main__":
    main()
