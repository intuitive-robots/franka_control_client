"""A URDF arm in Rerun: the Panda's kinematic chain with the MuJoCo Menagerie visual meshes (examples/gello_rerun_viz.py,
code/ode_policy/droid_rollout/viz/replay.py). Logs to the global Rerun recording."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation

PANDA_MODEL_DIR = Path(__file__).resolve().parents[4] / "franka_control_proxy/models/franka_emika_panda"


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
