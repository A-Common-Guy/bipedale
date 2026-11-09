"""The robot's CAD as rigid groups the browser poses from the joint angles.

Everything is expressed in the IMU frame of robot_full_floating.xml (x
forward, y left, z up, origin on the chip), which is the body frame the
controllers use. Each leg is a five-bar: two cranks on the motors, two lower
links on the knees and the wheel on the axle. The browser recomputes knees
and axle with the controllers' forward kinematics and moves every group by
the rotation of its link relative to the CAD pose, so only these anchors and
the CAD joint angles are needed here.

Meshes are clustered on a grid (cell_mm) and quantised to 16 bits, and parts
smaller than min_part_mm are left out: about a megabyte for the whole robot.
"""

from __future__ import annotations

import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

LEG_GEOMETRY = {"base": 0.09, "upper": 0.08, "lower": 0.13}


def _quat_matrix(q) -> np.ndarray:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _pose(element) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = _quat_matrix([float(v) for v in element.get("quat", "1 0 0 0").split()])
    matrix[:3, 3] = [float(v) for v in element.get("pos", "0 0 0").split()]
    return matrix


def read_stl(path: Path) -> np.ndarray:
    """Triangles (n, 3, 3) of a binary or ASCII STL."""
    data = path.read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * count == len(data):
            record = np.dtype([("normal", "<3f4"), ("v", "<9f4"), ("attr", "<u2")])
            return np.frombuffer(data, record, count, 84)["v"].reshape(-1, 3, 3).astype(np.float64)
    vertices = [[float(v) for v in line.split()[1:4]]
                for line in data.decode(errors="replace").splitlines()
                if line.strip().startswith("vertex")]
    return np.array(vertices, dtype=np.float64).reshape(-1, 3, 3)


def cluster(triangles: np.ndarray, cell: float) -> tuple[np.ndarray, np.ndarray]:
    """(vertices, faces) after merging vertices on a `cell` grid; degenerate faces dropped."""
    vertices = triangles.reshape(-1, 3)
    cells = np.floor(vertices / cell).astype(np.int64)
    _, first, inverse = np.unique(cells, axis=0, return_index=True, return_inverse=True)
    inverse = inverse.reshape(-1)
    counts = np.bincount(inverse)
    merged = np.stack([np.bincount(inverse, weights=vertices[:, k]) / counts for k in range(3)], 1)
    faces = inverse.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2])
                  & (faces[:, 0] != faces[:, 2])]
    _, keep = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    faces = faces[np.sort(keep)]
    used = np.unique(faces)
    remap = np.full(len(merged), -1, dtype=np.int64)
    remap[used] = np.arange(len(used))
    return merged[used], remap[faces]


def planar(point, inner_pivot, direction: float) -> tuple[float, float]:
    """Body-frame point -> five-bar plane: x from the inner to the outer pivot, y down."""
    return (direction * (point[0] - inner_pivot[0]), -(point[2] - inner_pivot[2]))


def forward_kinematics(theta_inner: float, theta_outer: float, g=LEG_GEOMETRY):
    """Axle in the five-bar plane (sandro_controllers/kinematics.cpp), or None."""
    p = g["upper"] * (math.cos(theta_outer) - math.cos(theta_inner))
    q = g["base"] + g["upper"] * (math.sin(theta_inner) + math.sin(theta_outer))
    radius = math.hypot(p, q)
    if radius > 2.0 * g["lower"]:
        return None
    b1 = math.acos(min(1.0, radius / (2.0 * g["lower"]))) + math.atan2(p, q)
    return (-g["upper"] * math.sin(theta_inner) + g["lower"] * math.cos(b1),
            g["upper"] * math.cos(theta_inner) + g["lower"] * math.sin(b1))


class _Tree:
    """Bodies, joints and visual geoms of the torso subtree, posed in the IMU frame."""

    def __init__(self, path: Path) -> None:
        root = ET.parse(path).getroot()
        compiler = root.find("compiler")
        self.mesh_dir = path.parent / (compiler.get("meshdir", "") if compiler is not None else "")
        self.mesh_files = {m.get("name"): m.get("file") for m in root.iter("mesh")}
        self.colors = {m.get("name"): [float(v) for v in m.get("rgba", "0.7 0.7 0.7 1").split()]
                       for m in root.iter("material")}
        torso = root.find("./worldbody/body[@name='torso']")
        if torso is None:
            raise ValueError(f"{path}: no torso body")
        self.imu = None
        for frame in torso.iter("frame"):
            if frame.get("name") == "imu_mount":
                self.imu = _pose(frame)
                chip = frame.find("./site[@name='imu_chip']")
                self.chip_quat = [float(v) for v in chip.get("quat", "1 0 0 0").split()] \
                    if chip is not None else [1.0, 0.0, 0.0, 0.0]
        if self.imu is None:
            raise ValueError(f"{path}: no imu_mount frame")
        self.to_imu = np.linalg.inv(self.imu)
        self.joints: dict[str, dict] = {}
        self.geoms: list[dict] = []
        self._walk(torso, np.eye(4), None, [])

    def _walk(self, element, pose, joint, chain) -> None:
        for geom in element.findall("geom"):
            if "visual" in (geom.get("class") or "") and geom.get("mesh"):
                self.geoms.append({"mesh": geom.get("mesh"), "group": joint,
                                   "pose": self.to_imu @ pose @ _pose(geom),
                                   "material": geom.get("material")})
        for child in element:
            if child.tag not in ("body", "frame"):
                continue
            child_pose = pose @ _pose(child)
            child_joint, child_chain = joint, chain
            for j in child.findall("joint"):
                name = j.get("name")
                if name and j.get("type", "hinge") == "hinge":
                    point = self.to_imu @ child_pose @ np.array(
                        [float(v) for v in j.get("pos", "0 0 0").split()] + [1.0])
                    self.joints[name] = {"pos": point[:3], "chain": list(chain)}
                    child_joint, child_chain = name, chain + [name]
            self._walk(child, child_pose, child_joint, child_chain)


def sources(mjcf: Path) -> list[Path]:
    """The MJCF and every mesh file it names: what the built model depends on."""
    mjcf = Path(mjcf)
    root = ET.parse(mjcf).getroot()
    compiler = root.find("compiler")
    mesh_dir = mjcf.parent / (compiler.get("meshdir", "") if compiler is not None else "")
    return [mjcf] + sorted({mesh_dir / m.get("file") for m in root.iter("mesh") if m.get("file")})


def build(mjcf: Path, cell_mm: float = 1.0, min_part_mm: float = 10.0) -> tuple[dict, bytes]:
    """(model description, packed mesh buffer) for the browser."""
    tree = _Tree(Path(mjcf))
    legs, groups = {}, {"torso": {"kind": "torso"}}
    for side, direction in (("right", 1.0), ("left", -1.0)):
        inner = tree.joints[f"{side}_motor_inner"]["pos"]
        outer = tree.joints[f"{side}_motor_outer"]["pos"]
        wheel = tree.joints[f"{side}_wheel"]
        knees = {}
        for name, joint in tree.joints.items():
            if name.startswith(f"{side}_passive"):
                crank = "inner" if f"{side}_motor_inner" in joint["chain"] else "outer"
                knees[crank] = (name, joint["pos"])
                groups[name] = {"kind": "lower", "side": side, "crank": crank}
        carrier = next(c for c, (name, _) in knees.items() if name in wheel["chain"])
        ki = planar(knees["inner"][1], inner, direction)
        ko = planar(knees["outer"][1], inner, direction)
        upper = LEG_GEOMETRY["upper"]
        theta_inner = math.atan2(-ki[0], ki[1])
        theta_outer = math.atan2(ko[0] - LEG_GEOMETRY["base"], ko[1])
        groups[f"{side}_motor_inner"] = {"kind": "crank", "side": side, "crank": "inner"}
        groups[f"{side}_motor_outer"] = {"kind": "crank", "side": side, "crank": "outer"}
        groups[f"{side}_wheel"] = {"kind": "wheel", "side": side, "carrier": carrier}
        legs[side] = {
            "direction": direction,
            "inner_pivot": inner.round(6).tolist(), "outer_pivot": outer.round(6).tolist(),
            "theta_cad": [theta_inner, theta_outer],
            "knee_y": {c: float(p[1]) for c, (_, p) in knees.items()},
            "wheel_y": float(wheel["pos"][1]),
            "wheel_cad": wheel["pos"].round(6).tolist(),
            "wheel_on": carrier,
            "upper_check_m": [math.hypot(*ki) - upper,
                              math.hypot(ko[0] - LEG_GEOMETRY["base"], ko[1]) - upper],
        }

    cell, minimum = cell_mm / 1000.0, min_part_mm / 1000.0
    meshes, parts, blobs, offset, cache = [], [], [], 0, {}
    for geom in tree.geoms:
        name = geom["mesh"]
        if name not in cache:
            file = tree.mesh_files.get(name)
            path = tree.mesh_dir / file if file else None
            if path is None or not path.is_file():
                cache[name] = None
                continue
            triangles = read_stl(path)
            extent = np.ptp(triangles.reshape(-1, 3), axis=0) if len(triangles) else np.zeros(3)
            if extent.max() < minimum:
                cache[name] = None
                continue
            vertices, faces = cluster(triangles, cell)
            if not len(faces):
                cache[name] = None
                continue
            low, high = vertices.min(axis=0), vertices.max(axis=0)
            scale = np.maximum(high - low, 1e-9) / 65535.0
            quantised = np.round((vertices - low) / scale).astype("<u2")
            index_type = "<u2" if len(vertices) < 65536 else "<u4"
            indices = faces.astype(index_type)
            entry = {"vertices": len(vertices), "faces": len(faces), "origin": low.tolist(),
                     "scale": scale.tolist(), "index": index_type[1:]}
            for key, array in (("vertex_offset", quantised), ("index_offset", indices)):
                blob = array.tobytes()
                blob += b"\0" * (-len(blob) % 4)
                entry[key] = offset
                blobs.append(blob)
                offset += len(blob)
            cache[name] = len(meshes)
            meshes.append(entry)
        if cache[name] is None:
            continue
        color = tree.colors.get(geom["material"], [0.7, 0.7, 0.7, 1.0])
        parts.append({"mesh": cache[name], "group": geom["group"] or "torso",
                      "color": [round(c, 3) for c in color[:3]],
                      "matrix": geom["pose"].T.reshape(-1).round(7).tolist()})
    for part in parts:
        if part["group"] not in groups:
            part["group"] = "torso"
    model = {"frame": "imu", "geometry": LEG_GEOMETRY, "legs": legs, "groups": groups,
             "wheel_radius": 0.0742, "imu_chip_quat": tree.chip_quat,
             "meshes": meshes, "parts": parts, "bytes": offset,
             "triangles": int(sum(meshes[p["mesh"]]["faces"] for p in parts))}
    return model, b"".join(blobs)
