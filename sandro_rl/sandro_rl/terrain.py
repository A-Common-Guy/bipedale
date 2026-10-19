"""Terrain added around the generated robot model, never inside it.

`flat` uses scene_ros2.xml as generated. `obstacles` adds planks and hills
(an up ramp, a plateau and a down ramp) as mocap bodies through MjSpec at
load time; the robot, its contacts and the floor are untouched, so the
consistency check of the robot still applies.

Obstacles are mocap bodies because MuJoCo (CPU and Warp alike) only
collides static world geoms where they were compiled: moving a world geom
through `geom_pos` afterwards moves its drawing, not its collisions. Mocap
poses live in mjData, so every episode places each obstacle anew, and an
obstacle's height is set by sinking it into the floor. Shapes (plank
footprints, hill slopes) are fixed per model, spread over the configured
ranges with a fixed seed.
"""

from __future__ import annotations

import math

import jax
import jax.numpy as jp
import mujoco
import numpy as np

OBSTACLE_PREFIX = "obstacle_"
PARK_Z = -5.0          # where unused obstacles wait, below the floor
SHAPE_SEED = 12345


def _spread(rng, lo_hi, n):
    """n values covering [lo, hi] evenly, in a fixed shuffled order."""
    lo, hi = float(lo_hi[0]), float(lo_hi[1])
    values = lo + (hi - lo) * (np.arange(n) + 0.5) / max(n, 1)
    return rng.permutation(values)


def boxes(cfg: dict) -> list[dict]:
    """The obstacle bodies of a terrain config: name, kind, max height, geoms in the body frame."""
    if cfg.get("type", "flat") == "flat":
        return []
    rng = np.random.default_rng(SHAPE_SEED)
    out = []
    planks = cfg.get("planks") or {}
    n = int(planks.get("count", 0))
    if n:
        top = float(planks["height_m"][1])
        for i, (depth, width) in enumerate(zip(_spread(rng, planks["depth_m"], n), _spread(rng, planks["width_m"], n))):
            out.append({"name": f"{OBSTACLE_PREFIX}plank{i}", "kind": "plank", "top": top,
                        "geoms": [{"pos": [0.0, 0.0, -0.5 * top], "euler_y": 0.0,
                                   "size": [0.5 * depth, 0.5 * width, 0.5 * top]}]})
    hills = cfg.get("hills") or {}
    n = int(hills.get("count", 0))
    if n:
        top = float(hills["height_m"][1])
        thickness = 0.05
        for i, (slope, plateau, width) in enumerate(zip(_spread(rng, hills["slope_deg"], n),
                                                        _spread(rng, hills["plateau_m"], n),
                                                        _spread(rng, hills["width_m"], n))):
            a = math.radians(slope)
            run = top / math.tan(a)                       # horizontal length of one ramp at full height
            extra = 0.02                                  # past the foot, below the floor
            half_len = 0.5 * (run / math.cos(a) + extra)
            geoms = [{"pos": [0.0, 0.0, -0.5 * top], "euler_y": 0.0,
                      "size": [0.5 * plateau, 0.5 * width, 0.5 * top]}]
            for side in (-1.0, 1.0):
                # Top face from the foot (x = side*(plateau/2 + run), z = -top) to the
                # plateau edge (x = side*plateau/2, z = 0), in the body frame (top at z = 0).
                mid_x = side * (0.5 * plateau + 0.5 * run)
                mid_z = -0.5 * top
                normal = np.array([side * math.sin(a), 0.0, math.cos(a)])
                downhill = np.array([side * math.cos(a), 0.0, -math.sin(a)])
                centre = np.array([mid_x, 0.0, mid_z]) - thickness * normal + 0.5 * extra * downhill
                geoms.append({"pos": centre.tolist(), "euler_y": side * a,
                              "size": [half_len, 0.5 * width, thickness]})
            out.append({"name": f"{OBSTACLE_PREFIX}hill{i}", "kind": "hill", "top": top,
                        "footprint": 0.5 * plateau + run, "geoms": geoms})
    return out


def _quat_y(angle):
    return [math.cos(0.5 * angle), 0.0, math.sin(0.5 * angle), 0.0]


def add_bodies(mj_spec, items, friction, mocap: bool, poses=None):
    for k, item in enumerate(items):
        pos, quat = ([0.0, 0.0, PARK_Z], [1.0, 0.0, 0.0, 0.0]) if poses is None else poses[k]
        body = mj_spec.worldbody.add_body(name=item["name"], mocap=mocap, pos=pos, quat=quat)
        for j, g in enumerate(item["geoms"]):
            # contype 2 / conaffinity 1: they touch the robot (1/1) but not each
            # other; MuJoCo Warp does not drop static-static pairs as the CPU does.
            body.add_geom(name=f"{item['name']}_{j}", type=mujoco.mjtGeom.mjGEOM_BOX, size=g["size"],
                          pos=g["pos"], quat=_quat_y(g["euler_y"]), contype=2, conaffinity=1,
                          friction=[friction, 0.005, 0.0001], rgba=[0.55, 0.45, 0.35, 1.0])
        mj_spec.add_exclude(bodyname1="world", bodyname2=item["name"])


def build(spec, cfg: dict) -> mujoco.MjModel:
    kind = cfg.get("type", "flat")
    if kind == "flat":
        return spec.model
    if kind != "obstacles":
        raise ValueError(f"unknown terrain type {kind}")
    mj_spec = mujoco.MjSpec.from_file(str(spec.scene))
    add_bodies(mj_spec, boxes(cfg), float(cfg.get("friction", 0.8)), mocap=True)
    return mj_spec.compile()


def static_course(spec, items, poses, friction=0.8) -> mujoco.MjModel:
    """A fixed course for the CPU evaluation: the same shapes, compiled in place."""
    mj_spec = mujoco.MjSpec.from_file(str(spec.scene))
    add_bodies(mj_spec, items, friction, mocap=False, poses=poses)
    return mj_spec.compile()


def _quat_to_mat(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    return jp.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Terrain:
    def __init__(self, spec, cfg: dict):
        self.kind = cfg.get("type", "flat")
        self.cfg = cfg
        self.items = boxes(cfg)
        m = spec.model
        self.body_ids = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, it["name"]) for it in self.items],
                                 dtype=int)
        self.mocap_ids = np.array([m.body_mocapid[b] for b in self.body_ids], dtype=int)
        # Every obstacle geom in its body frame, for the analytic ground height.
        owner, centres, rots, sizes = [], [], [], []
        for k, item in enumerate(self.items):
            for g in item["geoms"]:
                owner.append(k)
                centres.append(g["pos"])
                a = g["euler_y"]
                rots.append([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
                sizes.append(g["size"])
        self.geom_owner = jp.asarray(np.array(owner, dtype=int))
        self.geom_centre = jp.asarray(np.array(centres, dtype=np.float32).reshape(-1, 3))
        self.geom_rot = jp.asarray(np.array(rots, dtype=np.float32).reshape(-1, 3, 3))
        self.geom_size = jp.asarray(np.array(sizes, dtype=np.float32).reshape(-1, 3))
        self.tops = np.array([it["top"] for it in self.items], dtype=np.float32)
        self.kinds = [it["kind"] for it in self.items]

    @property
    def active(self) -> bool:
        return len(self.items) > 0

    def layout(self, rng):
        """Per-episode mocap poses (pos [n, 3], quat [n, 4]) of all obstacles."""
        n = len(self.items)
        if n == 0:
            return jp.zeros((0, 3)), jp.zeros((0, 4))
        keys = jax.random.split(rng, 4)
        lo = np.zeros(n, np.float32)
        hi = np.zeros(n, np.float32)
        r_lo = np.zeros(n, np.float32)
        r_hi = np.zeros(n, np.float32)
        for k, item in enumerate(self.items):
            c = self.cfg["planks" if item["kind"] == "plank" else "hills"]
            lo[k], hi[k] = c["height_m"]
            r_lo[k], r_hi[k] = c["radius_m"]
            r_lo[k] += item.get("footprint", 0.0)
        height = jax.random.uniform(keys[0], (n,), minval=lo, maxval=hi)
        radius = jax.random.uniform(keys[1], (n,), minval=r_lo, maxval=jp.maximum(r_hi, r_lo + 0.1))
        angle = jax.random.uniform(keys[2], (n,), minval=-math.pi, maxval=math.pi)
        yaw = jax.random.uniform(keys[3], (n,), minval=-math.pi, maxval=math.pi)
        pos = jp.stack([radius * jp.cos(angle), radius * jp.sin(angle), height], -1)
        quat = jp.stack([jp.cos(0.5 * yaw), jp.zeros(n), jp.zeros(n), jp.sin(0.5 * yaw)], -1)
        return pos, quat

    def apply(self, data, pos, quat):
        if not self.active:
            return data
        idx = jp.asarray(self.mocap_ids)
        return data.replace(mocap_pos=data.mocap_pos.at[idx].set(pos), mocap_quat=data.mocap_quat.at[idx].set(quat))

    def ground_height(self, points, data):
        """Terrain height under each point (x, y): the highest box top face covering it, or the floor."""
        if not self.active:
            return jp.zeros(points.shape[0])
        idx = jp.asarray(self.mocap_ids)
        body_pos = data.mocap_pos[idx][self.geom_owner]
        body_rot = jax.vmap(_quat_to_mat)(data.mocap_quat[idx])[self.geom_owner]
        rot = body_rot @ self.geom_rot                                   # [g, 3, 3]
        centre = body_pos + jp.einsum("gij,gj->gi", body_rot, self.geom_centre)
        normal = rot[:, :, 2]

        def one(p):
            dx, dy = p[0] - centre[:, 0], p[1] - centre[:, 1]
            z = centre[:, 2] + (self.geom_size[:, 2] - dx * normal[:, 0] - dy * normal[:, 1]) / normal[:, 2]
            rel = jp.stack([dx, dy, z - centre[:, 2]], -1)
            local = jp.einsum("gji,gj->gi", rot, rel)
            inside = (jp.abs(local[:, 0]) <= self.geom_size[:, 0]) & (jp.abs(local[:, 1]) <= self.geom_size[:, 1])
            return jp.maximum(jp.max(jp.where(inside, z, 0.0)), 0.0)

        return jax.vmap(one)(points)
