"""The range scanner and the pillar clearances of the avoid task.

terrain.ray_distance must return what MuJoCo's own mj_ray returns on the
world geoms (floor and obstacles, never the robot), and terrain.pillar_gaps
the distance between two footprints found by brute force.
"""

import jax
import jax.numpy as jp
import mujoco
import numpy as np
import pytest

from sandro_rl import terrain
from sandro_rl.robot import RobotSpec

CFG = {"type": "obstacles",
       "planks": {"count": 3, "height_m": [0.005, 0.03], "depth_m": [0.04, 0.3], "width_m": [0.2, 1.0],
                  "radius_m": [0.5, 2.0]},
       "hills": {"count": 2, "height_m": [0.02, 0.1], "slope_deg": [5.0, 15.0], "plateau_m": [0.2, 0.8],
                 "width_m": [0.8, 1.6], "radius_m": [0.3, 2.0]},
       "pillars": {"count": 8, "height_m": [0.45, 0.8], "depth_m": [0.08, 0.4], "width_m": [0.08, 1.2],
                   "radius_m": [0.4, 2.0]}}
WORLD_GROUPS = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)   # floor and obstacles (group 0), not the robot
MAX_RANGE = 2.5


@pytest.fixture(scope="module")
def world():
    spec = RobotSpec.load()
    m = terrain.build(spec, CFG)
    t = terrain.Terrain(RobotSpec.load(spec.scene, model=m), CFG)
    d = mujoco.MjData(m)
    pos, quat = t.layout(jax.random.PRNGKey(5))
    d.mocap_pos[:], d.mocap_quat[:] = np.asarray(pos), np.asarray(quat)
    mujoco.mj_forward(m, d)

    class Data:
        mocap_pos = jp.asarray(d.mocap_pos)
        mocap_quat = jp.asarray(d.mocap_quat)

    return m, d, t, Data


def test_pillars_are_taller_than_the_robot(world):
    m, d, t, _ = world
    assert len(t.pillar_items) == CFG["pillars"]["count"]
    for k in t.pillar_items:
        top = d.mocap_pos[t.mocap_ids[k]][2]
        assert CFG["pillars"]["height_m"][0] - 1e-6 <= top <= CFG["pillars"]["height_m"][1] + 1e-6


def test_ray_distance_matches_mj_ray(world):
    m, d, t, data = world
    rng = np.random.default_rng(0)
    geom = np.zeros(1, np.int32)
    hits = 0
    for _ in range(40):
        origin = np.array([*rng.uniform(-1.5, 1.5, 2), rng.uniform(0.05, 0.4)])
        # The scanner never starts inside an obstacle (that is a collision);
        # from inside, mj_ray reports the far face.
        inside_pillar = np.min(np.asarray(t.pillar_gaps(jp.asarray(origin[:2], jp.float32), jp.array([1.0, 0.0]),
                                                        np.array([1e-3, 1e-3]), data))) <= 0.0
        if inside_pillar or float(t.ground_height(jp.asarray(origin[None], jp.float32), data)[0]) >= origin[2] - 0.05:
            continue
        angles = rng.uniform(-np.pi, np.pi, 16)
        dip = rng.uniform(-0.4, 0.15, 16)
        dirs = np.stack([np.cos(angles) * np.cos(dip), np.sin(angles) * np.cos(dip), np.sin(dip)], -1)
        got = np.asarray(t.ray_distance(jp.asarray(origin, jp.float32), jp.asarray(dirs, jp.float32), data, MAX_RANGE))
        for direction, value in zip(dirs, got):
            want = mujoco.mj_ray(m, d, origin, direction, WORLD_GROUPS, 1, -1, geom)
            want = MAX_RANGE if want < 0 else min(want, MAX_RANGE)
            assert value == pytest.approx(want, abs=2e-4)
            hits += want < MAX_RANGE and m.geom_bodyid[geom[0]] != 0
    assert hits > 20, "too few rays hit an obstacle to test anything"


def _perimeter(c, heading, half, n=400):
    a = np.asarray(heading) / np.linalg.norm(heading)
    b = np.array([-a[1], a[0]])
    s = np.linspace(-1, 1, n // 4)
    edges = [np.c_[s, np.ones_like(s)], np.c_[s, -np.ones_like(s)], np.c_[np.ones_like(s), s], np.c_[-np.ones_like(s), s]]
    local = np.concatenate(edges) * half
    return c + local[:, :1] * a + local[:, 1:] * b, (c, a, b, half)


def _inside(points, box):
    c, a, b, half = box
    rel = points - c
    return (np.abs(rel @ a) <= half[0]) & (np.abs(rel @ b) <= half[1])


def test_pillar_gaps_match_brute_force(world):
    m, d, t, data = world
    rng = np.random.default_rng(1)
    pillars = []
    for k in t.pillar_items:
        size = t.geom_size[t.pillar_geoms[list(t.pillar_items).index(k)]]
        q = d.mocap_quat[t.mocap_ids[k]]
        yaw = 2 * np.arctan2(q[3], q[0])
        pillars.append(_perimeter(d.mocap_pos[t.mocap_ids[k]][:2], [np.cos(yaw), np.sin(yaw)], np.asarray(size[:2])))
    half = np.array([0.125, 0.17])
    overlaps = 0
    for _ in range(60):
        centre = rng.uniform(-1.5, 1.5, 2)
        yaw = rng.uniform(-np.pi, np.pi)
        heading = np.array([np.cos(yaw), np.sin(yaw)])
        got = np.asarray(t.pillar_gaps(jp.asarray(centre, jp.float32), jp.asarray(heading, jp.float32), half, data))
        own, own_box = _perimeter(centre, heading, half)
        for value, (theirs, box) in zip(got, pillars):
            overlap = _inside(own, box).any() or _inside(theirs, own_box).any()
            if overlap:
                overlaps += 1
                assert value <= 1e-4
            else:
                gap = np.min(np.linalg.norm(own[:, None] - theirs[None], axis=-1))
                assert value == pytest.approx(gap, abs=0.01)
    assert overlaps > 0
