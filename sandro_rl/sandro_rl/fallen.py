"""Lying poses for the stand-up task: the robot dropped and at rest, limp or legs held.

Each state starts at the stand keyframe tipped by a random pitch (up to
80 degrees, either way), a little roll, a fore-aft shove and leg joint noise,
then falls with every drive unpowered for `seconds`, on the model with the leg
collision boxes (collision.py). Unpowered, the legs fold until their links
meet: in the CAD-derived model the robot ends up leaning on a folded crank,
front or back, 15-40 degrees from upright, or flatter after a harder fall.

The robot's legs do not fold like that: the geared drives' friction keeps them
where they were, and a controller that was holding them leaves them there. So
`held` of the states fall with the legs held where a draw over their whole
reach put them (leg_targets), stretched ones included: lying on the torso
with the legs out, the tyres clear of the floor and turning them moves nothing.

    uv run python -m sandro_rl.fallen            # writes data/fallen_states.npz
"""

from __future__ import annotations

import argparse
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np

from .robot import RobotSpec

DEFAULT_PATH = Path(__file__).resolve().parent.parent / "data" / "fallen_states.npz"


def _rotation_quat(axis, angle):
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.asarray(axis, dtype=float), float(angle))
    return q


def tipped_qpos(spec, m, pitch: float, roll: float = 0.0) -> np.ndarray:
    """The stand keyframe turned about the axle midpoint: pitch about the lateral axis, roll about the heading."""
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    mujoco.mj_kinematics(m, d)
    r = d.site_xmat[spec.imu_site].reshape(3, 3)
    heading = np.array([r[0, 0], r[1, 0], 0.0])
    heading /= np.linalg.norm(heading)
    lateral = np.array([-heading[1], heading[0], 0.0])
    pivot = d.xanchor[spec.wheel_joint_ids].mean(axis=0)
    # Positive pitch tips the nose down (about +lateral, the right-hand rule).
    q = np.zeros(4)
    mujoco.mju_mulQuat(q, _rotation_quat(heading, roll), _rotation_quat(lateral, pitch))
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, q)
    qpos = d.qpos.copy()
    qpos[0:3] = pivot + rot.reshape(3, 3) @ (qpos[0:3] - pivot)
    body = np.zeros(4)
    mujoco.mju_mulQuat(body, q, qpos[3:7])
    qpos[3:7] = body / np.linalg.norm(body)
    qpos[2] += 0.01
    return qpos


def leg_targets(spec, rng, same: bool | None = None) -> np.ndarray:
    """Leg joint angles (leg_index order) drawn over the legs' reach: folded to nearly
    stretched, the axle ahead of or behind the pivots. Both legs alike in half the draws."""
    from . import kinematics
    g = spec.limits.leg_geometry["right"]

    def one():
        while True:
            inner, outer = rng.uniform(-1.0, 1.7, size=2)
            p = g["upper_m"] * (math.cos(outer) - math.cos(inner))
            q = g["base_m"] + g["upper_m"] * (math.sin(inner) + math.sin(outer))
            if math.hypot(p, q) >= 2.0 * g["lower_m"]:
                continue                            # the lower links do not reach each other
            x, y, reach = kinematics.forward(inner, outer, g["base_m"], g["upper_m"], g["lower_m"])
            if 0.04 <= y <= 0.205 and reach <= 0.985 and -0.06 <= x - 0.5 * g["base_m"] <= 0.06:
                return inner, outer

    right = one()
    # The left leg is the right one mirrored: its inner drive is the right's outer.
    left = right[::-1] if (rng.uniform() < 0.5 if same is None else same) else one()[::-1]
    return np.array([*right, *left])


def hold(spec, d, targets, kp: float = 20.0, kd: float = 0.6) -> None:
    """Leg drives holding `targets` (a position loop within the drives' peak torque)."""
    legs = spec.leg_index
    q, v = d.qpos[spec.joint_qpos[legs]], d.qvel[spec.joint_dof[legs]]
    peak = spec.limits.leg_torque_nm
    d.ctrl[spec.joint_actuator[legs]] = np.clip(kp * (targets - q) - kd * v, -peak, peak)


def _drop(args):
    seed, seconds, held, hulls = args
    from . import terrain
    spec = RobotSpec.load()
    m = terrain.build(spec, {}, leg_boxes=True, leg_hulls=hulls)
    rng = np.random.default_rng(seed)
    held = rng.uniform() < held
    pitch = rng.uniform(-1.6, 1.6) if held else rng.uniform(-1.4, 1.4)
    roll = rng.uniform(-0.3, 0.3)
    targets = leg_targets(spec, rng) if held else None
    d = mujoco.MjData(m)
    d.qpos[:] = tipped_qpos(spec, m, pitch, roll)
    legs = np.asarray(spec.joint_qpos)[list(spec.leg_index)]
    d.qpos[legs] += rng.uniform(-0.2, 0.2, size=len(legs))
    mujoco.mj_forward(m, d)
    r = d.site_xmat[spec.imu_site].reshape(3, 3)
    heading = np.array([r[0, 0], r[1, 0], 0.0]) / max(math.hypot(r[0, 0], r[1, 0]), 1e-9)
    d.qvel[0:3] = rng.uniform(-0.8, 0.8) * heading
    d.ctrl[:] = 0.0
    for _ in range(int(seconds / m.opt.timestep)):
        if held:
            hold(spec, d, targets)
        mujoco.mj_step(m, d)
    return d.qpos.copy(), float(np.max(np.abs(d.qvel))), pitch, held


def generate(count: int = 1024, seed: int = 0, seconds: float = 3.0, workers: int = 8, held: float = 0.0,
             hulls: int | None = None):
    """`hulls`: the robot lies on its links' and torso's hulls of that many vertices, not on the
    link boxes (collision.add_floor_hulls); the model a config with collision.leg_hulls trains on."""
    seeds = np.random.SeedSequence(seed).generate_state(count)
    with ProcessPoolExecutor(workers) as pool:
        results = list(pool.map(_drop, [(int(s), seconds, held, hulls) for s in seeds], chunksize=16))
    qpos = np.stack([r[0] for r in results])
    still = np.array([r[1] for r in results])
    pitch = np.array([r[2] for r in results])
    return qpos, still, pitch, np.array([r[3] for r in results])


def load(path: Path | str = DEFAULT_PATH, nq: int | None = None) -> np.ndarray:
    data = np.load(path)
    qpos = data["qpos"]
    if nq is not None and qpos.shape[1] != nq:
        raise ValueError(f"{path}: states have nq {qpos.shape[1]}, the model {nq}; regenerate with "
                         "python -m sandro_rl.fallen")
    return qpos


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--count", type=int, default=1024)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--held", type=float, default=0.0, help="share of the states with the legs held")
    parser.add_argument("--hulls", type=int, default=None,
                        help="lie on the links' and torso's hulls of this many vertices (collision.leg_hulls)")
    parser.add_argument("--out", type=Path, default=DEFAULT_PATH)
    args = parser.parse_args(argv)
    qpos, still, pitch, held = generate(args.count, args.seed, args.seconds, held=args.held, hulls=args.hulls)
    keep = still < 0.05
    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, qpos=qpos[keep], start_pitch=pitch[keep], held=held[keep])
    print(f"{keep.sum()}/{len(keep)} states at rest (max |qvel| < 0.05), {int(held[keep].sum())} with the legs "
          f"held -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
