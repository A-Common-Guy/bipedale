"""Model consistency check: the GPU environment must simulate the ROS model.

    uv run python -m sandro_rl.consistency                 # current model
    uv run python -m sandro_rl.consistency --scene /tmp/altered/scene_ros2.xml

Two parts, both must pass:
  1. The contract (robot.RobotSpec.load): joint names, one direct motor per
     joint, the stand keyframe, the IMU sensors, the four loop-closing
     connects, the tyre contact settings the randomisation handles, a physics
     step that divides the control period. A change the environment does not
     handle fails here with the reason.
  2. Dynamics: identical open-loop torque sequences from the stand keyframe
     (landed on the floor) in CPU MuJoCo (float64) and in the training
     backend (MJX with the training configuration's graph mode, float32),
     compared after one step from identical states and over a 0.2 s
     rollout, against the tolerances below.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")

import mujoco
import numpy as np

from . import sources
from .robot import ModelContractError, RobotSpec

# Tolerances. The one-step tolerance bounds float32 against float64 for the
# same state and torque. Over a rollout a fixed tolerance is meaningless:
# the passive five-bar links (armature 1e-5) and the tyre contacts amplify
# any per-step error, float32 rounding included. So the rollout is judged
# against the model's own sensitivity: the GPU trajectory must stay within
# ROLLOUT_SPREAD_FACTOR times the largest deviation of CPU trajectories whose
# velocities get a relative error of ROLLOUT_NOISE per step (the size of the
# one-step float32 error), plus a floor. Measured values: docs/DESIGN.md.
ONE_STEP_QVEL_TOL = 2e-3          # rad/s or m/s, max over dofs and states
ROLLOUT_STEPS = 240               # 0.2 s at 1200 Hz
ROLLOUT_NOISE = 4e-5
ROLLOUT_NOISE_SEEDS = 4
ROLLOUT_SPREAD_FACTOR = 3.0
ROLLOUT_FLOOR = {"legs": 2e-3, "wheels": 2e-3, "base": 2e-4}     # rad, rad, m


@dataclass
class Report:
    passed: bool = True
    checks: list = field(default_factory=list)

    def add(self, name, value, tolerance, ok=None):
        ok = bool(value <= tolerance) if ok is None else bool(ok)
        self.checks.append({"check": name, "value": float(value), "tolerance": float(tolerance), "pass": ok})
        self.passed = self.passed and ok

    def text(self) -> str:
        return "\n".join(f"{'PASS' if c['pass'] else 'FAIL'}  {c['check']}: {c['value']:.3g} (tolerance {c['tolerance']:.3g})"
                         for c in self.checks)


def torque_sequences(spec: RobotSpec, n: int, steps: int, seed: int) -> np.ndarray:
    """[n, steps, 6]: the settled holding torque plus sinusoids, within the
    controllers.yaml torque limits."""
    rng = np.random.default_rng(seed)
    lim = spec.limits
    limit = np.array([lim.wheel_torque_nm if j in sources.WHEEL_JOINTS else lim.leg_torque_nm
                      for j in sources.JOINTS])
    _, hold = settled_start(spec)
    t = np.arange(steps) * spec.physics_dt
    freq = rng.uniform(0.5, 4.0, (n, 1, 6))
    phase = rng.uniform(0, 2 * np.pi, (n, 1, 6))
    amp = rng.uniform(0.1, 0.3, (n, 1, 6)) * limit
    return np.clip(hold + amp * np.sin(2 * np.pi * freq * t[None, :, None] + phase), -limit, limit)


SETTLE_S = 0.3


def settled_start(spec: RobotSpec) -> tuple[mujoco.MjData, np.ndarray]:
    """The stand keyframe landed on the floor under the drives' leg PD, and the
    torque that held it there at the end (the open-loop sequences' offset)."""
    m = spec.model
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    mujoco.mj_forward(m, d)
    lim = spec.limits
    legs, wheels = spec.leg_index, spec.wheel_index
    tau = np.zeros(6)
    for _ in range(int(round(SETTLE_S / spec.physics_dt))):
        q, v = d.qpos[spec.joint_qpos], d.qvel[spec.joint_dof]
        tau[legs] = lim.leg_kp_nm_rad * (spec.stand_q[legs] - q[legs]) - lim.leg_kd_nm_s_rad * v[legs]
        tau[wheels] = -0.4 * v[wheels]
        d.ctrl[:] = 0.0
        d.ctrl[spec.joint_actuator] = tau
        mujoco.mj_step(m, d)
    hold = tau.copy()
    hold[wheels] = 0.0
    return d, hold


def cpu_rollout(spec: RobotSpec, torques: np.ndarray, noise: float = 0.0, seed: int = 0):
    """Per sequence (qpos before each step, qvel before each step, contacts).
    noise > 0 multiplies the velocities by (1 + noise * N(0, 1)) after every
    step: a float32-sized error injected into the float64 simulation."""
    m = spec.model
    start, _ = settled_start(spec)
    rng = np.random.default_rng(seed)
    out = []
    for seq in torques:
        d = mujoco.MjData(m)
        d.qpos[:], d.qvel[:], d.time = start.qpos, start.qvel, 0.0
        mujoco.mj_forward(m, d)
        qpos, qvel, ncon = [], [], []
        for tau in seq:
            d.ctrl[:] = 0.0
            d.ctrl[spec.joint_actuator] = tau
            qpos.append(d.qpos.copy())
            qvel.append(d.qvel.copy())
            mujoco.mj_step(m, d)
            if noise > 0:
                d.qvel[:] *= 1.0 + noise * rng.normal(size=m.nv)
            ncon.append(d.ncon)
        out.append((np.array(qpos), np.array(qvel), np.array(ncon)))
    return out


def rollout_errors(spec: RobotSpec, qpos_a: np.ndarray, qpos_b: np.ndarray) -> dict:
    """Max differences of [n, steps, nq] trajectories: leg joints, wheels, base position."""
    err = np.abs(qpos_a[..., spec.joint_qpos] - qpos_b[..., spec.joint_qpos])
    return {"legs": float(np.max(err[..., spec.leg_index])),
            "wheels": float(np.max(err[..., spec.wheel_index])),
            "base": float(np.max(np.linalg.norm(qpos_a[..., 0:3] - qpos_b[..., 0:3], axis=-1)))}


def check(scene: Path | str = sources.SCENE, impl: str = "warp", sequences: int = 4, seed: int = 0,
          graph_mode: str | None = None) -> Report:
    """graph_mode None: the training configuration's (configs/base.yaml)."""
    report = Report()
    try:
        spec = RobotSpec.load(scene)
    except ModelContractError as error:
        report.add(f"model contract: {error}", 1, 0, ok=False)
        return report
    report.add("model contract", 0, 0, ok=True)

    import jax
    import jax.numpy as jp
    from mujoco import mjx

    m = spec.model
    if graph_mode is None:
        from . import config as config_mod
        graph_mode = config_mod.load("base.yaml")["env"].get("graph_mode")
    mode = None
    if impl == "warp" and graph_mode:
        import warp as wp
        mode = getattr(wp.JaxCallableGraphMode, graph_mode.upper())
    try:
        mx = mjx.put_model(m, impl=impl, graph_mode=mode)
    except Exception as error:  # the backend cannot represent the model
        report.add(f"mjx.put_model(impl={impl}): {type(error).__name__}: {error}", 1, 0, ok=False)
        return report
    torques = torque_sequences(spec, sequences, ROLLOUT_STEPS, seed)
    cpu = cpu_rollout(spec, torques)
    act = jp.array(spec.joint_actuator)

    def make(_):
        return mjx.make_data(m, impl=impl, naconmax=16 * sequences, njmax=128) if impl == "warp" \
            else mjx.make_data(m, impl=impl)

    data0 = jax.vmap(make)(jp.arange(sequences))

    def set_state(d, qpos, qvel):
        return mjx.forward(mx, d.replace(qpos=qpos, qvel=qvel))

    def step(d, tau):
        d = d.replace(ctrl=jp.zeros(m.nu).at[act].set(tau))
        d = mjx.step(mx, d)
        return d

    set_state_v = jax.jit(jax.vmap(set_state))
    step_v = jax.jit(jax.vmap(step))

    # One step from identical states sampled along the CPU trajectories.
    worst = 0.0
    for k in range(0, ROLLOUT_STEPS, 20):
        qpos = jp.array(np.stack([c[0][k] for c in cpu]), dtype=jp.float32)
        qvel = jp.array(np.stack([c[1][k] for c in cpu]), dtype=jp.float32)
        d = set_state_v(data0, qpos, qvel)
        d = step_v(d, jp.array(torques[:, k], dtype=jp.float32))
        want = np.stack([c[1][k + 1] if k + 1 < ROLLOUT_STEPS else c[1][k] for c in cpu])
        if k + 1 < ROLLOUT_STEPS:
            worst = max(worst, float(np.max(np.abs(np.asarray(d.qvel) - want))))
    report.add("one-step qvel, max |GPU - CPU|", worst, ONE_STEP_QVEL_TOL)

    # Open-loop rollout.
    qpos0 = jp.array(np.stack([c[0][0] for c in cpu]), dtype=jp.float32)
    qvel0 = jp.array(np.stack([c[1][0] for c in cpu]), dtype=jp.float32)
    d = set_state_v(data0, qpos0, qvel0)

    @jax.jit
    def rollout(d, taus):
        def body(d, tau):
            d = jax.vmap(step)(d, tau)
            return d, d.qpos
        return jax.lax.scan(body, d, taus)

    d_end, qpos_gpu = rollout(d, jp.array(np.swapaxes(torques, 0, 1), dtype=jp.float32))
    qpos_gpu = np.swapaxes(np.asarray(qpos_gpu), 0, 1)        # [n, steps, nq]
    after = lambda runs: np.stack([r[0][1:] for r in runs])          # qpos after steps 0..n-2
    qpos_cpu = after(cpu)
    qpos_gpu = qpos_gpu[:, :-1]
    gpu_err = rollout_errors(spec, qpos_gpu, qpos_cpu)
    spread = {k: 0.0 for k in gpu_err}
    for s in range(ROLLOUT_NOISE_SEEDS):
        noisy = rollout_errors(spec, after(cpu_rollout(spec, torques, ROLLOUT_NOISE, seed + 1 + s)), qpos_cpu)
        spread = {k: max(spread[k], noisy[k]) for k in spread}
    units = {"legs": "rad", "wheels": "rad", "base": "m"}
    names = {"legs": "leg joints", "wheels": "wheel angles", "base": "base position"}
    for k in gpu_err:
        report.add(f"{ROLLOUT_STEPS}-step open-loop rollout, {names[k]} max |GPU - CPU| ({units[k]}); "
                   f"CPU float32-noise spread {spread[k]:.3g}",
                   gpu_err[k], ROLLOUT_SPREAD_FACTOR * spread[k] + ROLLOUT_FLOOR[k])
    finite = bool(np.all(np.isfinite(qpos_gpu)))
    report.add("GPU rollout finite", 0 if finite else 1, 0)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--scene", default=str(sources.SCENE))
    parser.add_argument("--impl", default="warp")
    parser.add_argument("--graph-mode", help="Warp CUDA graph mode (default: configs/base.yaml's)")
    parser.add_argument("--json")
    args = parser.parse_args()
    report = check(args.scene, args.impl, graph_mode=args.graph_mode)
    print(report.text())
    print("consistent" if report.passed else "INCONSISTENT: the environment does not match this model")
    if args.json:
        Path(args.json).write_text(json.dumps(report.checks, indent=1))
    return 0 if report.passed else 1


if __name__ == "__main__":
    sys.exit(main())
