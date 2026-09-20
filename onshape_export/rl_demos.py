"""Collect expert demonstrations for behavior cloning.

Runs the cascade controller (drive_controller) under the RL env's command
schedules and records (observation, action) pairs in the ENV's spaces: the
cascade's actuator commands map exactly onto the env action space
(leg ctrl = stand + 0.6*a, wheel ctrl = fwd_map(25*a)), so its behavior is
directly imitable. Jump episodes are excluded (the expert cannot jump).

    python rl_demos.py --episodes 100 --out rl_demos.npz
"""
from __future__ import annotations

import argparse
import multiprocessing as mp

import numpy as np

from rl_env import (WheeledBipedEnv, POLICY_HZ, EP_STEPS,
                    LEG_ACT_SCALE, WHEEL_ACT_SCALE, SIDES, TILT_LIMIT, Z_MIN)


def _episode(seed):
    import drive_controller as dc
    from controller_demo import Robot

    env = WheeledBipedEnv(seed=0, jump_prob=0.0)
    robot = Robot()
    ctl = dc.WholeBodyController(robot)
    # reward/obs math on the controller's model/data
    env.model, env.data = robot.model, robot.data
    env.rng = np.random.default_rng(seed)
    env._step = 0
    env._next_cmd_t = 0.0
    env._jumps = []
    env._max_clear = 0.0
    env._a_prev[:] = 0.0
    env._sample_cmd()
    n_ctl = max(1, round(1 / (POLICY_HZ * ctl.dt)))

    O, A = [], []
    for k in range(EP_STEPS):
        lo, hi = ctl.cmd.h_range
        ctl.cmd.v_tgt = float(env.cmd[0])
        ctl.cmd.yaw_tgt = float(env.cmd[1])
        ctl.cmd.h_tgt = float(np.clip(env.cmd[2], lo, hi))
        obs = env._obs()
        for _ in range(n_ctl):
            ctl.tick()
            robot.step_control_period()
        # expert action = inverse map of the applied actuator commands
        a = np.zeros(6)
        for i, n in enumerate(env.leg_acts):
            a[i] = (robot.data.ctrl[env._act_id[n]]
                    - env.stand_ctrl[env._act_id[n]]) / LEG_ACT_SCALE
        for j, s in enumerate(SIDES):
            w_fwd = env.fwd_sign * env.wheel_dir[s] \
                * robot.data.ctrl[env._act_id[f"{s}_wheel"]]
            a[4 + j] = w_fwd / WHEEL_ACT_SCALE
        a = np.clip(a, -1.0, 1.0)
        O.append(obs)
        A.append(a)
        env._a_prev = a
        env._step += 1
        if env._step / POLICY_HZ >= env._next_cmd_t:
            env._sample_cmd()
        _, _, _, upz = env._true_state()
        if upz < TILT_LIMIT or float(robot.data.qpos[2]) < Z_MIN:
            break
    return np.array(O), np.array(A)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=100)
    ap.add_argument("--out", default="rl_demos.npz")
    ap.add_argument("--workers", type=int, default=8)
    args = ap.parse_args()
    with mp.get_context("fork").Pool(args.workers) as pool:
        results = pool.map(_episode, range(20_000, 20_000 + args.episodes))
    obs = np.concatenate([o for o, _ in results])
    act = np.concatenate([a for _, a in results])
    np.savez_compressed(args.out, obs=obs, act=act)
    print(f"saved {len(obs):,} (obs, action) pairs from "
          f"{args.episodes} expert episodes -> {args.out}")
    print(f"action stats: mean {np.round(act.mean(0), 3)}")


if __name__ == "__main__":
    main()
