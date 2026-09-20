"""Run a trained RL policy: viewer teleop or headless evaluation.

    python rl_play.py rl_policy_best.npz            # viewer + teleop keys
    python rl_play.py rl_ppo_policy.pt              # same, PPO checkpoint
    python rl_play.py <policy> --headless 20        # no viewer, print stats

Teleop keys match the controllers: W/S speed, A/D yaw, R/F height, X stop.
"""
from __future__ import annotations

import argparse
import time

import numpy as np

from rl_env import (WheeledBipedEnv, V_RANGE, YAW_RANGE, H_RANGE,
                    POLICY_HZ, rollout)


def load_policy(path):
    if path.endswith(".npz"):
        ck = np.load(path)
        W, mu, sd = ck["W"], ck["mu"], ck["sd"]
        return lambda o: np.clip(W @ ((o - mu) / sd), -1, 1)
    if path.endswith(".pt"):
        import torch
        ck = torch.load(path, map_location="cpu", weights_only=False)
        mu = np.asarray(ck["obs_mean"], dtype=np.float64)
        if "obs_std" in ck:
            sd = np.asarray(ck["obs_std"], dtype=np.float64)
        else:
            sd = np.sqrt(np.asarray(ck["obs_var"], dtype=np.float64))
        sizes = ck.get("hidden", [128, 128])
        layers = []
        dims = [len(mu)] + list(sizes)
        import torch.nn as nn
        net = []
        for i in range(len(dims) - 1):
            net += [nn.Linear(dims[i], dims[i + 1]), nn.Tanh()]
        net += [nn.Linear(dims[-1], 6)]
        actor = nn.Sequential(*net)
        actor.load_state_dict(ck["actor"])
        actor.eval()

        def policy(o):
            x = torch.as_tensor((o - mu) / np.maximum(sd, 1e-6),
                                dtype=torch.float32)
            with torch.no_grad():
                a = actor(x).numpy()
            return np.clip(a, -1, 1)
        return policy
    raise SystemExit(f"unknown policy format: {path}")


def teleop(policy):
    import mujoco.viewer
    env = WheeledBipedEnv(seed=0)
    obs, _ = env.reset(seed=0)
    cmd = dict(v=0.0, yaw=0.0, h=0.15)

    def key(code):
        c = chr(code).upper() if 0 <= code < 256 else ""
        if c == "W":
            cmd["v"] = np.clip(cmd["v"] + 0.3, -V_RANGE, V_RANGE)
        elif c == "S":
            cmd["v"] = np.clip(cmd["v"] - 0.3, -V_RANGE, V_RANGE)
        elif c == "A":
            cmd["yaw"] = np.clip(cmd["yaw"] + 0.4, -YAW_RANGE, YAW_RANGE)
        elif c == "D":
            cmd["yaw"] = np.clip(cmd["yaw"] - 0.4, -YAW_RANGE, YAW_RANGE)
        elif c == "R":
            cmd["h"] = np.clip(cmd["h"] + 0.02, *H_RANGE)
        elif c == "F":
            cmd["h"] = np.clip(cmd["h"] - 0.02, *H_RANGE)
        elif c == "J":
            # schedule a jump: full 0.4 s pre-window ramp, then the window
            t_now = env._step / POLICY_HZ
            env._jumps.append(t_now + 0.45)
            print("\nJUMP cued")
        elif c == "X":
            cmd["v"] = cmd["yaw"] = 0.0

    with mujoco.viewer.launch_passive(env.model, env.data,
                                      key_callback=key) as viewer:
        obs, _ = env.reset(seed=0)
        env._jumps = []
        while viewer.is_running():
            # freeze the env's internal command schedule; keys rule
            env.cmd = np.array([cmd["v"], cmd["yaw"], cmd["h"]])
            env._next_cmd_t = 1e9
            obs, r, term, trunc, info = env.step(policy(obs))
            viewer.sync()
            if term:
                print("\nfell -- resetting")
                obs, _ = env.reset()
            time.sleep(max(0.0, 1.0 / POLICY_HZ - 0.005))
            print(f"\rv={env.cmd[0]:+.1f} yaw={env.cmd[1]:+.1f} h={env.cmd[2]:.2f} "
                  f"| v_err={info['v_err']:.2f} up={info['upz']:.2f}   ",
                  end="", flush=True)


def headless(policy, episodes):
    env = WheeledBipedEnv(seed=1)
    rets, lens = [], []
    for ep in range(episodes):
        total, steps, mean = rollout(env, policy, seed=10_000 + ep)
        rets.append(total)
        lens.append(steps)
        print(f"  ep {ep}: return {total:7.1f} steps {steps:4d} "
              f"v_err {mean['v_err']:.3f} yaw_err {mean['yaw_err']:.3f} "
              f"z_err {mean['z_err']*1000:.1f}mm")
    print(f"return {np.mean(rets):7.1f} +- {np.std(rets):5.1f}, "
          f"len {np.mean(lens):5.0f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("policy")
    ap.add_argument("--headless", type=int, default=0, metavar="EPISODES")
    args = ap.parse_args()
    pol = load_policy(args.policy)
    if args.headless:
        headless(pol, args.headless)
    else:
        teleop(pol)
