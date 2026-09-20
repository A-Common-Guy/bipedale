"""ARS (Augmented Random Search, V2-t) trainer for the wheeled biped env.

Zero-dependency (numpy + multiprocessing) policy search: linear policy on
normalized observations, antithetic direction pairs with common random
numbers (same env seed for +delta and -delta -- essential here because the
command schedule is random), top-b update, running observation stats.

    python rl_train.py                     # train (background-friendly)
    python rl_train.py --iters 300 --out rl_policy.npz
    python rl_train.py --eval rl_policy.npz   # deterministic evaluation
"""
from __future__ import annotations

import argparse
import multiprocessing as mp
import time

import numpy as np

N_DIRS = 32
TOP_B = 16
STEP = 0.015
NOISE = 0.025
WORKERS = 18
EVAL_EVERY = 20
EVAL_EPS = 5

_env = None


def _init_worker():
    global _env
    from rl_env import WheeledBipedEnv
    _env = WheeledBipedEnv()


def _rollout(args):
    W_flat, mu, sd, seed, collect = args
    global _env
    env = _env
    W = W_flat.reshape(6, -1)
    obs, _ = env.reset(seed=int(seed))
    total, steps = 0.0, 0
    s1 = np.zeros_like(mu)
    s2 = np.zeros_like(mu)
    n = 0
    for _ in range(1000):
        if collect:
            s1 += obs
            s2 += obs * obs
            n += 1
        a = np.clip(W @ ((obs - mu) / sd), -1.0, 1.0)
        obs, r, term, trunc, _ = env.step(a)
        total += r
        steps += 1
        if term or trunc:
            break
    return total, steps, s1, s2, n


def train(iters, out, resume=None):
    from rl_env import WheeledBipedEnv
    env = WheeledBipedEnv(seed=0)
    dim = env.obs_dim
    if resume:
        ck = np.load(resume)
        W = ck["W"]
        count, mean, m2 = float(ck["count"]), ck["mu"].copy(), ck["m2"].copy()
        print(f"resumed from {resume}")
    else:
        W = np.zeros((6, dim))
        count, mean, m2 = 1e-4, np.zeros(dim), np.ones(dim) * 1e-4
    rng = np.random.default_rng(0)
    best_eval = -np.inf

    pool = mp.get_context("fork").Pool(WORKERS, initializer=_init_worker)
    log = open(out.replace(".npz", "_log.txt"), "a", buffering=1)
    t0 = time.time()
    for it in range(iters):
        sd = np.sqrt(np.maximum(m2 / count, 1e-8))
        mu = mean.copy()
        deltas = rng.standard_normal((N_DIRS, 6, dim))
        seeds = rng.integers(0, 2**31 - 1, N_DIRS)
        jobs = []
        for k in range(N_DIRS):
            for sgn in (+1.0, -1.0):
                Wk = (W + sgn * NOISE * deltas[k]).ravel()
                jobs.append((Wk, mu, sd, seeds[k], True))
        results = pool.map(_rollout, jobs)
        r_pos = np.array([results[2 * k][0] for k in range(N_DIRS)])
        r_neg = np.array([results[2 * k + 1][0] for k in range(N_DIRS)])
        # observation stats from all rollouts
        for _, _, s1, s2, n in results:
            if n:
                count += n
                mean += (s1 - n * mean) / count
                m2 += s2 - (s1 * s1) / max(n, 1)   # crude but adequate
        m2 = np.maximum(m2, 1e-8)
        # top-b update
        scores = np.maximum(r_pos, r_neg)
        top = np.argsort(-scores)[:TOP_B]
        sig = np.std(np.concatenate([r_pos[top], r_neg[top]])) + 1e-6
        grad = np.zeros_like(W)
        for k in top:
            grad += (r_pos[k] - r_neg[k]) * deltas[k]
        W += STEP / (TOP_B * sig) * grad

        msg = (f"iter {it:4d} t={time.time()-t0:6.0f}s "
               f"ret mean {np.mean(np.concatenate([r_pos, r_neg])):7.1f} "
               f"max {max(r_pos.max(), r_neg.max()):7.1f}")
        if it % EVAL_EVERY == 0 or it == iters - 1:
            evals = pool.map(_rollout, [(W.ravel(), mu, sd, 10_000 + e, False)
                                        for e in range(EVAL_EPS)])
            ev = np.mean([r for r, *_ in evals])
            ev_len = np.mean([s for _, s, *_ in evals])
            msg += f" | eval {ev:7.1f} len {ev_len:5.0f}"
            np.savez(out, W=W, mu=mu, sd=sd, count=count, m2=m2)
            if ev > best_eval:
                best_eval = ev
                np.savez(out.replace(".npz", "_best.npz"),
                         W=W, mu=mu, sd=sd, count=count, m2=m2)
                msg += " *"
        print(msg)
        log.write(msg + "\n")
    pool.close()
    print(f"done, best eval {best_eval:.1f}")


def evaluate(path, episodes=10):
    from rl_env import WheeledBipedEnv, rollout
    ck = np.load(path)
    W, mu, sd = ck["W"], ck["mu"], ck["sd"]
    env = WheeledBipedEnv(seed=1)
    policy = lambda o: np.clip(W @ ((o - mu) / sd), -1, 1)
    rets, lens, errs = [], [], []
    for ep in range(episodes):
        total, steps, mean = rollout(env, policy, seed=10_000 + ep)
        rets.append(total)
        lens.append(steps)
        errs.append(mean)
        print(f"  ep {ep}: return {total:7.1f} steps {steps:4d} "
              f"v_err {mean['v_err']:.3f} yaw_err {mean['yaw_err']:.3f} "
              f"z_err {mean['z_err']*1000:.1f}mm")
    print(f"policy: return {np.mean(rets):7.1f} +- {np.std(rets):5.1f}, "
          f"len {np.mean(lens):5.0f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=400)
    ap.add_argument("--out", default="rl_policy.npz")
    ap.add_argument("--resume", default=None)
    ap.add_argument("--eval", default=None)
    args = ap.parse_args()
    if args.eval:
        evaluate(args.eval)
    else:
        train(args.iters, args.out, args.resume)
