"""PPO trainer for the wheeled biped env (torch, CPU).

Standard PPO with GAE on 12 parallel WheeledBipedEnv workers (fork +
pipes; each env lives in its own process, created inside the worker).
MLP actor-critic (2x128 tanh), diagonal Gaussian with state-independent
learnable log_std, running observation normalization (frozen at eval).
Optional behavior-cloning warm start from expert demonstrations.

    python rl_ppo.py                          # train from scratch
    python rl_ppo.py --bc rl_demos.npz --bc-out rl_ppo_bc.pt
                                              # BC-pretrain actor -> ckpt
    python rl_ppo.py --resume rl_ppo_bc.pt --warm-lr 1e-4
                                              # PPO fine-tune from BC init
    python rl_ppo.py --eval rl_ppo_policy.pt  # deterministic evaluation
                                              # (seeds 10000..10009, same
                                              # convention as rl_train.py)
"""
from __future__ import annotations

import argparse
import multiprocessing as mp
import os
import time

import numpy as np

# ----------------------------------------------------------------- config
N_ENVS = 12
ROLLOUT = 1024                 # steps per env per update
BATCH = N_ENVS * ROLLOUT       # 12288 transitions / update
MINIBATCH = 4096
EPOCHS = 4
LR = 3e-4
GAMMA = 0.99
GAE_LAMBDA = 0.95
CLIP = 0.2
ENT_COEF = 0.003
VF_COEF = 0.5
MAX_GRAD_NORM = 0.5
LOG_STD_INIT = -0.5
HIDDEN = (128, 128)
TOTAL_STEPS = 4_000_000
MAX_WALL_S = 40 * 60
CKPT_EVERY = 20                # updates
OUT = "rl_ppo_policy.pt"
LOG = "rl_ppo_log.txt"
OBS_CLIP = 10.0


# ------------------------------------------------------------ env workers
def _worker(remote, seeds):
    """One process holding several envs (fat workers amortize IPC and
    scheduler stalls when the machine is contended)."""
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    from rl_env import WheeledBipedEnv
    envs = [WheeledBipedEnv(seed=s) for s in seeds]
    obs = [e.reset(seed=s)[0] for e, s in zip(envs, seeds)]
    remote.send(np.stack(obs))
    k = len(envs)
    while True:
        cmd, data = remote.recv()
        if cmd == "step":
            obs, rew, term, trunc, finals = [], [], [], [], []
            for e, a in zip(envs, data):
                o, r, tm, tr, _ = e.step(a)
                f = None
                if tm or tr:
                    f = o
                    o, _ = e.reset()        # keep the worker's rng stream
                obs.append(o)
                rew.append(r)
                term.append(tm)
                trunc.append(tr)
                finals.append(f)
            remote.send((np.stack(obs), np.array(rew),
                         np.array(term, bool), np.array(trunc, bool), finals))
        elif cmd == "close":
            remote.close()
            break


class VecEnv:
    """Minimal synchronous multi-process vector env with same-step
    autoreset: on term/trunc the returned obs is the *reset* obs and the
    true final obs rides along for value bootstrapping.

    n_envs are distributed over n_workers processes."""

    def __init__(self, n, base_seed=0, n_workers=None):
        n_workers = min(n_workers or n, n)
        chunks = np.array_split(np.arange(n), n_workers)
        ctx = mp.get_context("fork")
        self.remotes, self.procs, self.chunks = [], [], chunks
        for c in chunks:
            parent, child = ctx.Pipe()
            p = ctx.Process(target=_worker,
                            args=(child, [base_seed + int(i) for i in c]),
                            daemon=True)
            p.start()
            child.close()
            self.remotes.append(parent)
            self.procs.append(p)
        self.obs = np.concatenate([r.recv() for r in self.remotes])

    def step(self, actions):
        for r, c in zip(self.remotes, self.chunks):
            r.send(("step", actions[c[0]:c[-1] + 1]))
        out = [r.recv() for r in self.remotes]
        obs = np.concatenate([o[0] for o in out])
        rew = np.concatenate([o[1] for o in out])
        term = np.concatenate([o[2] for o in out])
        trunc = np.concatenate([o[3] for o in out])
        finals = [f for o in out for f in o[4]]
        self.obs = obs
        return obs, rew, term, trunc, finals

    def close(self):
        for r in self.remotes:
            try:
                r.send(("close", None))
            except (BrokenPipeError, OSError):
                pass
        for p in self.procs:
            p.join(timeout=2)


# --------------------------------------------------------------- policy
class RunningNorm:
    def __init__(self, dim):
        self.mean = np.zeros(dim)
        self.var = np.ones(dim)
        self.count = 1e-4

    def update(self, x):
        x = np.atleast_2d(x)
        b_mean, b_var, b_n = x.mean(0), x.var(0), x.shape[0]
        delta = b_mean - self.mean
        tot = self.count + b_n
        self.mean += delta * b_n / tot
        m_a = self.var * self.count
        m_b = b_var * b_n
        self.var = (m_a + m_b + delta ** 2 * self.count * b_n / tot) / tot
        self.count = tot

    def norm(self, x):
        return np.clip((x - self.mean) / np.sqrt(self.var + 1e-8),
                       -OBS_CLIP, OBS_CLIP)


def build_nets(obs_dim, act_dim, hidden):
    import torch
    import torch.nn as nn

    def mlp(out_dim, out_gain):
        layers, d = [], obs_dim
        for h in hidden:
            lin = nn.Linear(d, h)
            nn.init.orthogonal_(lin.weight, np.sqrt(2))
            nn.init.zeros_(lin.bias)
            layers += [lin, nn.Tanh()]
            d = h
        last = nn.Linear(d, out_dim)
        nn.init.orthogonal_(last.weight, out_gain)
        nn.init.zeros_(last.bias)
        layers.append(last)
        return nn.Sequential(*layers)

    class ActorCritic(nn.Module):
        def __init__(self):
            super().__init__()
            self.actor = mlp(act_dim, 0.01)
            self.critic = mlp(1, 1.0)
            self.log_std = nn.Parameter(
                torch.full((act_dim,), float(LOG_STD_INIT)))

        def dist(self, obs):
            mean = self.actor(obs)
            std = torch.exp(torch.clamp(self.log_std, -4.0, 1.0))
            return torch.distributions.Normal(mean, std)

        def value(self, obs):
            return self.critic(obs).squeeze(-1)

    return ActorCritic()


# ---------------------------------------------------------------- training
def train(args):
    import torch
    torch.set_num_threads(args.threads)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    vec = VecEnv(N_ENVS, base_seed=args.seed, n_workers=args.workers)
    obs_dim, act_dim = vec.obs.shape[1], 6
    ac = build_nets(obs_dim, act_dim, HIDDEN)
    opt = torch.optim.Adam(ac.parameters(), lr=LR, eps=1e-5)
    rn = RunningNorm(obs_dim)

    n_updates = (args.steps + BATCH - 1) // BATCH
    log = open(LOG, "a", buffering=1)
    log.write(f"# PPO start seed={args.seed} envs={N_ENVS} "
              f"workers={args.workers} threads={args.threads} "
              f"rollout={ROLLOUT} lr={LR} ent={args.ent} "
              f"warm_lr={args.warm_lr} warm_steps={args.warm_steps}\n")

    ep_ret = np.zeros(N_ENVS)
    ep_len = np.zeros(N_ENVS, dtype=int)
    recent = []                       # (return, length) of finished episodes

    def save_ckpt(path, steps, update):
        tmp = path + ".tmp"
        torch.save(dict(actor=ac.actor.state_dict(),
                        critic=ac.critic.state_dict(),
                        log_std=ac.log_std.detach().clone(),
                        optimizer=opt.state_dict(),
                        obs_mean=rn.mean.copy(),
                        obs_var=rn.var.copy(),
                        obs_count=rn.count,
                        hidden=list(HIDDEN),
                        obs_dim=obs_dim, act_dim=act_dim,
                        steps=steps, update=update), tmp)
        os.replace(tmp, path)

    global_steps = 0
    start_update = 1
    if args.resume and os.path.exists(args.resume):
        ck = torch.load(args.resume, map_location="cpu", weights_only=False)
        ac.actor.load_state_dict(ck["actor"])
        ac.critic.load_state_dict(ck["critic"])
        with torch.no_grad():
            ac.log_std.copy_(ck["log_std"])
        if "optimizer" in ck:
            opt.load_state_dict(ck["optimizer"])
        rn.mean = np.array(ck["obs_mean"], dtype=float)
        rn.var = np.array(ck["obs_var"], dtype=float)
        rn.count = float(ck["obs_count"])
        global_steps = int(ck.get("steps", 0))
        start_update = int(ck.get("update", 0)) + 1
        msg = (f"# resumed {args.resume}: {global_steps} steps, "
               f"continuing at update {start_update}")
        print(msg, flush=True)
        log.write(msg + "\n")

    t0 = time.time()
    t_ckpt = t0
    base_steps = global_steps
    update = start_update - 1
    rn.update(vec.obs)
    obs_n = rn.norm(vec.obs)

    for update in range(start_update, n_updates + 1):
        frac = 1.0 - (update - 1) / n_updates
        lr_now = (args.warm_lr if args.warm_lr and global_steps < args.warm_steps
                  else LR * frac)
        for g in opt.param_groups:
            g["lr"] = lr_now

        b_obs = np.zeros((ROLLOUT, N_ENVS, obs_dim), np.float32)
        b_act = np.zeros((ROLLOUT, N_ENVS, act_dim), np.float32)
        b_logp = np.zeros((ROLLOUT, N_ENVS), np.float32)
        b_rew = np.zeros((ROLLOUT, N_ENVS), np.float32)
        b_val = np.zeros((ROLLOUT, N_ENVS), np.float32)
        b_done = np.zeros((ROLLOUT, N_ENVS), bool)
        b_term = np.zeros((ROLLOUT, N_ENVS), bool)
        boot_obs, boot_idx = [], []   # normalized final obs of truncations

        with torch.no_grad():
            for t in range(ROLLOUT):
                to = torch.as_tensor(obs_n, dtype=torch.float32)
                dist = ac.dist(to)
                act = dist.sample()
                logp = dist.log_prob(act).sum(-1)
                val = ac.value(to)
                a_np = np.clip(act.numpy(), -1.0, 1.0)

                nobs, rew, term, trunc, finals = vec.step(a_np)
                b_obs[t] = obs_n
                b_act[t] = act.numpy()
                b_logp[t] = logp.numpy()
                b_val[t] = val.numpy()
                b_rew[t] = rew
                done = term | trunc
                b_done[t] = done
                b_term[t] = term
                for i in range(N_ENVS):
                    if trunc[i] and not term[i]:
                        boot_obs.append(rn.norm(finals[i]))
                        boot_idx.append((t, i))
                ep_ret += rew
                ep_len += 1
                for i in np.nonzero(done)[0]:
                    recent.append((ep_ret[i], ep_len[i]))
                    ep_ret[i] = 0.0
                    ep_len[i] = 0
                rn.update(nobs)
                obs_n = rn.norm(nobs)
                global_steps += N_ENVS

            last_val = ac.value(
                torch.as_tensor(obs_n, dtype=torch.float32)).numpy()
            boot_val = np.zeros((ROLLOUT, N_ENVS), np.float32)
            if boot_obs:
                bv = ac.value(torch.as_tensor(
                    np.array(boot_obs), dtype=torch.float32)).numpy()
                for (t, i), v in zip(boot_idx, bv):
                    boot_val[t, i] = v

        # GAE (bootstrap through truncations, cut through terminations)
        adv = np.zeros((ROLLOUT, N_ENVS), np.float32)
        lastgaelam = np.zeros(N_ENVS, np.float32)
        for t in reversed(range(ROLLOUT)):
            nextval = b_val[t + 1] if t < ROLLOUT - 1 else last_val
            nextval = np.where(b_done[t],
                               np.where(b_term[t], 0.0, boot_val[t]),
                               nextval)
            nonterminal = (~b_done[t]).astype(np.float32)
            delta = b_rew[t] + GAMMA * nextval - b_val[t]
            lastgaelam = delta + GAMMA * GAE_LAMBDA * nonterminal * lastgaelam
            adv[t] = lastgaelam
        ret = adv + b_val

        f_obs = torch.as_tensor(b_obs.reshape(BATCH, obs_dim))
        f_act = torch.as_tensor(b_act.reshape(BATCH, act_dim))
        f_logp = torch.as_tensor(b_logp.reshape(BATCH))
        f_adv = torch.as_tensor(adv.reshape(BATCH))
        f_ret = torch.as_tensor(ret.reshape(BATCH))

        idx = np.arange(BATCH)
        pl_s, vl_s, ent_s, nmb = 0.0, 0.0, 0.0, 0
        for _ in range(EPOCHS):
            np.random.shuffle(idx)
            for s in range(0, BATCH, MINIBATCH):
                mb = idx[s:s + MINIBATCH]
                dist = ac.dist(f_obs[mb])
                logp = dist.log_prob(f_act[mb]).sum(-1)
                ent = dist.entropy().sum(-1).mean()
                ratio = torch.exp(logp - f_logp[mb])
                madv = f_adv[mb]
                madv = (madv - madv.mean()) / (madv.std() + 1e-8)
                pg = -torch.min(ratio * madv,
                                torch.clamp(ratio, 1 - CLIP, 1 + CLIP) * madv
                                ).mean()
                v = ac.value(f_obs[mb])
                vloss = 0.5 * ((v - f_ret[mb]) ** 2).mean()
                loss = pg + VF_COEF * vloss - args.ent * ent
                opt.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(ac.parameters(), MAX_GRAD_NORM)
                opt.step()
                pl_s += pg.item()
                vl_s += vloss.item()
                ent_s += ent.item()
                nmb += 1

        window = recent[-20:]
        mret = np.mean([r for r, _ in window]) if window else float("nan")
        mlen = np.mean([l for _, l in window]) if window else float("nan")
        el = time.time() - t0
        msg = (f"upd {update:4d} steps {global_steps:8d} t={el:6.0f}s "
               f"sps {(global_steps-base_steps)/el:6.0f} | ep_ret {mret:7.1f} "
               f"ep_len {mlen:5.0f} (n={len(window)}) | "
               f"pi {pl_s/nmb:+.4f} v {vl_s/nmb:8.3f} ent {ent_s/nmb:6.3f} "
               f"lr {lr_now:.2e} std {torch.exp(ac.log_std).mean():.3f}")
        print(msg, flush=True)
        log.write(msg + "\n")

        if time.time() - t_ckpt > 100:
            save_ckpt(OUT, global_steps, update)
            t_ckpt = time.time()
        if global_steps >= args.steps or el > args.max_minutes * 60:
            break

    save_ckpt(OUT, global_steps, update)
    el = time.time() - t0
    final = (f"# done: {global_steps} env steps in {el:.0f}s "
             f"({el/60:.1f} min), saved {OUT}")
    print(final, flush=True)
    log.write(final + "\n")
    log.close()
    vec.close()


# ----------------------------------------------------- behavior cloning
BC_LOG_STD = -1.0          # log_std after BC: keep PPO close to the prior
PHASE_DIM = 26             # jump-phase obs index (constant 0 in demos)
PHASE_STD = 0.35           # natural scale for the [0, 1] phase signal


def behavior_clone(args):
    """Pretrain the actor with MSE on expert (obs, act) pairs; save a
    checkpoint in the trainer's format so --resume / --eval work on it."""
    import torch
    torch.set_num_threads(max(args.threads, 2))
    torch.manual_seed(args.seed)
    d = np.load(args.bc)
    obs = np.asarray(d["obs"], dtype=np.float64)
    act = np.asarray(d["act"], dtype=np.float64)
    n, obs_dim = obs.shape
    act_dim = act.shape[1]
    print(f"BC: {n:,} pairs, obs {obs_dim}, act {act_dim}")

    mean, var = obs.mean(0), obs.var(0)
    var = np.maximum(var, 1e-6)
    # demos are jump-free -> phase is constant; give it its natural scale
    # instead of a degenerate one so the phase input is usable during PPO
    var[PHASE_DIM] = max(var[PHASE_DIM], PHASE_STD ** 2)
    obs_n = np.clip((obs - mean) / np.sqrt(var + 1e-8), -OBS_CLIP, OBS_CLIP)

    rng = np.random.default_rng(args.seed)
    perm = rng.permutation(n)
    n_val = max(1000, n // 20)
    vi, ti = perm[:n_val], perm[n_val:]
    to = torch.as_tensor(obs_n[ti], dtype=torch.float32)
    ta = torch.as_tensor(act[ti], dtype=torch.float32)
    vo = torch.as_tensor(obs_n[vi], dtype=torch.float32)
    va = torch.as_tensor(act[vi], dtype=torch.float32)

    ac = build_nets(obs_dim, act_dim, HIDDEN)
    opt = torch.optim.Adam(ac.actor.parameters(), lr=1e-3)
    best_val, best_state, bad = np.inf, None, 0
    bs = 1024
    for epoch in range(200):
        idx = torch.randperm(len(to))
        for s in range(0, len(to), bs):
            mb = idx[s:s + bs]
            loss = ((ac.actor(to[mb]) - ta[mb]) ** 2).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
        with torch.no_grad():
            val = ((ac.actor(vo) - va) ** 2).mean().item()
        if val < best_val - 1e-5:
            best_val, bad = val, 0
            best_state = {k: v.clone() for k, v in ac.actor.state_dict().items()}
        else:
            bad += 1
        if epoch % 5 == 0 or bad >= 8:
            print(f"  epoch {epoch:3d} val MSE {val:.5f} (best {best_val:.5f})",
                  flush=True)
        if bad >= 8:
            break
    ac.actor.load_state_dict(best_state)
    with torch.no_grad():
        ac.log_std.fill_(BC_LOG_STD)
    torch.save(dict(actor=ac.actor.state_dict(),
                    critic=ac.critic.state_dict(),
                    log_std=ac.log_std.detach().clone(),
                    obs_mean=mean, obs_var=var, obs_count=float(n),
                    hidden=list(HIDDEN), obs_dim=obs_dim, act_dim=act_dim,
                    steps=0, update=0, bc_val_mse=best_val), args.bc_out)
    print(f"BC done: best val MSE {best_val:.5f} -> {args.bc_out}")


# -------------------------------------------------------------- evaluation
def evaluate(path, episodes=10):
    import torch
    torch.set_num_threads(4)
    from rl_env import WheeledBipedEnv, EP_STEPS

    ck = torch.load(path, map_location="cpu", weights_only=False)
    ac = build_nets(ck["obs_dim"], ck["act_dim"], tuple(ck["hidden"]))
    ac.actor.load_state_dict(ck["actor"])
    ac.eval()
    mu, var = ck["obs_mean"], ck["obs_var"]

    def policy(o):
        on = np.clip((o - mu) / np.sqrt(var + 1e-8), -OBS_CLIP, OBS_CLIP)
        with torch.no_grad():
            a = ac.actor(torch.as_tensor(on, dtype=torch.float32)).numpy()
        return np.clip(a, -1.0, 1.0)

    env = WheeledBipedEnv(seed=1)
    rets, lens, clears = [], [], []
    for ep in range(episodes):
        obs, _ = env.reset(seed=10_000 + ep)
        total, steps, infos = 0.0, 0, []
        for _ in range(EP_STEPS):
            obs, r, term, trunc, info = env.step(policy(obs))
            total += r
            steps += 1
            infos.append(info)
            if term or trunc:
                break
        mean = {k: float(np.mean([i[k] for i in infos]))
                for k in ("v_err", "yaw_err", "z_err")}
        clear = infos[-1].get("max_clear", 0.0)   # running max -> last step
        n_jumps = len(env._jumps)
        rets.append(total)
        lens.append(steps)
        clears.append(clear)
        print(f"  ep {ep}: return {total:7.1f} steps {steps:4d} "
              f"v_err {mean['v_err']:.3f} yaw_err {mean['yaw_err']:.3f} "
              f"z_err {mean['z_err']*1000:.1f}mm "
              f"max_clear {clear*1000:5.1f}mm (jumps {n_jumps})")
    print(f"policy: return {np.mean(rets):7.1f} +- {np.std(rets):5.1f}, "
          f"len {np.mean(lens):5.0f}, max_clear {np.mean(clears)*1000:.1f}mm")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--eval", default=None)
    ap.add_argument("--steps", type=int, default=TOTAL_STEPS)
    ap.add_argument("--max-minutes", type=float, default=MAX_WALL_S / 60)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=6,
                    help="processes; N_ENVS envs are split across them")
    ap.add_argument("--threads", type=int, default=1)
    ap.add_argument("--resume", default=None,
                    help="checkpoint .pt to continue training from")
    ap.add_argument("--bc", default=None,
                    help="demos .npz: run behavior cloning and exit")
    ap.add_argument("--bc-out", default="rl_ppo_bc.pt")
    ap.add_argument("--ent", type=float, default=ENT_COEF)
    ap.add_argument("--warm-lr", type=float, default=0.0,
                    help="fixed lr for the first --warm-steps env steps")
    ap.add_argument("--warm-steps", type=int, default=2_000_000)
    args = ap.parse_args()
    if args.bc:
        behavior_clone(args)
    elif args.eval:
        evaluate(args.eval)
    else:
        train(args)
