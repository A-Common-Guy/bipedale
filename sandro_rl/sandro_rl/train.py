"""Train a policy on the GPU.

    uv run python -m sandro_rl.train configs/flat_tracking.yaml --out policies/flat_tracking
    uv run python -m sandro_rl.train configs/flat_tracking.yaml --set train.iterations=200

Writes into --out: checkpoint.msgpack (+ checkpoint_<iter>.msgpack), meta.json
(config, model fingerprint, environment contract, library versions),
train_log.csv and curves.png.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import time
from pathlib import Path

# MuJoCo Warp allocates outside XLA's pool: XLA gets a fixed share of the GPU
# and the rest stays free for Warp (a growing XLA pool starves Warp).
os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "true")
os.environ.setdefault("XLA_PYTHON_CLIENT_MEM_FRACTION", "0.6")

import jax
import jax.numpy as jp
import numpy as np

from . import checkpoint, config as config_mod, randomize
from .env import SandroEnv
from .ppo import PPO


def build(config: dict, num_envs: int | None = None, seed: int | None = None):
    env = SandroEnv(config)
    n = num_envs or config["train"]["num_envs"]
    env.batch_size = n
    seed = config.get("seed", 0) if seed is None else seed
    key = jax.random.key(seed)
    k_model, k_init, k_runner = jax.random.split(key, 3)
    model, in_axes, model_params = randomize.randomize(env, env.mjx_model, k_model, n)
    agent = PPO(env, config)
    reset, autoreset_step = agent.make_env_fns(model, in_axes, model_params)
    return env, agent, reset, autoreset_step, k_init, k_runner


def curriculum_stages(config: dict) -> list[dict]:
    """configs' `curriculum.stages`: [{iteration: N, set: {dotted.key: value}}], sorted."""
    stages = sorted((config.get("curriculum") or {}).get("stages", []), key=lambda s: s["iteration"])
    return stages


def config_at(config: dict, stages: list[dict], iteration: int) -> tuple[dict, int]:
    """The configuration in effect at an iteration (1-based) and the stage index (-1: none)."""
    import copy

    cfg = copy.deepcopy(config)
    index = -1
    for i, stage in enumerate(stages):
        if iteration > stage["iteration"]:
            for key, value in stage["set"].items():
                config_mod.set_path(cfg, key, value)
            index = i
    return cfg, index


def plot_curves(log_path: Path, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = list(csv.DictReader(log_path.open()))
    if not rows:
        return
    it = np.array([float(r["iteration"]) for r in rows])
    steps = np.array([float(r["env_steps"]) for r in rows]) / 1e6
    get = lambda k: np.array([float(r.get(k, "nan") or "nan") for r in rows])
    panels = [("mean episode return", ["episode_return"]),
              ("mean episode length (s)", ["episode_length_s"]),
              ("falls per 1000 episodes-steps", ["fall_rate"]),
              ("tracking error", ["m/err_forward", "m/err_yaw", "m/err_height"]),
              ("CaT delta (mean)", ["m/cat_delta"]),
              ("action std / learning rate x100", ["action_std", "lr_x100"])]
    fig, axes = plt.subplots(2, 3, figsize=(14, 7))
    for ax, (title, keys) in zip(axes.flat, panels):
        for k in keys:
            if k == "lr_x100":
                ax.plot(steps, get("learning_rate") * 100, label="lr x100")
            else:
                ax.plot(steps, get(k), label=k.replace("m/", ""))
        ax.set_title(title)
        ax.set_xlabel("env steps (M)")
        ax.grid(alpha=0.3)
        if len(keys) > 1:
            ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("config")
    parser.add_argument("--out", help="output folder (default policies/<config name>)")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--resume", help="checkpoint folder to fine-tune from")
    args = parser.parse_args()
    config = config_mod.load(args.config, args.set)
    out = Path(args.out or Path(__file__).resolve().parents[1] / "policies" / config["name"])
    out.mkdir(parents=True, exist_ok=True)
    t = config["train"]

    print(f"devices: {jax.devices()}")
    stages = curriculum_stages(config)
    stage_config, stage = config_at(config, stages, 1)
    env, agent, reset, autoreset_step, k_init, k_runner = build(stage_config)
    print(f"env: {t['num_envs']} envs, policy {1/env.dt:.0f} Hz, {env.decimation} physics steps per action, "
          f"actor obs {env.actor_obs_size}, critic obs {env.critic_obs_size}")
    ts = agent.init(k_init)
    if args.resume:
        payload, meta = checkpoint.load(Path(args.resume))
        checkpoint.check_model(meta)
        from .networks import Normalizer
        ts = ts.replace(params=jax.tree.map(jp.asarray, payload["params"]),
                        actor_norm=Normalizer(**{k: jp.asarray(v) for k, v in payload["actor_norm"].items()}),
                        critic_norm=Normalizer(**{k: jp.asarray(v) for k, v in payload["critic_norm"].items()}))
        ts = ts.replace(opt_state=agent.optimizer.init(ts.params))
        print(f"resumed from {args.resume}")
    runner = agent.init_runner(jax.jit(reset), k_runner)
    iteration = agent.make_iteration(autoreset_step)

    steps_per_iter = t["num_envs"] * t["steps_per_env"]
    log_path = out / "train_log.csv"
    writer = None
    start = time.time()
    compile_s = None
    best = -np.inf
    for i in range(1, t["iterations"] + 1):
        new_config, new_stage = config_at(config, stages, i)
        if new_stage != stage:
            # Rebuild the environment (and its randomised model batch) for the
            # stage; the networks, normalisers and optimiser carry over.
            stage, stage_config = new_stage, new_config
            print(f"curriculum stage {stage} from iteration {i}: {stages[stage]['set']}", flush=True)
            env, agent, reset, autoreset_step, _, _ = build(stage_config, seed=config.get("seed", 0) + 1000 * (stage + 1))
            runner = agent.init_runner(jax.jit(reset), jax.random.fold_in(k_runner, stage + 1))
            iteration = agent.make_iteration(autoreset_step)
        t0 = time.time()
        ts, runner, summary = iteration(ts, runner)
        summary = jax.device_get(summary)
        if compile_s is None:
            compile_s = time.time() - t0
            print(f"first iteration (compile) {compile_s:.1f} s")
        elapsed = time.time() - t0
        episodes = max(float(summary["episodes"]), 1.0)
        row = {
            "iteration": i, "env_steps": i * steps_per_iter, "wall_s": round(time.time() - start, 1),
            "steps_per_s": round(steps_per_iter / elapsed),
            "episode_return": float(summary["return_sum"]) / episodes if summary["episodes"] else float("nan"),
            "episode_length_s": float(summary["length_sum"]) / episodes * env.dt if summary["episodes"] else float("nan"),
            "fall_rate": 1000.0 * float(summary["falls"]) / steps_per_iter,
        }
        row.update({k: float(v) for k, v in summary.items()
                    if k not in ("return_sum", "length_sum", "episodes", "falls")})
        if writer is None:
            handle = log_path.open("w", newline="")
            writer = csv.DictWriter(handle, fieldnames=list(row))
            writer.writeheader()
        writer.writerow(row)
        handle.flush()
        if i % 10 == 0 or i == 1:
            print(f"it {i:5d} | {row['steps_per_s']:>8,d} steps/s | ret {row['episode_return']:7.2f} "
                  f"len {row['episode_length_s']:5.1f}s falls/1k {row['fall_rate']:6.3f} | "
                  f"err v {row['m/err_forward']:.3f} yaw {row['m/err_yaw']:.3f} h {row['m/err_height']:.4f} | "
                  f"cat {row['m/cat_delta']:.4f} kl {row['kl']:.4f} lr {row['learning_rate']:.1e} "
                  f"std {row['action_std']:.3f} sym {row['symmetry']:.4f}", flush=True)
        extra = {"iteration": i, "env_steps": i * steps_per_iter, "wall_s": time.time() - start,
                 "compile_s": compile_s, "curriculum_stage": stage, "base_config": config}
        if i % t["checkpoint_every"] == 0 or i == t["iterations"]:
            checkpoint.save(out, ts, stage_config, env.spec.contract(), extra)
            checkpoint.save(out, ts, stage_config, env.spec.contract(), extra, name=f"checkpoint_{i:05d}")
            plot_curves(log_path, out / "curves.png")
    total = time.time() - start
    rate = t["iterations"] * steps_per_iter / total
    print(f"done: {t['iterations'] * steps_per_iter / 1e6:.1f} M env steps in {total / 60:.1f} min "
          f"({rate:,.0f} env steps/s incl. compile, {rate * env.decimation:,.0f} physics steps/s)")
    (out / "train_summary.json").write_text(json.dumps({
        "env_steps": t["iterations"] * steps_per_iter, "wall_s": total, "env_steps_per_s": rate,
        "physics_steps_per_s": rate * env.decimation, "compile_s": compile_s,
        "num_envs": t["num_envs"], "gpu": str(jax.devices()[0])}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
