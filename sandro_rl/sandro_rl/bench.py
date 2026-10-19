"""Physics throughput of the candidate backends on the generated ROS model.

    uv run python -m sandro_rl.bench                      # all, default batches
    uv run python -m sandro_rl.bench --backends warp --batches 1024 4096 8192

Unpowered robot from the stand keyframe (it lands and tips over, so tyre and
body contacts are both exercised), physics steps only, no policy. MJX's JAX
backend cannot collide the tyre cylinders with the body meshes
(`put_model` raises), so for it the body meshes are taken out of collision;
its numbers are for a simpler model. Reports physics steps per second.
"""

from __future__ import annotations

import argparse
import json
import os
import time

os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")

import mujoco
import numpy as np

from .robot import RobotSpec


def _start(m):
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    return d


def bench_cpu(m, steps=20000) -> float:
    d = _start(m)
    t0 = time.perf_counter()
    for _ in range(steps):
        mujoco.mj_step(m, d)
    return steps / (time.perf_counter() - t0)


def bench_mjx(m, impl: str, batch: int, steps: int = 240, graph_mode: str | None = None,
              naconmax_per_env: int = 12, njmax: int = 96) -> tuple[float, float]:
    import jax
    import jax.numpy as jp
    from mujoco import mjx

    mode = None
    if impl == "warp" and graph_mode:
        import warp as wp
        mode = getattr(wp.JaxCallableGraphMode, graph_mode.upper())
    mx = mjx.put_model(m, impl=impl, graph_mode=mode) if impl == "warp" else mjx.put_model(m, impl=impl)
    qpos = jp.asarray(_start(m).qpos, dtype=jp.float32)

    def make(_):
        if impl == "warp":
            d = mjx.make_data(m, impl=impl, naconmax=naconmax_per_env * batch, njmax=njmax)
        else:
            d = mjx.make_data(m, impl=impl)
        return d.replace(qpos=qpos)

    data = jax.jit(jax.vmap(make))(jp.arange(batch))

    @jax.jit
    def rollout(d):
        def body(d, _):
            return jax.vmap(lambda x: mjx.step(mx, x))(d), None
        return jax.lax.scan(body, d, None, length=steps)[0]

    t0 = time.perf_counter()
    jax.block_until_ready(rollout(data).qpos)
    compile_s = time.perf_counter() - t0
    t0 = time.perf_counter()
    out = rollout(data)
    jax.block_until_ready(out.qpos)
    rate = batch * steps / (time.perf_counter() - t0)
    return rate, compile_s


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--backends", nargs="+", default=["cpu", "warp", "warp_staged", "jax"])
    parser.add_argument("--batches", nargs="+", type=int, default=[1024, 4096, 8192])
    parser.add_argument("--json")
    args = parser.parse_args()
    spec = RobotSpec.load()
    m = spec.model
    results = []
    for backend in args.backends:
        if backend == "cpu":
            rate = bench_cpu(m)
            results.append({"backend": "CPU MuJoCo (1 thread)", "batch": 1, "physics_steps_per_s": rate})
            print(f"CPU MuJoCo, one thread: {rate:,.0f} physics steps/s", flush=True)
            continue
        model = m
        if backend == "jax":
            model = mujoco.MjModel.from_xml_path(str(spec.scene))
            model.geom_contype[spec.body_collision_geoms] = 0
            model.geom_conaffinity[spec.body_collision_geoms] = 0
        for batch in args.batches:
            impl = "jax" if backend == "jax" else "warp"
            graph = {"warp": "warp", "warp_staged": "warp_staged"}.get(backend)
            try:
                rate, compile_s = bench_mjx(model, impl, batch, graph_mode=graph)
            except Exception as error:  # noqa: BLE001 - report and continue
                print(f"{backend} batch {batch}: {type(error).__name__}: {str(error)[:200]}", flush=True)
                continue
            label = {"warp": "MJX-Warp (graph mode warp)", "warp_staged": "MJX-Warp (graph mode warp_staged)",
                     "jax": "MJX-JAX (body meshes not colliding)"}[backend]
            results.append({"backend": label, "batch": batch, "physics_steps_per_s": rate, "compile_s": compile_s})
            print(f"{label}, batch {batch}: {rate:,.0f} physics steps/s (compile {compile_s:.1f} s)", flush=True)
    if args.json:
        with open(args.json, "w") as handle:
            json.dump(results, handle, indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
