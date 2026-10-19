"""Seeded runs repeat.

Everything sampled on the host and by JAX's counter-based generator is
bit-exact. MuJoCo Warp accumulates contact and constraint terms with float
atomics whose order varies between launches, so GPU rollouts agree to
float32 rounding (measured: 6e-7 after one step), not bit for bit; Warp's
run-to-run deterministic mode made the differences larger on this model
(docs/DESIGN.md), so it is not used. The CPU evaluator is bit-exact.
"""

import numpy as np
import pytest

from sandro_rl import config as config_mod

OVERRIDES = ["train.num_envs=64", "train.steps_per_env=8", "train.minibatches=2", "train.epochs=1"]


def _gpu_run(seed):
    import jax
    import jax.numpy as jp
    from sandro_rl import train

    config = config_mod.load("flat_tracking.yaml", OVERRIDES)
    env, agent, reset, autoreset_step, k_init, k_runner = train.build(config, seed=seed)
    step = jax.jit(autoreset_step)
    state = jax.jit(reset)(jax.random.key(seed))
    qpos = []
    for i in range(10):
        state = step(state, jp.full((64, 6), 0.1 * np.sin(i)), jax.random.key(100 + i))
        qpos.append(np.asarray(state.data.qpos))
    ts = agent.init(k_init)
    runner = agent.init_runner(jax.jit(reset), k_runner)
    ts, runner, _ = agent.make_iteration(autoreset_step)(ts, runner)
    params = np.concatenate([np.ravel(x) for x in jax.tree.leaves(jax.device_get(ts.params))])
    return np.stack(qpos), params, np.asarray(env.mjx_model.body_mass)


@pytest.mark.gpu
def test_gpu_same_seed_repeats_to_float_rounding():
    qpos_a, params_a, _ = _gpu_run(3)
    qpos_b, params_b, _ = _gpu_run(3)
    np.testing.assert_allclose(qpos_a, qpos_b, atol=1e-4)
    # Adam's per-parameter normalisation turns float32-rounding differences
    # of near-zero gradients into visible steps on a few weights (measured:
    # 0.05 % of the weights, up to 3e-3); the network as a whole agrees.
    assert np.linalg.norm(params_a - params_b) / np.linalg.norm(params_a) < 1e-3


@pytest.mark.gpu
def test_gpu_different_seed_differs():
    qpos_a, _, _ = _gpu_run(3)
    qpos_c, _, _ = _gpu_run(4)
    assert np.abs(qpos_a - qpos_c).max() > 1e-2


@pytest.mark.gpu
def test_randomisation_is_bit_exact():
    import jax
    from sandro_rl import randomize
    from sandro_rl.env import SandroEnv

    config = config_mod.load("flat_tracking.yaml")
    env = SandroEnv(config)
    a = randomize.randomize(env, env.mjx_model, jax.random.key(7), 32)[2]
    b = randomize.randomize(env, env.mjx_model, jax.random.key(7), 32)[2]
    np.testing.assert_array_equal(np.asarray(a), np.asarray(b))


def test_cpu_evaluation_is_bit_exact(tmp_path):
    from pathlib import Path
    from sandro_rl import evaluate

    policy = Path(__file__).resolve().parents[1] / "policies" / "flat_tracking"
    if not (policy / "policy.onnx").exists():
        pytest.skip("no exported policy")
    a = evaluate.run(policy, "stand", {"settle": "0.5"}, tmp_path / "a.csv")[1].read_text()
    b = evaluate.run(policy, "stand", {"settle": "0.5"}, tmp_path / "b.csv")[1].read_text()
    assert a == b
