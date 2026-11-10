"""FPO++ (fpo.py) on a toy problem, and the export of its flow.

The toy is a contextual bandit with one-step episodes through ppo.py's
rollout: the observation is a random context and the reward minus the
squared distance of the action to a target that depends on it.
"""

import jax
import jax.numpy as jp
import numpy as np
import pytest

from sandro_rl.env import State
from sandro_rl.fpo import FlowPPO, reference_action

N_ENVS = 256


class Bandit:
    n_actions = 6
    actor_obs_size = 2
    critic_obs_size = 2

    @staticmethod
    def target(c):
        return 0.5 * jp.stack([c[..., 0], -c[..., 1], c[..., 0] * c[..., 1],
                               jp.full(c.shape[:-1], 0.3), jp.full(c.shape[:-1], -0.3), c[..., 1]], -1)

    def state(self, key, action=None, context=None):
        reward = jp.zeros(N_ENVS) if action is None else -jp.sum((action - self.target(context)) ** 2, -1)
        c = jax.random.uniform(key, (N_ENVS, 2), minval=-1, maxval=1)
        # Separate buffers: the iteration donates the runner's state. Every
        # step ends its episode (terminated, so nothing is bootstrapped).
        return State(data=jp.zeros(N_ENVS), obs={"actor": c, "critic": jp.array(c, copy=True)}, reward=reward,
                     done=jp.ones(N_ENVS), terminated=jp.ones(N_ENVS), cat_delta=jp.zeros(N_ENVS), info={},
                     metrics={"error": jp.zeros(N_ENVS) if action is None else -reward})

    def autoreset_step(self, state, action, key):
        return self.state(key, action, state.obs["actor"])


MODE = jp.array([0.6, -0.6, 0.6, -0.6, 0.6, -0.6])


class TwoWays(Bandit):
    """Two equally good actions, +MODE and -MODE, whatever the context: a
    pillar passed on the left or on the right."""

    def state(self, key, action=None, context=None):
        s = super().state(key)
        if action is None:
            return s
        reward = -jp.minimum(jp.sum((action - MODE) ** 2, -1), jp.sum((action + MODE) ** 2, -1))
        return s.replace(reward=reward, metrics={"error": -reward})


def _agent(env=None, **fpo):
    config = {"train": {"actor_hidden": [64, 64], "critic_hidden": [64, 64], "activation": "elu",
                        "max_grad_norm": 1.0, "gamma": 0.99, "lam": 0.95, "value_coef": 1.0,
                        "steps_per_env": 4, "epochs": 4, "minibatches": 4, "symmetry_coef": 0.0,
                        "fpo": {"flow_steps": 8, "deploy_steps": 8, "mc_samples": 8, "learning_rate": 1e-3,
                                "clip": 0.05, **fpo}}}
    env = env or Bandit()
    agent = FlowPPO(env, config)
    ts = agent.init(jax.random.key(0))
    runner = agent.init_runner(lambda key: env.state(key), jax.random.key(1))
    return env, agent, ts, runner


def _error(env, agent, ts):
    c = jax.random.uniform(jax.random.key(99), (512, 2), minval=-1, maxval=1)
    return float(jp.mean(jp.sum((agent.act(ts, c) - env.target(c)) ** 2, -1)))


def test_flow_policy_learns_the_bandit():
    env, agent, ts, runner = _agent()
    iteration = agent.make_iteration(env.autoreset_step)
    before = _error(env, agent, ts)
    for _ in range(150):
        ts, runner, summary = iteration(ts, runner)
    after = _error(env, agent, ts)
    assert np.isfinite(float(summary["surrogate"]))
    # Measured: 0.32 -> 0.037 (Gaussian PPO on the same budget: 0.0007).
    assert after < 0.15 * before, f"zero-sampled action error {before:.3f} -> {after:.3f}"
    # The sampled actions stay spread around the deterministic one: no collapse.
    assert 0.01 < float(summary["action_std"]) < 1.0


def test_flow_policy_keeps_both_ways():
    """Gaussian PPO leans to one of the two actions (measured, two seeds: 16
    and 20 % of samples on the + side, mean reward -0.23 and -0.21); the
    flow keeps both (53 and 48 %, -0.03 and -0.02), and its zero-sampled
    action still lands on one of them."""
    env, agent, ts, runner = _agent(TwoWays())
    iteration = agent.make_iteration(env.autoreset_step)
    for _ in range(400):
        ts, runner, _ = iteration(ts, runner)
    c = jax.random.uniform(jax.random.key(7), (2048, 2), minval=-1, maxval=1)
    action, _ = agent.sample(ts, c, jax.random.key(8))
    plus = float(np.mean(np.asarray(action @ MODE) > 0))
    assert 0.25 < plus < 0.75, f"{plus:.2f} of the sampled actions on the + side"
    det = agent.act(ts, c)
    error = float(jp.mean(jp.minimum(jp.sum((det - MODE) ** 2, -1), jp.sum((det + MODE) ** 2, -1))))
    assert error < 0.2, f"zero-sampled action {error:.3f} from the nearest way (2.16 between them)"


def test_ratios_are_one_on_policy():
    env, agent, ts, runner = _agent()
    agent.cfg.update({"epochs": 1, "minibatches": 1})
    _, _, summary = agent.make_iteration(env.autoreset_step)(ts, runner)
    assert float(summary["kl"]) == pytest.approx(0.0, abs=1e-7)
    assert float(summary["clip_fraction"]) == 0.0


def test_exported_flow_matches_the_policy(tmp_path):
    import onnxruntime as ort
    import onnx
    from sandro_rl.fpo import build_onnx

    env, agent, ts, _ = _agent(deploy_steps=5)
    # Larger output weights than the init, so the flow moves the action.
    params = jax.tree.map(lambda x: x * 30.0 if x.ndim == 2 and x.shape[1] == env.n_actions else x, ts.params)
    ts = ts.replace(params=params)
    payload = {"params": jax.device_get(ts.params),
               "actor_norm": {"mean": np.array([0.1, -0.2]), "var": np.array([0.5, 2.0])}}
    from sandro_rl.networks import Normalizer
    ts = ts.replace(actor_norm=Normalizer(jp.asarray(payload["actor_norm"]["mean"]),
                                          jp.asarray(payload["actor_norm"]["var"]), jp.float32(1.0)))
    model = build_onnx(payload, agent.cfg, 10.0, 1.0)
    path = tmp_path / "flow.onnx"
    onnx.save(model, path)
    obs = np.random.default_rng(0).normal(size=(64, 2)).astype(np.float32)
    got = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"]).run(None, {"obs": obs})[0]
    want = reference_action(payload, agent.cfg, obs.astype(np.float64))
    jax_action = np.clip(np.asarray(agent.act(ts, jp.asarray(obs))), -1, 1)
    assert np.max(np.abs(want)) > 0.05, "the test flow barely moves the action"
    np.testing.assert_allclose(got, want, atol=1e-5)
    np.testing.assert_allclose(jax_action, want, atol=1e-5)
