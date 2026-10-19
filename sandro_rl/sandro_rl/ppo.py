"""PPO for the batched Sandro environment, in JAX.

The rsl_rl recipe (Rudin et al. 2021): GAE, clipped surrogate and value
loss, an adaptive learning rate from the KL divergence, a state-independent
Gaussian. Additions:
  - asymmetric actor-critic: the actor sees obs["actor"] (robot sensors),
    the critic obs["critic"] (privileged);
  - Constraints as Terminations (Chane-Sane et al. 2024): the environment
    reports a termination probability delta per step and the return is
    discounted by gamma * (1 - delta);
  - a mirror loss on the actor mean (Mittal et al. 2024) using the robot's
    left-right symmetry.
One iteration (rollout plus update) is a single jitted function.
"""

from __future__ import annotations

from typing import Any

import jax
import jax.numpy as jp
import optax
from flax import struct

from .networks import MLP, Normalizer


@struct.dataclass
class TrainState:
    params: Any              # {"actor", "critic", "log_std"}
    opt_state: Any
    actor_norm: Normalizer
    critic_norm: Normalizer
    learning_rate: jax.Array


@struct.dataclass
class Runner:
    state: Any               # batched env.State
    ep_return: jax.Array
    ep_length: jax.Array
    rng: jax.Array


@struct.dataclass
class Batch:
    actor_obs: jax.Array
    critic_obs: jax.Array
    action: jax.Array
    log_prob: jax.Array
    mean: jax.Array
    value: jax.Array
    reward: jax.Array
    discount: jax.Array      # gamma * (1 - delta) * (1 - done) for the step


def gaussian_log_prob(x, mean, log_std):
    return jp.sum(-0.5 * ((x - mean) / jp.exp(log_std)) ** 2 - log_std - 0.5 * jp.log(2 * jp.pi), -1)


class PPO:
    def __init__(self, env, config: dict):
        self.env = env
        self.cfg = config["train"]
        t = self.cfg
        self.actor_net = MLP(tuple(t["actor_hidden"]), env.n_actions, t["activation"], out_scale=0.01)
        self.critic_net = MLP(tuple(t["critic_hidden"]), 1, t["activation"], out_scale=1.0)
        self.optimizer = optax.chain(
            optax.clip_by_global_norm(t["max_grad_norm"]),
            optax.inject_hyperparams(optax.adam)(learning_rate=t["learning_rate"]),
        )

    def init(self, rng) -> TrainState:
        k1, k2 = jax.random.split(rng)
        env = self.env
        params = {
            "actor": self.actor_net.init(k1, jp.zeros((1, env.actor_obs_size))),
            "critic": self.critic_net.init(k2, jp.zeros((1, env.critic_obs_size))),
            "log_std": jp.full(env.n_actions, self.cfg["init_log_std"], dtype=jp.float32),
        }
        return TrainState(params=params, opt_state=self.optimizer.init(params),
                          actor_norm=Normalizer.create(env.actor_obs_size),
                          critic_norm=Normalizer.create(env.critic_obs_size),
                          learning_rate=jp.float32(self.cfg["learning_rate"]))

    def act(self, ts: TrainState, actor_obs):
        """Deterministic action (the exported policy)."""
        return self.actor_net.apply(ts.params["actor"], ts.actor_norm(actor_obs))

    # ------------------------------------------------------------ environment
    def make_env_fns(self, model, in_axes, model_params):
        env = self.env
        v_step = jax.vmap(env.step, in_axes=(in_axes, 0, 0, 0))
        v_reset = jax.vmap(env.reset, in_axes=(in_axes, 0, 0))

        def reset(rng):
            n = model_params.shape[0]
            return v_reset(model, jax.random.split(rng, n), model_params)

        def autoreset_step(state, action, key):
            nxt = v_step(model, state, action, model_params)
            fresh = v_reset(model, jax.random.split(key, action.shape[0]), model_params)
            done = nxt.done > 0

            def pick(a, b):
                if a.ndim == 0 or a.shape[0] != done.shape[0]:
                    return b   # unbatched (e.g. empty) fields
                d = done.reshape(done.shape + (1,) * (a.ndim - 1))
                return jp.where(d, a, b)

            return nxt.replace(data=jax.tree.map(pick, fresh.data, nxt.data),
                               obs=jax.tree.map(pick, fresh.obs, nxt.obs),
                               info=jax.tree.map(pick, fresh.info, nxt.info))

        return reset, autoreset_step

    def init_runner(self, reset, rng) -> Runner:
        k1, k2 = jax.random.split(rng)
        state = reset(k1)
        n = state.reward.shape[0]
        return Runner(state=state, ep_return=jp.zeros(n), ep_length=jp.zeros(n), rng=k2)

    # ------------------------------------------------------------ iteration
    def make_iteration(self, autoreset_step):
        env = self.env
        t = self.cfg
        gamma, lam = t["gamma"], t["lam"]

        def rollout(ts: TrainState, runner: Runner):
            def one(carry, key):
                state, ep_ret, ep_len = carry
                k_act, k_reset = jax.random.split(key)
                p = ts.params
                mean = self.actor_net.apply(p["actor"], ts.actor_norm(state.obs["actor"]))
                action = mean + jp.exp(p["log_std"]) * jax.random.normal(k_act, mean.shape)
                log_prob = gaussian_log_prob(action, mean, p["log_std"])
                value = self.critic_net.apply(p["critic"], ts.critic_norm(state.obs["critic"]))[:, 0]
                nxt = autoreset_step(state, action, k_reset)
                timeout = ((nxt.done > 0) & (nxt.terminated == 0)).astype(jp.float32)
                # Time-outs bootstrap from the value of the last state (rsl_rl).
                reward = nxt.reward + gamma * value * timeout
                discount = gamma * (1.0 - nxt.cat_delta) * (1.0 - nxt.done)
                ep_ret = ep_ret + nxt.reward
                ep_len = ep_len + 1
                finished = nxt.done > 0
                stats = {"return_sum": jp.sum(jp.where(finished, ep_ret, 0.0)),
                         "length_sum": jp.sum(jp.where(finished, ep_len, 0.0)),
                         "episodes": jp.sum(finished.astype(jp.float32)),
                         "falls": jp.sum(nxt.terminated),
                         **{f"m/{k}": jp.mean(v) for k, v in nxt.metrics.items()}}
                batch = Batch(state.obs["actor"], state.obs["critic"], action, log_prob, mean, value,
                              reward, discount)
                return (nxt, jp.where(finished, 0.0, ep_ret), jp.where(finished, 0.0, ep_len)), (batch, stats)

            rng, key = jax.random.split(runner.rng)
            keys = jax.random.split(key, t["steps_per_env"])
            (state, ep_ret, ep_len), (batch, stats) = jax.lax.scan(
                one, (runner.state, runner.ep_return, runner.ep_length), keys)
            last_value = self.critic_net.apply(ts.params["critic"], ts.critic_norm(state.obs["critic"]))[:, 0]
            return Runner(state, ep_ret, ep_len, rng), batch, stats, last_value

        def gae(batch: Batch, last_value):
            def back(carry, x):
                adv, next_value = carry
                reward, discount, value = x
                delta = reward + discount * next_value - value
                adv = delta + discount * lam * adv
                return (adv, value), adv

            _, adv = jax.lax.scan(back, (jp.zeros_like(last_value), last_value),
                                  (batch.reward, batch.discount, batch.value), reverse=True)
            return adv, adv + batch.value

        def loss_fn(params, ts: TrainState, mb):
            obs_a = ts.actor_norm(mb["actor_obs"])
            mean = self.actor_net.apply(params["actor"], obs_a)
            log_std = params["log_std"]
            log_prob = gaussian_log_prob(mb["action"], mean, log_std)
            ratio = jp.exp(log_prob - mb["log_prob"])
            adv = mb["adv"]
            adv = (adv - adv.mean()) / (adv.std() + 1e-8)
            surrogate = -jp.mean(jp.minimum(ratio * adv, jp.clip(ratio, 1 - t["clip"], 1 + t["clip"]) * adv))
            value = self.critic_net.apply(params["critic"], ts.critic_norm(mb["critic_obs"]))[:, 0]
            v_clipped = mb["value"] + jp.clip(value - mb["value"], -t["clip"], t["clip"])
            value_loss = jp.mean(jp.maximum((value - mb["ret"]) ** 2, (v_clipped - mb["ret"]) ** 2))
            entropy = jp.sum(log_std + 0.5 * jp.log(2 * jp.pi * jp.e))
            loss = surrogate + t["value_coef"] * value_loss - t["entropy_coef"] * entropy
            sym = jp.float32(0.0)
            if t.get("symmetry_coef", 0.0) > 0:
                mirrored = self.actor_net.apply(params["actor"], ts.actor_norm(env.mirror_actor_obs(mb["actor_obs"])))
                sym = jp.mean(jp.sum((mirrored - env.mirror_action(mean)) ** 2, -1))
                loss = loss + t["symmetry_coef"] * sym
            old_std, new_std = jp.exp(mb["log_std_old"]), jp.exp(log_std)
            kl = jp.mean(jp.sum(log_std - mb["log_std_old"]
                                + (old_std ** 2 + (mb["mean"] - mean) ** 2) / (2 * new_std ** 2) - 0.5, -1))
            return loss, {"surrogate": surrogate, "value_loss": value_loss, "entropy": entropy,
                          "symmetry": sym, "kl": kl}

        def update(ts: TrainState, batch: Batch, adv, ret, rng):
            n = adv.size
            flat = {"actor_obs": batch.actor_obs.reshape(n, -1), "critic_obs": batch.critic_obs.reshape(n, -1),
                    "action": batch.action.reshape(n, -1), "log_prob": batch.log_prob.reshape(n),
                    "mean": batch.mean.reshape(n, -1), "value": batch.value.reshape(n),
                    "adv": adv.reshape(n), "ret": ret.reshape(n)}
            mb_size = n // t["minibatches"]
            log_std_old = ts.params["log_std"]

            def epoch(carry, key):
                ts = carry
                perm = jax.random.permutation(key, n)[: mb_size * t["minibatches"]]
                perm = perm.reshape(t["minibatches"], mb_size)

                def minibatch(ts, idx):
                    mb = jax.tree.map(lambda x: x[idx], flat)
                    mb["log_std_old"] = log_std_old
                    (_, info), grads = jax.value_and_grad(loss_fn, has_aux=True)(ts.params, ts, mb)
                    # Adaptive learning rate from the KL of this minibatch (rsl_rl).
                    lr = ts.learning_rate
                    lr = jp.where(info["kl"] > 2.0 * t["desired_kl"], jp.maximum(1e-5, lr / 1.5), lr)
                    lr = jp.where(info["kl"] < 0.5 * t["desired_kl"], jp.minimum(1e-2, lr * 1.5), lr)
                    opt_state = ts.opt_state
                    opt_state[1].hyperparams["learning_rate"] = lr
                    updates, opt_state = self.optimizer.update(grads, opt_state, ts.params)
                    params = optax.apply_updates(ts.params, updates)
                    return ts.replace(params=params, opt_state=opt_state, learning_rate=lr), info

                ts, info = jax.lax.scan(minibatch, ts, perm)
                return ts, info

            ts, info = jax.lax.scan(epoch, ts, jax.random.split(rng, t["epochs"]))
            return ts, jax.tree.map(jp.mean, info)

        def iteration(ts: TrainState, runner: Runner):
            runner, batch, stats, last_value = rollout(ts, runner)
            adv, ret = gae(batch, last_value)
            rng, key = jax.random.split(runner.rng)
            ts, info = update(ts, batch, adv, ret, key)
            # The normalisers move between iterations only, so the stored log
            # probabilities stay consistent within one update.
            ts = ts.replace(actor_norm=ts.actor_norm.update(batch.actor_obs),
                            critic_norm=ts.critic_norm.update(batch.critic_obs))
            summary = {k: jp.sum(v) for k, v in stats.items() if not k.startswith("m/")}
            summary.update({k: jp.mean(v) for k, v in stats.items() if k.startswith("m/")})
            summary.update(info)
            summary["learning_rate"] = ts.learning_rate
            summary["action_std"] = jp.mean(jp.exp(ts.params["log_std"]))
            summary["reward_per_step"] = jp.mean(batch.reward)
            return ts, runner.replace(rng=rng), summary

        return jax.jit(iteration, donate_argnums=(1,))
