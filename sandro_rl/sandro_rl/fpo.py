"""Flow policies trained with FPO++ (Yi et al. 2026, arXiv 2602.02481).

`train.algorithm: fpo` selects it; everything else of ppo.py (rollout, GAE,
asymmetric critic, CaT, normalisers, mirror loss) is shared.

The actor is a velocity field v(x_tau, tau, obs) over the scaled action
x = a / action_scale. Exploration draws x_0 ~ N(0, I) at tau = 0 and
integrates dx/dtau = v to tau = 1 in `flow_steps` Euler steps. The exported
policy is deterministic: it starts from x_0 = 0 ("zero-sampling", which the
paper found better than sampling) and takes `deploy_steps` steps.

A flow has no tractable likelihood, so the update uses the conditional flow
matching loss of `mc_samples` (tau, eps) pairs per action,
    l = |v(tau x + (1 - tau) eps, tau, obs) - (x - eps)|^2,
as a stand-in for the log-likelihood (FPO, McAllister et al. 2025), with the
FPO++ changes:
  - one ratio per pair, rho_i = exp(l_old_i - l_new_i), each clipped alone;
  - ASPO: the PPO clip for positive advantages, the SPO penalty
    rho A - |A| (rho - 1)^2 / (2 clip) for negative ones;
  - the CFM losses clamped before the difference, the difference before exp;
  - AdamW at a fixed learning rate, no entropy bonus.
The mirror loss becomes equivariance of the field: a left-right symmetric
policy has v(M x, tau, M obs) = M v(x, tau, obs).
"""

from __future__ import annotations

import math

import jax
import jax.numpy as jp
import numpy as np
import optax

from .networks import MLP, Normalizer
from .ppo import PPO, TrainState

DEFAULTS = {"flow_steps": 16, "deploy_steps": 8, "mc_samples": 8, "action_scale": 0.5, "time_features": 4,
            "clip": 0.05, "learning_rate": 1.0e-4, "weight_decay": 1.0e-4, "loss_clamp": 20.0,
            "log_ratio_clamp": 2.0, "value_clip": 0.2, "aspo": True}


def time_embedding(tau, features: int, xp=jp):
    """(tau, sin 2 pi k tau, cos 2 pi k tau for k = 1..features)."""
    tau = xp.asarray(tau)
    angle = 2.0 * math.pi * tau[..., None] * xp.arange(1, features + 1)
    return xp.concatenate([tau[..., None], xp.sin(angle), xp.cos(angle)], -1)


def settings(train_cfg: dict) -> dict:
    return {**DEFAULTS, **(train_cfg.get("fpo") or {})}


class FlowPPO(PPO):
    def __init__(self, env, config: dict):
        self.env = env
        self.cfg = config["train"]
        t = self.cfg
        self.f = settings(t)
        self.n_time = 1 + 2 * int(self.f["time_features"])
        self.actor_net = MLP(tuple(t["actor_hidden"]), env.n_actions, t["activation"], out_scale=0.01)
        self.critic_net = MLP(tuple(t["critic_hidden"]), 1, t["activation"], out_scale=1.0)
        self.optimizer = optax.chain(
            optax.clip_by_global_norm(t["max_grad_norm"]),
            optax.inject_hyperparams(optax.adamw)(learning_rate=self.f["learning_rate"], b1=0.9, b2=0.95,
                                                  weight_decay=self.f["weight_decay"]),
        )

    def init(self, rng) -> TrainState:
        k1, k2 = jax.random.split(rng)
        env = self.env
        params = {
            "actor": self.actor_net.init(k1, jp.zeros((1, env.actor_obs_size + env.n_actions + self.n_time))),
            "critic": self.critic_net.init(k2, jp.zeros((1, env.critic_obs_size))),
        }
        return TrainState(params=params, opt_state=self.optimizer.init(params),
                          actor_norm=Normalizer.create(env.actor_obs_size),
                          critic_norm=Normalizer.create(env.critic_obs_size),
                          learning_rate=jp.float32(self.f["learning_rate"]))

    # ------------------------------------------------------------ the flow
    def field(self, params, obs_n, x, tau):
        """Velocity at (x, tau) for normalised observations; any leading batch shape."""
        h = jp.concatenate([obs_n, x, time_embedding(tau, int(self.f["time_features"]))], -1)
        return self.actor_net.apply(params["actor"], h)

    def integrate(self, params, obs_n, x0, steps: int):
        def euler(x, k):
            tau = jp.full(x.shape[:-1], k / steps, dtype=x.dtype)
            return x + self.field(params, obs_n, x, tau) / steps, None

        return jax.lax.scan(euler, x0, jp.arange(steps))[0]

    def cfm(self, params, obs_n, x, tau, eps):
        """Flow matching loss [B, N] of N (tau, eps) pairs per action, and the field there."""
        x_tau = tau[..., None] * x[:, None] + (1.0 - tau[..., None]) * eps
        obs = jp.broadcast_to(obs_n[:, None], x_tau.shape[:2] + obs_n.shape[-1:])
        v = self.field(params, obs, x_tau, tau)
        return jp.sum((v - (x[:, None] - eps)) ** 2, -1), (obs, x_tau, v)

    def act(self, ts: TrainState, actor_obs):
        """Deterministic action (the exported policy): zero-sampling."""
        obs_n = ts.actor_norm(actor_obs)
        x0 = jp.zeros(obs_n.shape[:-1] + (self.env.n_actions,))
        return self.f["action_scale"] * self.integrate(ts.params, obs_n, x0, int(self.f["deploy_steps"]))

    def sample(self, ts: TrainState, actor_obs, key):
        f = self.f
        k_x, k_tau, k_eps = jax.random.split(key, 3)
        obs_n = ts.actor_norm(actor_obs)
        b, n_act, n_mc = obs_n.shape[0], self.env.n_actions, int(f["mc_samples"])
        x = self.integrate(ts.params, obs_n, jax.random.normal(k_x, (b, n_act)), int(f["flow_steps"]))
        tau = jax.random.uniform(k_tau, (b, n_mc))
        eps = jax.random.normal(k_eps, (b, n_mc, n_act))
        cfm_old, _ = self.cfm(ts.params, obs_n, x, tau, eps)
        deterministic = self.act(ts, actor_obs)
        action = f["action_scale"] * x
        return action, {"tau": tau, "eps": eps, "cfm_old": cfm_old,
                        "noise_sq": jp.mean((action - deterministic) ** 2, -1)}

    def exploration(self, ts: TrainState, batch):
        """RMS distance of the sampled actions from the zero-sampled ones."""
        return jp.sqrt(jp.mean(batch.policy["noise_sq"]))

    # ------------------------------------------------------------ update
    def make_update(self):
        env = self.env
        t = self.cfg
        f = self.f
        clip = f["clip"]

        def loss_fn(params, ts: TrainState, mb):
            obs_n = ts.actor_norm(mb["actor_obs"])
            x = mb["action"] / f["action_scale"]
            cfm_new, (obs, x_tau, v) = self.cfm(params, obs_n, x, mb["tau"], mb["eps"])
            clamp = f["loss_clamp"]
            log_ratio = jp.clip(jp.minimum(mb["cfm_old"], clamp) - jp.minimum(cfm_new, clamp),
                                -f["log_ratio_clamp"], f["log_ratio_clamp"])
            ratio = jp.exp(log_ratio)                                    # [B, N]
            adv = mb["adv"]
            adv = ((adv - adv.mean()) / (adv.std() + 1e-8))[:, None]
            ppo = jp.minimum(ratio * adv, jp.clip(ratio, 1 - clip, 1 + clip) * adv)
            spo = ratio * adv - jp.abs(adv) / (2.0 * clip) * (ratio - 1.0) ** 2
            surrogate = -jp.mean(jp.where(adv >= 0, ppo, spo) if f["aspo"] else ppo)
            value_loss = self.value_loss(params, ts, mb, f["value_clip"])
            loss = surrogate + t["value_coef"] * value_loss
            sym = jp.float32(0.0)
            if t.get("symmetry_coef", 0.0) > 0:
                obs_m = ts.actor_norm(env.mirror_actor_obs(mb["actor_obs"]))
                obs_m = jp.broadcast_to(obs_m[:, None], obs.shape)
                mirrored = self.field(params, obs_m, env.mirror_action(x_tau), mb["tau"])
                sym = jp.mean(jp.sum((mirrored - env.mirror_action(v)) ** 2, -1))
                loss = loss + t["symmetry_coef"] * sym
            # k3 estimate of the KL from the per-sample ratios, for the log.
            kl = jp.mean(ratio - 1.0 - log_ratio)
            clipped = jp.mean((jp.abs(ratio - 1.0) > clip).astype(jp.float32))
            return loss, {"surrogate": surrogate, "value_loss": value_loss, "entropy": jp.float32(0.0),
                          "symmetry": sym, "kl": kl, "clip_fraction": clipped}

        def update(ts: TrainState, batch, adv, ret, rng):
            n = adv.size
            flat = self.flatten(batch, adv, ret)
            mb_size = n // t["minibatches"]

            def epoch(ts, key):
                perm = jax.random.permutation(key, n)[: mb_size * t["minibatches"]]
                perm = perm.reshape(t["minibatches"], mb_size)

                def minibatch(ts, idx):
                    mb = jax.tree.map(lambda x: x[idx], flat)
                    (_, info), grads = jax.value_and_grad(loss_fn, has_aux=True)(ts.params, ts, mb)
                    updates, opt_state = self.optimizer.update(grads, ts.opt_state, ts.params)
                    return ts.replace(params=optax.apply_updates(ts.params, updates), opt_state=opt_state), info

                return jax.lax.scan(minibatch, ts, perm)

            ts, info = jax.lax.scan(epoch, ts, jax.random.split(rng, t["epochs"]))
            return ts, jax.tree.map(jp.mean, info)

        return update


# ---------------------------------------------------------------- export
def reference_action(payload: dict, train_cfg: dict, obs: np.ndarray, clip_obs=10.0, clip_action=1.0):
    """The zero-sampled action in numpy, from the concatenated network input."""
    from .export import _dense_layers

    f = settings(train_cfg)
    norm = payload["actor_norm"]
    h0 = np.clip((obs - norm["mean"]) / np.sqrt(norm["var"] + 1e-8), -clip_obs, clip_obs)
    layers = _dense_layers(payload["params"]["actor"])
    act = {"elu": lambda v: np.where(v > 0, v, np.expm1(v)), "tanh": np.tanh, "relu": lambda v: np.maximum(v, 0)}
    activation = act[train_cfg["activation"]]
    steps = int(f["deploy_steps"])
    x = np.zeros((obs.shape[0], layers[-1][0].shape[1]))
    for k in range(steps):
        tau = np.full(obs.shape[0], k / steps)
        h = np.concatenate([h0, x, time_embedding(tau, int(f["time_features"]), np)], -1)
        for i, (kernel, bias) in enumerate(layers):
            h = h @ kernel + bias
            if i < len(layers) - 1:
                h = activation(h)
        x = x + h / steps
    return np.clip(f["action_scale"] * x, -clip_action, clip_action)


def build_onnx(payload: dict, train_cfg: dict, clip_obs: float, clip_action: float):
    """The zero-sampled flow unrolled into one graph, obs -> action. The first
    layer is split: the observation part is computed once, the time part is
    a constant bias per step, so the graph has no Concat."""
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    from .export import OPSET, _dense_layers

    f = settings(train_cfg)
    steps, features = int(f["deploy_steps"]), int(f["time_features"])
    norm = payload["actor_norm"]
    mean = np.asarray(norm["mean"], np.float32)
    std = np.sqrt(np.asarray(norm["var"], np.float32) + 1e-8).astype(np.float32)
    layers = _dense_layers(payload["params"]["actor"])
    obs_size = mean.size
    act_size = layers[-1][0].shape[1]
    w0, b0 = layers[0]
    w_obs, w_x, w_t = w0[:obs_size], w0[obs_size:obs_size + act_size], w0[obs_size + act_size:]
    f32 = lambda a: np.asarray(a, np.float32)
    inits = [numpy_helper.from_array(mean, "obs_mean"), numpy_helper.from_array(std, "obs_std"),
             numpy_helper.from_array(np.array(-clip_obs, np.float32), "obs_lo"),
             numpy_helper.from_array(np.array(clip_obs, np.float32), "obs_hi"),
             numpy_helper.from_array(np.array(-clip_action, np.float32), "act_lo"),
             numpy_helper.from_array(np.array(clip_action, np.float32), "act_hi"),
             numpy_helper.from_array(f32(w_obs), "W0_obs"), numpy_helper.from_array(f32(w_x), "W0_x"),
             numpy_helper.from_array(np.array(1.0 / steps, np.float32), "dtau"),
             numpy_helper.from_array(np.array(f["action_scale"], np.float32), "action_scale")]
    for i, (kernel, bias) in enumerate(layers[1:], start=1):
        inits += [numpy_helper.from_array(f32(kernel), f"W{i}"), numpy_helper.from_array(f32(bias), f"b{i}")]
    for k in range(steps):
        bias = b0 + time_embedding(np.array(k / steps), features, np) @ w_t
        inits.append(numpy_helper.from_array(f32(bias), f"b0_step{k}"))
    nodes = [helper.make_node("Sub", ["obs", "obs_mean"], ["centred"]),
             helper.make_node("Div", ["centred", "obs_std"], ["scaled"]),
             helper.make_node("Clip", ["scaled", "obs_lo", "obs_hi"], ["h0"]),
             helper.make_node("MatMul", ["h0", "W0_obs"], ["obs_part"])]
    op = {"elu": "Elu", "tanh": "Tanh", "relu": "Relu"}[train_cfg["activation"]]
    x = None
    for k in range(steps):
        pre = f"s{k}_"
        if x is None:   # x_0 = 0: no action term
            nodes.append(helper.make_node("Add", ["obs_part", f"b0_step{k}"], [pre + "z0"]))
        else:
            nodes += [helper.make_node("MatMul", [x, "W0_x"], [pre + "x_part"]),
                      helper.make_node("Add", ["obs_part", pre + "x_part"], [pre + "z0a"]),
                      helper.make_node("Add", [pre + "z0a", f"b0_step{k}"], [pre + "z0"])]
        h = pre + "z0"
        for i in range(1, len(layers)):
            nodes.append(helper.make_node(op, [h], [pre + f"a{i - 1}"]))
            nodes.append(helper.make_node("Gemm", [pre + f"a{i - 1}", f"W{i}", f"b{i}"], [pre + f"z{i}"]))
            h = pre + f"z{i}"
        nodes.append(helper.make_node("Mul", [h, "dtau"], [pre + "dx"]))
        if x is None:
            x = pre + "dx"
        else:
            nodes.append(helper.make_node("Add", [x, pre + "dx"], [pre + "x"]))
            x = pre + "x"
    nodes += [helper.make_node("Mul", [x, "action_scale"], ["raw_action"]),
              helper.make_node("Clip", ["raw_action", "act_lo", "act_hi"], ["action"])]
    graph = helper.make_graph(
        nodes, "sandro_flow_policy",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, ["batch", obs_size])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, ["batch", act_size])],
        initializer=inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", OPSET)], producer_name="sandro_rl")
    model.ir_version = 8
    onnx.checker.check_model(model)
    return model
