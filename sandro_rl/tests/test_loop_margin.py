"""The yaw-loop penalty (loop_margin.py): its transfer function against a
time-domain run of the same recursion, its numbers against the ONNX tool in
recordings/ on a trained policy, and that descending it restores the margin."""

import importlib.util
import json
from pathlib import Path

import jax
import jax.numpy as jp
import numpy as np
import optax
import pytest

from sandro_rl import checkpoint
from sandro_rl.loop_margin import Channels, YawLoop, load_plant
from sandro_rl.networks import MLP, Normalizer

ROOT = Path(__file__).resolve().parents[1]
PLANT = "yaw_plant_2026-11-12.json"


def channels(frame=25, stride=1, rate=100.0):
    return Channels(frame=frame, history=4, stride=stride, rate_hz=rate, gravity_z=2, gyro_z=5,
                    wheel_right=14, wheel_left=15, last_action=16, n_actions=6, action_right=2,
                    action_left=5, gyro_scale=0.25, wheel_scale=0.05, torque_per_action=8.0, drive_kd=0.4)


@pytest.mark.parametrize("stride,rate", [(1, 100.0), (3, 300.0)])
def test_response_matches_the_recursion_in_time(stride, rate):
    """A random linear policy driven by a sinusoidal yaw rate: the steady
    opposed wheel torque command is the transfer function's."""
    c = channels(stride=stride, rate=rate)
    rng = np.random.default_rng(stride)
    weight = rng.normal(0.0, 0.08, (6, c.history, c.frame))
    weight[:, :, 16:22] *= 0.5                       # a stable last-action recursion
    loop = YawLoop(c, load_plant(PLANT))
    from_gyro, _ = loop.responses(jp.asarray(weight.reshape(6, -1), dtype=jp.float32))
    span = (c.history - 1) * stride + 1
    for hz in (5.0, 9.0):
        k = int(np.argmin(abs(loop.f_hz - hz)))
        f = loop.f_hz[k]
        steps = int(8 * rate)
        frames = np.zeros((span, c.frame))
        action = np.zeros(6)
        out = np.zeros(steps)
        for n in range(steps):
            frame = np.zeros(c.frame)
            frame[c.gyro_z] = c.gyro_scale * np.sin(2 * np.pi * f * n / rate)   # 1 rad/s of yaw rate
            frame[16:22] = action
            frames = np.concatenate([frame[None], frames[:-1]])
            action = np.einsum("ahc,hc->a", weight, frames[::stride])
            out[n] = 0.5 * (action[2] - action[5]) * c.torque_per_action
        tail = slice(steps // 2, steps)
        t = np.arange(steps)[tail] / rate
        measured = 2 * np.mean(out[tail] * np.exp(-2j * np.pi * f * t)) * 1j   # response to sin
        np.testing.assert_allclose(measured, complex(from_gyro[k]), rtol=0.02, atol=0.002)


def actor(directory):
    payload, meta = checkpoint.load(directory)
    train = meta["config"]["train"]
    net = MLP(tuple(train["actor_hidden"]), 6, train["activation"], out_scale=0.01)
    norm = Normalizer(**{k: jp.asarray(v) for k, v in payload["actor_norm"].items()})
    params = jax.tree.map(jp.asarray, payload["params"]["actor"])
    return (lambda p, o: net.apply(p, norm(o))), params


def test_matches_the_onnx_tool_on_a_trained_policy():
    """recordings/yaw_loop_margin.py linearises policy.onnx by finite
    differences in numpy; the loss differentiates the checkpoint in JAX."""
    directory = ROOT / "policies/flat_level_real"
    spec = importlib.util.spec_from_file_location("tool", ROOT.parent / "recordings/yaw_loop_margin.py")
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    policy = tool.Policy(directory)
    plant = load_plant(PLANT)
    frame = policy.operating_point()
    reference, _ = tool.loop(policy, policy.jacobian(frame), plant["f_hz"], plant["to_gyro"], plant["to_wheel"])

    meta = json.loads((directory / "policy.json").read_text())
    wheels = meta["action"]["wheels"]
    c = Channels(frame=policy.frame, history=policy.history, stride=policy.stride, rate_hz=policy.rate,
                 gravity_z=2, gyro_z=5, wheel_right=14, wheel_left=15, last_action=16, n_actions=6,
                 action_right=2, action_left=5, gyro_scale=0.25, wheel_scale=0.05,
                 torque_per_action=wheels["scale_rad_s"] * wheels["kd_nm_s_rad"], drive_kd=wheels["kd_nm_s_rad"])
    loop = YawLoop(c, plant, rolloff_hz=(30.0, 40.0))
    apply, params = actor(directory)
    obs = jp.asarray(np.tile(frame, policy.history), dtype=jp.float32)
    inside, above = loop.loop(jax.jacrev(lambda o: apply(params, o))(obs))
    np.testing.assert_allclose(np.asarray(inside), reference, rtol=0.03, atol=0.03)
    # This policy rings on the robot at 9-10 Hz: well past the margin there.
    worst = int(np.argmax(np.real(inside)))
    assert np.real(inside)[worst] > 1.0 and 7.0 < loop.f_hz[worst] < 11.0
    assert above.shape == (2,) and float(above.min()) > 0.0
    penalty, info = loop.penalty(lambda o: apply(params, o), obs[None])
    assert float(penalty) > 0.25 and float(info["loop_re"]) == pytest.approx(float(np.real(inside)[worst]))


def test_descending_the_penalty_restores_the_margin():
    """The loss term alone, on a trained policy at its standing point: a few
    hundred Adam steps bring Re L and the roll-off bound under the margin."""
    directory = ROOT / "policies/flat_level_real"
    apply, params = actor(directory)
    loop = YawLoop(channels(), load_plant(PLANT), rolloff_hz=(24.0, 30.0, 40.0, 45.0))
    frame = np.zeros(25, dtype=np.float32)
    frame[2] = -1.0
    obs = jp.asarray(np.tile(frame, (8, 4)))
    obs = obs + 0.01 * jax.random.normal(jax.random.key(0), obs.shape)
    loss = lambda p: loop.penalty(lambda o: apply(p, o), obs)
    start, info = loss(params)
    assert float(start) > 0.25 and float(info["loop_re"]) > 1.0
    optimizer = optax.adam(1e-4)
    state = optimizer.init(params)

    @jax.jit
    def step(p, s):
        (value, aux), grads = jax.value_and_grad(loss, has_aux=True)(p)
        updates, s = optimizer.update(grads, s)
        return optax.apply_updates(p, updates), s, value, aux

    for _ in range(400):
        params, state, value, info = step(params, state)
    assert np.isfinite(float(value))
    assert float(info["loop_re"]) < 0.6 and float(info["loop_rolloff"]) < 0.6


def test_fallen_observations_are_left_out():
    loop = YawLoop(channels(), load_plant(PLANT))
    weight = jp.zeros((6, 100)).at[2, 5].set(-4.0).at[5, 5].set(4.0)   # a stiff yaw damper
    upright = np.zeros(100, dtype=np.float32)
    upright[2] = -1.0
    lying = upright.copy()
    lying[2] = 0.0
    mean = lambda o: weight @ o
    standing, _ = loop.penalty(mean, jp.asarray(upright)[None])
    fallen, _ = loop.penalty(mean, jp.asarray(lying)[None])
    assert float(standing) > 0.0 and float(fallen) == 0.0


def test_a_plant_table_is_used_up_to_its_first_incoherent_frequency(tmp_path):
    table = {"f_hz": [3.0, 10.0, 20.0, 30.0, 40.0], "to_gyro": [[0, -1]] * 5, "to_wheel": [[1, 0]] * 5,
             "coherence": [0.95, 0.9, 0.8, 0.5, 0.9]}
    (tmp_path / "plant.json").write_text(json.dumps(table))
    assert list(load_plant(tmp_path / "plant.json")["f_hz"]) == [3.0, 10.0, 20.0]
    # The 2026-11-16 run swept to 45 Hz; above 31.6 Hz its answer is in the gyro's noise.
    plant = load_plant("yaw_plant_2026-11-16.json")
    assert 3.0 < plant["f_hz"][0] < 3.5 and 30.0 < plant["f_hz"][-1] < 33.0
    assert abs(plant["to_gyro"][-1]) < 0.2 < abs(plant["to_gyro"][0])
