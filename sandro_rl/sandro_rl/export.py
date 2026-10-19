"""Export the actor to ONNX: normalisation, MLP and action clip in one graph.

    uv run python -m sandro_rl.export policies/flat_tracking

Writes policy.onnx and policy.json (observation layout, action mapping,
rates, the model fingerprint) next to the checkpoint, and checks the ONNX
output against the JAX network on random observations.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

from . import checkpoint, sources
from .interface import PolicyInterface
from .robot import RobotSpec
from .sources import JOINTS

OPSET = 17


def _dense_layers(actor_params: dict) -> list[tuple[np.ndarray, np.ndarray]]:
    layers = actor_params["params"]
    names = sorted(layers, key=lambda n: int(n.split("_")[-1]))
    return [(np.asarray(layers[n]["kernel"], np.float32), np.asarray(layers[n]["bias"], np.float32))
            for n in names]


def build_onnx(payload: dict, activation: str, clip_obs: float, clip_action: float) -> onnx.ModelProto:
    norm = payload["actor_norm"]
    mean = np.asarray(norm["mean"], np.float32)
    std = np.sqrt(np.asarray(norm["var"], np.float32) + 1e-8).astype(np.float32)
    layers = _dense_layers(payload["params"]["actor"])
    obs_size = mean.size
    act_size = layers[-1][0].shape[1]
    inits = [numpy_helper.from_array(mean, "obs_mean"), numpy_helper.from_array(std, "obs_std"),
             numpy_helper.from_array(np.array(-clip_obs, np.float32), "obs_lo"),
             numpy_helper.from_array(np.array(clip_obs, np.float32), "obs_hi"),
             numpy_helper.from_array(np.array(-clip_action, np.float32), "act_lo"),
             numpy_helper.from_array(np.array(clip_action, np.float32), "act_hi")]
    nodes = [helper.make_node("Sub", ["obs", "obs_mean"], ["centred"]),
             helper.make_node("Div", ["centred", "obs_std"], ["scaled"]),
             helper.make_node("Clip", ["scaled", "obs_lo", "obs_hi"], ["h0"])]
    x = "h0"
    for i, (kernel, bias) in enumerate(layers):
        inits += [numpy_helper.from_array(kernel, f"W{i}"), numpy_helper.from_array(bias, f"b{i}")]
        nodes.append(helper.make_node("Gemm", [x, f"W{i}", f"b{i}"], [f"z{i}"]))
        x = f"z{i}"
        if i < len(layers) - 1:
            op = {"elu": "Elu", "tanh": "Tanh", "relu": "Relu"}[activation]
            nodes.append(helper.make_node(op, [x], [f"a{i}"]))
            x = f"a{i}"
    nodes.append(helper.make_node("Clip", [x, "act_lo", "act_hi"], ["action"]))
    graph = helper.make_graph(
        nodes, "sandro_policy",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, ["batch", obs_size])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, ["batch", act_size])],
        initializer=inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", OPSET)],
                              producer_name="sandro_rl")
    model.ir_version = 8
    onnx.checker.check_model(model)
    return model


def reference_forward(payload: dict, activation: str, obs: np.ndarray, clip_obs=10.0, clip_action=1.0):
    norm = payload["actor_norm"]
    x = np.clip((obs - norm["mean"]) / np.sqrt(norm["var"] + 1e-8), -clip_obs, clip_obs)
    layers = _dense_layers(payload["params"]["actor"])
    act = {"elu": lambda v: np.where(v > 0, v, np.expm1(v)), "tanh": np.tanh, "relu": lambda v: np.maximum(v, 0)}
    for i, (kernel, bias) in enumerate(layers):
        x = x @ kernel + bias
        if i < len(layers) - 1:
            x = act[activation](x)
    return np.clip(x, -clip_action, clip_action)


def export(policy_dir: Path) -> Path:
    import onnxruntime as ort

    payload, meta = checkpoint.load(policy_dir)
    cfg = meta["config"]
    activation = cfg["train"]["activation"]
    clip_action = float(cfg["env"]["action"]["clip"])
    model = build_onnx(payload, activation, 10.0, clip_action)
    out = policy_dir / "policy.onnx"
    onnx.save(model, out)
    session = ort.InferenceSession(str(out), providers=["CPUExecutionProvider"])
    obs = np.random.default_rng(0).normal(size=(256, payload["actor_norm"]["mean"].size)).astype(np.float32)
    obs = obs * np.sqrt(payload["actor_norm"]["var"]) + payload["actor_norm"]["mean"]
    got = session.run(None, {"obs": obs.astype(np.float32)})[0]
    want = reference_forward(payload, activation, obs.astype(np.float64))
    err = float(np.max(np.abs(got - want)))
    if err > 1e-4:
        raise RuntimeError(f"ONNX output differs from the reference network by {err}")

    spec = RobotSpec.load()
    i = PolicyInterface(cfg, spec)
    frame = [("gravity", 3), ("gyro", 3)] + ([("accel", 3)] if i.accelerometer else []) + [
        ("leg_pos", 4), ("leg_vel", 4), ("wheel_vel", 2), ("last_action", 6), ("command", 4 if i.has_roll else 3)]
    if sum(n for _, n in frame) * i.history != payload["actor_norm"]["mean"].size:
        raise RuntimeError("interface.PolicyInterface no longer matches the trained observation size")
    a = cfg["env"]["action"]
    (policy_dir / "policy.json").write_text(json.dumps({
        "onnx": out.name, "onnx_max_abs_error_vs_reference": err,
        "policy_rate_hz": cfg["env"]["policy_rate_hz"], "control_rate_hz": i.control_rate_hz,
        "observation": {"history": i.history, "order": "newest frame first", "frame": frame,
                        "legs": list(sources.LEG_JOINTS), "wheels": list(sources.WHEEL_JOINTS),
                        "scales": i.scales, "imu_offset_correction_rad": i.imu_offset_correction,
                        "command_normalisation": {"forward_m_s": i.v_max, "yaw_rad_s": i.yaw_max,
                                                  "height_m": [i.h_min, i.h_max],
                                                  **({"roll_rad": i.roll_max} if i.has_roll else {})}},
        "action": {"joints": list(JOINTS),
                   "legs": {"q_ref": "stand_q + scale_rad * a", "scale_rad": i.leg_scale,
                            "kp": i.leg_kp, "kd": i.leg_kd, "range_rad": [i.q_lo, i.q_hi]},
                   "wheels": a["wheels"], "stand_q": [float(v) for v in i.stand_q],
                   "torque_limits_nm": [float(v) for v in i.torque_limit]},
        "command_shaping": {"forward_accel_m_s2": i.accel_limit, "yaw_time_constant_s": i.yaw_tau,
                            "height_rate_m_s": i.h_rate,
                            **({"roll_rate_rad_s": i.roll_rate, "leg_height_limits_m": [i.leg_h_lo, i.leg_h_hi],
                                "track_width_m": i.track_width} if i.has_roll else {})},
        "fingerprint": meta["fingerprint"]["sha256"],
    }, indent=1))
    print(f"wrote {out} (max |onnx - reference| = {err:.2e})")
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("policy_dir")
    args = parser.parse_args()
    export(Path(args.policy_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
