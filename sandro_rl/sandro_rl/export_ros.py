"""Hand exported policies to the ROS stack's RlPolicyController.

    uv run python -m sandro_rl.export_ros policies/flat_level:rl_flat_level \
        policies/agile_terrain:rl_agile_terrain

For each policy directory (policy.onnx + policy.json, written by export.py)
and controller name this writes:

- ros2_ws/src/sandro_controllers/policies/<policy>.bin: the actor (observation
  normaliser, dense layers, clips) in the controller's flat binary format;
- ros2_ws/src/sandro_controllers/test/data/<policy>_reference.txt: random
  observations and the onnxruntime actions for them, for the controller's
  unit test;
- with --frames, ros2_ws/src/sandro_controllers/test/data/<policy>_frames.txt:
  random measured signals and interface.py's observation frame for them;
- ros2_ws/src/sandro_bringup/config/rl_policies.yaml: one controller entry and
  parameter block per policy, generated from policy.json.

Needs only onnx, onnxruntime, numpy and PyYAML (no JAX), and MuJoCo for --frames.
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
ROS_SRC = REPO / "ros2_ws" / "src"
MAGIC = b"SNDRMLP1"
ACTIVATIONS = {"Elu": 0, "Tanh": 1, "Relu": 2}


def read_actor(onnx_path: Path) -> dict:
    """Normaliser, layers and clips of export.build_onnx's graph."""
    import onnx
    from onnx import numpy_helper

    model = onnx.load(str(onnx_path))
    init = {i.name: numpy_helper.to_array(i) for i in model.graph.initializer}
    ops = [n.op_type for n in model.graph.node]
    hidden_ops = {op for op in ops if op in ACTIVATIONS}
    if ops[:3] != ["Sub", "Div", "Clip"] or ops[-1] != "Clip" or len(hidden_ops) != 1:
        raise ValueError(f"{onnx_path}: not an export.build_onnx graph: {ops}")
    layers = []
    while f"W{len(layers)}" in init:
        i = len(layers)
        layers.append((init[f"W{i}"].astype(np.float32), init[f"b{i}"].astype(np.float32)))
    return {"mean": init["obs_mean"].astype(np.float32), "std": init["obs_std"].astype(np.float32),
            "obs_clip": float(init["obs_hi"]), "act_clip": float(init["act_hi"]),
            "activation": ACTIVATIONS[hidden_ops.pop()], "layers": layers}


def write_actor(actor: dict, path: Path) -> None:
    """Little-endian: magic, activation, layer count, observation size,
    observation and action clips, mean, std, then per layer (in, out),
    W (in x out, row-major, y = x W + b) and b."""
    with path.open("wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<III", actor["activation"], len(actor["layers"]), actor["mean"].size))
        f.write(struct.pack("<ff", actor["obs_clip"], actor["act_clip"]))
        f.write(actor["mean"].tobytes())
        f.write(actor["std"].tobytes())
        for kernel, bias in actor["layers"]:
            f.write(struct.pack("<II", *kernel.shape))
            f.write(np.ascontiguousarray(kernel).tobytes())
            f.write(bias.tobytes())


def write_reference(onnx_path: Path, actor: dict, path: Path, count: int = 16) -> None:
    import onnxruntime as ort

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(0)
    obs = (rng.normal(size=(count, actor["mean"].size)) * 1.5 * actor["std"] + actor["mean"]).astype(np.float32)
    actions = session.run(None, {"obs": obs})[0]
    with path.open("w") as f:
        f.write(f"{count} {obs.shape[1]} {actions.shape[1]}\n")
        for o, a in zip(obs, actions):
            f.write(" ".join(f"{v:.9g}" for v in o) + "\n")
            f.write(" ".join(f"{v:.9g}" for v in a) + "\n")


def write_frames(policy_dir: Path, path: Path, count: int = 32) -> None:
    """Random measured signals and interface.PolicyInterface's observation
    frame for each, with zero last action and command reference."""
    from .interface import PolicyInterface
    from .robot import RobotSpec

    meta = json.loads((policy_dir / "meta.json").read_text())
    iface = PolicyInterface(meta["config"], RobotSpec.load())
    rng = np.random.default_rng(1)
    with path.open("w") as f:
        f.write(f"{count} {iface.imu_offset_correction:.9g} "
                + " ".join(f"{v:.9g}" for v in iface.stand_q) + "\n")
        for _ in range(count):
            quat_xyzw = rng.normal(size=4)
            quat_xyzw /= np.linalg.norm(quat_xyzw)
            gyro = rng.normal(size=3) * 2.0
            q = iface.stand_q + rng.normal(size=6) * 0.3
            v = rng.normal(size=6) * 5.0
            ref = np.zeros(4 if iface.has_roll else 3)
            frame = iface.frame(np.roll(quat_xyzw, 1), gyro, np.zeros(3), q, v, np.zeros(6), ref)
            f.write(" ".join(f"{x:.9g}" for x in [*quat_xyzw, *gyro, *q, *v]) + "\n")
            f.write(" ".join(f"{x:.9g}" for x in frame) + "\n")


def _f(value) -> str:
    """A float ROS reads as a double, never as an integer (1.0, not 1)."""
    text = f"{float(value):.9g}"
    return text if any(c in text for c in ".en") else text + ".0"


def _list(values) -> str:
    return "[" + ", ".join(_f(v) for v in values) + "]"


def initial_height_m() -> float:
    """The whole-body controller's height target, the height command evaluate.py starts from."""
    import yaml

    config = yaml.safe_load((ROS_SRC / "sandro_bringup" / "config" / "controllers.yaml").read_text())
    return float(config["whole_body_controller"]["ros__parameters"]["tasks"]["height"]["target_m"])


def parameter_block(name: str, policy: str, contract: dict, actor: dict) -> str:
    obs = contract["observation"]
    frame = dict((k, n) for k, n in obs["frame"])
    unsupported = set(frame) - {"gravity", "gyro", "leg_pos", "leg_vel", "wheel_vel", "last_action", "command"}
    if unsupported:
        raise ValueError(f"{policy}: observation terms {unsupported} are not implemented by RlPolicyController")
    if obs["order"] != "newest frame first":
        raise ValueError(f"{policy}: unexpected history order {obs['order']}")
    if sum(frame.values()) * obs["history"] != actor["mean"].size:
        raise ValueError(f"{policy}: policy.json frame does not match the ONNX input size")
    norm = obs["command_normalisation"]
    shaping = contract["command_shaping"]
    action = contract["action"]
    wheels = action["wheels"]
    height = norm["height_m"]
    joints = action["joints"]
    return f"""{name}:
  ros__parameters:
    joints: [{", ".join(joints)}]
    imu_name: imu
    command_topic: /cmd_vel
    policy_file: package://sandro_controllers/policies/{policy}.bin
    policy_fingerprint: {contract["fingerprint"]}
    policy_rate_hz: {_f(float(contract["policy_rate_hz"]))}
    observation:
      history: {obs["history"]}
      command_size: {frame["command"]}
      gyro_scale: {_f(obs["scales"]["gyro"])}
      leg_velocity_scale: {_f(obs["scales"]["leg_vel"])}
      wheel_velocity_scale: {_f(obs["scales"]["wheel_vel"])}
      # The simulated IMU's mounting (sim_identified.yaml imu_mount); the
      # robot overrides set the physical one.
      imu_offset_rad: {_f(obs["imu_offset_correction_rad"])}
    commands:
      forward_m_s: {_f(norm["forward_m_s"])}
      yaw_rad_s: {_f(norm["yaw_rad_s"])}
      height_min_m: {_f(height[0])}
      height_max_m: {_f(height[1])}
      roll_rad: {_f(norm.get("roll_rad", 0.0))}
      # Height command on engaging (the WBC's tasks.height.target_m); the
      # reference starts at the measured height and moves there.
      initial_height_m: {_f(initial_height_m())}
    shaping:
      forward_accel_m_s2: {_f(shaping["forward_accel_m_s2"])}
      yaw_time_constant_s: {_f(shaping["yaw_time_constant_s"])}
      height_rate_m_s: {_f(shaping["height_rate_m_s"])}
      roll_rate_rad_s: {_f(shaping.get("roll_rate_rad_s", 0.5))}
      leg_height_min_m: {_f(shaping.get("leg_height_limits_m", height)[0])}
      leg_height_max_m: {_f(shaping.get("leg_height_limits_m", height)[1])}
      track_width_m: {_f(shaping.get("track_width_m", 0.304))}
    action:
      leg_scale_rad: {_f(action["legs"]["scale_rad"])}
      leg_kp_nm_rad: {_f(action["legs"]["kp"])}
      leg_kd_nm_s_rad: {_f(action["legs"]["kd"])}
      joint_min_rad: {_f(action["legs"]["range_rad"][0])}
      joint_max_rad: {_f(action["legs"]["range_rad"][1])}
      wheel_mode: {wheels["mode"]}
      wheel_scale_rad_s: {_f(wheels["scale_rad_s"])}
      wheel_kd_nm_s_rad: {_f(wheels["kd_nm_s_rad"])}
      wheel_scale_nm: {_f(wheels["scale_nm"])}
      stand_q: {_list(action["stand_q"])}
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("policies", nargs="+", help="policy_dir:controller_name")
    parser.add_argument("--frames", action="store_true",
                        help="also write the observation-frame reference (needs MuJoCo)")
    args = parser.parse_args()
    out_dir = ROS_SRC / "sandro_controllers" / "policies"
    ref_dir = ROS_SRC / "sandro_controllers" / "test" / "data"
    out_dir.mkdir(exist_ok=True)
    ref_dir.mkdir(parents=True, exist_ok=True)
    types, blocks = [], []
    for item in args.policies:
        policy_dir, name = item.split(":")
        policy_dir = Path(policy_dir)
        policy = policy_dir.name
        contract = json.loads((policy_dir / "policy.json").read_text())
        onnx_path = policy_dir / contract["onnx"]
        actor = read_actor(onnx_path)
        write_actor(actor, out_dir / f"{policy}.bin")
        write_reference(onnx_path, actor, ref_dir / f"{policy}_reference.txt")
        if args.frames:
            write_frames(policy_dir, ref_dir / f"{policy}_frames.txt")
        types.append(f"    {name}:\n      type: sandro_controllers/RlPolicyController\n")
        blocks.append(parameter_block(name, policy, contract, actor))
        print(f"{policy} -> {name}: {actor['mean'].size} inputs, layers "
              f"{[k.shape[1] for k, _ in actor['layers']]}")
    yaml_path = ROS_SRC / "sandro_bringup" / "config" / "rl_policies.yaml"
    yaml_path.write_text(
        "# Generated by sandro_rl/sandro_rl/export_ros.py from each policy's policy.json;\n"
        "# regenerate rather than edit. Per-robot values go in sandro_real_overrides.yaml.\n"
        "# Launch with controller:=<name>; like the other balancers, activate only one.\n"
        "controller_manager:\n  ros__parameters:\n" + "".join(types) + "\n" + "\n".join(blocks))
    print(f"wrote {yaml_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
