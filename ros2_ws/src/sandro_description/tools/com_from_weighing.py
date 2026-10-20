#!/usr/bin/env python3
"""Centre of mass of the physical robot from a scale, and the matching sim payload.

Lock both legs (spacer or zip ties) at a known pose, drives unpowered. Weigh
the whole robot. Then stand it on its wheels, tilt the body to a pitch and
hold it there with one point resting on a kitchen scale; note the IMU pitch,
the scale reading and the horizontal distance from the wheel axle to that
point. Do it at two or more pitches, forward and backward (+-15 to 30 deg,
support 0.2 m or more from the axle):

    python3 tools/com_from_weighing.py --mass 4.825 \\
        --tilt 25,0.731,0.30 --tilt -20,0.512,-0.28 [--legs RI,RO,LI,LO] [--write]

Each --tilt is "pitch_deg,scale_kg,x_m": IMU pitch with the top of the body
forward positive (ROS pitch), the scale reading, and the support point's
horizontal distance from the axle, forward positive. With the floor reaction
directly below the axle, moments about it give

    scale_kg * x_m = mass * (L sin(pitch) + e cos(pitch))

where L is the CoM height above the axle and e its forward offset, both in
the IMU frame. Two pitches determine L and e; more are fitted by least
squares.

The MuJoCo model is evaluated at the same leg pose (the stand keyframe, or
--legs with the four leg joint positions from /joint_states) to print the
payload that makes the simulated CoM match, and the controller parameters
that follow. --write puts the payload into config/sim_identified.yaml; run
`tools/sync_model.py sync` afterwards.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
SCENE = PACKAGE / "mjcf" / "scene_ros2.xml"
IDENTIFIED = PACKAGE / "config" / "sim_identified.yaml"
CONTROLLERS = PACKAGE.parent / "sandro_bringup" / "config" / "controllers.yaml"
LEG_JOINTS = ("right_motor_inner", "right_motor_outer", "left_motor_inner", "left_motor_outer")
WHEELS = ("right_wheel", "left_wheel")
PAYLOAD_BODY = "identified_payload"


def parse_tilt(text: str) -> tuple[float, float, float]:
    try:
        pitch_deg, scale_kg, x_m = (float(v) for v in text.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"tilt {text!r} must be pitch_deg,scale_kg,x_m") from error
    if scale_kg <= 0.0 or x_m == 0.0:
        raise argparse.ArgumentTypeError(f"tilt {text!r}: scale reading > 0 and x != 0")
    return math.radians(pitch_deg), scale_kg, x_m


def solve(mass: float, tilts, sigma_pitch_deg: float, sigma_scale_kg: float,
          sigma_x_m: float, samples: int = 4000, seed: int = 0):
    """(L, e) by least squares, their 1-sigma from the measurement tolerances, residual RMS."""

    def fit(pitch, scale, x):
        a = np.column_stack([np.sin(pitch), np.cos(pitch)])
        b = scale * x / mass
        return np.linalg.lstsq(a, b, rcond=None)[0], a, b

    pitch, scale, x = (np.array(v, dtype=float) for v in zip(*tilts))
    (height, offset), a, b = fit(pitch, scale, x)
    residual = float(np.sqrt(np.mean((a @ np.array([height, offset]) - b) ** 2)))
    rng = np.random.default_rng(seed)
    draws = np.array([
        fit(pitch + rng.normal(0.0, math.radians(sigma_pitch_deg), pitch.size),
            scale + rng.normal(0.0, sigma_scale_kg, scale.size),
            x + rng.normal(0.0, sigma_x_m, x.size))[0]
        for _ in range(samples)])
    spread = draws.std(axis=0)
    condition = float(np.linalg.cond(a))
    return float(height), float(offset), float(spread[0]), float(spread[1]), residual, condition


def _pose(model, data, legs):
    """Stand keyframe, or the keyframe with the leg motors moved to `legs` and the
    five-bar closure settled (torso pinned, no gravity, no contact)."""
    import mujoco

    mujoco.mj_resetDataKeyframe(model, data, 0)
    if legs is None:
        mujoco.mj_forward(model, data)
        return
    free = data.qpos[:7].copy()
    addresses = [model.jnt_dofadr[model.joint(n).id] for n in LEG_JOINTS]
    qaddresses = [model.jnt_qposadr[model.joint(n).id] for n in LEG_JOINTS]
    gravity = model.opt.gravity.copy()
    disable = model.opt.disableflags
    model.opt.gravity[:] = 0.0
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    try:
        for _ in range(4000):
            data.qpos[:7], data.qvel[:6] = free, 0.0
            error = np.array(legs) - data.qpos[qaddresses]
            data.qfrc_applied[:] = 0.0
            data.qfrc_applied[addresses] = 20.0 * error - 0.5 * data.qvel[addresses]
            mujoco.mj_step(model, data)
        data.qpos[:7], data.qvel[:] = free, 0.0
        data.qfrc_applied[:] = 0.0
        mujoco.mj_forward(model, data)
    finally:
        model.opt.gravity[:] = gravity
        model.opt.disableflags = disable
    miss = np.max(np.abs(np.array(legs) - data.qpos[qaddresses]))
    if miss > 0.01:
        raise RuntimeError(f"leg pose not reachable (off by {miss:.3f} rad)")


def model_com(legs=None) -> dict:
    """Mass and CoM of the simulated robot in the IMU frame, at the given leg pose."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    _pose(model, data, legs)
    site = model.site("imu").id
    origin, rotation = data.site_xpos[site].copy(), data.site_xmat[site].reshape(3, 3).copy()

    def local(world):
        return rotation.T @ (np.asarray(world) - origin)

    axle = 0.5 * sum(data.xanchor[model.joint(n).id] for n in WHEELS)
    masses = model.body_mass.copy()
    positions = np.array([local(p) for p in data.xipos])
    try:
        payload_id = model.body(PAYLOAD_BODY).id
    except KeyError:
        payload_id = -1
    base = np.ones(model.nbody, dtype=bool)
    if payload_id >= 0:
        base[payload_id] = False
    total = float(masses.sum())
    return {
        "axle": local(axle),
        "mass": total,
        "com": (masses[:, None] * positions).sum(axis=0) / total,
        "base_mass": float(masses[base].sum()),
        "base_com": (masses[base, None] * positions[base]).sum(axis=0) / masses[base].sum(),
        "payload_mass": float(masses[payload_id]) if payload_id >= 0 else 0.0,
        "payload_pos": positions[payload_id] if payload_id >= 0 else None,
        "wheel_mass": float(sum(masses[model.jnt_bodyid[model.joint(n).id]] for n in WHEELS)),
    }


def payload_for(sim: dict, mass: float, height: float, offset: float, keep_y: float):
    """Payload (mass, IMU-frame position) giving the measured total mass and CoM."""
    extra = mass - sim["base_mass"]
    if extra <= 0.0:
        raise ValueError(
            f"weighed {mass:.3f} kg is not more than the CAD's {sim['base_mass']:.3f} kg; "
            "a payload cannot remove mass")
    target = sim["axle"] + np.array([offset, 0.0, height])
    position = (mass * target - sim["base_mass"] * sim["base_com"]) / extra
    position[1] = keep_y
    return extra, position


def write_payload(mass: float, position) -> None:
    text = IDENTIFIED.read_text(encoding="utf-8")
    block = re.search(r"^payload:\n((?:[ \t]+.*\n)+)", text, flags=re.MULTILINE)
    if block is None:
        raise RuntimeError(f"no payload block in {IDENTIFIED}")
    body = block.group(1)
    body = re.sub(r"(mass_kg:\s*)[^\s#]+", lambda m: f"{m.group(1)}{mass:.3f}", body, count=1)
    body = re.sub(r"(position_m:\s*)\[[^\]]*\]",
                  lambda m: m.group(1) + "[" + ", ".join(f"{v:.3f}" for v in position) + "]",
                  body, count=1)
    IDENTIFIED.write_text(text[:block.start(1)] + body + text[block.end(1):], encoding="utf-8")


def controller_model() -> dict:
    try:
        import yaml
        with CONTROLLERS.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        return config["whole_body_controller"]["ros__parameters"]["model"]
    except Exception:  # noqa: BLE001 - only used for printed hints
        return {}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--mass", type=float, required=True, help="weighed total, kg")
    parser.add_argument("--tilt", type=parse_tilt, action="append", required=True,
                        metavar="PITCH_DEG,SCALE_KG,X_M")
    parser.add_argument("--legs", metavar="RI,RO,LI,LO",
                        help="leg joint positions (rad, /joint_states) of the locked pose; "
                             "default: the stand keyframe")
    parser.add_argument("--leg-height", type=float,
                        help="diagnostics leg_height_m at the locked pose, for com_height_offset_m")
    parser.add_argument("--sigma-pitch-deg", type=float, default=0.5)
    parser.add_argument("--sigma-scale-kg", type=float, default=0.002)
    parser.add_argument("--sigma-x-m", type=float, default=0.002)
    parser.add_argument("--write", action="store_true", help="update the payload in sim_identified.yaml")
    args = parser.parse_args()
    if len(args.tilt) < 2:
        parser.error("need at least two --tilt measurements")
    legs = None
    if args.legs:
        legs = [float(v) for v in args.legs.split(",")]
        if len(legs) != 4:
            parser.error("--legs needs four values")

    height, offset, sd_height, sd_offset, residual, condition = solve(
        args.mass, args.tilt, args.sigma_pitch_deg, args.sigma_scale_kg, args.sigma_x_m)
    print(f"measured   CoM height above axle L = {1e3 * height:7.1f} +- {1e3 * sd_height:.1f} mm")
    print(f"           CoM forward of axle   e = {1e3 * offset:7.1f} +- {1e3 * sd_offset:.1f} mm")
    if len(args.tilt) > 2:
        print(f"           fit residual {1e3 * residual:.2f} mm (should be near the tolerances)")
    if condition > 5.0:
        print(f"warning: tilts too alike (condition {condition:.1f}); use one forward and one "
              "backward pitch, 15 deg or more apart from upright", file=sys.stderr)

    try:
        sim = model_com(legs)
    except ImportError:
        print("mujoco not importable; skipping the model comparison", file=sys.stderr)
        return 0
    sim_rel = sim["com"] - sim["axle"]
    base_rel = sim["base_com"] - sim["axle"]
    print(f"\nmodel (pose: {'--legs' if legs else 'stand keyframe'})")
    print(f"  no payload   {sim['base_mass']:.3f} kg   L = {1e3 * base_rel[2]:6.1f} mm   "
          f"e = {1e3 * base_rel[0]:6.1f} mm")
    print(f"  with payload {sim['mass']:.3f} kg   L = {1e3 * sim_rel[2]:6.1f} mm   "
          f"e = {1e3 * sim_rel[0]:6.1f} mm   (current sim_identified.yaml)")

    keep_y = float(sim["payload_pos"][1]) if sim["payload_pos"] is not None else 0.0
    extra, position = payload_for(sim, args.mass, height, offset, keep_y)
    print("\npayload for config/sim_identified.yaml (IMU frame):")
    print(f"  mass_kg: {extra:.3f}")
    print(f"  position_m: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
    if np.linalg.norm(position) > 0.35:
        print("warning: the payload lands outside the body; the mass difference is too small "
              "to move the CoM that far, so check the measurements", file=sys.stderr)

    model = controller_model()
    wheel = float(model.get("wheel_mass_kg", sim["wheel_mass"] / 2.0))
    print("\ncontroller (config/sandro_real_overrides.yaml):")
    print(f"  model.body_mass_kg: {args.mass - 2.0 * wheel:.3f}")
    print(f"  tasks.fore_aft.neutral_offset_m: {offset - base_rel[0]:+.4f}"
          "   (measured e minus the CAD's at the same pose)")
    per_height = model.get("com_height_per_leg_height")
    if args.leg_height and per_height:
        print(f"  model.com_height_offset_m: {height - per_height * args.leg_height:+.4f}"
              f"   (L - {per_height} * {args.leg_height})")
    elif per_height:
        print("  model.com_height_offset_m: pass --leg-height (diagnostics leg_height_m) to compute")

    if args.write:
        write_payload(extra, position)
        print(f"\nwrote the payload to {IDENTIFIED}; now run tools/sync_model.py sync")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
