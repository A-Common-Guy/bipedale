#!/usr/bin/env python3
"""Derive the LqrBalancerController gains from the MuJoCo model.

The cascade controller's gains are hand-tuned; these are solved from the
plant. Run this whenever the chassis mass, wheel size or centre of mass
changes, and paste the printed block into sandro_bringup/config/controllers.yaml.

    mujoco_sim/venv/bin/python \
        ros2_ws/src/sandro_controllers/tools/lqr_design.py

State vector is [position_m, velocity_m_s, pitch_rad, pitch_rate_rad_s] and the
input is the common-mode wheel torque in Nm (the sum across both wheels).
"""

from __future__ import annotations

import argparse
import pathlib

import mujoco
import numpy as np

GRAVITY = 9.81
# Wheel bodies in the MJCF. Everything else is lumped into the pendulum.
WHEEL_BODIES = ("right_motor_shaft_2_3", "left_motor_shaft_2_2")


def extract_plant(model_path: pathlib.Path) -> dict:
    """Reduce the full model to planar wheeled-inverted-pendulum parameters."""
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    wheel_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        for name in WHEEL_BODIES
    ]
    if any(i < 0 for i in wheel_ids):
        raise SystemExit(f"could not find wheel bodies {WHEEL_BODIES}")

    axle_z = float(data.xipos[wheel_ids[0]][2])
    wheel_mass = float(sum(model.body_mass[i] for i in wheel_ids))
    # Spin inertia about the axle (body y) for both wheels together.
    wheel_inertia = float(sum(model.body_inertia[i][1] for i in wheel_ids))

    body_mass = 0.0
    weighted = np.zeros(3)
    for i in range(1, model.nbody):
        if i in wheel_ids:
            continue
        body_mass += float(model.body_mass[i])
        weighted += float(model.body_mass[i]) * data.xipos[i]
    com = weighted / body_mass
    com_height = float(com[2] - axle_z)

    # Pitch inertia of the pendulum about its own centre of mass.
    inertia_about_axle = 0.0
    for i in range(1, model.nbody):
        if i in wheel_ids:
            continue
        rot = data.ximat[i].reshape(3, 3)
        inertia = rot @ np.diag(model.body_inertia[i]) @ rot.T
        dx = float(data.xipos[i][0])
        dz = float(data.xipos[i][2]) - axle_z
        inertia_about_axle += float(inertia[1, 1]) + float(model.body_mass[i]) * (
            dx * dx + dz * dz
        )
    inertia_com = inertia_about_axle - body_mass * com_height**2

    radius = axle_z
    return {
        "body_mass": body_mass,
        "com_height": com_height,
        "inertia_com": inertia_com,
        "inertia_about_axle": inertia_about_axle,
        "wheel_mass": wheel_mass,
        "wheel_inertia": wheel_inertia,
        "wheel_radius": radius,
    }


def linearize(plant: dict) -> tuple[np.ndarray, np.ndarray]:
    """Continuous-time A, B about the upright equilibrium.

    Coupled equations of motion, linearised at pitch = 0:

        M11 * x_ddot + M12 * pitch_ddot = torque / radius
        M21 * x_ddot + M22 * pitch_ddot - m*g*l * pitch = -torque
    """
    mass = plant["body_mass"]
    length = plant["com_height"]
    radius = plant["wheel_radius"]

    m11 = mass + plant["wheel_mass"] + plant["wheel_inertia"] / radius**2
    m12 = mass * length
    m22 = plant["inertia_com"] + mass * length**2
    mass_matrix = np.array([[m11, m12], [m12, m22]])

    gravity_term = mass * GRAVITY * length
    # d/d(pitch) of the right-hand side, and the torque column.
    stiffness = np.array([0.0, gravity_term])
    torque_map = np.array([1.0 / radius, -1.0])

    inverse = np.linalg.inv(mass_matrix)
    accel_from_pitch = inverse @ stiffness
    accel_from_torque = inverse @ torque_map

    # [position, velocity, pitch, pitch_rate]
    a = np.zeros((4, 4))
    a[0, 1] = 1.0
    a[2, 3] = 1.0
    a[1, 2] = accel_from_pitch[0]
    a[3, 2] = accel_from_pitch[1]
    b = np.zeros((4, 1))
    b[1, 0] = accel_from_torque[0]
    b[3, 0] = accel_from_torque[1]
    return a, b


def solve_care(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray):
    """Continuous algebraic Riccati solution via the Hamiltonian eigenbasis.

    Avoids a scipy dependency; the venv only carries mujoco and numpy.
    """
    n = a.shape[0]
    hamiltonian = np.block(
        [[a, -b @ np.linalg.inv(r) @ b.T], [-q, -a.T]]
    )
    values, vectors = np.linalg.eig(hamiltonian)
    stable = np.argsort(values.real)[:n]
    basis = vectors[:, stable]
    upper, lower = basis[:n, :], basis[n:, :]
    riccati = np.real(lower @ np.linalg.inv(upper))
    return np.linalg.inv(r) @ b.T @ riccati


def main() -> None:
    root = pathlib.Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        type=pathlib.Path,
        default=root / "ros2_ws/src/sandro_description/mjcf/scene_ros2.xml",
    )
    parser.add_argument(
        "--wheel-radius",
        type=float,
        default=None,
        help="override the MJCF axle height, to match drive.wheel_radius_m",
    )
    parser.add_argument("--position-weight", type=float, default=4.0)
    parser.add_argument("--velocity-weight", type=float, default=8.0)
    parser.add_argument("--pitch-weight", type=float, default=120.0)
    parser.add_argument("--pitch-rate-weight", type=float, default=2.0)
    parser.add_argument("--effort-weight", type=float, default=6.0)
    args = parser.parse_args()

    plant = extract_plant(args.model)
    if args.wheel_radius is not None:
        plant["wheel_radius"] = args.wheel_radius
    a, b = linearize(plant)

    open_loop = np.linalg.eigvals(a)
    unstable = max(open_loop.real)

    q = np.diag(
        [
            args.position_weight,
            args.velocity_weight,
            args.pitch_weight,
            args.pitch_rate_weight,
        ]
    )
    r = np.array([[args.effort_weight]])
    riccati_gains = solve_care(a, b, q, r).ravel()
    closed_loop = np.linalg.eigvals(a - b @ riccati_gains.reshape(1, -1))
    # The controller applies torque = sum(gain * state), so fold in the sign of
    # the optimal law u = -Kx here rather than in the realtime path.
    gains = -riccati_gains

    print("plant, reduced from the MJCF")
    print(f"  pendulum mass        {plant['body_mass']:.3f} kg")
    print(f"  centre of mass       {plant['com_height'] * 100:.1f} cm above axle")
    print(f"  pitch inertia (CoM)  {plant['inertia_com']:.4f} kg m^2")
    print(f"  wheel pair mass      {plant['wheel_mass']:.3f} kg")
    print(f"  wheel pair inertia   {plant['wheel_inertia']:.5f} kg m^2")
    print(f"  wheel radius         {plant['wheel_radius']:.4f} m")
    print(f"  unstable pole        {unstable:.2f} rad/s "
          f"({np.log(2) / unstable * 1000:.0f} ms to double)")
    print()
    print("closed-loop poles")
    for pole in sorted(closed_loop, key=lambda v: v.real):
        print(f"  {pole.real:+.2f} {pole.imag:+.2f}j")
    print()
    print("paste into sandro_bringup/config/controllers.yaml under lqr_balancer,")
    print("as total wheel torque (the controller halves it per wheel):")
    print(f"      lqr.position_nm_m: {gains[0]:.4f}")
    print(f"      lqr.velocity_nm_s_m: {gains[1]:.4f}")
    print(f"      lqr.pitch_nm_rad: {gains[2]:.4f}")
    print(f"      lqr.pitch_rate_nm_s_rad: {gains[3]:.4f}")
    print()
    print("the position gain is positive on purpose: to move backward the")
    print("wheels must first drive forward to tip the body that way.")


if __name__ == "__main__":
    main()
