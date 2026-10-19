#!/usr/bin/env python3
"""Compare LQR weightings on the reduced plant before committing to hardware.

Solves gains for each candidate weighting, runs the same control law the
controller uses against the linearised plant, and reports what you actually
care about: how fast it reaches a commanded speed, how hard it leans and pushes
to get there, and how well it recovers from a shove.

    python3 ros2_ws/src/sandro_controllers/tools/lqr_tune.py   # needs pip install mujoco
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from lqr_design import extract_plant, linearize, solve_care  # noqa: E402

# Matches sandro_bringup/config/controllers.yaml.
WHEEL_TORQUE_LIMIT_NM = 6.0
FORWARD_ACCEL_LIMIT = 1.0
DT = 0.001


def gains_for(plant: dict, weights: dict) -> np.ndarray:
    a, b = linearize(plant)
    q = np.diag([weights["position"], weights["velocity"],
                 weights["pitch"], weights["pitch_rate"]])
    r = np.array([[weights["effort"]]])
    return -solve_care(a, b, q, r).ravel()


def simulate(plant: dict, gains: np.ndarray, command: float, pitch0: float,
             seconds: float) -> dict:
    """Closed loop on the linearised plant, mirroring LqrBalancerCore."""
    a, b = linearize(plant)
    state = np.array([0.0, 0.0, pitch0, 0.0])  # position, velocity, pitch, rate
    reference = 0.0
    slewed = 0.0
    peak_torque = 0.0
    peak_pitch = abs(pitch0)
    min_position = 0.0
    reached = None
    steps = int(seconds / DT)
    for i in range(steps):
        slewed += np.clip(command - slewed, -FORWARD_ACCEL_LIMIT * DT,
                          FORWARD_ACCEL_LIMIT * DT)
        # Driving pins the position reference to the robot, so only the speed
        # error acts; released, the position gain holds station.
        if abs(command) > 0.01 or abs(slewed) > 0.01:
            reference = state[0]
        errors = np.array([state[0] - reference, state[1] - slewed,
                           state[2], state[3]])
        total = float(gains @ errors)
        per_wheel = np.clip(0.5 * total, -WHEEL_TORQUE_LIMIT_NM,
                            WHEEL_TORQUE_LIMIT_NM)
        total = 2.0 * per_wheel
        state = state + (a @ state + b.ravel() * total) * DT
        peak_torque = max(peak_torque, abs(per_wheel))
        peak_pitch = max(peak_pitch, abs(state[2]))
        min_position = min(min_position, state[0])
        if reached is None and command and state[1] >= 0.9 * command:
            reached = i * DT
    return {
        "peak_torque_per_wheel": peak_torque,
        "peak_pitch_deg": np.degrees(peak_pitch),
        "backward_excursion_mm": -min_position * 1000.0,
        "reach_90pct_s": reached,
        "final_velocity": state[1],
        "final_pitch_deg": np.degrees(state[2]),
        "final_position": state[0],
    }


def recover(plant: dict, gains: np.ndarray, pitch0: float) -> dict:
    """Response to a shove: time to park upright, and what it costs."""
    a, b = linearize(plant)
    state = np.array([0.0, 0.0, pitch0, 0.0])
    last_bad = 0.0
    peak_torque = 0.0
    for i in range(int(12.0 / DT)):
        total = float(gains @ state)
        per_wheel = np.clip(0.5 * total, -WHEEL_TORQUE_LIMIT_NM,
                            WHEEL_TORQUE_LIMIT_NM)
        state = state + (a @ state + b.ravel() * 2.0 * per_wheel) * DT
        peak_torque = max(peak_torque, abs(per_wheel))
        if abs(state[2]) > 0.01 or abs(state[0]) > 0.02:
            last_bad = i * DT
    return {"settle_s": last_bad, "peak_torque_per_wheel": peak_torque}


def loop_margin(plant: dict, gains: np.ndarray, rate_hz: float) -> dict:
    """How close the design sits to what a 300 Hz loop can actually track.

    Sampled feedback holds each command for a whole period, so a closed-loop
    pole approaching the loop rate stops being stabilised and starts being
    excited. Keep the fastest pole roughly an order of magnitude below it.
    """
    a, b = linearize(plant)
    closed = np.linalg.eigvals(a - b @ (-gains).reshape(1, -1))
    fastest_hz = max(abs(p.real) for p in closed) / (2.0 * np.pi)
    return {
        "fastest_pole_hz": fastest_hz,
        "fraction_of_loop": fastest_hz / rate_hz,
        "stable": bool(max(p.real for p in closed) < 0.0),
    }


# Every one of these is comfortably inside the torque and loop-rate budgets
# the metrics below report, so those are not what decides between them. What
# decides it is the five-bar leg compliance, which the rigid reduction cannot
# see: the legs are a springy element between the wheels and the body mass, and
# past a certain wheel gain the two start fighting. Each entry carries what
# MuJoCo actually did, because the linear numbers alone will mislead you here.
CANDIDATES = {
    # Stable, but only reaches 0.16 m/s of a 0.30 m/s command: too little
    # velocity gain to push through friction the rigid model ignores.
    "softer": dict(position=2.0, velocity=4.0, pitch=160.0, pitch_rate=3.0,
                   effort=20.0),
    # Shipped. Balances indefinitely and tracks 0.308 m/s against 0.300.
    "shipped": dict(position=4.0, velocity=8.0, pitch=120.0, pitch_rate=2.0,
                    effort=6.0),
    # Falls. Only 28% more pitch gain than shipped, which is how narrow the
    # margin is and why these have to be tried in MuJoCo rather than trusted.
    "brisk": dict(position=7.0, velocity=14.0, pitch=110.0, pitch_rate=1.8,
                  effort=3.5),
    # Falls.
    "snappier": dict(position=12.0, velocity=25.0, pitch=100.0,
                     pitch_rate=1.5, effort=2.0),
    "aggressive": dict(position=40.0, velocity=70.0, pitch=80.0,
                       pitch_rate=1.0, effort=0.8),
}


def main() -> None:
    root = pathlib.Path(__file__).resolve().parents[4]
    plant = extract_plant(
        root / "ros2_ws/src/sandro_description/mjcf/scene_ros2.xml")
    plant["wheel_radius"] = 0.0742

    rate_hz = 300.0  # controller_manager update_rate
    gyro_noise_rad_s = 0.02  # BNO085, order of magnitude

    print("driving to 0.3 m/s, and recovering from a 7 degree shove")
    print(f"{'weighting':12} {'reach 90%':>10} {'lean':>7} {'roll-back':>10} "
          f"{'settle':>8} {'shove tau':>10} {'pole':>8} {'noise tau':>10}")
    print("-" * 82)
    results = {}
    for name, weights in CANDIDATES.items():
        gains = gains_for(plant, weights)
        drive = simulate(plant, gains, command=0.3, pitch0=0.0, seconds=8.0)
        shove = recover(plant, gains, pitch0=0.12)
        margin = loop_margin(plant, gains, rate_hz)
        results[name] = (gains, drive, shove, margin)
        reach = (f"{drive['reach_90pct_s']:.2f}s"
                 if drive["reach_90pct_s"] is not None else "never")
        print(f"{name:12} {reach:>10} {drive['peak_pitch_deg']:6.1f}d "
              f"{drive['backward_excursion_mm']:8.1f}mm "
              f"{shove['settle_s']:7.2f}s "
              f"{shove['peak_torque_per_wheel']:9.2f}N "
              f"{margin['fastest_pole_hz']:7.1f}Hz "
              f"{gains[3] * gyro_noise_rad_s:9.3f}N")

    print()
    print(f"the loop runs at {rate_hz:.0f} Hz; keep the fastest pole an order "
          "of magnitude below it")
    for name, (_, _, _, margin) in results.items():
        verdict = "ok" if margin["fraction_of_loop"] < 0.1 else "TOO FAST"
        share = margin["fraction_of_loop"] * 100.0
        print(f"  {name:12} fastest pole {margin['fastest_pole_hz']:6.1f} Hz, "
              f"{share:5.1f}% of loop rate  {verdict}")
    print()
    for name, (gains, _, _, _) in results.items():
        print(f"  # {name}")
        print(f"      position_nm_m: {gains[0]:.4f}")
        print(f"      velocity_nm_s_m: {gains[1]:.4f}")
        print(f"      pitch_nm_rad: {gains[2]:.4f}")
        print(f"      pitch_rate_nm_s_rad: {gains[3]:.4f}")


if __name__ == "__main__":
    main()
