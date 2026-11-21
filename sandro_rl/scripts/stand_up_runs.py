"""How reliably a stand-up policy gets up (docs/DESIGN.md 8, stand_up): the robot tipped by
each pitch, limp for 3 s, then the policy; 8 s to get up, then 3 s at 0.4 m/s, 3 s turning at
1 rad/s, 2 s still. Pitches +-0.3 .. +-1.4 rad x seeds x latency profiles x plants (the MJCF
with 40 and 80 ms of wheel lag, and the fitted drive train; --hard: see HARD; --held: the legs
not limp but held where they were, six draws over their reach, on the fitted plant; --grid: the
legs held alike over a grid of their whole reach, 48 poses and limp, lying up to 1.9 rad either
way, one seed and the nominal latency: where a policy fails, not how often).

--outline: the robot lies on the hulls of its CAD meshes (evaluate.LEG_HULLS = 0), not on the
link boxes: what it is on the floor, whatever the policy trained on (docs/DESIGN.md 30).

    cd sandro_rl && uv run python scripts/stand_up_runs.py policies/stand_up [more policies]

Success: tilt below 0.25 rad with the IMU above 0.27 m at some point, and the tilt never above
0.5 rad from the first drive phase on. Per policy: successes, the time to get up (median and
longest), and the largest tilt afterwards.
"""
import csv
import os
import statistics
import sys
import warnings
from multiprocessing import Pool
from pathlib import Path

warnings.filterwarnings("ignore")

PITCHES = (-1.4, -1.1, -0.8, -0.5, -0.3, 0.3, 0.5, 0.8, 1.1, 1.4)
SEEDS = (1, 2)
PROFILES = ("nominal", "spi_slow", "worst")
FITTED = {"wheel_lag": "0", "wheel_backlash": "0.000001", "rotor_inertia": "22", "gear_stiffness": "19",
          "gear_damping": "0.1", "tyre_torsion": "0.01", "wheel_friction": "0", "leg_armature": "22"}
PLANTS = {"lag 40 ms": {"wheel_lag": "0.04"}, "lag 80 ms": {"wheel_lag": "0.08"}, "fitted": FITTED}
# --hard: what the standard grid does not have. The fitted plant with a slippery and a grippy
# floor, drives at 80 % of their torque, and tipped 0.3 rad to a side as well.
HARD = {"slippery": {**FITTED, "floor_friction": "0.5"}, "grippy": {**FITTED, "floor_friction": "1.5"},
        "weak drives": {**FITTED, "torque_scale": "0.8"}, "rolled": {**FITTED, "fallen_roll": "0.3"},
        "rolled, slippery, weak": {**FITTED, "fallen_roll": "-0.3", "floor_friction": "0.6", "torque_scale": "0.85"}}
HELD = {f"legs {n}": {**FITTED, "fallen_legs": str(n)} for n in range(1, 7)}
if "--held" in sys.argv:
    sys.argv.remove("--held")
    PLANTS = HELD
    PITCHES = (-1.6, -1.4, -1.1, -0.8, -0.5, 0.5, 0.8, 1.1, 1.4, 1.6)
if "--hard" in sys.argv:
    sys.argv.remove("--hard")
    PLANTS = HARD
OUTLINE = "--outline" in sys.argv
if OUTLINE:
    sys.argv.remove("--outline")
GRID = None


def grid_poses():
    """name -> leg joint targets (leg_index order), both legs alike: an 8 x 8 grid of the two
    drive angles, kept where the linkage closes; named by the axle's height below the pivots,
    its offset fore and aft and the reach (1 = stretched)."""
    import math
    import numpy as np
    from sandro_rl import kinematics
    from sandro_rl.robot import RobotSpec
    g = RobotSpec.load().limits.leg_geometry["right"]
    out = {}
    for inner in np.linspace(-1.0, 1.7, 8):
        for outer in np.linspace(-1.0, 1.7, 8):
            p = g["upper_m"] * (math.cos(outer) - math.cos(inner))
            q = g["base_m"] + g["upper_m"] * (math.sin(inner) + math.sin(outer))
            if math.hypot(p, q) >= 2.0 * g["lower_m"]:
                continue
            x, y, reach = kinematics.forward(inner, outer, g["base_m"], g["upper_m"], g["lower_m"])
            if y >= 0.03 and reach <= 0.99:
                out[f"y{y:.3f} x{x - 0.5 * g['base_m']:+.3f} r{reach:.2f}"] = [inner, outer, outer, inner]
    return out


if "--grid" in sys.argv:
    sys.argv.remove("--grid")
    GRID = grid_poses()
    PLANTS = {"limp": FITTED, **{name: {**FITTED, "fallen_legs": "1"} for name in GRID}}
    PITCHES = (-1.9, -1.6, -1.3, -1.0, 1.0, 1.3, 1.6, 1.9)
    SEEDS = (1,)
    PROFILES = ("nominal",)


def one(job):
    from sandro_rl import evaluate as e, fallen
    policy, plant, profile, pitch, seed = job
    if OUTLINE:
        e.LEG_HULLS = 0
    if GRID is not None and plant in GRID:
        import numpy as np
        fallen.leg_targets = lambda spec, rng, same=None, targets=GRID[plant]: np.array(targets)
    out = Path(f"/tmp/stand_up_runs/{Path(policy).name}_{'outline_' if OUTLINE else ''}"
               f"{plant.replace(' ', '').replace(',', '')}_{profile}_{pitch}_{seed}.csv")
    _, path = e.run(Path(policy), "stand_up", {**e.PROFILES[profile], **PLANTS[plant]}, out, seed=seed,
                    fallen_pitch=pitch)
    rows = [r for r in csv.DictReader(open(path)) if r.get("diag_truth_tilt_rad") not in ("", None)]
    t = [float(r["t_s"]) for r in rows]
    tilt = [float(r["diag_truth_tilt_rad"]) for r in rows]
    height = [float(r["diag_truth_imu_height_m"]) for r in rows]
    phase = [int(float(r["phase"])) for r in rows]
    up = next((i for i in range(len(rows)) if tilt[i] < 0.25 and height[i] > 0.27), None)
    start = t[0]
    driving = [tilt[i] for i in range(len(rows)) if phase[i] >= 1]
    worst = max(driving) if driving else 9.9
    # Tyres clear of the floor when the policy engages: turning them moves nothing.
    clear = float(rows[0]["diag_truth_wheel_clearance_m"]) > 0.01
    return {"ok": up is not None and worst <= 0.5, "time": None if up is None else t[up] - start, "tilt": worst,
            "clear": clear}


def main():
    Path("/tmp/stand_up_runs").mkdir(exist_ok=True)
    policies = sys.argv[1:]
    jobs = [(p, plant, profile, pitch, seed) for p in policies for plant in PLANTS for profile in PROFILES
            for pitch in PITCHES for seed in SEEDS]
    with Pool(int(os.environ.get("POOL", 14)), maxtasksperchild=8) as pool:
        results = dict(zip(jobs, pool.map(one, jobs, chunksize=1)))
    for p in policies:
        rs = {k: r for k, r in results.items() if k[0] == p}
        times = [r["time"] for r in rs.values() if r["ok"]]
        per_plant = ", ".join(f"{plant} {sum(r['ok'] for k, r in rs.items() if k[1] == plant)}/"
                              f"{sum(1 for k in rs if k[1] == plant)}" for plant in PLANTS)
        failed = sorted({(k[3]) for k, r in rs.items() if not r["ok"]})
        if GRID is not None:
            bad = [(k, r) for k, r in rs.items() if not r["ok"]]
            never = [k for k, r in bad if r["time"] is None]
            clear = [r for r in rs.values() if r["clear"]]
            per_plant = (f"never up {len(never)}, up only after the 8 s {len(bad) - len(never)}; tyres clear of the "
                         f"floor at the start {sum(r['ok'] for r in clear)}/{len(clear)}; poses with a failure: "
                         + ", ".join(sorted({k[1] for k, _ in bad})))
        print(f"{p:32s} {sum(r['ok'] for r in rs.values()):3d} of {len(rs)}   {per_plant}   up in "
              f"{statistics.median(times):.2f} s median, {max(times):.2f} s at most; tilt afterwards up to "
              f"{max(r['tilt'] for r in rs.values() if r['ok']):.2f} rad; failed pitches {failed}")


if __name__ == "__main__":
    main()
