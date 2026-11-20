"""What a show policy (configs/fun_show.yaml) does with the sticks at their ends: spins on
the spot and while driving, the stops after them, the sway, the bob and a hop in a spin and
standing. On the MJCF and the fitted plant, three latency profiles, three seeds (18 runs).

    cd sandro_rl && uv run python scripts/show_runs.py policies/fun_show

Per policy: the yaw rate reached per spin against the one asked for, the yaw rate left 1 s
after a stop, how far the axle wandered in a spin on the spot, the roll and the heights
reached, hops answered (both wheels more than 1.5 cm up), the fastest wheel outside the hops
and in them (the drives limit theirs to 19.5 rad/s), the largest leg torque and tilt, and falls.
"""
import csv
import sys
import warnings
from multiprocessing import Pool
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore")

FITTED = {"wheel_lag": "0", "wheel_backlash": "0.000001", "rotor_inertia": "22", "gear_stiffness": "19",
          "gear_damping": "0.1", "tyre_torsion": "0.01", "wheel_friction": "0", "leg_armature": "22"}
OUT = Path(__file__).resolve().parents[1] / "runs" / "show_runs"

# duration, forward, yaw, height rate, roll, hop
SHOW = ["2,0,0", "3,0,5", "1.5,0,0", "3,0,-5", "1.5,0,0", "3,0.25,4", "1.5,0,0",       # 0-6
        "0.5,0,0,-0.08", "1.5,0,0,0,0.4", "1.5,0,0,0,-0.4", "1,0,0",                     # 7-10
        "1,0,0,-0.4", "1,0,0,0.4", "1,0,0,-0.4", "1,0,0,0.4", "1.5,0,0",                # 11-15
        "2,0,4", "1,0,4,0,0,0.05", "2,0,0", "1,0,0,0,0,0.05", "2,0,0"]                  # 16-20
SPINS, STOPS, ROLLS, LOW, HIGH, HOPS = (1, 3, 5), (2, 4, 6), (8, 9), (11, 13), (12, 14), (17, 19)
# The video: a spin, the sway, the bob, a hop in a spin, a hop standing.
VIDEO = ["1.5,0,0", "2.5,0,5", "1.5,0,0", "0.5,0,0,-0.08", "1.2,0,0,0,0.4", "1.2,0,0,0,-0.4", "1.2,0,0,0,0.4",
         "0.8,0,0", "0.8,0,0,-0.4", "0.8,0,0,0.4", "0.8,0,0,-0.4", "0.8,0,0,0.4", "1,0,0",
         "1.5,0,-4", "1,0,-4,0,0,0.05", "1.5,0,0", "1,0,0,0,0,0.05", "1.5,0,0"]


def one(job):
    from sandro_rl import evaluate as e
    policy, plant, profile, seed = job
    e.SCENARIOS["show"] = SHOW
    OUT.mkdir(parents=True, exist_ok=True)
    out = OUT / f"{Path(policy).name}_{plant}_{profile}_{seed}.csv"
    code, path = e.run(Path(policy), "show", {**e.PROFILES[profile], **(FITTED if plant else {})}, out, seed=seed)
    rows = [r for r in csv.DictReader(open(path)) if r.get("diag_truth_wheel_lift_m") not in ("", None)]
    col = lambda k: np.array([float(r[k] or 0) for r in rows])
    t, phase, yaw, roll, height, clear, tilt = (col(k) for k in (
        "t_s", "phase", "diag_truth_yaw_rate_rad_s", "diag_truth_roll_rad", "diag_truth_height_m",
        "diag_truth_wheel_clearance_m", "diag_truth_tilt_rad"))
    x, y, asked = col("diag_truth_axle_x_m"), col("diag_truth_axle_y_m"), col("cmd_yaw")
    joints = [r for r in csv.DictReader(open(path)) if r["stream"] == "joint"]
    jcol = lambda k: np.array([float(r[k] or 0) for r in joints])
    wheels = np.abs(np.stack([jcol("vel_right_wheel"), jcol("vel_left_wheel")])).max(axis=0)
    legs = np.abs(np.stack([jcol(f"eff_{s}_motor_{j}") for s in ("right", "left") for j in ("inner", "outer")])).max()
    # In a hop the unloaded wheels spin up; the drives' speed limit caps that. Reported apart.
    jphase = jcol("phase").astype(int)
    hopping = np.isin(jphase, [h + k for h in HOPS for k in (0, 1)])
    result = {"fell": code == 3, "spins": [], "stops": [], "wander": [], "rolls": [], "low": [], "high": [],
              "hops": [], "tilt": float(tilt.max()), "legs": float(legs)}
    result["wheel"] = float(wheels[~hopping].max())
    result["wheel_hop"] = float(wheels[hopping].max()) if hopping.any() else 0.0
    for p in range(len(SHOW)):
        mask = phase == p
        if not mask.any():
            continue
        tp = t[mask] - t[mask][0]
        last = tp > tp[-1] - 1.0
        if p in SPINS:
            result["spins"].append(float(yaw[mask][last].mean() / asked[mask][-1]))
        if p in (1, 3):
            result["wander"].append(float(np.hypot(x[mask] - x[mask][0], y[mask] - y[mask][0]).max()))
        if p in STOPS:
            result["stops"].append(float(np.abs(yaw[mask][tp > 1.0]).max()))
        if p in ROLLS:
            result["rolls"].append(float(np.abs(roll[mask][tp > tp[-1] - 0.5]).mean()))
        if p in LOW:
            result["low"].append(float(height[mask][-10:].mean()))
        if p in HIGH:
            result["high"].append(float(height[mask][-10:].mean()))
        if p in HOPS:
            window = (t >= t[mask][0]) & (t < t[mask][0] + 1.3)
            result["hops"].append(float(clear[window].max()))
    return result


def span(values, digits=2):
    values = sorted(values)
    return (f"{values[0]:.{digits}f} .. {values[len(values) // 2]:.{digits}f} .. {values[-1]:.{digits}f}"
            if values else "none")


def video(policy):
    from sandro_rl import evaluate as e
    e.SCENARIOS["show_video"] = VIDEO
    policy = Path(policy)
    # On the fitted plant: the robot's tyres turn on the spot as freely as that, not as the MJCF's.
    e.run(policy, "show_video", {**e.PROFILES["nominal"], **FITTED}, OUT / "video.csv",
          video=policy / f"{policy.name}.mp4")


def main():
    args = sys.argv[1:]
    if "--video" in args:
        for policy in (a for a in args if a != "--video"):
            video(policy)
        return
    for policy in args:
        jobs = [(policy, plant, profile, seed) for plant in (0, 1) for profile in ("nominal", "spi_slow", "mit")
                for seed in (1, 2, 3)]
        with Pool(14) as pool:
            results = pool.map(one, jobs)
        every = lambda key: [v for r in results for v in r[key]]
        hops = every("hops")
        print(f"{policy}: falls {sum(r['fell'] for r in results)} of {len(results)}")
        print(f"    spin reached / asked {span(every('spins'))}; yaw rate 1 s after the stop up to "
              f"{max(every('stops') or [0]):.2f} rad/s; wander in a spin on the spot {span(every('wander'))} m")
        print(f"    roll held {span(every('rolls'))} rad; height low {span(every('low'), 3)} m, high "
              f"{span(every('high'), 3)} m")
        print(f"    hops answered {sum(h > 0.015 for h in hops)} of {len(hops)}, clearance {span(hops, 3)} m")
        print(f"    fastest wheel {max(r['wheel'] for r in results):.1f} rad/s outside the hops, "
              f"{max(r['wheel_hop'] for r in results):.1f} in them; leg torque up to "
              f"{max(r['legs'] for r in results):.1f} Nm, tilt up to {max(r['tilt'] for r in results):.2f} rad")


if __name__ == "__main__":
    main()
