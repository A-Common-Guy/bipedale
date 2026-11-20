"""The three answers to the terrain on one course (evaluate.py COURSES "three"): a 1.5 cm
plank at 0.9 m to drive over, a 5 cm step with a 1 m top from 1.9 m to hop onto and drop off,
a 15 cm block at 4.3 m to stop at; 0.5 m/s, the MJCF and the fitted plant, three latency
profiles, three seeds. POLICY:camera=0 leaves the terrain profile unknown; --step M sets the
step's height (0.05).

    cd sandro_rl && uv run python scripts/three_runs.py policies/agile_step policies/agile_step:camera=0

Per policy: runs over the plank, onto the step (the axle 0.3 m past its face, upright), off
its far end, stopped short of the block (the tyre 1 to 60 cm from the face, on the course),
runs that drifted past the block's end (off course), the highest wheel
clearance in the 0.5 m before the step, the speed at which the tyre met the block, and falls.
"""
import csv
import math
import sys
import warnings
from multiprocessing import Pool
from pathlib import Path

warnings.filterwarnings("ignore")

FITTED = {"wheel_lag": "0", "wheel_backlash": "0.000001", "rotor_inertia": "22", "gear_stiffness": "19",
          "gear_damping": "0.1", "tyre_torsion": "0.01", "wheel_friction": "0", "leg_armature": "22"}
PLANTS = {"mjcf": {}, "fitted": FITTED}
PROFILES = ("nominal", "spi_slow", "mit")
SEEDS = (1, 2, 3)
PLANK_M, STEP_M, STEP_END_M, FACE_M, TYRE_M = 0.9, 1.9, 2.9, 4.3 - 0.05, 0.067


def one(job):
    from sandro_rl import evaluate as e
    spec, plant, profile, seed, step = job
    e.COURSES["three"][1]["height_m"] = step
    policy, _, option = spec.partition(":")
    sim = {**e.PROFILES[profile], **PLANTS[plant], **(dict([option.split("=")]) if option else {})}
    out = Path(f"/tmp/three_runs/{Path(policy).name}{option}_{plant}_{profile}_{seed}_{step}.csv")
    code, path = e.run(Path(policy), "three", sim, out, seed=seed, course="three")
    rows = [r for r in csv.DictReader(open(path)) if r.get("diag_truth_axle_x_m") not in ("", None)]
    xy = [(float(r["diag_truth_axle_x_m"]), float(r["diag_truth_axle_y_m"])) for r in rows]
    speed = [float(r["diag_truth_speed_m_s"]) for r in rows]
    lift = [float(r["diag_truth_wheel_clearance_m"]) for r in rows]
    tilt = [float(r["diag_truth_tilt_rad"]) for r in rows]
    # The course lies along the world's x from the origin, 1.2 m wide at the block.
    along = [x for x, _ in xy]
    beside = max(abs(y) for x, y in xy if x > FACE_M - 0.3) if along and max(along) > FACE_M - 0.3 else 0.0
    upright_at = lambda d: any(a >= d and t < 0.4 for a, t in zip(along, tilt))
    touch = FACE_M - TYRE_M
    hit = next((i for i, a in enumerate(along) if a >= touch - 0.005), None)
    approach = [c for a, c in zip(along, lift) if STEP_M - 0.5 <= a <= STEP_M + 0.1]
    fell = code == 3
    return {"fell": fell, "plank": upright_at(PLANK_M + 0.3), "on": upright_at(STEP_M + 0.3),
            "off": upright_at(STEP_END_M + 0.3),
            "stopped": not fell and 0.01 < touch - max(along) < 0.6 and beside < 0.4, "beside": beside >= 0.4,
            "hop": max([c for c in approach if not math.isnan(c)], default=0.0),
            "impact": 0.0 if hit is None or beside >= 0.4 else abs(speed[hit])}


def main():
    Path("/tmp/three_runs").mkdir(exist_ok=True)
    args = sys.argv[1:]
    step = float(args.pop(args.index("--step") + 1)) if "--step" in args else 0.05
    args = [a for a in args if a != "--step"]
    jobs = [(spec, plant, profile, seed, step) for spec in args for plant in PLANTS
            for profile in PROFILES for seed in SEEDS]
    with Pool(14) as pool:
        results = dict(zip(jobs, pool.map(one, jobs)))
    n = len(PLANTS) * len(PROFILES) * len(SEEDS)
    print(f"{'policy':38s} plank  on step  off step  stopped  hop before the step, m   impact, m/s  falls  off course   (of {n})")
    for spec in args:
        rs = [r for k, r in results.items() if k[0] == spec]
        hops = sorted(r["hop"] for r in rs)
        count = lambda key: sum(bool(r[key]) for r in rs)
        print(f"{spec:38s} {count('plank'):3d}    {count('on'):3d}      {count('off'):3d}       {count('stopped'):3d}     "
              f"{hops[0]:.3f} .. {hops[len(hops) // 2]:.3f} .. {hops[-1]:.3f}     {max(r['impact'] for r in rs):.2f}        {count('fell'):3d}      {count('beside'):3d}")


if __name__ == "__main__":
    main()
