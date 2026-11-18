"""The camera's effect: the guard course (a 2 cm plank at 1.0 m to cross, a 15 cm block at
2.4 m to stop in front of) at 0.5 m/s, on the MJCF and the fitted plant, three latency
profiles, three seeds. POLICY:camera=0 runs a perceptive policy with its profile unknown.

    cd sandro_rl && uv run python scripts/guard_runs.py policies/agile_guard policies/agile_guard:camera=0 policies/agile_terrain_v6

Per policy: runs that crossed the plank, runs that stopped before touching the block, the
gap left between the tyre and the block's face (m; smallest, median, largest; negative: into
or past it), the speed at which the tyre met the face (0 if it never did), and falls.
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
PLANK_M, FACE_M, TYRE_M = 1.0, 2.4 - 0.05, 0.067


def one(job):
    from sandro_rl import evaluate as e
    spec, plant, profile, seed = job
    policy, _, option = spec.partition(":")
    sim = {**e.PROFILES[profile], **PLANTS[plant], **(dict([option.split("=")]) if option else {})}
    out = Path(f"/tmp/guard_runs/{Path(policy).name}{option}_{plant}_{profile}_{seed}.csv")
    code, path = e.run(Path(policy), "guard", sim, out, seed=seed, course="guard")
    rows = [r for r in csv.DictReader(open(path)) if r.get("diag_truth_axle_x_m") not in ("", None)]
    xy = [(float(r["diag_truth_axle_x_m"]), float(r["diag_truth_axle_y_m"])) for r in rows]
    speed = [float(r["diag_truth_speed_m_s"]) for r in rows]
    x0, y0 = xy[0]
    # The course runs along the start heading: the direction of the first 0.5 m driven.
    first = next(((x - x0, y - y0) for x, y in xy if math.hypot(x - x0, y - y0) > 0.5), (1.0, 0.0))
    norm = math.hypot(*first)
    along = [((x - x0) * first[0] + (y - y0) * first[1]) / norm for x, y in xy]
    touch = FACE_M - TYRE_M
    hit = next((i for i, a in enumerate(along) if a >= touch - 0.005), None)
    return {"fell": code == 3, "plank": max(along) > PLANK_M + 0.2, "gap": touch - max(along),
            "speed": 0.0 if hit is None else abs(speed[hit])}


def main():
    Path("/tmp/guard_runs").mkdir(exist_ok=True)
    jobs = [(spec, plant, profile, seed) for spec in sys.argv[1:] for plant in PLANTS
            for profile in PROFILES for seed in SEEDS]
    with Pool(14) as pool:
        results = dict(zip(jobs, pool.map(one, jobs)))
    n = len(PLANTS) * len(PROFILES) * len(SEEDS)
    print(f"{'policy':40s} plank  stopped  gap to the block, m      speed at impact    falls   (of {n})")
    for spec in sys.argv[1:]:
        rs = [r for k, r in results.items() if k[0] == spec]
        gaps = sorted(r["gap"] for r in rs)
        print(f"{spec:40s} {sum(r['plank'] for r in rs):3d}    {sum(r['gap'] > 0.01 and not r['fell'] for r in rs):3d}      "
              f"{gaps[0]:+.2f} .. {gaps[len(gaps) // 2]:+.2f} .. {gaps[-1]:+.2f}     "
              f"{max(r['speed'] for r in rs):.2f} m/s           {sum(r['fell'] for r in rs):3d}")


if __name__ == "__main__":
    main()
