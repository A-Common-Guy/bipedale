"""How long a one-leg policy (commands.jump.one_leg) stands on one wheel per request:
evaluate.py's `jump` scenario (three requests standing, two at 0.5 m/s) on the MJCF and the
fitted plant, three latency profiles, three seeds.

    cd sandro_rl && uv run python scripts/one_leg_runs.py policies/fun_one_leg

Per policy: requests answered (one wheel more than 1.5 cm up with the other on the floor for
at least 0.5 s), the time on one wheel and the lift per request, the largest roll, and falls.
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


def one(job):
    from sandro_rl import evaluate as e
    policy, plant, profile, seed = job
    out = Path(f"/tmp/one_leg_runs/{Path(policy).name}_{plant}_{profile}_{seed}.csv")
    code, path = e.run(Path(policy), "jump", {**e.PROFILES[profile], **(FITTED if plant else {})}, out, seed=seed)
    rows = [r for r in csv.DictReader(open(path)) if r.get("diag_truth_wheel_lift_m") not in ("", None)]
    t, high, low, roll, request = (np.array([float(r[k] or 0) for r in rows]) for k in (
        "t_s", "diag_truth_wheel_lift_m", "diag_truth_wheel_clearance_m", "diag_truth_roll_rad", "cmd_jump"))
    dt = float(np.median(np.diff(t)))
    stands = []
    for start in t[1:][(request[1:] > 0) & (request[:-1] <= 0)]:
        window = (t >= start) & (t < start + 3.5)
        stands.append(((window & (high > 0.015) & (low < 0.005)).sum() * dt, high[window].max()))
    return code == 3, stands, float(np.abs(roll).max())


def main():
    Path("/tmp/one_leg_runs").mkdir(exist_ok=True)
    for policy in sys.argv[1:]:
        jobs = [(policy, plant, profile, seed) for plant in (0, 1) for profile in ("nominal", "spi_slow", "mit")
                for seed in (1, 2, 3)]
        with Pool(14) as pool:
            results = pool.map(one, jobs)
        stands = [s for r in results for s in r[1]]
        seconds = sorted(s[0] for s in stands)
        lifts = sorted(s[1] for s in stands)
        print(f"{policy}: {sum(s >= 0.5 for s in seconds)} of {len(stands)} requests answered, on one wheel "
              f"{seconds[0]:.2f} .. {seconds[len(seconds) // 2]:.2f} .. {seconds[-1]:.2f} s, lift "
              f"{lifts[0]:.3f} .. {lifts[len(lifts) // 2]:.3f} .. {lifts[-1]:.3f} m, roll up to "
              f"{max(r[2] for r in results):.2f} rad, falls {sum(r[0] for r in results)} of {len(results)}")


if __name__ == "__main__":
    main()
