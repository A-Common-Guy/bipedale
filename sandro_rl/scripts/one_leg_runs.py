"""How long a one-leg policy (commands.jump.one_leg) stands on one wheel per request:
two requests standing and one at 0.5 m/s, on the MJCF and the fitted plant, three latency
profiles, three seeds. --hold S: the policy's window (commands.jump.window_s, 2 s).

    cd sandro_rl && uv run python scripts/one_leg_runs.py policies/fun_one_leg
    cd sandro_rl && uv run python scripts/one_leg_runs.py --hold 8 policies/fun_one_leg_long

Per policy: requests answered (one wheel more than 1.5 cm up with the other on the floor for
at least 0.5 s), the time on one wheel and the lift per request, the largest roll, and falls.
Then what touched the floor during the windows, from the simulation's contacts: the left tyre
alone (the stand), both tyres, nothing (a hop), or any other part of the robot (propped).
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
    policy, plant, profile, seed, hold = job
    rest = f"{hold + 2:g}"
    e.SCENARIOS["one_leg"] = ["2,0,0", "1,0,0,0,0,0.05", f"{rest},0,0", "1,0,0,0,0,0.05", f"{rest},0,0",
                               "2,0.5,0", "1,0.5,0,0,0,0.05", f"{rest},0.5,0", "2,0,0"]
    out = Path(f"/tmp/one_leg_runs/{Path(policy).name}_{plant}_{profile}_{seed}.csv")
    code, path = e.run(Path(policy), "one_leg", {**e.PROFILES[profile], **(FITTED if plant else {})}, out, seed=seed)
    rows = [r for r in csv.DictReader(open(path)) if r.get("diag_truth_wheel_lift_m") not in ("", None)]
    t, high, low, roll, request, touch = (np.array([float(r[k] or 0) for r in rows]) for k in (
        "t_s", "diag_truth_wheel_lift_m", "diag_truth_wheel_clearance_m", "diag_truth_roll_rad", "cmd_jump",
        "diag_truth_floor_contact"))
    dt = float(np.median(np.diff(t)))
    stands, touched = [], []
    for start in t[1:][(request[1:] > 0) & (request[:-1] <= 0)]:
        window = (t >= start) & (t < start + hold + 1.5)
        stands.append(((window & (high > 0.015) & (low < 0.005)).sum() * dt, high[window].max()))
        touched.extend(touch[(t >= start + 0.7) & (t < start + hold - 0.3)].astype(int).tolist())
    return code == 3, stands, float(np.abs(roll).max()), touched


def main():
    Path("/tmp/one_leg_runs").mkdir(exist_ok=True)
    args = sys.argv[1:]
    hold = float(args.pop(args.index("--hold") + 1)) if "--hold" in args else 2.0
    args = [a for a in args if a != "--hold"]
    for policy in args:
        jobs = [(policy, plant, profile, seed, hold) for plant in (0, 1) for profile in ("nominal", "spi_slow", "mit")
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
        touch = np.array([c for r in results for c in r[3]])
        share = lambda mask: 100.0 * float(np.mean(mask)) if len(touch) else float("nan")
        print(f"    floor contact in the windows: left tyre alone {share(touch == 2):.0f} %, both tyres "
              f"{share(touch == 3):.0f} %, right tyre alone {share(touch == 1):.0f} %, nothing "
              f"{share(touch == 0):.0f} %, another part {share(touch >= 4):.0f} %")


if __name__ == "__main__":
    main()
