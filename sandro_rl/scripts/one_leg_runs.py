"""How long a one-leg policy (commands.jump.one_leg) stands on one wheel per request:
two requests standing and one at 0.5 m/s, on the MJCF and the fitted plant, three latency
profiles, three seeds. --hold S: the policy's window (commands.jump.window_s, 2 s).
--boxes: the leg links' boxes collide with the floor, each other and the torso whatever the
policy trained with. --floor-boxes: with the floor only (the boxes of one leg's two upper
links collide where the real links pass each other).

    cd sandro_rl && uv run python scripts/one_leg_runs.py policies/fun_one_leg
    cd sandro_rl && uv run python scripts/one_leg_runs.py --hold 8 policies/fun_one_leg_long

Per policy: requests answered (one wheel more than 1.5 cm up with the other on the floor for
at least 0.5 s), the time on one wheel and the lift per request, the largest roll, and falls.
Then what touched the floor during the windows, from the simulation's contacts: the left tyre
alone (the stand), both tyres, nothing (a hop), or any other part of the robot (propped).
And the longest stand without a break (the shares hide a rocking from tap to tap), and how
close the leg links' CAD meshes come to the floor (evaluate.LinkFloor): the description
collides none of them, and their boxes (--boxes) reach 18 mm beyond them at the wheels.
SANDRO_OUTLINE=1 in the environment: the hulls of the CAD meshes meet the floor instead (any
run script; evaluate.run, docs/DESIGN.md 31).
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
    policy, plant, profile, seed, hold, boxes = job
    e.LEG_BOXES = boxes or None
    e.FLOOR_PARTS = []
    e.LINK_FLOOR = True
    rest = f"{hold + 2:g}"
    e.SCENARIOS["one_leg"] = ["2,0,0", "1,0,0,0,0,0.05", f"{rest},0,0", "1,0,0,0,0,0.05", f"{rest},0,0",
                               "2,0.5,0", "1,0.5,0,0,0,0.05", f"{rest},0.5,0", "2,0,0"]
    out = Path(f"/tmp/one_leg_runs/{Path(policy).name}_{plant}_{profile}_{seed}.csv")
    code, path = e.run(Path(policy), "one_leg", {**e.PROFILES[profile], **(FITTED if plant else {})}, out, seed=seed)
    rows = [r for r in csv.DictReader(open(path)) if r.get("diag_truth_wheel_lift_m") not in ("", None)]
    t, high, low, roll, request, touch, link = (np.array([float(r[k] or 0) for r in rows]) for k in (
        "t_s", "diag_truth_wheel_lift_m", "diag_truth_wheel_clearance_m", "diag_truth_roll_rad", "cmd_jump",
        "diag_truth_floor_contact", "diag_truth_link_floor_m"))
    dt = float(np.median(np.diff(t)))
    stands, touched, unbroken, links, rolls = [], [], [], [], []
    for start in t[1:][(request[1:] > 0) & (request[:-1] <= 0)]:
        window = (t >= start) & (t < start + hold + 1.5)
        stands.append(((window & (high > 0.015) & (low < 0.005)).sum() * dt, high[window].max()))
        # The stand without a break: the raised wheel 2 cm clear, the other on the floor.
        alone = (high > 0.02) & (low < 0.005)
        held = (t >= start + 0.7) & (t < start + hold - 0.3)
        run = best = breaks = 0
        for a, b in zip(alone[held][:-1], alone[held][1:]):
            run = run + 1 if b else 0
            best = max(best, run)
            breaks += int(a and not b)
        unbroken.append((best * dt, breaks))
        links.extend(link[held].tolist())
        rolls.extend(roll[held].tolist())
        touched.extend(touch[(t >= start + 0.7) & (t < start + hold - 0.3)].astype(int).tolist())
    return code == 3, stands, float(np.abs(roll).max()), touched, sorted(set(e.FLOOR_PARTS)), unbroken, links, rolls


def main():
    Path("/tmp/one_leg_runs").mkdir(exist_ok=True)
    args = sys.argv[1:]
    hold = float(args.pop(args.index("--hold") + 1)) if "--hold" in args else 2.0
    boxes = True if "--boxes" in args else "floor" if "--floor-boxes" in args else None
    args = [a for a in args if a not in ("--hold", "--boxes", "--floor-boxes")]
    for policy in args:
        jobs = [(policy, plant, profile, seed, hold, boxes) for plant in (0, 1) for profile in ("nominal", "spi_slow", "mit")
                for seed in (1, 2, 3)]
        with Pool(14, maxtasksperchild=1) as pool:
            results = pool.map(one, jobs, chunksize=1)
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
        runs = sorted(u[0] for r in results for u in r[5])
        breaks = sorted(u[1] for r in results for u in r[5])
        print(f"    longest stand without a break per request {runs[0]:.2f} .. {runs[len(runs) // 2]:.2f} .. "
              f"{runs[-1]:.2f} s; breaks (the raised wheel within 2 cm of the floor, or a hop) per request "
              f"{breaks[0]} .. {breaks[len(breaks) // 2]} .. {breaks[-1]}")
        link = np.array([x for r in results for x in r[6]])
        rolls = [x for r in results for x in r[7]]
        if len(link):
            print(f"    the leg links' meshes above the floor in the holds: {1e3 * link.min():.0f} mm at least, "
                  f"{1e3 * np.percentile(link, 1):.0f} mm for 99 % of the time; under 3 mm "
                  f"{100.0 * float(np.mean(link < 0.003)):.1f} %; roll {np.median(np.abs([abs(x) for x in rolls])):.2f} rad (median)")
        parts = sorted({p for r in results for p in r[4]})
        if parts:
            print(f"    other parts on the floor at any time: {', '.join(parts)}")


if __name__ == "__main__":
    main()
