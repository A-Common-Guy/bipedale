"""agile_terrain as deployed (3 Hz differential wheel filter from the robot
overrides) against a candidate, on the standard scenarios, the fitted yaw
plant and the obstacle course.

    cd sandro_rl && uv run python ../recordings/sysid_2026-11-12/compare_agile.py policies/agile_terrain_v2
"""
import math
import sys
import warnings
from multiprocessing import Pool
from pathlib import Path

warnings.filterwarnings("ignore")

FITTED = {"wheel_lag": "0", "wheel_backlash": "0.000001", "rotor_inertia": "22", "gear_stiffness": "19",
          "gear_damping": "0.1", "tyre_torsion": "0.01", "wheel_friction": "0", "leg_armature": "22"}
CASES = [(scenario, profile, plant, course) for scenario, course in (("hardware", False), ("default", False),
                                                                    ("agile", False), ("push", False),
                                                                    ("shove", False), ("course", True))
         for profile in ("nominal", "spi_slow") for plant in ("mjcf", "fitted")]


def one(job):
    from sandro_rl import evaluate as e
    policy, deployed_filter, (scenario, profile, plant, course) = job
    sim = {**e.PROFILES[profile], **(FITTED if plant == "fitted" else {})}
    if deployed_filter:
        sim["differential_filter"] = str(deployed_filter)
    tag = f"{Path(policy).name}_{scenario}_{profile}_{plant}"
    code, path = e.run(Path(policy), scenario, sim, Path(f"/tmp/compare_agile/{tag}.csv"), course=course)
    phases = list(e.all_metrics(path, scenario).values())
    mean = lambda k: sum(p[k] for p in phases if not math.isnan(p[k])) / max(1, sum(not math.isnan(p[k]) for p in phases))
    return {"fell": code == 3, "pitch": mean("truth_pitch_rms"), "speed": mean("truth_speed_err_rms"),
            "yaw": mean("yaw_err_rms"), "tau": max(p["wheel_tau_rms"] for p in phases)}


def main():
    Path("/tmp/compare_agile").mkdir(exist_ok=True)
    runs = [("policies/agile_terrain", 3.0, "agile_terrain + 3 Hz filter")] + \
           [(p, 0.0, Path(p).name) for p in sys.argv[1:]]
    jobs = [(policy, filt, case) for policy, filt, _ in runs for case in CASES]
    with Pool(14) as pool:
        results = pool.map(one, jobs)
    table = {}
    for (policy, _, case), r in zip(jobs, results):
        table.setdefault(case, {})[policy] = r
    names = " / ".join(label for _, _, label in runs)
    print(f"{'case':34s} pitch rms | speed err | yaw err | worst wheel torque   ({names}); F = fell")
    for case, by in table.items():
        rs = [by[p] for p, _, _ in runs]
        cells = [" / ".join(f"{r[k]:.3f}" for r in rs) for k in ("pitch", "speed", "yaw")]
        cells.append(" / ".join(f"{r['tau']:.2f}" for r in rs))
        print(f"{' '.join(map(str, case[:3])):34s} " + " | ".join(cells) + "  " + "".join("F" if r["fell"] else "." for r in rs))


if __name__ == "__main__":
    main()
