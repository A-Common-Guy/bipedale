"""How often policies cross the planks-and-hill course at 0.5 m/s: three plants (the MJCF,
the fitted yaw plant, the same with leg rotors at 35 x) x three latency profiles x three seeds.
POLICY:HZ adds the differential wheel filter of the robot overrides, POLICY:KEY=VALUE any
evaluator option (camera=0: a perceptive policy with its terrain profile unknown).
--steps first: the planks of 3, 4, 5 and 6 cm and the 6 cm curb instead (evaluate.py COURSES).

    cd sandro_rl && uv run python ../recordings/sysid_2026-11-12/course_runs.py policies/agile_terrain:3 policies/agile_terrain_v3
"""
import sys
import warnings
from multiprocessing import Pool
from pathlib import Path

warnings.filterwarnings("ignore")
sys.path.insert(0, str(Path(__file__).parent))
from compare_agile import FITTED  # noqa: E402

PLANTS = {"mjcf": {}, "fitted": FITTED, "heavy legs": {**FITTED, "leg_armature": "35"}}
PROFILES = ("nominal", "spi_slow", "mit")
SEEDS = (1, 2, 3)
STEPS = "--steps" in sys.argv


def one(job):
    from sandro_rl import evaluate as e
    spec, plant, profile, seed = job
    policy, _, filt = spec.partition(":")
    extra = dict([filt.split("=")]) if "=" in filt else ({"differential_filter": filt} if filt else {})
    sim = {**e.PROFILES[profile], **PLANTS[plant], **extra}
    out = Path(f"/tmp/course_runs/{Path(policy).name}{filt}_{plant.replace(' ', '')}_{profile}_{seed}.csv")
    code, _ = e.run(Path(policy), *(("steps",) if STEPS else ("course_slow",)), sim, out, seed=seed,
                    course="steps" if STEPS else True)
    return code != 3


def main():
    Path("/tmp/course_runs").mkdir(exist_ok=True)
    if STEPS:
        sys.argv.remove("--steps")
    jobs = [(spec, plant, profile, seed) for spec in sys.argv[1:] for plant in PLANTS
            for profile in PROFILES for seed in SEEDS]
    with Pool(14) as pool:
        crossed = dict(zip(jobs, pool.map(one, jobs)))
    for spec in sys.argv[1:]:
        cells = [f"{plant} {sum(crossed[spec, plant, p, s] for p in PROFILES for s in SEEDS)}/9" for plant in PLANTS]
        print(f"{spec:36s} {sum(v for k, v in crossed.items() if k[0] == spec):2d} of 27   " + ", ".join(cells))


if __name__ == "__main__":
    main()
