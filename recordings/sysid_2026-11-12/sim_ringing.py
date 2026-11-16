"""Do the policies ring in simulation as on the robot? 5-15 Hz yaw rate, its
peak and the opposed wheel motion while standing, per policy and simulation.

On the robot (2026-11-09 and 12): rl_flat_level_fast 0.88 rad/s (watchdog),
rl_flat_level 0.37, rl_agile_terrain 0.10, the drivetrain policies up to
0.95, all at 8.8-9.4 Hz; the WBC 0.03-0.04.

    cd sandro_rl && uv run python ../recordings/sysid_2026-11-12/sim_ringing.py \
        policies/flat_level_real,policies/agile_terrain 'rotor_inertia=22,gear_stiffness=19,...'
"""
import csv
import sys
import warnings
from multiprocessing import Pool
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore")
FS, R, TRACK = 300.0, 0.0672, 0.304


def band(x, lo, hi):
    f = np.fft.rfftfreq(len(x), 1 / FS)
    X = np.fft.rfft(x - x.mean())
    X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, len(x))


def peak(x, lo=3.0, hi=30.0):
    w = np.hanning(len(x))
    X = np.abs(np.fft.rfft((x - x.mean()) * w))
    f = np.fft.rfftfreq(len(x), 1 / FS)
    b = (f >= lo) & (f <= hi)
    return f[b][np.argmax(X[b])]


def metrics(path, t_from=2.0):
    rows = list(csv.DictReader(open(path)))
    joint = [r for r in rows if r["stream"] == "joint"]
    imu = [r for r in rows if r["stream"] == "imu"]
    tj = np.array([float(r["t_s"]) for r in joint])
    ti = np.array([float(r["t_s"]) for r in imu])
    if tj[-1] - t_from < 2.0:
        return None
    grid = np.arange(t_from, tj[-1], 1 / FS)
    col = lambda c: np.interp(grid, tj, np.array([float(r[c]) for r in joint]))
    gz = np.interp(grid, ti, np.array([float(r["imu_gz"]) for r in imu]))
    opposed = 0.5 * (col("vel_right_wheel") - col("vel_left_wheel"))
    return {"yaw": band(gz, 5, 15).std(), "peak": peak(gz), "opposed": band(opposed, 5, 15).std(),
            "seconds": tj[-1]}


def one(job):
    from sandro_rl import evaluate as e
    policy, sim, scenario = job
    tag = "_".join(f"{k}{v}" for k, v in sim.items())
    out = Path("/tmp/sim_ringing") / f"{Path(policy).name}_{scenario}_{tag}.csv"
    out.parent.mkdir(exist_ok=True)
    code, path = e.run(Path(policy), scenario, sim, out)
    return job, code, metrics(path)


def main():
    policies = sys.argv[1].split(",")
    sims = [dict(kv.split("=") for kv in arg.split(",")) if arg != "default" else {} for arg in sys.argv[2:]]
    scenario = "stand"
    jobs = [(p, sim, scenario) for sim in sims for p in policies]
    with Pool(14) as pool:
        results = pool.map(one, jobs)
    for (policy, sim, _), code, m in results:
        label = ",".join(f"{k}={v}" for k, v in sim.items()) or "default"
        if m is None:
            print(f"{Path(policy).name:28s} FELL at once  {label}")
            continue
        fell = " FELL" if code == 3 else ""
        print(f"{Path(policy).name:28s} yaw 5-15 Hz {m['yaw']:.3f} rad/s at {m['peak']:4.1f} Hz, opposed wheels "
              f"{m['opposed']:.2f} rad/s, {m['seconds']:.0f} s{fell}  {label}")


if __name__ == "__main__":
    main()
