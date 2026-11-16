"""Fit the policy evaluator's yaw plant to the robot's 2026-11-12 yaw chirp.

The robot's response (gyro yaw rate per Nm of differential wheel torque, kp
12, 0.2 Nm, sysid_20261112-231529) against the same chirp in
sandro_rl/evaluate.py (--scenario sysid --sim yaw_chirp=0.2) with a balancing
policy, over a grid of simulation options; both analysed with sysid_yaw.py's
response(). IMU and joint delays are set equal, as the robot's fit finds no
relative delay. Ranked by the coherence-weighted complex error, 3-18 Hz.

    cd sandro_rl && uv run python ../recordings/sysid_2026-11-12/sim_fit.py \
        policies/flat_level_drivetrain '{"rotor_inertia": [22], "gear_stiffness": [11, 19]}'
"""
import csv
import itertools
import json
import os
import sys
import warnings
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "ros2_ws/src/sandro_bringup/tools"))
warnings.filterwarnings("ignore")
import sysid_yaw as s  # noqa: E402

RUN, AMPLITUDE = "sysid_20261112-231529", 0.2
QS = (4, 8, 12, 16)


def robot_response():
    cache = Path("/tmp") / f"robot_frf_{RUN}_{AMPLITUDE}.npz"
    if cache.exists():
        z = np.load(cache)
        return z["f"], z["h"], z["coh"]
    plan = json.loads((HERE / RUN / "plan.json").read_text())
    joints, gyro, _ = s.read_bag(HERE / RUN / "bag")
    event = next(e for e in plan["events"] if e["kind"] == "chirp" and e["kp"] == 12.0
                 and (e.get("amplitude_nm") or 0.2) == AMPLITUDE)
    _, torque, yaw, _, _ = s.signals(joints, gyro, event["t"], event["t"] + event["duration_s"])
    f, h, coh = s.response(torque, yaw)
    np.savez(cache, f=f, h=h, coh=coh)
    return f, h, coh


def sim_response(path, t0=2.0, t1=22.0):
    rows = list(csv.DictReader(open(path)))
    joint = [r for r in rows if r["stream"] == "joint"]
    imu = [r for r in rows if r["stream"] == "imu"]
    tj = np.array([float(r["t_s"]) for r in joint])
    ti = np.array([float(r["t_s"]) for r in imu])
    if tj[-1] < t1:
        return None
    grid = np.arange(t0, t1, 1 / s.FS)
    col = lambda c: np.interp(grid, tj, np.array([float(r[c]) for r in joint]))
    torque = 0.5 * (col("eff_right_wheel") - col("eff_left_wheel"))
    yaw = np.interp(grid, ti, np.array([float(r["imu_gz"]) for r in imu]))
    return s.response(torque, yaw)


def one(job):
    from sandro_rl import evaluate as e
    policy, sim, (F, H, COH) = job
    tag = "_".join(f"{k}{v}" for k, v in sim.items())
    out = Path("/tmp/sim_fit") / f"{tag}.csv"
    out.parent.mkdir(exist_ok=True)
    code, path = e.run(Path(policy), "sysid",
                       {"yaw_chirp": str(AMPLITUDE), "imu_delay": "0.001", "wheel_lag": "0", **sim}, out)
    r = sim_response(path)
    if r is None:
        return sim, None
    f, h, coh = r
    band = (F >= 3.0) & (F <= 18.0)
    hs = np.interp(F[band], f, h.real) + 1j * np.interp(F[band], f, h.imag)
    err = float(np.sum(COH[band] * abs(hs - H[band]) ** 2) / np.sum(COH[band] * abs(H[band]) ** 2))
    at = lambda q: hs[np.argmin(abs(F[band] - q))]
    return sim, {"err": err, "mag": [abs(at(q)) for q in QS], "phase": [np.degrees(np.angle(at(q))) for q in QS],
                 "coh": float(np.median(coh[(f > 3) & (f < 18)]))}


def main():
    policy, space = sys.argv[1], json.loads(sys.argv[2])
    robot = robot_response()
    keys = list(space)
    jobs = []
    for combo in itertools.product(*(space[k] for k in keys)):
        sim = dict(zip(keys, map(str, combo)))
        sim["wheel_backlash"] = str(max(float(sim.pop("play", 0.0)), 1e-6)) if "gear_stiffness" in sim else "0"
        jobs.append((policy, sim, robot))
    with Pool(int(os.environ.get("WORKERS", 14))) as pool:
        results = pool.map(one, jobs)
    F, H, _ = robot
    at = lambda q: H[np.argmin(abs(F - q))]
    print(f"robot  |H| {'/'.join(map(str, QS))} Hz " + " ".join(f"{abs(at(q)):.2f}" for q in QS)
          + "  phase " + " ".join(f"{np.degrees(np.angle(at(q))):+.0f}" for q in QS))
    good = sorted([r for r in results if r[1]], key=lambda x: x[1]["err"])
    print(f"{len(results) - len(good)} of {len(results)} runs fell")
    for sim, r in good[:int(os.environ.get("TOP", 15))]:
        print(f"err {r['err']:.3f}  |H| " + " ".join(f"{m:.2f}" for m in r["mag"])
              + "  phase " + " ".join(f"{p:+.0f}" for p in r["phase"]) + f"  coh {r['coh']:.2f}  "
              + " ".join(f"{k}={v}" for k, v in sim.items()))


if __name__ == "__main__":
    main()
