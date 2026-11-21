"""The Drive page's routines (ros2_ws/src/sandro_gui/config/routines.yaml) replayed with the
policy each is meant for, as joy_teleop would turn the panel's sticks into commands
(evaluate.py `--scenario routine:NAME:SPEED`), under the drives' modelled speed limit
(19.5 rad/s), on the MJCF and the fitted plant, at 50 % and 100 % of the page's speed.

    cd sandro_rl && uv run python scripts/routine_runs.py            # the table
    MUJOCO_GL=egl uv run python scripts/routine_runs.py --video      # one mp4 per routine, in its policy's folder

Per run: fell or not, the fastest wheel on the floor (while both tyres touch), the floor the
axle's path covered (along the start heading x across it), where the routine ended relative
to its start, and the height reference's error at the end against its start.
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
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "runs" / "routine_runs"
POLICY = {"greeting": "fun_show", "spin": "fun_show", "sway": "fun_show", "eight": "fun_show", "show": "fun_show",
          "stride": "fun_stride", "bow": "fun_bow", "hops": "fun_hop_high"}
# distance, elevation, azimuth of the tracking camera per routine
VIEW = {"stride": (1.3, -8.0, 90.0), "bow": (1.1, -8.0, 90.0), "hops": (1.3, -8.0, 90.0),
        "eight": (2.2, -35.0, 120.0), "show": (1.5, -15.0, 120.0)}


def one(job):
    from sandro_rl import evaluate as e
    routine, plant, speed = job
    OUT.mkdir(parents=True, exist_ok=True)
    out = OUT / f"{routine}_{'fitted' if plant else 'mjcf'}_{int(speed * 100)}.csv"
    sim = {**e.PROFILES["nominal"], "wheel_speed_limit": "19.5", **(FITTED if plant else {})}
    code, path = e.run(ROOT / "policies" / POLICY[routine], f"routine:{routine}:{speed}", sim, out, seed=1)
    rows = [r for r in csv.DictReader(open(path)) if r.get("diag_truth_wheel_lift_m") not in ("", None)]
    col = lambda k: np.array([float(r[k] or 0) for r in rows])
    x, y, height = col("diag_truth_axle_x_m"), col("diag_truth_axle_y_m"), col("diag_truth_height_m")
    phase, contact = col("phase"), col("diag_truth_floor_contact")
    joints = [r for r in csv.DictReader(open(path)) if r["stream"] == "joint"]
    wheels = np.abs(np.array([[float(r["vel_right_wheel"] or 0), float(r["vel_left_wheel"] or 0)]
                              for r in joints])).max(axis=1)
    jt = np.array([float(r["t_s"]) for r in joints])
    t = col("t_s")
    grounded = np.interp(jt, t, (contact == 3).astype(float)) > 0.99      # both tyres down, nothing else
    before, after = phase == 0, phase == 2
    result = {"routine": routine, "plant": "fitted" if plant else "mjcf", "speed": speed, "fell": code == 3,
              "wheel": float(wheels.max()),
              "wheel_floor": float(wheels[grounded].max()) if grounded.any() else float("nan"),
              "other": bool((contact.astype(int) & 4).any()),
              "box": (float(x.max() - x.min()), float(y.max() - y.min())),
              "end": (float(x[-1] - x[0]), float(y[-1] - y[0])),
              "height": float(height[after][-20:].mean() - height[before][-20:].mean()) if after.any() else float("nan"),
              "tilt": float(col("diag_truth_tilt_rad").max())}
    return result


def video(routine):
    from sandro_rl import evaluate as e
    e.VIDEO_VIEW = VIEW.get(routine, (1.3, -15.0, 120.0))
    target = ROOT / "policies" / POLICY[routine] / f"routine_{routine}.mp4"
    sim = {**e.PROFILES["nominal"], "wheel_speed_limit": "19.5", **FITTED}
    code, _ = e.run(ROOT / "policies" / POLICY[routine], f"routine:{routine}:0.5", sim,
                    OUT / f"video_{routine}.csv", video=target, seed=1)
    return routine, code, target


def main():
    names = [a for a in sys.argv[1:] if not a.startswith("--")] or list(POLICY)
    OUT.mkdir(parents=True, exist_ok=True)
    if "--video" in sys.argv:
        with Pool(4, maxtasksperchild=1) as pool:
            for routine, code, target in pool.imap(video, names, chunksize=1):
                print(f"{routine}: {'FELL' if code == 3 else 'ok'} {target}")
        return
    jobs = [(r, plant, speed) for r in names for plant in (0, 1) for speed in (0.5, 1.0)]
    with Pool(14, maxtasksperchild=1) as pool:
        results = pool.map(one, jobs, chunksize=1)
    print(f"{'routine':9s} {'policy':13s} {'plant':6s} {'speed':>5s} {'fell':>5s} {'wheel rad/s':>12s} {'on floor':>9s} "
          f"{'floor used, m':>14s} {'ends at, m':>14s} {'height, mm':>10s} {'tilt':>5s} {'other part on the floor'}")
    for r in results:
        print(f"{r['routine']:9s} {POLICY[r['routine']]:13s} {r['plant']:6s} {r['speed']:5.1f} {'FELL' if r['fell'] else 'no':>5s} "
              f"{r['wheel']:12.1f} {r['wheel_floor']:9.1f} {r['box'][0]:6.2f} x {r['box'][1]:4.2f} "
              f"{r['end'][0]:+6.2f} {r['end'][1]:+6.2f} {1000 * r['height']:+10.1f} {r['tilt']:5.2f} {'YES' if r['other'] else 'no'}")


if __name__ == "__main__":
    main()
