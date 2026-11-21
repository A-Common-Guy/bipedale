"""How fast a policy drives with the wheel drives' velocity limiter in the loop (19.5 rad/s,
1.31 m/s at the tyre; evaluate.py's --sim wheel_speed_limit, docs/DESIGN.md 24). On the MJCF
and the fitted plant, three latency profiles, three seeds (18 runs): cruise at the speed asked,
brake to a stop, cruise and turn at 1.5 rad/s, then cruise through a shove from behind and one
from the front (12 N for 0.1 s).

    cd sandro_rl && uv run python scripts/fast_runs.py policies/fun_fast_v2
    ... --ask 1.5        the forward speed asked (default: the policy's own limit)
    ... --limit 0        without the limiter (default 19.5; the gain is 1 Nm per rad/s)
    ... --video          policies/X/X.mp4, from the side

Per policy: falls and where, the cruise speed against the one asked, the fastest wheel (the
drive's own speed where a gear is modelled) and the share of the run within 1 rad/s of the
limit, how far the axle travels after the stop is asked from the cruise and when the speed
is first under 0.05 m/s, the largest tilt, the speed and yaw rate
kept in the turn.
"""
import csv
import json
import sys
import warnings
from multiprocessing import Pool
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore")

FITTED = {"wheel_lag": "0", "wheel_backlash": "0.000001", "rotor_inertia": "22", "gear_stiffness": "19",
          "gear_damping": "0.1", "tyre_torsion": "0.01", "wheel_friction": "0", "leg_armature": "22"}
OUT = Path(__file__).resolve().parents[1] / "runs" / "fast_runs"
NAMES = ("standing", "cruise", "brake", "cruise", "turn", "stop", "shoves", "stop")
PUSHES = ["19,12,0.1", "20.5,-12,0.1"]


def scenario(v):
    # duration, forward, yaw
    return ["2,0,0", f"4,{v},0", "2.5,0,0", f"3,{v},0", f"3,{v},1.5", "2,0,0", f"6,{v},0", "2.5,0,0"]


def one(job):
    from sandro_rl import evaluate as e
    policy, plant, profile, seed, ask, limit = job
    e.SCENARIOS["fast"] = scenario(ask)
    e.closed_loop.SCENARIO_PUSHES["fast"] = PUSHES
    OUT.mkdir(parents=True, exist_ok=True)
    out = OUT / f"{Path(policy).name}_{ask}_{limit}_{plant}_{profile}_{seed}.csv"
    sim = {**e.PROFILES[profile], **(FITTED if plant else {}), "wheel_speed_limit": str(limit)}
    code, path = e.run(Path(policy), "fast", sim, out, seed=seed)
    rows = list(csv.DictReader(open(path)))
    diag = [r for r in rows if r.get("diag_truth_tilt_rad") not in ("", None)]
    col = lambda k: np.array([float(r[k] or 0) for r in diag])
    t, phase, speed, yaw, tilt = (col(k) for k in (
        "t_s", "phase", "diag_truth_forward_m_s", "diag_truth_yaw_rate_rad_s", "diag_truth_tilt_rad"))
    x, y = col("diag_truth_axle_x_m"), col("diag_truth_axle_y_m")
    joints = [r for r in rows if r["stream"] == "joint"]
    wheels = np.abs(np.array([[float(r["vel_right_wheel"] or 0), float(r["vel_left_wheel"] or 0)]
                              for r in joints])).max(axis=1)
    result = {"fell": code == 3, "where": NAMES[int(phase[-1])] if code == 3 else "", "tilt": float(tilt.max()),
              "wheel": float(wheels.max()), "near": float((wheels > limit - 1.0).mean()) if limit else 0.0}
    last = lambda p, s: (phase == p) & (t > t[phase == p][-1] - s) if (phase == p).any() else np.zeros(len(t), bool)
    if (phase == 1).any() and (phase > 1).any():
        result["cruise"] = float(speed[last(1, 1.5)].mean())
    if (phase == 2).any() and (phase > 2).any():
        m = phase == 2
        slow = np.abs(speed[m]) < 0.05
        stop = int(np.argmax(slow)) if slow.any() else m.sum() - 1
        result["stop_m"] = float(np.hypot(x[m] - x[m][0], y[m] - y[m][0]).max())    # the farthest it got
        result["stop_s"] = float(t[m][stop] - t[m][0])
    if (phase == 4).any() and (phase > 4).any():
        result["turn"] = (float(speed[last(4, 1.5)].mean()), float(yaw[last(4, 1.5)].mean()))
    if (phase == 6).any() and (phase > 6).any():
        result["shoved"] = float(speed[last(6, 1.0)].mean())
    return result


def span(values, digits=2):
    values = sorted(values)
    return (f"{values[0]:.{digits}f} .. {values[len(values) // 2]:.{digits}f} .. {values[-1]:.{digits}f}"
            if values else "none")


def video(policy, ask, limit):
    from sandro_rl import evaluate as e
    policy = Path(policy)
    e.SCENARIOS["fast_video"] = ["1.5,0,0", f"3,{ask},0", "2,0,0", f"2,-{min(ask, 0.6)},0", "1.5,0,0"]
    e.VIDEO_VIEW = (1.6, -10.0, 90.0)      # from the side
    e.run(policy, "fast_video", {**e.PROFILES["nominal"], **FITTED, "wheel_speed_limit": str(limit)},
          OUT / "video.csv", video=policy / f"{policy.name}.mp4")


def main():
    args = sys.argv[1:]
    value = lambda flag, default: float(args[args.index(flag) + 1]) if flag in args else default
    limit = value("--limit", 19.5)
    policies = [a for i, a in enumerate(args) if not a.startswith("--") and (i == 0 or args[i - 1] not in ("--ask", "--limit"))]
    for policy in policies:
        own = json.load(open(Path(policy) / "meta.json"))["config"]["env"]["commands"]["forward_m_s"]
        ask = value("--ask", float(own))
        if "--video" in args:
            video(policy, ask, limit)
            continue
        jobs = [(policy, plant, profile, seed, ask, limit) for plant in (0, 1)
                for profile in ("nominal", "spi_slow", "mit") for seed in (1, 2, 3)]
        with Pool(14) as pool:
            results = pool.map(one, jobs)
        every = lambda key: [r[key] for r in results if key in r]
        fell = [r for r in results if r["fell"]]
        where = ", ".join(f"{n} {sum(r['where'] == n for r in fell)}" for n in dict.fromkeys(r["where"] for r in fell))
        print(f"{policy}, asked {ask:g} m/s, limiter {f'{limit:g} rad/s' if limit else 'off'}: "
              f"falls {len(fell)} of {len(results)}{f' ({where})' if fell else ''}")
        print(f"    cruise {span(every('cruise'))} m/s; after the shoves {span(every('shoved'))} m/s; "
              f"in the turn {span([v for v, _ in every('turn')])} m/s at {span([w for _, w in every('turn')])} rad/s")
        print(f"    fastest wheel {max(every('wheel')):.1f} rad/s, within 1 rad/s of the limit for up to "
              f"{100 * max(every('near')):.0f} % of a run; tilt up to {max(every('tilt')):.2f} rad")
        print(f"    stop from the cruise in {span(every('stop_m'))} m, {span(every('stop_s'))} s")


if __name__ == "__main__":
    main()
