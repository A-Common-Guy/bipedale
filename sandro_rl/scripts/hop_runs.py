"""How high a hop policy (configs/fun_hop_high.yaml) gets, and how it lands. On the MJCF and
the fitted plant, three latency profiles, three seeds (18 runs): a hop standing, another, one
at 0.3 m/s, then three in a row (a press every 1.4 s). The request is joy_teleop's (5 cm).

    cd sandro_rl && MUJOCO_GL=egl uv run python scripts/hop_runs.py policies/fun_hop_high
    ... --video        policies/X/X.mp4, from the side, with a slow-motion copy X_slow.mp4
    ... --flip         the flip policies: also the pitch turned through per hop, and upright after

Per policy: requests answered (both wheels over 1.5 cm up), the clearance of the lower wheel,
the time in the air, the speed the axles come down at, a second lift-off after the landing
(a bounce over 1 cm), what touches the floor besides the tyres, the knees' lowest point, the
fastest wheel in the hops (unloaded wheels spin up; the drives limit theirs to 19.5 rad/s)
and outside them, the largest leg torque and leg joint speed (the same limit), tilt, and falls.
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
OUT = Path(__file__).resolve().parents[1] / "runs" / "hop_runs"
ASK = "0.05"

# duration, forward, yaw, height rate, roll, hop
HOP = ["2.5,0,0", f"1,0,0,0,0,{ASK}", "2,0,0", f"1,0,0,0,0,{ASK}", "2,0,0",          # 0-4
       "1.5,0.3,0", f"1,0.3,0,0,0,{ASK}", "1.5,0.3,0", "1.5,0,0",                    # 5-8
       f"1,0,0,0,0,{ASK}", "0.4,0,0", f"1,0,0,0,0,{ASK}", "0.4,0,0", f"1,0,0,0,0,{ASK}", "3,0,0"]   # 9-14
HOPS = (1, 3, 6, 9, 11, 13)
VIDEO = ["1.5,0,0", f"1,0,0,0,0,{ASK}", "1.5,0,0", f"1,0,0,0,0,{ASK}", "0.4,0,0", f"1,0,0,0,0,{ASK}", "0.4,0,0",
         f"1,0,0,0,0,{ASK}", "2,0,0"]
SPAN_S = 1.4          # a hop is measured from its request for this long
# The flips (--flip): four standing, 3 s apart, and one at 0.3 m/s.
FLIP = ["2.5,0,0", f"1,0,0,0,0,{ASK}", "2,0,0", f"1,0,0,0,0,{ASK}", "2,0,0", f"1,0,0,0,0,{ASK}", "2,0,0",
        f"1,0,0,0,0,{ASK}", "2,0,0", "1.5,0.3,0", f"1,0.3,0,0,0,{ASK}", "1,0.3,0", "2,0,0"]
FLIPS = (1, 3, 5, 7, 10)
FLIP_SPAN_S = 1.8     # the window and its landing


def one(job):
    from sandro_rl import evaluate as e
    policy, plant, profile, seed, flip = job
    e.SCENARIOS["hop_runs"] = FLIP if flip else HOP
    hops, span_s = (FLIPS, FLIP_SPAN_S) if flip else (HOPS, SPAN_S)
    e.LINK_SHADOW = True
    OUT.mkdir(parents=True, exist_ok=True)
    out = OUT / f"{Path(policy).name}_{plant}_{profile}_{seed}.csv"
    code, path = e.run(Path(policy), "hop_runs", {**e.PROFILES[profile], **(FITTED if plant else {})}, out, seed=seed)
    rows = [r for r in csv.DictReader(open(path)) if r.get("diag_truth_wheel_lift_m") not in ("", None)]
    col = lambda k: np.array([float(r[k] or 0) for r in rows])
    t, phase, clear, tilt, contact, knee, turn, height = (col(k) for k in (
        "t_s", "phase", "diag_truth_wheel_clearance_m", "diag_truth_tilt_rad", "diag_truth_floor_contact",
        "diag_truth_knee_height_m", "diag_truth_pitch_turn_rad", "diag_truth_imu_height_m"))
    joints = [r for r in csv.DictReader(open(path)) if r["stream"] == "joint"]
    jcol = lambda k: np.array([float(r[k] or 0) for r in joints])
    tj = jcol("t_s")
    wheels = np.abs(np.stack([jcol("vel_right_wheel"), jcol("vel_left_wheel")])).max(axis=0)
    legs = np.abs(np.stack([jcol(f"eff_{s}_motor_{j}") for s in ("right", "left") for j in ("inner", "outer")])).max()
    leg_speed = np.abs(np.stack([jcol(f"vel_{s}_motor_{j}") for s in ("right", "left")
                                 for j in ("inner", "outer")])).max()
    result = {"fell": code == 3, "job": f"{plant} {profile} {seed}", "fell_in": int(phase[-1]) if code == 3 else None,
              "hops": [], "air": [], "impact": [], "bounce": [], "turn": [], "upright": [], "flip_other": [],
              "legs": float(legs), "leg_speed": float(leg_speed), "other": float(np.mean(contact.astype(int) & 4 > 0)),
              "knee": float(knee[tilt < 0.9].min()) if (tilt < 0.9).any() else 0.0}
    hopping = np.zeros(len(tj), bool)
    tilt_out = np.ones(len(t), bool)
    for p in hops:
        mask = phase == p
        if not mask.any():
            continue
        start = t[mask][0]
        window = (t >= start) & (t < start + span_s)
        hopping |= (tj >= start) & (tj < start + span_s)
        tilt_out &= ~window
        c, tw = clear[window], t[window]
        result["hops"].append(float(c.max()))
        up = c > 0.005
        if up.any():
            first = int(np.argmax(up))
            down = first + int(np.argmax(~up[first:])) if (~up[first:]).any() else len(c) - 1
            result["air"].append(float(tw[down] - tw[first]))
            # The lower wheel's speed over the last 20 ms before it touches.
            k = max(down - 2, first)
            result["impact"].append(float((c[k] - c[down]) / max(tw[down] - tw[k], 1e-3)))
            result["bounce"].append(float(c[down:].max()) if down < len(c) - 1 else 0.0)
        if flip:
            result["turn"].append(float(turn[window][-1] - turn[window][0]))
            after = (t >= start + span_s) & (t < start + span_s + 1.0)
            result["upright"].append(bool(after.any() and tilt[after].max() < 0.5))
            result["flip_other"].append(float(np.mean(contact[window].astype(int) & 4 > 0)))
    result["tilt"] = float(tilt[tilt_out].max()) if tilt_out.any() else 0.0
    result["wheel"] = float(wheels[~hopping].max())
    result["wheel_hop"] = float(wheels[hopping].max()) if hopping.any() else 0.0
    return result


def span(values, digits=2):
    values = sorted(values)
    return (f"{values[0]:.{digits}f} .. {values[len(values) // 2]:.{digits}f} .. {values[-1]:.{digits}f}"
            if values else "none")


def video(policy, flip=False):
    """policies/X/X.mp4, and X_slow.mp4: the first two hops rendered at 120 frames a second
    and played at 30."""
    import imageio.v2 as imageio
    from sandro_rl import evaluate as e
    e.VIDEO_VIEW = (1.3, -8.0, 90.0)       # from the side
    e.SCENARIOS["hop_video"] = FLIP[:3] if flip else VIDEO
    e.SCENARIOS["hop_slow"] = FLIP[:3] if flip else VIDEO[:4]
    policy = Path(policy)
    e.run(policy, "hop_video", {**e.PROFILES["nominal"], **FITTED}, OUT / "video.csv",
          video=policy / f"{policy.name}.mp4")
    slow = policy / f"{policy.name}_slow.mp4"
    e.run(policy, "hop_slow", {**e.PROFILES["nominal"], **FITTED}, OUT / "video_slow.csv", video=slow, video_fps=120)
    frames = imageio.mimread(slow, memtest=False)
    imageio.mimsave(slow, frames[120:], fps=30, codec="libx264", quality=7)


def main():
    args = sys.argv[1:]
    flags = {a for a in args if a.startswith("--")}
    policies = [a for a in args if not a.startswith("--")]
    if "--video" in flags:
        for policy in policies:
            video(policy, "--flip" in flags)
        return
    for policy in policies:
        jobs = [(policy, plant, profile, seed, "--flip" in flags) for plant in (0, 1)
                for profile in ("nominal", "spi_slow", "mit") for seed in (1, 2, 3)]
        with Pool(14, maxtasksperchild=1) as pool:
            results = pool.map(one, jobs, chunksize=1)
        every = lambda key: [v for r in results for v in r[key]]
        hops = every("hops")
        print(f"{policy}: falls {sum(r['fell'] for r in results)} of {len(results)}"
              + "".join(f"\n    fell: {r['job']} in phase {r['fell_in']}" for r in results if r["fell"]))
        print(f"    hops answered {sum(h > 0.015 for h in hops)} of {len(hops)}, clearance {span(hops, 3)} m "
              f"(MJCF {span([v for r in results if r['job'][0] == '0' for v in r['hops']], 3)}, "
              f"fitted {span([v for r in results if r['job'][0] == '1' for v in r['hops']], 3)})")
        print(f"    in the air {span(every('air'))} s; touchdown at {span(every('impact'))} m/s; "
              f"bounce after it up to {max(every('bounce') or [0]):.3f} m")
        if "--flip" in flags:
            turns = every("turn")
            print(f"    pitch turned per request {span(turns)} rad (nose up is negative); whole turns "
                  f"{sum(abs(v) > 5.5 for v in turns)} of {len(turns)}; upright for the second after "
                  f"{sum(every('upright'))} of {len(every('upright'))}; another part on the floor in "
                  f"{sum(v > 0 for v in every('flip_other'))} of them")
        print(f"    another part on the floor {100 * np.mean([r['other'] for r in results]):.1f} % of the time; "
              f"knees at least {min(r['knee'] for r in results):.3f} m up")
        print(f"    fastest wheel {max(r['wheel'] for r in results):.1f} rad/s outside the hops, "
              f"{max(r['wheel_hop'] for r in results):.1f} in them; leg torque up to "
              f"{max(r['legs'] for r in results):.1f} Nm at up to {max(r['leg_speed'] for r in results):.1f} rad/s, tilt outside the hops up to "
              f"{max(r['tilt'] for r in results):.2f} rad")


if __name__ == "__main__":
    main()
