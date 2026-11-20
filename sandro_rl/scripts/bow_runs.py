"""What a bow policy (configs/fun_bow.yaml) does with the roll stick: leans held forward and
back, nods (the stick swept as a sinusoid) standing, a bow while driving and while turning. On
the MJCF and the fitted plant, three latency profiles, three seeds (18 runs). The leg links do
not collide in these runs or in training; a copy of the model with the links as boxes follows
each pose (evaluate.LinkShadow) and says whether a leg touches the torso or the other leg, and
how low the knees come.

    cd sandro_rl && MUJOCO_GL=egl uv run python scripts/bow_runs.py policies/fun_bow
    ... --video    the policy folder's mp4, from the side

Per policy: the pitch held against the one asked for (the channel times lean.gain), how far the
axle creeps while it holds, the amplitude of a nod against the reference's and how far the axle
wanders in it, the roll and yaw rate that come with them, the speed kept in a bow at 0.4 m/s
and the yaw rate in a bow at 1 rad/s, how still it stands upright again, what touches the floor
besides the tyres, the knees' lowest point and whether the robot touches itself, the fastest
wheel (the drives limit theirs to 19.5 rad/s), what RlPolicyController's oscillation watchdog
would read, the largest leg torque and tilt, and falls.
"""
import csv
import sys
import warnings
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from stride_runs import FITTED, scenario, span, sweep, watchdog  # noqa: E402

warnings.filterwarnings("ignore")

OUT = Path(__file__).resolve().parents[1] / "runs" / "bow_runs"
FULL = 0.25      # the channel's range (commands.roll_rad)

# The height goes from the start's 0.15 m to 0.13 m first: there the legs reach the whole lean.
SEGMENTS = [("settle", ["2,0,0", "1,0,0,-0.02", "1,0,0"], None)]
SEGMENTS += [("hold", [f"3,0,0,0,{s}"], s) for s in (FULL, 0.5 * FULL, -0.5 * FULL, -FULL, 0.0)]
SEGMENTS += [("nod", sweep(6, 0, 0, a, hz), (a, hz)) for a, hz in ((0.15, 0.5), (0.08, 1.0))]
SEGMENTS += [("rest", ["1.5,0,0"], None)]
SEGMENTS += [("drive", ["1.5,0.4,0", f"4,0.4,0,0,{s}"], s) for s in (FULL, -FULL)]
SEGMENTS += [("turn", [f"4,0.2,1.0,0,{FULL}"], FULL), ("rest", ["2,0,0"], None)]
# The stick thrown from end to end every 0.6 s: the fastest nod the rate limit lets through.
SEGMENTS += [("flick", [f"0.6,0,0,0,{s}" for s in (FULL, -FULL) * 3], None), ("rest", ["2.5,0,0"], None)]
PHASES, INDEX = scenario(SEGMENTS)

VIDEO, _ = scenario([("", ["1.5,0,0", "1,0,0,-0.02", f"3,0,0,0,{FULL}", "1.5,0,0", f"2.5,0,0,0,{-FULL}", "1.2,0,0"], None),
                     ("", sweep(3, 0, 0, 0.12, 1.0), None), ("", ["0.8,0,0", "1,0.4,0", f"3,0.4,0,0,{FULL}"], None),
                     ("", [f"2.5,0.2,1.2,0,{FULL}", "2,0,0"], None)])


def one(job):
    from sandro_rl import evaluate as e
    policy, plant, profile, seed, gain = job
    e.SCENARIOS["bow"] = PHASES
    e.LINK_SHADOW = True
    OUT.mkdir(parents=True, exist_ok=True)
    out = OUT / f"{Path(policy).name}_{plant}_{profile}_{seed}.csv"
    code, path = e.run(Path(policy), "bow", {**e.PROFILES[profile], **(FITTED if plant else {})}, out, seed=seed)
    rows = [r for r in csv.DictReader(open(path)) if r.get("diag_truth_pitch_rad") not in ("", None)]
    col = lambda k: np.array([float(r[k] or 0) for r in rows])
    t, phase, ref, pitch, tilt, forward, contact, touching, link, roll, yaw, x, y, height = (col(k) for k in (
        "t_s", "phase", "diag_roll_reference_rad", "diag_truth_pitch_rad", "diag_truth_tilt_rad",
        "diag_truth_forward_m_s", "diag_truth_floor_contact", "diag_truth_self_contact", "diag_truth_knee_height_m",
        "diag_truth_roll_rad", "diag_truth_yaw_rate_rad_s", "diag_truth_axle_x_m", "diag_truth_axle_y_m",
        "diag_truth_height_m"))
    ref = gain * ref
    joints = [r for r in csv.DictReader(open(path)) if r["stream"] == "joint"]
    jcol = lambda k: np.array([float(r[k] or 0) for r in joints])
    wheels = np.abs(np.stack([jcol("vel_right_wheel"), jcol("vel_left_wheel")])).max()
    legs = np.abs(np.stack([jcol(f"eff_{s}_motor_{j}") for s in ("right", "left") for j in ("inner", "outer")])).max()
    tj = jcol("t_s")
    watch = watchdog(tj, np.stack([jcol(f"vel_{n}") for n in (
        "right_motor_inner", "right_motor_outer", "right_wheel", "left_motor_inner", "left_motor_outer",
        "left_wheel")], axis=1))[tj > 3.0]
    if not len(watch):
        watch = np.zeros((1, 2))
    over = watch[:, 0] > 1.0
    edges = np.flatnonzero(np.diff(np.concatenate([[0], over.astype(int), [0]])))
    result = {"fell": code == 3, "end": float(t[-1]), "job": f"{plant} {profile} {seed}", "hold": [], "creep": [], "nod": [], "wander": [], "roll": [], "yaw": [], "still": [],
              "drive": [], "turn": [], "tilt": float(tilt.max()), "legs": float(legs), "wheel": float(wheels),
              "watch_leg": float(watch[:, 0].max()), "watch_wheel": float(watch[:, 1].max()),
              "watch_over": float((edges[1::2] - edges[::2]).max() * np.median(np.diff(tj))) if over.any() else 0.0,
              "other": float(np.mean(contact.astype(int) & 4 > 0)), "self": float(np.mean(touching > 0)),
              "link": float(link[tilt < 0.9].min()), "height": float(height[phase >= INDEX[1][1]].mean())
              if (phase >= INDEX[1][1]).any() else 0.0}
    if result["fell"]:
        at = int(phase[-1])
        result["fell_in"] = next((label for label, first, last, _ in INDEX if first <= at <= last), 'after the end')
    for label, first, last, params in INDEX:
        mask = (phase >= first) & (phase <= last)
        if not mask.any() or t[mask][-1] - t[mask][0] < 1.5:
            continue
        tp = t[mask] - t[mask][0]
        late = tp > tp[-1] - 0.8
        if label == "hold":
            result["hold"].append((params, float(pitch[mask][late].mean()), float(ref[mask][late].mean())))
            result["creep"].append(float(np.abs(forward[mask][tp > tp[-1] - 1.5]).mean()))
            result["roll"].append(float(np.abs(roll[mask]).max()))
            result["yaw"].append(float(np.abs(yaw[mask]).max()))
            if params == 0.0:
                result["still"].append(float(np.abs(forward[mask][tp > 1.5]).max()))
        if label == "nod" and tp[-1] > 4.5:
            whole = tp > tp[-1] - 4.0
            reached = 0.5 * (pitch[mask][whole].max() - pitch[mask][whole].min())
            asked = 0.5 * (ref[mask][whole].max() - ref[mask][whole].min())
            result["nod"].append((params[1], float(reached / max(asked, 1e-6)), float(reached)))
            result["wander"].append(float(np.hypot(x[mask] - x[mask][0], y[mask] - y[mask][0]).max()))
            result["roll"].append(float(np.abs(roll[mask]).max()))
            result["yaw"].append(float(np.abs(yaw[mask]).max()))
        if label == "drive" and (phase[mask] == last).any():
            part = phase[mask] == last
            tl = t[mask][part] - t[mask][part][0]
            if not (tl > 2.5).any():
                continue
            result["drive"].append((params, float(pitch[mask][part][tl > 2.5].mean()),
                                    float(forward[mask][part][tl > 2.0].mean())))
        if label == "turn" and (tp > 2.0).any():
            result["turn"].append((float(pitch[mask][late].mean()), float(yaw[mask][tp > 2.0].mean())))
        if label == "flick" and (tp > 1.2).any():
            result["flick"] = float(0.5 * (pitch[mask][tp > 1.2].max() - pitch[mask][tp > 1.2].min()))
    return result


def video(policy):
    from sandro_rl import evaluate as e
    e.SCENARIOS["bow_video"] = VIDEO
    policy = Path(policy)
    e.VIDEO_VIEW = (1.1, -8.0, 90.0)
    e.run(policy, "bow_video", {**e.PROFILES["nominal"], **FITTED}, OUT / "video.csv", video=policy / f"{policy.name}.mp4")


def main():
    from sandro_rl import checkpoint
    args = sys.argv[1:]
    if "--video" in args:
        for policy in (a for a in args if a != "--video"):
            video(policy)
        return
    for policy in args:
        gain = checkpoint.load(Path(policy))[1]["config"]["env"]["commands"]["lean"]["gain"]
        jobs = [(policy, plant, profile, seed, gain) for plant in (0, 1)
                for profile in ("nominal", "spi_slow", "mit") for seed in (1, 2, 3)]
        with Pool(14, maxtasksperchild=1) as pool:
            results = pool.map(one, jobs, chunksize=1)
        every = lambda key: [v for r in results for v in r[key]]
        print(f"{policy}: falls {sum(r['fell'] for r in results)} of {len(results)}, height "
              f"{np.mean([r['height'] for r in results]):.3f} m")
        for r in results:
            if r["fell"]:
                print(f"    fell: plant {r['job']} at {r['end']:.1f} s, in '{r['fell_in']}'")
        for asked in (FULL, 0.5 * FULL, -0.5 * FULL, -FULL, 0.0):
            got = [h[1] for h in every("hold") if h[0] == asked]
            print(f"    lean asked {gain * asked:+.2f} rad: held {span(got)} rad")
        for hz in (0.5, 1.0):
            part = [v for v in every("nod") if v[0] == hz]
            if part:
                print(f"    nod at {hz} Hz: amplitude / the reference's {span([v[1] for v in part])}, "
                      f"+-{span([v[2] for v in part])} rad")
        print(f"    holding a lean: the axle creeps at {span(every('creep'))} m/s; nodding: it wanders "
              f"{span(every('wander'))} m; upright again, speed up to {span(every('still'))} m/s")
        for asked in (FULL, -FULL):
            part = [v for v in every("drive") if v[0] == asked]
            print(f"    at 0.4 m/s with {gain * asked:+.2f} rad asked: pitch {span([v[1] for v in part])} rad, "
                  f"speed {span([v[2] for v in part])} m/s")
        print(f"    turning at 1 rad/s in a full bow: pitch {span([v[0] for v in every('turn')])} rad, yaw rate "
              f"{span([v[1] for v in every('turn')])} rad/s")
        print(f"    standing leans and nods: roll up to {max(every('roll')):.2f} rad, yaw rate up to "
              f"{span(every('yaw'))} rad/s")
        print(f"    another part on the floor {100 * np.mean([r['other'] for r in results]):.1f} % of the time; "
              f"knees at least {min(r['link'] for r in results):.3f} m above the floor, a leg touching the "
              f"torso or the other leg {100 * np.mean([r['self'] for r in results]):.1f} % of the time")
        print(f"    the stick thrown end to end every 0.6 s: +-{span([r.get('flick', 0.0) for r in results])} rad")
        print(f"    the controller's oscillation watchdog would read up to {max(r['watch_leg'] for r in results):.2f} rad/s "
              f"at the legs (limit 1.0) and {max(r['watch_wheel'] for r in results):.2f} at the wheels (5.0); legs "
              f"beyond the limit for at most {max(r['watch_over'] for r in results):.2f} s at a time")
        print(f"    fastest wheel {max(r['wheel'] for r in results):.1f} rad/s, leg torque up to "
              f"{max(r['legs'] for r in results):.1f} Nm, tilt up to {max(r['tilt'] for r in results):.2f} rad")


if __name__ == "__main__":
    main()
