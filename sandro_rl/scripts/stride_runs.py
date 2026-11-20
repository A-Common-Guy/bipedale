"""What a stride policy (configs/fun_stride.yaml) does with the roll stick: splits held, and
strides (the stick swept as a sinusoid) standing, driving and turning. On the MJCF and the
fitted plant, three latency profiles, three seeds (18 runs). The leg links do not collide in
these runs or in training; a copy of the model with the links as boxes follows each pose
(evaluate.LinkShadow) and says whether a leg touches the torso or the other leg, and how low the
knees come.

    cd sandro_rl && MUJOCO_GL=egl uv run python scripts/stride_runs.py policies/fun_stride
    ... --video    the policy folder's mp4s, from the side and from the front quarter

Per policy: the split held against the one asked for (what the legs reach at that height), the
amplitude of the stride against the reference's, the roll and the yaw rate that come with
them (the body should stay level and keep its heading), the pitch swing and how far the axle wanders
in a stride on the spot, how still it stands with the legs back together, the speed kept in a stride at 0.4 m/s, what touches the floor besides
the tyres, the knees' lowest point and whether the robot touches itself, the fastest wheel (the drives limit theirs to
19.5 rad/s), what RlPolicyController's oscillation watchdog would read, the largest leg torque
and tilt, and falls.
"""
import csv
import math
import sys
import warnings
from multiprocessing import Pool
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore")

FITTED = {"wheel_lag": "0", "wheel_backlash": "0.000001", "rotor_inertia": "22", "gear_stiffness": "19",
          "gear_damping": "0.1", "tyre_torsion": "0.01", "wheel_friction": "0", "leg_armature": "22"}
OUT = Path(__file__).resolve().parents[1] / "runs" / "stride_runs"
STEP = 0.05     # a sinusoid on the stick, as phases this long


def sweep(seconds, forward, yaw, amplitude, hz):
    return [f"{STEP},{forward},{yaw},0,{amplitude * math.sin(2 * math.pi * hz * (k + 1) * STEP):.4f}"
            for k in range(round(seconds / STEP))]


def scenario(segments):
    """Phases of the evaluator and (label, first phase, last phase, parameters) per segment."""
    phases, index = [], []
    for label, part, params in segments:
        index.append((label, len(phases), len(phases) + len(part) - 1, params))
        phases += part
    return phases, index


# The height goes from the start's 0.15 m to 0.12 m first: there the legs reach the whole command.
SEGMENTS = [("settle", ["2,0,0", "1,0,0,-0.03", "1,0,0"], None)]
SEGMENTS += [("hold", [f"2.5,0,0,0,{s}"], s) for s in (0.16, 0.08, -0.08, -0.16, 0.0)]
SEGMENTS += [("stand", sweep(6, 0, 0, a, hz), (a, hz)) for a, hz in ((0.12, 0.5), (0.09, 1.0))]
SEGMENTS += [("rest", ["1.5,0,0"], None)]
SEGMENTS += [("drive", ["1.5,0.4,0"] + sweep(6, 0.4, 0, a, hz), (a, hz)) for a, hz in ((0.12, 0.5), (0.09, 1.0))]
SEGMENTS += [("turn", sweep(5, 0.3, 1.0, 0.12, 0.5), (0.12, 0.5)), ("rest", ["2,0,0"], None)]
# The stick thrown from end to end every half second: the fastest stride the rate limit lets through.
SEGMENTS += [("flick", [f"0.5,0,0,0,{s}" for s in (0.16, -0.16) * 4], None), ("rest", ["2,0,0"], None)]
PHASES, INDEX = scenario(SEGMENTS)

VIDEO, _ = scenario([("", ["1.5,0,0", "1,0,0,-0.03"], None), ("", ["1.2,0,0,0,0.16", "1.2,0,0,0,-0.16"], None),
                     ("", sweep(4, 0, 0, 0.12, 0.5), None), ("", sweep(3, 0, 0, 0.09, 1.0), None),
                     ("", sweep(5, 0.4, 0, 0.12, 0.6), None), ("", sweep(4, 0.3, 1.2, 0.12, 0.6), None),
                     ("", ["1.5,0,0"], None)])


def watchdog(t, velocity, window_s=0.3):
    """RlPolicyController's oscillation watchdog (rl_policy_core.cpp watch_oscillation) on
    recorded joint velocities [n, 6]: per joint the 5-15 Hz band's RMS; (legs, wheels) per row."""
    high = [velocity[0].copy(), np.zeros(6)]
    low = [np.zeros(6), np.zeros(6)]
    mean_square, out = np.zeros(6), np.zeros((len(t), 2))
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        a_low, a_high = 1 - math.exp(-2 * math.pi * 5.0 * dt), 1 - math.exp(-2 * math.pi * 15.0 * dt)
        x = velocity[i].copy()
        for stage in high:
            stage += a_low * (x - stage)
            x = x - stage
        for stage in low:
            stage += a_high * (x - stage)
            x = stage.copy()
        mean_square += (1 - math.exp(-dt / window_s)) * (x * x - mean_square)
        rms = np.sqrt(mean_square)
        out[i] = rms[[0, 1, 3, 4]].max(), rms[[2, 5]].max()
    return out


def one(job):
    from sandro_rl import evaluate as e
    policy, plant, profile, seed = job
    e.SCENARIOS["stride"] = PHASES
    e.LINK_SHADOW = True
    OUT.mkdir(parents=True, exist_ok=True)
    out = OUT / f"{Path(policy).name}_{plant}_{profile}_{seed}.csv"
    code, path = e.run(Path(policy), "stride", {**e.PROFILES[profile], **(FITTED if plant else {})}, out, seed=seed)
    rows = [r for r in csv.DictReader(open(path)) if r.get("diag_truth_leg_split_m") not in ("", None)]
    col = lambda k: np.array([float(r[k] or 0) for r in rows])
    t, phase, split, ref, pitch, tilt, forward, contact, touching, height = (col(k) for k in (
        "t_s", "phase", "diag_truth_leg_split_m", "diag_roll_reference_rad", "diag_truth_pitch_rad",
        "diag_truth_tilt_rad", "diag_truth_forward_m_s", "diag_truth_floor_contact", "diag_truth_self_contact",
        "diag_truth_height_m"))
    link, roll, yaw = (col(k) for k in ("diag_truth_knee_height_m", "diag_truth_roll_rad",
                                         "diag_truth_yaw_rate_rad_s"))
    x, y = col("diag_truth_axle_x_m"), col("diag_truth_axle_y_m")
    joints = [r for r in csv.DictReader(open(path)) if r["stream"] == "joint"]
    jcol = lambda k: np.array([float(r[k] or 0) for r in joints])
    wheels = np.abs(np.stack([jcol("vel_right_wheel"), jcol("vel_left_wheel")])).max()
    legs = np.abs(np.stack([jcol(f"eff_{s}_motor_{j}") for s in ("right", "left") for j in ("inner", "outer")])).max()
    tj = jcol("t_s")
    watch = watchdog(tj, np.stack([jcol(f"vel_{n}") for n in (
        "right_motor_inner", "right_motor_outer", "right_wheel", "left_motor_inner", "left_motor_outer",
        "left_wheel")], axis=1))[tj > 3.0]
    over = watch[:, 0] > 1.0
    edges = np.flatnonzero(np.diff(np.concatenate([[0], over.astype(int), [0]])))
    result = {"fell": code == 3, "hold": [], "stand": [], "drive": [], "turn": [], "pitch": [], "wander": [], "hold_roll": [], "hold_yaw": [],
              "roll": [], "yaw": [], "turn_yaw": [], "still": [],
              "speed": [], "tilt": float(tilt.max()), "legs": float(legs), "wheel": float(wheels),
              "watch_leg": float(watch[:, 0].max()), "watch_wheel": float(watch[:, 1].max()),
              "watch_over": float((edges[1::2] - edges[::2]).max() * np.median(np.diff(tj))) if over.any() else 0.0,
              "other": float(np.mean(contact.astype(int) & 4 > 0)), "self": float(np.mean(touching > 0)), "link": float(link[tilt < 0.6].min()),
              "height": float(height[(phase >= INDEX[1][1])].mean()) if (phase >= INDEX[1][1]).any() else 0.0}
    for label, first, last, params in INDEX:
        mask = (phase >= first) & (phase <= last)
        if not mask.any() or t[mask][-1] - t[mask][0] < 1.5:
            continue
        tp = t[mask] - t[mask][0]
        if label == "hold":
            late = tp > tp[-1] - 0.7
            result["hold"].append((params, float(split[mask][late].mean()), float(ref[mask][late].mean())))
            result["hold_roll"].append(float(np.abs(roll[mask]).max()))
            result["hold_yaw"].append(float(np.abs(yaw[mask][late]).mean()))
            if params == 0.0:       # back together: does it stand still again
                result["still"].append(float(np.abs(forward[mask][tp > 1.0]).max()))
        if label == "flick":
            result["flick"] = float(0.5 * (split[mask][tp > 1.0].max() - split[mask][tp > 1.0].min()))
            result["yaw"].append(float(np.abs(yaw[mask][tp > 1.0]).max()))
        if label in ("stand", "drive", "turn"):
            late = tp > tp[-1] - 4.0        # whole periods of 0.5 and 1 Hz
            reached = 0.5 * (split[mask][late].max() - split[mask][late].min())
            asked = 0.5 * (ref[mask][late].max() - ref[mask][late].min())
            result[label].append((params[1], float(reached / max(asked, 1e-6)), float(reached)))
        if label in ("stand", "drive"):
            result["roll"].append(float(np.abs(roll[mask]).max()))
            result["yaw"].append(float(np.abs(yaw[mask][tp > 1.5]).max()))
        if label == "turn":
            result["turn_yaw"].append(float(yaw[mask][tp > 1.5].mean()))
        if label == "stand":
            result["pitch"].append(float(pitch[mask].max() - pitch[mask].min()))
            result["wander"].append(float(np.hypot(x[mask] - x[mask][0], y[mask] - y[mask][0]).max()))
        if label == "drive":
            result["speed"].append(float(forward[mask][tp > 2.5].mean()))
    return result


def span(values, digits=2):
    values = sorted(values)
    return (f"{values[0]:.{digits}f} .. {values[len(values) // 2]:.{digits}f} .. {values[-1]:.{digits}f}"
            if values else "none")


def video(policy):
    from sandro_rl import evaluate as e
    e.SCENARIOS["stride_video"] = VIDEO
    policy = Path(policy)
    for name, view in ((f"{policy.name}.mp4", (1.1, -8.0, 90.0)), (f"{policy.name}_front.mp4", (1.2, -15.0, 140.0))):
        e.VIDEO_VIEW = view
        e.run(policy, "stride_video", {**e.PROFILES["nominal"], **FITTED}, OUT / "video.csv", video=policy / name)


def main():
    args = sys.argv[1:]
    if "--video" in args:
        for policy in (a for a in args if a != "--video"):
            video(policy)
        return
    for policy in args:
        jobs = [(policy, plant, profile, seed) for plant in (0, 1)
                for profile in ("nominal", "spi_slow", "mit") for seed in (1, 2, 3)]
        with Pool(14, maxtasksperchild=1) as pool:
            results = pool.map(one, jobs, chunksize=1)
        every = lambda key: [v for r in results for v in r[key]]
        print(f"{policy}: falls {sum(r['fell'] for r in results)} of {len(results)}, height "
              f"{np.mean([r['height'] for r in results]):.3f} m")
        for asked in (0.16, 0.08, -0.08, -0.16, 0.0):
            got = [h[1] for h in every("hold") if h[0] == asked]
            print(f"    split asked {asked:+.2f} m: held {span(got, 3)} m")
        for label in ("stand", "drive", "turn"):
            for hz in (0.5, 1.0):
                part = [v for v in every(label) if v[0] == hz]
                if part:
                    print(f"    stride {label} at {hz} Hz: amplitude / the reference's {span([v[1] for v in part])}, "
                          f"{span([v[2] for v in part], 3)} m")
        print(f"    standing stride: pitch swing {span(every('pitch'))} rad, axle wander {span(every('wander'))} m; "
              f"speed in a stride at 0.4 m/s {span(every('speed'))} m/s; standing with the legs back together, speed up to "
              f"{span(every('still'))} m/s")
        print(f"    holding a split: roll up to {max(every('hold_roll')):.2f} rad, yaw rate left {span(every('hold_yaw'))} "
              f"rad/s; in a stride: roll up to {span(every('roll'))} rad, yaw rate up to {span(every('yaw'))} rad/s; "
              f"turning at 1 rad/s in a stride {span(every('turn_yaw'))} rad/s")
        print(f"    another part on the floor {100 * np.mean([r['other'] for r in results]):.1f} % of the time; "
              f"knees at least {min(r['link'] for r in results):.3f} m above the floor, a leg touching the "
              f"torso or the other leg {100 * np.mean([r['self'] for r in results]):.1f} % of the time")
        print(f"    the stick thrown end to end at 1 Hz: +-{span([r.get('flick', 0.0) for r in results], 3)} m of split")
        print(f"    the controller's oscillation watchdog would read up to {max(r['watch_leg'] for r in results):.2f} rad/s "
              f"at the legs (limit 1.0) and {max(r['watch_wheel'] for r in results):.2f} at the wheels (5.0); legs "
              f"beyond the limit for at most {max(r['watch_over'] for r in results):.2f} s at a time")
        print(f"    fastest wheel {max(r['wheel'] for r in results):.1f} rad/s, leg torque up to "
              f"{max(r['legs'] for r in results):.1f} Nm, tilt up to {max(r['tilt'] for r in results):.2f} rad")


if __name__ == "__main__":
    main()
