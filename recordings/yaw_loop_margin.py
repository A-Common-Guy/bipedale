"""Would this policy buzz? Its yaw loop closed on the robot's measured yaw response.

    python3 yaw_loop_margin.py [POLICY_DIR ...] [--filter HZ] [--plant JSON]

No simulator: the policy (policy.onnx + policy.json in a sandro_rl/policies
directory) is linearised about standing still, which gives its response from
the gyro's yaw rate and the opposed wheel velocity to the opposed wheel
command, the stacked frames and the last-action recursion included. The plant
is the WBC's sysid chirp: measured opposed wheel torque to gyro yaw rate and
to opposed wheel velocity. The drive's own kd loop
is closed first, then the policy's.

Read the table as: at a "phase crossing" the loop feeds a yaw oscillation back
in phase; |L| above 1 there and it grows (the robot's 9 Hz buzz), and 0.5 is a
6 dB gain margin. The peak of |1/(1-L)| says how much the loop amplifies a
disturbance near that frequency. "max Re L" is what the trainer
penalises above its margin (sandro_rl/loop_margin.py, train.loop_margin): at or
below 0.5 the policy keeps a gain margin of 2 and 60 degrees of phase margin.
The second part closes the whole left-against-right loop, the wheels and the
rear and front leg motors together, on the 3-input plant of a run of all three
sweeps (opposed_plant.json): the roll buzz through the legs shows there.
The plant table is sysid_yaw.py's yaw_plant.json (the panel's Sysid page). The
roll-rate gain is the legs' loop behind the 13-14 Hz roll buzz, which has no
measured plant yet: compare it with the robot's numbers in roll_gain(). Standing at three heights, and (for
comparison) driving and turning, because the policies' yaw gain depends on the
command.

Needs numpy and onnxruntime.
"""
import argparse
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
RADIUS, TRACK = 0.0742, 0.304
GX, GZ, WHEEL_R, WHEEL_L = 3, 5, 14, 15    # frame indices: gyro x and z, wheel velocities
ACTIONS = slice(16, 22)                   # last action in the frame
A_RW, A_LW = 2, 5                         # wheel actions


class Policy:
    def __init__(self, directory):
        import onnxruntime as ort
        directory = Path(directory)
        self.name = directory.name
        self.meta = json.loads((directory / "policy.json").read_text())
        options = ort.SessionOptions()
        options.log_severity_level = 3
        self.session = ort.InferenceSession(str(directory / "policy.onnx"), options,
                                            providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0].name
        obs = self.meta["observation"]
        self.frame = sum(n for _, n in obs["frame"])
        self.history = obs["history"]
        self.stride = obs.get("history_stride") or 1
        self.scales = obs["scales"]
        self.norm = obs["command_normalisation"]
        self.rate = self.meta["policy_rate_hz"]
        wheels = self.meta["action"]["wheels"]
        self.wheel_scale, self.kd = wheels["scale_rad_s"], wheels["kd_nm_s_rad"]

    def __call__(self, observation):
        x = np.asarray(observation, np.float32).reshape(1, -1)
        return self.session.run(None, {self.input: x})[0][0].astype(np.float64)

    def operating_point(self, forward=0.0, yaw=0.0, height=None):
        """A steady frame (level, the last action equal to the action) at a command."""
        frame = np.zeros(self.frame)
        frame[2] = -1.0
        right = (forward + yaw * TRACK / 2) / RADIUS
        left = (forward - yaw * TRACK / 2) / RADIUS
        frame[GZ] = yaw * self.scales["gyro"]
        frame[WHEEL_R], frame[WHEEL_L] = right * self.scales["wheel_vel"], left * self.scales["wheel_vel"]
        frame[22], frame[23] = forward / self.norm["forward_m_s"], yaw / self.norm["yaw_rad_s"]
        if height is not None:
            frame[24] = height
        action = np.zeros(6)
        for _ in range(200):
            frame[ACTIONS] = action
            frame[6:10] = 0.5 * action[[0, 1, 3, 4]]     # legs at their references
            new = self(np.tile(frame, self.history))
            if np.abs(new - action).max() < 1e-6:
                break
            action = new
        return frame

    def jacobian(self, frame, eps=1e-3):
        x = np.tile(frame, self.history)
        jac = np.zeros((6, x.size))
        for i in range(x.size):
            step = np.zeros(x.size)
            step[i] = eps
            jac[:, i] = (self(x + step) - self(x - step)) / (2 * eps)
        return jac.reshape(6, self.history, self.frame)

    def response(self, jac, f):
        """[frequency, action, sensor channel]: the frames' delays and the last-action loop."""
        out = []
        for hz in f:
            step = np.exp(-2j * np.pi * hz / self.rate)
            lag = step ** self.stride
            sensors = sum(jac[:, i, :16] * lag ** i for i in range(self.history))
            actions = sum(jac[:, i, ACTIONS] * lag ** i * step for i in range(self.history))
            out.append(np.linalg.solve(np.eye(6) - actions, sensors))
        return np.array(out)


def plant(table, min_coherence=0.7):
    """A plant table (sysid_yaw.py's yaw_plant.json): opposed wheel torque (per wheel) to gyro
    yaw rate and to opposed wheel velocity, up to the first frequency of low coherence."""
    raw = json.loads(Path(table).read_text())
    coherence = np.array(raw.get("coherence", [1.0] * len(raw["f_hz"])))
    low = np.flatnonzero(coherence < min_coherence)
    n = int(low[0]) if len(low) else len(coherence)
    cplx = lambda rows: np.array([complex(re, im) for re, im in rows[:n]])
    return np.array(raw["f_hz"][:n]), cplx(raw["to_gyro"]), cplx(raw["to_wheel"])


def roll_gain(policy, jac, hz=14.0):
    """Opposed leg reference (rad per joint, right minus left) per rad/s of the gyro's roll
    rate: the loop behind the 13-14 Hz roll buzz. No roll plant is measured yet; on the robot
    0.044 was quiet (0.04 rad/s), 0.07 buzzed at 0.10-0.18 and 0.094-0.105 at 0.27-0.30."""
    h = policy.response(jac, np.array([hz]))[0]
    legs = [i for i in range(6) if i not in (A_RW, A_LW)]           # right inner, outer, left inner, outer
    gain = (h[legs[0], GX] + h[legs[1], GX] - h[legs[2], GX] - h[legs[3], GX]) / 4
    return abs(gain) * policy.scales["gyro"] * policy.meta["action"]["legs"]["scale_rad"]


def loop(policy, jac, f, to_gyro, to_wheel, filter_hz=0.0, command_delay_s=0.001):
    h = policy.response(jac, f)
    gain = policy.wheel_scale * policy.kd          # Nm per unit of wheel action
    from_gyro = (h[:, A_RW, GZ] - h[:, A_LW, GZ]) / 2 * policy.scales["gyro"] * gain
    from_wheel = ((h[:, A_RW, WHEEL_R] - h[:, A_RW, WHEEL_L]) - (h[:, A_LW, WHEEL_R] - h[:, A_LW, WHEEL_L])) \
        / 2 * policy.scales["wheel_vel"] * gain
    w, period = 2j * np.pi * f, 1.0 / policy.rate
    path = np.exp(-w * command_delay_s) * (1 - np.exp(-w * period)) / (w * period)   # delay and hold
    if filter_hz > 0:
        a = 1 - np.exp(-2 * np.pi * filter_hz * period)
        path = path * a / (1 - (1 - a) * np.exp(-w * period))
    drive = 1 + policy.kd * to_wheel               # the drive's kd on its own wheel, no bus delay
    return path * (from_gyro * to_gyro + from_wheel * to_wheel) / drive, from_gyro


def opposed_plant(table, min_coherence=0.7):
    """sysid_yaw.py's opposed_plant.json: (f, plant[f, 5, 3]) from the opposed wheel, rear and
    front motor torques to the roll and yaw rates, the rear and front leg angles and the wheel
    velocity."""
    raw = json.loads(Path(table).read_text())
    low = np.flatnonzero(np.array(raw.get("coherence", [1.0] * len(raw["f_hz"]))) < min_coherence)
    n = int(low[0]) if len(low) else len(raw["f_hz"])
    g = np.array(raw["plant"][:n], dtype=float)
    return np.array(raw["f_hz"][:n]), g[..., 0] + 1j * g[..., 1]


def opposed_loop(policy, jac, f, g, filter_hz=0.0, keep=(1.0, 1.0, 1.0), command_delay_s=0.001):
    """Eigenvalues [f, 3] of the whole left-against-right loop: the policy's opposed wheel,
    rear-motor and front-motor actions closed on the measured 3-input plant, the drives' own
    loops (kd on the wheels, the PD on the legs) closed first. The loop oscillates where an
    eigenvalue is real and reaches 1. keep: 0 drops an action (wheels, rear, front), to see
    which loop a crossing belongs to."""
    h = policy.response(jac, f)
    legs, wheels = policy.meta["action"]["legs"], policy.meta["action"]["wheels"]
    period = 1.0 / policy.rate
    project = np.zeros((3, 6))
    project[0, [A_RW, A_LW]] = 0.5, -0.5
    project[1, [0, 4]] = 0.5, -0.5                 # rear: right inner against left outer
    project[2, [1, 3]] = 0.5, -0.5                 # front: right outer against left inner
    gain = np.diag([policy.wheel_scale * policy.kd, legs["kp"] * legs["scale_rad"],
                    legs["kp"] * legs["scale_rad"]]) * np.array(keep)
    out = []
    for i, hz in enumerate(f):
        w = 2j * np.pi * hz
        sense = np.zeros((16, 5), dtype=complex)   # roll rate, yaw rate, rear, front angle, wheel velocity
        sense[1, 0] = -1.0 / w                     # gravity y is minus the roll angle
        sense[GX, 0] = sense[GZ, 1] = policy.scales["gyro"]
        for k, (column, sign) in enumerate(((2, 1.0), (3, 1.0), (3, -1.0), (2, -1.0))):
            sense[6 + k, column] = sign
            sense[10 + k, column] = sign * policy.scales["leg_vel"] * w
        sense[WHEEL_R, 4], sense[WHEEL_L, 4] = policy.scales["wheel_vel"], -policy.scales["wheel_vel"]
        path = np.exp(-w * command_delay_s) * (1 - np.exp(-w * period)) / (w * period) * np.ones(3, dtype=complex)
        if filter_hz > 0:
            a = 1 - np.exp(-2 * np.pi * filter_hz * period)
            path[0] *= a / (1 - (1 - a) * np.exp(-w * period))
        drive = np.zeros((3, 5), dtype=complex)
        drive[0, 4] = policy.kd
        drive[1, 2] = drive[2, 3] = legs["kp"] + w * legs["kd"]
        back = g[i] @ np.linalg.solve(np.eye(3) + drive @ g[i], gain @ np.diag(path))
        out.append(np.linalg.eigvals(project @ h[i] @ sense @ back))
    return np.array(out)


def eigen_crossings(f, eig, floor=0.3):
    """(Hz, value) where an eigenvalue crosses the positive real axis, the branches followed
    from one frequency to the next."""
    from itertools import permutations
    found, branches = [], eig[0]
    for k in range(1, len(f)):
        order = min(permutations(range(3)), key=lambda p: sum(abs(eig[k][list(p)] - branches)))
        following = eig[k][list(order)]
        for a, b in zip(branches, following):
            if a.real > floor and b.real > floor and a.imag * b.imag < 0:
                t = a.imag / (a.imag - b.imag)
                found.append((f[k - 1] + t * (f[k] - f[k - 1]), a.real + t * (b.real - a.real)))
        branches = following
    return found


def opposed_report(policy, f, g, filter_hz):
    """One policy on the opposed plant: standing at three heights, each loop on its own."""
    print(f"\n{policy.name} ({policy.rate} Hz)")
    worst = 0.0
    for label, command in (("standing", {}), ("standing low", {"height": -0.8}), ("standing high", {"height": 0.8})):
        jac = policy.jacobian(policy.operating_point(**command))
        cells = []
        for name, keep in (("all", (1, 1, 1)), ("wheels alone", (1, 0, 0)), ("legs alone", (0, 1, 1))):
            eig = opposed_loop(policy, jac, f, g, filter_hz, keep)
            found = sorted(eigen_crossings(f, eig), key=lambda c: -c[1])[:2]
            if name == "all":
                worst = max([worst] + [value for _, value in found])
                re = eig.real.max(1)
                cells.append(f"max Re {re.max():4.2f} at {f[re.argmax()]:4.1f} Hz")
            cells.append(f"{name}: " + ("; ".join(f"{hz:4.1f} Hz {value:4.2f}" for hz, value in found) or "no crossing"))
        print(f"  {label:14s} " + "   ".join(cells))
    return worst


def crossings(f, L):
    """(Hz, |L|) where L is real and positive: 1 - L = 0 there when |L| = 1."""
    out, phase = [], np.unwrap(np.angle(L))
    for turn in range(-3, 4):
        s = phase - 2 * np.pi * turn
        for i in np.where(np.sign(s[:-1]) != np.sign(s[1:]))[0]:
            t = s[i] / (s[i] - s[i + 1])
            out.append((f[i] + t * (f[i + 1] - f[i]), abs(L[i]) + t * (abs(L[i + 1]) - abs(L[i]))))
    return sorted(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("policies", nargs="*", help="sandro_rl/policies/<name> directories (default: the deployed ones)")
    parser.add_argument("--filter", type=float, default=0.0, help="action.wheel_differential_filter_hz to include")
    parser.add_argument("--opposed", default=str(HERE.parent / "sandro_rl/data/opposed_plant_2026-11-17.json"),
                        help="3-input plant (opposed_plant.json of a run of all three axes); 'none' skips it")
    parser.add_argument("--plant", default=str(HERE.parent / "sandro_rl/data/yaw_plant_2026-11-16.json"),
                        help="plant table (yaw_plant.json of a sysid_yaw.py run or of the panel's Sysid page)")
    args = parser.parse_args()
    root = HERE.parent / "sandro_rl/policies"
    names = args.policies or [str(root / n) for n in (
        "flat_level_real", "flat_level_fast", "flat_level_drivetrain", "flat_level_fast_drivetrain",
        "flat_level_sysid", "flat_level_fast_sysid", "agile_terrain")]
    f, to_gyro, to_wheel = plant(args.plant)
    points = [("standing", {}), ("standing low", {"height": -0.8}), ("standing high", {"height": 0.8}),
              ("0.5 m/s", {"forward": 0.5}), ("turning 1 rad/s", {"yaw": 1.0})]
    print(f"yaw loop on the measured plant, {f[0]:.0f}-{f[-1]:.0f} Hz"
          + (f", opposed wheel filter {args.filter:g} Hz" if args.filter else ""))
    worst_standing = {}
    for directory in names:
        policy = Policy(directory)
        print(f"\n{policy.name} ({policy.rate} Hz)")
        for label, command in points:
            jac = policy.jacobian(policy.operating_point(**command))
            L, from_gyro = loop(policy, jac, f, to_gyro, to_wheel, args.filter)
            found = crossings(f, L)
            sensitivity = np.abs(1 / (1 - L))
            nine = np.argmin(abs(f - 9))
            print(f"  {label:16s} yaw-rate gain at 9 Hz {abs(from_gyro[nine]):4.2f} Nm per rad/s   "
                  f"phase crossing {'; '.join(f'{hz:4.1f} Hz |L| {g:4.2f}' for hz, g in found) or 'none':24s} "
                  f"peak |1/(1-L)| {sensitivity.max():5.1f} at {f[sensitivity.argmax()]:4.1f} Hz   "
                  f"max Re L {L.real.max():5.2f} at {f[L.real.argmax()]:4.1f} Hz"
                  + (f"   roll-rate gain at 14 Hz {roll_gain(policy, jac):.3f} rad per rad/s"
                     if label == "standing" else ""))
            if label.startswith("standing"):
                worst_standing[policy.name] = max([worst_standing.get(policy.name, 0.0)] + [g for _, g in found])
    print("\nworst standing |L| at a phase crossing (above 1 it oscillates; aim for 0.5 or less):")
    for name, value in worst_standing.items():
        print(f"  {name:28s} {value:4.2f}  {'BUZZ' if value >= 1 else 'marginal' if value > 0.7 else 'ok'}")
    if args.opposed != "none":
        f, g = opposed_plant(args.opposed)
        print(f"\nthe whole left-against-right loop (wheels, rear and front leg motors) on the measured "
              f"3-input plant, {f[0]:.0f}-{f[-1]:.0f} Hz"
              + (f", opposed wheel filter {args.filter:g} Hz" if args.filter else "")
              + "\ncrossing: where an eigenvalue of the loop is real, and its value there (1: it oscillates)."
              " The yaw buzz is the\ncrossing near 9-12 Hz of the wheels, the roll buzz the one near 14-15 Hz"
              " of the legs.")
        worst = {Path(d).name: opposed_report(Policy(d), f, g, args.filter) for d in names}
        print("\nworst standing crossing of the whole loop (rl_flat_level_sysid rings on the robot at "
              "0.8-0.9: aim for 0.5 or less):")
        for name, value in worst.items():
            print(f"  {name:28s} {value:4.2f}  {'BUZZ' if value >= 0.8 else 'marginal' if value > 0.6 else 'ok'}")


if __name__ == "__main__":
    main()
