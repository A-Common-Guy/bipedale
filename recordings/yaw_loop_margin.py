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


if __name__ == "__main__":
    main()
