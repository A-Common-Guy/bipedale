"""The policy's yaw loop on the robot's measured yaw response, as a training loss.

The 9 Hz buzz on the robot is the policy's own feedback from the gyro's yaw
rate to the opposed wheel command, closed with a gain above 1 where the real
yaw plant has turned the phase round (docs/DESIGN.md, recordings/
yaw_loop_margin.py). No simulated drive train taught the policies otherwise,
so the measured plant goes into the loss directly:

  - the actor mean is linearised at observations of the minibatch (its
    Jacobian), which with the stacked frames and the last-action recursion is
    a transfer function from the gyro's yaw rate and the opposed wheel
    velocity to the opposed wheel command;
  - the plant is a table: measured opposed wheel torque to gyro yaw rate and
    to opposed wheel velocity (the WBC's sysid chirp, exported by
    recordings/yaw_loop_margin.py --export), sensor delays included;
  - the drive's own kd loop is closed first, then the policy's. The loop
    L(f) written so that the closed loop is 1 / (1 - L): it oscillates where
    L is real and reaches 1.

Penalised, per sampled observation:

  - inside the measured band, the largest Re L above `margin`. Re L <= 0.5
    everywhere keeps the Nyquist curve left of the line through 0.5: a gain
    margin of 2 at any phase crossing, 60 degrees of phase margin and a
    sensitivity peak of at most 2;
  - above the band (`rolloff_hz`), where nothing is measured, the loop's
    magnitude with the plant held at its last measured magnitudes, above
    `margin`: the policy must not differentiate the gyro there.

Both are zero for a policy that keeps the margin, so the term costs a
compliant policy nothing. train.loop_margin.coef > 0 switches it on (ppo.py).

The yaw loop through the wheels is one of three. The robot buzzes in roll
too, at 13-14 Hz, through the legs, and a policy has three actions that
change sign under the robot's left-right mirror: the wheels against each
other and the rear and the front leg motors against each other. OpposedLoop
closes all three at once on the 3-input plant the WBC's three sweeps measure
(sysid_yaw.py --axis yaw roll fore_aft, opposed_plant.json): from the opposed
wheel, rear and front motor torques to the gyro's roll and yaw rates, the
opposed leg angles and the opposed wheel velocity. The loop is then a 3 x 3
matrix and the scalar L becomes its eigenvalues: the closed loop oscillates
where one of them is real and reaches 1, and Re of every eigenvalue is held
under the margin. A plant file with a "plant" entry selects it (make_loop).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import jax
import jax.numpy as jp
import numpy as np

DATA_DIR = Path(__file__).resolve().parents[1] / "data"


@dataclass(frozen=True)
class Channels:
    """Where the yaw loop's signals sit in the actor observation and the action."""

    frame: int               # values per stacked frame
    history: int             # frames in the observation, newest first
    stride: int              # policy steps between stacked frames
    rate_hz: float           # policy rate
    gravity_z: int           # index in a frame
    gyro_z: int
    wheel_right: int         # wheel velocities in a frame
    wheel_left: int
    last_action: int         # first index of the last action in a frame
    n_actions: int
    action_right: int        # wheel actions
    action_left: int
    gyro_scale: float        # observation per rad/s
    wheel_scale: float       # observation per rad/s
    torque_per_action: float  # Nm of wheel torque per unit of wheel action
    drive_kd: float          # the drive's own damping on the wheel velocity, Nm s/rad (0: torque mode)
    # The legs, for the opposed loop. Leg observations and actions are in the
    # order right inner, right outer, left inner, left outer; the rear motors
    # are the right inner and the left outer (robot.py MIRROR_JOINT).
    gravity_y: int = 1
    gyro_x: int = 3
    leg_pos: int = 6         # first of the four leg angles in a frame
    leg_vel: int = 10
    leg_actions: tuple = (0, 1, 3, 4)
    leg_vel_scale: float = 0.1
    leg_scale: float = 0.5   # rad of leg reference per unit of action
    leg_kp: float = 12.0     # the drives' PD on the legs
    leg_kd: float = 0.5

    @classmethod
    def from_env(cls, env) -> "Channels":
        layout = env.actor_layout
        offsets = dict(zip(layout.names, np.concatenate([[0], np.cumsum(layout.sizes)[:-1]])))
        wheels = env.cfg["action"]["wheels"]
        scales = env.cfg["observation"]["scales"]
        velocity = env.wheel_mode == "velocity"
        right, left = (int(i) for i in np.asarray(env.wheel_idx))
        return cls(
            frame=int(layout.size), history=env.history, stride=env.history_stride, rate_hz=1.0 / env.dt,
            gravity_z=int(offsets["gravity"]) + 2, gyro_z=int(offsets["gyro"]) + 2,
            wheel_right=int(offsets["wheel_vel"]), wheel_left=int(offsets["wheel_vel"]) + 1,
            last_action=int(offsets["last_action"]), n_actions=env.n_actions,
            action_right=right, action_left=left,
            gyro_scale=float(scales["gyro"]), wheel_scale=float(scales["wheel_vel"]),
            torque_per_action=float(wheels["scale_rad_s"] * wheels["kd_nm_s_rad"] if velocity else wheels["scale_nm"]),
            drive_kd=float(wheels["kd_nm_s_rad"]) if velocity else 0.0,
            gravity_y=int(offsets["gravity"]) + 1, gyro_x=int(offsets["gyro"]),
            leg_pos=int(offsets["leg_pos"]), leg_vel=int(offsets["leg_vel"]),
            leg_actions=tuple(int(i) for i in np.asarray(env.leg_idx)),
            leg_vel_scale=float(scales["leg_vel"]), leg_scale=float(env.leg_scale),
            leg_kp=float(env.leg_kp), leg_kd=float(env.leg_kd))


def usable(f_hz, coherence, minimum: float = 0.7) -> int:
    """How many leading frequencies of a plant table to trust: up to the first
    whose coherence is below `minimum` (above ~30 Hz the yaw answer to the
    chirp's 0.4 Nm sinks into the gyro's noise)."""
    low = np.flatnonzero(np.asarray(coherence, dtype=float) < minimum)
    return int(low[0]) if len(low) else len(f_hz)


def load_plant(path: str | Path, min_coherence: float = 0.7) -> dict:
    """{"f_hz", "to_gyro", "to_wheel"}: complex responses per Nm of opposed wheel torque (per wheel)."""
    path = Path(path)
    if not path.is_file():
        path = DATA_DIR / path.name
    raw = json.loads(path.read_text())
    n = usable(raw["f_hz"], raw.get("coherence", [1.0] * len(raw["f_hz"])), min_coherence)
    cplx = lambda rows: np.array([complex(re, im) for re, im in rows[:n]])
    return {"f_hz": np.array(raw["f_hz"][:n], dtype=float), "to_gyro": cplx(raw["to_gyro"]),
            "to_wheel": cplx(raw["to_wheel"])}


class YawLoop:
    """The loop, the penalty and their constants for one environment."""

    def __init__(self, channels: Channels, plant: dict, margin: float = 0.5,
                 rolloff_hz=(), command_delay_s: float = 0.001, upright: float = -0.95):
        self.ch = c = channels
        self.margin = float(margin)
        self.upright = float(upright)
        f_band = np.asarray(plant["f_hz"], dtype=float)
        nyquist = 0.5 * c.rate_hz
        f_roll = np.array([f for f in rolloff_hz if f_band[-1] < f < nyquist], dtype=float)
        self.n_band = len(f_band)
        f = np.concatenate([f_band, f_roll])
        drive = 1.0 + c.drive_kd * plant["to_wheel"]
        to_gyro, to_wheel = plant["to_gyro"] / drive, plant["to_wheel"] / drive
        # Above the band only the last measured magnitudes are assumed.
        pad = lambda g: np.concatenate([g, np.full(len(f_roll), abs(g[-1]))])
        w, period = 2j * np.pi * f, 1.0 / c.rate_hz
        step = np.exp(-w * period)                       # one policy step
        self.f_hz = f
        self.step = jp.asarray(step, dtype=jp.complex64)
        self.lags = jp.asarray(np.stack([step ** (c.stride * i) for i in range(c.history)], 1), dtype=jp.complex64)
        # Command delay and the hold of the action over one policy period.
        self.path = jp.asarray(np.exp(-w * command_delay_s) * (1.0 - step) / (w * period), dtype=jp.complex64)
        self.to_gyro = jp.asarray(pad(to_gyro), dtype=jp.complex64)
        self.to_wheel = jp.asarray(pad(to_wheel), dtype=jp.complex64)

    @classmethod
    def from_config(cls, env, cfg: dict) -> "YawLoop":
        return cls(Channels.from_env(env), load_plant(cfg["plant"]), margin=cfg.get("margin", 0.5),
                   rolloff_hz=cfg.get("rolloff_hz", ()), command_delay_s=cfg.get("command_delay_s", 0.001))

    def responses(self, jacobian):
        """(from the gyro's yaw rate, from the opposed wheel velocity) to the opposed
        wheel torque command, Nm per rad/s, at self.f_hz. jacobian: [actions, observation]."""
        c = self.ch
        jac = jacobian.reshape(c.n_actions, c.history, c.frame).astype(jp.complex64)
        sensors = jp.stack([jac[:, :, c.gyro_z], jac[:, :, c.wheel_right] - jac[:, :, c.wheel_left]], -1)
        a = jp.einsum("fh,ahs->fas", self.lags, sensors)                      # [f, action, 2]
        last = jac[:, :, c.last_action:c.last_action + c.n_actions]
        b = jp.einsum("fh,ahb->fab", self.lags, last) * self.step[:, None, None]
        x = jp.linalg.solve(jp.eye(c.n_actions, dtype=jp.complex64) - b, a)   # the last-action recursion
        opposed = 0.5 * (x[:, c.action_right] - x[:, c.action_left]) * c.torque_per_action
        return opposed[:, 0] * c.gyro_scale, opposed[:, 1] * c.wheel_scale

    def loop(self, jacobian):
        """(L inside the measured band, a bound of |L| at the roll-off frequencies)."""
        from_gyro, from_wheel = self.responses(jacobian)
        n = self.n_band
        inside = self.path[:n] * (from_gyro[:n] * self.to_gyro[:n] + from_wheel[:n] * self.to_wheel[:n])
        above = jp.abs(self.path[n:]) * (jp.abs(from_gyro[n:]) * jp.abs(self.to_gyro[n:])
                                         + jp.abs(from_wheel[n:]) * jp.abs(self.to_wheel[n:]))
        return inside, above

    def measure(self, jacobian):
        """(worst Re L inside the band, the bound of |L| above it) of one linearisation."""
        inside, above = self.loop(jacobian)
        return jp.max(jp.real(inside)), jp.max(above) if above.shape[0] else jp.float32(0.0)

    def penalty(self, mean_fn, observations):
        """mean_fn: one raw actor observation -> the action mean. observations: [k, obs].
        Returns (penalty, {"loop_re": worst Re L, "loop_rolloff": worst |L| bound above the band})."""
        c = self.ch
        worst, high = jax.vmap(lambda obs: self.measure(jax.jacrev(mean_fn)(obs)))(observations)
        # Only where the robot stands on its wheels: the plant was measured there.
        weight = (observations[:, c.gravity_z] < self.upright).astype(jp.float32)
        total = jp.maximum(jp.sum(weight), 1.0)
        excess = jp.maximum(worst - self.margin, 0.0) ** 2 + jp.maximum(high - self.margin, 0.0) ** 2
        return jp.sum(weight * excess) / total, {"loop_re": jp.sum(weight * worst) / total,
                                                 "loop_rolloff": jp.sum(weight * high) / total}


def load_opposed_plant(path: str | Path, min_coherence: float = 0.7) -> dict:
    """{"f_hz", "plant"[f, 5, 3]} of sysid_yaw.py's opposed_plant.json: outputs (roll rate, yaw
    rate, rear angle, front angle, wheel velocity) per Nm of the opposed wheel, rear and front
    motor torques."""
    path = Path(path)
    if not path.is_file():
        path = DATA_DIR / path.name
    raw = json.loads(path.read_text())
    n = usable(raw["f_hz"], raw.get("coherence", [1.0] * len(raw["f_hz"])), min_coherence)
    plant = np.array(raw["plant"][:n], dtype=float)
    return {"f_hz": np.array(raw["f_hz"][:n], dtype=float), "plant": plant[..., 0] + 1j * plant[..., 1]}


def eigenvalues3(m):
    """The eigenvalues of a complex 3 x 3 matrix, differentiable, without an eigensolver
    (jax's is CPU only): the roots of the characteristic cubic by Durand-Kerner iteration,
    then one Newton step that carries the exact first-order gradient to the matrix."""
    c2 = m[0, 0] + m[1, 1] + m[2, 2]
    c1 = (m[0, 0] * m[1, 1] - m[0, 1] * m[1, 0] + m[0, 0] * m[2, 2] - m[0, 2] * m[2, 0]
          + m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1])
    c0 = (m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0])
          + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]))
    poly = lambda z, a2, a1, a0: ((z - a2) * z + a1) * z - a0
    s2, s1, s0 = (jax.lax.stop_gradient(c) for c in (c2, c1, c0))
    radius = 1.0 + jp.abs(s2) + jp.sqrt(jp.abs(s1)) + jp.abs(s0) ** (1.0 / 3.0)
    z = radius * jp.asarray([0.4 + 0.9j, -0.65 + 0.72j, 0.2 - 0.95j], dtype=m.dtype)

    def sweep(z, _):
        others = z[:, None] - z[None, :] + jp.eye(3, dtype=z.dtype)
        return z - poly(z, s2, s1, s0) / (jp.prod(others, axis=1) + 1e-20), None

    z, _ = jax.lax.scan(sweep, z, None, length=40)
    slope = (3.0 * z - 2.0 * s2) * z + s1
    slope = jp.where(jp.abs(slope) < 1e-6, 1e-6, slope)       # a repeated root: no gradient blow-up
    return z - poly(z, c2, c1, c0) / slope


class OpposedLoop(YawLoop):
    """The whole left-against-right loop: wheels, rear and front leg motors (see the module text)."""

    def __init__(self, channels: Channels, plant: dict, margin: float = 0.5,
                 rolloff_hz=(), command_delay_s: float = 0.001, upright: float = -0.95):
        self.ch = c = channels
        self.margin = float(margin)
        self.upright = float(upright)
        f_band = np.asarray(plant["f_hz"], dtype=float)
        f_roll = np.array([f for f in rolloff_hz if f_band[-1] < f < 0.5 * c.rate_hz], dtype=float)
        self.n_band = len(f_band)
        f = np.concatenate([f_band, f_roll])
        w, period = 2j * np.pi * f, 1.0 / c.rate_hz
        step = np.exp(-w * period)
        self.f_hz = f
        self.step = jp.asarray(step, dtype=jp.complex64)
        self.lags = jp.asarray(np.stack([step ** (c.stride * i) for i in range(c.history)], 1), dtype=jp.complex64)
        path = np.exp(-w * command_delay_s) * (1.0 - step) / (w * period)
        # The plant's outputs (roll rate, yaw rate, rear angle, front angle, wheel velocity) as
        # the policy observes them: [f, frame, 5]. Gravity's y is minus the roll angle; the leg
        # angles and velocities of a mirrored motion are +rear, +front, -front, -rear.
        sense = np.zeros((len(f), c.frame, 5), dtype=complex)
        sense[:, c.gravity_y, 0] = -1.0 / w
        sense[:, c.gyro_x, 0] = c.gyro_scale
        sense[:, c.gyro_z, 1] = c.gyro_scale
        for k, (column, sign) in enumerate(((2, 1.0), (3, 1.0), (3, -1.0), (2, -1.0))):
            sense[:, c.leg_pos + k, column] = sign
            sense[:, c.leg_vel + k, column] = sign * c.leg_vel_scale * w
        sense[:, c.wheel_right, 4] = c.wheel_scale
        sense[:, c.wheel_left, 4] = -c.wheel_scale
        self.sense = jp.asarray(sense, dtype=jp.complex64)
        # The policy's three opposed actions out of its six: wheels, rear, front.
        ri, ro, li, lo = c.leg_actions
        project = np.zeros((3, c.n_actions))
        project[0, c.action_right], project[0, c.action_left] = 0.5, -0.5
        project[1, ri], project[1, lo] = 0.5, -0.5
        project[2, ro], project[2, li] = 0.5, -0.5
        self.project = jp.asarray(project, dtype=jp.complex64)
        # From the actions back to the outputs: the drives' torque per action, their own loops
        # (kd on the wheel velocity, the PD on the leg angles) closed on the plant, and the
        # command path. Above the band the last measured magnitudes are assumed.
        gain = np.diag([c.torque_per_action, c.leg_kp * c.leg_scale, c.leg_kp * c.leg_scale])
        back = np.zeros((len(f), 5, 3), dtype=complex)
        for i in range(len(f)):
            g = plant["plant"][min(i, self.n_band - 1)]
            drive = np.zeros((3, 5), dtype=complex)
            drive[0, 4] = c.drive_kd
            drive[1, 2] = drive[2, 3] = c.leg_kp + w[min(i, self.n_band - 1)] * c.leg_kd
            closed = g @ np.linalg.solve(np.eye(3) + drive @ g, gain)
            back[i] = path[i] * closed if i < self.n_band else abs(path[i]) * np.abs(closed)
        self.back = jp.asarray(back, dtype=jp.complex64)

    @classmethod
    def from_config(cls, env, cfg: dict) -> "OpposedLoop":
        return cls(Channels.from_env(env), load_opposed_plant(cfg["plant"]), margin=cfg.get("margin", 0.5),
                   rolloff_hz=cfg.get("rolloff_hz", ()), command_delay_s=cfg.get("command_delay_s", 0.001))

    def matrices(self, jacobian):
        """The loop at the actions, [f, 3, 3]: policy response times the way back through the
        drives and the plant. Above the band the policy's part only ([f, 3, 5])."""
        c = self.ch
        jac = jacobian.reshape(c.n_actions, c.history, c.frame).astype(jp.complex64)
        seen = jp.einsum("fh,ahc,fcy->fay", self.lags, jac, self.sense)             # [f, action, 5]
        last = jac[:, :, c.last_action:c.last_action + c.n_actions]
        b = jp.einsum("fh,ahb->fab", self.lags, last) * self.step[:, None, None]
        x = jp.linalg.solve(jp.eye(c.n_actions, dtype=jp.complex64) - b, seen)      # the last-action recursion
        return jp.einsum("oa,fay->foy", self.project, x)                            # [f, 3, 5]

    def loop(self, jacobian):
        """(the loop's eigenvalues inside the band [f, 3], a bound of its gain above it)."""
        policy = self.matrices(jacobian)
        n = self.n_band
        inside = jax.vmap(eigenvalues3)(policy[:n] @ self.back[:n])
        # No phase is known up there: magnitudes, and the Frobenius norm over the eigenvalues.
        above = jp.sqrt(jp.sum((jp.abs(policy[n:]) @ jp.abs(self.back[n:])) ** 2, axis=(1, 2)))
        return inside, above


def make_loop(env, cfg: dict):
    """The loop a plant file is for: an opposed_plant.json (a "plant" entry) closes the whole
    left-against-right loop, a yaw_plant.json the yaw loop through the wheels alone."""
    path = Path(cfg["plant"])
    if not path.is_file():
        path = DATA_DIR / path.name
    opposed = "plant" in json.loads(path.read_text())
    return (OpposedLoop if opposed else YawLoop).from_config(env, cfg)
