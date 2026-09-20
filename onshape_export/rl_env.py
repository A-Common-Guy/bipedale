"""Gymnasium environment for the wheeled biped: command-following RL.

The task mirrors what the hand-built controllers do: follow velocity / yaw
rate / ride-height commands that change every few seconds, without falling
-- plus (v3) JUMP on cue: some episodes schedule jump events, announced by
a phase signal in the observation; during the jump window the reward pays
wheel ground clearance instead of tracking.

  observation (27, real-robot only -- IMU + encoders + own memory):
      up vector in body frame (3), gyro (3),
      leg motor angles rel. stand (4), leg motor velocities (4),
      wheel velocities fwd-convention (2), previous action (6),
      commands [v, yaw_rate, height] normalized (3),
      odometry forward-speed estimate (1, from wheel encoders + gyro --
      the same sensor math the hand controllers use),
      jump phase (1: ramps 0->1 in the 0.4 s pre-window, 1 during the
      0.5 s jump window, else 0)
  action (6, in [-1, 1], low-passed inside the env):
      4 leg MIT-mode position targets = stand pose + 0.6 rad * a
      2 wheel speed targets (forward convention) = 25 rad/s * a
      a = 0 exactly holds the stand pose with wheels stopped.
  reward:
      command tracking (velocity, yaw, height) + uprightness + alive,
      minus action-rate and torque penalties. The reward uses privileged
      sim state; the policy never sees it.
  episode:
      starts at the (perturbed) stand keyframe, commands resampled every
      2-4 s, terminates on fall (tilt > ~41 deg or base too low),
      truncates at 20 s (1000 steps at 50 Hz).

CLI:
    python rl_env.py --check    # mechanics, timing, zero/random baselines
    python rl_env.py --expert   # score the cascade controller with the
                                # same reward (calibration target for RL)
"""
from __future__ import annotations

import argparse

import mujoco
import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:            # keep the numpy-only training path alive
    gym = None
    spaces = None

from stand_pose import HERE, jid

SCENE = HERE / "robot" / "scene_floating.xml"

PHYSICS_HZ = 500
POLICY_HZ = 50
DECIMATION = PHYSICS_HZ // POLICY_HZ
EP_STEPS = 1000                # 20 s
ACT_FILTER = 0.4               # action low-pass blend per policy step

LEG_JOINTS = ("motor_inner", "motor_outer")
SIDES = ("right", "left")
LEG_ACT_SCALE = 0.6            # rad around the stand pose
WHEEL_ACT_SCALE = 25.0         # rad/s, forward convention

V_RANGE = 1.5                  # m/s command range
YAW_RANGE = 2.0                # rad/s
H_RANGE = (0.09, 0.185)        # commandable ride height (base-z mapped)

# reward v2: v1's bounded Gaussian bonuses let both ARS and PPO converge on
# a "pirouette" exploit -- spin at ~10 rad/s for gyroscopic stability and
# farm the height/upright/alive terms while ignoring commands. v2 adds
# LINEAR tracking-error penalties (unbounded incentive to actually follow
# commands) and widens the kernels so partial tracking pays a gradient.
# v3 adds narrow PRECISION bonuses (paying only near-exact tracking) and
# the jump task.
W_VEL, W_YAW, W_HGT, W_UP, W_ALIVE = 0.30, 0.20, 0.20, 0.15, 0.15
SIG_V, SIG_YAW = 0.40, 0.50
W_VEL_FINE, W_YAW_FINE = 0.10, 0.10
SIG_V_FINE, SIG_YAW_FINE = 0.12, 0.15
P_VEL_LIN, P_YAW_LIN = 0.08, 0.08     # linear |error| penalties (clipped)
P_ACT_RATE, P_TORQUE = 0.05, 5e-5
TILT_LIMIT = 0.75              # cos(tilt) below this ends the episode
Z_MIN = 0.045

# jump task: scheduled in JUMP_PROB of episodes; the phase signal ramps
# during PRE (crouch warning), then the JUMP window pays wheel clearance
JUMP_PROB = 0.35
JUMP_PRE = 0.4                 # s
JUMP_WIN = 0.5                 # s
JUMP_W = 2.0                   # per-step weight on clip(clearance/5cm, 0, 1)
JUMP_CLEAR_REF = 0.05          # m
# v3.1 shaping: clearance alone is a hard-exploration cliff (zero gradient
# until a full crouch-slam is coordinated). Pay the stepping stones: crouch
# depth during the pre-window, upward base velocity during the window.
JUMP_W_CROUCH = 0.4
JUMP_W_VZ = 0.6


class WheeledBipedEnv(gym.Env if gym else object):
    metadata = {"render_modes": []}

    def __init__(self, seed=None, jump_prob=JUMP_PROB):
        self.model = mujoco.MjModel.from_xml_path(str(SCENE))
        self.data = mujoco.MjData(self.model)
        self.rng = np.random.default_rng(seed)
        self.jump_prob = jump_prob
        m = self.model

        self.key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand")
        self._act_id = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a): a
                        for a in range(m.nu)}
        self._sens = {}
        for s in range(m.nsensor):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, s)
            self._sens[name] = (m.sensor_adr[s], m.sensor_dim[s])

        # frames + wheel sign conventions, from the stand keyframe (the same
        # derivation the controllers use). These axes are contracted against
        # imu_quat/imu_gyro, so they must be expressed in the IMU SITE frame,
        # not the torso body frame: the site is mounted flat on the roof while
        # the exported torso frame points along the wheel axle, 113 deg away.
        qk = m.key_qpos[self.key]
        mujoco.mj_resetDataKeyframe(m, self.data, self.key)
        mujoco.mj_forward(m, self.data)
        imu = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "imu")
        R0 = self.data.site_xmat[imu].reshape(3, 3).copy()
        self.fwd_t = R0.T @ [1.0, 0, 0]
        self.axle_t = R0.T @ [0, 1.0, 0]
        self.up_t = R0.T @ [0, 0, 1.0]
        ax = {s: self.data.xaxis[jid(m, f"{s}_wheel")].copy() for s in SIDES}
        self.wheel_dir = {s: (1.0 if ax[s] @ ax[SIDES[0]] > 0 else -1.0)
                          for s in SIDES}
        f_w = R0 @ self.fwd_t
        self.fwd_sign = 1.0 if (np.cross(ax[SIDES[0]], [0, 0, 1.0]) @ f_w) > 0 else -1.0
        self.wheel_jnt = {s: jid(m, f"{s}_wheel") for s in SIDES}
        self.wheel_dof = {s: m.jnt_dofadr[self.wheel_jnt[s]] for s in SIDES}
        # stand targets for the leg actuators (keyframe ctrl holds the pose)
        self.stand_ctrl = m.key_ctrl[self.key].copy()
        self.leg_acts = [f"{s}_{j}" for s in SIDES for j in LEG_JOINTS]
        self.z0 = float(qk[2])
        self.h0 = 0.186        # commanded-height label of the stand pose
        # wheel radius from the keyframe (tires sit 1 mm above the floor)
        self.wheel_r = float(np.mean(
            [self.data.xanchor[self.wheel_jnt[s]][2] for s in SIDES])) - 0.001

        self.obs_dim = 27
        self.act_dim = 6
        self._jumps = []       # scheduled jump onset times (s)
        if gym:
            self.observation_space = spaces.Box(-np.inf, np.inf,
                                                (self.obs_dim,), np.float64)
            self.action_space = spaces.Box(-1.0, 1.0, (self.act_dim,), np.float64)

        self._a_filt = np.zeros(self.act_dim)
        self._a_prev = np.zeros(self.act_dim)
        self.cmd = np.zeros(3)
        self._next_cmd_t = 0.0
        self._step = 0

    # ------------------------------------------------------------- plumbing
    def sensor(self, name):
        adr, dim = self._sens[name]
        return self.data.sensordata[adr:adr + dim]

    def _R(self):
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, self.sensor("imu_quat"))
        return R.reshape(3, 3)

    def _sample_cmd(self):
        # 20% pure station-keeping, otherwise random commands
        if self.rng.random() < 0.2:
            self.cmd = np.array([0.0, 0.0, self.rng.uniform(*H_RANGE)])
        else:
            self.cmd = np.array([self.rng.uniform(-V_RANGE, V_RANGE),
                                 self.rng.uniform(-YAW_RANGE, YAW_RANGE),
                                 self.rng.uniform(*H_RANGE)])
        self._next_cmd_t = self._step / POLICY_HZ + self.rng.uniform(2.0, 4.0)

    def _jump_phase(self):
        """(phase signal, inside-jump-window) for the current time."""
        t = self._step / POLICY_HZ
        for tj in self._jumps:
            if tj - JUMP_PRE <= t < tj:
                return (t - (tj - JUMP_PRE)) / JUMP_PRE, False
            if tj <= t < tj + JUMP_WIN:
                return 1.0, True
        return 0.0, False

    def _clearance(self):
        return float(min(self.data.xanchor[self.wheel_jnt[s]][2]
                         for s in SIDES)) - self.wheel_r

    # ------------------------------------------------------ true quantities
    def _true_state(self):
        """Privileged quantities for the reward (never in the observation)."""
        m, d = self.model, self.data
        R = self._R()
        z = np.array([0.0, 0, 1.0])
        up_w = R @ self.up_t
        f_w = R @ self.fwd_t
        fh = f_w - z * (f_w @ z)
        fh /= max(np.linalg.norm(fh), 1e-9)
        # axle-midpoint world velocity along the heading
        vs = []
        for s in SIDES:
            j = self.wheel_jnt[s]
            b = int(m.jnt_bodyid[j])
            Jp = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, Jp, None, d.xanchor[j], b)
            vs.append(Jp @ d.qvel)
        v_fwd = float(np.mean(vs, axis=0) @ fh)
        omega_w = R @ self.sensor("imu_gyro")
        yaw_rate = float(omega_w @ z)
        return v_fwd, yaw_rate, float(d.qpos[2]), float(up_w[2])

    # -------------------------------------------------------------- obs/act
    def _obs(self):
        R = self._R()
        up_b = R.T @ [0.0, 0, 1.0]
        gyro = np.asarray(self.sensor("imu_gyro"))
        legs_q = np.array([self.sensor(f"{n}_pos")[0] - self.stand_ctrl[self._act_id[n]]
                           for n in self.leg_acts])
        legs_v = np.array([self.sensor(f"{n}_vel")[0] for n in self.leg_acts])
        wheels = np.array([self.fwd_sign * self.wheel_dir[s]
                           * self.sensor(f"{s}_wheel_vel")[0] for s in SIDES])
        h_mid, h_half = np.mean(H_RANGE), (H_RANGE[1] - H_RANGE[0]) / 2
        cmd_n = np.array([self.cmd[0] / V_RANGE, self.cmd[1] / YAW_RANGE,
                          (self.cmd[2] - h_mid) / h_half])
        # sensor-only forward-speed estimate (wheel odometry + pitch-rate
        # correction) -- without it the policy has to LEARN odometry, and
        # velocity tracking was the biggest gap to the scripted expert
        pitch_rate = float(gyro @ self.axle_t)
        v_est = self.wheel_r * (float(np.mean(wheels)) + pitch_rate)
        phase, _ = self._jump_phase()
        return np.concatenate([up_b, 0.25 * gyro, legs_q, 0.1 * legs_v,
                               0.05 * wheels, self._a_prev, cmd_n,
                               [v_est / V_RANGE, phase]])

    def _apply(self, a):
        self._a_filt += ACT_FILTER * (a - self._a_filt)
        af = self._a_filt
        for i, n in enumerate(self.leg_acts):
            self.data.ctrl[self._act_id[n]] = \
                self.stand_ctrl[self._act_id[n]] + LEG_ACT_SCALE * af[i]
        for k, s in enumerate(SIDES):
            self.data.ctrl[self._act_id[f"{s}_wheel"]] = \
                self.fwd_sign * self.wheel_dir[s] * WHEEL_ACT_SCALE * af[4 + k]

    # ------------------------------------------------------------- gym API
    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        m, d = self.model, self.data
        mujoco.mj_resetDataKeyframe(m, d, self.key)
        # small perturbations: joints, base tilt, velocities
        d.qpos[7:] += self.rng.uniform(-0.02, 0.02, m.nq - 7)
        tilt = self.rng.uniform(-0.05, 0.05, 2)
        dq = np.zeros(4)
        mujoco.mju_axisAngle2Quat(dq, [tilt[0], tilt[1], 0.0],
                                  float(np.linalg.norm(tilt) + 1e-9))
        qn = np.zeros(4)
        mujoco.mju_mulQuat(qn, dq, d.qpos[3:7])
        d.qpos[3:7] = qn / np.linalg.norm(qn)
        d.qvel[:] = self.rng.uniform(-0.05, 0.05, m.nv)
        d.ctrl[:] = self.stand_ctrl
        mujoco.mj_forward(m, d)
        self._a_filt[:] = 0.0
        self._a_prev[:] = 0.0
        self._step = 0
        self._next_cmd_t = 0.0
        self._sample_cmd()
        # schedule jump events for this episode
        self._jumps = []
        if self.rng.random() < self.jump_prob:
            t1 = float(self.rng.uniform(3.0, 12.0))
            self._jumps.append(t1)
            if self.rng.random() < 0.5:
                self._jumps.append(t1 + float(self.rng.uniform(5.0, 7.0)))
        self._max_clear = 0.0
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        self._apply(a)
        for _ in range(DECIMATION):
            mujoco.mj_step(self.model, self.data)
        self._step += 1
        if self._step / POLICY_HZ >= self._next_cmd_t:
            self._sample_cmd()

        v_fwd, yaw_rate, z, upz = self._true_state()
        z_cmd = self.z0 + (self.cmd[2] - self.h0)
        act_rate = float(np.mean((a - self._a_prev) ** 2))
        torque = float(np.mean(self.data.actuator_force ** 2))
        phase, in_jump = self._jump_phase()
        if phase > 0.0:
            # jump mode: tracking terms gated off; the window pays clearance
            clear = self._clearance()
            self._max_clear = max(self._max_clear, clear)
            r = (W_UP * np.clip((upz - TILT_LIMIT) / (1 - TILT_LIMIT), 0, 1)
                 + W_ALIVE
                 - P_TORQUE * torque)
            if in_jump:
                vz = float(self.data.qvel[2])
                r += (JUMP_W * np.clip(clear / JUMP_CLEAR_REF, 0, 1)
                      + JUMP_W_VZ * np.clip(vz / 0.6, 0, 1))
            else:
                # pre-window: pay crouch depth, scaled by the phase ramp
                r += JUMP_W_CROUCH * phase * np.clip(
                    (self.z0 - z) / 0.10, 0, 1)
        else:
            r = (W_VEL * np.exp(-((v_fwd - self.cmd[0]) / SIG_V) ** 2)
                 + W_YAW * np.exp(-((yaw_rate - self.cmd[1]) / SIG_YAW) ** 2)
                 + W_VEL_FINE * np.exp(-((v_fwd - self.cmd[0]) / SIG_V_FINE) ** 2)
                 + W_YAW_FINE * np.exp(-((yaw_rate - self.cmd[1]) / SIG_YAW_FINE) ** 2)
                 + W_HGT * np.exp(-((z - z_cmd) / 0.02) ** 2)
                 + W_UP * np.clip((upz - TILT_LIMIT) / (1 - TILT_LIMIT), 0, 1)
                 + W_ALIVE
                 - P_VEL_LIN * min(abs(v_fwd - self.cmd[0]), 2.0)
                 - P_YAW_LIN * min(abs(yaw_rate - self.cmd[1]), 3.0)
                 - P_ACT_RATE * act_rate - P_TORQUE * torque)
        self._a_prev = a
        terminated = bool(upz < TILT_LIMIT or z < Z_MIN)
        truncated = bool(self._step >= EP_STEPS)
        info = dict(v_err=abs(v_fwd - self.cmd[0]),
                    yaw_err=abs(yaw_rate - self.cmd[1]),
                    z_err=abs(z - z_cmd), upz=upz,
                    max_clear=self._max_clear)
        return self._obs(), float(r), terminated, truncated, info


# ---------------------------------------------------------------- baselines
def rollout(env, policy, max_steps=EP_STEPS, seed=None):
    obs, _ = env.reset(seed=seed)
    total, steps = 0.0, 0
    infos = []
    for _ in range(max_steps):
        obs, r, term, trunc, info = env.step(policy(obs))
        total += r
        steps += 1
        infos.append(info)
        if term or trunc:
            break
    mean = {k: float(np.mean([i[k] for i in infos])) for k in infos[0]}
    return total, steps, mean


def check():
    import time
    env = WheeledBipedEnv(seed=0)
    o, _ = env.reset(seed=0)
    assert o.shape == (env.obs_dim,), o.shape
    t0 = time.perf_counter()
    n = 0
    for ep in range(3):
        env.reset(seed=ep)
        for _ in range(EP_STEPS):
            _, r, term, trunc, _ = env.step(env.rng.uniform(-1, 1, 6))
            n += 1
            if term or trunc:
                break
    dt = time.perf_counter() - t0
    print(f"random policy: {n} steps in {dt:.2f} s -> {n/dt:,.0f} policy steps/s")
    for name, pol in (("zero-action", lambda o: np.zeros(6)),
                      ("random", lambda o: env.rng.uniform(-1, 1, 6))):
        rets = []
        for ep in range(5):
            total, steps, mean = rollout(env, pol, seed=100 + ep)
            rets.append((total, steps))
        print(f"{name:12s}: return {np.mean([r for r, _ in rets]):7.1f} "
              f"+- {np.std([r for r, _ in rets]):5.1f}, "
              f"mean ep len {np.mean([s for _, s in rets]):5.0f}")


def expert():
    """Score the cascade controller under the env's command schedule and
    reward -- the calibration target for RL."""
    import drive_controller as dc
    from controller_demo import Robot

    env = WheeledBipedEnv(seed=0)   # used only for schedules + reward math
    rets = []
    for ep in range(5):
        rng = np.random.default_rng(1000 + ep)
        robot = Robot()
        ctl = dc.WholeBodyController(robot)
        # drive the cascade with env-like commands; score with env reward
        env.model, env.data = robot.model, robot.data   # reuse reward math
        env.rng = rng
        env._step = 0
        env._next_cmd_t = 0.0
        env._sample_cmd()
        total, steps = 0.0, 0
        n_ctl = round(1 / (POLICY_HZ * ctl.dt))          # ctl ticks per env step
        for k in range(EP_STEPS):
            lo, hi = ctl.cmd.h_range
            ctl.cmd.v_tgt = float(env.cmd[0])
            ctl.cmd.yaw_tgt = float(env.cmd[1])
            ctl.cmd.h_tgt = float(np.clip(env.cmd[2], lo, hi))
            for _ in range(max(1, n_ctl)):
                ctl.tick()
                robot.step_control_period()
            env._step += 1
            if env._step / POLICY_HZ >= env._next_cmd_t:
                env._sample_cmd()
            v_fwd, yaw_rate, z, upz = env._true_state()
            z_cmd = env.z0 + (env.cmd[2] - env.h0)
            r = (W_VEL * np.exp(-((v_fwd - env.cmd[0]) / SIG_V) ** 2)
                 + W_YAW * np.exp(-((yaw_rate - env.cmd[1]) / SIG_YAW) ** 2)
                 + W_VEL_FINE * np.exp(-((v_fwd - env.cmd[0]) / SIG_V_FINE) ** 2)
                 + W_YAW_FINE * np.exp(-((yaw_rate - env.cmd[1]) / SIG_YAW_FINE) ** 2)
                 + W_HGT * np.exp(-((z - z_cmd) / 0.02) ** 2)
                 + W_UP * np.clip((upz - TILT_LIMIT) / (1 - TILT_LIMIT), 0, 1)
                 + W_ALIVE
                 - P_VEL_LIN * min(abs(v_fwd - env.cmd[0]), 2.0)
                 - P_YAW_LIN * min(abs(yaw_rate - env.cmd[1]), 3.0))
            total += r
            steps += 1
            if upz < TILT_LIMIT or z < Z_MIN:
                break
        rets.append((total, steps))
        print(f"  expert ep {ep}: return {total:7.1f}  steps {steps}")
    print(f"expert (cascade): return {np.mean([r for r, _ in rets]):7.1f} "
          f"+- {np.std([r for r, _ in rets]):5.1f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--expert", action="store_true")
    args = ap.parse_args()
    if args.check:
        check()
    if args.expert:
        expert()
