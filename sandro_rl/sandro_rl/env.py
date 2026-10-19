"""Sandro command-tracking environment on MJX (MuJoCo Warp backend).

One policy step is `decimation` physics steps of the generated ROS model.
Before every physics step the six drives execute the MIT law on the live
joint state (sandro_sim/MitDrivePlugin), each with the frame that was due
at that instant (sandro_sim/DriveBus: a frame becomes due `command_delay`
after it was written). After every physics step the true sensor values,
with noise, go into a history buffer, and the policy reads them through the
per-channel sample-and-hold delay lines of latency.py (sandro_sim/DelayLine
timing). The actor sees only what the robot measures; the critic also gets
the true state and the randomised parameters.

All functions act on a single environment; the trainer vmaps them over the
batch and over the per-environment randomised model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import partial
from typing import Any

import jax
import jax.numpy as jp
import mujoco
import numpy as np
from flax import struct
from mujoco import mjx

from . import kinematics, latency, terrain
from .robot import RobotSpec, rotation_x
from .sources import JOINTS

N_JOINTS = 6
IMU_CHANNELS = 10           # quaternion wxyz, gyro, accelerometer
N_CHANNELS = IMU_CHANNELS + 2 * N_JOINTS
HISTORY_LENGTH = 160        # physics steps (133 ms): longest delay plus the slowest sample period


@struct.dataclass
class State:
    data: mjx.Data
    obs: dict
    reward: jax.Array
    done: jax.Array          # episode ended (termination or time-out)
    terminated: jax.Array    # ended by a failure (no bootstrapping)
    cat_delta: jax.Array     # constraint termination probability of this step
    info: dict
    metrics: dict


def _ros(value, fallback):
    return fallback if value == "ros" else value


def _quat_to_mat(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    return jp.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _rot_xy(roll, pitch):
    cr, sr, cp, sp = jp.cos(roll), jp.sin(roll), jp.cos(pitch), jp.sin(pitch)
    rx = jp.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = jp.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return ry @ rx


@dataclass
class ObsLayout:
    """Names, sizes and mirror maps of the observation components."""

    names: list
    sizes: list
    perm: np.ndarray
    sign: np.ndarray

    @property
    def size(self) -> int:
        return int(sum(self.sizes))


class SandroEnv:
    def __init__(self, config: dict, spec: RobotSpec | None = None):
        self.config = config
        cfg = config["env"]
        self.cfg = cfg
        base_spec = spec or RobotSpec.load()
        model = terrain.build(base_spec, cfg.get("terrain", {}))
        self.spec = RobotSpec.load(base_spec.scene, model=model) if model is not base_spec.model else base_spec
        s = self.spec
        lim = s.limits
        self.mj_model = s.model
        self.impl = cfg.get("impl", "warp")
        graph_mode = None
        if self.impl == "warp" and cfg.get("graph_mode"):
            import warp as wp
            graph_mode = getattr(wp.JaxCallableGraphMode, cfg["graph_mode"].upper())
        self.mjx_model = mjx.put_model(s.model, impl=self.impl, graph_mode=graph_mode)
        self.dt_physics = s.physics_dt
        control_steps = s.steps_per_control
        control_rate = lim.control_rate_hz
        cycles = control_rate / float(cfg["policy_rate_hz"])
        if abs(cycles - round(cycles)) > 1e-9:
            raise ValueError(f"policy rate {cfg['policy_rate_hz']} Hz must divide the {control_rate:g} Hz control rate")
        self.decimation = int(round(cycles)) * control_steps
        self.dt = self.decimation * self.dt_physics
        self.episode_steps = int(round(cfg["episode_s"] / self.dt))

        # Indices and constants.
        self.qadr = jp.array(s.joint_qpos)
        self.dadr = jp.array(s.joint_dof)
        self.act_ids = jp.array(s.joint_actuator)
        self.leg_idx = s.leg_index
        self.wheel_idx = s.wheel_index
        self.stand_q = jp.array(s.stand_q, dtype=jp.float32)
        self.stand_qpos = jp.array(s.stand_qpos, dtype=jp.float32)
        self.geometry = lim.leg_geometry
        self.neutral = lim.fore_aft_neutral_m
        self.wheel_radius = lim.wheel_radius_m
        quat_adr, _ = s.sensor_adr["imu_quat"]
        gyro_adr, _ = s.sensor_adr["imu_gyro"]
        accel_adr, _ = s.sensor_adr["imu_accel"]
        self.sensor_index = jp.array([quat_adr + i for i in range(4)] + [gyro_adr + i for i in range(3)]
                                     + [accel_adr + i for i in range(3)])
        self.imu_noise = np.array([s.sensor_noise["imu_quat"]] * 4 + [s.sensor_noise["imu_gyro"]] * 3
                                  + [s.sensor_noise["imu_accel"]] * 3, dtype=np.float32)
        self.body_from_imu = jp.array(rotation_x(-s.imu_roll_bias_rad), dtype=jp.float32)
        offset = cfg["observation"].get("imu_offset_correction", "ros")
        from . import sources
        self.imu_offset_correction = float(
            sources.controller_parameters()["tasks"]["roll"].get("imu_offset_rad", 0.0)
            if offset == "ros" else offset)

        a = cfg["action"]
        self.leg_kp = float(_ros(a["legs"]["kp"], lim.leg_kp_nm_rad))
        self.leg_kd = float(_ros(a["legs"]["kd"], lim.leg_kd_nm_s_rad))
        self.leg_scale = float(a["legs"]["scale_rad"])
        self.wheel_mode = a["wheels"]["mode"]
        self.torque_limit = jp.array([lim.leg_torque_nm if j not in ("right_wheel", "left_wheel")
                                      else lim.wheel_torque_nm for j in JOINTS], dtype=jp.float32)
        self.q_lo, self.q_hi = s.leg_joint_range

        c = cfg["commands"]
        self.v_max = float(_ros(c["forward_m_s"], lim.max_forward_m_s))
        self.yaw_max = float(_ros(c["yaw_rad_s"], lim.max_yaw_rad_s))
        h = _ros(c["height_m"], [lim.height_min_m, lim.height_max_m])
        self.h_min, self.h_max = float(h[0]), float(h[1])
        self.accel_limit = float(_ros(c["forward_accel_m_s2"], lim.forward_accel_limit_m_s2))
        self.yaw_tau = float(_ros(c["yaw_time_constant_s"], lim.yaw_command_time_constant_s))
        self.h_rate = float(_ros(c["height_rate_m_s"], lim.max_height_rate_m_s))
        # Optional body roll command (differential leg height), for animation.
        self.roll_max = float(c.get("roll_rad", 0.0))
        self.has_roll = self.roll_max > 0.0
        self.n_command = 4 if self.has_roll else 3
        self.roll_rate = float(c.get("roll_rate_rad_s", 0.5))
        reach = c.get("leg_height_limits_m", [self.h_min, self.h_max])
        self.leg_h_lo, self.leg_h_hi = float(reach[0]), float(reach[1])
        self.track_width = float(lim.track_width_m)
        p = cfg["pushes"]
        self.push_dim = 5 if ("vertical_force_n" in p or "torque_nm" in p) else 2

        ct = cfg["constraints"]
        self.cat_torque_limit = jp.array(
            [float(_ros(ct["leg_torque"]["limit"], lim.leg_torque_nm))] * 4, dtype=jp.float32)

        self.n_actions = N_JOINTS
        self.batch_size = 1   # set by the caller before vmapping reset (contact buffer size)
        self.actor_layout = self._actor_layout()
        self.critic_layout = self._critic_layout()
        self.history = int(cfg["observation"]["history"])
        self.actor_obs_size = self.actor_layout.size * self.history
        self.critic_obs_size = self.critic_layout.size
        self.action_perm = s.mirror_perm
        self.payload_body = mujoco.mj_name2id(s.model, mujoco.mjtObj.mjOBJ_BODY, "identified_payload")
        self.terrain = terrain.Terrain(self.spec, cfg.get("terrain", {}))
        d = mujoco.MjData(s.model)
        d.qpos[:] = s.stand_qpos
        mujoco.mj_kinematics(s.model, d)
        self.stand_axle = d.xanchor[s.wheel_joint_ids].mean(axis=0)

    # ------------------------------------------------------------ layouts
    def _component_mirror(self, name: str, size: int):
        perm6 = self.spec.mirror_perm
        legs = list(self.leg_idx)
        leg_perm = np.array([legs.index(perm6[j]) for j in legs])
        wheel_perm = np.array([1, 0])
        table = {
            "gravity": (np.arange(3), np.array([1, -1, 1])),
            "gyro": (np.arange(3), np.array([-1, 1, -1])),
            "accel": (np.arange(3), np.array([1, -1, 1])),
            "leg_pos": (leg_perm, np.ones(4)),
            "leg_vel": (leg_perm, np.ones(4)),
            "wheel_vel": (wheel_perm, np.ones(2)),
            "last_action": (perm6, np.ones(6)),
            "command": (np.arange(size), np.array([1, -1, 1, -1])[:size]),
            "true_gravity": (np.arange(3), np.array([1, -1, 1])),
            "true_gyro": (np.arange(3), np.array([-1, 1, -1])),
            "base_velocity": (np.arange(3), np.array([1, -1, 1])),
            "true_leg_pos": (leg_perm, np.ones(4)),
            "true_leg_vel": (leg_perm, np.ones(4)),
            "true_wheel_vel": (wheel_perm, np.ones(2)),
            "leg_height": (np.array([1, 0]), np.ones(2)),
            "leg_offset": (np.array([1, 0]), np.ones(2)),
            "wheel_clearance": (np.array([1, 0]), np.ones(2)),
            "push": (np.arange(size), np.array([1, -1, 1, -1, -1])[:size]),
            "params": (np.arange(size), np.ones(size)),
        }
        return table[name]

    def _layout(self, components):
        names, sizes, perm, sign, offset = [], [], [], [], 0
        for name, size in components:
            p, sg = self._component_mirror(name, size)
            names.append(name)
            sizes.append(size)
            perm.append(np.asarray(p) + offset)
            sign.append(np.asarray(sg, dtype=np.float32))
            offset += size
        return ObsLayout(names, sizes, np.concatenate(perm), np.concatenate(sign))

    def _actor_layout(self):
        comps = [("gravity", 3), ("gyro", 3)]
        if self.cfg["observation"].get("accelerometer", False):
            comps.append(("accel", 3))
        comps += [("leg_pos", 4), ("leg_vel", 4), ("wheel_vel", 2), ("last_action", 6), ("command", self.n_command)]
        return self._layout(comps)

    N_PARAMS = 8 + 8   # episode parameters + randomize.MODEL_PARAMS

    def _critic_layout(self):
        comps = [("gravity", 3), ("gyro", 3), ("leg_pos", 4), ("leg_vel", 4), ("wheel_vel", 2),
                 ("last_action", 6), ("command", self.n_command),
                 ("true_gravity", 3), ("true_gyro", 3), ("base_velocity", 3), ("true_leg_pos", 4),
                 ("true_leg_vel", 4), ("true_wheel_vel", 2), ("leg_height", 2), ("leg_offset", 2),
                 ("wheel_clearance", 2), ("push", self.push_dim), ("params", self.N_PARAMS)]
        return self._layout(comps)

    def mirror_actor_obs(self, obs):
        frame = self.actor_layout.size
        perm = np.concatenate([self.actor_layout.perm + k * frame for k in range(self.history)])
        sign = np.tile(self.actor_layout.sign, self.history)
        return obs[..., perm] * sign

    def mirror_critic_obs(self, obs):
        return obs[..., self.critic_layout.perm] * self.critic_layout.sign

    def mirror_action(self, action):
        return action[..., self.action_perm]

    # ------------------------------------------------------------ sampling
    def _sample_episode_params(self, rng, randomize: bool):
        r = self.cfg["randomization"]
        lat = self.spec.latency
        keys = jax.random.split(rng, 16)
        u = lambda k, lo_hi, shape=(): jax.random.uniform(k, shape, minval=lo_hi[0], maxval=lo_hi[1])
        if randomize and r.get("enabled", True):
            L = r["latency"]
            imu_delay = u(keys[0], L["imu_delay_s"])
            degraded = jax.random.uniform(keys[1]) < L["degraded_imu_probability"]
            imu_rate = jp.where(degraded, u(keys[2], L["degraded_imu_rate_hz"]), u(keys[3], L["imu_rate_hz"]))
            joint_delay = u(keys[4], L["joint_delay_s"])
            joint_rate = u(keys[5], L["joint_rate_hz"])
            sync = jax.random.uniform(keys[6]) < L["synchronous_joints_probability"]
            joint_phase = jp.where(sync, jp.zeros(N_JOINTS), jax.random.uniform(keys[7], (N_JOINTS,)) / joint_rate)
            imu_phase = jax.random.uniform(keys[8]) / imu_rate
            command_delay = u(keys[9], L["command_delay_s"])
            S = r["sensors"]
            noise_scale = u(keys[10], S["imu_noise_scale"])
            vel_gain = u(keys[11], S["joint_velocity_gain"], (N_JOINTS,))
            k2 = jax.random.split(keys[12], 4)
            mount = jax.random.uniform(k2[0], (2,), minval=-1, maxval=1) * S["imu_mount_bias_rad"]
            gyro_bias = jax.random.uniform(k2[1], (3,), minval=-1, maxval=1) * S["gyro_bias_rad_s"]
            D = r["drives"]
            torque_scale = u(keys[13], D["torque_scale"], (N_JOINTS,))
            gain_scale = u(keys[14], D["leg_gain_scale"])
            enc_noise, vel_noise = S["encoder_noise_rad"], S["joint_velocity_noise"]
        else:
            imu_delay = jp.float32(lat.imu_delay_s)
            imu_rate = jp.float32(lat.imu_rate_hz)
            joint_delay = jp.float32(lat.joint_delay_s)
            joint_rate = jp.float32(lat.joint_rate_hz)
            period = 1.0 / lat.joint_rate_hz if lat.joint_rate_hz > 0 else 0.0
            joint_phase = (jp.arange(N_JOINTS) * period / N_JOINTS if lat.joint_stagger
                           else jp.zeros(N_JOINTS))
            imu_phase = jp.float32(0.0)
            command_delay = jp.float32(lat.command_delay_s)
            noise_scale = jp.float32(1.0)
            vel_gain = jp.ones(N_JOINTS)
            mount = jp.zeros(2)
            gyro_bias = jp.zeros(3)
            torque_scale = jp.ones(N_JOINTS)
            gain_scale = jp.float32(1.0)
            enc_noise, vel_noise = 0.0, 0.0
        imu_period = jp.where(imu_rate > 0, 1.0 / jp.maximum(imu_rate, 1e-6), 0.0)
        joint_period = jp.where(joint_rate > 0, 1.0 / jp.maximum(joint_rate, 1e-6), 0.0)
        delay = jp.concatenate([jp.full(IMU_CHANNELS, imu_delay), jp.full(2 * N_JOINTS, joint_delay)])
        period = jp.concatenate([jp.full(IMU_CHANNELS, imu_period), jp.full(2 * N_JOINTS, joint_period)])
        phase = jp.concatenate([jp.full(IMU_CHANNELS, imu_phase), joint_phase, joint_phase])
        noise = jp.concatenate([jp.asarray(self.imu_noise) * noise_scale,
                                jp.full(N_JOINTS, enc_noise), jp.full(N_JOINTS, vel_noise)])
        return {
            "delay": delay.astype(jp.float32), "period": period.astype(jp.float32),
            "phase": phase.astype(jp.float32), "noise": noise.astype(jp.float32),
            "command_delay": jp.float32(command_delay),
            "vel_gain": vel_gain.astype(jp.float32),
            "mount": _rot_xy(mount[0], mount[1]).astype(jp.float32),
            "gyro_bias": gyro_bias.astype(jp.float32),
            "torque_scale": torque_scale.astype(jp.float32),
            "gain_scale": jp.float32(gain_scale),
            "imu_rate": jp.float32(imu_rate), "imu_delay": jp.float32(imu_delay),
        }

    def _sample_command(self, rng, previous_height):
        c = self.cfg["commands"]
        k = jax.random.split(rng, 10)
        # Curriculum stages narrow the sampled range; the observation keeps
        # normalising by the full range, so the policy's inputs keep their meaning.
        fraction = float(c.get("sample_fraction", 1.0))
        v = jax.random.uniform(k[0], minval=-self.v_max, maxval=self.v_max) * fraction
        w = jax.random.uniform(k[1], minval=-self.yaw_max, maxval=self.yaw_max) * fraction
        v = jp.where(jax.random.uniform(k[2]) < c["component_zero_probability"], 0.0, v)
        w = jp.where(jax.random.uniform(k[3]) < c["component_zero_probability"], 0.0, w)
        zero = jax.random.uniform(k[4]) < c["zero_probability"]
        v, w = jp.where(zero, 0.0, v), jp.where(zero, 0.0, w)
        h = jax.random.uniform(k[5], minval=self.h_min, maxval=self.h_max)
        h = jp.where(jax.random.uniform(k[6]) < c["height_change_probability"], h, previous_height)
        duration = jax.random.uniform(k[7], minval=c["resample_s"][0], maxval=c["resample_s"][1])
        if not self.has_roll:
            return jp.array([v, w, h]), duration
        roll = jax.random.uniform(k[8], minval=-self.roll_max, maxval=self.roll_max) * fraction
        roll = jp.where(jax.random.uniform(k[9]) < c.get("roll_zero_probability", 0.4), 0.0, roll)
        return jp.array([v, w, h, self.feasible_roll(roll, h)]), duration

    def feasible_roll(self, roll, height):
        """Clip a roll command so both legs stay inside their reachable height range."""
        room = jp.maximum(jp.minimum(height - self.leg_h_lo, self.leg_h_hi - height), 0.0)
        limit = jp.arctan(2.0 * room / self.track_width)
        return jp.clip(roll, -limit, limit)

    def _sample_push(self, rng):
        """Fore-aft, lateral (and vertical force, roll and yaw torque) in the heading frame."""
        p = self.cfg["pushes"]
        k = jax.random.split(rng, 4)
        wait = jax.random.uniform(k[0], minval=p["interval_s"][0], maxval=p["interval_s"][1])
        duration = jax.random.uniform(k[1], minval=p["duration_s"][0], maxval=p["duration_s"][1])
        scale = jp.array([p["force_n"], p["lateral_force_n"], p.get("vertical_force_n", 0.0),
                          *p.get("torque_nm", [0.0, 0.0])])[: self.push_dim]
        force = jax.random.uniform(k[2], (self.push_dim,), minval=-1, maxval=1) * scale
        enabled = jp.float32(1.0 if p.get("enabled", True) else 0.0)
        return wait, duration, force * enabled

    # ------------------------------------------------------------ physics helpers
    def _record(self, data, params, rng):
        raw = jp.concatenate([data.sensordata[self.sensor_index], data.qpos[self.qadr], data.qvel[self.dadr]])
        return raw + params["noise"] * jax.random.normal(rng, (N_CHANNELS,))

    def _frames(self, action, params):
        """MIT frame per joint (q_ref, v_ref, kp, kd, tau_ff) from the policy action."""
        a = jp.clip(action, -self.cfg["action"]["clip"], self.cfg["action"]["clip"])
        q_ref = jp.zeros(N_JOINTS)
        v_ref = jp.zeros(N_JOINTS)
        kp = jp.zeros(N_JOINTS)
        kd = jp.zeros(N_JOINTS)
        ff = jp.zeros(N_JOINTS)
        legs = self.leg_idx
        q_ref = q_ref.at[legs].set(jp.clip(self.stand_q[legs] + self.leg_scale * a[legs], self.q_lo, self.q_hi))
        kp = kp.at[legs].set(self.leg_kp * params["gain_scale"])
        kd = kd.at[legs].set(self.leg_kd * params["gain_scale"])
        wheels = self.wheel_idx
        w = self.cfg["action"]["wheels"]
        if self.wheel_mode == "velocity":
            v_ref = v_ref.at[wheels].set(w["scale_rad_s"] * a[wheels])
            kd = kd.at[wheels].set(w["kd_nm_s_rad"])
        elif self.wheel_mode == "torque":
            ff = ff.at[wheels].set(w["scale_nm"] * a[wheels])
        else:
            raise ValueError(f"unknown wheel action mode {self.wheel_mode}")
        return jp.stack([q_ref, v_ref, kp, kd, ff])

    def _mit_torque(self, frame, data, params):
        q = data.qpos[self.qadr]
        v = data.qvel[self.dadr]
        q_ref = jp.where(frame[2] > 0, frame[0], q)
        tau = frame[2] * (q_ref - q) + frame[3] * (frame[1] - v) + frame[4]
        tau = jp.clip(tau, -self.torque_limit, self.torque_limit)
        return tau

    def _body_frame(self, data):
        r_imu = data.site_xmat[self.spec.imu_site].reshape(3, 3)
        return r_imu, r_imu @ self.body_from_imu

    def _axle_mid(self, data):
        return jp.mean(data.xanchor[self.spec.wheel_joint_ids], axis=0)

    # ------------------------------------------------------------ observation
    def _read_sensors(self, info):
        params = info["params"]
        step = info["physics_step"]
        now = step.astype(jp.float32) * self.dt_physics
        when = latency.sample_time(now, params["delay"], params["period"], params["phase"])
        return latency.read(info["history"], step, self.dt_physics, when)

    def _actor_frame(self, sensed, info):
        params = info["params"]
        quat = sensed[0:4] / jp.linalg.norm(sensed[0:4])
        r_imu = _quat_to_mat(quat) @ params["mount"]
        # The deployment controller removes the known mounting roll, as
        # tasks.roll.imu_offset_rad does in the whole-body controller.
        r_body = r_imu @ jp.array(rotation_x(-self.imu_offset_correction), dtype=jp.float32)
        gravity = r_body.T @ jp.array([0.0, 0.0, -1.0])
        gyro = params["mount"].T @ sensed[4:7] + params["gyro_bias"]
        accel = params["mount"].T @ sensed[7:10]
        q = sensed[10:16]
        v = sensed[16:22] * params["vel_gain"]
        sc = self.cfg["observation"]["scales"]
        cmd = info["command_ref"]
        command = jp.array([cmd[0] / self.v_max, cmd[1] / self.yaw_max,
                            (cmd[2] - 0.5 * (self.h_min + self.h_max)) / (0.5 * (self.h_max - self.h_min))])
        if self.has_roll:
            command = jp.concatenate([command, (cmd[3] / self.roll_max)[None]])
        parts = {"gravity": gravity, "gyro": gyro * sc["gyro"], "accel": accel / 9.81,
                 "leg_pos": q[self.leg_idx] - self.stand_q[self.leg_idx],
                 "leg_vel": v[self.leg_idx] * sc["leg_vel"], "wheel_vel": v[self.wheel_idx] * sc["wheel_vel"],
                 "last_action": info["last_action"], "command": command}
        return jp.concatenate([parts[n] for n in self.actor_layout.names]), parts

    def _truth(self, data, info):
        r_imu, r_body = self._body_frame(data)
        gyro_true = data.sensordata[self.sensor_index[4:7]]
        omega_world = r_imu @ gyro_true
        fwd = r_body[:, 0]
        heading = jp.array([fwd[0], fwd[1], 0.0])
        heading = heading / jp.maximum(jp.linalg.norm(heading), 1e-6)
        lateral = jp.array([-heading[1], heading[0], 0.0])
        mid = self._axle_mid(data)
        vel = (mid - info["axle_prev"]) / self.dt
        q = data.qpos[self.qadr]
        qd = data.qvel[self.dadr]
        heights, offsets, stretch = kinematics.leg_state(q[self.leg_idx], self.geometry, self.neutral, jp)
        pitch = jp.arctan2(-r_body[2, 0], jp.sqrt(r_body[0, 0] ** 2 + r_body[1, 0] ** 2))
        roll = jp.arctan2(r_body[2, 1], r_body[2, 2])
        anchors = data.xanchor[self.spec.wheel_joint_ids]
        ground = self.terrain.ground_height(jp.concatenate([anchors, mid[None]]), data)
        clearance = anchors[:, 2] - self.wheel_radius - ground[:2]
        return {
            "r_body": r_body, "gravity": r_body.T @ jp.array([0.0, 0.0, -1.0]),
            "gyro": self.body_from_imu.T @ gyro_true,
            "forward_speed": vel @ heading, "lateral_speed": vel @ lateral, "vertical_speed": vel[2],
            "yaw_rate": omega_world[2], "pitch": pitch, "roll": roll,
            "pitch_rate": (self.body_from_imu.T @ gyro_true)[1],
            "tilt": jp.arccos(jp.clip(r_body[2, 2], -1.0, 1.0)),
            "q": q, "qd": qd, "heights": heights, "offsets": offsets, "stretch": stretch,
            "imu_height": data.site_xpos[self.spec.imu_site][2] - ground[2], "clearance": clearance, "axle": mid,
        }

    def _critic_obs(self, actor_parts, truth, info, model_params):
        sc = self.cfg["observation"]["scales"]
        p = info["params"]
        params = jp.concatenate([
            jp.array([p["imu_delay"] * 50.0, jp.minimum(p["imu_rate"], 400.0) / 200.0,
                      p["delay"][IMU_CHANNELS] * 300.0, p["command_delay"] * 300.0, p["gain_scale"]]),
            p["torque_scale"][self.wheel_idx], jp.mean(p["torque_scale"][self.leg_idx])[None],
            model_params,
        ])
        parts = dict(actor_parts)
        parts.update({
            "true_gravity": truth["gravity"], "true_gyro": truth["gyro"] * sc["gyro"],
            "base_velocity": jp.array([truth["forward_speed"], truth["lateral_speed"], truth["vertical_speed"]]),
            "true_leg_pos": truth["q"][self.leg_idx] - self.stand_q[self.leg_idx],
            "true_leg_vel": truth["qd"][self.leg_idx] * sc["leg_vel"],
            "true_wheel_vel": truth["qd"][self.wheel_idx] * sc["wheel_vel"],
            "leg_height": (truth["heights"] - 0.17) * 20.0, "leg_offset": truth["offsets"] * 20.0,
            "wheel_clearance": jp.clip(truth["clearance"], -0.05, 0.1) * 20.0,
            "push": info["push_force"] / 10.0, "params": params,
        })
        return jp.concatenate([parts[n] for n in self.critic_layout.names])

    # ------------------------------------------------------------ API
    def reset(self, model: mjx.Model, rng: jax.Array, model_params: jax.Array,
              randomize: bool = True) -> State:
        k = jax.random.split(rng, 10)
        init = self.cfg["init"]
        # MuJoCo Warp sizes the contact buffer for the whole batch.
        data = mjx.make_data(self.mj_model, impl=self.impl,
                             naconmax=self.cfg["naconmax_per_env"] * self.batch_size,
                             njmax=self.cfg["njmax_per_env"])
        qpos = self.stand_qpos
        noise = jax.random.uniform(k[0], (N_JOINTS,), minval=-1, maxval=1) * init["joint_noise_rad"]
        qpos = qpos.at[self.qadr].add(noise)
        tilt = jax.random.uniform(k[1], (2,), minval=-1, maxval=1) * init["tilt_rad"]
        yaw = jax.random.uniform(k[2], minval=-math.pi, maxval=math.pi)
        qpos = self._rotate_base(qpos, tilt, yaw)
        qvel = jax.random.uniform(k[4], (self.mj_model.nv,), minval=-1, maxval=1) * init["velocity_noise"]
        data = data.replace(qpos=qpos, qvel=qvel, ctrl=jp.zeros(self.mj_model.nu))
        data = self.terrain.apply(data, *self.terrain.layout(k[3]))
        data = mjx.forward(model, data)
        params = self._sample_episode_params(k[5], randomize)
        h0 = jp.clip(jp.mean(kinematics.leg_state(qpos[self.qadr][self.leg_idx], self.geometry,
                                                  self.neutral, jp)[0]), self.h_min, self.h_max)
        command, duration = self._sample_command(k[6], h0)
        ref0 = jp.array([0.0, 0.0, h0, 0.0][: self.n_command])
        wait, push_duration, push_force = self._sample_push(k[7])
        history = jp.zeros((HISTORY_LENGTH, N_CHANNELS))
        history = latency.write(history, jp.int32(0), self._record(data, params, k[8]))
        zero_frame = jp.zeros((5, N_JOINTS))
        info = {
            "rng": k[9], "step": jp.int32(0), "physics_step": jp.int32(0), "params": params,
            "history": history, "command": command, "command_ref": ref0,
            "command_timer": duration, "push_timer": wait, "push_duration": push_duration,
            "push_force_sample": push_force, "push_force": jp.zeros(self.push_dim),
            "last_action": jp.zeros(N_JOINTS), "last_action2": jp.zeros(N_JOINTS),
            "frame": zero_frame, "axle_prev": self._axle_mid(data),
            "obs_history": jp.zeros((self.history, self.actor_layout.size)),
        }
        sensed = self._read_sensors(info)
        frame, parts = self._actor_frame(sensed, info)
        obs_history = jp.tile(frame[None], (self.history, 1))
        info["obs_history"] = obs_history
        truth = self._truth(data, info)
        obs = {"actor": obs_history.reshape(-1),
               "critic": self._critic_obs(parts, truth, info, model_params)}
        metrics = {name: jp.float32(0.0) for name in self.metric_names()}
        zero = jp.float32(0.0)
        return State(data=data, obs=obs, reward=zero, done=zero, terminated=zero, cat_delta=zero,
                     info=info, metrics=metrics)

    def _rotate_base(self, qpos, tilt, yaw):
        """Tilt and turn the whole robot about its axle midpoint (the keyframe's)."""
        q0 = qpos[3:7]
        half = 0.5 * jp.linalg.norm(tilt) + 1e-9
        axis = jp.array([tilt[0], tilt[1], 0.0]) / (2 * half)
        dq = jp.concatenate([jp.cos(half)[None], jp.sin(half) * axis])
        qyaw = jp.array([jp.cos(0.5 * yaw), 0.0, 0.0, jp.sin(0.5 * yaw)])
        rotation = _quat_mul(qyaw, dq)
        q = _quat_mul(rotation, q0)
        pivot = jp.asarray(self.stand_axle, dtype=jp.float32)
        position = pivot + _quat_to_mat(rotation) @ (qpos[0:3] - pivot)
        qpos = qpos.at[0:3].set(position)
        return qpos.at[3:7].set(q / jp.linalg.norm(q))

    def step(self, model: mjx.Model, state: State, action: jax.Array, model_params: jax.Array) -> State:
        info = dict(state.info)
        params = info["params"]
        rng, k_noise, k_cmd, k_push = jax.random.split(info["rng"], 4)
        action = jp.clip(action, -self.cfg["action"]["clip"], self.cfg["action"]["clip"])
        new_frame = self._frames(action, params)
        old_frame = info["frame"]
        delay = params["command_delay"]

        # Push: active while push_timer is in (-duration, 0].
        t_push = info["push_timer"]
        active = (t_push <= 0.0) & (t_push > -info["push_duration"])
        push_force = jp.where(active, info["push_force_sample"], 0.0)
        r_imu, r_body = self._body_frame(state.data)
        fwd = r_body[:, 0]
        heading = jp.array([fwd[0], fwd[1], 0.0]) / jp.maximum(jp.linalg.norm(fwd[:2]), 1e-6)
        lateral = jp.array([-heading[1], heading[0], 0.0])
        force = push_force[0] * heading + push_force[1] * lateral
        xfrc = jp.zeros_like(state.data.xfrc_applied)
        if self.push_dim == 5:
            force = force + push_force[2] * jp.array([0.0, 0.0, 1.0])
            torque = push_force[3] * heading + push_force[4] * jp.array([0.0, 0.0, 1.0])
            xfrc = xfrc.at[self.spec.torso_body, 3:6].set(torque)
        xfrc = xfrc.at[self.spec.torso_body, 0:3].set(force)

        noise_keys = jax.random.split(k_noise, self.decimation)

        def substep(carry, inputs):
            data, history, pstep = carry
            j, key = inputs
            due = j.astype(jp.float32) * self.dt_physics >= delay - 1e-9
            frame = jp.where(due, new_frame, old_frame)
            tau = self._mit_torque(frame, data, params) * params["torque_scale"]
            ctrl = jp.zeros(self.mj_model.nu).at[self.act_ids].set(tau)
            data = data.replace(ctrl=ctrl, xfrc_applied=xfrc)
            data = mjx.step(model, data)
            pstep = pstep + 1
            history = latency.write(history, pstep, self._record(data, params, key))
            return (data, history, pstep), tau

        (data, history, pstep), taus = jax.lax.scan(
            substep, (state.data, info["history"], info["physics_step"]),
            (jp.arange(self.decimation), noise_keys))
        info["history"] = history
        info["physics_step"] = pstep
        info["frame"] = new_frame
        info["step"] = info["step"] + 1
        info["push_force"] = push_force
        info["push_timer"] = info["push_timer"] - self.dt
        wait, duration, force_sample = self._sample_push(k_push)
        expired = info["push_timer"] <= -info["push_duration"]
        info["push_timer"] = jp.where(expired, wait, info["push_timer"])
        info["push_duration"] = jp.where(expired, duration, info["push_duration"])
        info["push_force_sample"] = jp.where(expired, force_sample, info["push_force_sample"])

        # Commands and the WBC's shaping of them.
        info["command_timer"] = info["command_timer"] - self.dt
        new_command, new_duration = self._sample_command(k_cmd, info["command"][2])
        resample = info["command_timer"] <= 0.0
        info["command"] = jp.where(resample, new_command, info["command"])
        info["command_timer"] = jp.where(resample, new_duration, info["command_timer"])
        ref = info["command_ref"]
        cmd = info["command"]
        v_ref = ref[0] + jp.clip(cmd[0] - ref[0], -self.accel_limit * self.dt, self.accel_limit * self.dt)
        w_ref = ref[1] + (cmd[1] - ref[1]) * (1.0 - math.exp(-self.dt / self.yaw_tau))
        h_ref = ref[2] + jp.clip(cmd[2] - ref[2], -self.h_rate * self.dt, self.h_rate * self.dt)
        if self.has_roll:
            roll_cmd = self.feasible_roll(cmd[3], h_ref)
            roll_ref = ref[3] + jp.clip(roll_cmd - ref[3], -self.roll_rate * self.dt, self.roll_rate * self.dt)
            info["command_ref"] = jp.array([v_ref, w_ref, h_ref, roll_ref])
        else:
            info["command_ref"] = jp.array([v_ref, w_ref, h_ref])

        truth = self._truth(data, info)
        info["axle_prev"] = truth["axle"]
        sensed = self._read_sensors(info)
        info["last_action2"] = info["last_action"]
        prev_action = info["last_action"]
        info["last_action"] = action
        frame_obs, parts = self._actor_frame(sensed, info)
        obs_history = jp.concatenate([frame_obs[None], info["obs_history"][:-1]], axis=0)
        info["obs_history"] = obs_history
        info["rng"] = rng

        # A diverged world must not reach the rollout buffer or the observation
        # normaliser with NaNs: one of them poisons all worlds.
        max_qvel = self.cfg["termination"].get("max_qvel", 200.0)
        diverged = ~(jp.all(jp.isfinite(data.qpos)) & jp.all(jp.isfinite(data.qvel))
                     & (jp.max(jp.abs(data.qvel)) < max_qvel))
        terminated = jp.maximum(self._terminated(truth), diverged.astype(jp.float32))
        cat_delta = self._constraints(truth, taus) if self.cfg["constraints"].get("enabled", True) else jp.float32(0)
        reward, terms = self._reward(truth, info, action, prev_action, info["last_action2"], taus, terminated)
        timeout = info["step"] >= self.episode_steps
        done = jp.maximum(terminated, timeout.astype(jp.float32))
        obs = {"actor": obs_history.reshape(-1),
               "critic": self._critic_obs(parts, truth, info, model_params)}
        obs = jax.tree.map(lambda x: jp.where(diverged, 0.0, x), obs)
        termination_penalty = self.cfg["rewards"]["weights"].get("termination", 0.0) * self.dt
        reward = jp.where(diverged | ~jp.isfinite(reward), termination_penalty, reward)
        terms = {k: jp.where(diverged, 0.0, v) for k, v in terms.items()}
        cat_delta = jp.where(diverged, 0.0, cat_delta)
        info["obs_history"] = jp.where(diverged, 0.0, info["obs_history"])
        metrics = dict(terms)
        metrics.update({
            "err_forward": jp.abs(truth["forward_speed"] - v_ref),
            "err_yaw": jp.abs(truth["yaw_rate"] - w_ref),
            "err_height": jp.abs(jp.mean(truth["heights"]) - h_ref),
            "cat_delta": cat_delta, "tilt": truth["tilt"],
            "err_roll": jp.abs(truth["roll"] - info["command_ref"][3]) if self.has_roll else jp.float32(0.0),
        })
        metrics = {k: jp.where(diverged, 0.0, v) for k, v in metrics.items()}
        return State(data=data, obs=obs, reward=reward, done=done, terminated=terminated,
                     cat_delta=cat_delta, info=info, metrics=metrics)

    def metric_names(self):
        return list(self.cfg["rewards"]["weights"]) + ["err_forward", "err_yaw", "err_height", "cat_delta", "tilt",
                                                       "err_roll"]

    # ------------------------------------------------------------ task terms
    def _terminated(self, truth):
        t = self.cfg["termination"]
        bad = (truth["tilt"] > t["tilt_rad"]) | (truth["imu_height"] < t["imu_height_m"])
        bad = bad | ~jp.all(jp.isfinite(truth["q"]))
        return bad.astype(jp.float32)

    def _constraints(self, truth, taus):
        c = self.cfg["constraints"]
        legs = self.leg_idx
        tau_leg = jp.max(jp.abs(taus[:, legs]), axis=0)
        violations = [
            jp.max(tau_leg - (self.cat_torque_limit - 0.05)) / c["leg_torque"]["slack"],
            jp.max(truth["stretch"] - c["stretch"]["limit"]) / c["stretch"]["slack"],
            jp.max(jp.maximum(self.q_lo + c["joint_range"]["margin_rad"] - truth["q"][legs],
                              truth["q"][legs] - (self.q_hi - c["joint_range"]["margin_rad"])))
            / c["joint_range"]["slack"],
            jp.max(jp.maximum(self.leg_h_lo - c["height"]["margin_m"] - truth["heights"],
                              truth["heights"] - (self.leg_h_hi + c["height"]["margin_m"]))) / c["height"]["slack"],
            (truth["tilt"] - c["tilt"]["limit_rad"]) / c["tilt"]["slack"],
        ]
        if "wheel_lift" in c and c["wheel_lift"]:
            violations.append(jp.max(truth["clearance"] - c["wheel_lift"]["limit_m"]) / c["wheel_lift"]["slack"])
        v = jp.clip(jp.stack(violations), 0.0, 1.0)
        return c["p_max"] * jp.max(v)

    def _reward(self, truth, info, action, prev_action, prev_action2, taus, terminated):
        r = self.cfg["rewards"]
        w, sig = r["weights"], r["sigma"]
        ref = info["command_ref"]
        g = 9.81
        lean = jp.arctan(ref[0] * ref[1] / g)
        # Commanded roll plus the coordinated-turn lean (roll is positive with
        # the left side up, so a left turn leans to -lean).
        roll_target = (ref[3] if self.has_roll else 0.0) - lean
        legs, wheels = self.leg_idx, self.wheel_idx
        terms = {
            "alive": jp.float32(1.0),
            "track_forward": jp.exp(-((truth["forward_speed"] - ref[0]) / sig["forward"]) ** 2),
            "track_yaw": jp.exp(-((truth["yaw_rate"] - ref[1]) / sig["yaw"]) ** 2),
            "track_height": jp.exp(-((jp.mean(truth["heights"]) - ref[2]) / sig["height"]) ** 2),
            "roll": (truth["roll"] - roll_target) ** 2,
            "track_roll": jp.exp(-((truth["roll"] - roll_target) / sig.get("roll", 0.05)) ** 2),
            "pitch": truth["pitch"] ** 2,
            "pitch_rate": truth["pitch_rate"] ** 2,
            "vertical_velocity": truth["vertical_speed"] ** 2,
            "leg_symmetry": (truth["offsets"][0] - truth["offsets"][1]) ** 2,
            "action_rate": jp.sum((action - prev_action) ** 2),
            "action_accel": jp.sum((action - 2 * prev_action + prev_action2) ** 2),
            "leg_torque": jp.mean(jp.sum(taus[:, legs] ** 2, axis=1)),
            "wheel_torque": jp.mean(jp.sum(taus[:, wheels] ** 2, axis=1)),
            "leg_velocity": jp.sum(truth["qd"][legs] ** 2),
            "termination": terminated,
        }
        terms = {k: v for k, v in terms.items() if k in w}
        scale = self.dt if r.get("scale_by_dt", True) else 1.0
        weighted = {k: w[k] * v * scale for k, v in terms.items()}
        reward = sum(weighted.values())
        return reward, weighted


def _quat_mul(a, b):
    w1, x1, y1, z1 = a[0], a[1], a[2], a[3]
    w2, x2, y2, z2 = b[0], b[1], b[2], b[3]
    return jp.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
