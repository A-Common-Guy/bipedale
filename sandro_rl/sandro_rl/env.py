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

import copy
import math
from dataclasses import dataclass
from pathlib import Path
from functools import partial
from typing import Any

import jax
import jax.numpy as jp
import mujoco
import numpy as np
from flax import struct
from mujoco import mjx

from . import kinematics, latency, terrain, torque_envelope
from .interface import (PROFILE_SCALE, goal_observation, profile_known, profile_observation, profile_occluded,
                        profile_shadow, profile_travel, ray_angles, ray_observation)
from .robot import RobotSpec, rotation_x
from .sources import JOINTS

N_JOINTS = 6
IMU_CHANNELS = 10           # quaternion wxyz, gyro, accelerometer
N_CHANNELS = IMU_CHANNELS + 2 * N_JOINTS
EVENT_TERMS = ("termination",)  # reward terms paid once per event, not per second
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
        self.leg_boxes = bool((cfg.get("collision") or {}).get("leg_boxes", False))
        model = terrain.build(base_spec, cfg.get("terrain", {}), leg_boxes=self.leg_boxes)
        self.spec = RobotSpec.load(base_spec.scene, model=model) if model is not base_spec.model else base_spec
        s = self.spec
        lim = s.limits
        self.mj_model = s.model
        # The leg links' collision boxes (collision.leg_boxes), for their height above the floor.
        self.link_boxes = np.array([g for g in range(s.model.ngeom)
                                    if (s.model.geom(g).name or "").endswith("_link_box")], dtype=int)
        self.link_half = jp.asarray(s.model.geom_size[self.link_boxes], dtype=jp.float32)
        self.impl = cfg.get("impl", "warp")
        graph_mode = None
        if self.impl == "warp" and cfg.get("graph_mode"):
            import warp as wp
            graph_mode = getattr(wp.JaxCallableGraphMode, cfg["graph_mode"].upper())
        physics = s.model
        if self.impl == "jax":
            # MJX's JAX backend cannot collide the tyre cylinders with meshes
            # or boxes (put_model raises). For debugging runs on it (the CPU),
            # only the tyres and the floor collide, as in bench.py; obstacles
            # still reach the rays, clearances and ground heights (analytic).
            physics = copy.deepcopy(s.model)
            keep = {*s.tyre_geoms.tolist(), s.floor_geom}
            off = [g for g in range(physics.ngeom) if g not in keep]
            physics.geom_contype[off] = 0
            physics.geom_conaffinity[off] = 0
            self.mj_model = physics       # make_data sizes the contacts from it
        self.mjx_model = mjx.put_model(physics, impl=self.impl, graph_mode=graph_mode)
        self.dt_physics = s.physics_dt
        control_steps = s.steps_per_control
        control_rate = lim.control_rate_hz
        cycles = control_rate / float(cfg["policy_rate_hz"])
        if abs(cycles - round(cycles)) > 1e-9:
            raise ValueError(f"policy rate {cfg['policy_rate_hz']} Hz must divide the {control_rate:g} Hz control rate")
        self.decimation = int(round(cycles)) * control_steps
        self.dt = self.decimation * self.dt_physics
        self.difference_scale = 0.01 / self.dt
        self.gear_substeps = int(((cfg["randomization"].get("drives") or {}).get("gear") or {}).get("substeps", 8))
        self.event_scale = 0.01 if cfg["rewards"].get("scale_by_dt", True) else 1.0
        self.episode_steps = int(round(cfg["episode_s"] / self.dt))

        # Indices and constants.
        self.qadr = jp.array(s.joint_qpos)
        self.dadr = jp.array(s.joint_dof)
        self.act_ids = jp.array(s.joint_actuator)
        self.leg_idx = s.leg_index
        self.wheel_idx = s.wheel_index
        self.wheel_armature = np.asarray(s.model.dof_armature[np.asarray(s.joint_dof)[s.wheel_index]], np.float32)
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
        self.imu_offset_correction = sources.simulated_imu_roll_offset() if offset == "ros" else float(offset)

        a = cfg["action"]
        self.leg_kp = float(_ros(a["legs"]["kp"], lim.leg_kp_nm_rad))
        self.leg_kd = float(_ros(a["legs"]["kd"], lim.leg_kd_nm_s_rad))
        self.leg_scale = float(a["legs"]["scale_rad"])
        self.wheel_mode = a["wheels"]["mode"]
        # "ros": the drives' envelope (its peak, folding back with heat as on
        # the robot); a number caps it further.
        self.envelope = s.envelope
        self.thermal = bool(a.get("torque_envelope", True))
        self.differential_filter_hz = float(a.get("wheel_differential_filter_hz", 0.0))
        tl = a.get("torque_limit_nm", {})
        self.leg_torque_nm = float(_ros(tl.get("legs", "ros"), self.envelope.peak_nm))
        self.wheel_torque_nm = float(_ros(tl.get("wheels", "ros"), self.envelope.peak_nm))
        self.torque_limit = jp.array([self.leg_torque_nm if j not in ("right_wheel", "left_wheel")
                                      else self.wheel_torque_nm for j in JOINTS], dtype=jp.float32)
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
        # Optional: the forward speed that is paid for shrinks with the yaw rate, so the faster
        # wheel stays under this rim speed (the drives' speed limit). The policy still sees
        # the command as it comes; it learns which part of it to follow.
        self.rim_speed = float(c.get("wheel_speed_m_s", 0.0))
        # Optional: the roll channel commands the legs' fore-aft split instead of the body's
        # roll (commands.split; roll_rad is then the largest split in m and roll_rate_rad_s
        # its rate in m/s, so the deployment controller shapes it as it shapes a roll). The
        # body is held level. Part of the commands are sinusoids: a stride.
        self.split = c.get("split") or {}
        self.has_split = bool(self.split)
        if self.has_split and not self.has_roll:
            raise ValueError("commands.split needs commands.roll_rad, the largest split in m")
        # Optional: the roll channel commands the body's pitch instead (commands.lean;
        # roll_rad is then the largest lean in rad, forward positive, and roll_rate_rad_s its
        # rate, both times lean.gain: the controller clips a roll to what the leg heights
        # allow, so the channel stays inside that and the gain makes the lean of it).
        # The legs carry the axles under the centre of mass; the roll stays level.
        # Part of the commands are sinusoids: a nod.
        self.lean = c.get("lean") or {}
        self.has_lean = bool(self.lean)
        if self.has_lean and (self.has_split or not self.has_roll):
            raise ValueError("commands.lean needs commands.roll_rad, the largest lean in rad, and no commands.split")
        self.wave = self.split or self.lean
        # Optional jump events: every interval_s a jump of a sampled wheel
        # clearance is requested; the policy sees its target and a countdown.
        self.jump = c.get("jump") or {}
        self.has_jump = bool(self.jump.get("enabled", bool(self.jump)))
        self.jump_h_max = float(self.jump.get("height_m", [0.0, 0.1])[1])
        # A flip in the jump window (simulation only): a whole turn about the pitch axis,
        # direction +1 nose down first. The tilt limits are off from the request to landing_s
        # after the window; the torso near the floor ends the episode instead.
        self.flip = self.jump.get("flip") or {}
        self.has_flip = bool(self.flip)
        torso_geoms = [g for g in range(s.model.ngeom) if s.model.geom_bodyid[g] == s.torso_body
                       and (s.model.geom_contype[g] or s.model.geom_conaffinity[g])]
        self.torso_geoms = np.array(torso_geoms if self.has_flip else [], dtype=int)
        self.torso_centre = jp.asarray(s.model.geom_aabb[self.torso_geoms, :3], dtype=jp.float32)
        self.torso_half = jp.asarray(s.model.geom_aabb[self.torso_geoms, 3:], dtype=jp.float32)
        # Optional goal commands (avoid task): reach a point at a commanded
        # cruise speed, instead of tracking a forward speed and a yaw rate.
        self.goals = c.get("goals") or {}
        self.has_goal = bool(self.goals.get("enabled", bool(self.goals)))
        # Optional range scanner: rays in the body's horizontal plane from a
        # point on the torso, against the obstacles and the floor.
        self.rays = cfg["observation"].get("rays") or {}
        self.n_rays = int(self.rays.get("count", 0))
        if self.n_rays:
            self.ray_angles = jp.asarray(ray_angles(self.n_rays, float(self.rays.get("fov_deg", 360.0))),
                                         dtype=jp.float32)
            self.ray_range = float(self.rays["max_range_m"])
            self.ray_offset = jp.asarray(self.rays.get("offset_m", [0.0, 0.0, 0.0]), dtype=jp.float32)
        # Optional terrain profile (sandro_perception's /terrain_profile): the floor height
        # ahead of each wheel from the depth camera's map.
        self.profile = cfg["observation"].get("profile") or {}
        self.profile_distances = jp.asarray(self.profile.get("distances_m", []), dtype=jp.float32)
        self.n_profile = 4 * int(self.profile_distances.shape[0])
        # Where the terrain along a track is sampled for what hides the floor from the camera.
        self.profile_line = jp.linspace(0.14, 1.34, 25)
        # Optional guard (rewards.guard): the forward speed the reward asks for falls to zero
        # in front of terrain taller than block_m, whatever the command says. Only a policy
        # that sees the terrain (the profile) can follow it.
        self.guard = cfg["rewards"].get("guard") or {}
        self.guard_line = jp.linspace(-0.02, 1.38, 29)    # from under the tyre's front on
        # The robot's footprint for pillar clearances, a box around the axle midpoint.
        self.hull_half = jp.asarray((cfg.get("avoidance") or {}).get("hull_half_m", [0.125, 0.17]),
                                    dtype=jp.float32)
        self.has_pillars = any(b["kind"] == "pillar" for b in terrain.boxes(cfg.get("terrain", {})))
        p = cfg["pushes"]
        self.push_dim = 5 if ("vertical_force_n" in p or "torque_nm" in p) else 2

        ct = cfg["constraints"]
        self.cat_torque_limit = jp.array(
            [float(_ros(ct["leg_torque"]["limit"], self.envelope.peak_nm))] * 4, dtype=jp.float32)

        self.n_actions = N_JOINTS
        self.batch_size = 1   # set by the caller before vmapping reset (contact buffer size)
        self.actor_layout = self._actor_layout()
        self.critic_layout = self._critic_layout()
        self.history = int(cfg["observation"]["history"])
        # Policy steps between stacked frames: a 300 Hz policy with stride 3
        # sees the same 10 ms-spaced history as a 100 Hz one.
        self.history_stride = int(cfg["observation"].get("history_stride", 1))
        self.history_frames = (self.history - 1) * self.history_stride + 1
        self.actor_obs_size = self.actor_layout.size * self.history
        self.critic_obs_size = self.critic_layout.size
        self.action_perm = s.mirror_perm
        self.payload_body = mujoco.mj_name2id(s.model, mujoco.mjtObj.mjOBJ_BODY, "identified_payload")
        self.terrain = terrain.Terrain(self.spec, cfg.get("terrain", {}))
        d = mujoco.MjData(s.model)
        d.qpos[:] = s.stand_qpos
        mujoco.mj_kinematics(s.model, d)
        self.stand_axle = d.xanchor[s.wheel_joint_ids].mean(axis=0)
        self.stand_imu_height = float(d.site_xpos[s.imu_site][2])
        # Optional switched-off starts (stand-up task): limp rest poses from fallen.py.
        self.fallen = cfg["init"].get("fallen") or {}
        self.fallen_states = None
        if self.fallen:
            from . import fallen
            path = Path(self.fallen.get("states", fallen.DEFAULT_PATH))
            if not path.is_absolute():
                path = Path(__file__).resolve().parent.parent / path
            self.fallen_states = jp.array(fallen.load(path, nq=s.model.nq), dtype=jp.float32)

    # ------------------------------------------------------------ layouts
    def _component_mirror(self, name: str, size: int):
        perm6 = self.spec.mirror_perm
        legs = list(self.leg_idx)
        leg_perm = np.array([legs.index(perm6[j]) for j in legs])
        wheel_perm = np.array([1, 0])
        swap_tracks = lambda blocks, order: (np.arange(size).reshape(blocks, -1)[order].reshape(-1)
                                             if size % blocks == 0 else np.arange(size))
        table = {
            "gravity": (np.arange(3), np.array([1, -1, 1])),
            "gyro": (np.arange(3), np.array([-1, 1, -1])),
            "accel": (np.arange(3), np.array([1, -1, 1])),
            "leg_pos": (leg_perm, np.ones(4)),
            "leg_vel": (leg_perm, np.ones(4)),
            "wheel_vel": (wheel_perm, np.ones(2)),
            "last_action": (perm6, np.ones(6)),
            # The fourth is a roll or a split, which the mirror reverses, or a lean, which it keeps.
            "command": (np.arange(size), np.array([1, -1, 1, 1 if self.has_lean else -1])[:size]),
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
            "jump": (np.arange(size), np.ones(size)),
            "params": (np.arange(size), np.ones(size)),
            # (distance, cos, sin of the bearing); the rays are symmetric
            # about the forward axis (interface.ray_angles), so they reverse.
            "goal": (np.arange(3), np.array([1, 1, -1])),
            "true_goal": (np.arange(3), np.array([1, 1, -1])),
            # Right track, left track, and their known flags: the tracks swap.
            "profile": (swap_tracks(4, [1, 0, 3, 2]), np.ones(size)),
            "true_profile": (swap_tracks(2, [1, 0]), np.ones(size)),
            "rays": (np.arange(size)[::-1], np.ones(size)),
            "true_rays": (np.arange(size)[::-1], np.ones(size)),
            "clearance": (np.arange(1), np.ones(1)),
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
        if self.has_jump:
            comps.append(("jump", 2))
        comps += self._exteroceptive_components()
        return self._layout(comps)

    def _exteroceptive_components(self):
        return (([("goal", 3)] if self.has_goal else []) + ([("rays", self.n_rays)] if self.n_rays else [])
                + ([("profile", self.n_profile)] if self.n_profile else []))

    N_PARAMS = 8 + 8   # episode parameters + randomize.MODEL_PARAMS

    def _critic_layout(self):
        comps = [("gravity", 3), ("gyro", 3), ("leg_pos", 4), ("leg_vel", 4), ("wheel_vel", 2),
                 ("last_action", 6), ("command", self.n_command)]
        if self.has_jump:
            comps.append(("jump", 2))
        comps += self._exteroceptive_components()
        comps += [("true_gravity", 3), ("true_gyro", 3), ("base_velocity", 3), ("true_leg_pos", 4),
                 ("true_leg_vel", 4), ("true_wheel_vel", 2), ("leg_height", 2), ("leg_offset", 2),
                 ("wheel_clearance", 2), ("push", self.push_dim), ("params", self.N_PARAMS)]
        if self.has_goal:
            comps.append(("true_goal", 3))
        if self.n_rays:
            comps.append(("true_rays", self.n_rays))
        if self.has_pillars:
            comps.append(("clearance", 1))
        if self.n_profile:
            comps.append(("true_profile", self.n_profile // 2))
        return self._layout(comps)

    def blind_indices(self):
        """(actor, critic) observation indices of everything but the terrain profile (and the
        critic's clearance to blocks, which comes with the guard task): where a policy trained
        without them has its inputs (train.py --resume)."""
        def keep(layout, repeat):
            offsets = np.concatenate([[0], np.cumsum(layout.sizes)])
            frame = np.concatenate([np.arange(offsets[i], offsets[i + 1]) for i, name in enumerate(layout.names)
                                    if name not in ("profile", "true_profile", "clearance")])
            return np.concatenate([frame + k * layout.size for k in range(repeat)])
        return keep(self.actor_layout, self.history), keep(self.critic_layout, 1)

    def profile_statistics(self):
        """(mean, variance) of the actor and of the critic observation where the terrain
        profile sits (0 and 1 elsewhere): heights of a few cm, flags half set. A resumed
        normaliser has counted too many samples to learn them."""
        def stats(layout, repeat):
            mean, var = np.zeros(layout.size, np.float32), np.ones(layout.size, np.float32)
            offsets = np.concatenate([[0], np.cumsum(layout.sizes)])
            for i, name in enumerate(layout.names):
                if name in ("profile", "true_profile"):
                    heights = layout.sizes[i] // 2 if name == "profile" else layout.sizes[i]
                    var[offsets[i]:offsets[i] + heights] = (0.02 * PROFILE_SCALE) ** 2
                    mean[offsets[i] + heights:offsets[i + 1]] = 0.5
                    var[offsets[i] + heights:offsets[i + 1]] = 0.25
            return np.tile(mean, repeat), np.tile(var, repeat)
        return stats(self.actor_layout, self.history), stats(self.critic_layout, 1)

    def _actor_obs(self, obs_history):
        """Every history_stride-th frame of the buffer, newest first, flattened."""
        return obs_history[:: self.history_stride].reshape(-1)

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
            imu_rate = u(keys[3], L["imu_rate_hz"])
            joint_delay = u(keys[4], L["joint_delay_s"])
            joint_rate = u(keys[5], L["joint_rate_hz"])
            sync = jax.random.uniform(keys[6]) < L["synchronous_joints_probability"]
            # state_source:=mit: every drive replies to each command, so the
            # joints are sampled together at the control rate.
            mit = jax.random.uniform(jax.random.fold_in(keys[6], 1)) < L.get("mit_joints_probability", 0.0)
            mit_delay = u(jax.random.fold_in(keys[4], 1), L.get("mit_joint_delay_s", [0.0023, 0.0023]))
            joint_delay = jp.where(mit, mit_delay, joint_delay)
            joint_rate = jp.where(mit, jp.float32(self.spec.limits.control_rate_hz), joint_rate)
            sync = sync | mit
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
            wheel_lag = u(keys[15], D.get("wheel_lag_s", [0.0, 0.0]))
            gear = self._sample_gear(jax.random.fold_in(keys[15], 1), D.get("gear") or {})
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
            wheel_lag = jp.float32(r["drives"].get("nominal_wheel_lag_s", 0.0))
            gear = self._sample_gear(None, {})
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
            # The gear's spring is the drive train's lag where it is modelled.
            "wheel_lag": jp.where(gear["gear_on"] > 0, 0.0, wheel_lag).astype(jp.float32),
            "imu_rate": jp.float32(imu_rate), "imu_delay": jp.float32(imu_delay),
            **gear,
        }

    def _sample_gear(self, rng, g):
        """The wheel drive trains' gear (Gear in evaluate.py): rigid, or per wheel a
        rotor behind play and a spring, in g["probability"] of the episodes.
        Stiffness log-uniform; rotor inertia a multiple of the MJCF armature."""
        if rng is None or not g.get("probability", 0.0):
            return {"gear_on": jp.float32(0.0), "gear_k": jp.full(2, 1000.0, jp.float32),
                    "gear_half_play": jp.zeros(2, jp.float32), "gear_c": jp.full(2, 0.02, jp.float32),
                    "rotor_j": jp.asarray(self.wheel_armature, jp.float32)}
        k = jax.random.split(rng, 5)
        u = lambda key, lo_hi: jax.random.uniform(key, (2,), minval=lo_hi[0], maxval=lo_hi[1])
        lo, hi = g["stiffness_nm_rad"]
        return {
            "gear_on": (jax.random.uniform(k[0]) < g["probability"]).astype(jp.float32),
            "gear_k": jp.exp(u(k[1], [math.log(lo), math.log(hi)])).astype(jp.float32),
            "gear_half_play": (0.5 * u(k[2], g.get("play_rad", [0.0, 0.0]))).astype(jp.float32),
            "gear_c": u(k[3], g.get("damping_nm_s_rad", [0.02, 0.02])).astype(jp.float32),
            "rotor_j": (jp.asarray(self.wheel_armature) * u(k[4], g.get("rotor_inertia_scale", [1.0, 1.0])))
                       .astype(jp.float32),
        }

    def _sample_command(self, rng, previous_height):
        c = self.cfg["commands"]
        k = jax.random.split(rng, 10)
        # Curriculum stages narrow the sampled range; the observation keeps
        # normalising by the full range, so the policy's inputs keep their meaning.
        fraction = float(c.get("sample_fraction", 1.0))
        if self.has_goal:
            # The operator sets a cruise speed; the goal gives the direction.
            lo, hi = self.goals["speed_m_s"]
            v, w = jax.random.uniform(k[0], minval=lo, maxval=hi), jp.float32(0.0)
        else:
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

    def feasible_split(self, split, height):
        """Clip a split to what both legs reach at this height: each axle half of it from the
        neutral offset, the leg at most split.stretch of its length from its farther pivot."""
        g = self.geometry["right"]
        radius = float(self.split.get("stretch", 0.9)) * (g["upper_m"] + g["lower_m"])
        limit = 2.0 * jp.maximum(jp.sqrt(jp.maximum(radius ** 2 - height ** 2, 0.0)) - 0.5 * g["base_m"], 0.0)
        return jp.clip(split, -limit, limit)

    def feasible_lean(self, lean, height):
        """Clip a lean to what the legs can balance at this height: the axles under the centre
        of mass, which sits lean.com_below_pivots_m under the leg pivots, each leg at most
        lean.stretch of its length from its farther pivot."""
        g = self.geometry["right"]
        radius = float(self.lean.get("stretch", 0.9)) * (g["upper_m"] + g["lower_m"])
        reach = jp.maximum(jp.sqrt(jp.maximum(radius ** 2 - height ** 2, 0.0)) - 0.5 * g["base_m"], 0.0)
        limit = jp.arctan(reach / jp.maximum(height - float(self.lean.get("com_below_pivots_m", 0.0)), 0.02))
        return jp.clip(lean, -limit, limit)

    def _sample_split_wave(self, rng):
        """(amplitude, rad/s, phase) of a stride or a nod; amplitude 0 leaves the sampled command."""
        k = jax.random.split(rng, 4)
        lo, hi = self.wave["wave_hz"]
        least = self.wave.get("wave_min_m", self.wave.get("wave_min_rad", 0.03))
        amplitude = jax.random.uniform(k[0], minval=least, maxval=self.wave.get("wave_max", self.roll_max))
        amplitude = jp.where(jax.random.uniform(k[1]) < self.wave["wave_probability"], amplitude, 0.0)
        return jp.array([amplitude, 2.0 * math.pi * jax.random.uniform(k[2], minval=lo, maxval=hi),
                         jax.random.uniform(k[3], minval=0.0, maxval=2.0 * math.pi)])

    def _sample_jump(self, rng):
        """Wait until the next jump request and its target wheel clearance."""
        j = self.jump
        k = jax.random.split(rng, 2)
        wait = jax.random.uniform(k[0], minval=j["interval_s"][0], maxval=j["interval_s"][1])
        height = jax.random.uniform(k[1], minval=j["height_m"][0], maxval=j["height_m"][1])
        return wait, height

    def jump_observation(self, timer, height):
        """(target clearance / max, remaining fraction of the window), zero outside a jump."""
        active = timer > 0.0
        return jp.where(active, jp.array([height / self.jump_h_max, timer / self.jump["window_s"]]), 0.0)

    def _goal_free(self, point, data):
        """Clearance of a robot footprint (any heading) centred on a point."""
        half = jp.full(2, jp.max(self.hull_half))
        return self.terrain.clearance(point, jp.array([1.0, 0.0]), half, data)

    def _sample_goal(self, rng, origin, data):
        """A goal distance_m from origin in any direction, the first of four
        candidates that is clear of the pillars (else the first)."""
        g = self.goals
        k = jax.random.split(rng, 2)
        dist = jax.random.uniform(k[0], (4,), minval=g["distance_m"][0], maxval=g["distance_m"][1])
        angle = jax.random.uniform(k[1], (4,), minval=-math.pi, maxval=math.pi)
        candidates = origin[None] + dist[:, None] * jp.stack([jp.cos(angle), jp.sin(angle)], -1)
        if not self.has_pillars:
            return candidates[0]
        free = jax.vmap(lambda p: self._goal_free(p, data))(candidates) > g.get("free_m", 0.15)
        return candidates[jp.argmax(free)]

    def _goal_time(self, start, goal, cruise):
        """Time allowed for a goal: twice the straight-line time plus a margin."""
        g = self.goals
        return (g.get("time_factor", 2.0) * jp.linalg.norm(goal - start) / jp.maximum(cruise, 0.1)
                + g.get("time_margin_s", 3.0))

    def _goal_course(self, rng, data, pos, quat, yaw):
        """The episode's first goal, with `blockers` pillars put on the way to
        it, and every pillar on the start pose or the goal parked."""
        g = self.goals
        k = jax.random.split(rng, 3)
        start = jp.asarray(self.stand_axle[:2], dtype=jp.float32)
        goal = self._sample_goal(k[0], start, self.terrain.apply(data, pos, quat))
        idx = jp.asarray(self.terrain.pillar_items)
        blockers = min(int(g.get("blockers", 0)), len(self.terrain.pillar_items))
        if blockers:
            dist = jp.maximum(jp.linalg.norm(goal - start), 1e-6)
            along = (goal - start) / dist
            lo = jp.clip(g.get("blocker_start_m", 0.8) / dist, 0.3, 0.65)
            u = jax.random.uniform(k[1], (blockers,), minval=lo, maxval=0.7)
            side = jax.random.uniform(k[2], (blockers,), minval=-1, maxval=1) * g.get("blocker_offset_m", 0.2)
            xy = start[None] + (u * dist)[:, None] * along[None] + side[:, None] * jp.array([-along[1], along[0]])[None]
            pos = pos.at[idx[:blockers], 0:2].set(xy)
        placed = self.terrain.apply(data, pos, quat)
        heading = jp.array([jp.cos(yaw), jp.sin(yaw)])
        on_start = self.terrain.pillar_gaps(start, heading, self.hull_half, placed) < g.get("start_clearance_m", 0.2)
        on_goal = self.terrain.pillar_gaps(goal, jp.array([1.0, 0.0]), jp.full(2, jp.max(self.hull_half)),
                                           placed) < g.get("free_m", 0.15)
        pos = pos.at[idx, 2].set(jp.where(on_start | on_goal, terrain.PARK_Z, pos[idx, 2]))
        return pos, goal

    def _scan(self, data, r_body):
        """True range of every scanner ray, m."""
        origin = data.site_xpos[self.spec.imu_site] + r_body @ self.ray_offset
        local = jp.stack([jp.cos(self.ray_angles), jp.sin(self.ray_angles), jp.zeros_like(self.ray_angles)], -1)
        return self.terrain.ray_distance(origin, local @ r_body.T, data, self.ray_range)

    def _measure_rays(self, true, rng):
        """One scan as the robot reads it: range noise and dropouts (no return)."""
        r = self.rays
        k = jax.random.split(rng, 2)
        sigma = r.get("noise_m", 0.0) + r.get("noise_fraction", 0.0) * true
        measured = true + sigma * jax.random.normal(k[0], true.shape)
        measured = jp.where(jax.random.uniform(k[1], true.shape) < r.get("dropout", 0.0), self.ray_range, measured)
        return ray_observation(measured, self.ray_range, jp)

    def _profile_heights(self, data, shift=0.0, distances=None):
        """Floor height at the profile's distances (+ shift, m) ahead of each wheel, above the
        floor under that wheel: [right track, left track]."""
        distances = self.profile_distances if distances is None else distances
        _, r_body = self._body_frame(data)
        heading = jp.array([r_body[0, 0], r_body[1, 0], 0.0])
        heading = heading / jp.maximum(jp.linalg.norm(heading), 1e-6)
        anchors = data.xanchor[self.spec.wheel_joint_ids]
        ahead = (distances + shift)[None, :, None] * heading[None, None, :]
        points = (anchors[:, None, :] + ahead).reshape(-1, 3)
        ground = self.terrain.ground_height(jp.concatenate([anchors, points]), data)
        return (ground[2:].reshape(2, -1) - ground[:2, None]).reshape(-1)

    def _sample_profile(self, rng):
        """One episode's camera and map: where its view of the floor starts and ends, its
        calibration errors, its rate, and how much floor it has already seen."""
        c = self.profile
        k = jax.random.split(rng, 9)
        u = lambda key, lo_hi, shape=(): jax.random.uniform(key, shape, minval=lo_hi[0], maxval=lo_hi[1])
        sym = lambda key, half, shape=(): half * jax.random.uniform(key, shape, minval=-1.0, maxval=1.0)
        period = 1.0 / u(k[0], c.get("rate_hz", [12.0, 25.0]))
        seen = jax.random.uniform(k[7]) < c.get("seen_probability", 0.5)
        return {"profile_near": u(k[1], c.get("near_m", [0.3, 0.8])), "profile_far": u(k[2], c.get("far_m", [1.2, 2.5])),
                "profile_slope": sym(k[3], c.get("slope_rad", 0.015)), "profile_bias": sym(k[4], c.get("bias_m", 0.005), (2,)),
                "profile_shift": sym(k[5], c.get("position_error_m", 0.03)),
                "profile_lift": jax.random.uniform(jax.random.fold_in(k[5], 2)) * c.get("airborne_gain", 0.0),
                "profile_face": u(jax.random.fold_in(k[5], 1), c.get("face_m", [0.2, 0.3])),
                "profile_blind": (jax.random.uniform(k[6]) < c.get("blind_probability", 0.15)).astype(jp.float32),
                "profile_period": period, "profile_timer": jax.random.uniform(k[8]) * period,
                "profile_travel": jp.where(seen, jp.float32(c.get("travel_max_m", 2.0)), 0.0)}

    def _measure_profile(self, data, info, rng, clearance=None):
        """The profile as the node would publish it now: the map's heights with the mount's
        pitch error (a slope), a height bias per track, the map's misplacement along the
        track, noise and holes, and unknown where the camera has not seen the floor. The
        node takes the body's height from the legs with the wheels on the floor: off the
        floor by `clearance` [right, left], it reads everything lower by up to that much."""
        c = self.profile
        k = jax.random.split(rng, 3)
        n = self.n_profile // 4
        c_ahead = c.get("camera_ahead_m", 0.1)
        jitter = c.get("position_noise_m", 0.02) * jax.random.normal(k[0])
        shift = info["profile_shift"] + jitter
        both = self._profile_heights(data, shift, jp.concatenate([self.profile_distances, self.profile_line]))
        both = both.reshape(2, -1)
        heights, line = both[:, :n], both[:, n:]
        # The camera above the floor each wheel stands on, at the IMU's height.
        anchors = data.xanchor[self.spec.wheel_joint_ids]
        floor = self.terrain.ground_height(anchors, data)
        camera = data.site_xpos[self.spec.imu_site][2] - floor
        hidden = jax.vmap(lambda h, l, c: profile_occluded(self.profile_distances, h, self.profile_line, l, c,
                                                           c_ahead, jp))(heights, line, camera)
        # The node's shadow fill: an obstacle reads as one, capped at what a face averages.
        heights, filled = jax.vmap(lambda h, hid, l: profile_shadow(
            self.profile_distances, h, hid, self.profile_line, l, info["profile_face"], xp=jp))(heights, hidden, line)
        hidden = hidden * (1.0 - filled)
        heights = (heights + info["profile_slope"] * self.profile_distances[None]
                   + info["profile_bias"][:, None] + c.get("noise_m", 0.005) * jax.random.normal(k[1], (2, n)))
        if clearance is not None:
            heights = heights - info["profile_lift"] * jp.maximum(clearance, 0.0)[:, None]
        known = profile_known(self.profile_distances, info["profile_travel"], info["profile_near"],
                              info["profile_far"], jp)
        known = jp.tile(known[None], (2, 1)) * (1.0 - info["profile_blind"]) * (1.0 - hidden)
        known = known * (jax.random.uniform(k[2], (2, n)) >= c.get("dropout", 0.03))
        return profile_observation(heights.reshape(-1), known.reshape(-1), jp)

    def _goal_vector(self, goal, axle, heading):
        """(forward, left) of a goal from the axle midpoint, heading frame."""
        rel = goal - axle[:2]
        return rel @ heading, rel[1] * heading[0] - rel[0] * heading[1]

    def _goal_obs(self, data, info):
        """The goal as the robot's odometry places it (truth plus drift)."""
        _, r_body = self._body_frame(data)
        heading = r_body[:2, 0] / jp.maximum(jp.linalg.norm(r_body[:2, 0]), 1e-6)
        forward, left = self._goal_vector(info["goal"] + info["odo_error"], self._axle_mid(data), heading)
        return goal_observation(forward, left, self.goals["range_m"], jp)

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
    def _record(self, data, params, rng, rotor):
        q, v = self._encoder_state(data, params, rotor)
        raw = jp.concatenate([data.sensordata[self.sensor_index], q, v])
        return raw + params["noise"] * jax.random.normal(rng, (N_CHANNELS,))

    def _encoder_state(self, data, params, rotor):
        """Joint positions and velocities as the drives' motor-side encoders see them."""
        w, on = self.wheel_idx, params["gear_on"] > 0
        q = data.qpos[self.qadr]
        v = data.qvel[self.dadr]
        return (q.at[w].set(jp.where(on, rotor[0], q[w])), v.at[w].set(jp.where(on, rotor[1], v[w])))

    def _gear(self, frame, data, params, limit, rotor):
        """The wheel drives through their gears over one physics step: the MIT law
        on the rotor every substep (the drive's own loop runs far above the
        physics rate), the rotor against the spring beyond the play. The mean
        motor torque and the mean torque the gears pass to the wheels."""
        w = self.wheel_idx
        wq, wv = data.qpos[self.qadr[w]], data.qvel[self.dadr[w]]
        q_ref, v_ref, kp, kd, ff = (frame[i][w] for i in range(5))
        lim, scale = limit[w], params["torque_scale"][w]
        k, half, c, j = params["gear_k"], params["gear_half_play"], params["gear_c"], params["rotor_j"]
        h = self.dt_physics / self.gear_substeps
        rq, rv = rotor[0], rotor[1]
        motor = passed = jp.zeros(2)
        for _ in range(self.gear_substeps):
            tm = jp.clip(kp * (jp.where(kp > 0, q_ref, rq) - rq) + kd * (v_ref - rv) + ff, -lim, lim) * scale
            delta = rq - wq
            over = jp.sign(delta) * jp.maximum(jp.abs(delta) - half, 0.0)
            ts = k * over + jp.where(over != 0, c * (rv - wv), 0.0)
            rv = rv + (tm - ts) / j * h
            rq = rq + rv * h
            wq = wq + wv * h
            motor, passed = motor + tm, passed + ts
        return motor / self.gear_substeps, passed / self.gear_substeps, jp.stack([rq, rv])

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

    def _mit_torque(self, frame, data, params, limit):
        q = data.qpos[self.qadr]
        v = data.qvel[self.dadr]
        q_ref = jp.where(frame[2] > 0, frame[0], q)
        tau = frame[2] * (q_ref - q) + frame[3] * (frame[1] - v) + frame[4]
        tau = jp.clip(tau, -limit, limit)
        return tau

    def _torque_limit(self, heat):
        if not self.thermal:
            return self.torque_limit
        return jp.minimum(self.torque_limit, torque_envelope.limit(heat, self.envelope, jp))

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
        if self.has_jump:
            parts["jump"] = (self.jump_observation(info["jump_timer"], info["jump_height"])
                             * info.get("jump_shown", 1.0))
        if self.has_goal:
            parts["goal"] = info["goal_obs"]
        if self.n_rays:
            parts["rays"] = info["ray_obs"]
        if self.n_profile:
            parts["profile"] = info["profile_obs"]
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
        extra = {}
        if self.has_goal:
            forward, left = self._goal_vector(info["goal"], mid, heading[:2])
            extra["goal_vec"] = jp.array([forward, left])
            extra["goal_dist"] = jp.sqrt(forward ** 2 + left ** 2)
        if self.n_rays:
            extra["rays"] = self._scan(data, r_body)
        if len(self.link_boxes):
            # The lowest corner of any leg link above a flat floor.
            up = data.geom_xmat[self.link_boxes].reshape(-1, 3, 3)[:, 2, :]
            extra["link_clearance"] = jp.min(data.geom_xpos[self.link_boxes][:, 2]
                                             - jp.sum(jp.abs(up) * self.link_half, axis=1))
        if len(self.torso_geoms):
            # The lowest corner of the torso's bounding boxes above a flat floor.
            mats = data.geom_xmat[self.torso_geoms].reshape(-1, 3, 3)
            centre = data.geom_xpos[self.torso_geoms] + jp.einsum("gij,gj->gi", mats, self.torso_centre)
            extra["torso_clearance"] = jp.min(centre[:, 2] - jp.sum(jp.abs(mats[:, 2, :]) * self.torso_half, axis=1))
        if self.has_pillars:
            extra["obstacle_clearance"] = self.terrain.clearance(mid[:2], heading[:2], self.hull_half, data)
        if self.n_profile:
            extra["profile"] = self._profile_heights(data)
        if self.guard:
            # The nearest floor ahead of either wheel that is too tall to drive over.
            line = self._profile_heights(data, 0.0, self.guard_line).reshape(2, -1)
            if self.guard.get("face_run_m", 0.0) > 0.0:
                # A face, not a ramp: the floor rising by more than block_m within face_run_m.
                n = max(1, round(self.guard["face_run_m"] / 0.05))     # guard_line's step
                rise = line[:, n:] - line[:, :-n]
                blocked = jp.concatenate([jp.zeros(n, bool), jp.any(rise > self.guard["block_m"], axis=0)])
            else:
                blocked = jp.any(line > self.guard["block_m"], axis=0)
            extra["block_distance"] = jp.min(jp.where(blocked, self.guard_line, 10.0))
        return extra | {
            "r_body": r_body, "gravity": r_body.T @ jp.array([0.0, 0.0, -1.0]),
            "gyro": self.body_from_imu.T @ gyro_true,
            "forward_speed": vel @ heading, "lateral_speed": vel @ lateral, "vertical_speed": vel[2],
            "yaw_rate": omega_world[2], "pitch": pitch, "roll": roll,
            "pitch_rate": (self.body_from_imu.T @ gyro_true)[1],
            "tilt": jp.arccos(jp.clip(r_body[2, 2], -1.0, 1.0)),
            "q": q, "qd": qd, "heights": heights, "offsets": offsets, "stretch": stretch,
            "imu_height": data.site_xpos[self.spec.imu_site][2] - ground[2], "clearance": clearance, "axle": mid,
            "base_vz": data.qvel[2],   # the free joint's world linear velocity
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
        if self.has_goal:
            parts["true_goal"] = goal_observation(truth["goal_vec"][0], truth["goal_vec"][1],
                                                  self.goals["range_m"], jp)
        if self.n_rays:
            parts["true_rays"] = ray_observation(truth["rays"], self.ray_range, jp)
        if self.has_pillars:
            parts["clearance"] = jp.clip(truth["obstacle_clearance"], -0.1, 1.0)[None]
        if self.n_profile:
            parts["true_profile"] = profile_observation(truth["profile"], jp.ones(self.n_profile // 2),
                                                        jp)[: self.n_profile // 2]
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
        if self.fallen_states is not None:
            kf = jax.random.split(jax.random.fold_in(k[2], 3))
            pick = jax.random.randint(kf[0], (), 0, self.fallen_states.shape[0])
            off = jax.random.uniform(kf[1]) < self.fallen["fraction"]
            qpos = jp.where(off, self._rotate_base(self.fallen_states[pick], jp.zeros(2), yaw), qpos)
            qvel = jp.where(off, 0.0, qvel)
        data = data.replace(qpos=qpos, qvel=qvel, ctrl=jp.zeros(self.mj_model.nu))
        pos, quat = self.terrain.layout(k[3])
        if self.has_goal:
            pos, goal = self._goal_course(jax.random.fold_in(k[3], 1), data, pos, quat, yaw)
        data = self.terrain.apply(data, pos, quat)
        data = mjx.forward(model, data)
        params = self._sample_episode_params(k[5], randomize)
        h0 = jp.clip(jp.mean(kinematics.leg_state(qpos[self.qadr][self.leg_idx], self.geometry,
                                                  self.neutral, jp)[0]), self.h_min, self.h_max)
        command, duration = self._sample_command(k[6], h0)
        ref0 = jp.array([0.0, 0.0, h0, 0.0][: self.n_command])
        wait, push_duration, push_force = self._sample_push(k[7])
        # The rotors start where the wheels are, at rest against the play.
        rotor = jp.stack([data.qpos[self.qadr[self.wheel_idx]], data.qvel[self.dadr[self.wheel_idx]]])
        history = jp.zeros((HISTORY_LENGTH, N_CHANNELS))
        history = latency.write(history, jp.int32(0), self._record(data, params, k[8], rotor))
        zero_frame = jp.zeros((5, N_JOINTS))
        info = {
            "rng": k[9], "step": jp.int32(0), "physics_step": jp.int32(0), "params": params,
            "history": history, "command": command, "command_ref": ref0,
            "command_timer": duration, "push_timer": wait, "push_duration": push_duration,
            "push_force_sample": push_force, "push_force": jp.zeros(self.push_dim),
            "last_action": jp.zeros(N_JOINTS), "last_action2": jp.zeros(N_JOINTS),
            "wheel_differential": jp.float32(0.0),
            "frame": zero_frame, "drive_tau": jp.zeros(N_JOINTS), "axle_prev": self._axle_mid(data),
            "rotor": rotor,
            # As if the robot had been working: up to half the heat budget.
            "heat": jax.random.uniform(jax.random.fold_in(k[9], 3), (N_JOINTS,), maxval=0.5)
                    * self.envelope.rated_nm ** 2,
            "obs_history": jp.zeros((self.history_frames, self.actor_layout.size)),
        }
        if self.wave:
            info["split_wave"] = self._sample_split_wave(jax.random.fold_in(k[9], 5))
        if self.has_jump:
            jump_wait, jump_height = self._sample_jump(jax.random.fold_in(k[9], 1))
            info.update({"jump_wait": jump_wait, "jump_timer": jp.float32(-10.0), "jump_height": jump_height,
                         "jump_shown": jp.float32(1.0), "jump_air": jp.float32(0.0), "jump_done": jp.float32(0.0),
                         "jump_clearance": jp.float32(0.0), "flip_angle": jp.float32(0.0)})
        if self.has_goal:
            start = jp.asarray(self.stand_axle[:2], dtype=jp.float32)
            info.update({"goal": goal, "goal_timer": self._goal_time(start, goal, command[0]),
                         "goal_hold": jp.float32(0.0), "odo_error": jp.zeros(2)})
            info["goal_obs"] = self._goal_obs(data, info)
        if self.n_rays:
            kr = jax.random.split(jax.random.fold_in(k[9], 2), 3)
            period = 1.0 / jax.random.uniform(kr[0], minval=self.rays["rate_hz"][0], maxval=self.rays["rate_hz"][1])
            info.update({"ray_period": period, "ray_timer": jax.random.uniform(kr[1]) * period,
                         "ray_obs": self._measure_rays(self._scan(data, self._body_frame(data)[1]), kr[2])})
        if self.n_profile:
            info.update(self._sample_profile(jax.random.fold_in(k[9], 3)))
            info["profile_obs"] = self._measure_profile(data, info, jax.random.fold_in(k[9], 4))
            info["profile_pending"] = info["profile_obs"]
        sensed = self._read_sensors(info)
        frame, parts = self._actor_frame(sensed, info)
        obs_history = jp.tile(frame[None], (self.history_frames, 1))
        info["obs_history"] = obs_history
        truth = self._truth(data, info)
        obs = {"actor": self._actor_obs(obs_history),
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
        if self.has_jump:
            # Requested jumps: the window opens when the wait runs out.
            info["jump_timer"] = info["jump_timer"] - self.dt
            info["jump_wait"] = info["jump_wait"] - self.dt
            start = (info["jump_wait"] <= 0.0) & (info["jump_timer"] <= 0.0)
            wait, height = self._sample_jump(jax.random.fold_in(k_cmd, 7))
            edges = self.jump.get("at_edges")
            if edges:
                # Hop on command: a request whenever the ground ahead rises, as
                # an operator or a forward range sensor would send it.
                _, r_now = self._body_frame(state.data)
                fwd = jp.array([r_now[0, 0], r_now[1, 0], 0.0])
                fwd = fwd / jp.maximum(jp.linalg.norm(fwd), 1e-6)
                mid = self._axle_mid(state.data)
                ahead = jp.linspace(edges["lookahead_m"][0], edges["lookahead_m"][1], 4)
                points = jp.concatenate([mid[None], mid[None] + ahead[:, None] * fwd[None]])
                ground = self.terrain.ground_height(points, state.data)
                rise = jp.max(ground[1:]) - ground[0]
                # Above max_rise_m there is nothing to hop onto (rewards.guard stops there).
                edge = ((rise > edges["min_rise_m"]) & (rise <= edges.get("max_rise_m", 1.0))
                        & (info["jump_timer"] <= -self.jump.get("landing_s", 0.3)))
                start = start | edge
                height = jp.where(edge, jp.clip(rise + edges["margin_m"], *self.jump["height_m"]), height)
                # hidden: the window pays as a requested jump does, but the policy is not told;
                # it has to see the edge in the terrain profile.
                if edges.get("hidden", False):
                    info["jump_shown"] = jp.where(start, 1.0 - edge.astype(jp.float32), info["jump_shown"])
            if self.jump.get("single_hop", False):
                # The window stops paying at the first touchdown after a lift-off: one jump,
                # not as many hops as fit in it. The countdown the policy sees runs on, as the
                # controller's does, which cannot tell a landing.
                lifted = state.info["jump_clearance"] > self.jump.get("liftoff_m", 0.01)
                info["jump_air"] = jp.where(start, 0.0, jp.maximum(info["jump_air"], lifted.astype(jp.float32)))
                landed = (info["jump_air"] > 0.0) & (state.info["jump_clearance"] < 0.003)
                info["jump_done"] = jp.where(start, 0.0, jp.maximum(info["jump_done"], landed.astype(jp.float32)))
            info["flip_angle"] = jp.where(start, 0.0, info["flip_angle"])
            info["jump_timer"] = jp.where(start, self.jump["window_s"], info["jump_timer"])
            info["jump_height"] = jp.where(start, height, info["jump_height"])
            info["jump_wait"] = jp.where(start, wait + self.jump["window_s"], info["jump_wait"])
        action = jp.clip(action, -self.cfg["action"]["clip"], self.cfg["action"]["clip"])
        applied = action
        if self.differential_filter_hz > 0.0:
            # RlPolicyCore's action.wheel_differential_filter_hz: a first-order
            # low-pass on the opposed half of the wheel actions; the
            # observation keeps the policy's own action.
            r, l = self.wheel_idx
            blend = 1.0 - math.exp(-2.0 * math.pi * self.differential_filter_hz * self.dt)
            info["wheel_differential"] = info["wheel_differential"] + blend * (
                0.5 * (action[r] - action[l]) - info["wheel_differential"])
            common = 0.5 * (action[r] + action[l])
            applied = action.at[r].set(common + info["wheel_differential"]).at[l].set(
                common - info["wheel_differential"])
        new_frame = self._frames(applied, params)
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
        if self.fallen.get("assist_n", 0.0):
            # Stand-up curriculum: an upward pull on the torso while it is low,
            # gone at standing height; the curriculum takes it to zero.
            low = jp.clip(2.0 * (1.0 - state.data.site_xpos[self.spec.imu_site][2] / self.stand_imu_height), 0.0, 1.0)
            force = force + self.fallen["assist_n"] * low * jp.array([0.0, 0.0, 1.0])
        xfrc = xfrc.at[self.spec.torso_body, 0:3].set(force)

        noise_keys = jax.random.split(k_noise, self.decimation)
        # The wheel torque reaches the body through the drive train's backlash
        # and compliance: a first-order lag of wheel_lag seconds (0: none).
        lag = params["wheel_lag"]
        blend = jp.where(lag > 0, 1.0 - jp.exp(-self.dt_physics / jp.maximum(lag, 1e-6)), 1.0)
        blend = jp.ones(N_JOINTS).at[self.wheel_idx].set(blend)
        limit = self._torque_limit(info["heat"])

        gear_on = params["gear_on"] > 0
        w = self.wheel_idx

        def substep(carry, inputs):
            data, history, pstep, drive_tau, rotor = carry
            j, key = inputs
            due = j.astype(jp.float32) * self.dt_physics >= delay - 1e-9
            frame = jp.where(due, new_frame, old_frame)
            tau = self._mit_torque(frame, data, params, limit) * params["torque_scale"]
            motor, passed, geared = self._gear(frame, data, params, limit, rotor)
            rigid = jp.stack([data.qpos[self.qadr[w]], data.qvel[self.dadr[w]]])
            rotor = jp.where(gear_on, geared, rigid)
            out = tau.at[w].set(jp.where(gear_on, passed, tau[w]))
            tau = tau.at[w].set(jp.where(gear_on, motor, tau[w]))
            drive_tau = drive_tau + blend * (out - drive_tau)
            ctrl = jp.zeros(self.mj_model.nu).at[self.act_ids].set(drive_tau)
            data = data.replace(ctrl=ctrl, xfrc_applied=xfrc)
            data = mjx.step(model, data)
            pstep = pstep + 1
            history = latency.write(history, pstep, self._record(data, params, key, rotor))
            return (data, history, pstep, drive_tau, rotor), tau

        (data, history, pstep, drive_tau, rotor), taus = jax.lax.scan(
            substep, (state.data, info["history"], info["physics_step"], info["drive_tau"], info["rotor"]),
            (jp.arange(self.decimation), noise_keys))
        info["drive_tau"] = drive_tau
        info["rotor"] = rotor
        # The heat of the torque each drive was asked for (the spread in
        # torque_scale is the torque constant's, not the current's).
        info["heat"] = torque_envelope.account(
            info["heat"], jp.mean((taus / params["torque_scale"]) ** 2, axis=0), self.dt, self.envelope)
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
        if self.wave:
            wave = jp.where(resample, self._sample_split_wave(jax.random.fold_in(k_cmd, 31)), info["split_wave"])
            wave = wave.at[2].add(wave[1] * self.dt)
            info["split_wave"] = wave
            info["command"] = info["command"].at[3].set(
                jp.where(wave[0] > 0.0, wave[0] * jp.sin(wave[2]), info["command"][3]))
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
        if self.has_jump:
            info["jump_clearance"] = jp.min(truth["clearance"])
            if self.has_flip:
                # The turn made since the request, about the body's pitch axis.
                info["flip_angle"] = info["flip_angle"] + (self._jumping(info) * float(self.flip.get("direction", 1.0))
                                                           * truth["pitch_rate"] * self.dt)
        if self.has_goal:
            # Held within reach_m for hold_s, or out of time: the next goal is
            # sampled from where the robot stands. The rewards of this step
            # are still for the goal it drove to (truth).
            g = self.goals
            k_goal = jax.random.split(jax.random.fold_in(k_cmd, 21), 2)
            near = truth["goal_dist"] < g["reach_m"]
            info["goal_hold"] = jp.where(near, info["goal_hold"] + self.dt, 0.0)
            info["goal_timer"] = info["goal_timer"] - self.dt
            reached = info["goal_hold"] >= g.get("hold_s", 1.0)
            renew = reached | (info["goal_timer"] <= 0.0)
            here = truth["axle"][:2]
            new_goal = self._sample_goal(k_goal[0], here, data)
            info["goal"] = jp.where(renew, new_goal, info["goal"])
            info["goal_timer"] = jp.where(renew, self._goal_time(here, new_goal, info["command"][0]),
                                          info["goal_timer"])
            info["goal_hold"] = jp.where(renew, 0.0, info["goal_hold"])
            # Odometry drift: a random walk that grows with the distance driven.
            driven = jp.hypot(truth["forward_speed"], truth["lateral_speed"]) * self.dt
            drift = g.get("odometry_drift", 0.0) * driven * jax.random.normal(k_goal[1], (2,))
            info["odo_error"] = jp.where(renew, 0.0, info["odo_error"] + drift)
            info["goal_obs"] = self._goal_obs(data, info)
        if self.n_rays:
            # The scanner's sample and hold at its own rate.
            info["ray_timer"] = info["ray_timer"] - self.dt
            due = info["ray_timer"] <= 0.0
            fresh = self._measure_rays(truth["rays"], jax.random.fold_in(k_noise, 23))
            info["ray_obs"] = jp.where(due, fresh, info["ray_obs"])
            info["ray_timer"] = jp.where(due, info["ray_timer"] + info["ray_period"], info["ray_timer"])
        if self.n_profile:
            # The map's sample and hold at the camera's rate, published one period late.
            info["profile_travel"] = profile_travel(info["profile_travel"], truth["forward_speed"],
                                                    truth["yaw_rate"], self.dt, self.profile, jp)
            info["profile_timer"] = info["profile_timer"] - self.dt
            due = info["profile_timer"] <= 0.0
            # The floor the node believes each wheel stands on wanders: a bias per track that
            # walks between measurements (profile.bias_walk_m per measurement, within bias_max_m).
            walk = self.profile.get("bias_walk_m", 0.0)
            if walk > 0.0:
                limit = self.profile.get("bias_max_m", 0.02)
                moved = jp.clip(info["profile_bias"] + walk * jax.random.normal(jax.random.fold_in(k_noise, 31), (2,)),
                                -limit, limit)
                info["profile_bias"] = jp.where(due, moved, info["profile_bias"])
            fresh = self._measure_profile(data, info, jax.random.fold_in(k_noise, 29), truth["clearance"])
            info["profile_obs"] = jp.where(due, info["profile_pending"], info["profile_obs"])
            info["profile_pending"] = jp.where(due, fresh, info["profile_pending"])
            info["profile_timer"] = jp.where(due, info["profile_timer"] + info["profile_period"], info["profile_timer"])
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
        terminated = jp.maximum(self._terminated(truth, info), diverged.astype(jp.float32))
        cat_delta = (self._constraints(truth, taus, info) if self.cfg["constraints"].get("enabled", True)
                     else jp.float32(0))
        reward, terms = self._reward(truth, info, action, prev_action, info["last_action2"], taus, terminated)
        timeout = info["step"] >= self.episode_steps
        done = jp.maximum(terminated, timeout.astype(jp.float32))
        obs = {"actor": self._actor_obs(obs_history),
               "critic": self._critic_obs(parts, truth, info, model_params)}
        obs = jax.tree.map(lambda x: jp.where(diverged, 0.0, x), obs)
        termination_penalty = self.cfg["rewards"]["weights"].get("termination", 0.0) * self.event_scale
        reward = jp.where(diverged | ~jp.isfinite(reward), termination_penalty, reward)
        terms = {k: jp.where(diverged, 0.0, v) for k, v in terms.items()}
        cat_delta = jp.where(diverged, 0.0, cat_delta)
        info["obs_history"] = jp.where(diverged, 0.0, info["obs_history"])
        info["heat"] = jp.where(diverged | ~jp.isfinite(info["heat"]), 0.0, info["heat"])
        info["rotor"] = jp.where(diverged | ~jp.all(jp.isfinite(info["rotor"])), 0.0, info["rotor"])
        metrics = dict(terms)
        if self.has_jump:
            metrics["jump_active"] = (info["jump_timer"] > 0.0).astype(jp.float32)
            metrics["clearance_min"] = jp.min(truth["clearance"])
        metrics.update({
            "err_forward": jp.abs(truth["forward_speed"] - v_ref),
            "err_yaw": jp.abs(truth["yaw_rate"] - w_ref),
            "err_height": jp.abs(jp.mean(truth["heights"]) - h_ref),
            "cat_delta": cat_delta, "tilt": truth["tilt"],
            "err_roll": (jp.abs(truth["offsets"][0] - truth["offsets"][1]
                                - self.feasible_split(info["command_ref"][3], h_ref)) if self.has_split
                         else jp.abs(truth["pitch"] - self._lean_target(info))
                         if self.has_lean
                         else jp.abs(truth["roll"] - info["command_ref"][3]) if self.has_roll else jp.float32(0.0)),
        })
        if self.has_goal:
            metrics["goal_distance"] = truth["goal_dist"]
            metrics["goal_reached"] = reached.astype(jp.float32)
        if self.has_pillars:
            metrics["clearance"] = jp.minimum(truth["obstacle_clearance"], 2.0)
        metrics = {k: jp.where(diverged, 0.0, v) for k, v in metrics.items()}
        return State(data=data, obs=obs, reward=reward, done=done, terminated=terminated,
                     cat_delta=cat_delta, info=info, metrics=metrics)

    def metric_names(self):
        names = list(self.cfg["rewards"]["weights"]) + ["err_forward", "err_yaw", "err_height", "cat_delta", "tilt",
                                                        "err_roll"]
        return (names + (["jump_active", "clearance_min"] if self.has_jump else [])
                + (["goal_distance", "goal_reached"] if self.has_goal else [])
                + (["clearance"] if self.has_pillars else []))

    # ------------------------------------------------------------ task terms
    def _terminated(self, truth, info=None):
        t = self.cfg["termination"]
        bad = (truth["tilt"] > t["tilt_rad"]) | (truth["imu_height"] < t["imu_height_m"])
        if self.has_flip and info is not None:
            # Upside down is part of a flip; the torso on the floor is not.
            bad = jp.where(self._jumping(info) > 0.0,
                           truth["torso_clearance"] < self.flip.get("torso_floor_m", 0.01), bad)
        bad = bad | ~jp.all(jp.isfinite(truth["q"]))
        if self.has_pillars:
            # Driving into a pillar is a failure, like a fall.
            bad = bad | (truth["obstacle_clearance"] < t.get("obstacle_m", 0.0))
        return bad.astype(jp.float32)

    def _jumping(self, info):
        """1 inside a jump window and for landing_s after it."""
        if not self.has_jump:
            return jp.float32(0.0)
        return (info["jump_timer"] > -self.jump.get("landing_s", 0.3)).astype(jp.float32)

    def _lean_target(self, info):
        """The pitch asked for by a lean command (0 without one)."""
        if not self.has_lean:
            return jp.float32(0.0)
        ref = info["command_ref"]
        return self.feasible_lean(float(self.lean.get("gain", 1.0)) * ref[3], ref[2])

    def _constraints(self, truth, taus, info):
        c = self.cfg["constraints"]
        legs = self.leg_idx
        tau_leg = jp.max(jp.abs(taus[:, legs]), axis=0)
        stretch = jp.max(truth["stretch"] - c["stretch"]["limit"]) / c["stretch"]["slack"]
        if "upright_rad" in c["stretch"]:
            # Folded flat against their stops, unpowered legs are stretched too.
            stretch = stretch * (truth["tilt"] < c["stretch"]["upright_rad"])
        violations = [
            jp.max(tau_leg - (self.cat_torque_limit - 0.05)) / c["leg_torque"]["slack"],
            stretch,
            jp.max(jp.maximum(self.q_lo + c["joint_range"]["margin_rad"] - truth["q"][legs],
                              truth["q"][legs] - (self.q_hi - c["joint_range"]["margin_rad"])))
            / c["joint_range"]["slack"],
            jp.max(jp.maximum(self.leg_h_lo - c["height"]["margin_m"] - truth["heights"],
                              truth["heights"] - (self.leg_h_hi + c["height"]["margin_m"]))) / c["height"]["slack"],
            (truth["tilt"] - self._lean_target(info) * jp.sign(truth["pitch"]) - c["tilt"]["limit_rad"])
            / c["tilt"]["slack"] * (1.0 - self._jumping(info) if self.has_flip else 1.0),
        ]
        if c.get("leg_split"):
            # Legs split fore and aft make the robot statically stable in
            # pitch: a stance the jump training found and kept (DESIGN.md 8).
            split = jp.abs(truth["offsets"][0] - truth["offsets"][1])
            split = (split - c["leg_split"]["limit_m"]) / c["leg_split"]["slack"]
            if "upright_rad" in c["leg_split"]:
                # Only the stance counts: getting up may brace on one leg.
                split = split * (truth["tilt"] < c["leg_split"]["upright_rad"])
            violations.append(split)
        if self.has_pillars and c.get("obstacle"):
            violations.append((c["obstacle"]["margin_m"] - truth["obstacle_clearance"]) / c["obstacle"]["slack"])
        if "wheel_lift" in c and c["wheel_lift"]:
            lift = jp.max(truth["clearance"] - c["wheel_lift"]["limit_m"]) / c["wheel_lift"]["slack"]
            violations.append(lift * (1.0 - self._jumping(info)))
        if c.get("wheel_speed"):
            # The drives' speed limit, which the simulation does not have.
            spin = jp.max(jp.abs(truth["qd"][self.wheel_idx]))
            violations.append((spin - c["wheel_speed"]["limit_rad_s"]) / c["wheel_speed"]["slack"]
                              * (1.0 - self._jumping(info)))
        if c.get("leg_speed"):
            # The same limit at the leg drives: it, not the torque, bounds a hop's take-off.
            violations.append((jp.max(jp.abs(truth["qd"][legs])) - c["leg_speed"]["limit_rad_s"])
                              / c["leg_speed"]["slack"])
        if c.get("thermal", {}).get("enabled", False):
            # The heat budget running out: 0 until the fold-back starts.
            violations.append(1.0 - torque_envelope.budget(jp.max(info["heat"]), self.envelope, jp))
        v = jp.clip(jp.stack(violations), 0.0, 1.0)
        # p_max is per 10 ms; a faster policy terminates as often per second.
        return 1.0 - (1.0 - c["p_max"] * jp.max(v)) ** (self.dt / 0.01)

    def _guarded_speed(self, forward, truth):
        """The forward reference with the guard: full speed slow_m before the stopping
        point, zero stop_m (axle to face) before terrain too tall to cross. Backing away is
        never limited."""
        if not self.guard:
            return forward
        allowed = self.v_max * jp.clip((truth["block_distance"] - self.guard["stop_m"]) / self.guard["slow_m"],
                                       0.0, 1.0)
        return jp.minimum(forward, allowed)

    def _reward(self, truth, info, action, prev_action, prev_action2, taus, terminated):
        r = self.cfg["rewards"]
        w, sig = r["weights"], r["sigma"]
        ref = info["command_ref"]
        if self.rim_speed > 0.0:
            room = jp.maximum(self.rim_speed - 0.5 * self.track_width * jp.abs(ref[1]), 0.0)
            ref = ref.at[0].set(jp.clip(ref[0], -room, room))
        g = 9.81
        # Goal commands carry no yaw rate: the lean follows the turn driven.
        lean = (jp.arctan(truth["forward_speed"] * truth["yaw_rate"] / g) if self.has_goal
                else jp.arctan(ref[0] * ref[1] / g))
        # Commanded roll plus the coordinated-turn lean (roll is positive with
        # the left side up, so a left turn leans to -lean).
        roll_target = (ref[3] if self.has_roll and not self.has_split and not self.has_lean else 0.0) - lean
        pitch_target = self._lean_target(info)
        # The split asked for: the command's (right axle ahead is positive) or the config's.
        split_target = self.feasible_split(ref[3], ref[2]) if self.has_split else r.get("split_m", 0.0)
        legs, wheels = self.leg_idx, self.wheel_idx
        terms = {
            "alive": jp.float32(1.0),
            "track_forward": jp.exp(-((truth["forward_speed"] - self._guarded_speed(ref[0], truth))
                                      / sig["forward"]) ** 2),
            "track_yaw": jp.exp(-((truth["yaw_rate"] - ref[1]) / sig["yaw"]) ** 2),
            # The kernel is flat far from the reference; this pulls from there (fast spins).
            "yaw_error": jp.minimum((truth["yaw_rate"] - ref[1]) ** 2, 9.0),
            "track_height": jp.exp(-((jp.mean(truth["heights"]) - ref[2]) / sig["height"]) ** 2),
            "roll": (truth["roll"] - roll_target) ** 2,
            "track_roll": jp.exp(-((truth["roll"] - roll_target) / sig.get("roll", 0.05)) ** 2),
            "pitch": (truth["pitch"] - pitch_target) ** 2,
            "track_pitch": jp.exp(-((truth["pitch"] - pitch_target) / sig.get("lean", 0.08)) ** 2),
            "pitch_rate": truth["pitch_rate"] ** 2,
            "vertical_velocity": truth["vertical_speed"] ** 2,
            "leg_symmetry": (truth["offsets"][0] - truth["offsets"][1] - split_target) ** 2,
            # A split stance (rewards.split_m: the right axle that far ahead of
            # the left one), held about the neutral offset.
            "track_split": jp.exp(-((truth["offsets"][0] - truth["offsets"][1] - split_target)
                                    / sig.get("split", 0.03)) ** 2),
            "split_centre": (truth["offsets"][0] + truth["offsets"][1]) ** 2,
            # Differences per 10 ms, so a faster policy is held to the same
            # smoothness (unchanged at 100 Hz).
            "action_rate": jp.sum(((action - prev_action) * self.difference_scale) ** 2),
            "action_accel": jp.sum(((action - 2 * prev_action + prev_action2)
                                    * self.difference_scale ** 2) ** 2),
            # The opposed half of the wheel actions drives the body's yaw.
            "wheel_differential_rate": (((action[wheels[0]] - action[wheels[1]])
                                         - (prev_action[wheels[0]] - prev_action[wheels[1]]))
                                        * self.difference_scale) ** 2,
            "leg_torque": jp.mean(jp.sum(taus[:, legs] ** 2, axis=1)),
            "wheel_torque": jp.mean(jp.sum(taus[:, wheels] ** 2, axis=1)),
            "leg_velocity": jp.sum(truth["qd"][legs] ** 2),
            "termination": terminated,
            "upright": 0.5 * (1.0 + truth["r_body"][2, 2]),
            "stand_height": jp.clip((truth["imu_height"] - r.get("stand_height_from_m", 0.0))
                                    / (self.stand_imu_height - r.get("stand_height_from_m", 0.0)), 0.0, 1.0),
        }
        if "upright_gate_rad" in r:
            # Tracking pays only once up, so lying still is not a way to earn it.
            up = jp.exp(-(truth["tilt"] / r["upright_gate_rad"]) ** 2)
            for name in ("track_forward", "track_yaw", "track_height", "track_roll"):
                terms[name] = terms[name] * up
        if self.has_jump:
            # In a jump window: reward both wheels' clearance up to the target,
            # and drop the terms that hold the body at a steady height.
            jumping = self._jumping(info)
            in_window = (info["jump_timer"] > 0.0).astype(jp.float32) * (1.0 - info["jump_done"])
            one_leg = bool(self.jump.get("one_leg", False))
            if one_leg:
                # Standing on the left leg: the right wheel's clearance counts
                # while the left tyre stays on the ground.
                grounded = (truth["clearance"][1] < 0.005).astype(jp.float32)
                if "link_clearance" in truth:
                    # On the tyre alone: no leg link within link_margin_m of the floor.
                    grounded = grounded * (truth["link_clearance"]
                                           > self.jump.get("link_margin_m", 0.015)).astype(jp.float32)
                raised = truth["clearance"][0]
                lift = grounded * jp.clip(raised, 0.0, info["jump_height"]) / info["jump_height"]
            else:
                raised = jp.min(truth["clearance"])
                # target_m: every request is paid up to this clearance, whatever height it
                # names (the gamepad always asks for the same one): the highest hop there is.
                target = self.jump.get("target_m") or info["jump_height"]
                lift = jp.clip(raised, 0.0, target) / target
            target = info["jump_height"] if one_leg else target
            terms["jump_clearance"] = in_window * lift
            terms["jump_vz"] = in_window * jp.clip(truth["base_vz"], 0.0, self.jump.get("vz_clip_m_s", 1.5))
            terms["jump_overshoot"] = in_window * jp.maximum(raised - target, 0.0) ** 2
            free = (("track_height", "roll", "track_roll", "leg_symmetry", "track_forward", "track_yaw")
                    if one_leg and self.jump.get("steady", False) else
                    ("track_height", "vertical_velocity") + (
                        ("roll", "track_roll", "leg_symmetry", "track_forward", "track_yaw") if one_leg else ()))
            if one_leg and self.jump.get("steady", False):
                # A stand, not a series of hops: the supporting tyre leaving the floor costs.
                terms["jump_airborne"] = in_window * (truth["clearance"][1] > 0.005)
            if self.has_flip:
                turn = float(self.flip.get("angle_rad", 2.0 * math.pi))
                rate = float(self.flip.get("rate_clip_rad_s", 20.0))
                spin = float(self.flip.get("direction", 1.0)) * truth["pitch_rate"]
                turned = (info["flip_angle"] > turn - self.flip.get("tolerance_rad", 0.4)).astype(jp.float32)
                # The turn itself, paid per radian until it is whole (in the window only).
                # airborne_m: only with both wheels that far off the floor, or falling over
                # backwards on the spot is two radians of it (the first run did that).
                terms["flip_rate"] = ((info["jump_timer"] > 0.0) * (info["flip_angle"] < turn)
                                      * (jp.min(truth["clearance"]) >= self.flip.get("airborne_m", 0.0))
                                      * jp.clip(spin, -rate, rate))
                # Then: on the wheels and upright, until landing_s after the window.
                terms["flip_landed"] = (jumping * turned * jp.exp(-(truth["tilt"] / 0.3) ** 2)
                                        * (jp.max(truth["clearance"]) < 0.01))
                # Still turning after the whole turn costs.
                terms["flip_over"] = jumping * turned * truth["pitch_rate"] ** 2
                free = free + ("pitch", "track_pitch", "pitch_rate", "track_forward", "upright")
            for name in free:
                terms[name] = terms[name] * (1.0 - jumping)
        if self.has_goal:
            # Agile But Safe (He et al. 2024) style: speed towards the goal up to
            # the cruise speed, then stand still on it.
            dist = truth["goal_dist"]
            at_goal = (dist < self.goals["reach_m"]).astype(jp.float32)
            toward = truth["goal_vec"] / jp.maximum(dist, 1e-6)
            velocity = jp.array([truth["forward_speed"], truth["lateral_speed"]])
            cruise = jp.maximum(ref[0], 0.1)
            terms["goal_progress"] = (1.0 - at_goal) * jp.clip(velocity @ toward, -cruise, cruise) / cruise
            terms["goal_facing"] = (1.0 - at_goal) * toward[0]
            terms["goal_reach"] = at_goal * jp.exp(-(jp.linalg.norm(velocity) / 0.15) ** 2)
        if self.has_pillars:
            margin = r.get("obstacle_margin_m", 0.3)
            terms["obstacle"] = jp.clip(1.0 - truth["obstacle_clearance"] / margin, 0.0, 1.0) ** 2
        terms = {k: v for k, v in terms.items() if k in w}
        scale = self.dt if r.get("scale_by_dt", True) else 1.0
        # Events happen once, whatever the policy rate: their weight is per
        # 10 ms step.
        weighted = {k: w[k] * v * (self.event_scale if k in EVENT_TERMS else scale) for k, v in terms.items()}
        reward = sum(weighted.values())
        return reward, weighted


def _quat_mul(a, b):
    w1, x1, y1, z1 = a[0], a[1], a[2], a[3]
    w2, x2, y2, z2 = b[0], b[1], b[2], b[3]
    return jp.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
