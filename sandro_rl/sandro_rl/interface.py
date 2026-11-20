"""The policy's deployment contract in plain numpy: observation, action, commands.

This is what a ros2_control controller running the exported policy must do
every cycle, written without JAX so the CPU evaluator (evaluate.py) and a
future C++ port follow one reference. tests/test_observation_contract.py
checks it against the training environment value by value.
"""

from __future__ import annotations

import math

import numpy as np

from . import sources
from .robot import RobotSpec, rotation_x
from .sources import JOINTS


def _ros(value, fallback):
    return fallback if value == "ros" else value


def quat_to_mat(w, x, y, z) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# The goal and range observations of the avoid task, for numpy (deployment)
# and jax.numpy (training) alike, so both sides compute them with one code.
def ray_angles(count: int, fov_deg: float) -> np.ndarray:
    """Scanner ray directions in the body frame, rad (0 ahead, + to the left),
    symmetric about the forward axis so the left-right mirror reverses them."""
    if fov_deg >= 360.0:
        return -math.pi + (np.arange(count) + 0.5) * 2.0 * math.pi / count
    half = math.radians(fov_deg) / 2.0
    return np.linspace(-half, half, count) if count > 1 else np.zeros(1)


def ray_observation(distances, max_range_m, xp=np):
    """Ranges as fractions of the maximum range, 1 for no return."""
    return xp.clip(distances / max_range_m, 0.0, 1.0)


def goal_observation(forward, left, range_m, xp=np):
    """(distance capped at range_m / range_m, cos, sin of the bearing) of a
    goal at (forward, left) m from the axle midpoint in the heading frame."""
    dist = xp.sqrt(forward * forward + left * left)
    safe = xp.maximum(dist, 1e-6)
    near = dist < 1e-6
    return xp.stack([xp.minimum(dist, range_m) / range_m, xp.where(near, 1.0, forward / safe),
                     xp.where(near, 0.0, left / safe)])


# The terrain profile of sandro_perception (/terrain_profile): the floor height at
# `distances_m` ahead of each axle end, relative to the floor that wheel stands on, right
# track then left, and a known flag per point. The camera sees the floor from `near` m
# ahead of the wheels; nearer floor is known only once the robot has driven over what it
# saw (the node's map), which the three functions below model with one number per robot:
# the distance driven since the map was empty.
PROFILE_SCALE = 10.0          # observation per m of height
PROFILE_CLIP_M = (-0.15, 0.25)  # sandro_perception's clip


def profile_known(distances, travel, near_m, far_m, xp=np):
    """1 where the floor `distances` m ahead of a wheel is in the map: in the camera's view
    now (near_m to far_m), or seen before the robot drove `travel` m up to it."""
    return ((distances >= near_m - travel) & (distances <= far_m)) * 1.0


def profile_occluded(distances, heights, line_distances, line_heights, camera_height, camera_ahead, xp=np):
    """1 where the camera cannot see the floor at `distances` (heights above the wheel's
    floor) over the terrain along the track (`line_heights` at `line_distances`): the floor
    behind a plank or a hill, which the node's map then never holds. The camera is
    `camera_height` above the wheel's floor and `camera_ahead` of the axle."""
    run = xp.maximum(line_distances - camera_ahead, 1e-3)[None, :]
    between = (line_distances[None, :] > camera_ahead + 0.02) & (line_distances[None, :] < distances[:, None] - 0.02)
    # The sight line over each nearer terrain point, at the sample's distance.
    sight = camera_height - (camera_height - line_heights[None, :]) * (distances[:, None] - camera_ahead) / run
    return xp.any(between & (heights[:, None] < sight - 0.005), axis=1) * 1.0


def profile_shadow(distances, heights, hidden, line_distances, line_heights, face_m, obstacle_m=0.07,
                   shadow_m=0.6, xp=np):
    """(heights, filled): sandro_perception's answer to an obstacle's shadow. A hidden sample
    up to shadow_m behind terrain more than obstacle_m above the wheel's floor reads that
    terrain's height, known. Nothing reads higher than face_m: of an obstacle taller than the
    camera only the face is seen, and its cell averages 0.2-0.3 m."""
    before = ((line_distances[None, :] < distances[:, None])
              & (line_distances[None, :] >= distances[:, None] - shadow_m))
    cast = xp.max(xp.where(before, line_heights[None, :], -1.0), axis=1)
    filled = (hidden > 0.5) & (cast > obstacle_m)
    return xp.minimum(xp.where(filled, cast, heights), face_m), filled * 1.0


def profile_travel(travel, forward_speed, yaw_rate, dt, cfg, xp=np):
    """The distance driven since the map was empty, one policy step on. Turning brings
    floor ahead of the wheels that the camera never saw, and the node forgets floor it has
    not seen for max_age_s."""
    step = forward_speed * dt - (cfg.get("forget_m_per_rad", 0.4) * xp.abs(yaw_rate)
                                 + cfg.get("forget_m_per_s", 0.08)) * dt
    return xp.clip(travel + step, 0.0, cfg.get("travel_max_m", 2.0))


def profile_observation(heights, known, xp=np):
    """[heights (clipped, PROFILE_SCALE per m, 0 where unknown), known flags], as
    /terrain_profile orders them: right track, left track."""
    h = xp.clip(heights, PROFILE_CLIP_M[0], PROFILE_CLIP_M[1]) * known
    return xp.concatenate([h * PROFILE_SCALE, known])


class GoalOdometry:
    """Keeps a goal in the robot's heading frame from wheel odometry and the
    IMU yaw, as the deployment controller must (training adds the drift of
    this estimate, commands.goals.odometry_drift)."""

    def __init__(self, wheel_radius_m: float):
        self.radius = wheel_radius_m
        self.goal = None              # in the odometry frame
        self.position = np.zeros(2)
        self.yaw = 0.0

    def set_goal(self, forward, left, yaw):
        """A new goal, given relative to the robot now."""
        self.position, self.yaw = np.zeros(2), yaw
        c, s = math.cos(yaw), math.sin(yaw)
        self.goal = np.array([c * forward - s * left, s * forward + c * left])

    def update(self, wheel_velocity, pitch_rate, yaw, dt):
        """wheel_velocity: (right, left) rad/s; yaw from the IMU, rad."""
        # The wheels turn relative to the body: add the pitch rate back (the
        # WBC's forward_velocity_m_s does the same).
        speed = self.radius * (float(np.mean(wheel_velocity)) + pitch_rate)
        mid = self.yaw + 0.5 * math.remainder(yaw - self.yaw, 2.0 * math.pi)
        self.position = self.position + speed * dt * np.array([math.cos(mid), math.sin(mid)])
        self.yaw = yaw

    def relative(self):
        """(forward, left) of the goal from the robot now."""
        rel = self.goal - self.position
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return float(c * rel[0] + s * rel[1]), float(-s * rel[0] + c * rel[1])


class PolicyInterface:
    def __init__(self, config: dict, spec: RobotSpec):
        cfg = config["env"]
        self.cfg = cfg
        self.spec = spec
        lim = spec.limits
        self.leg_idx = spec.leg_index
        self.wheel_idx = spec.wheel_index
        self.stand_q = np.asarray(spec.stand_q, float)
        self.control_rate_hz = lim.control_rate_hz
        self.cycles_per_action = int(round(lim.control_rate_hz / float(cfg["policy_rate_hz"])))
        self.dt = self.cycles_per_action / lim.control_rate_hz
        a = cfg["action"]
        self.clip = float(a["clip"])
        self.leg_kp = float(_ros(a["legs"]["kp"], lim.leg_kp_nm_rad))
        self.leg_kd = float(_ros(a["legs"]["kd"], lim.leg_kd_nm_s_rad))
        self.leg_scale = float(a["legs"]["scale_rad"])
        self.wheels = a["wheels"]
        tl = a.get("torque_limit_nm", {})
        # "ros": the drives' envelope peak (spec.envelope), folding back with heat.
        self.envelope = spec.envelope
        self.thermal = bool(a.get("torque_envelope", True))
        self.differential_filter_hz = float(a.get("wheel_differential_filter_hz", 0.0))
        peak = spec.envelope.peak_nm
        self.torque_limit = np.array([float(_ros(tl.get("wheels", "ros"), peak))
                                      if j in sources.WHEEL_JOINTS else float(_ros(tl.get("legs", "ros"), peak))
                                      for j in JOINTS])
        self.q_lo, self.q_hi = spec.leg_joint_range
        offset = cfg["observation"].get("imu_offset_correction", "ros")
        self.imu_offset_correction = sources.simulated_imu_roll_offset() if offset == "ros" else float(offset)
        self.body_from_imu_correction = rotation_x(-self.imu_offset_correction)
        self.scales = cfg["observation"]["scales"]
        self.history = int(cfg["observation"]["history"])
        self.history_stride = int(cfg["observation"].get("history_stride", 1))
        self.history_frames = (self.history - 1) * self.history_stride + 1
        self.accelerometer = bool(cfg["observation"].get("accelerometer", False))
        c = cfg["commands"]
        self.v_max = float(_ros(c["forward_m_s"], lim.max_forward_m_s))
        self.yaw_max = float(_ros(c["yaw_rad_s"], lim.max_yaw_rad_s))
        h = _ros(c["height_m"], [lim.height_min_m, lim.height_max_m])
        self.h_min, self.h_max = float(h[0]), float(h[1])
        self.accel_limit = float(_ros(c["forward_accel_m_s2"], lim.forward_accel_limit_m_s2))
        self.yaw_tau = float(_ros(c["yaw_time_constant_s"], lim.yaw_command_time_constant_s))
        self.h_rate = float(_ros(c["height_rate_m_s"], lim.max_height_rate_m_s))
        self.roll_max = float(c.get("roll_rad", 0.0))
        self.has_roll = self.roll_max > 0.0
        self.roll_rate = float(c.get("roll_rate_rad_s", 0.5))
        reach = c.get("leg_height_limits_m", [self.h_min, self.h_max])
        self.leg_h_lo, self.leg_h_hi = float(reach[0]), float(reach[1])
        self.track_width = float(lim.track_width_m)
        jump = c.get("jump") or {}
        self.has_jump = bool(jump.get("enabled", bool(jump)))
        self.jump_h_max = float(jump.get("height_m", [0.0, 0.1])[1])
        self.jump_window = float(jump.get("window_s", 0.8))
        # A flip policy (simulation only): the fall cutoff waits out the window and its landing.
        self.has_flip = bool(jump.get("flip"))
        self.jump_landing = float(jump.get("landing_s", 0.3))
        goals = c.get("goals") or {}
        self.has_goal = bool(goals.get("enabled", bool(goals)))
        self.goal_range = float(goals.get("range_m", 4.0))
        rays = cfg["observation"].get("rays") or {}
        self.n_rays = int(rays.get("count", 0))
        self.rays = rays
        self.profile = cfg["observation"].get("profile") or {}
        self.profile_distances = np.asarray(self.profile.get("distances_m", []), float)
        self.n_profile = 4 * len(self.profile_distances)

    # ------------------------------------------------------------ observation
    def body_rotation(self, quat_wxyz) -> np.ndarray:
        q = np.asarray(quat_wxyz, float)
        q = q / np.linalg.norm(q)
        return quat_to_mat(*q) @ self.body_from_imu_correction

    def jump_observation(self, timer, height) -> np.ndarray:
        """(target clearance / max, remaining fraction of the window), zero outside a jump."""
        if timer <= 0.0:
            return np.zeros(2)
        return np.array([height / self.jump_h_max, timer / self.jump_window])

    def frame(self, quat_wxyz, gyro, accel, q, v, last_action, command_ref, jump=(0.0, 0.0),
              goal=None, rays=None, profile=None) -> np.ndarray:
        """One observation frame from the measured signals (JOINTS order).

        jump: jump_observation(...) for policies trained with jump events;
        goal: goal_observation(...) and rays: ray_observation(...) for the
        avoid task; profile: profile_observation(...) of /terrain_profile."""
        r_body = self.body_rotation(quat_wxyz)
        gravity = r_body.T @ np.array([0.0, 0.0, -1.0])
        q, v = np.asarray(q, float), np.asarray(v, float)
        command = np.array([command_ref[0] / self.v_max, command_ref[1] / self.yaw_max,
                            (command_ref[2] - 0.5 * (self.h_min + self.h_max)) / (0.5 * (self.h_max - self.h_min))])
        if self.has_roll:
            command = np.append(command, command_ref[3] / self.roll_max)
        parts = [gravity, np.asarray(gyro, float) * self.scales["gyro"]]
        if self.accelerometer:
            parts.append(np.asarray(accel, float) / 9.81)
        parts += [q[self.leg_idx] - self.stand_q[self.leg_idx], v[self.leg_idx] * self.scales["leg_vel"],
                  v[self.wheel_idx] * self.scales["wheel_vel"], np.asarray(last_action, float), command]
        if self.has_jump:
            parts.append(np.asarray(jump, float))
        if self.has_goal:
            parts.append(np.asarray(goal, float))
        if self.n_rays:
            parts.append(np.asarray(rays, float))
        if self.n_profile:
            parts.append(np.asarray(profile, float))
        return np.concatenate(parts)

    # ------------------------------------------------------------ action
    def frames(self, action) -> np.ndarray:
        """MIT frames [5 (q_ref, v_ref, kp, kd, tau_ff), 6] from one action."""
        a = np.clip(np.asarray(action, float), -self.clip, self.clip)
        f = np.zeros((5, len(JOINTS)))
        legs, wheels = self.leg_idx, self.wheel_idx
        f[0, legs] = np.clip(self.stand_q[legs] + self.leg_scale * a[legs], self.q_lo, self.q_hi)
        f[2, legs] = self.leg_kp
        f[3, legs] = self.leg_kd
        if self.wheels["mode"] == "velocity":
            f[1, wheels] = self.wheels["scale_rad_s"] * a[wheels]
            f[3, wheels] = self.wheels["kd_nm_s_rad"]
        elif self.wheels["mode"] == "torque":
            f[4, wheels] = self.wheels["scale_nm"] * a[wheels]
        else:
            raise ValueError(f"unknown wheel action mode {self.wheels['mode']}")
        return f

    def mit_torque(self, frame_column, q, v, limit) -> float:
        q_ref, v_ref, kp, kd, ff = frame_column
        q_ref = q_ref if kp > 0 else q
        return float(np.clip(kp * (q_ref - q) + kd * (v_ref - v) + ff, -limit, limit))

    # ------------------------------------------------------------ commands
    def shape(self, ref, command) -> np.ndarray:
        """One policy step of the whole-body controller's command shaping."""
        v = ref[0] + np.clip(command[0] - ref[0], -self.accel_limit * self.dt, self.accel_limit * self.dt)
        w = ref[1] + (command[1] - ref[1]) * (1.0 - math.exp(-self.dt / self.yaw_tau))
        h = ref[2] + np.clip(command[2] - ref[2], -self.h_rate * self.dt, self.h_rate * self.dt)
        if not self.has_roll:
            return np.array([v, w, h])
        target = self.feasible_roll(np.clip(command[3], -self.roll_max, self.roll_max), h)
        roll = ref[3] + np.clip(target - ref[3], -self.roll_rate * self.dt, self.roll_rate * self.dt)
        return np.array([v, w, h, roll])

    def feasible_roll(self, roll, height) -> float:
        """Clip a roll command so both legs stay inside their reachable height range."""
        room = max(min(height - self.leg_h_lo, self.leg_h_hi - height), 0.0)
        limit = math.atan(2.0 * room / self.track_width)
        return float(np.clip(roll, -limit, limit))
