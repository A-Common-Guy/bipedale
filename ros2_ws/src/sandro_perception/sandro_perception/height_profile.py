"""The terrain ahead of each wheel, from the depth camera: the perception contract.

Plain numpy, no ROS: the node (node.py) feeds it on the robot, the tests and
sandro_rl's evaluator feed it rendered depth, and the training environment
emulates what it outputs.

What comes out, `profile()`: for the right and the left wheel track, the
floor height at `distances_m` ahead of the axle along the heading, relative
to the floor that wheel stands on now, with a flag per point saying whether
it is known. A step up 0.3 m ahead of the right wheel reads +0.03 at that
point. Unknown points read 0: a policy that gets no terrain behaves as a
blind one.

How it gets there:

  - Poses. `update_state()` takes the joint angles, wheel velocities and the
    IMU attitude at the control side's rate and keeps a planar odometry of
    the axle midpoint (wheel speeds plus the body's pitch rate, heading from
    the IMU's yaw) in a gravity-aligned frame. The body's height in that
    frame follows from the leg kinematics and the floor under the wheels.
  - Depth. `add_depth()` deprojects an image with the camera intrinsics,
    carries the points through the mount and the body pose *at the image's
    stamp* (the frame is 30-60 ms old when it arrives) and bins them into
    a grid of `cell_m` cells fixed to the floor, a square window that travels
    with the robot. A cell's height is the upper end of its points (mean plus
    `spread` standard deviations, at most the highest), not their mean: a cell on an edge or on the
    face of a block holds points from the floor to the top, and the top is what
    a wheel meets. A cell changes by more than `confirm_m`, or becomes known,
    only when two images in a row agree: one bad image leaves no obstacle
    behind in the part of the map the camera cannot see again.
  - Memory. The camera looks ahead: at 0.34 m height and level, a D435 sees
    the floor only from about 0.6 m. Cells keep their height until they are
    seen again or `max_age_s` old, so a plank seen at 0.8 m is still known
    when the wheel reaches it. Cells never seen (after a turn on the spot, or
    at start) are unknown.
  - The floor under a wheel is the map there when it is known, so the
    heights stay consistent while the robot climbs what it has seen; where
    it is not, the last value is kept (flat is assumed). It is the lowest of
    the cells within `footprint_m` before and behind the contact point: the
    odometry is a few cm off after a metre, and a wheel 5 cm short of a step
    must not read as standing on it, or the step vanishes from the profile
    just when the policy has to act on it.

Frames: body is x forward, y left, z up with its origin at the IMU, the frame
of /imu_broadcaster/imu. The optical frame of the depth image is z along the
view, x right, y down.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Layout:
    """The contract with the policy: where the profile is sampled."""

    distances_m: tuple = (0.10, 0.20, 0.30, 0.45, 0.60, 0.80, 1.05, 1.35)   # ahead of the axle
    half_track_m: float = 0.152
    clip_m: tuple = (-0.15, 0.25)       # a wall ahead reads as the upper clip


@dataclass
class Body:
    """Sandro's legs and wheels in the body frame (sandro_description, stand keyframe)."""

    pivot_z_m: float = -0.0922          # the leg pivots' line below the IMU
    rear_pivot_x_m: float = -0.0131     # right inner, left outer
    front_pivot_x_m: float = 0.0769     # right outer, left inner
    base_m: float = 0.09                # five-bar: pivot spacing, upper and lower link
    upper_m: float = 0.08
    lower_m: float = 0.13
    wheel_radius_m: float = 0.0672      # loaded tyre
    half_track_m: float = 0.152


@dataclass
class Mount:
    """The camera's depth origin in the body frame. From the CAD camera on the 20-degree
    bracket (camera_link in the URDF); the D435's depth origin is its left imager."""

    x_m: float = 0.11411
    y_m: float = 0.0175
    z_m: float = 0.00056
    pitch_rad: float = 0.349066         # positive looks down
    roll_rad: float = 0.0
    yaw_rad: float = 0.0


@dataclass
class Settings:
    layout: Layout = field(default_factory=Layout)
    body: Body = field(default_factory=Body)
    mount: Mount = field(default_factory=Mount)
    cell_m: float = 0.04
    cells: int = 64                     # the window is cells x cells, 2.56 m square,
    window_ahead_m: float = 0.5         # centred this far ahead of the robot
    min_range_m: float = 0.25           # of the depth along the view
    max_range_m: float = 2.5
    stride: int = 2                     # use every stride-th pixel each way
    min_points: int = 3                 # per cell and image
    blend: float = 0.6                  # weight of a new image on a cell seen before
    spread: float = 1.5                 # a cell's height: its points' mean + spread x their deviation
    confirm_m: float = 0.03             # a larger change of a cell needs two images that agree,
    confirm_s: float = 0.4              # this close in time
    footprint_m: float = 0.05           # the floor under a wheel: the lowest cell this far either way
    max_age_s: float = 8.0
    band_m: tuple = (-0.30, 0.45)       # heights kept, about the floor under the wheels
    floor_rate_m_s: float = 0.5         # how fast the floor under a wheel may move
    history_s: float = 1.0              # poses kept for late images
    # The camera cannot see the floor behind an obstacle, nor the top of one taller than
    # itself: only the face. A sample that is unknown up to shadow_m behind a known cell
    # more than obstacle_m above the wheel's floor takes that cell's height, known: an
    # obstacle then reads as one, not as a hole in the map.
    obstacle_m: float = 0.07
    shadow_m: float = 0.6


def leg_forward_kinematics(theta_inner, theta_outer, body: Body):
    """Planar axle position (x toward the outer pivot, y down from the pivot line): the
    closed form of sandro_controllers/kinematics.cpp."""
    p = body.upper_m * (math.cos(theta_outer) - math.cos(theta_inner))
    q = body.base_m + body.upper_m * (math.sin(theta_inner) + math.sin(theta_outer))
    radius = math.hypot(p, q)
    if radius > 2.0 * body.lower_m:
        return None
    b1 = math.acos(min(1.0, radius / (2.0 * body.lower_m))) + math.atan2(p, q)
    return (-body.upper_m * math.sin(theta_inner) + body.lower_m * math.cos(b1),
            body.upper_m * math.cos(theta_inner) + body.lower_m * math.sin(b1))


def axles(joints: dict, body: Body) -> np.ndarray:
    """The two wheel centres in the body frame, [right, left] x (x, y, z). `joints`: angle by
    joint name. The right leg's inner motor is the rear one, the left leg's the front one."""
    out = np.zeros((2, 3))
    for k, (side, inner_x, sign, y) in enumerate((("right", body.rear_pivot_x_m, 1.0, -body.half_track_m),
                                                  ("left", body.front_pivot_x_m, -1.0, body.half_track_m))):
        planar = leg_forward_kinematics(joints[f"{side}_motor_inner"], joints[f"{side}_motor_outer"], body)
        if planar is None:
            planar = (0.5 * body.base_m, 0.17)
        out[k] = (inner_x + sign * planar[0], y, body.pivot_z_m - planar[1])
    return out


def rotation(roll, pitch, yaw) -> np.ndarray:
    """Body to world, ZYX."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def euler(quaternion_xyzw):
    """(roll, pitch, yaw) of a body-to-world quaternion (x, y, z, w)."""
    x, y, z, w = quaternion_xyzw
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def optical_to_body(mount: Mount) -> np.ndarray:
    """Rotation taking optical-frame vectors (z along the view, x right, y down) to the body."""
    level = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    return rotation(mount.roll_rad, mount.pitch_rad, mount.yaw_rad) @ level


class HeightProfile:
    def __init__(self, settings: Settings | None = None) -> None:
        self.s = s = settings or Settings()
        n = s.cells
        self.height = np.zeros((n, n), dtype=np.float32)
        self.seen = np.full((n, n), -1e9)              # time of the last look
        self.candidate = np.zeros((n, n), dtype=np.float32)   # a height one image saw, to be confirmed
        self.candidate_seen = np.full((n, n), -1e9)
        self.key_i = np.full((n, n), np.iinfo(np.int64).min, dtype=np.int64)   # the floor cell a slot holds
        self.key_j = np.full((n, n), np.iinfo(np.int64).min, dtype=np.int64)
        self.x = self.y = 0.0                          # the axle midpoint on the floor
        self.yaw = 0.0
        self.roll = self.pitch = 0.0
        self.floor = np.zeros(2)                       # under the right and the left wheel
        self.axles = axles({"right_motor_inner": 0.41, "right_motor_outer": 0.22,
                            "left_motor_inner": 0.22, "left_motor_outer": 0.41}, s.body)
        self.time = None
        self._history: list[tuple] = []                # (t, x, y, z, roll, pitch, yaw) of the body origin
        self._rays = None                              # per intrinsics
        self.images = 0
        self._fit_input, self._fit = None, None

    # Poses --------------------------------------------------------------

    def _origin(self):
        """The body origin in the floor frame from the pose, the legs and the floor under the wheels."""
        r = rotation(self.roll, self.pitch, self.yaw)
        mid = r @ self.axles.mean(0)
        z = float(np.mean(self.floor + self.s.body.wheel_radius_m - (self.axles @ r.T)[:, 2]))
        return np.array([self.x - mid[0], self.y - mid[1], z]), r

    def wheel_points(self):
        """Where the wheels touch the floor, [right, left] x (x, y)."""
        origin, r = self._origin()
        return (self.axles @ r.T)[:, :2] + origin[:2]

    def update_state(self, t: float, joints: dict, wheel_velocity, quaternion_xyzw, pitch_rate: float) -> None:
        """One sample of the robot's state. wheel_velocity: (right, left) rad/s, positive rolling
        forward; quaternion: the IMU's body-to-world attitude; pitch_rate: its gyro y, rad/s."""
        roll, pitch, yaw = euler(quaternion_xyzw)
        if self.time is not None and 0.0 < t - self.time < 0.5:
            dt = t - self.time
            # The wheels turn relative to the body: add its pitch rate (as the WBC's forward speed).
            speed = self.s.body.wheel_radius_m * (0.5 * (wheel_velocity[0] + wheel_velocity[1]) + pitch_rate)
            heading = self.yaw + 0.5 * math.remainder(yaw - self.yaw, 2 * math.pi)
            self.x += speed * math.cos(heading) * dt
            self.y += speed * math.sin(heading) * dt
        else:
            dt = 0.0
        self.time, self.roll, self.pitch, self.yaw = t, roll, pitch, yaw
        self.axles = axles(joints, self.s.body)
        # The floor under each wheel: the map there when it is known, else unchanged.
        height, known = self._floor_under(self.wheel_points(), t)
        step = self.s.floor_rate_m_s * dt
        self.floor = np.where(known, self.floor + np.clip(height - self.floor, -step, step), self.floor)
        origin, _ = self._origin()
        self._history.append((t, origin[0], origin[1], origin[2], roll, pitch, yaw))
        while self._history and self._history[0][0] < t - self.s.history_s:
            self._history.pop(0)

    def _pose_at(self, t: float):
        """The body origin and rotation at an earlier time, between the kept samples."""
        h = self._history
        if not h:
            return None
        if t <= h[0][0]:
            a = b = h[0]
        elif t >= h[-1][0]:
            a = b = h[-1]
        else:
            k = next(i for i in range(1, len(h)) if h[i][0] >= t)
            a, b = h[k - 1], h[k]
        w = 0.0 if b[0] == a[0] else (t - a[0]) / (b[0] - a[0])
        mix = lambda i: a[i] + w * (b[i] - a[i])
        yaw = a[6] + w * math.remainder(b[6] - a[6], 2 * math.pi)
        return np.array([mix(1), mix(2), mix(3)]), rotation(mix(4), mix(5), yaw)

    # Map ----------------------------------------------------------------

    def _slots(self, xy):
        i = np.floor(xy[..., 0] / self.s.cell_m).astype(np.int64)
        j = np.floor(xy[..., 1] / self.s.cell_m).astype(np.int64)
        return i, j, i % self.s.cells, j % self.s.cells

    def _lookup(self, xy, t):
        """(height, known) of the floor cells at points [n, 2]."""
        i, j, a, b = self._slots(np.asarray(xy))
        known = (self.key_i[a, b] == i) & (self.key_j[a, b] == j) & (t - self.seen[a, b] <= self.s.max_age_s)
        return self.height[a, b].astype(float), known

    def _floor_under(self, wheels, t):
        """(height, known) of the floor each wheel stands on: the lowest known cell from
        footprint_m behind the contact point to footprint_m ahead of it."""
        heading = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        offsets = np.array([-self.s.footprint_m, 0.0, self.s.footprint_m])
        points = wheels[:, None, :] + offsets[None, :, None] * heading[None, None, :]
        height, known = self._lookup(points.reshape(-1, 2), t)
        height, known = height.reshape(2, -1), known.reshape(2, -1)
        return np.where(known, height, np.inf).min(axis=1), known.any(axis=1)

    def add_depth(self, t: float, depth_m: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                  scale: float = 1.0) -> int:
        """One depth image taken at time t: depth along the view, 0 where there is none, in
        metres times `scale` (a D435's 16-bit image is millimetres: scale 0.001). Returns how
        many floor cells it refreshed."""
        s = self.s
        pose = self._pose_at(t)
        if pose is None:
            return 0
        origin, r_body = pose
        # Only the pixels used are converted: this runs beside the control loop.
        depth = np.asarray(depth_m)[::s.stride, ::s.stride].astype(np.float32)
        if scale != 1.0:
            depth *= scale
        shape = depth.shape + (fx, fy, cx, cy)
        if self._rays is None or self._rays[0] != shape:
            v, u = np.mgrid[0:depth_m.shape[0]:s.stride, 0:depth_m.shape[1]:s.stride]
            rays = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, dtype=float)], -1)
            # Into the body frame once: optical x, y, z -> body, scaled by the depth later.
            self._rays = (shape, (rays @ optical_to_body(s.mount).T).astype(np.float32))
        good = (depth > s.min_range_m) & (depth < s.max_range_m)
        if not good.any():
            return 0
        mount = np.array([s.mount.x_m, s.mount.y_m, s.mount.z_m])
        points = (self._rays[1][good] * depth[good][:, None] + mount) @ r_body.T + origin
        reference = float(np.mean(self.floor))
        half = 0.5 * s.cells * s.cell_m - s.cell_m
        centre = origin[:2] + s.window_ahead_m * r_body[:2, 0]
        keep = ((points[:, 2] > reference + s.band_m[0]) & (points[:, 2] < reference + s.band_m[1])
                & (np.abs(points[:, 0] - centre[0]) < half) & (np.abs(points[:, 1] - centre[1]) < half))
        points = points[keep]
        if not len(points):
            return 0
        i, j, a, b = self._slots(points[:, :2])
        flat = a * s.cells + b
        count = np.bincount(flat, minlength=s.cells * s.cells)
        z = points[:, 2] - reference                 # about the floor, for the squares' precision
        total = np.bincount(flat, weights=z, minlength=s.cells * s.cells)
        squares = np.bincount(flat, weights=z * z, minlength=s.cells * s.cells)
        hit = np.flatnonzero(count >= s.min_points)
        if not len(hit):
            return 0
        mean = total[hit] / count[hit]
        deviation = np.sqrt(np.maximum(squares[hit] / count[hit] - mean * mean, 0.0))
        # Never above the cell's highest point: half floor and half plank would overshoot.
        order = np.argsort(flat, kind="stable")
        top = np.maximum.reduceat(z[order], np.searchsorted(flat[order], hit))
        mean = (reference + np.minimum(mean + s.spread * deviation, top)).astype(np.float32)
        a_hit, b_hit = hit // s.cells, hit % s.cells
        # The floor cell each slot now stands for (the window is narrower than the slots wrap).
        first = np.zeros(s.cells * s.cells, dtype=np.int64)
        first[flat] = np.arange(len(flat))
        ci, cj = i[first[hit]], j[first[hit]]
        held = (self.key_i[a_hit, b_hit] == ci) & (self.key_j[a_hit, b_hit] == cj)
        same = held & (t - self.seen[a_hit, b_hit] <= s.max_age_s)
        # A cell that is new, or that this image moves by more than confirm_m, waits for the
        # next image to say the same.
        agrees = (held & (t - self.candidate_seen[a_hit, b_hit] <= s.confirm_s)
                  & (np.abs(self.candidate[a_hit, b_hit] - mean) <= s.confirm_m))
        small = same & (np.abs(self.height[a_hit, b_hit] - mean) <= s.confirm_m)
        accept = small | agrees
        self.candidate[a_hit, b_hit] = mean
        self.candidate_seen[a_hit, b_hit] = t
        # A slot that now stands for another floor cell forgets what it held.
        self.seen[a_hit, b_hit] = np.where(held, self.seen[a_hit, b_hit], -1e9)
        self.key_i[a_hit, b_hit], self.key_j[a_hit, b_hit] = ci, cj
        a_hit, b_hit, mean, same = a_hit[accept], b_hit[accept], mean[accept], same[accept]
        self.height[a_hit, b_hit] = np.where(same, (1 - s.blend) * self.height[a_hit, b_hit] + s.blend * mean, mean)
        self.seen[a_hit, b_hit] = t
        self.images += 1
        self._fit_input, self._fit = (points, origin, r_body), False
        return int(len(a_hit))

    @property
    def last_fit(self):
        """A plane through the floor points of the last image 0.6-1.6 m ahead: on a flat floor
        its slopes are the mount's pitch and roll errors and its offset the height error.
        (pitch, roll, offset, points), or None. Computed when asked for, not per image."""
        if self._fit is False:
            self._fit = self._floor_fit(*self._fit_input)
        return self._fit

    def _floor_fit(self, points, origin, r_body):
        forward = r_body[:2, 0] / max(np.linalg.norm(r_body[:2, 0]), 1e-9)
        left = np.array([-forward[1], forward[0]])
        rel = points[:, :2] - origin[:2]
        along, across = rel @ forward, rel @ left
        z = points[:, 2] - float(np.mean(self.floor))
        use = (along > 0.6) & (along < 1.6) & (np.abs(across) < 0.4) & (np.abs(z) < 0.08)
        if use.sum() < 200:
            return None
        basis = np.stack([along[use], across[use], np.ones(int(use.sum()))], 1)
        (slope_x, slope_y, offset), *_ = np.linalg.lstsq(basis, z[use], rcond=None)
        # A camera that really looks further down than the mount says sees the floor rise ahead.
        return float(math.atan(slope_x)), float(math.atan(slope_y)), float(offset), int(use.sum())

    # Output -------------------------------------------------------------

    def profile(self, t: float | None = None):
        """(heights [2, k], known [2, k]): right then left track, at the layout's distances ahead
        of each wheel along the heading, relative to the floor under that wheel; 0 where unknown."""
        t = self.time if t is None else t
        layout = self.s.layout
        wheels = self.wheel_points()
        heading = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        distances = np.asarray(layout.distances_m)
        points = wheels[:, None, :] + distances[None, :, None] * heading[None, None, :]
        height, known = self._lookup(points.reshape(-1, 2), t)
        height = height.reshape(2, -1) - self.floor[:, None]
        known = known.reshape(2, -1)
        if self.s.shadow_m > 0.0 and not known.all():
            # Every cell along each track up to the last sample, for the obstacle's shadow.
            steps = np.arange(self.s.cell_m, distances[-1] + self.s.cell_m, self.s.cell_m)
            line = wheels[:, None, :] + steps[None, :, None] * heading[None, None, :]
            line_h, line_known = self._lookup(line.reshape(-1, 2), t)
            line_h = line_h.reshape(2, -1) - self.floor[:, None]
            line_known = line_known.reshape(2, -1)
            for w in range(2):
                for k in np.flatnonzero(~known[w]):
                    before = np.flatnonzero(line_known[w] & (steps < distances[k])
                                            & (steps >= distances[k] - self.s.shadow_m))
                    if len(before) and line_h[w, before[-1]] > self.s.obstacle_m:
                        height[w, k], known[w, k] = line_h[w, before[-1]], True
        return np.where(known, np.clip(height, *layout.clip_m), 0.0), known
