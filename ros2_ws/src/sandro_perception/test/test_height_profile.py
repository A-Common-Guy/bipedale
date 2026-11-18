"""height_profile.py against rendered depth: a pinhole camera on the robot's mount looking at
a floor with boxes, the robot driven over it kinematically."""

import math

import numpy as np
import pytest

from sandro_perception.height_profile import (Body, HeightProfile, Mount, Settings, axles,
                                              optical_to_body, rotation)

W, H, FX, FY, CX, CY = 424, 240, 215.0, 215.0, 212.0, 120.0     # a D435 at 424 x 240
STAND = {"right_motor_inner": 0.4113, "right_motor_outer": 0.2171,
         "left_motor_inner": 0.2152, "left_motor_outer": 0.4129}
PLANK = (1.5, 1.7, -0.5, 0.0, 0.03)      # x0, x1, y0, y1, height: under the right track only


def floor_height(x, y, boxes):
    return max([h for x0, x1, y0, y1, h in boxes if x0 <= x <= x1 and y0 <= y <= y1], default=0.0)


def render(origin, r_body, mount: Mount, boxes, noise=0.0, rng=None):
    """Depth along the view (m) of a floor at z = 0 with boxes on it; 0 beyond 6 m."""
    v, u = np.mgrid[0:H, 0:W]
    rays = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones((H, W))], -1) @ (r_body @ optical_to_body(mount)).T
    eye = origin + r_body @ np.array([mount.x_m, mount.y_m, mount.z_m])
    depth = np.full((H, W), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = -eye[2] / rays[..., 2]
        depth = np.where((rays[..., 2] < 0) & (t > 0), t, depth)
        for x0, x1, y0, y1, h in boxes:
            lo, hi = np.array([x0, y0, -1.0]), np.array([x1, y1, h])
            t0, t1 = (lo - eye) / rays, (hi - eye) / rays
            near, far = np.minimum(t0, t1).max(-1), np.maximum(t0, t1).min(-1)
            depth = np.where((near < far) & (near > 0) & (near < depth), near, depth)
    depth = np.where(depth < 6.0, depth, 0.0)
    if noise:
        depth = np.where(depth > 0, depth + rng.normal(0.0, noise, depth.shape) * depth ** 2, 0.0)
    return depth.astype(np.float32)


class Robot:
    """The true robot: where it is, and the pose its wheels on the floor give its body."""

    def __init__(self, boxes=(), body=Body()):
        self.boxes, self.body = list(boxes), body
        self.x = self.y = self.yaw = 0.0
        self.pitch = 0.0
        self.axles = axles(STAND, body)

    def pose(self):
        wheels = np.array([[self.x, self.y]] * 2) + np.outer(self.axles[:, 1], [-math.sin(self.yaw), math.cos(self.yaw)])
        floor = np.array([floor_height(*w, self.boxes) for w in wheels])
        roll = math.atan2(floor[1] - floor[0], 2 * self.body.half_track_m)     # the left side higher: positive
        r = rotation(roll, self.pitch, self.yaw)
        mid = r @ self.axles.mean(0)
        z = float(np.mean(floor + self.body.wheel_radius_m - (self.axles @ r.T)[:, 2]))
        return np.array([self.x - mid[0], self.y - mid[1], z]), r, roll

    def quaternion(self, roll):
        cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(self.pitch / 2),
                                  math.sin(self.pitch / 2), math.cos(self.yaw / 2), math.sin(self.yaw / 2))
        return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def drive(robot, profile, seconds, speed=0.0, yaw_rate=0.0, t0=0.0, pitch=lambda t: 0.0, delay=0.0,
          true_mount=None, noise=0.0, rate=50, camera_hz=15):
    """Advance the true robot and feed the estimator its sensors: state at `rate`, images at
    camera_hz, each `delay` s late."""
    rng = np.random.default_rng(0)
    pending, dt, t = [], 1.0 / rate, t0
    every = round(rate / camera_hz)
    for step in range(int(seconds * rate)):
        t = t0 + step * dt
        previous = robot.pitch
        robot.pitch = pitch(t)
        robot.x += speed * math.cos(robot.yaw) * dt
        robot.y += speed * math.sin(robot.yaw) * dt
        robot.yaw += yaw_rate * dt
        origin, r, roll = robot.pose()
        pitch_rate = (robot.pitch - previous) / dt
        # The wheels roll for the distance travelled, relative to a body that pitches.
        wheel = speed / robot.body.wheel_radius_m - pitch_rate
        profile.update_state(t, STAND, (wheel, wheel), robot.quaternion(roll), pitch_rate)
        if step % every == 0:
            pending.append((t, render(origin, r, true_mount or profile.s.mount, robot.boxes, noise, rng)))
        while pending and pending[0][0] <= t - delay:
            stamp, image = pending.pop(0)
            profile.add_depth(stamp, image, FX, FY, CX, CY)
    return t + dt


def test_the_legs_put_the_axles_where_the_model_has_them():
    points = axles(STAND, Body())
    # sandro_description's stand keyframe, wheel joints in the IMU's level frame.
    np.testing.assert_allclose(points[0], [0.0207, -0.152, -0.2776], atol=1.5e-3)
    np.testing.assert_allclose(points[1], [0.0207, 0.152, -0.2776], atol=1.5e-3)


def test_a_flat_floor_reads_zero_and_the_near_floor_is_unknown_until_driven_over():
    robot, profile = Robot(), HeightProfile()
    t = drive(robot, profile, 1.0)
    heights, known = profile.profile()
    distances = np.array(profile.s.layout.distances_m)
    # Level at 0.34 m a D435 sees the floor from about 0.6 m: nearer is not known yet.
    assert not known[:, distances < 0.5].any() and known[:, distances > 0.75].all()
    assert np.abs(heights).max() < 0.005
    pitch, roll, offset, count = profile.last_fit
    assert abs(pitch) < 2e-3 and abs(roll) < 2e-3 and abs(offset) < 2e-3 and count > 200
    # 0.8 m on, what was seen is under the wheels: everything is known, still flat.
    drive(robot, profile, 1.6, speed=0.5, t0=t)
    heights, known = profile.profile()
    assert known.all() and np.abs(heights).max() < 0.005


def test_a_plank_is_remembered_through_the_blind_zone_and_the_floor_follows_the_wheel():
    robot, profile = Robot([PLANK]), HeightProfile()
    t = drive(robot, profile, 0.5)
    # To 0.3 m before the plank: it lies 0.3-0.5 m ahead of the right wheel, out of view.
    t = drive(robot, profile, (1.2 - 0.0207) / 0.5, speed=0.5, t0=t)
    heights, known = profile.profile()
    k = profile.s.layout.distances_m.index(0.45)
    assert known[:, k].all()
    assert heights[0, k] == pytest.approx(0.03, abs=0.008)      # right track: on the plank
    assert abs(heights[1, k]) < 0.005                           # left track: floor
    assert abs(heights[0, profile.s.layout.distances_m.index(0.80)]) < 0.006     # beyond it: floor
    # Onto the plank with the right wheel: the floor ahead of it is now 3 cm down.
    t = drive(robot, profile, 0.4 / 0.5, speed=0.5, t0=t)
    assert profile.floor[0] == pytest.approx(0.03, abs=0.008) and abs(profile.floor[1]) < 0.005
    heights, known = profile.profile()
    k = profile.s.layout.distances_m.index(0.30)                # 0.3 m ahead: past the plank
    assert known[0, k] and heights[0, k] == pytest.approx(-0.03, abs=0.008)
    assert abs(heights[1, k]) < 0.006
    # And off it again.
    drive(robot, profile, 0.5 / 0.5, speed=0.5, t0=t)
    assert np.abs(profile.floor).max() < 0.008


def test_late_images_of_a_pitching_body_land_where_they_were_taken():
    """The body rocks 3 degrees at 1 Hz while driving and the images arrive 60 ms late: at
    1.3 m a 60 ms old attitude would be 2 cm off."""
    robot, profile = Robot(), HeightProfile()
    rock = lambda t: math.radians(3.0) * math.sin(2 * math.pi * t)
    drive(robot, profile, 3.0, speed=0.4, pitch=rock, delay=0.06, noise=0.002)
    heights, known = profile.profile()
    assert known.all() and np.abs(heights).max() < 0.008


def test_the_floor_beside_the_robot_is_known_only_if_it_was_seen_on_the_way():
    # Turning on the spot from the start: the floor now ahead of the wheels was never in view.
    robot, profile = Robot(), HeightProfile()
    t = drive(robot, profile, 1.0)
    drive(robot, profile, 1.6, yaw_rate=1.0, t0=t)              # about 90 degrees
    heights, known = profile.profile()
    distances = np.array(profile.s.layout.distances_m)
    assert not known[:, distances < 0.4].any() and known[:, distances > 0.8].all()
    assert np.abs(heights).max() < 0.006
    # After driving there, the same turn finds it in the memory: the view is 87 degrees wide.
    robot, profile = Robot(), HeightProfile()
    t = drive(robot, profile, 2.5, speed=0.5)
    drive(robot, profile, 1.6, yaw_rate=1.0, t0=t)
    heights, known = profile.profile()
    assert known.all() and np.abs(heights).max() < 0.006
    # And forgotten when it is old.
    profile.update_state(profile.time + profile.s.max_age_s + 1.0, STAND, (0.0, 0.0),
                         robot.quaternion(0.0), 0.0)
    assert not profile.profile()[1].any()


def test_the_floor_fit_shows_a_mount_that_looks_further_down_than_configured():
    robot, profile = Robot(), HeightProfile()
    real = Mount(pitch_rad=profile.s.mount.pitch_rad + math.radians(2.0))
    drive(robot, profile, 1.0, true_mount=real)
    pitch, roll, offset, _ = profile.last_fit
    assert pitch == pytest.approx(math.radians(2.0), abs=math.radians(0.3)) and abs(roll) < 5e-3


def test_a_camera_tilted_down_20_degrees_sees_the_floor_from_nearer():
    """Level, the floor is in view from about 0.7 m ahead of the wheels; tilted 20 degrees
    down, from about 0.4 m."""
    settings = Settings(mount=Mount(pitch_rad=math.radians(20.0)))
    robot, profile = Robot(), HeightProfile(settings)
    drive(robot, profile, 1.0)
    heights, known = profile.profile()
    distances = np.array(settings.layout.distances_m)
    assert known[:, distances >= 0.45].all() and not known[:, distances <= 0.3].any()
    assert np.abs(heights).max() < 0.005
