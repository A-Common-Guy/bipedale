"""The robot's perception in the evaluator's loop: depth images rendered from the camera
mount, through sandro_perception's own height_profile.py, into the policy's observation.

`--sim depth=1` selects it over the evaluator's emulation of the profile (true floor heights
with the training's noise). What it adds is everything the emulation leaves out: the map's
memory, the floor under the wheels taken from that map, images placed with the pose the
robot measured, cells averaged over the faces of what stands on the floor, depth noise that
grows with range, and whatever the room holds beyond the course.

The camera is the node's: a D435 at 424 x 240, 15 images a second, each delivered
`delay_s` after it was taken; the robot's state at 50 Hz; the profile out at 25 Hz.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import mujoco
import numpy as np

from . import interface

PERCEPTION = Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "sandro_perception"
WIDTH, HEIGHT, FOCAL = 424, 240, 215.0
LEGS = ("right_motor_inner", "right_motor_outer", "left_motor_inner", "left_motor_outer")


def height_profile_module():
    if str(PERCEPTION) not in sys.path:
        sys.path.insert(0, str(PERCEPTION))
    from sandro_perception import height_profile
    return height_profile


def quaternion_xyzw(r: np.ndarray):
    """Of a rotation matrix (body to world)."""
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(r, float).reshape(9))
    return (q[1], q[2], q[3], q[0])


class DepthPipeline:
    def __init__(self, m: mujoco.MjModel, rng, image_hz: float = 15.0, delay_s: float = 0.06,
                 noise: float = 0.006, dropout: float = 0.02, settings=None, mount_error_rad: float = 0.0):
        hp = height_profile_module()
        self.hp = hp
        self.settings = settings or hp.Settings()
        self.profile = hp.HeightProfile(self.settings)
        self.m, self.rng = m, rng
        self.image_period, self.delay_s, self.noise, self.dropout = 1.0 / image_hz, delay_s, noise, dropout
        os.environ.setdefault("MUJOCO_GL", "egl")
        self.renderer = mujoco.Renderer(m, HEIGHT, WIDTH)
        self.renderer.enable_depth_rendering()
        self.free = mujoco.MjvCamera()
        # The true camera: the configured mount, off by mount_error_rad in pitch (a bracket
        # that is not what the CAD says, which the node cannot know).
        mount = self.settings.mount
        true = hp.Mount(**{**mount.__dict__, "pitch_rad": mount.pitch_rad + mount_error_rad})
        self.mount_xyz = np.array([mount.x_m, mount.y_m, mount.z_m])
        self.optical = hp.optical_to_body(true)
        self.next_state = self.next_image = self.next_publish = 0.0
        self.pending: list[tuple] = []
        self.observation = None
        self.images = 0

    def render(self, d: mujoco.MjData, origin, r_body) -> np.ndarray:
        """Depth along the view, m, from the mount on a body at (origin, r_body)."""
        self.renderer.update_scene(d, self.free)
        r = r_body @ self.optical                          # optical axes in the world, as columns
        eye = origin + r_body @ self.mount_xyz
        near = self.m.vis.map.znear * self.m.stat.extent
        for camera in self.renderer.scene.camera:
            camera.pos[:], camera.forward[:], camera.up[:] = eye, r[:, 2], -r[:, 1]
            camera.frustum_center = 0.0
            camera.frustum_width = near * 0.5 * WIDTH / FOCAL
            camera.frustum_top = near * 0.5 * HEIGHT / FOCAL
            camera.frustum_bottom = -camera.frustum_top
        depth = self.renderer.render().astype(np.float32)
        depth[depth > 6.0] = 0.0
        if self.noise:
            # A stereo camera's error grows with the square of the range.
            depth = np.where(depth > 0, depth + self.rng.normal(0.0, self.noise, depth.shape) * depth ** 2, 0.0)
        if self.dropout:
            depth = np.where(self.rng.uniform(size=depth.shape) < self.dropout, 0.0, depth)
        return depth.astype(np.float32)

    def update(self, now: float, d: mujoco.MjData, true_origin, true_r_body, legs, wheel_velocity,
               measured_r_body, pitch_rate: float):
        """One control cycle. `legs`: the four leg angles in LEGS' order, as measured;
        wheel_velocity (right, left); measured_r_body: the attitude the IMU reports. Returns
        the profile observation (interface.profile_observation) as last published."""
        if now >= self.next_state:
            self.next_state = now + 0.02
            self.profile.update_state(now, dict(zip(LEGS, legs)), wheel_velocity,
                                      quaternion_xyzw(measured_r_body), pitch_rate)
        if now >= self.next_image:
            self.next_image = now + self.image_period
            self.pending.append((now, self.render(d, np.asarray(true_origin), np.asarray(true_r_body))))
        while self.pending and now >= self.pending[0][0] + self.delay_s:
            stamp, depth = self.pending.pop(0)
            self.profile.add_depth(stamp, depth, FOCAL, FOCAL, 0.5 * WIDTH, 0.5 * HEIGHT)
            self.images += 1
        if now >= self.next_publish and self.profile.time is not None:
            self.next_publish = now + 0.04
            heights, known = self.profile.profile()
            self.observation = interface.profile_observation(heights.reshape(-1), known.reshape(-1).astype(float))
        return self.observation
