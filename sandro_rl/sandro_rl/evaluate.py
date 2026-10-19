"""Run an exported policy in CPU MuJoCo with the full latency emulation.

    uv run python -m sandro_rl.evaluate policies/flat_tracking --scenario hardware
    uv run python -m sandro_rl.evaluate policies/flat_tracking --compare      # vs the WBC, all profiles
    uv run python -m sandro_rl.evaluate policies/flat_tracking --scenario default --video out.mp4

The loop is sandro_sim/tools/closed_loop/closed_loop.cpp with the
whole-body controller replaced by the policy:
  - MuJoCo on scene_ros2.xml from the stand keyframe, 0.03 s unpowered;
  - the controller manager's wake-up rule (first physics step at or past the
    last wake plus 1/300 s);
  - LatencySystem's read: a DelayLine per state interface (joint position,
    velocity and effort on the joint's staggered broadcast clock, the ten IMU
    values on the IMU clock), the MJCF IMU noise added per read;
  - LatencySystem's write and MitDrivePlugin: every frame onto a DriveBus,
    due command_delay later, the MIT law on the live joint state before every
    physics step;
  - drive_test.py's commands at 20 Hz from closed_loop.py's scenarios, a
    3 s settle after engagement, the same pushes.
The deployment-side controller (interface.PolicyInterface) runs the policy
every third cycle and writes its frames every cycle. Recordings have
drive_test.py's layout plus three truth columns, so compare_runs.py reads
them next to the whole-body controller's.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import tempfile
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from . import checkpoint, kinematics, sources
from .interface import PolicyInterface, quat_to_mat
from .robot import RobotSpec, rotation_x
from .sources import JOINTS

CLOSED_LOOP_DIR = sources.SRC / "sandro_sim" / "tools"
sys.path.insert(0, str(CLOSED_LOOP_DIR))
import closed_loop  # noqa: E402  (scenarios, pushes, gates and the WBC runner)
import compare_runs  # noqa: E402  (imported by closed_loop from sandro_bringup/tools)

DIAG_KEYS = [
    "engaged", "status_mask", "pitch_rad", "pitch_reference_rad", "pitch_rate_rad_s",
    "roll_rad", "roll_reference_rad", "forward_velocity_m_s", "velocity_reference_m_s",
    "position_error_m", "yaw_rate_rad_s", "wheel_yaw_rate_rad_s", "yaw_rate_reference_rad_s",
    "leg_height_m", "height_reference_m", "height_differential_m", "height_support_n",
    "leg_offset_m", "leg_offset_reference_m", "leg_offset_command_m", "leg_offset_model_m",
    "leg_offset_state_m", "leg_offset_rate_m_s", "trim_m", "balance_torque_nm",
    "yaw_torque_nm", "yaw_feedforward_nm", "truth_pitch_rad", "truth_speed_m_s"]
TRUTH_KEYS = ["truth_forward_m_s", "truth_yaw_rate_rad_s", "truth_height_m", "truth_roll_rad"]
IMU_COLUMNS = ["imu_qx", "imu_qy", "imu_qz", "imu_qw", "imu_gx", "imu_gy", "imu_gz", "imu_ax", "imu_ay", "imu_az"]

# Latency profiles of the comparison. "uart_imu" is the gate closed_loop.py
# uses for the BNO085 UART path; "worst" stacks the far ends of the training
# ranges that were measured on the robot (SIM_TO_REAL.md).
PROFILES = {
    "nominal": {},
    "uart_imu": dict(closed_loop.UART_IMU),
    "worst": {"imu_rate": "30", "imu_delay": "0.025", "joint_delay": "0.005", "command_delay": "0.003"},
}


class DelayLine:
    """sandro_sim/delay_line.hpp, line for line."""

    CAPACITY = 256

    def __init__(self, delay_s=0.0, rate_hz=0.0, phase_s=0.0, interpolate=False):
        self.delay = delay_s if math.isfinite(delay_s) and delay_s > 0 else 0.0
        self.period = 1.0 / rate_hz if math.isfinite(rate_hz) and rate_hz > 0 else 0.0
        self.phase = phase_s if math.isfinite(phase_s) else 0.0
        self.interpolate = interpolate
        self.samples: deque = deque(maxlen=self.CAPACITY)
        self.output = None
        self.last_index = None

    def update(self, now, value):
        if self.delay == 0.0 and self.period == 0.0:
            return value
        self.samples.append((now, value))
        sampled_at = now - self.delay
        index = None
        if self.period > 0:
            index = math.floor((now - self.delay - self.phase) / self.period + 1e-9)
            if self.output is not None and self.last_index == index:
                return self.output
            sampled_at = self.phase + index * self.period
        delayed = self._value_at(sampled_at)
        if delayed is not None:
            self.output = delayed
            self.last_index = index
        elif self.output is None:
            self.output = value
        return self.output

    def _value_at(self, t):
        before = None
        for i, (ts, _) in enumerate(self.samples):
            if ts <= t:
                before = i
            else:
                break
        if before is None:
            return None
        ta, va = self.samples[before]
        if not self.interpolate or before + 1 >= len(self.samples):
            return va
        tb, vb = self.samples[before + 1]
        span = tb - ta
        if not span > 0:
            return va
        return va + (vb - va) * (t - ta) / span


class DriveBus:
    """sandro_sim/drive_bus.hpp: a frame is due command_delay after its stamp."""

    def __init__(self, n, command_delay_s):
        self.delay = max(command_delay_s, 0.0)
        self.pending = [deque(maxlen=64) for _ in range(n)]
        self.current = [None] * n

    def push(self, joint, stamp, frame):
        self.pending[joint].append((stamp, frame))

    def due(self, now):
        for i, queue in enumerate(self.pending):
            while queue and queue[0][0] + self.delay <= now + 1e-9:
                self.current[i] = queue.popleft()[1]
        return self.current

    def clear(self):
        for q in self.pending:
            q.clear()
        self.current = [None] * len(self.current)


@dataclass
class SimOptions:
    imu_delay_s: float
    imu_rate_hz: float
    joint_delay_s: float
    joint_rate_hz: float
    joint_stagger: bool
    command_delay_s: float
    imu_noise: bool = True
    unpowered_s: float = 0.03
    settle_s: float = 3.0
    command_rate_hz: float = 20.0
    diagnostics_rate_hz: float = 50.0
    seed: int = 1

    @classmethod
    def from_closed_loop(cls, sim: dict[str, str], latency: sources.Latency) -> "SimOptions":
        o = cls(latency.imu_delay_s, latency.imu_rate_hz, latency.joint_delay_s, latency.joint_rate_hz,
                latency.joint_stagger, latency.command_delay_s)
        names = {"imu_delay": "imu_delay_s", "imu_rate": "imu_rate_hz", "joint_delay": "joint_delay_s",
                 "joint_rate": "joint_rate_hz", "command_delay": "command_delay_s", "settle": "settle_s",
                 "seed": "seed"}
        for key, value in sim.items():
            if key == "joint_stagger":
                o.joint_stagger = value in ("true", "1")
            elif key == "imu_noise":
                o.imu_noise = value in ("true", "1")
            elif key in names:
                setattr(o, names[key], type(getattr(o, names[key]))(float(value)))
            else:
                raise ValueError(f"simulation option '{key}' is not emulated by the policy evaluator")
        return o


# Scenarios beyond closed_loop.py's, for policies trained past the WBC's
# envelope: drive_test.py's "duration,forward,yaw[,height_rate]" plus an
# optional fifth field, the body roll target in rad (roll-commanded policies).
EXTRA_SCENARIOS = {
    "agile": ["2,0,0", "3,0.6,0", "3,1.0,0", "3,0.8,2.0", "2,0,-3.0", "3,-0.5,0", "3,0,0,-0.1",
              "3,0.5,1.5", "3,0,0,0.1", "2,0,0"],
    "course": ["2,0,0", "3,0.5,0", "6,0.8,0", "2,0,0", "2,0,2.5", "4,0.6,0", "2,0,0"],
    # Fields: duration, forward, yaw, height rate, roll. Roll is feasible up
    # to +-0.3 rad only near mid height (0.13 m), so the rolls happen there.
    "expressive": ["2,0,0", "3,0,0,-0.013,0", "3,0,0,0,0.3", "3,0,0,0,-0.3", "2,0,0,0,0",
                   "3,0,0,-0.013,0", "3,0,0,0.033,0", "4,0.4,1.0,0,0", "3,0,0,-0.017,0",
                   "3,0.3,0,0,-0.25", "3,-0.3,0,0,0.25", "2,0,0,0,0"],
}
SCENARIOS = {**closed_loop.SCENARIOS, **EXTRA_SCENARIOS}

# The obstacle course of `--course`, along the start heading: planks 0.1 m
# deep and 1.2 m wide at these distances (m) and heights (m), then a hill
# (10 degree ramps, 0.6 m plateau, 1.6 m wide) 8 cm high centred at 5.0 m.
COURSE = [(1.0, 0.010), (1.8, 0.015), (2.6, 0.020), (3.4, 0.025)]
COURSE_HILL = {"distance_m": 5.0, "height_m": 0.08, "slope_deg": 10.0, "plateau_m": 0.6, "width_m": 1.6}


def course_model(spec: RobotSpec, terrain_cfg: dict | None = None) -> mujoco.MjModel:
    """The course compiled in place (static geoms collide only where compiled)."""
    from . import terrain

    d = mujoco.MjData(spec.model)
    mujoco.mj_resetDataKeyframe(spec.model, d, mujoco.mj_name2id(spec.model, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    mujoco.mj_forward(spec.model, d)
    r = d.site_xmat[spec.imu_site].reshape(3, 3)
    heading = np.array([r[0, 0], r[1, 0], 0.0])
    heading /= np.linalg.norm(heading)
    start = d.xanchor[spec.wheel_joint_ids].mean(axis=0)
    yaw = math.atan2(heading[1], heading[0])
    quat = [math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)]
    items, poses = [], []
    for i, (distance, height) in enumerate(COURSE):
        plank = terrain.boxes({"type": "obstacles", "planks": {
            "count": 1, "height_m": [height, height], "depth_m": [0.1, 0.1], "width_m": [1.2, 1.2]}})[0]
        plank["name"] = f"{terrain.OBSTACLE_PREFIX}course_plank{i}"
        centre = start + distance * heading
        items.append(plank)
        poses.append(([centre[0], centre[1], height], quat))
    h = COURSE_HILL
    hill = terrain.boxes({"type": "obstacles", "hills": {
        "count": 1, "height_m": [h["height_m"]] * 2, "slope_deg": [h["slope_deg"]] * 2,
        "plateau_m": [h["plateau_m"]] * 2, "width_m": [h["width_m"]] * 2}})[0]
    hill["name"] = f"{terrain.OBSTACLE_PREFIX}course_hill"
    centre = start + h["distance_m"] * heading
    items.append(hill)
    poses.append(([centre[0], centre[1], h["height_m"]], quat))
    return terrain.static_course(spec, items, poses)


class OnnxPolicy:
    def __init__(self, path: Path):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        return self.session.run(None, {"obs": obs.astype(np.float32)[None]})[0][0].astype(float)


class PolicyController:
    """The deployment-side controller: what a ros2_control controller would run."""

    def __init__(self, iface: PolicyInterface, policy, fall_angle_rad: float):
        self.iface = iface
        self.policy = policy
        self.fall_angle = fall_angle_rad
        self.cycle = 0
        self.last_action = np.zeros(len(JOINTS))
        self.frames = iface.frames(self.last_action)
        self.history = None
        self.ref = None
        self.fallen = False

    def reset(self, quat, gyro, accel, q, v, height_target):
        heights = self._heights(q)
        self.ref = np.array([0.0, 0.0, float(np.clip(np.mean(heights), self.iface.h_min, self.iface.h_max)),
                             0.0][: 4 if self.iface.has_roll else 3])
        self.height_command = height_target
        frame = self.iface.frame(quat, gyro, accel, q, v, self.last_action, self.ref)
        self.history = np.tile(frame, (self.iface.history, 1))
        self.cycle = 0

    def _heights(self, q):
        lim = self.iface.spec.limits
        h, _, _ = kinematics.leg_state(np.asarray(q)[self.iface.leg_idx], lim.leg_geometry,
                                       lim.fore_aft_neutral_m, np)
        return h

    def update(self, quat, gyro, accel, q, v, command, period_s):
        """command: (forward m/s, yaw rad/s, height rate m/s[, roll rad]); the first three as /cmd_vel."""
        r = self.iface.body_rotation(quat)
        tilt = math.acos(float(np.clip(r[2, 2], -1.0, 1.0)))
        if tilt > self.fall_angle:
            self.fallen = True
        if self.fallen:
            # Safety fallback: drives limp with damping, as the WBC does on a fall.
            f = np.zeros((5, len(JOINTS)))
            f[3, :] = 0.5
            self.frames = f
            return self.frames
        if self.cycle % self.iface.cycles_per_action == 0:
            i = self.iface
            self.height_command = float(np.clip(self.height_command + np.clip(command[2], -i.h_rate, i.h_rate)
                                                * i.dt, i.h_min, i.h_max))
            target = np.array([np.clip(command[0], -i.v_max, i.v_max), np.clip(command[1], -i.yaw_max, i.yaw_max),
                               self.height_command, command[3] if len(command) > 3 else 0.0])
            self.ref = i.shape(self.ref, target)
            frame = i.frame(quat, gyro, accel, q, v, self.last_action, self.ref)
            self.history = np.concatenate([frame[None], self.history[:-1]], axis=0)
            action = np.clip(self.policy(self.history.reshape(-1)), -i.clip, i.clip)
            self.last_action = action
            self.frames = i.frames(action)
        self.cycle += 1
        return self.frames


def run(policy_dir: Path, scenario: str, sim: dict[str, str], output: Path,
        video: Path | None = None, video_fps: int = 30, seed: int = 1, course: bool = False) -> tuple[int, Path]:
    policy_dir = Path(policy_dir)
    _, meta = checkpoint.load(policy_dir)
    checkpoint.check_model(meta)
    config = meta["config"]
    spec = RobotSpec.load()
    iface = PolicyInterface(config, spec)
    lim = spec.limits
    options = SimOptions.from_closed_loop({"seed": str(seed), **sim}, spec.latency)
    rng = np.random.default_rng(options.seed)
    controller = PolicyController(iface, OnnxPolicy(policy_dir / "policy.onnx"), lim.fall_angle_rad)

    m = course_model(spec) if course else mujoco.MjModel.from_xml_path(str(spec.scene))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand"))
    mujoco.mj_forward(m, d)
    qadr, dadr, act = spec.joint_qpos, spec.joint_dof, spec.joint_actuator
    sensors = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, s) for s in ("imu_quat", "imu_gyro", "imu_accel")]
    sensor_of = [0, 0, 0, 0, 1, 1, 1, 2, 2, 2]
    torso, imu_site = spec.torso_body, spec.imu_site
    wheel_joints = spec.wheel_joint_ids
    body_from_imu = rotation_x(-spec.imu_roll_bias_rad)

    joint_period = 1.0 / options.joint_rate_hz if options.joint_rate_hz > 0 else 0.0
    joint_lines = [[DelayLine(options.joint_delay_s, options.joint_rate_hz,
                              joint_period * i / len(JOINTS) if options.joint_stagger else 0.0, True)
                    for _ in range(3)] for i in range(len(JOINTS))]
    imu_lines = [DelayLine(options.imu_delay_s, options.imu_rate_hz, 0.0, True) for _ in range(10)]
    fallback_lines = [[DelayLine(options.command_delay_s, 0.0) for _ in range(5)] for _ in range(len(JOINTS))]
    bus = DriveBus(len(JOINTS), options.command_delay_s)
    staged = np.zeros(m.nu)
    limits = iface.torque_limit

    phases = [[float(x) for x in p.split(",")] + [0.0] * (5 - len(p.split(","))) for p in SCENARIOS[scenario]]
    pushes = [[float(x) for x in p.split(",")] for p in closed_loop.SCENARIO_PUSHES.get(scenario, [])]
    total_phase_s = sum(p[0] for p in phases)

    renderer = None
    frames_out = []
    if video is not None:
        os.environ.setdefault("MUJOCO_GL", "egl")
        renderer = mujoco.Renderer(m, 480, 640)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        camera.trackbodyid = torso
        camera.distance, camera.elevation, camera.azimuth = 1.2, -15.0, 120.0
        last_frame_t = -1.0

    def step(push_force):
        nonlocal last_frame_t
        d.ctrl[:] = staged
        frames = bus.due(d.time)
        for i in range(len(JOINTS)):
            if frames[i] is not None:
                d.ctrl[act[i]] = iface.mit_torque(frames[i], d.qpos[qadr[i]], d.qvel[dadr[i]], limits[i])
        d.xfrc_applied[torso, :] = 0.0
        if push_force != 0.0:
            r = d.xmat[torso].reshape(3, 3)
            n = math.hypot(r[0, 0], r[1, 0])
            if n > 1e-6:
                d.xfrc_applied[torso, 0] = push_force * r[0, 0] / n
                d.xfrc_applied[torso, 1] = push_force * r[1, 0] / n
        mujoco.mj_step(m, d)
        if renderer is not None and d.time - last_frame_t >= 1.0 / video_fps - 1e-9:
            last_frame_t = d.time
            renderer.update_scene(d, camera)
            frames_out.append(renderer.render())

    while d.time < options.unpowered_s - 1e-12:
        step(0.0)

    period_ns = int(1e9 / lim.control_rate_hz)
    step_ns = lambda: int(d.time * 1e9)
    last_ns = wake_ns = step_ns()
    activated = False
    recording_start = phase_start = engaged_at = last_diag = last_publish = -1.0
    published = [0.0] * 5
    received = [0.0] * 4
    phase_index = 0
    fall_time = -1.0
    finished = False
    rows = []
    axle_prev, t_prev = None, None

    while not finished:
        push = 0.0
        if phase_start >= 0:
            t = d.time - phase_start
            for start, force, duration in pushes:
                if start <= t < start + duration:
                    push += force
        step(push)
        now_ns = step_ns()
        if now_ns < wake_ns:
            continue
        now = now_ns * 1e-9
        period = (now_ns - last_ns) * 1e-9
        last_ns, wake_ns = now_ns, now_ns + period_ns

        # read()
        pos = np.array([joint_lines[i][0].update(now, d.qpos[qadr[i]]) for i in range(6)])
        vel = np.array([joint_lines[i][1].update(now, d.qvel[dadr[i]]) for i in range(6)])
        eff = np.array([joint_lines[i][2].update(now, d.actuator_force[act[i]]) for i in range(6)])
        qs = d.sensordata[m.sensor_adr[sensors[0]]:m.sensor_adr[sensors[0]] + 4]
        gs = d.sensordata[m.sensor_adr[sensors[1]]:m.sensor_adr[sensors[1]] + 3]
        acs = d.sensordata[m.sensor_adr[sensors[2]]:m.sensor_adr[sensors[2]] + 3]
        raw = np.array([qs[1], qs[2], qs[3], qs[0], *gs, *acs])
        if options.imu_noise:
            sig = np.array([m.sensor_noise[sensors[k]] for k in sensor_of])
            raw = raw + np.where(sig > 0, rng.normal(0.0, 1.0, 10) * sig, 0.0)
        imu = np.array([imu_lines[i].update(now, raw[i]) for i in range(10)])
        quat = np.array([imu[3], imu[0], imu[1], imu[2]])
        gyro, accel = imu[4:7], imu[7:10]

        if not activated:
            controller.reset(quat, gyro, accel, pos, vel, lim.height_target_m)
            activated = True
            continue

        if last_publish < 0 or now - last_publish >= 1.0 / options.command_rate_hz - 1e-9:
            last_publish = now
            if phase_start >= 0:
                elapsed, index = now - phase_start, 0
                while index < len(phases) and elapsed >= phases[index][0]:
                    elapsed -= phases[index][0]
                    index += 1
                phase_index = index
                published = phases[index] if index < len(phases) else [0.0] * 5
            else:
                published = [0.0] * 5
            if engaged_at >= 0:
                received = [published[1], published[2], published[3], published[4]]
        frames = controller.update(quat, gyro, accel, pos, vel, received, period)

        # write()
        for i in range(6):
            delayed = [fallback_lines[i][f].update(now, frames[f, i]) for f in range(5)]
            bus.push(i, now, frames[:, i].copy())
            staged[act[i]] = iface.mit_torque(delayed, d.qpos[qadr[i]], d.qvel[dadr[i]], limits[i])

        if engaged_at < 0:
            engaged_at = recording_start = now
        if phase_start < 0 and now - engaged_at >= options.settle_s:
            phase_start = now
        if controller.fallen and fall_time < 0:
            fall_time = now
        if phase_start >= 0 and now - phase_start >= total_phase_s:
            finished = True
        if fall_time >= 0 and now - fall_time > 0.5:
            finished = True

        phase = -1 if phase_start < 0 else phase_index
        base = {"t_s": f"{now - recording_start:.6g}", "phase": phase, "cmd_forward": published[1],
                "cmd_yaw": published[2], "cmd_height_rate": published[3], "cmd_roll": published[4]}
        if last_diag < 0 or now - last_diag >= 1.0 / options.diagnostics_rate_hz - 1e-9:
            last_diag = now
            r_body = iface.body_rotation(quat)
            r_imu = quat_to_mat(*(quat / np.linalg.norm(quat)))
            gyro_body = iface.body_from_imu_correction.T @ gyro
            pitch = math.atan2(-r_body[2, 0], math.hypot(r_body[0, 0], r_body[1, 0]))
            roll = math.atan2(r_body[2, 1], r_body[2, 2])
            radius = lim.wheel_radius_m
            wheels = vel[iface.wheel_idx]
            heights, offsets, _ = kinematics.leg_state(pos[iface.leg_idx], lim.leg_geometry,
                                                       lim.fore_aft_neutral_m, np)
            taus = np.array([iface.mit_torque(frames[:, i], pos[i], vel[i], limits[i]) for i in range(6)])
            w_r, w_l = taus[iface.wheel_idx]
            rt = d.site_xmat[imu_site].reshape(3, 3)
            truth_pitch = math.atan2(-rt[2, 0], math.hypot(rt[0, 0], rt[1, 0]))
            rb = rt @ body_from_imu
            truth_speed = math.hypot(d.qvel[0], d.qvel[1])
            axle = d.xanchor[wheel_joints].mean(axis=0)
            heading = np.array([rt[0, 0], rt[1, 0], 0.0])
            heading /= max(np.linalg.norm(heading), 1e-9)
            truth_forward = (float((axle - axle_prev) @ heading / (d.time - t_prev))
                             if axle_prev is not None and d.time > t_prev else 0.0)
            axle_prev, t_prev = axle.copy(), d.time
            gyro_true = d.sensordata[m.sensor_adr[sensors[1]]:m.sensor_adr[sensors[1]] + 3]
            truth_yaw = float((rt @ gyro_true)[2])
            true_heights, _, _ = kinematics.leg_state(d.qpos[qadr][iface.leg_idx], lim.leg_geometry,
                                                      lim.fore_aft_neutral_m, np)
            nan = math.nan
            values = {
                "engaged": 0.0 if controller.fallen else 1.0, "status_mask": 0.0, "pitch_rad": pitch,
                "pitch_reference_rad": nan, "pitch_rate_rad_s": gyro_body[1], "roll_rad": roll,
                "roll_reference_rad": controller.ref[3] if len(controller.ref) > 3 else nan,
                "forward_velocity_m_s": radius * (float(np.mean(wheels)) + gyro_body[1]),
                "velocity_reference_m_s": controller.ref[0], "position_error_m": nan,
                "yaw_rate_rad_s": float((r_imu @ gyro)[2]),
                "wheel_yaw_rate_rad_s": radius * (wheels[0] - wheels[1]) / lim.track_width_m,
                "yaw_rate_reference_rad_s": controller.ref[1], "leg_height_m": float(np.mean(heights)),
                "height_reference_m": controller.ref[2], "height_differential_m": float(heights[0] - heights[1]),
                "height_support_n": nan, "leg_offset_m": float(np.mean(offsets)),
                "leg_offset_reference_m": nan, "leg_offset_command_m": nan, "leg_offset_model_m": nan,
                "leg_offset_state_m": nan, "leg_offset_rate_m_s": nan, "trim_m": nan,
                "balance_torque_nm": float(w_r + w_l), "yaw_torque_nm": float(0.5 * (w_r - w_l)),
                "yaw_feedforward_nm": nan, "truth_pitch_rad": truth_pitch, "truth_speed_m_s": truth_speed,
                "truth_forward_m_s": truth_forward, "truth_yaw_rate_rad_s": truth_yaw,
                "truth_height_m": float(np.mean(true_heights)),
                "truth_roll_rad": math.atan2(rb[2, 1], rb[2, 2]),
            }
            rows.append({**base, "stream": "diag", **{f"diag_{k}": f"{v:.6g}" for k, v in values.items()}})
        joint_row = {**base, "stream": "joint"}
        for prefix, values in (("pos_", pos), ("vel_", vel), ("eff_", eff)):
            joint_row.update({f"{prefix}{j}": f"{x:.6g}" for j, x in zip(JOINTS, values)})
        rows.append(joint_row)
        rows.append({**base, "stream": "imu", **{k: f"{x:.6g}" for k, x in zip(IMU_COLUMNS, imu)}})

    header = (["t_s", "stream", "phase", "cmd_forward", "cmd_yaw", "cmd_height_rate", "cmd_roll"]
              + [f"diag_{k}" for k in DIAG_KEYS]
              + [f"{p}{j}" for p in ("pos_", "vel_", "eff_") for j in JOINTS] + IMU_COLUMNS
              + [f"diag_{k}" for k in TRUTH_KEYS])
    output.parent.mkdir(parents=True, exist_ok=True)
    with open(output, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=header)
        writer.writeheader()
        writer.writerows(rows)
    if renderer is not None and frames_out:
        import imageio.v2 as imageio
        imageio.mimsave(video, frames_out, fps=video_fps, codec="libx264", quality=7)
    return (3 if fall_time >= 0 else 0), output


# ---------------------------------------------------------------- metrics

def truth_metrics(path: Path, phase: int) -> dict:
    """Ground-truth metrics both recordings carry (truth_pitch_rad, truth_speed_m_s)."""
    run_ = compare_runs.load(str(path))
    d = compare_runs.select(run_["diag"], phase)
    if not d or not d["t_s"].size:
        return {}
    pitch = d["diag_truth_pitch_rad"]
    speed = d["diag_truth_speed_m_s"]
    cmd = np.abs(d["cmd_forward"])
    return {"truth_pitch_rms": compare_runs.rms(pitch - np.mean(pitch)),
            "truth_pitch_p2p": float(np.max(pitch) - np.min(pitch)),
            "truth_speed_mean": float(np.mean(speed)),
            "truth_speed_err_rms": compare_runs.rms(speed - cmd)}


def all_metrics(path: Path, scenario: str) -> dict[int, dict]:
    """closed_loop.py's per-phase compare_runs metrics plus the truth metrics."""
    run_ = compare_runs.load(str(path))
    phases = sorted(int(p) for p in np.unique(run_["diag"]["phase"]) if 0 <= p < len(SCENARIOS[scenario]))
    out = {p: compare_runs.metrics(run_, p) for p in phases}
    for phase in out:
        out[phase].update(truth_metrics(path, phase))
    return out


def run_wbc(scenario: str, sim: dict[str, str], output: Path) -> int:
    eigen = Path(os.environ.get("EIGEN_DIR", Path(__file__).resolve().parents[1] / ".cache" / "eigen-3.4.0"))
    binary = closed_loop.build(closed_loop.find_mujoco(None), eigen)
    parameters = closed_loop.controller_parameters([str(closed_loop.ROBOT_OVERRIDES)], [])
    code, _ = closed_loop.run_once(binary, parameters, scenario, sim, output, quiet=True)
    return code


COMPARE_KEYS = ["truth_pitch_rms", "truth_speed_err_rms", "pitch_rms", "pitch_peak_hz", "vel_err_rms",
                "yaw_err_rms", "yaw_rise_s", "height_err", "wheel_tau_rms", "wheel_flip_frac", "leg_eff_rms"]


def compare(policy_dir: Path, scenarios: list[str], profiles: list[str], out_dir: Path) -> dict:
    """Policy and WBC on the same scenarios and latency profiles; returns the table."""
    out_dir.mkdir(parents=True, exist_ok=True)
    table = {}
    for scenario in scenarios:
        for profile in profiles:
            sim = PROFILES[profile]
            entry = {}
            for who in ("policy", "wbc"):
                path = out_dir / f"{who}_{scenario}_{profile}.csv"
                code = run(policy_dir, scenario, sim, path)[0] if who == "policy" else run_wbc(scenario, sim, path)
                entry[who] = {"fell": code == 3, "exit": code,
                              "phases": {str(k): v for k, v in all_metrics(path, scenario).items()} if code in (0, 3) else {}}
            table[f"{scenario}/{profile}"] = entry
    (out_dir / "comparison.json").write_text(json.dumps(table, indent=1, default=float))
    return table


def markdown(table: dict) -> str:
    lines = []
    for key, entry in table.items():
        scenario = key.split("/")[0]
        lines.append(f"\n**{key}** — policy {'FELL' if entry['policy']['fell'] else 'completed'}, "
                     f"WBC {'FELL' if entry['wbc']['fell'] else 'completed'}\n")
        lines.append("| phase | command | " + " | ".join(COMPARE_KEYS) + " |")
        lines.append("|" + "---|" * (len(COMPARE_KEYS) + 2))
        phases = sorted(set(entry["policy"]["phases"]) | set(entry["wbc"]["phases"]), key=int)
        for p in phases:
            cmd = SCENARIOS[scenario][int(p)] if int(p) < len(SCENARIOS[scenario]) else ""
            for who in ("policy", "wbc"):
                m = entry[who]["phases"].get(p, {})
                cells = [f"{m.get(k, math.nan):.3f}" for k in COMPARE_KEYS]
                lines.append(f"| {p} {who} | {cmd} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("policy_dir")
    parser.add_argument("--scenario", default="hardware", choices=sorted(SCENARIOS))
    parser.add_argument("--profile", default="nominal", choices=sorted(PROFILES))
    parser.add_argument("--sim", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--output")
    parser.add_argument("--video")
    parser.add_argument("--course", action="store_true",
                        help=f"planks {COURSE} and a hill {COURSE_HILL} along the start heading")
    parser.add_argument("--compare", action="store_true",
                        help="policy and WBC on hardware/default/push x nominal/uart_imu/worst")
    args = parser.parse_args()
    policy_dir = Path(args.policy_dir)
    if args.compare:
        table = compare(policy_dir, ["hardware", "default", "push"], list(PROFILES), policy_dir / "eval")
        text = markdown(table)
        (policy_dir / "eval" / "comparison.md").write_text(text + "\n")
        print(text)
        return 0
    sim = {**PROFILES[args.profile], **closed_loop.parse_sim(args.sim)}
    output = Path(args.output) if args.output else Path(tempfile.mkstemp(suffix=".csv")[1])
    code, _ = run(policy_dir, args.scenario, sim, output, Path(args.video) if args.video else None,
                  course=args.course)
    print(f"{'FELL' if code == 3 else 'completed all phases'}; recording: {output}")
    per_phase = all_metrics(output, args.scenario)
    keys = COMPARE_KEYS
    print(f"{'phase':>5s} " + " ".join(f"{k[:15]:>15s}" for k in keys))
    for phase, values in per_phase.items():
        print(f"{phase:5d} " + " ".join(f"{values.get(k, math.nan):15.4f}" for k in keys))
    return code


if __name__ == "__main__":
    raise SystemExit(main())
