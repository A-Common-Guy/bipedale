"""MuJoCo implementation of the same MIT command/state boundary as hardware."""
from __future__ import annotations

import io
import logging
import math
import threading
import time
from collections import deque
from typing import Dict

import numpy as np

from identified import load_identified
from models import ImuState, MitCommand, MotorState, RobotState

log = logging.getLogger("sandro.mujoco")


class MujocoBackend:
    name = "simulation"

    def __init__(self, cfg: dict):
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError("MuJoCo is required for simulation mode") from exc
        self.mj = mujoco
        mc = cfg["mujoco"]
        self.model = mujoco.MjModel.from_xml_path(str(mc["model"]))
        self.data = mujoco.MjData(self.model)
        motor_cfg = cfg["robot"]["motors"]
        self.motor_cfg = motor_cfg
        self.names = tuple(motor_cfg)
        self._joint = {
            name: mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                motor_cfg[name].get("sim_joint", name),
            )
            for name in self.names
        }
        missing = [name for name, jid in self._joint.items() if jid < 0]
        if missing:
            raise ValueError(f"MuJoCo model is missing joints: {missing}")
        self._sim_sign = {name: 1.0 for name in self.names}
        self._sim_zero = {name: 0.0 for name in self.names}
        self._sensor = {}
        for sensor_id in range(self.model.nsensor):
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_id
            )
            self._sensor[name] = (
                int(self.model.sensor_adr[sensor_id]),
                int(self.model.sensor_dim[sensor_id]),
            )

        # The IMU site sits flat on the roof (x forward, y left, z up), which
        # is deliberately not the torso body frame -- the Onshape export left
        # the torso axes pointing along the wheel axle. Mirroring a physical
        # orientation onto the free joint therefore has to undo that fixed
        # rotation, or the visual model shows the robot lying on its side.
        self._imu_to_torso = np.array([1.0, 0.0, 0.0, 0.0])
        imu_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu")
        if imu_site >= 0:
            mujoco.mju_negQuat(self._imu_to_torso, self.model.site_quat[imu_site])

        self.decimation = max(
            1,
            round(
                1.0
                / (
                    float(cfg["runtime"]["control_hz"])
                    * self.model.opt.timestep
                )
            ),
        )
        self.enabled = False
        self._commands = {name: MitCommand() for name in self.names}
        self._torque_scale = {name: 1.0 for name in self.names}
        self._torque_limit = {
            name: float(
                cfg["control"][
                    "leg_torque_limit"
                    if motor_cfg[name]["kind"] == "leg"
                    else "wheel_torque_limit"
                ]
            )
            for name in self.names
        }
        identified = load_identified(mc["identified_params"], mc["model"])
        delay_s = 0.0
        if identified:
            self._apply_identified(identified)
            delay_s = float(identified.get("command_delay_s", 0.0))
        self._delay_steps = round(delay_s * float(cfg["runtime"]["control_hz"]))
        self._command_history = deque(maxlen=max(1, self._delay_steps + 1))
        self._lock = threading.RLock()
        self._renderer = None
        self._renderer_owner = None
        self._renderer_closed = False
        self._render_data = mujoco.MjData(self.model)
        self._camera = mujoco.MjvCamera()
        self._camera_lock = threading.Lock()
        mujoco.mjv_defaultFreeCamera(self.model, self._camera)
        self._render_size = (
            int(mc["render_width"]),
            int(mc["render_height"]),
        )
        self.model.vis.global_.offwidth = max(
            self.model.vis.global_.offwidth, self._render_size[0]
        )
        self.model.vis.global_.offheight = max(
            self.model.vis.global_.offheight, self._render_size[1]
        )
        self._keyframe = str(mc["keyframe"])
        self.hanged = bool(mc.get("hanged", False))
        self._hang_height = float(mc.get("hang_height_m", 0.25))
        self._base_reference = None
        self._has_free_base = (
            self.model.njnt > 0
            and self.model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
        )

        # The exported model contains convenience position/velocity actuators.
        # Disable them: MIT torque is applied explicitly at each joint DOF.
        self.model.actuator_gainprm[:, :] = 0.0
        self.model.actuator_biasprm[:, :] = 0.0

    def _apply_identified(self, values: dict) -> None:
        groups = values["groups"]
        for name in self.names:
            joint = self._joint[name]
            dof = int(self.model.jnt_dofadr[joint])
            group = groups[self.motor_cfg[name]["kind"]]
            self.model.dof_damping[dof] = float(group["damping"])
            self.model.dof_frictionloss[dof] = float(group["frictionloss"])
            self.model.dof_armature[dof] = float(group["armature"])
            self._torque_scale[name] = float(group["torque_scale"])

    def start(self) -> None:
        with self._lock:
            key = self.mj.mj_name2id(
                self.model, self.mj.mjtObj.mjOBJ_KEY, self._keyframe
            )
            if key >= 0:
                self.mj.mj_resetDataKeyframe(self.model, self.data, key)
            else:
                self.mj.mj_resetData(self.model, self.data)
            self.mj.mj_forward(self.model, self.data)
            self._derive_leg_frames()
            if self._has_free_base:
                self._base_reference = self.data.qpos[:7].copy()
                self._base_reference[2] += self._hang_height
                if self.hanged:
                    self._pin_base()
            self.mj.mj_forward(self.model, self.data)

    def _derive_leg_frames(self) -> None:
        """Map raw MJCF hinges to zero-down five-bar joint coordinates."""
        for side in ("right", "left"):
            names = (f"{side}_inner", f"{side}_outer")
            joints = tuple(self._joint[name] for name in names)
            anchors = tuple(self.data.xanchor[joint].copy() for joint in joints)
            base_direction = anchors[1] - anchors[0]
            base_direction /= np.linalg.norm(base_direction)
            axis0 = self.data.xaxis[joints[0]].copy()
            axis0 /= np.linalg.norm(axis0)
            down = np.cross(axis0, base_direction)
            down /= np.linalg.norm(down)
            wheel = self.data.xanchor[self._joint[f"{side}_wheel"]]
            if float(down @ (wheel - 0.5 * (anchors[0] + anchors[1]))) < 0:
                down = -down
            for index, (name, joint) in enumerate(zip(names, joints)):
                passive = self._descendant_passive_joint(joint)
                axis = self.data.xaxis[joint].copy()
                axis /= np.linalg.norm(axis)
                vertical = down - axis * float(down @ axis)
                vertical /= np.linalg.norm(vertical)
                link = self.data.xanchor[passive] - self.data.xanchor[joint]
                link -= axis * float(link @ axis)
                link /= np.linalg.norm(link)
                angle = math.atan2(
                    float(axis @ np.cross(vertical, link)),
                    float(vertical @ link),
                )
                qadr = int(self.model.jnt_qposadr[joint])
                self._sim_zero[name] = float(self.data.qpos[qadr]) - angle

                # Match kinematics.py: positive angles open each upper link
                # away from the center (inner toward -x, outer toward +x).
                derivative = np.cross(axis, vertical)
                desired = -base_direction if index == 0 else base_direction
                self._sim_sign[name] = (
                    1.0 if float(derivative @ desired) >= 0.0 else -1.0
                )

    def _descendant_passive_joint(self, motor_joint: int) -> int:
        motor_body = int(self.model.jnt_bodyid[motor_joint])
        for joint in range(self.model.njnt):
            name = self.mj.mj_id2name(
                self.model, self.mj.mjtObj.mjOBJ_JOINT, joint
            )
            if not name or "passive" not in name:
                continue
            body = int(self.model.jnt_bodyid[joint])
            while body and body != motor_body:
                body = int(self.model.body_parentid[body])
            if body == motor_body:
                return joint
        raise RuntimeError(
            f"no passive link below motor joint {motor_joint}"
        )

    def _logical_position(self, name: str) -> float:
        joint = self._joint[name]
        q = float(self.data.qpos[int(self.model.jnt_qposadr[joint])])
        return self._sim_sign[name] * (q - self._sim_zero[name])

    def _logical_velocity(self, name: str) -> float:
        joint = self._joint[name]
        qd = float(self.data.qvel[int(self.model.jnt_dofadr[joint])])
        return self._sim_sign[name] * qd

    def _model_position(self, name: str, logical: float) -> float:
        return self._sim_zero[name] + self._sim_sign[name] * logical

    def set_hanged(self, enabled: bool) -> None:
        """Suspend the floating torso while leaving every leg joint dynamic."""
        with self._lock:
            if enabled and not self._has_free_base:
                raise RuntimeError("hanged mode requires a floating-base model")
            self.hanged = bool(enabled)
            if self.hanged:
                self._pin_base()
                self.mj.mj_forward(self.model, self.data)

    def move_camera(self, action: str, dx: float, dy: float) -> None:
        """Apply normalized browser mouse motion to the offscreen camera."""
        dx = min(1.0, max(-1.0, float(dx)))
        dy = min(1.0, max(-1.0, float(dy)))
        with self._camera_lock:
            if action == "orbit":
                self._camera.azimuth -= 180.0 * dx
                self._camera.elevation = min(
                    89.0, max(-89.0, self._camera.elevation - 180.0 * dy)
                )
            elif action == "pan":
                azimuth = math.radians(self._camera.azimuth)
                scale = max(self._camera.distance, 0.05)
                self._camera.lookat[0] += scale * (
                    dx * math.cos(azimuth) + dy * math.sin(azimuth)
                )
                self._camera.lookat[1] += scale * (
                    dx * math.sin(azimuth) - dy * math.cos(azimuth)
                )
                self._camera.lookat[2] += scale * dy
            elif action == "zoom":
                self._camera.distance = min(
                    10.0,
                    max(0.05, self._camera.distance * math.exp(2.5 * dy)),
                )
            else:
                raise ValueError(f"unknown camera action {action!r}")

    def reset_camera(self) -> None:
        with self._camera_lock:
            self.mj.mjv_defaultFreeCamera(self.model, self._camera)

    def _pin_base(self) -> None:
        if self._base_reference is not None:
            self.data.qpos[:7] = self._base_reference
            self.data.qvel[:6] = 0.0

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False
        self._commands = {name: MitCommand() for name in self.names}
        self._command_history.clear()

    def estop(self) -> None:
        self.disable()

    def write(self, commands: Dict[str, MitCommand]) -> None:
        with self._lock:
            incoming = dict(commands) if self.enabled else {
                name: MitCommand() for name in self.names
            }
            self._command_history.append(incoming)
            if len(self._command_history) <= self._delay_steps:
                self._commands = {name: MitCommand() for name in self.names}
            else:
                self._commands = self._command_history[0]
            for _ in range(self.decimation):
                if self.hanged:
                    self._pin_base()
                self.data.qfrc_applied[:] = 0.0
                for name, command in self._commands.items():
                    joint = self._joint[name]
                    dadr = int(self.model.jnt_dofadr[joint])
                    q = self._logical_position(name)
                    qd = self._logical_velocity(name)
                    torque = self._mit_torque(name, command, q, qd)
                    self.data.qfrc_applied[dadr] += (
                        self._sim_sign[name] * torque
                    )
                self.mj.mj_step(self.model, self.data)
                if self.hanged:
                    self._pin_base()
                    self.mj.mj_forward(self.model, self.data)

    def read(self) -> RobotState:
        with self._lock:
            motors = {}
            for name in self.names:
                joint = self._joint[name]
                cmd = self._commands[name]
                q = self._logical_position(name)
                qd = self._logical_velocity(name)
                motors[name] = MotorState(
                    position=q,
                    velocity=qd,
                    torque=self._mit_torque(name, cmd, q, qd),
                    mit_position=q,
                    mit_velocity=qd,
                    mit_torque=self._mit_torque(name, cmd, q, qd),
                    online=True,
                    age_s=0.0,
                )
            return RobotState(
                motors=motors,
                imu=ImuState(
                    quaternion=tuple(self._read_sensor("imu_quat", 4, (1, 0, 0, 0))),
                    gyro=tuple(self._read_sensor("imu_gyro", 3, (0, 0, 0))),
                    accel=tuple(self._read_sensor("imu_accel", 3, (0, 0, 0))),
                    online=True,
                    age_s=0.0,
                ),
                timestamp=time.monotonic(),
                source=self.name,
            )

    def _mit_torque(
        self, name: str, command: MitCommand, position: float, velocity: float
    ) -> float:
        torque = self._torque_scale[name] * (
            command.kp * (command.position - position)
            + command.kd * (command.velocity - velocity)
            + command.torque
        )
        limit = self._torque_limit[name]
        return min(max(torque, -limit), limit)

    def _read_sensor(self, name: str, size: int, default: tuple) -> np.ndarray:
        if name not in self._sensor:
            return np.asarray(default, dtype=float)
        address, dimension = self._sensor[name]
        if dimension != size:
            return np.asarray(default, dtype=float)
        return self.data.sensordata[address : address + dimension].copy()

    def render_jpeg(self) -> bytes:
        from PIL import Image

        # Copy only the dynamic state while briefly holding the control lock.
        # OpenGL and JPEG encoding then use independent MjData, so rendering
        # can never stall the 200 Hz simulation/control loop.
        with self._lock:
            self._render_data.qpos[:] = self.data.qpos
            self._render_data.qvel[:] = self.data.qvel
            self._render_data.ctrl[:] = self.data.ctrl
            self._render_data.time = self.data.time
        self.mj.mj_forward(self.model, self._render_data)
        width, height = self._render_size
        if self._renderer is None:
            if self._renderer_closed:
                raise RuntimeError("renderer is closed")
            self._renderer = self.mj.Renderer(
                self.model, height=height, width=width
            )
            self._renderer_owner = threading.get_ident()
        with self._camera_lock:
            self._renderer.update_scene(self._render_data, camera=self._camera)
        pixels = self._renderer.render()
        stream = io.BytesIO()
        Image.fromarray(pixels).save(stream, format="JPEG", quality=78)
        return stream.getvalue()

    def mirror_state(self, state: RobotState) -> None:
        """Pose the visual model from physical encoder/IMU state, without stepping."""
        with self._lock:
            for name, motor in state.motors.items():
                if name in self._joint:
                    joint = self._joint[name]
                    self.data.qpos[int(self.model.jnt_qposadr[joint])] = (
                        self._model_position(name, motor.position)
                    )
            if self.model.nq >= 7 and state.imu.online:
                measured = np.asarray(state.imu.quaternion, dtype=float)
                norm = np.linalg.norm(measured)
                if norm > 1e-9:
                    torso = np.zeros(4)
                    self.mj.mju_mulQuat(
                        torso, measured / norm, self._imu_to_torso
                    )
                    self.data.qpos[3:7] = torso
            self.mj.mj_forward(self.model, self.data)

    def close_renderer(self) -> None:
        """Close OpenGL on the same dedicated thread that rendered frames.

        The flag also stops render_jpeg() from rebuilding a context out of a
        request that was still queued behind this call.
        """
        self._renderer_closed = True
        if self._renderer is None:
            return
        if self._renderer_owner not in (None, threading.get_ident()):
            # Freeing a GL context from a foreign thread aborts the process.
            # Leaking it is survivable; shutdown is already under way.
            log.warning("skipping GL teardown requested off the render thread")
            return
        renderer, self._renderer = self._renderer, None
        self._renderer_owner = None
        renderer.close()

    def close(self) -> None:
        self.close_renderer()
