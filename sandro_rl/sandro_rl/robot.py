"""What the environment needs from the MuJoCo model, resolved by name and checked.

`RobotSpec.load()` reads the generated ROS model and the ROS parameters
(sources.py), resolves every index the environment uses and fails loudly if
the model no longer has the structure the environment was written for: the
six actuated joints with one direct motor each, the stand keyframe, the IMU
sensors on the `imu` site, the four loop-closing connect constraints, two
tyre cylinders with a contact dimension the environment handles, and a
physics step that divides the 300 Hz control period.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from . import sources
from .sources import IMU_SENSORS, JOINTS, LEG_JOINTS, TYRE_SUFFIX, WHEEL_JOINTS

# Contact dimensions the environment's randomisation and consistency check
# are written for: 3 (sliding only) and 4 (plus torsional scrub). 6 adds
# rolling friction, which the friction randomisation would leave unscaled.
HANDLED_TYRE_CONDIM = (3, 4)
EXPECTED_CONNECT_CONSTRAINTS = 4

# Left-right mirror of the joint order. The motor names are mirrored between
# the sides (right inner is the rear motor, left inner the front one), so the
# rear motor of one side maps to the rear motor of the other; logical angles
# open away from the leg centre on both sides, so no sign changes.
MIRROR_JOINT = {"right_motor_inner": "left_motor_outer", "right_motor_outer": "left_motor_inner",
                "right_wheel": "left_wheel", "left_motor_inner": "right_motor_outer",
                "left_motor_outer": "right_motor_inner", "left_wheel": "right_wheel"}


class ModelContractError(RuntimeError):
    """The model changed in a way the environment does not handle."""


@dataclass
class RobotSpec:
    model: mujoco.MjModel
    scene: Path
    limits: sources.RobotLimits
    latency: sources.Latency
    identified: dict
    physics_dt: float
    steps_per_control: int
    joint_qpos: np.ndarray        # [6] qpos address per JOINTS entry
    joint_dof: np.ndarray         # [6]
    joint_actuator: np.ndarray    # [6] actuator id per JOINTS entry
    actuator_gear: np.ndarray     # [6]
    sensor_adr: dict              # name -> (adr, dim)
    sensor_noise: dict            # name -> sigma from the MJCF (sim_identified imu_noise)
    imu_site: int
    torso_body: int
    floor_geom: int
    tyre_geoms: np.ndarray        # [2] right, left
    body_collision_geoms: np.ndarray
    wheel_joint_ids: np.ndarray   # [2] right, left (for xanchor)
    stand_qpos: np.ndarray
    stand_q: np.ndarray           # [6] joint values of the keyframe in JOINTS order
    imu_roll_bias_rad: float
    leg_joint_range: tuple[float, float]
    mirror_perm: np.ndarray = field(default_factory=lambda: np.zeros(6, int))

    @property
    def leg_index(self) -> np.ndarray:
        return np.array([JOINTS.index(j) for j in LEG_JOINTS])

    @property
    def wheel_index(self) -> np.ndarray:
        return np.array([JOINTS.index(j) for j in WHEEL_JOINTS])

    @classmethod
    def load(cls, scene: Path | str = sources.SCENE, overrides: bool = True,
             model: mujoco.MjModel | None = None) -> "RobotSpec":
        scene = Path(scene)
        try:
            m = model if model is not None else mujoco.MjModel.from_xml_path(str(scene))
        except ValueError as error:
            raise ModelContractError(f"cannot load {scene}: {error}") from error
        limits = sources.RobotLimits.from_ros(overrides)
        latency = sources.Latency.from_launch()
        identified = sources.identified()
        errors: list[str] = []

        def need(condition: bool, message: str) -> None:
            if not condition:
                errors.append(message)

        control_dt = 1.0 / limits.control_rate_hz
        steps = control_dt / m.opt.timestep
        need(abs(steps - round(steps)) < 1e-6,
             f"physics step {m.opt.timestep} does not divide the {limits.control_rate_hz:g} Hz control period")
        need(m.nkey > 0 and mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand") >= 0,
             "keyframe 'stand' missing")
        torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
        need(torso >= 0 and m.body_jntnum[torso] == 1
             and m.jnt_type[m.body_jntadr[torso]] == mujoco.mjtJoint.mjJNT_FREE,
             "body 'torso' with a free joint missing")
        imu = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "imu")
        need(imu >= 0, "site 'imu' missing")

        qpos, dof, act, gear, jids = [], [], [], [], {}
        for name in JOINTS:
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if j < 0:
                errors.append(f"joint '{name}' missing")
                continue
            need(m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE, f"joint '{name}' is not a hinge")
            jids[name] = j
            acts = [a for a in range(m.nu)
                    if m.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT and m.actuator_trnid[a, 0] == j]
            if len(acts) != 1:
                errors.append(f"joint '{name}' has {len(acts)} actuators, expected one")
                continue
            a = acts[0]
            # A direct torque motor: gain on ctrl only, no bias. Anything else
            # (position servo, affine bias) would double the drive law.
            need(m.actuator_gaintype[a] == mujoco.mjtGain.mjGAIN_FIXED
                 and m.actuator_biastype[a] == mujoco.mjtBias.mjBIAS_NONE
                 and abs(m.actuator_gainprm[a, 0] - 1.0) < 1e-12,
                 f"actuator of '{name}' is not a unit-gain motor")
            # The MIT torque is joint torque; a gear would scale it silently.
            need(abs(m.actuator_gear[a, 0] - 1.0) < 1e-12,
                 f"actuator of '{name}' has gear {m.actuator_gear[a, 0]:g}, expected 1")
            qpos.append(m.jnt_qposadr[j])
            dof.append(m.jnt_dofadr[j])
            act.append(a)
            gear.append(m.actuator_gear[a, 0])
        need(m.nu == len(JOINTS), f"model has {m.nu} actuators, the environment drives {len(JOINTS)}")

        sensor_adr, sensor_noise = {}, {}
        for name in IMU_SENSORS:
            s = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if s < 0:
                errors.append(f"sensor '{name}' missing")
                continue
            need(m.sensor_objtype[s] == mujoco.mjtObj.mjOBJ_SITE and m.sensor_objid[s] == imu,
                 f"sensor '{name}' is not on the imu site")
            sensor_adr[name] = (int(m.sensor_adr[s]), int(m.sensor_dim[s]))
            sensor_noise[name] = float(m.sensor_noise[s])

        connects = int(np.sum(m.eq_type == mujoco.mjtEq.mjEQ_CONNECT))
        need(connects == EXPECTED_CONNECT_CONSTRAINTS and m.neq == connects,
             f"expected {EXPECTED_CONNECT_CONSTRAINTS} connect constraints closing the legs, "
             f"found {connects} of {m.neq} equality constraints")

        tyres = []
        for side in ("right", "left"):
            names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "" for g in range(m.ngeom)]
            found = [g for g, n in enumerate(names) if n.startswith(side) and n.endswith(TYRE_SUFFIX)]
            if len(found) != 1:
                errors.append(f"expected one {side} tyre geom '*{TYRE_SUFFIX}', found {len(found)}")
                continue
            g = found[0]
            need(m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER, f"{side} tyre is not a cylinder")
            need(int(m.geom_condim[g]) in HANDLED_TYRE_CONDIM,
                 f"{side} tyre condim {m.geom_condim[g]} not handled (handled: {HANDLED_TYRE_CONDIM})")
            tyre = identified.get("tyre") or {}
            if "condim" in tyre:
                need(int(m.geom_condim[g]) == int(tyre["condim"]),
                     f"{side} tyre condim {m.geom_condim[g]} differs from sim_identified.yaml {tyre['condim']}")
            if "friction" in tyre:
                need(np.allclose(m.geom_friction[g], tyre["friction"], rtol=1e-6),
                     f"{side} tyre friction {m.geom_friction[g]} differs from sim_identified.yaml")
            tyres.append(g)

        floors = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE]
        need(len(floors) == 1, f"expected one floor plane, found {len(floors)}")
        body_geoms = [g for g in range(m.ngeom)
                      if m.geom_bodyid[g] == torso and (m.geom_contype[g] or m.geom_conaffinity[g])]
        if errors:
            raise ModelContractError("model contract violated:\n  " + "\n  ".join(errors))

        key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand")
        stand_qpos = m.key_qpos[key].copy()
        mirror = np.array([JOINTS.index(MIRROR_JOINT[j]) for j in JOINTS])
        bal = sources.controller_parameters(True, "driving_balancer")["legs"]
        return cls(
            model=m, scene=scene, limits=limits, latency=latency, identified=identified,
            physics_dt=float(m.opt.timestep), steps_per_control=int(round(steps)),
            joint_qpos=np.array(qpos), joint_dof=np.array(dof), joint_actuator=np.array(act),
            actuator_gear=np.array(gear), sensor_adr=sensor_adr, sensor_noise=sensor_noise,
            imu_site=imu, torso_body=torso, floor_geom=floors[0], tyre_geoms=np.array(tyres),
            body_collision_geoms=np.array(body_geoms),
            wheel_joint_ids=np.array([jids["right_wheel"], jids["left_wheel"]]),
            stand_qpos=stand_qpos, stand_q=stand_qpos[np.array(qpos)],
            imu_roll_bias_rad=float((identified.get("imu_mount") or {}).get("roll_offset_rad", 0.0)),
            leg_joint_range=(float(bal["joint_min_rad"]), float(bal["joint_max_rad"])),
            mirror_perm=mirror,
        )

    def contract(self) -> dict:
        """What the environment relies on, for the consistency report and checkpoints."""
        m = self.model
        return {
            "joints": list(JOINTS),
            "actuator_gear": [float(g) for g in self.actuator_gear],
            "tyre_condim": [int(m.geom_condim[g]) for g in self.tyre_geoms],
            "tyre_friction": [[float(v) for v in m.geom_friction[g]] for g in self.tyre_geoms],
            "physics_dt": self.physics_dt,
            "steps_per_control": self.steps_per_control,
            "total_mass_kg": float(m.body_subtreemass[self.torso_body]),
            "stand_q": [float(v) for v in self.stand_q],
            "imu_roll_bias_rad": self.imu_roll_bias_rad,
            "control_rate_hz": self.limits.control_rate_hz,
        }


def rotation_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
