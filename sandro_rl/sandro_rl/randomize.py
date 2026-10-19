"""Per-environment randomisation of the model around the identified values.

Each environment of the batch gets its own copy of the randomised fields
for the whole training run (the MuJoCo Playground pattern); latency,
sensor and drive parameters and the terrain layout (mocap poses, terrain.py)
are resampled every episode in env.py instead.
Ranges come from configs/*.yaml; docs/DESIGN.md gives the source of each.
"""

from __future__ import annotations

import jax
import jax.numpy as jp
import mujoco
import numpy as np
from mujoco import mjx

from .sources import LEG_JOINTS, WHEEL_JOINTS

MODEL_PARAMS = 8
FIELDS = ("body_mass", "body_inertia", "body_pos", "geom_friction", "dof_frictionloss",
          "dof_damping", "dof_armature")


def _uniform(key, lo_hi, shape=()):
    return jax.random.uniform(key, shape, minval=lo_hi[0], maxval=lo_hi[1])


def _log_uniform(key, lo_hi, shape=()):
    return jp.exp(jax.random.uniform(key, shape, minval=np.log(lo_hi[0]), maxval=np.log(lo_hi[1])))


def randomize(env, mjx_model: mjx.Model, rng: jax.Array, num_envs: int, enabled: bool = True):
    """Returns (batched model, in_axes, model_params [num_envs, MODEL_PARAMS])."""
    m = env.mj_model
    spec = env.spec
    cfg = env.cfg["randomization"]["model"]
    payload = env.payload_body
    leg_dofs = jp.array([m.jnt_dofadr[m.joint(j).id] for j in LEG_JOINTS])
    wheel_dofs = jp.array([m.jnt_dofadr[m.joint(j).id] for j in WHEEL_JOINTS])
    robot_dofs = jp.concatenate([leg_dofs, wheel_dofs])
    tyres = jp.asarray(spec.tyre_geoms)
    # Payload offsets are given in the IMU frame, the frame of sim_identified.yaml.
    imu_in_torso = np.zeros(9)
    mujoco.mju_quat2Mat(imu_in_torso, m.site_quat[spec.imu_site])
    imu_in_torso = jp.asarray(imu_in_torso.reshape(3, 3))
    robot_bodies = jp.arange(1, m.nbody)

    def one(key):
        k = jax.random.split(key, 12)
        mass_scale = _uniform(k[0], cfg["body_mass_scale"])
        body_mass = mjx_model.body_mass.at[robot_bodies].multiply(mass_scale)
        body_inertia = mjx_model.body_inertia.at[robot_bodies].multiply(mass_scale)
        body_pos = mjx_model.body_pos
        add = jp.float32(0.0)
        offset = jp.zeros(3)
        if payload >= 0:
            add = _uniform(k[1], cfg["payload_mass_add_kg"])
            base = mjx_model.body_mass[payload] * mass_scale
            new = jp.maximum(base + add, 0.05)
            body_mass = body_mass.at[payload].set(new)
            body_inertia = body_inertia.at[payload].multiply(new / jp.maximum(base, 1e-6))
            offset = jax.random.uniform(k[2], (3,), minval=-1, maxval=1) * jp.asarray(cfg["payload_offset_m"])
            body_pos = body_pos.at[payload].add(imu_in_torso @ offset)
        sliding = _uniform(k[3], cfg["tyre_sliding_friction"])
        torsional = _log_uniform(k[4], cfg["tyre_torsional_scale"])
        friction = mjx_model.geom_friction.at[tyres, 0].set(sliding)
        friction = friction.at[tyres, 1].multiply(torsional)
        leg_fl = _log_uniform(k[5], cfg["leg_frictionloss_scale"])
        wheel_fl = _log_uniform(k[6], cfg["wheel_frictionloss_scale"])
        frictionloss = mjx_model.dof_frictionloss.at[leg_dofs].multiply(leg_fl)
        frictionloss = frictionloss.at[wheel_dofs].multiply(wheel_fl)
        damping = mjx_model.dof_damping.at[robot_dofs].multiply(_log_uniform(k[7], cfg["damping_scale"]))
        armature_scale = _log_uniform(k[8], cfg["armature_scale"])
        armature = mjx_model.dof_armature.at[robot_dofs].multiply(armature_scale)
        fields = {"body_mass": body_mass, "body_inertia": body_inertia, "body_pos": body_pos,
                  "geom_friction": friction, "dof_frictionloss": frictionloss,
                  "dof_damping": damping, "dof_armature": armature}
        params = jp.array([(mass_scale - 1.0) * 10.0, add / 0.4, offset[0] / 0.04, offset[2] / 0.03,
                           (sliding - 0.85) / 0.35, jp.log(torsional), jp.log(wheel_fl), jp.log(armature_scale) / 2])
        return fields, params

    keys = jax.random.split(rng, num_envs)
    if enabled and env.cfg["randomization"].get("enabled", True):
        fields, params = jax.vmap(one)(keys)
    else:
        fields = {name: jp.broadcast_to(getattr(mjx_model, name), (num_envs,) + getattr(mjx_model, name).shape)
                  for name in FIELDS}
        params = jp.zeros((num_envs, MODEL_PARAMS))
    in_axes = jax.tree_util.tree_map(lambda _: None, mjx_model)
    in_axes = in_axes.tree_replace({name: 0 for name in fields})
    model = mjx_model.tree_replace(fields)
    return model, in_axes, params
