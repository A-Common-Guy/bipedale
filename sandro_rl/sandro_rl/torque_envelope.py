"""The drives' thermal torque envelope, as sandro_hardware/torque_envelope.hpp
applies it on the robot: the peak torque in bursts, folding back to the
rated torque as the heat (torque squared low-passed over the thermal time
constant) approaches rated^2. Works on numpy and jax arrays alike. The
temperature derating of the hardware is not modelled: the simulation has no
temperature.
"""

from __future__ import annotations

import math

from .sources import DriveEnvelope


def budget(heat, env: DriveEnvelope, xp):
    """1 while the heat is below knee * rated^2, 0 at rated^2."""
    rated2 = env.rated_nm ** 2
    return xp.clip((rated2 - heat) / ((1.0 - env.knee) * rated2), 0.0, 1.0)


def limit(heat, env: DriveEnvelope, xp):
    return env.rated_nm + (env.peak_nm - env.rated_nm) * budget(heat, env, xp)


def account(heat, torque_squared, period_s: float, env: DriveEnvelope):
    """Heat after period_s of torque_squared (mean over the period)."""
    alpha = 1.0 - math.exp(-period_s / env.time_constant_s)
    return heat + alpha * (torque_squared - heat)
