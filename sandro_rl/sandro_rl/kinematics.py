"""Planar five-bar forward kinematics, the closed form of sandro_controllers/kinematics.cpp.

Works on numpy and jax.numpy arrays alike (pass the module as `xp`). Leg
coordinates: +x from the inner toward the outer pivot, +y down toward the
wheel; the inner pivot is the origin.
"""

from __future__ import annotations

import numpy as np


def forward(theta_inner, theta_outer, base, upper, lower, xp=np):
    """Axle position (x, y) of one leg; y is the leg height below the pivots."""
    p = upper * (xp.cos(theta_outer) - xp.cos(theta_inner))
    q = base + upper * (xp.sin(theta_inner) + xp.sin(theta_outer))
    radius = xp.sqrt(p * p + q * q)
    direction = xp.arctan2(p, q)
    branch = xp.arccos(xp.clip(radius / (2.0 * lower), -1.0, 1.0))
    b1 = branch + direction
    x = -upper * xp.sin(theta_inner) + lower * xp.cos(b1)
    y = upper * xp.cos(theta_inner) + lower * xp.sin(b1)
    # Reach from the farther pivot over upper + lower: 1 is the stretched
    # singularity the inverse kinematics rejects.
    reach = xp.maximum(xp.sqrt(x * x + y * y), xp.sqrt((x - base) ** 2 + y * y)) / (upper + lower)
    return x, y, reach


def leg_state(q_legs, geometry, neutral_m, xp=np):
    """Leg heights and body-forward axle offsets from the four leg joint angles.

    q_legs: [..., 4] in LEG_JOINTS order (right inner, right outer, left
    inner, left outer). Returns (height[..., 2], offset[..., 2], stretch[..., 2])
    per side (right, left); offset is positive when the axle is ahead of the
    pivot centre, the whole-body controller's leg offset convention;
    stretch is the reach from the farther pivot over its maximum (1 = singular).
    """
    heights, offsets, stretch = [], [], []
    for side, (i, o) in (("right", (0, 1)), ("left", (2, 3))):
        g = geometry[side]
        x, y, s = forward(q_legs[..., i], q_legs[..., o], g["base_m"], g["upper_m"], g["lower_m"], xp)
        heights.append(y)
        offsets.append(g["forward_sign"] * (x - neutral_m))
        stretch.append(s)
    return xp.stack(heights, -1), xp.stack(offsets, -1), xp.stack(stretch, -1)
