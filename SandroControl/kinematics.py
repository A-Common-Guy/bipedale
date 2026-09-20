"""Planar five-bar kinematics used by the manual Cartesian interface."""
from __future__ import annotations

import math
from typing import Mapping, Optional

from models import LegState, MotorState


def inverse(
    x: float, y: float, base: float, upper: float, lower: float
) -> Optional[tuple[float, float]]:
    """Return the two motor angles (rad), or None outside the workspace.

    Coordinates match ``leg_control/leg_server.py``: +x across the two
    pivots, +y down, zero angles point down, with both positive directions
    opening the upper links away from the leg centre.
    """
    r1 = math.hypot(x, y)
    r2 = math.hypot(x - base, y)
    lo, hi = abs(upper - lower), upper + lower
    if not (max(lo, 1e-9) <= r1 <= hi and max(lo, 1e-9) <= r2 <= hi):
        return None
    clamp = lambda value: max(-1.0, min(1.0, value))
    a1 = math.acos(clamp((upper**2 + r1**2 - lower**2) / (2 * upper * r1)))
    a2 = math.acos(clamp((upper**2 + r2**2 - lower**2) / (2 * upper * r2)))
    d1 = math.atan2(y, x) + a1
    d2 = math.atan2(y, x - base) - a2
    wrap = lambda value: math.atan2(math.sin(value), math.cos(value))
    return wrap(d1 - math.pi / 2), wrap(math.pi / 2 - d2)


def forward(
    th1: float, th2: float, base: float, upper: float, lower: float
) -> Optional[tuple[float, float]]:
    p = upper * (math.cos(th2) - math.cos(th1))
    q = base + upper * (math.sin(th1) + math.sin(th2))
    radius = math.hypot(p, q)
    if radius > 2 * lower:
        return None
    direction = math.atan2(p, q)
    branch = math.acos(min(1.0, radius / (2 * lower)))
    b1 = branch + direction
    return (
        -upper * math.sin(th1) + lower * math.cos(b1),
        upper * math.cos(th1) + lower * math.sin(b1),
    )


def jacobian(
    th1: float,
    th2: float,
    base: float,
    upper: float,
    lower: float,
    epsilon: float = 1e-6,
) -> Optional[tuple[tuple[float, float], tuple[float, float]]]:
    """Numerical endpoint Jacobian d(x,y)/d(th1,th2)."""
    columns = []
    for d1, d2 in ((epsilon, 0.0), (0.0, epsilon)):
        plus = forward(th1 + d1, th2 + d2, base, upper, lower)
        minus = forward(th1 - d1, th2 - d2, base, upper, lower)
        if plus is None or minus is None:
            return None
        columns.append(
            (
                (plus[0] - minus[0]) / (2.0 * epsilon),
                (plus[1] - minus[1]) / (2.0 * epsilon),
            )
        )
    return (
        (columns[0][0], columns[1][0]),
        (columns[0][1], columns[1][1]),
    )


def map_legs(
    motors: Mapping[str, MotorState], legs_cfg: Mapping[str, Mapping[str, float]]
) -> dict[str, LegState]:
    """Derive Cartesian endpoint position and velocity from logical joints."""
    result = {}
    for side, geometry in legs_cfg.items():
        inner = motors.get(f"{side}_inner")
        outer = motors.get(f"{side}_outer")
        if inner is None or outer is None:
            result[side] = LegState()
            continue
        values = (
            float(geometry["base"]),
            float(geometry["upper"]),
            float(geometry["lower"]),
        )
        position = forward(inner.position, outer.position, *values)
        matrix = jacobian(inner.position, outer.position, *values)
        if position is None or matrix is None:
            result[side] = LegState()
            continue
        vx = matrix[0][0] * inner.velocity + matrix[0][1] * outer.velocity
        vy = matrix[1][0] * inner.velocity + matrix[1][1] * outer.velocity
        result[side] = LegState(
            x=position[0],
            y=position[1],
            vx=vx,
            vy=vy,
            reachable=True,
        )
    return result
