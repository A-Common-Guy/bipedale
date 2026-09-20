"""Small, backend-independent command and state types."""
from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from typing import Dict, Optional, Protocol


@dataclass
class MitCommand:
    """One GIM MIT frame, in output-shaft SI units."""

    position: float = 0.0
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque: float = 0.0


@dataclass
class MotorState:
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    iq: Optional[float] = None
    mit_position: Optional[float] = None
    mit_velocity: Optional[float] = None
    mit_torque: Optional[float] = None
    vbus: Optional[float] = None
    temperature: Optional[float] = None
    online: bool = False
    error: str = ""
    age_s: float = float("inf")


@dataclass
class LegState:
    """Measured five-bar endpoint in the logical leg plane."""

    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    reachable: bool = False


@dataclass
class LegCommand:
    """Cartesian endpoint target; +x crosses the pivots and +y points down."""

    x: float = 0.0
    y: float = 0.0
    kp: Optional[float] = None
    kd: Optional[float] = None


@dataclass
class WheelCommand:
    """Logical wheel target in MIT torque/velocity form."""

    torque: float = 0.0
    velocity: float = 0.0
    kd: Optional[float] = None


@dataclass
class RobotCommand:
    """High-level command mapped to six MIT frames by the runtime."""

    legs: Dict[str, LegCommand] = field(default_factory=dict)
    wheels: Dict[str, WheelCommand] = field(default_factory=dict)


@dataclass
class ImuState:
    quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    online: bool = False
    age_s: float = float("inf")


@dataclass
class RobotState:
    motors: Dict[str, MotorState] = field(default_factory=dict)
    legs: Dict[str, LegState] = field(default_factory=dict)
    motor_commands: Dict[str, MitCommand] = field(default_factory=dict)
    leg_commands: Dict[str, LegCommand] = field(default_factory=dict)
    imu: ImuState = field(default_factory=ImuState)
    timestamp: float = field(default_factory=time.monotonic)
    source: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class UserTarget:
    joint_positions: Dict[str, float] = field(default_factory=dict)
    wheel_torques: Dict[str, float] = field(
        default_factory=lambda: {"right_wheel": 0.0, "left_wheel": 0.0}
    )
    forward_velocity_m_s: float = 0.0
    yaw_rate_rad_s: float = 0.0


class Controller(Protocol):
    def reset(self, state: RobotState) -> None: ...

    def update(
        self, state: RobotState, target: UserTarget, dt: float
    ) -> Dict[str, MitCommand] | RobotCommand: ...


class Backend(Protocol):
    name: str

    def start(self) -> None: ...

    def read(self) -> RobotState: ...

    def write(self, commands: Dict[str, MitCommand]) -> None: ...

    def enable(self) -> None: ...

    def disable(self) -> None: ...

    def estop(self) -> None: ...

    def close(self) -> None: ...
