"""Read a PS5 DualSense controller without coupling it to robot control.

This module only turns Linux input events into immutable snapshots. It does
not start a thread, open a device at import time, or send commands to the
SandroControl runtime. The application using it owns polling and routing.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
import select
import time
from typing import Any

try:
    from evdev import InputDevice, ecodes, list_devices
except ImportError:  # Keep the rest of SandroControl usable without gamepads.
    InputDevice = None
    ecodes = None
    list_devices = None


class ControllerNotFound(RuntimeError):
    """No matching DualSense event device was found."""


class ControllerDisconnected(RuntimeError):
    """The selected controller disappeared while being read."""


@dataclass(frozen=True)
class Ps5Snapshot:
    """Normalized controller state.

    Stick axes are in [-1, 1]. Positive ``left_y`` and ``right_y`` mean up,
    unlike Linux's raw axis convention. Triggers are in [0, 1]. Buttons use
    stable names such as ``cross``, ``circle``, ``l1`` and ``dpad_up``.
    """

    connected: bool = False
    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0
    l2: float = 0.0
    r2: float = 0.0
    buttons: frozenset[str] = frozenset()
    timestamp: float = 0.0


@dataclass(frozen=True)
class DriveCommand:
    """Backend-independent driving request derived from a controller snapshot."""

    forward_velocity_m_s: float
    yaw_rate_rad_s: float


_BUTTON_NAMES = {
    304: "cross",       # BTN_SOUTH
    305: "circle",      # BTN_EAST
    307: "triangle",    # BTN_NORTH
    308: "square",      # BTN_WEST
    310: "l1",          # BTN_TL
    311: "r1",          # BTN_TR
    312: "l2",          # BTN_TL2
    313: "r2",          # BTN_TR2
    314: "create",      # BTN_SELECT
    315: "options",     # BTN_START
    316: "ps",          # BTN_MODE
    317: "left_stick",  # BTN_THUMBL
    318: "right_stick", # BTN_THUMBR
}


def snapshot_to_drive_command(
    snapshot: Ps5Snapshot,
    *,
    max_forward_velocity_m_s: float,
    max_yaw_rate_rad_s: float,
    deadzone: float = 0.08,
) -> DriveCommand:
    """Map left-stick forward and right-stick horizontal to an SI request.

    This pure helper is optional: callers can instead map any snapshot fields
    to their own command type. A disconnected snapshot always maps to zero.
    """
    if not 0.0 <= deadzone < 1.0:
        raise ValueError("deadzone must be in [0, 1)")
    if not math.isfinite(max_forward_velocity_m_s) or max_forward_velocity_m_s < 0:
        raise ValueError("max_forward_velocity_m_s must be finite and non-negative")
    if not math.isfinite(max_yaw_rate_rad_s) or max_yaw_rate_rad_s < 0:
        raise ValueError("max_yaw_rate_rad_s must be finite and non-negative")
    if not snapshot.connected:
        return DriveCommand(0.0, 0.0)
    return DriveCommand(
        forward_velocity_m_s=(
            _apply_deadzone(snapshot.left_y, deadzone) * max_forward_velocity_m_s
        ),
        yaw_rate_rad_s=(
            _apply_deadzone(snapshot.right_x, deadzone) * max_yaw_rate_rad_s
        ),
    )


class Ps5Controller:
    """Explicitly connected, caller-polled Linux DualSense reader."""

    NAME_MARKERS = ("dualsense", "wireless controller")

    def __init__(self, device_path: str | None = None) -> None:
        self.device_path = device_path
        self._device: Any | None = None
        self._axis_info: dict[int, Any] = {}
        self._snapshot = Ps5Snapshot()

    @property
    def snapshot(self) -> Ps5Snapshot:
        return self._snapshot

    @property
    def name(self) -> str | None:
        return None if self._device is None else str(self._device.name)

    @classmethod
    def discover(cls) -> list[tuple[str, str]]:
        """Return ``(event-device path, device name)`` for likely DualSense pads."""
        _require_evdev()
        found = []
        for path in list_devices():
            try:
                device = InputDevice(path)
            except (FileNotFoundError, PermissionError, OSError):
                continue        # Unreadable or unplugged between listing and open.
            try:
                name = device.name or ""
                # absinfo=False keeps this a list of axis codes; the default
                # returns (code, AbsInfo) pairs, which never compare equal.
                capabilities = device.capabilities(absinfo=False)
                axes = capabilities.get(ecodes.EV_ABS, ())
                keys = capabilities.get(ecodes.EV_KEY, ())
                # BTN_SOUTH separates the pad itself from the motion-sensor and
                # touchpad nodes, which share its name and also report ABS_X/Y.
                is_gamepad = (
                    ecodes.ABS_X in axes
                    and ecodes.ABS_Y in axes
                    and ecodes.BTN_SOUTH in keys
                )
                if (
                    is_gamepad
                    and any(marker in name.lower() for marker in cls.NAME_MARKERS)
                ):
                    found.append((path, name))
            finally:
                device.close()
        return found

    def connect(self) -> Ps5Snapshot:
        """Open the requested device, or the first discovered DualSense device."""
        _require_evdev()
        if self._device is not None:
            return self._snapshot
        path = self.device_path
        if path is None:
            devices = self.discover()
            if not devices:
                raise ControllerNotFound(
                    "no PS5 DualSense controller found under /dev/input/event*"
                )
            path = devices[0][0]
        try:
            self._device = InputDevice(path)
            capabilities = self._device.capabilities(absinfo=True)
            self._axis_info = dict(capabilities.get(ecodes.EV_ABS, ()))
        except (FileNotFoundError, PermissionError, OSError) as exc:
            self._device = None
            raise ControllerNotFound(f"cannot open controller {path}: {exc}") from exc
        self.device_path = path
        self._snapshot = Ps5Snapshot(connected=True, timestamp=time.monotonic())
        return self._snapshot

    def poll(self, timeout_s: float = 0.0) -> Ps5Snapshot:
        """Consume available events and return the latest complete snapshot."""
        if self._device is None:
            raise ControllerDisconnected("controller is not connected")
        if timeout_s < 0 or not math.isfinite(timeout_s):
            raise ValueError("timeout_s must be finite and non-negative")
        try:
            readable, _, _ = select.select([self._device.fd], [], [], timeout_s)
            if readable:
                for event in self._device.read():
                    self._consume(event)
        except (BlockingIOError, InterruptedError):
            pass
        except (FileNotFoundError, OSError) as exc:
            self.close()
            raise ControllerDisconnected("PS5 controller disconnected") from exc
        return self._snapshot

    def close(self) -> None:
        if self._device is not None:
            try:
                self._device.close()
            except OSError:
                pass
            self._device = None
        self._axis_info = {}
        self._snapshot = replace(
            self._snapshot, connected=False, timestamp=time.monotonic()
        )

    def __enter__(self) -> "Ps5Controller":
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _consume(self, event: Any) -> None:
        now = time.monotonic()
        if event.type == ecodes.EV_KEY:
            name = _BUTTON_NAMES.get(event.code)
            if name is not None:
                buttons = set(self._snapshot.buttons)
                (buttons.add if event.value else buttons.discard)(name)
                self._snapshot = replace(
                    self._snapshot, buttons=frozenset(buttons), timestamp=now
                )
            return
        if event.type != ecodes.EV_ABS:
            return

        value = _normalize_axis(event.value, self._axis_info.get(event.code))
        fields: dict[str, float | frozenset[str]] = {}
        if event.code == ecodes.ABS_X:
            fields["left_x"] = value
        elif event.code == ecodes.ABS_Y:
            fields["left_y"] = -value
        elif event.code == ecodes.ABS_RX:
            fields["right_x"] = value
        elif event.code == ecodes.ABS_RY:
            fields["right_y"] = -value
        elif event.code == ecodes.ABS_Z:
            fields["l2"] = (value + 1.0) / 2.0
        elif event.code == ecodes.ABS_RZ:
            fields["r2"] = (value + 1.0) / 2.0
        elif event.code in (ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y):
            fields["buttons"] = self._dpad_buttons(event.code, event.value)
        if fields:
            self._snapshot = replace(self._snapshot, timestamp=now, **fields)

    def _dpad_buttons(self, code: int, value: int) -> frozenset[str]:
        buttons = set(self._snapshot.buttons)
        if code == ecodes.ABS_HAT0X:
            buttons.discard("dpad_left")
            buttons.discard("dpad_right")
            if value < 0:
                buttons.add("dpad_left")
            elif value > 0:
                buttons.add("dpad_right")
        else:
            buttons.discard("dpad_up")
            buttons.discard("dpad_down")
            if value < 0:
                buttons.add("dpad_up")
            elif value > 0:
                buttons.add("dpad_down")
        return frozenset(buttons)


def _require_evdev() -> None:
    if InputDevice is None:
        raise RuntimeError(
            "PS5 input requires the optional 'evdev' package; "
            "install SandroControl requirements"
        )


def _normalize_axis(value: int, info: Any | None) -> float:
    if info is None or info.max <= info.min:
        return 0.0
    normalized = 2.0 * (float(value) - info.min) / (info.max - info.min) - 1.0
    return max(-1.0, min(1.0, normalized))


def _apply_deadzone(value: float, deadzone: float) -> float:
    value = max(-1.0, min(1.0, float(value)))
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    return math.copysign((magnitude - deadzone) / (1.0 - deadzone), value)
