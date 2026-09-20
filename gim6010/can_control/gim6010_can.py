"""CANSimple library for SteadyWin/CyberBeast GIM6010 drives.

Implements the CAN Simple protocol from the CyberBeast Driver Protocol
Manual (manual/CyberBeast_Driver_Protocol_Manual_EN.md, section 1):
standard 11-bit frames, ``CAN ID = (node_id << 5) | cmd_id``, little-endian
payloads, IEEE-754 float32.

Only dependency: python-can (SocketCAN by default). The odrive USB package
is NOT required.

Feedback model
--------------
A background RX thread decodes every incoming frame into a per-node
``MotorState`` with monotonic timestamps, so periodic broadcasts (heartbeat,
encoder estimates, ...) are always available as ``motor.state``. On top of
that, every ``get_*()`` method sends an RTR (remote) frame and blocks for the
reply, giving true request/response semantics — use those right after a
torque command for synchronous feedback.

Units / frames
--------------
Classic CANSimple commands and feedback are ROTOR-side (rev, rev/s, motor Nm)
— manual §1.3, note under 0x08. With ``frame="output"`` the Motor converts
to/from the output shaft using ``ratio`` (8:1 on the GIM6010-8): pos/vel are
divided by ratio, torque multiplied, and commands the other way around.
MIT mode (0x08) is ALWAYS output-side in radians per the protocol itself and
is never converted here.
"""

from __future__ import annotations

import json
import logging
import struct
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import can

log = logging.getLogger("gim6010")

# --------------------------------------------------------------------------
# Protocol constants (protocol manual §1.3)
# --------------------------------------------------------------------------


class Cmd:
    HEARTBEAT = 0x01
    ESTOP = 0x02
    GET_ERROR = 0x03
    RXSDO = 0x04
    TXSDO = 0x05
    SET_AXIS_NODE_ID = 0x06
    SET_AXIS_STATE = 0x07
    MIT_CONTROL = 0x08
    GET_ENCODER_ESTIMATES = 0x09
    GET_ENCODER_COUNT = 0x0A
    SET_CONTROLLER_MODE = 0x0B
    SET_INPUT_POS = 0x0C
    SET_INPUT_VEL = 0x0D
    SET_INPUT_TORQUE = 0x0E
    SET_LIMITS = 0x0F
    START_ANTICOGGING = 0x10
    SET_TRAJ_VEL_LIMIT = 0x11
    SET_TRAJ_ACCEL_LIMITS = 0x12
    SET_TRAJ_INERTIA = 0x13
    GET_IQ = 0x14
    GET_TEMPERATURE = 0x15
    REBOOT = 0x16
    GET_BUS_VOLTAGE_CURRENT = 0x17
    CLEAR_ERRORS = 0x18
    SET_MOVE_INCREMENTAL = 0x19
    SET_POS_GAIN = 0x1A
    SET_VEL_GAINS = 0x1B
    GET_TORQUES = 0x1C
    GET_POWERS = 0x1D
    DISABLE_CAN = 0x1E
    SAVE_CONFIGURATION = 0x1F


AXIS_STATE = {
    0: "UNDEFINED",
    1: "IDLE",
    2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION_SEQUENCE",
    4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH",
    7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL",
    9: "LOCKIN_SPIN",
    10: "ENCODER_DIR_FIND",
    11: "HOMING",
    12: "ENCODER_HALL_POLARITY_CALIBRATION",
    13: "ENCODER_HALL_PHASE_CALIBRATION",
}

CONTROL_MODE = {0: "VOLTAGE", 1: "TORQUE", 2: "VELOCITY", 3: "POSITION"}

INPUT_MODE = {
    0: "INACTIVE",
    1: "PASSTHROUGH",
    2: "VEL_RAMP",
    3: "POS_FILTER",
    5: "TRAP_TRAJ",
    6: "TORQUE_RAMP",
    7: "MIRROR",
    8: "TUNING",
    9: "MIT",
}

# Error bit tables: ODrive 0.5.x flags per the protocol manual §4.1 plus the
# remaining upstream 0.5.x bits the (incomplete) manual table omits.
SYSTEM_ERRORS = {
    0x01: "CONTROL_ITERATION_MISSED",
    0x02: "DC_BUS_UNDER_VOLTAGE",
    0x04: "DC_BUS_OVER_VOLTAGE",
    0x08: "DC_BUS_OVER_REGEN_CURRENT",
    0x10: "DC_BUS_OVER_CURRENT",
    0x20: "BRAKE_DEADTIME_VIOLATION",
    0x40: "BRAKE_DUTY_CYCLE_NAN",
    0x80: "INVALID_BRAKE_RESISTANCE",
}

AXIS_ERRORS = {
    0x01: "INVALID_STATE",
    0x40: "MOTOR_FAILED",
    0x80: "SENSORLESS_ESTIMATOR_FAILED",
    0x100: "ENCODER_FAILED",
    0x200: "CONTROLLER_FAILED",
    0x800: "WATCHDOG_TIMER_EXPIRED",
    0x1000: "MIN_ENDSTOP_PRESSED",
    0x2000: "MAX_ENDSTOP_PRESSED",
    0x4000: "ESTOP_REQUESTED",
    0x20000: "HOMING_WITHOUT_ENDSTOP",
    0x40000: "OVER_TEMP",
    0x80000: "UNKNOWN_POSITION",
    0x100000: "CAN_BUS_FAILED",
}

MOTOR_ERRORS = {
    0x01: "PHASE_RESISTANCE_OUT_OF_RANGE",
    0x02: "PHASE_INDUCTANCE_OUT_OF_RANGE",
    0x08: "DRV_FAULT",
    0x10: "CONTROL_DEADLINE_MISSED",
    0x80: "MODULATION_MAGNITUDE",
    0x400: "CURRENT_SENSE_SATURATION",
    0x1000: "CURRENT_LIMIT_VIOLATION",
    0x10000: "MODULATION_IS_NAN",
    0x20000: "MOTOR_THERMISTOR_OVER_TEMP",
    0x40000: "FET_THERMISTOR_OVER_TEMP",
    0x80000: "TIMER_UPDATE_MISSED",
    0x100000: "CURRENT_MEASUREMENT_UNAVAILABLE",
    0x200000: "CONTROLLER_FAILED",
    0x400000: "I_BUS_OUT_OF_RANGE",
    0x800000: "BRAKE_RESISTOR_DISARMED",
    0x1000000: "SYSTEM_LEVEL",
    0x2000000: "BAD_TIMING",
    0x4000000: "UNKNOWN_PHASE_ESTIMATE",
    0x8000000: "UNKNOWN_PHASE_VEL",
    0x10000000: "UNKNOWN_TORQUE",
    0x20000000: "UNKNOWN_CURRENT_COMMAND",
    0x40000000: "UNKNOWN_CURRENT_MEASUREMENT",
    0x80000000: "UNKNOWN_VBUS_VOLTAGE",
    0x100000000: "UNKNOWN_VOLTAGE_COMMAND",
    0x200000000: "UNKNOWN_GAINS",
    0x400000000: "CONTROLLER_INITIALIZING",
    0x800000000: "UNBALANCED_PHASES",
}

ENCODER_ERRORS = {
    0x01: "UNSTABLE_GAIN",
    0x02: "CPR_POLEPAIRS_MISMATCH",
    0x04: "NO_RESPONSE",
    0x08: "UNSUPPORTED_ENCODER_MODE",
    0x10: "ILLEGAL_HALL_STATE",
    0x20: "INDEX_NOT_FOUND_YET",
    0x40: "ABS_SPI_TIMEOUT",
    0x80: "ABS_SPI_COM_FAIL",
    0x100: "ABS_SPI_NOT_READY",
    0x200: "HALL_NOT_CALIBRATED_YET",
    0x400: "SEC_ENC_COM_FAIL",  # SteadyWin extension
}

CONTROLLER_ERRORS = {
    0x01: "OVERSPEED",
    0x02: "INVALID_INPUT_MODE",
    0x04: "UNSTABLE_GAIN",
    0x08: "INVALID_MIRROR_AXIS",
    0x10: "INVALID_LOAD_ENCODER",
    0x20: "INVALID_ESTIMATE",
    0x40: "INVALID_CIRCULAR_RANGE",
    0x80: "SPINOUT_DETECTED",
}

# Periodic broadcast rate config endpoints (ms, 0 = off). Written over CAN
# via SDO; RAM-only until save_configuration().
RATE_ENDPOINTS = {
    "heartbeat": "axis0.config.can.heartbeat_rate_ms",
    "encoder": "axis0.config.can.encoder_rate_ms",
    "motor_error": "axis0.config.can.motor_error_rate_ms",
    "encoder_error": "axis0.config.can.encoder_error_rate_ms",
    "controller_error": "axis0.config.can.controller_error_rate_ms",
    "encoder_count": "axis0.config.can.encoder_count_rate_ms",
    "iq": "axis0.config.can.iq_rate_ms",
    "temperature": "axis0.config.can.thermistor_rate_ms",
    "bus_vi": "axis0.config.can.bus_vi_rate_ms",
}

# Get_Error (0x03) request type byte -> (name, decode table, response length)
ERROR_TYPES = {
    "motor": (0, MOTOR_ERRORS),
    "encoder": (1, ENCODER_ERRORS),
    "controller": (3, CONTROLLER_ERRORS),
    "system": (4, SYSTEM_ERRORS),
}


def decode_flags(table: Dict[int, str], value: int) -> List[str]:
    """Decode an error bitmask into flag names; unknown bits become hex."""
    names: List[str] = []
    remaining = int(value)
    for bit, name in table.items():
        if remaining & bit:
            names.append(name)
            remaining &= ~bit
    if remaining:
        names.append(f"0x{remaining:X}")
    return names


# --------------------------------------------------------------------------
# MIT mode packing (protocol manual §1.3 / 0x08)
# --------------------------------------------------------------------------


@dataclass
class MitLimits:
    """Must match the drive's mit_max_* configuration (defaults per manual)."""

    max_pos: float = 12.5     # rad, output shaft
    max_vel: float = 65.0     # rad/s
    max_kp: float = 500.0     # Nm/rad
    max_kd: float = 5.0       # Nm/(rad/s)
    max_torque: float = 50.0  # Nm


def _to_uint(x: float, lo: float, hi: float, bits: int) -> int:
    span = (1 << bits) - 1
    x = min(max(x, lo), hi)
    return int(round((x - lo) * span / (hi - lo)))


def _from_uint(u: int, lo: float, hi: float, bits: int) -> float:
    span = (1 << bits) - 1
    return u * (hi - lo) / span + lo


def mit_pack(pos: float, vel: float, kp: float, kd: float, torque: float,
             lim: MitLimits) -> bytes:
    p = _to_uint(pos, -lim.max_pos, lim.max_pos, 16)
    v = _to_uint(vel, -lim.max_vel, lim.max_vel, 12)
    kp_i = _to_uint(kp, 0.0, lim.max_kp, 12)
    kd_i = _to_uint(kd, 0.0, lim.max_kd, 12)
    t = _to_uint(torque, -lim.max_torque, lim.max_torque, 12)
    return bytes([
        (p >> 8) & 0xFF, p & 0xFF,
        (v >> 4) & 0xFF, ((v & 0xF) << 4) | ((kp_i >> 8) & 0xF), kp_i & 0xFF,
        (kd_i >> 4) & 0xFF, ((kd_i & 0xF) << 4) | ((t >> 8) & 0xF), t & 0xFF,
    ])


def mit_unpack(data: bytes, lim: MitLimits) -> Tuple[int, float, float, float]:
    """Decode a MIT feedback frame -> (node_id, pos rad, vel rad/s, torque Nm)."""
    node = data[0]
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0xF) << 8) | data[5]
    return (
        node,
        _from_uint(p, -lim.max_pos, lim.max_pos, 16),
        _from_uint(v, -lim.max_vel, lim.max_vel, 12),
        _from_uint(t, -lim.max_torque, lim.max_torque, 12),
    )


# --------------------------------------------------------------------------
# Endpoint (SDO) table
# --------------------------------------------------------------------------

_SDO_PACK = {
    "float": ("<f", float),
    "bool": ("<B", lambda v: 1 if v in (True, 1, "1", "true", "True") else 0),
    "uint8": ("<B", int),
    "uint16": ("<H", int),
    "uint32": ("<I", int),
    "int32": ("<i", int),
}


class EndpointTable:
    """Flattened ODrive endpoint tree from an endpoints_*.json file."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.by_path: Dict[str, dict] = {}
        self.by_id: Dict[int, str] = {}
        tree = json.loads(self.path.read_text())
        self._walk(tree, "")

    def _walk(self, entries: List[dict], prefix: str) -> None:
        for e in entries:
            name = e.get("name", "")
            full = f"{prefix}{name}" if name else prefix.rstrip(".")
            if "id" in e and e.get("type") not in ("json", "object"):
                self.by_path[full] = e
                self.by_id[e["id"]] = full
            if "members" in e:
                self._walk(e["members"], f"{full}." if full else "")

    def lookup(self, path: str) -> dict:
        e = self.by_path.get(path)
        if e is None:
            matches = [p for p in self.by_path if p.endswith(path)]
            if len(matches) == 1:
                e = self.by_path[matches[0]]
            elif len(matches) > 1:
                raise KeyError(f"'{path}' is ambiguous: {matches[:5]} ...")
            else:
                raise KeyError(f"endpoint '{path}' not found in {self.path.name}")
        if e["type"] not in _SDO_PACK:
            raise TypeError(
                f"endpoint '{path}' has type '{e['type']}' which cannot be "
                "accessed through the 4-byte SDO value field")
        return e

    def search(self, needle: str) -> List[str]:
        return sorted(p for p in self.by_path if needle.lower() in p.lower())


# --------------------------------------------------------------------------
# Per-motor live state (filled by the RX thread)
# --------------------------------------------------------------------------


@dataclass
class MotorState:
    # heartbeat (0x01)
    axis_error: Optional[int] = None
    axis_state: Optional[int] = None
    motor_err: bool = False
    encoder_err: bool = False
    controller_err: bool = False
    system_err: bool = False
    traj_done: bool = False
    temperature: Optional[int] = None  # max(motor, fet), °C
    life: Optional[int] = None
    lost_heartbeats: int = 0
    t_heartbeat: float = 0.0
    # encoder estimates (0x09) — rotor frame as received
    pos: Optional[float] = None
    vel: Optional[float] = None
    t_encoder: float = 0.0
    # encoder count (0x0A)
    shadow_count: Optional[int] = None
    count_in_cpr: Optional[int] = None
    t_count: float = 0.0
    # iq (0x14)
    iq_setpoint: Optional[float] = None
    iq_measured: Optional[float] = None
    t_iq: float = 0.0
    # torques (0x1C) — rotor frame
    torque_setpoint: Optional[float] = None
    torque: Optional[float] = None
    t_torque: float = 0.0
    # temperature (0x15)
    motor_temp: Optional[float] = None
    fet_temp: Optional[float] = None
    t_temp: float = 0.0
    # bus (0x17)
    vbus: Optional[float] = None
    ibus: Optional[float] = None
    t_bus: float = 0.0
    # powers (0x1D)
    electrical_power: Optional[float] = None
    mechanical_power: Optional[float] = None
    t_power: float = 0.0
    # MIT feedback (0x08) — output frame, rad
    mit_pos: Optional[float] = None
    mit_vel: Optional[float] = None
    mit_torque: Optional[float] = None
    t_mit: float = 0.0

    @property
    def any_error(self) -> bool:
        return bool(self.axis_error) or self.motor_err or self.encoder_err \
            or self.controller_err or self.system_err

    def error_summary(self) -> str:
        """Return decoded heartbeat errors without issuing CAN requests."""
        parts: List[str] = []
        if self.axis_error:
            names = decode_flags(AXIS_ERRORS, self.axis_error)
            parts.append(f"axis={','.join(names)}")
        for active, name in (
            (self.motor_err, "motor"),
            (self.encoder_err, "encoder"),
            (self.controller_err, "controller"),
            (self.system_err, "system"),
        ):
            if active:
                parts.append(f"{name}=FAULT")
        return "; ".join(parts)

    def axis_state_name(self) -> str:
        if self.axis_state is None:
            return "?"
        return AXIS_STATE.get(self.axis_state, str(self.axis_state))

    def encoder_age(self) -> float:
        """Seconds since the last pos/vel frame (inf if never seen)."""
        return time.monotonic() - self.t_encoder if self.t_encoder else float("inf")

    def heartbeat_age(self) -> float:
        return time.monotonic() - self.t_heartbeat if self.t_heartbeat else float("inf")


class Timeout(Exception):
    pass


# --------------------------------------------------------------------------
# Bus + Motor
# --------------------------------------------------------------------------


class GIM6010Bus:
    """Owns the CAN bus and the RX thread; hands out Motor objects."""

    def __init__(self, channel: str = "can0", interface: str = "socketcan",
                 bus: Optional[can.BusABC] = None, ratio: float = 8.0,
                 frame: str = "motor",
                 endpoints: Optional[Path] = None):
        if frame not in ("motor", "output"):
            raise ValueError("frame must be 'motor' or 'output'")
        self.bus = bus or can.Bus(channel=channel, interface=interface)
        self.ratio = float(ratio)
        self.frame = frame
        self.endpoints: Optional[EndpointTable] = (
            EndpointTable(endpoints) if endpoints else None)
        self._motors: Dict[int, "Motor"] = {}
        self._lock = threading.Lock()
        self._pending: Dict[Tuple[int, int], threading.Event] = {}
        self._replies: Dict[Tuple[int, int], bytes] = {}
        self._stop = threading.Event()
        self._rx = threading.Thread(target=self._rx_loop, daemon=True,
                                    name="gim6010-rx")
        self._rx.start()

    # -- public ------------------------------------------------------------

    def motor(self, node_id: int) -> "Motor":
        if not 0 <= node_id <= 63:
            raise ValueError("node_id must be 0..63")
        with self._lock:
            if node_id not in self._motors:
                self._motors[node_id] = Motor(self, node_id)
            return self._motors[node_id]

    def nodes(self) -> List[int]:
        """Node IDs seen on the bus so far (via any decoded frame)."""
        with self._lock:
            return sorted(n for n, m in self._motors.items()
                          if m.state.t_heartbeat or m.state.t_encoder)

    def scan(self, duration: float = 1.5) -> List[int]:
        """Listen for heartbeats/broadcasts for `duration` seconds."""
        time.sleep(duration)
        return self.nodes()

    def estop_all(self) -> List[int]:
        """Send Estop to every node seen on the bus. Returns the node list."""
        nodes = self.nodes()
        for n in nodes:
            self.motor(n).estop()
        return nodes

    def close(self) -> None:
        self._stop.set()
        self._rx.join(timeout=1.0)
        self.bus.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- internals -----------------------------------------------------------

    def _send(self, node_id: int, cmd: int, data: bytes = b"",
              rtr: bool = False) -> None:
        msg = can.Message(
            arbitration_id=(node_id << 5) | cmd,
            is_extended_id=False,
            is_remote_frame=rtr,
            dlc=8 if rtr else len(data),
            data=None if rtr else data,
        )
        self.bus.send(msg)

    def _request(self, node_id: int, cmd: int, data: bytes = b"",
                 rtr: bool = True, timeout: float = 0.5) -> bytes:
        """Send a frame and wait for the motor's reply with the same cmd id
        (TxSdo 0x05 for RxSdo 0x04 requests)."""
        reply_cmd = Cmd.TXSDO if cmd == Cmd.RXSDO else cmd
        key = (node_id, reply_cmd)
        ev = threading.Event()
        with self._lock:
            self._pending[key] = ev
            self._replies.pop(key, None)
        self._send(node_id, cmd, data, rtr=rtr)
        if not ev.wait(timeout):
            with self._lock:
                self._pending.pop(key, None)
            raise Timeout(
                f"node {node_id}: no reply to cmd 0x{cmd:02X} within {timeout}s")
        with self._lock:
            self._pending.pop(key, None)
            return self._replies[key]

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self.bus.recv(timeout=0.2)
            except Exception:
                if self._stop.is_set():
                    return
                continue
            if msg is None or msg.is_error_frame or msg.is_remote_frame:
                continue
            if msg.is_extended_id:
                continue
            node_id = (msg.arbitration_id >> 5) & 0x3F
            cmd = msg.arbitration_id & 0x1F
            motor = self.motor(node_id)
            try:
                motor._decode(cmd, bytes(msg.data))
            except Exception:
                log.exception("decode failure cmd=0x%02X data=%s",
                              cmd, bytes(msg.data).hex())
            key = (node_id, cmd)
            with self._lock:
                ev = self._pending.get(key)
                if ev is not None:
                    self._replies[key] = bytes(msg.data)
                    ev.set()


class Motor:
    """One drive on the bus. All values rotor-frame unless the bus was
    created with frame='output' (MIT excluded — always output/rad)."""

    def __init__(self, bus: GIM6010Bus, node_id: int):
        self._bus = bus
        self.node_id = node_id
        self.state = MotorState()
        self.mit_limits = MitLimits()
        self.on_heartbeat: Optional[Callable[[MotorState], None]] = None
        self.on_encoder: Optional[Callable[[MotorState], None]] = None
        # Set by the RX thread on every pos/vel broadcast. A 500 Hz control
        # loop can pace itself off the drive instead of a local timer:
        #   m.encoder_event.wait(0.004); m.encoder_event.clear()
        self.encoder_event = threading.Event()

    # -- frame conversion helpers -------------------------------------------

    def _cmd_pos(self, v: float) -> float:   # user -> rotor
        return v * self._bus.ratio if self._bus.frame == "output" else v

    def _cmd_torque(self, v: float) -> float:
        return v / self._bus.ratio if self._bus.frame == "output" else v

    def _fb_pos(self, v: float) -> float:    # rotor -> user
        return v / self._bus.ratio if self._bus.frame == "output" else v

    def _fb_torque(self, v: float) -> float:
        return v * self._bus.ratio if self._bus.frame == "output" else v

    # -- commands (Host -> Motor) ---------------------------------------------

    def estop(self) -> None:
        """0x02: emergency stop. Latches ESTOP_REQUESTED — recover with
        clear_errors() and re-entering closed loop."""
        self._bus._send(self.node_id, Cmd.ESTOP, bytes(8))

    def clear_errors(self) -> None:
        self._bus._send(self.node_id, Cmd.CLEAR_ERRORS, bytes(8))

    def reboot(self) -> None:
        self._bus._send(self.node_id, Cmd.REBOOT, bytes(8))

    def save_configuration(self) -> None:
        """0x1F: persist config to flash and reboot the drive."""
        self._bus._send(self.node_id, Cmd.SAVE_CONFIGURATION, bytes(8))

    def set_axis_state(self, state: int) -> None:
        self._bus._send(self.node_id, Cmd.SET_AXIS_STATE,
                        struct.pack("<I", state))

    def set_controller_mode(self, control_mode: int, input_mode: int) -> None:
        self._bus._send(self.node_id, Cmd.SET_CONTROLLER_MODE,
                        struct.pack("<II", control_mode, input_mode))

    def set_input_torque(self, torque_nm: float) -> None:
        self._bus._send(self.node_id, Cmd.SET_INPUT_TORQUE,
                        struct.pack("<f", self._cmd_torque(torque_nm)))

    def set_input_vel(self, vel: float, torque_ff: float = 0.0) -> None:
        self._bus._send(self.node_id, Cmd.SET_INPUT_VEL,
                        struct.pack("<ff", self._cmd_pos(vel),
                                    self._cmd_torque(torque_ff)))

    def set_input_pos(self, pos: float, vel_ff: float = 0.0,
                      torque_ff: float = 0.0) -> None:
        # Vel_FF is int16 in 0.001 rev/s, Torque_FF int16 in 0.001 Nm.
        self._bus._send(self.node_id, Cmd.SET_INPUT_POS, struct.pack(
            "<fhh", self._cmd_pos(pos),
            int(round(self._cmd_pos(vel_ff) * 1000)),
            int(round(self._cmd_torque(torque_ff) * 1000))))

    def set_limits(self, vel_limit: float, current_limit: float) -> None:
        """Rotor-frame rev/s and motor amps (no frame conversion — these map
        straight onto firmware registers)."""
        self._bus._send(self.node_id, Cmd.SET_LIMITS,
                        struct.pack("<ff", vel_limit, current_limit))

    def set_pos_gain(self, kp: float) -> None:
        self._bus._send(self.node_id, Cmd.SET_POS_GAIN, struct.pack("<f", kp))

    def set_vel_gains(self, kp: float, ki: float) -> None:
        self._bus._send(self.node_id, Cmd.SET_VEL_GAINS,
                        struct.pack("<ff", kp, ki))

    def set_traj_vel_limit(self, v: float) -> None:
        self._bus._send(self.node_id, Cmd.SET_TRAJ_VEL_LIMIT,
                        struct.pack("<f", v))

    def set_traj_accel_limits(self, accel: float, decel: float) -> None:
        self._bus._send(self.node_id, Cmd.SET_TRAJ_ACCEL_LIMITS,
                        struct.pack("<ff", accel, decel))

    def set_traj_inertia(self, inertia: float) -> None:
        self._bus._send(self.node_id, Cmd.SET_TRAJ_INERTIA,
                        struct.pack("<f", inertia))

    def move_incremental(self, displacement: float) -> None:
        """0x19: relative move, rotor-side revolutions per the manual."""
        self._bus._send(self.node_id, Cmd.SET_MOVE_INCREMENTAL,
                        struct.pack("<f", displacement) + bytes(4))

    def set_node_id(self, new_id: int) -> None:
        self._bus._send(self.node_id, Cmd.SET_AXIS_NODE_ID,
                        struct.pack("<I", new_id))

    def start_anticogging(self) -> None:
        self._bus._send(self.node_id, Cmd.START_ANTICOGGING, bytes(8))

    # -- high-level helpers ---------------------------------------------------

    def enable_torque_mode(self, ramp: bool = False,
                           reconcile: bool = True,
                           timeout: float = 1.0) -> None:
        """Torque control + closed loop, with setpoint reconciliation:
        zero torque/vel inputs and pin input_pos to the current position
        before enabling, so the axis holds instead of jumping."""
        self.set_controller_mode(1, 6 if ramp else 1)
        if reconcile:
            self._reconcile(timeout)
        self.set_input_torque(0.0)
        self.set_axis_state(8)

    def enable_velocity_mode(self, ramp: bool = True,
                             reconcile: bool = True,
                             timeout: float = 1.0) -> None:
        self.set_controller_mode(2, 2 if ramp else 1)
        if reconcile:
            self._reconcile(timeout)
        self.set_input_vel(0.0)
        self.set_axis_state(8)

    def enable_position_mode(self, input_mode: int = 3,
                             reconcile: bool = True,
                             timeout: float = 1.0) -> None:
        """input_mode: 1=passthrough, 3=filtered, 5=trap trajectory."""
        self.set_controller_mode(3, input_mode)
        if reconcile:
            self._reconcile(timeout)
        self.set_axis_state(8)

    def idle(self) -> None:
        self.set_axis_state(1)

    def _reconcile(self, timeout: float) -> None:
        try:
            pos, _ = self.get_encoder_estimates(timeout=timeout)
        except Timeout:
            pos = None
        self.set_input_torque(0.0)
        self.set_input_vel(0.0, 0.0)
        if pos is not None:
            self.set_input_pos(pos, 0.0, 0.0)

    # -- MIT mode ---------------------------------------------------------------

    def mit_control(self, pos: float = 0.0, vel: float = 0.0, kp: float = 0.0,
                    kd: float = 0.0, torque: float = 0.0,
                    wait_feedback: bool = True, timeout: float = 0.2
                    ) -> Optional[Tuple[float, float, float]]:
        """0x08: MIT command. OUTPUT-shaft frame, radians — never converted.
        Requires control_mode 3 (POSITION) + input_mode 9 (MIT) + closed
        loop, per user manual §3.3.4 — with control_mode 1 the firmware
        ignores the MIT gains and produces no torque. With wait_feedback the
        drive's echo frame is awaited and (pos, vel, torque) returned."""
        data = mit_pack(pos, vel, kp, kd, torque, self.mit_limits)
        if not wait_feedback:
            self._bus._send(self.node_id, Cmd.MIT_CONTROL, data)
            return None
        reply = self._bus._request(self.node_id, Cmd.MIT_CONTROL, data,
                                   rtr=False, timeout=timeout)
        _, p, v, t = mit_unpack(reply, self.mit_limits)
        return p, v, t

    # -- synchronous getters (RTR request/response) -------------------------------

    def get_encoder_estimates(self, timeout: float = 0.5
                              ) -> Tuple[float, float]:
        """(pos, vel). Live only after the axis has been in closed loop once."""
        d = self._bus._request(self.node_id, Cmd.GET_ENCODER_ESTIMATES,
                               timeout=timeout)
        pos, vel = struct.unpack("<ff", d)
        return self._fb_pos(pos), self._fb_pos(vel)

    def get_encoder_count(self, timeout: float = 0.5) -> Tuple[int, int]:
        d = self._bus._request(self.node_id, Cmd.GET_ENCODER_COUNT,
                               timeout=timeout)
        return struct.unpack("<ii", d)

    def get_iq(self, timeout: float = 0.5) -> Tuple[float, float]:
        d = self._bus._request(self.node_id, Cmd.GET_IQ, timeout=timeout)
        return struct.unpack("<ff", d)

    def get_torques(self, timeout: float = 0.5) -> Tuple[float, float]:
        d = self._bus._request(self.node_id, Cmd.GET_TORQUES, timeout=timeout)
        sp, tq = struct.unpack("<ff", d)
        return self._fb_torque(sp), self._fb_torque(tq)

    def get_temperature(self, timeout: float = 0.5) -> Tuple[float, float]:
        d = self._bus._request(self.node_id, Cmd.GET_TEMPERATURE,
                               timeout=timeout)
        return struct.unpack("<ff", d)

    def get_bus_voltage_current(self, timeout: float = 0.5
                                ) -> Tuple[float, float]:
        d = self._bus._request(self.node_id, Cmd.GET_BUS_VOLTAGE_CURRENT,
                               timeout=timeout)
        return struct.unpack("<ff", d)

    def get_powers(self, timeout: float = 0.5) -> Tuple[float, float]:
        d = self._bus._request(self.node_id, Cmd.GET_POWERS, timeout=timeout)
        return struct.unpack("<ff", d)

    def get_error(self, which: str, timeout: float = 0.5) -> Tuple[int, List[str]]:
        """which: 'motor' | 'encoder' | 'controller' | 'system'.
        Returns (raw code, decoded flag names)."""
        type_byte, table = ERROR_TYPES[which]
        d = self._bus._request(self.node_id, Cmd.GET_ERROR,
                               bytes([type_byte]) + bytes(7),
                               rtr=False, timeout=timeout)
        raw = int.from_bytes(d, "little")  # 4 or 8 bytes depending on type
        return raw, decode_flags(table, raw)

    def get_all_errors(self, timeout: float = 0.5) -> Dict[str, Tuple[int, List[str]]]:
        return {w: self.get_error(w, timeout) for w in ERROR_TYPES}

    # -- SDO (arbitrary endpoint access) ----------------------------------------

    def _endpoints(self) -> EndpointTable:
        if self._bus.endpoints is None:
            raise RuntimeError(
                "no endpoints JSON loaded — pass endpoints=... to GIM6010Bus")
        return self._bus.endpoints

    def sdo_read(self, path: str, timeout: float = 0.5):
        e = self._endpoints().lookup(path)
        fmt, _ = _SDO_PACK[e["type"]]
        req = struct.pack("<BHB", 0, e["id"], 0) + bytes(4)
        d = self._bus._request(self.node_id, Cmd.RXSDO, req, rtr=False,
                               timeout=timeout)
        _, eid, _ = struct.unpack("<BHB", d[:4])
        if eid != e["id"]:
            raise IOError(f"TxSdo answered endpoint {eid}, expected {e['id']}")
        size = struct.calcsize(fmt)
        return struct.unpack(fmt, d[4:4 + size])[0]

    def sdo_write(self, path: str, value, verify: bool = True,
                  timeout: float = 0.5):
        e = self._endpoints().lookup(path)
        if "w" not in e.get("access", "rw"):
            raise PermissionError(f"endpoint '{path}' is read-only")
        fmt, conv = _SDO_PACK[e["type"]]
        payload = struct.pack(fmt, conv(value))
        req = struct.pack("<BHB", 1, e["id"], 0) + payload.ljust(4, b"\0")
        self._bus._send(self.node_id, Cmd.RXSDO, req)
        if verify:
            time.sleep(0.02)
            back = self.sdo_read(path, timeout=timeout)
            if e["type"] == "float":
                want = conv(value)
                ok = abs(back - want) <= max(1e-5, 1e-5 * abs(want))
            else:
                ok = int(back) == int(conv(value))
            if not ok:
                raise IOError(f"write verify failed: wrote {value}, read {back}")
            return back
        return None

    def set_deadman(self, ms: int) -> None:
        """Enable the CAN communication-timeout stop (user manual §5.1.9):
        if the command stream halts for `ms` milliseconds the drive faults.
        0 disables. RAM-only until save_configuration()."""
        self.sdo_write("can.config.break_timeout", ms)

    def set_feedback_rates(self, **rates: int) -> None:
        """Set periodic broadcast periods in ms (0 = off) over CAN.
        Keys: heartbeat, encoder, iq, temperature, bus_vi, encoder_count,
        motor_error, encoder_error, controller_error (see RATE_ENDPOINTS).
        e.g. m.set_feedback_rates(encoder=2, iq=10). RAM-only until
        save_configuration()."""
        for key, ms in rates.items():
            if key not in RATE_ENDPOINTS:
                raise KeyError(f"unknown rate '{key}' — one of "
                               f"{sorted(RATE_ENDPOINTS)}")
            self.sdo_write(RATE_ENDPOINTS[key], int(ms))

    def get_feedback_rates(self) -> Dict[str, int]:
        return {k: self.sdo_read(p) for k, p in RATE_ENDPOINTS.items()}

    # -- RX decoding (called from the bus thread) ----------------------------------

    def _decode(self, cmd: int, d: bytes) -> None:
        s = self.state
        now = time.monotonic()
        if cmd == Cmd.HEARTBEAT and len(d) >= 8:
            s.axis_error = struct.unpack("<I", d[0:4])[0]
            s.axis_state = d[4]
            flags = d[5]
            s.motor_err = bool(flags & 1)
            s.encoder_err = bool(flags & 2)
            s.controller_err = bool(flags & 4)
            s.system_err = bool(flags & 8)
            s.traj_done = bool(flags & 0x80)
            s.temperature = struct.unpack("<b", d[6:7])[0]
            life = d[7]
            if s.life is not None and (life - s.life) % 256 != 1:
                s.lost_heartbeats += ((life - s.life) % 256) - 1
            s.life = life
            s.t_heartbeat = now
            if self.on_heartbeat:
                self.on_heartbeat(s)
        elif cmd == Cmd.GET_ENCODER_ESTIMATES and len(d) >= 8:
            s.pos, s.vel = struct.unpack("<ff", d)
            s.t_encoder = now
            self.encoder_event.set()
            if self.on_encoder:
                self.on_encoder(s)
        elif cmd == Cmd.GET_ENCODER_COUNT and len(d) >= 8:
            s.shadow_count, s.count_in_cpr = struct.unpack("<ii", d)
            s.t_count = now
        elif cmd == Cmd.GET_IQ and len(d) >= 8:
            s.iq_setpoint, s.iq_measured = struct.unpack("<ff", d)
            s.t_iq = now
        elif cmd == Cmd.GET_TORQUES and len(d) >= 8:
            s.torque_setpoint, s.torque = struct.unpack("<ff", d)
            s.t_torque = now
        elif cmd == Cmd.GET_TEMPERATURE and len(d) >= 8:
            s.motor_temp, s.fet_temp = struct.unpack("<ff", d)
            s.t_temp = now
        elif cmd == Cmd.GET_BUS_VOLTAGE_CURRENT and len(d) >= 8:
            s.vbus, s.ibus = struct.unpack("<ff", d)
            s.t_bus = now
        elif cmd == Cmd.GET_POWERS and len(d) >= 8:
            s.electrical_power, s.mechanical_power = struct.unpack("<ff", d)
            s.t_power = now
        elif cmd == Cmd.MIT_CONTROL and len(d) >= 6:
            _, s.mit_pos, s.mit_vel, s.mit_torque = mit_unpack(d, self.mit_limits)
            s.t_mit = now
