"""Passive GIM6010 bus listener: what every drive sends, also before the drives are armed.

A raw CAN socket with a kernel filter receives only the drives' heartbeats and
encoder broadcasts, about 1.3k frames/s; the MIT traffic of an armed robot
(another 4-5k frames/s) never reaches Python. The bus's total rate comes from
the interface counters instead (system.can_link). The monitor transmits nothing.

Frame ids are (node << 5) | command (sandro_hardware/protocol.hpp). The
heartbeat carries axis error, axis state, fault flags and temperature, the
encoder broadcast the rotor position (revolutions) and velocity, which the
joint mapping of the ros2_control description turns into logical radians.

The one exception to read-only is EstopBus: a separate socket that can only
send the GIM6010 E-stop command, as sandro_hardware's gim6010_estop node does,
so the panel's E-STOP reaches the drives with no launch and no DDS.
"""

from __future__ import annotations

import math
import re
import socket
import struct
import threading
import time
from pathlib import Path

HEARTBEAT, ESTOP, MIT, ENCODER, IQ = 0x01, 0x02, 0x08, 0x09, 0x14
COMMANDS = {0x01: "heartbeat", 0x02: "estop", 0x04: "sdo_rx", 0x05: "sdo_tx", 0x07: "axis_state",
            0x08: "mit", 0x09: "encoder", 0x0B: "controller_mode", 0x14: "iq", 0x18: "clear_errors"}
AXIS_STATES = {0: "undefined", 1: "idle", 8: "closed loop"}
ESTOP_REQUESTED, CAN_WATCHDOG = 0x4000, 0x100000
_FRAME = struct.Struct("=IB3x8s")


def operator_clearable(error: int, flags: int) -> bool:
    """sandro_hardware protocol.hpp operator_clearable_error: only the E-stop and watchdog bits."""
    return error != 0 and (error & ~(ESTOP_REQUESTED | CAN_WATCHDOG)) == 0 and flags == 0


def estop_frame(node: int) -> bytes:
    """The struct can_frame estop_node writes: id (node << 5) | 0x02, 8 zero bytes."""
    if not 0 <= node <= 63:
        raise ValueError("GIM6010 node_id must be in [0, 63]")
    return _FRAME.pack(node << 5 | ESTOP, 8, bytes(8))


class EstopBus:
    """Sends the E-stop frame to every drive, and nothing else."""

    def __init__(self, interface: str, nodes: list[int]) -> None:
        self.interface = interface
        self.frames = [estop_frame(n) for n in nodes]

    def send(self) -> str:
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        except (OSError, AttributeError) as error:
            return f"CAN E-stop from the panel failed: {error}"
        try:
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b"")
            sock.settimeout(0.2)
            sock.bind((self.interface,))
            for frame in self.frames:
                sock.send(frame)
        except OSError as error:
            return f"CAN E-stop from the panel failed: {error}"
        finally:
            sock.close()
        return f"CAN E-stop sent by the panel to {len(self.frames)} drives on {self.interface}"


def drive_faults(nodes: list[dict]) -> list[str]:
    """One sentence per drive that keeps the hardware from arming, with what to do."""
    out = []
    for n in nodes:
        if not n.get("joint"):
            continue
        name = f"{n['joint']} (node {n['node']})"
        if n.get("heartbeat_age_s") is None or n["heartbeat_age_s"] > 1.0:
            out.append(f"{name} is silent: check its CAN cable and power")
            continue
        error, flags = n.get("axis_error") or 0, n.get("fault_flags") or 0
        if not error and not flags:
            continue
        text = axis_error_text(error, flags)
        if operator_clearable(error, flags):
            out.append(f"{name} {text}: clearable (E-stop or watchdog latch); arming clears it, "
                       "or Clear drive errors and Arm again")
        else:
            out.append(f"{name} {text}: a drive fault that is not clearable from the panel; "
                       "inspect the drive (Run log, temperature, encoder), power-cycle it once "
                       "the cause is fixed")
    return out


def axis_error_text(error: int, flags: int) -> str:
    if not error and not flags:
        return ""
    parts = [f"0x{error:X}"]
    if error & ESTOP_REQUESTED:
        parts.append("estop_requested")
    if error & CAN_WATCHDOG:
        parts.append("can_watchdog")
    if flags:
        parts.append(f"flags={flags}")
    return " ".join(parts)


def decode(can_id: int, data: bytes) -> tuple[int, int, dict]:
    """(node, command, fields) of one GIM6010 frame."""
    node, command = can_id >> 5 & 0x3F, can_id & 0x1F
    fields: dict = {}
    if command == HEARTBEAT and len(data) >= 8:
        error = struct.unpack_from("<I", data, 0)[0]
        fields = {"axis_error": error, "axis_state": data[4], "fault_flags": data[5] & 0x0F,
                  "temperature": struct.unpack_from("<b", data, 6)[0]}
    elif command == ENCODER and len(data) >= 8:
        position, velocity = struct.unpack_from("<ff", data, 0)
        fields = {"position_rev": position, "velocity_rev_s": velocity}
    return node, command, fields


def joint_mapping(xacro_path: str | Path) -> dict[int, dict]:
    """node id -> {name, sign, ratio, offset_rev} from sandro.ros2_control.xacro."""
    text = Path(xacro_path).read_text()
    ratio = re.search(r'<param name="ratio">([-\d.]+)</param>', text)
    mapping = {}
    for match in re.finditer(r"<xacro:controlled_joint\s+([^>]*?)/>", text, re.S):
        attrs = dict(re.findall(r'(\w+)="([^"]*)"', match[1]))
        if "node" not in attrs:
            continue
        mapping[int(attrs["node"])] = {
            "name": attrs["name"], "sign": float(attrs["sign"]),
            "ratio": float(ratio[1]) if ratio else 8.0,
            "offset_rev": float(attrs.get("offset_rev", 0.0))}
    return mapping


class CanMonitor:
    def __init__(self, interface: str, mapping: dict[int, dict]) -> None:
        self.interface = interface
        self.mapping = mapping
        self._lock = threading.Lock()
        self._nodes: dict[int, dict] = {}
        self._counts: dict[tuple[int, int, bool], int] = {}
        self._rates: dict[tuple[int, int, bool], float] = {}
        self._total = self._total_rate = 0.0
        self._error: str | None = None
        self._open = False
        threading.Thread(target=self._run, name="can-monitor", daemon=True).start()
        threading.Thread(target=self._rate_loop, name="can-rates", daemon=True).start()

    def _run(self) -> None:
        while True:
            try:
                sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
                mask = 0x1F | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG
                sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                                struct.pack("=IIII", HEARTBEAT, mask, ENCODER, mask))
                sock.bind((self.interface,))
                sock.settimeout(1.0)
            except (OSError, AttributeError) as error:
                self._error, self._open = str(error), False
                time.sleep(3.0)
                continue
            self._error, self._open = None, True
            try:
                while True:
                    try:
                        raw, _, flags, _ = sock.recvmsg(_FRAME.size)
                    except socket.timeout:
                        continue
                    if len(raw) < _FRAME.size:
                        continue
                    can_id, length, data = _FRAME.unpack(raw)
                    if can_id & (socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_ERR_FLAG):
                        continue
                    self._frame(can_id, data[:length], bool(flags & socket.MSG_DONTROUTE))
            except OSError as error:
                self._error, self._open = str(error), False
                sock.close()
                time.sleep(1.0)

    def _frame(self, can_id: int, data: bytes, from_host: bool) -> None:
        node, command, fields = decode(can_id, data)
        now = time.monotonic()
        with self._lock:
            key = (node, command, from_host)
            self._counts[key] = self._counts.get(key, 0) + 1
            self._total += 1
            if from_host:
                return
            state = self._nodes.setdefault(node, {"node": node})
            state["last"] = now
            if command == HEARTBEAT and fields:
                state.update(fields, heartbeat=now)
            elif command == ENCODER and fields:
                state.update(fields, encoder=now)

    def _rate_loop(self) -> None:
        previous, previous_total, last = {}, 0.0, time.monotonic()
        while True:
            time.sleep(1.0)
            now = time.monotonic()
            with self._lock:
                counts, total = dict(self._counts), self._total
            dt = now - last
            self._rates = {k: (v - previous.get(k, 0)) / dt for k, v in counts.items()}
            self._total_rate = (total - previous_total) / dt
            previous, previous_total, last = counts, total, now

    def snapshot(self) -> dict:
        now = time.monotonic()
        nodes = []
        with self._lock:
            known = sorted(set(self._nodes) | set(self.mapping))
            for node in known:
                state = dict(self._nodes.get(node, {"node": node}))
                joint = self.mapping.get(node)
                rates = {COMMANDS.get(c, hex(c)): round(r, 1)
                         for (n, c, host), r in self._rates.items() if n == node and not host and r > 0}
                host = {COMMANDS.get(c, hex(c)): round(r, 1)
                        for (n, c, h), r in self._rates.items() if n == node and h and r > 0}
                row = {
                    "node": node, "joint": joint["name"] if joint else None,
                    "seen": "last" in state,
                    "age_s": now - state["last"] if "last" in state else None,
                    "heartbeat_age_s": now - state["heartbeat"] if "heartbeat" in state else None,
                    "encoder_age_s": now - state["encoder"] if "encoder" in state else None,
                    "axis_state": AXIS_STATES.get(state.get("axis_state"), state.get("axis_state")),
                    "axis_error": state.get("axis_error"),
                    "fault_flags": state.get("fault_flags"),
                    "error_text": axis_error_text(state.get("axis_error", 0), state.get("fault_flags", 0)),
                    "temperature": state.get("temperature"),
                    "rates": rates, "host_rates": host,
                }
                if joint and "position_rev" in state:
                    row["position"] = (joint["sign"] * (state["position_rev"] - joint["offset_rev"])
                                       / joint["ratio"] * 2.0 * math.pi)
                    row["velocity"] = (joint["sign"] * state["velocity_rev_s"]
                                       / joint["ratio"] * 2.0 * math.pi)
                    row["position_rev"] = state["position_rev"]
                nodes.append(row)
        return {"interface": self.interface, "open": self._open, "error": self._error,
                "frames_hz": round(self._total_rate, 1), "nodes": nodes}
