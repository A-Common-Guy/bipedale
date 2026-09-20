"""Software GIM6010 emulator speaking CANSimple, for testing without hardware.

Runs on any python-can bus; with interface='virtual' it lets gim6010_can and
gim_cli be exercised fully in-process (see selftest.py). The model is crude —
a torque-driven inertia — but every protocol frame is encoded per the manual.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from typing import Dict, Optional, Tuple

import can

from gim6010_can import Cmd, MitLimits, _from_uint, _to_uint


class SimMotor:
    def __init__(self, bus: can.BusABC, node_id: int = 1,
                 heartbeat_ms: int = 100, encoder_ms: int = 10,
                 vbus: float = 24.0, inertia: float = 0.005):
        self.bus = bus
        self.node_id = node_id
        self.heartbeat_ms = heartbeat_ms
        self.encoder_ms = encoder_ms
        self.inertia = inertia
        self.mit_limits = MitLimits()

        self.axis_state = 1
        self.control_mode = 3
        self.input_mode = 3
        self.input_torque = 0.0
        self.pos = 0.0            # rotor rev
        self.vel = 0.0            # rotor rev/s
        self.axis_error = 0
        self.errors = {"motor": 0, "encoder": 0, "controller": 0, "system": 0}
        self.motor_temp = 32.5
        self.fet_temp = 30.0
        self.vbus = vbus
        self.ibus = 0.1
        self.life = 0
        # SDO parameter store, endpoint_id -> (fmt, value)
        self.params: Dict[int, list] = {
            2: ["<f", vbus],        # vbus_voltage
            6: ["<B", 3], 7: ["<B", 11],              # hw_version 3.11
            9: ["<B", 0], 10: ["<B", 6], 11: ["<B", 5],  # fw_version 0.6.5
            69: ["<I", 500000],     # can.config.baud_rate
            70: ["<B", 1],          # can.config.protocol (CANSimple)
            72: ["<B", 1],          # can.config.enable_r120
            73: ["<H", 0],          # can.config.break_timeout
            180: ["<I", node_id],   # axis0.config.can.node_id
            182: ["<I", heartbeat_ms],  # ...can.heartbeat_rate_ms
            183: ["<I", encoder_ms],    # ...can.encoder_rate_ms
            242: ["<f", 8.0],       # axis0.motor.config.gear_ratio
            335: ["<f", 12.5],      # axis0.controller.config.mit_max_pos
            336: ["<f", 65.0],      # ...mit_max_vel
            337: ["<f", 50.0],      # ...mit_max_torque
            338: ["<f", 500.0],     # ...mit_max_kp
            339: ["<f", 5.0],       # ...mit_max_kd
        }
        self.saved = False
        self._reboot_until = 0.0  # while in the future: deaf and silent
        self.last_raw_torque: Optional[float] = None
        # last MIT setpoint (output frame): (pos rad, vel rad/s, kp, kd, tff)
        self.mit_cmd: Optional[Tuple[float, float, float, float, float]] = None

        self._stop = threading.Event()
        self._t = threading.Thread(target=self._loop, daemon=True,
                                   name=f"sim-motor-{node_id}")
        self._t.start()

    def stop(self):
        self._stop.set()
        self._t.join(timeout=1.0)

    # ------------------------------------------------------------------ tx

    def _tx(self, cmd: int, data: bytes) -> None:
        self.bus.send(can.Message(arbitration_id=(self.node_id << 5) | cmd,
                                  is_extended_id=False, data=data))

    def _heartbeat(self) -> None:
        flags = ((1 if self.errors["motor"] else 0)
                 | (2 if self.errors["encoder"] else 0)
                 | (4 if self.errors["controller"] else 0)
                 | (8 if self.errors["system"] else 0))
        temp = int(max(self.motor_temp, self.fet_temp))
        self._tx(Cmd.HEARTBEAT, struct.pack(
            "<IBBbB", self.axis_error, self.axis_state, flags, temp, self.life))
        self.life = (self.life + 1) % 256

    def _encoder(self) -> None:
        self._tx(Cmd.GET_ENCODER_ESTIMATES,
                 struct.pack("<ff", self.pos, self.vel))

    # MIT mode runs the impedance law on the drive itself, in the OUTPUT
    # frame (rad): tau = kp*(pos_cmd - pos) + kd*(vel_cmd - vel) + tff
    def _gear(self) -> float:
        return float(self.params.get(242, ["<f", 8.0])[1]) or 8.0

    def _out_state(self) -> Tuple[float, float]:
        g = self._gear()
        return self.pos / g * 2 * math.pi, self.vel / g * 2 * math.pi

    def _mit_torque(self) -> float:
        """Output-shaft Nm from the stored MIT setpoint."""
        if self.mit_cmd is None:
            return 0.0
        p, v, kp, kd, tff = self.mit_cmd
        po, vo = self._out_state()
        return kp * (p - po) + kd * (v - vo) + tff

    # ------------------------------------------------------------------ rx

    def _handle(self, msg: can.Message) -> None:
        if msg.is_extended_id:
            return
        if (msg.arbitration_id >> 5) & 0x3F != self.node_id:
            return
        cmd = msg.arbitration_id & 0x1F
        d = bytes(msg.data)

        if msg.is_remote_frame:
            replies = {
                Cmd.GET_ENCODER_ESTIMATES: struct.pack("<ff", self.pos, self.vel),
                Cmd.GET_ENCODER_COUNT: struct.pack(
                    "<ii", int(self.pos), int((self.pos % 1.0) * 16384)),
                Cmd.GET_IQ: struct.pack("<ff", self.input_torque / 0.07,
                                        self.input_torque / 0.07 * 0.98),
                Cmd.GET_TORQUES: struct.pack("<ff", self.input_torque,
                                             self.input_torque * 0.97),
                Cmd.GET_TEMPERATURE: struct.pack(
                    "<ff", self.motor_temp, self.fet_temp),
                Cmd.GET_BUS_VOLTAGE_CURRENT: struct.pack(
                    "<ff", self.vbus, self.ibus),
                Cmd.GET_POWERS: struct.pack(
                    "<ff", self.vbus * self.ibus,
                    self.input_torque * self.vel * 6.283),
            }
            if cmd in replies:
                self._tx(cmd, replies[cmd])
            return

        if cmd == Cmd.ESTOP:
            self.axis_error |= 0x4000  # ESTOP_REQUESTED
            self.axis_state = 1
            self.input_torque = 0.0
            self.mit_cmd = None
            self.vel = 0.0
        elif cmd == Cmd.SAVE_CONFIGURATION:
            self.saved = True
            self._reboot_until = time.monotonic() + 0.6  # flash write + boot
        elif cmd == Cmd.CLEAR_ERRORS:
            self.axis_error = 0
            self.errors = dict.fromkeys(self.errors, 0)
        elif cmd == Cmd.SET_AXIS_STATE:
            req = struct.unpack("<I", d[:4])[0]
            if req == 8 and self.axis_error:
                pass  # refuse closed loop while faulted, like the firmware
            elif req in (1, 8):
                self.axis_state = req
                if req == 1:
                    self.mit_cmd = None
                    self.input_torque = 0.0
        elif cmd == Cmd.SET_CONTROLLER_MODE:
            self.control_mode, self.input_mode = struct.unpack("<II", d)
        elif cmd == Cmd.SET_INPUT_TORQUE:
            self.last_raw_torque = struct.unpack("<f", d[:4])[0]
            if self.axis_state == 8 and self.control_mode == 1:
                self.input_torque = self.last_raw_torque
        elif cmd == Cmd.SET_INPUT_VEL:
            if self.axis_state == 8 and self.control_mode == 2:
                self.vel = struct.unpack("<ff", d)[0]
        elif cmd == Cmd.SET_INPUT_POS:
            if self.axis_state == 8 and self.control_mode == 3 \
                    and self.input_mode != 9:
                self.pos = struct.unpack("<f", d[:4])[0]
        elif cmd == Cmd.GET_ERROR:
            which = d[0]
            if which == 0:
                self._tx(Cmd.GET_ERROR,
                         struct.pack("<Q", self.errors["motor"]))
            elif which == 1:
                self._tx(Cmd.GET_ERROR,
                         struct.pack("<I", self.errors["encoder"]))
            elif which == 3:
                self._tx(Cmd.GET_ERROR,
                         struct.pack("<I", self.errors["controller"]))
            elif which == 4:
                self._tx(Cmd.GET_ERROR,
                         struct.pack("<I", self.errors["system"]))
        elif cmd == Cmd.RXSDO:
            opcode, eid, _ = struct.unpack("<BHB", d[:4])
            entry = self.params.setdefault(eid, ["<I", 0])
            fmt = entry[0]
            if opcode == 1:  # write
                size = struct.calcsize(fmt)
                entry[1] = struct.unpack(fmt, d[4:4 + size])[0]
                if eid == 182:
                    self.heartbeat_ms = entry[1]
                elif eid == 183:
                    self.encoder_ms = entry[1]
            else:            # read -> TxSdo
                payload = struct.pack(fmt, entry[1]).ljust(4, b"\0")
                self._tx(Cmd.TXSDO, struct.pack("<BHB", 0, eid, 0) + payload)
        elif cmd == Cmd.MIT_CONTROL:
            # firmware requires POSITION + MIT (user manual 3.3.4); with any
            # other control_mode the frame is ignored and no torque results
            if self.axis_state == 8 and self.control_mode == 3 \
                    and self.input_mode == 9:
                # command layout: pos16 | vel12 | kp12 | kd12 | torque12
                lim = self.mit_limits
                p_i = (d[0] << 8) | d[1]
                v_i = (d[2] << 4) | (d[3] >> 4)
                kp_i = ((d[3] & 0xF) << 8) | d[4]
                kd_i = (d[5] << 4) | (d[6] >> 4)
                t_i = ((d[6] & 0xF) << 8) | d[7]
                self.mit_cmd = (
                    _from_uint(p_i, -lim.max_pos, lim.max_pos, 16),
                    _from_uint(v_i, -lim.max_vel, lim.max_vel, 12),
                    _from_uint(kp_i, 0.0, lim.max_kp, 12),
                    _from_uint(kd_i, 0.0, lim.max_kd, 12),
                    _from_uint(t_i, -lim.max_torque, lim.max_torque, 12),
                )
                torque_out = self._mit_torque()
                self.input_torque = torque_out / self._gear()  # rotor Nm
                # echo feedback: node id + pos16 + vel12 + torque12,
                # output frame like the command
                po, vo = self._out_state()
                fp = _to_uint(po, -lim.max_pos, lim.max_pos, 16)
                fv = _to_uint(vo, -lim.max_vel, lim.max_vel, 12)
                ft = _to_uint(torque_out, -lim.max_torque, lim.max_torque, 12)
                self._tx(Cmd.MIT_CONTROL, bytes([
                    self.node_id, (fp >> 8) & 0xFF, fp & 0xFF,
                    (fv >> 4) & 0xFF, ((fv & 0xF) << 4) | ((ft >> 8) & 0xF),
                    ft & 0xFF]))

    # ------------------------------------------------------------------ loop

    def _loop(self) -> None:
        last_hb = last_enc = last_dyn = time.monotonic()
        while not self._stop.is_set():
            msg = self.bus.recv(timeout=0.001)
            now = time.monotonic()
            if now < self._reboot_until:  # "rebooting": deaf and silent
                last_hb = last_enc = last_dyn = now
                continue
            if msg is not None:
                try:
                    self._handle(msg)
                except Exception:
                    pass
            dt, last_dyn = now - last_dyn, now
            mit_active = (self.axis_state == 8 and self.control_mode == 3
                          and self.input_mode == 9 and self.mit_cmd is not None)
            if self.axis_state == 8 and (self.control_mode == 1 or mit_active):
                if mit_active:
                    # continuous on-drive impedance between MIT frames
                    self.input_torque = self._mit_torque() / self._gear()
                self.vel += (self.input_torque / self.inertia) * dt
                self.vel *= max(0.0, 1.0 - 0.5 * dt)  # viscous drag
            elif self.axis_state != 8:
                self.vel = 0.0
            self.pos += self.vel * dt
            if self.heartbeat_ms and now - last_hb >= self.heartbeat_ms / 1e3:
                self._heartbeat(); last_hb = now
            if self.encoder_ms and now - last_enc >= self.encoder_ms / 1e3:
                self._encoder(); last_enc = now
