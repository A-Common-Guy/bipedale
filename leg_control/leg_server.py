#!/usr/bin/env python3
"""Five-bar leg controller: joint impedance control over CAN + web GUI.

ALL control runs here in Python — the browser GUI is only a remote:
it sends targets (foot x,y or joint angles) and renders the state that
this program broadcasts. The drives run in TORQUE mode; the position
loop is closed in software:

    tau_i = Kp * (th_des_i - th_i) - Kd * w_i        (leg frame, output Nm)

with torque clamping, target slew-rate limiting, joint soft limits,
overspeed kill, stale-feedback / heartbeat / drive-error watchdogs, and
the drives' own break_timeout dead-man as the last line of defence.

Frames & conventions
--------------------
Leg frame  = math/ik.py convention: +x right, +y down, motor 1 at (0,0),
motor 2 at (a,0), th = 0 with the upper arm straight down; th1 positive
CLOCKWISE, th2 positive COUNTER-clockwise (both arms swing inward).
Hardware   = both GIM6010 report positive CLOCKWISE with the same zero,
so the hw->leg map is signs (+1, -1): th_leg = sign * th_hw.
Wire units = rotor revolutions / rotor Nm (CANSimple classic frames);
this program converts with `ratio` (8:1 on the GIM6010-8) and the
per-motor rotor-rev offsets stored in leg_config.json ("Zero here").

Run
---
    V=~/Personal/gim6010/control/venv/bin/python   # has python-can + aiohttp
    $V leg_server.py --sim                # software motors, no hardware
    $V leg_server.py --channel can0 --nodes 1 2    # the real leg
then open http://localhost:8321 — the five-bar GUI connects by itself.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import sys
import threading
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import List, Optional, Tuple

log = logging.getLogger("leg")

HERE = Path(__file__).resolve().parent
DEFAULT_CAN_DIR = Path.home() / "Personal/gim6010/can_control"
DEFAULT_GUI = HERE.parent / "math" / "fivebar_viz.html"
DEFAULT_CONFIG = HERE / "leg_config.json"
TAU_HARD_MAX = 6.0          # Nm output — GUI/gains can never exceed this
TWO_PI = 2.0 * math.pi


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------

@dataclass
class LegConfig:
    # geometry, cm (matches math/ik.py: a=9, l1=8, l2=13)
    a: float = 9.0
    l1: float = 8.0
    l2: float = 13.0
    # motors
    nodes: Tuple[int, int] = (1, 2)
    signs: Tuple[float, float] = (1.0, -1.0)   # hw(CW+) -> leg frame
    ratio: float = 8.0                         # rotor:output
    offsets_rev: Tuple[float, float] = (0.0, 0.0)  # rotor rev at th=0
    # control mode: "mit" = impedance runs ON the drive (0x08 setpoints),
    # "torque" = software impedance loop with Set_Input_Torque
    mode: str = "mit"
    # impedance / safety (leg frame, output shaft)
    kp: float = 8.0            # Nm/rad
    kd: float = 0.5            # Nm*s/rad
    tau_max: float = 2.0       # Nm command clamp
    target_speed: float = 3.0  # rad/s max slew of the position target
    vel_kill: float = 20.0     # rad/s: fault above this measured speed
    th_min: float = math.radians(-120)
    th_max: float = math.radians(120)
    # timing
    loop_hz: float = 200.0
    encoder_ms: int = 5        # drive pos/vel broadcast period
    deadman_ms: int = 200      # drive-side break_timeout while enabled

    @staticmethod
    def load(path: Path) -> "LegConfig":
        cfg = LegConfig()
        if path.exists():
            data = json.loads(path.read_text())
            for k, v in data.items():
                if hasattr(cfg, k):
                    setattr(cfg, k, tuple(v) if isinstance(v, list) else v)
        return cfg

    def save(self, path: Path) -> None:
        path.write_text(json.dumps(asdict(self), indent=2) + "\n")


# --------------------------------------------------------------------------
# Kinematics (ports of math/ik.py + the GUI's IK, leg frame, radians/cm)
# --------------------------------------------------------------------------

def fk_foot(cfg: LegConfig, th1: float, th2: float,
            sigma: float = 1.0) -> Optional[Tuple[float, float]]:
    """Foot (x, y) for motor angles, elbows-out assembly branch sigma."""
    P = cfg.l1 * (math.cos(th2) - math.cos(th1))
    Q = cfg.a + cfg.l1 * (math.sin(th1) + math.sin(th2))
    r = math.hypot(P, Q)
    if r > 2 * cfg.l2:
        return None
    d = math.atan2(P, Q)
    s = sigma * math.acos(min(1.0, r / (2 * cfg.l2)))
    b1 = s + d
    return (-cfg.l1 * math.sin(th1) + cfg.l2 * math.cos(b1),
            cfg.l1 * math.cos(th1) + cfg.l2 * math.sin(b1))


def ik_joints(cfg: LegConfig, x: float, y: float
              ) -> Optional[Tuple[float, float]]:
    """Motor angles for a foot target — per-chain two-link circle
    intersection, elbows-out branch. None if unreachable."""
    r1 = math.hypot(x, y)
    r2 = math.hypot(x - cfg.a, y)
    lo, hi = abs(cfg.l1 - cfg.l2), cfg.l1 + cfg.l2
    if not (max(lo, 1e-9) <= r1 <= hi and max(lo, 1e-9) <= r2 <= hi):
        return None
    cl = lambda v: max(-1.0, min(1.0, v))
    a1 = math.acos(cl((cfg.l1**2 + r1**2 - cfg.l2**2) / (2 * cfg.l1 * r1)))
    a2 = math.acos(cl((cfg.l1**2 + r2**2 - cfg.l2**2) / (2 * cfg.l1 * r2)))
    d1 = math.atan2(y, x) + a1
    d2 = math.atan2(y, x - cfg.a) - a2
    wr = lambda v: math.atan2(math.sin(v), math.cos(v))
    return wr(d1 - math.pi / 2), wr(math.pi / 2 - d2)


# --------------------------------------------------------------------------
# Impedance controller (the only thing that ever commands torque)
# --------------------------------------------------------------------------

class LegController(threading.Thread):
    """200 Hz software position loop on top of the drives' torque mode."""

    def __init__(self, motors: list, cfg: LegConfig, cfg_path: Path):
        super().__init__(daemon=True, name="leg-control")
        self.motors = motors                    # [gim6010_can.Motor x2]
        self.cfg = cfg
        self.cfg_path = cfg_path
        self._lock = threading.RLock()
        self._halt = threading.Event()
        self.mode = "idle"                      # idle | active | fault
        self.fault_reason = ""
        self.goal: List[float] = [0.0, 0.0]     # commanded target (leg rad)
        self.thd: List[float] = [0.0, 0.0]      # slew-limited setpoint
        self.tau: List[float] = [0.0, 0.0]      # last torque command (leg Nm)
        self.loop_hz_meas = 0.0

    # -- hw <-> leg frame ---------------------------------------------------

    def _th(self, i: int) -> Optional[float]:
        pos = self.motors[i].state.pos          # rotor rev, raw
        if pos is None:
            return None
        return self.cfg.signs[i] * (pos - self.cfg.offsets_rev[i]) \
            / self.cfg.ratio * TWO_PI

    def _w(self, i: int) -> Optional[float]:
        vel = self.motors[i].state.vel
        if vel is None:
            return None
        return self.cfg.signs[i] * vel / self.cfg.ratio * TWO_PI

    def _send_tau(self, i: int, tau_leg: float) -> None:
        """Leg-frame output Nm -> rotor Nm on the wire."""
        self.motors[i].set_input_torque(
            self.cfg.signs[i] * tau_leg / self.cfg.ratio)

    # -- commands (called from the web thread pool) ---------------------------

    def enable(self) -> str:
        with self._lock:
            if self.mode == "active":
                return "already active"
            # preflight: refuse to touch a drive we have never heard from —
            # a dead bus otherwise turns into SDO timeouts + a full TX queue
            silent = [m.node_id for m in self.motors
                      if min(m.state.heartbeat_age(),
                             m.state.encoder_age()) > 1.0]
            if silent:
                raise RuntimeError(
                    f"nodes {silent} are not talking (no heartbeat/encoder "
                    "frames). Check drive power, bitrate + sample point "
                    "(./canup.sh), termination; verify with gim_cli.py scan")
            for i, m in enumerate(self.motors):
                if m.state.any_error:
                    raise RuntimeError(
                        f"node {m.node_id} has errors — clear them first")
            for m in self.motors:
                m.set_feedback_rates(encoder=self.cfg.encoder_ms)
                m.set_deadman(self.cfg.deadman_ms)
            if self.cfg.mode == "mit":
                for m in self.motors:
                    # the drive quantizes MIT fields over its own mit_max_*
                    # config — read the real values so kp/kd/pos can never
                    # silently decode to something else on the drive side
                    self._load_mit_limits(m)
                    # POSITION control + MIT input (user manual 3.3.4 —
                    # control_mode 1 makes the drive ignore the MIT gains);
                    # reconcile with a zero-gain setpoint at the current
                    # position so entering closed loop produces no torque
                    m.set_controller_mode(3, 9)
                    pos_rev, _ = m.get_encoder_estimates(timeout=1.0)
                    pos_out = pos_rev / self.cfg.ratio * TWO_PI
                    if abs(pos_out) > 0.8 * m.mit_limits.max_pos:
                        raise RuntimeError(
                            f"node {m.node_id}: output position "
                            f"{pos_out:+.1f} rad is near the MIT range "
                            f"±{m.mit_limits.max_pos:g} rad — the 16-bit "
                            "position field would saturate. Re-zero the "
                            "drive near 0 turns first")
                    m.mit_control(pos=pos_out, vel=0.0, kp=0.0, kd=0.0,
                                  torque=0.0, wait_feedback=False)
                    m.set_axis_state(8)
            else:
                for m in self.motors:
                    m.enable_torque_mode()      # reconciles, holds at zero tau
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.0:
                if all(m.state.axis_state == 8 for m in self.motors):
                    break
                time.sleep(0.02)
            else:
                for m in self.motors:
                    m.idle()
                raise RuntimeError("drives did not enter closed loop")
            th = [self._th(0), self._th(1)]
            if None in th:
                for m in self.motors:
                    m.idle()
                raise RuntimeError("no position feedback yet")
            self.goal = list(th)                # hold where we are: no jump
            self.thd = list(th)
            self.tau = [0.0, 0.0]
            self.fault_reason = ""
            self.mode = "active"
            log.info("enabled at th = %.1f / %.1f deg",
                     math.degrees(th[0]), math.degrees(th[1]))
            return "enabled"

    def _load_mit_limits(self, m) -> None:
        """Sync the encoder-side MitLimits with the drive's mit_max_* config
        so both ends quantize the 0x08 fields over the same ranges."""
        for attr, ep in (("max_pos", "mit_max_pos"),
                         ("max_vel", "mit_max_vel"),
                         ("max_kp", "mit_max_kp"),
                         ("max_kd", "mit_max_kd"),
                         ("max_torque", "mit_max_torque")):
            val = float(m.sdo_read(ep))
            if val <= 0:
                raise RuntimeError(
                    f"node {m.node_id}: drive reports {ep} = {val} — "
                    "fix the drive's MIT range config first")
            setattr(m.mit_limits, attr, val)
        lim = m.mit_limits
        log.info("node %d MIT ranges: pos ±%g rad, vel ±%g, kp %g, kd %g, "
                 "torque ±%g", m.node_id, lim.max_pos, lim.max_vel,
                 lim.max_kp, lim.max_kd, lim.max_torque)

    def disable(self) -> str:
        with self._lock:
            self.mode = "idle"
            self._soft_stop()
            return "disabled"

    def estop(self) -> str:
        with self._lock:
            failed = []
            for m in self.motors:
                try:
                    m.estop()
                except Exception as e:
                    failed.append(f"node {m.node_id}: {e}")
                    log.error("estop TX failed, node %d: %s", m.node_id, e)
            self.mode = "fault"
            self.fault_reason = "e-stop"
            if failed:
                return ("e-stop COMMAND FAILED TO SEND (" + "; ".join(failed)
                        + ") — cut power if the leg is moving")
            return "e-stopped"

    def clear_errors(self) -> str:
        with self._lock:
            if self.mode == "active":
                raise RuntimeError("disable first")
            for m in self.motors:
                m.clear_errors()
            time.sleep(0.25)                    # wait for a clean heartbeat
            still = [m.node_id for m in self.motors if m.state.any_error]
            if still:
                raise RuntimeError(f"nodes {still} still report errors")
            self.mode = "idle"
            self.fault_reason = ""
            return "errors cleared"

    def zero_here(self) -> str:
        """Store the current pose as th1 = th2 = 0 (both arms straight
        down). Only while idle; persists to leg_config.json."""
        with self._lock:
            if self.mode == "active":
                raise RuntimeError("disable first")
            offs = []
            for m in self.motors:
                pos, _ = m.get_encoder_estimates(timeout=1.0)
                offs.append(pos)
            self.cfg = replace(self.cfg, offsets_rev=tuple(offs))
            self.cfg.save(self.cfg_path)
            return f"zeroed (offsets {offs[0]:+.4f} / {offs[1]:+.4f} rev)"

    def set_target_joints(self, th1: float, th2: float) -> None:
        with self._lock:
            self.goal = [self._clamp_th(th1), self._clamp_th(th2)]

    def set_target_foot(self, x: float, y: float) -> str:
        sol = ik_joints(self.cfg, x, y)
        if sol is None:
            raise RuntimeError(f"({x:.1f}, {y:.1f}) cm is out of the workspace")
        self.set_target_joints(*sol)
        return "ok"

    def set_gains(self, kp=None, kd=None, tau_max=None) -> str:
        with self._lock:
            c = self.cfg
            if kp is not None:
                c = replace(c, kp=min(max(float(kp), 0.0), 100.0))
            if kd is not None:
                c = replace(c, kd=min(max(float(kd), 0.0), 10.0))
            if tau_max is not None:
                c = replace(c, tau_max=min(max(float(tau_max), 0.0),
                                           TAU_HARD_MAX))
            self.cfg = c
            return (f"kp={c.kp:g} kd={c.kd:g} tau_max={c.tau_max:g}")

    def shutdown(self) -> None:
        self._halt.set()
        self.join(timeout=2.0)
        self._soft_stop()

    def _clamp_th(self, th: float) -> float:
        return min(max(th, self.cfg.th_min), self.cfg.th_max)

    def _soft_stop(self) -> None:
        """Best effort, one guarded step at a time — a failing send must not
        prevent the later (more important) idle command from being tried."""
        for m in self.motors:
            steps = [("torque0", lambda m=m: m.set_input_torque(0.0)),
                     ("idle", lambda m=m: m.idle())]
            if self.cfg.mode == "mit":           # zero-gain, zero-torque frame
                steps.insert(0, ("mit0", lambda m=m: m.mit_control(
                    0.0, 0.0, 0.0, 0.0, 0.0, wait_feedback=False)))
            if m.state.heartbeat_age() < 1.0:    # SDO only if drive answers
                steps.append(("deadman0", lambda m=m: m.set_deadman(0)))
            failed = []
            for name, fn in steps:
                try:
                    fn()
                except Exception as e:
                    failed.append(f"{name}: {str(e).splitlines()[0]}")
            if failed:
                log.warning("soft stop node %d: %s",
                            m.node_id, " | ".join(failed))
        self.tau = [0.0, 0.0]

    def _fault(self, reason: str) -> None:
        log.error("FAULT: %s", reason)
        self.mode = "fault"
        self.fault_reason = reason
        self._soft_stop()

    # -- the loop -------------------------------------------------------------

    def run(self) -> None:
        dt = 1.0 / self.cfg.loop_hz
        next_t = time.monotonic()
        cycles, t_rate = 0, time.monotonic()
        while not self._halt.is_set():
            next_t += dt
            with self._lock:
                if self.mode == "active":
                    self._step(dt)
                cycles += 1
                now = time.monotonic()
                if now - t_rate >= 1.0:
                    self.loop_hz_meas = cycles / (now - t_rate)
                    cycles, t_rate = 0, now
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:                               # fell behind: resync
                next_t = time.monotonic()

    def _step(self, dt: float) -> None:
        c = self.cfg
        for m in self.motors:
            s = m.state
            if s.any_error:
                names = []
                try:
                    from gim6010_can import AXIS_ERRORS, decode_flags
                    names = decode_flags(AXIS_ERRORS, s.axis_error or 0)
                except Exception:
                    pass
                self._fault(f"node {m.node_id} error {names or hex(s.axis_error or 0)}")
                return
            if s.encoder_age() > 0.1:
                self._fault(f"node {m.node_id}: stale position feedback")
                return
            if s.heartbeat_age() > 0.5:
                self._fault(f"node {m.node_id}: heartbeat lost")
                return
        th = [self._th(0), self._th(1)]
        w = [self._w(0), self._w(1)]
        if None in th or None in w:
            self._fault("position feedback vanished")
            return
        for i in range(2):
            if abs(w[i]) > c.vel_kill:
                self._fault(f"joint {i + 1} overspeed ({w[i]:.1f} rad/s)")
                return
        step = c.target_speed * dt
        try:
            self._command_joints(th, w, step)
        except Exception as e:
            self._fault(f"CAN TX failed: {type(e).__name__}: "
                        f"{str(e).splitlines()[0]}")

    def _command_joints(self, th, w, step) -> None:
        c = self.cfg
        for i in range(2):
            err_goal = self.goal[i] - self.thd[i]
            self.thd[i] += min(max(err_goal, -step), step)
            if c.mode == "mit":
                # the drive computes tau = kp(pos_cmd - pos) + kd(0 - vel);
                # enforce tau_max by keeping the commanded position within
                # tau_max/kp of the measured one (spring force cap)
                cmd = self.thd[i]
                if c.kp > 1e-6:
                    reach = c.tau_max / c.kp
                    cmd = min(max(cmd, th[i] - reach), th[i] + reach)
                lim = self.motors[i].mit_limits
                self.tau[i] = c.kp * (cmd - th[i]) - c.kd * w[i]  # estimate
                # CAN MIT frames are OUTPUT-shaft rad / Nm (manual, 0x08) —
                # never apply the rotor `ratio` here; only the stored
                # rotor-rev zero offset needs converting
                self.motors[i].mit_control(
                    pos=c.signs[i] * cmd + c.offsets_rev[i] / c.ratio * TWO_PI,
                    vel=0.0,
                    kp=min(c.kp, lim.max_kp),
                    kd=min(c.kd, lim.max_kd),
                    torque=0.0, wait_feedback=False)
            else:
                tau = c.kp * (self.thd[i] - th[i]) - c.kd * w[i]
                tau = min(max(tau, -c.tau_max), c.tau_max)
                self.tau[i] = tau
                self._send_tau(i, tau)

    # -- telemetry -------------------------------------------------------------

    def snapshot(self) -> dict:
        with self._lock:
            th = [self._th(0), self._th(1)]
            w = [self._w(0), self._w(1)]
            per = []
            for m in self.motors:
                s = m.state
                per.append({
                    "node": m.node_id,
                    "axis_state": s.axis_state_name(),
                    "temp": s.temperature,
                    "error": bool(s.any_error),
                    "seen": min(s.heartbeat_age(), s.encoder_age()) < 1.0,
                    "enc_age_ms": None if s.encoder_age() == float("inf")
                    else round(s.encoder_age() * 1e3, 1),
                })
            rnd = lambda v: None if v is None else round(v, 4)
            return {
                "type": "state",
                "mode": self.mode,
                "ctrl": self.cfg.mode,
                "reason": self.fault_reason,
                "th": [rnd(v) for v in th],
                "w": [rnd(v) for v in w],
                "tau": [round(v, 3) for v in self.tau],
                "goal": [round(v, 4) for v in self.goal],
                "loop_hz": round(self.loop_hz_meas, 1),
                "motors": per,
            }

    def config_msg(self) -> dict:
        c = self.cfg
        return {"type": "config", "a": c.a, "l1": c.l1, "l2": c.l2,
                "kp": c.kp, "kd": c.kd, "tau_max": c.tau_max,
                "nodes": list(c.nodes), "ctrl": c.mode}


# --------------------------------------------------------------------------
# Web server (aiohttp): serves the GUI + a JSON WebSocket
# --------------------------------------------------------------------------

def make_app(ctrl: LegController, gui_path: Path):
    from aiohttp import WSMsgType, web

    app = web.Application()
    clients: set = set()

    async def index(_req):
        return web.FileResponse(gui_path, headers={
            "Cache-Control": "no-store",
            "Content-Type": "text/html; charset=utf-8",
        })

    def dispatch(data: dict) -> dict:
        t = data.get("type")
        try:
            if t == "enable":
                return {"type": "ack", "msg": ctrl.enable()}
            if t == "disable":
                return {"type": "ack", "msg": ctrl.disable()}
            if t == "estop":
                return {"type": "ack", "msg": ctrl.estop()}
            if t == "clear_errors":
                return {"type": "ack", "msg": ctrl.clear_errors()}
            if t == "zero_here":
                return {"type": "ack", "msg": ctrl.zero_here()}
            if t == "target_foot":
                ctrl.set_target_foot(float(data["x"]), float(data["y"]))
                return {"type": "ack", "msg": ""}
            if t == "target_joints":
                ctrl.set_target_joints(float(data["th1"]), float(data["th2"]))
                return {"type": "ack", "msg": ""}
            if t == "gains":
                return {"type": "ack", "msg": ctrl.set_gains(
                    data.get("kp"), data.get("kd"), data.get("tau_max"))}
            return {"type": "err", "msg": f"unknown command '{t}'"}
        except Exception as e:  # controller refusals -> readable GUI errors
            return {"type": "err", "msg": str(e)}

    async def wshandler(req):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(req)
        clients.add(ws)
        loop = asyncio.get_running_loop()
        await ws.send_json(ctrl.config_msg())
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                reply = await loop.run_in_executor(None, dispatch, data)
                if reply.get("msg") or reply["type"] == "err":
                    await ws.send_json(reply)
        finally:
            clients.discard(ws)
        return ws

    async def broadcaster(app):
        try:
            while True:
                await asyncio.sleep(1 / 30)
                if not clients:
                    continue
                snap = ctrl.snapshot()
                for ws in list(clients):
                    try:
                        await ws.send_json(snap)
                    except Exception:
                        clients.discard(ws)
        except asyncio.CancelledError:
            pass

    async def on_start(app):
        app["bcast"] = asyncio.create_task(broadcaster(app))

    async def on_stop(app):
        app["bcast"].cancel()
        for ws in list(clients):
            await ws.close()

    app.router.add_get("/", index)
    app.router.add_get("/ws", wshandler)
    app.on_startup.append(on_start)
    app.on_cleanup.append(on_stop)
    return app


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sim", action="store_true",
                    help="software motors (sim_motor.py) instead of hardware")
    ap.add_argument("--channel", "-c", default="can0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--nodes", type=int, nargs=2, metavar=("M1", "M2"),
                    help="CAN node ids for motor 1 (left, at x=0) and "
                         "motor 2 (right, at x=a); default from config")
    ap.add_argument("--mode", choices=("mit", "torque"),
                    help="mit: impedance on the drive (0x08 setpoints); "
                         "torque: software loop with Set_Input_Torque "
                         "(default from config, normally mit)")
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address (0.0.0.0 to reach from other devices)")
    ap.add_argument("--port", type=int, default=8321)
    ap.add_argument("--can-dir", type=Path, default=DEFAULT_CAN_DIR,
                    help="folder containing gim6010_can.py")
    ap.add_argument("--gui", type=Path, default=DEFAULT_GUI)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s")

    sys.path.insert(0, str(args.can_dir))
    import can
    from gim6010_can import GIM6010Bus

    cfg = LegConfig.load(args.config)
    if args.nodes:
        cfg = replace(cfg, nodes=tuple(args.nodes))
    if args.mode:
        cfg = replace(cfg, mode=args.mode)
    if not args.config.exists():
        cfg.save(args.config)
        log.info("wrote default config to %s", args.config)

    sims = []
    if args.sim:
        from sim_motor import SimMotor
        for node in cfg.nodes:
            sims.append(SimMotor(
                can.Bus(interface="virtual", channel="legsim",
                        receive_own_messages=False), node_id=node))
        host_bus = can.Bus(interface="virtual", channel="legsim",
                           receive_own_messages=False)
        log.info("SIM mode: emulated motors on nodes %s", list(cfg.nodes))
    else:
        host_bus = can.Bus(channel=args.channel, interface=args.interface)

    endpoints = args.can_dir.parent / "manual" / "assets_protocol_manual" \
        / "endpoints_v3.json"
    bus = GIM6010Bus(bus=host_bus,
                     endpoints=endpoints if endpoints.exists() else None)
    motors = [bus.motor(n) for n in cfg.nodes]

    ctrl = LegController(motors, cfg, args.config)
    ctrl.start()

    from aiohttp import web
    app = make_app(ctrl, args.gui)

    async def on_shutdown(app):
        ctrl.shutdown()

    app.on_shutdown.append(on_shutdown)
    log.info("GUI at http://%s:%d  (motors %s, %s, %s mode)", args.host,
             args.port, list(cfg.nodes),
             "sim" if args.sim else args.channel, cfg.mode)
    try:
        web.run_app(app, host=args.host, port=args.port,
                    print=None, handle_signals=True)
    finally:
        for s in sims:
            s.stop()
        bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
