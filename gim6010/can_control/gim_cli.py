#!/usr/bin/env python3
"""CLI for GIM6010 drives over CANSimple.

Examples (node 1, can0):
    gim_cli.py scan
    gim_cli.py status 1
    gim_cli.py errors 1            # decoded detailed errors; --clear to reset
    gim_cli.py enable 1 --mode torque
    gim_cli.py torque 1 0.05 --duration 3        # stream 0.05 Nm for 3 s
    gim_cli.py torque 1 0.05 --poll              # RTR-poll feedback each cycle
    gim_cli.py watch 1
    gim_cli.py estop all
    gim_cli.py idle 1
    gim_cli.py sdo read 1 vbus_voltage
    gim_cli.py sdo write 1 can.config.break_timeout 2000
    gim_cli.py limits 1 --vel 10 --current 8
    gim_cli.py save 1

Global flags: --channel/-c, --frame output (convert pos/vel/torque to the
output shaft, ratio --ratio, default 8), --endpoints for the SDO JSON.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

from gim6010_can import (
    AXIS_STATE, AXIS_ERRORS, CONTROL_MODE, INPUT_MODE, RATE_ENDPOINTS,
    GIM6010Bus, Timeout, decode_flags,
)

DEFAULT_ENDPOINTS = (Path(__file__).resolve().parent.parent
                     / "manual" / "assets_protocol_manual" / "endpoints_v3.json")

MODES = {  # name -> (control_mode, input_mode)
    "torque": (1, 1),
    "torque-ramp": (1, 6),
    "vel": (2, 1),
    "vel-ramp": (2, 2),
    "pos": (3, 1),
    "pos-filter": (3, 3),
    "pos-traj": (3, 5),
    "mit": (3, 9),
}


def fmt(v, spec=".3f"):
    return "-" if v is None else format(v, spec)


def make_bus(args) -> GIM6010Bus:
    ep = Path(args.endpoints) if args.endpoints else DEFAULT_ENDPOINTS
    return GIM6010Bus(channel=args.channel, interface=args.interface,
                      ratio=args.ratio, frame=args.frame,
                      endpoints=ep if ep.exists() else None)


# ----------------------------------------------------------------- commands


def cmd_scan(bus, args):
    print(f"listening {args.time:.1f}s for heartbeats on {args.channel} ...")
    nodes = bus.scan(args.time)
    if not nodes:
        print("no nodes found — check baud rate (factory default 500k, "
              "canup.sh sets 1M), wiring, termination, and that "
              "heartbeat_rate_ms > 0")
        return 1
    for n in nodes:
        s = bus.motor(n).state
        err = " ERROR" if s.any_error else ""
        print(f"  node {n:2d}  state={s.axis_state_name():<24}"
              f" temp={fmt(s.temperature, 'd')}°C{err}")
    return 0


def cmd_status(bus, args):
    m = bus.motor(args.node)
    time.sleep(0.3)  # catch a heartbeat
    s = m.state
    print(f"node {args.node} — axis state: {s.axis_state_name()}"
          f"   (heartbeat {'ok' if s.t_heartbeat else 'NOT SEEN'}"
          f", lost={s.lost_heartbeats})")
    if s.axis_error is not None:
        names = decode_flags(AXIS_ERRORS, s.axis_error) or ["none"]
        print(f"  axis_error : 0x{s.axis_error:08X} {' | '.join(names)}")
        print(f"  error bits : motor={s.motor_err} encoder={s.encoder_err} "
              f"controller={s.controller_err} system={s.system_err}")
    try:
        pos, vel = m.get_encoder_estimates()
        u = "out" if bus.frame == "output" else "rotor"
        print(f"  position   : {pos:9.4f} rev ({u})    velocity: {vel:9.4f} rev/s")
    except Timeout:
        print("  position   : (no reply)")
    for label, call, unit in (
            ("iq set/meas", m.get_iq, "A"),
            ("torque sp/act", m.get_torques, "Nm"),
            ("temp mot/fet", m.get_temperature, "°C"),
            ("bus V/I", m.get_bus_voltage_current, "V, A")):
        try:
            a, b = call()
            print(f"  {label:<12}: {a:8.3f} / {b:8.3f} {unit}")
        except Timeout:
            print(f"  {label:<12}: (no reply)")
    return 0


def cmd_errors(bus, args):
    m = bus.motor(args.node)
    if args.clear:
        m.clear_errors()
        print("clear_errors sent")
        time.sleep(0.1)
    any_set = False
    for which, (raw, names) in m.get_all_errors().items():
        mark = "  " if raw == 0 else "!!"
        any_set |= raw != 0
        print(f" {mark} {which:<10} 0x{raw:08X}  {' | '.join(names) or '-'}")
    if not any_set:
        print("no errors set")
    return 0


def cmd_enable(bus, args):
    m = bus.motor(args.node)
    cm, im = MODES[args.mode]
    m.set_controller_mode(cm, im)
    m._reconcile(timeout=1.0)
    m.set_axis_state(8)
    time.sleep(0.3)
    s = m.state
    print(f"mode={CONTROL_MODE[cm]}/{INPUT_MODE[im]}  "
          f"axis state: {s.axis_state_name()}")
    if s.axis_state != 8:
        print("did not reach CLOSED_LOOP_CONTROL — run 'errors' to see why")
        return 1
    return 0


def cmd_idle(bus, args):
    bus.motor(args.node).idle()
    print("idle requested")
    return 0


def cmd_estop(bus, args):
    if args.node == "all":
        time.sleep(0.5)  # discover heartbeats
        nodes = bus.estop_all()
        print(f"ESTOP sent to nodes: {nodes or 'none seen — none sent!'}")
    else:
        bus.motor(int(args.node)).estop()
        print(f"ESTOP sent to node {args.node}")
    print("(drive latches ESTOP_REQUESTED — 'errors --clear' + 'enable' to recover)")
    return 0


def cmd_torque(bus, args):
    m = bus.motor(args.node)
    unit = "Nm@output" if bus.frame == "output" else "Nm@rotor"
    if args.enable:
        m.enable_torque_mode(ramp=args.ramp)
        time.sleep(0.3)
        if m.state.axis_state != 8:
            print("could not enter closed loop — check errors"); return 1
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *a: stop.update(flag=True))
    period = 1.0 / args.rate
    t0 = time.monotonic()
    next_print = 0.0
    print(f"streaming {args.nm:+.4f} {unit} at {args.rate:.0f} Hz "
          f"(Ctrl-C to stop{'' if args.hold else ' -> zero torque + idle'})")
    try:
        while not stop["flag"]:
            now = time.monotonic() - t0
            if args.duration and now >= args.duration:
                break
            m.set_input_torque(args.nm)
            if args.poll:
                try:
                    pos, vel = m.get_encoder_estimates(timeout=period)
                    tsp, tq = m.get_torques(timeout=period)
                except Timeout:
                    pos = vel = tsp = tq = None
            else:
                s = m.state
                pos, vel, tsp, tq = s.pos, s.vel, s.torque_setpoint, s.torque
                if bus.frame == "output":
                    pos = None if pos is None else pos / bus.ratio
                    vel = None if vel is None else vel / bus.ratio
                    tsp = None if tsp is None else tsp * bus.ratio
                    tq = None if tq is None else tq * bus.ratio
            if m.state.any_error:
                print("\naxis reported an error — stopping"); break
            if now >= next_print:
                print(f"\r t={now:6.2f}s pos={fmt(pos, '9.4f')} "
                      f"vel={fmt(vel, '8.4f')} "
                      f"T sp/act={fmt(tsp, '7.4f')}/{fmt(tq, '7.4f')} "
                      f"temp={fmt(m.state.temperature, 'd')}°C   ",
                      end="", flush=True)
                next_print = now + max(period, 0.1)
            time.sleep(max(0.0, period - ((time.monotonic() - t0) % period)))
    finally:
        print()
        m.set_input_torque(0.0)
        if not args.hold:
            m.idle()
            print("torque zeroed, axis idled")
        else:
            print("torque zeroed (axis left in closed loop)")
    if m.state.any_error:
        names = decode_flags(AXIS_ERRORS, m.state.axis_error or 0)
        print(f"axis_error: {' | '.join(names) or hex(m.state.axis_error or 0)}")
        return 1
    return 0


def cmd_watch(bus, args):
    m = bus.motor(args.node)
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *a: stop.update(flag=True))
    print("watching (Ctrl-C to stop) — pos/vel from periodic 0x09, "
          "rest polled via RTR")
    while not stop["flag"]:
        try:
            iq = m.get_iq(timeout=0.3)
        except Timeout:
            iq = (None, None)
        try:
            tq = m.get_torques(timeout=0.3)
        except Timeout:
            tq = (None, None)
        s = m.state
        pos, vel = s.pos, s.vel
        if bus.frame == "output":
            pos = None if pos is None else pos / bus.ratio
            vel = None if vel is None else vel / bus.ratio
        age = time.monotonic() - s.t_encoder if s.t_encoder else None
        print(f"\r state={s.axis_state_name():<18} pos={fmt(pos, '9.4f')} "
              f"vel={fmt(vel, '8.4f')} iq={fmt(iq[1], '6.2f')}A "
              f"T={fmt(tq[1], '7.4f')} temp={fmt(s.temperature, 'd')}°C "
              f"enc_age={fmt(age, '.2f')}s "
              f"{'ERROR' if s.any_error else '     '}  ",
              end="", flush=True)
        time.sleep(1.0 / args.rate)
    print()
    return 0


def cmd_sdo(bus, args):
    m = bus.motor(args.node)
    if bus.endpoints is None:
        print("no endpoints JSON found — pass --endpoints"); return 1
    if args.op == "search":
        for p in bus.endpoints.search(args.path):
            e = bus.endpoints.by_path[p]
            print(f"  {e['id']:>4}  {e.get('type', ''):<7} "
                  f"{e.get('access', ''):<3} {p}")
        return 0
    if args.op == "read":
        v = m.sdo_read(args.path)
        print(v)
        return 0
    if args.op == "write":
        e = bus.endpoints.lookup(args.path)
        value = float(args.value) if e["type"] == "float" else int(args.value)
        back = m.sdo_write(args.path, value)
        print(f"written, read back: {back} (RAM only — 'save' to persist)")
        return 0
    return 1


def cmd_limits(bus, args):
    m = bus.motor(args.node)
    m.set_limits(args.vel, args.current)
    print(f"vel_limit={args.vel} rev/s (rotor), current_lim={args.current} A "
          "(RAM only — 'save' to persist)")
    return 0


def cmd_gains(bus, args):
    m = bus.motor(args.node)
    if args.pos_kp is not None:
        m.set_pos_gain(args.pos_kp)
    if args.vel_kp is not None or args.vel_ki is not None:
        if args.vel_kp is None or args.vel_ki is None:
            print("--vel-kp and --vel-ki must be given together"); return 1
        m.set_vel_gains(args.vel_kp, args.vel_ki)
    print("gains sent (RAM only — 'save' to persist)")
    return 0


def cmd_commission(bus, args):
    """Baud rate + 120R termination (+ optional node id / deadman), then save.

    In a chain, only the LAST drive keeps r120 on; the host adapter end
    provides the other 120R. After a baud change + save the drive reboots at
    the new rate — can0 must be reconfigured to match before it answers again.
    """
    m = bus.motor(args.node)
    if bus.endpoints is None:
        print("no endpoints JSON found — pass --endpoints"); return 1

    hw = [m.sdo_read(p) for p in ("hw_version_major", "hw_version_minor")]
    fw = [m.sdo_read(p) for p in
          ("fw_version_major", "fw_version_minor", "fw_version_revision")]
    baud = m.sdo_read("can.config.baud_rate")
    r120 = m.sdo_read("can.config.enable_r120")
    print(f"node {args.node}: hw {hw[0]}.{hw[1]}  fw {fw[0]}.{fw[1]}.{fw[2]}  "
          f"baud {baud}  r120 {'on' if r120 else 'off'}")

    changed = []
    if args.r120 in ("on", "off"):
        want = 1 if args.r120 == "on" else 0
        if r120 != want:
            m.sdo_write("can.config.enable_r120", want)
            changed.append(f"r120 -> {args.r120}")
    if args.deadman is not None:
        m.set_deadman(args.deadman)
        changed.append(f"break_timeout -> {args.deadman} ms")
    if args.node_id is not None and args.node_id != args.node:
        m.set_node_id(args.node_id)
        m = bus.motor(args.node_id)  # drive answers on the new id from now on
        changed.append(f"node_id -> {args.node_id}")

    baud_changed = args.baud is not None and args.baud != baud
    if baud_changed:
        m.sdo_write("can.config.baud_rate", args.baud, verify=False)
        try:
            back = m.sdo_read("can.config.baud_rate")
            if back != args.baud:
                print(f"baud write failed: read back {back}"); return 1
            changed.append(f"baud -> {args.baud}")
        except Timeout:
            # fw 0.6.5 behaves this way (bench-verified): the baud register
            # applies immediately, before any save.
            print(f"drive stopped answering — it switched to {args.baud} bps "
                  "immediately (RAM only).")
            print("Reconfigure the host interface to match:")
            _print_relink(args.channel, args.baud)
            if args.no_save:
                print("--no-save: power-cycle the drive to revert to its "
                      f"saved config, or run 'save {m.node_id}' at the new "
                      "baud to keep it (this also persists any other change "
                      "made above)")
            else:
                print(f"then run 'save {m.node_id}' to persist everything "
                      "set by this command")
            return 1

    if not changed:
        print("nothing to change")
        return 0
    print("changed: " + ", ".join(changed))
    if args.no_save:
        print("NOT saved (--no-save): RAM only, lost at reboot — and a later "
              "'save' would persist these too")
        return 0
    return 0 if _save_verified(
        bus, m, reboots_to_baud=args.baud if baud_changed else None) else 1


def _print_relink(channel: str, baud: int) -> None:
    print(f"  sudo ip link set {channel} down\n"
          f"  sudo ip link set {channel} type can bitrate {baud}\n"
          f"  sudo ip link set {channel} up")


def cmd_rates(bus, args):
    m = bus.motor(args.node)
    updates = {k: getattr(args, k) for k in RATE_ENDPOINTS
               if getattr(args, k) is not None}
    if updates:
        m.set_feedback_rates(**updates)
        print(f"set {updates} (RAM only — 'save' while idle to persist)")
    print("periodic broadcast periods (ms, 0 = off):")
    for k, v in m.get_feedback_rates().items():
        print(f"  {k:<17} {v}")
    return 0


def cmd_deadman(bus, args):
    m = bus.motor(args.node)
    m.set_deadman(args.ms)
    print(f"break_timeout={args.ms} ms "
          f"({'disabled' if args.ms == 0 else 'armed on next CAN command'}; "
          "'save' while idle to persist)")
    return 0


def _save_verified(bus, m, reboots_to_baud=None) -> bool:
    """Send Save_Configuration and verify the drive actually rebooted
    (heartbeat gap + recovery), then read config back. 0x1F is fire-and-forget
    on the wire — without this check a lost frame looks like success.
    reboots_to_baud: the drive is expected to come back at a DIFFERENT baud —
    only the reboot gap is verified."""
    if not m.state.t_heartbeat:
        time.sleep(0.4)  # maybe we just haven't seen one yet
    hb = bool(m.state.t_heartbeat)
    m.save_configuration()
    print("save_configuration sent — verifying reboot ...")
    if not hb:
        print("  (heartbeat broadcast disabled — cannot verify the reboot; "
              "checking reachability only)")
        time.sleep(3.0)
    else:
        gap = came_back = False
        t_end = time.monotonic() + 8.0
        while time.monotonic() < t_end:
            age = m.state.heartbeat_age()
            if not gap and age > 0.35:
                gap = True
                if reboots_to_baud is not None:
                    print(f"drive rebooted — it now talks at {reboots_to_baud} "
                          "bps. Reconfigure the host interface and verify:")
                    _print_relink("can0", reboots_to_baud)
                    return True
            elif gap and age < 0.2:
                came_back = True
                break
            time.sleep(0.05)
        if not gap:
            print("WARNING: no reboot detected — the drive most likely never "
                  "received the save frame; config NOT persisted.\n"
                  "Host->drive frames being dropped is the usual cause: check "
                  "bus health (at 1 Mbps use sample-point 0.875 — ./canup.sh "
                  "sets it), termination, and that the axis is IDLE.")
            return False
        if not came_back:
            print("drive rebooted but has not come back on this link — its "
                  "SAVED baud rate probably differs from the current "
                  "interface bitrate. Re-run ./canup.sh with the saved rate "
                  "and 'scan'.")
            return False
    try:
        if bus.endpoints is not None:
            baud = m.sdo_read("can.config.baud_rate")
            print(f"verified: drive rebooted and answers again "
                  f"(saved baud_rate = {baud}) — config persisted")
        else:
            print("drive rebooted and is heartbeating again — config persisted")
        return True
    except Timeout:
        print("drive is heartbeating but SDO reads fail — bus is marginal "
              "(sample point / termination); treat the save as unverified")
        return False


def cmd_save(bus, args):
    return 0 if _save_verified(bus, bus.motor(args.node)) else 1


def cmd_reboot(bus, args):
    bus.motor(args.node).reboot()
    print("reboot sent")
    return 0


def cmd_mit(bus, args):
    m = bus.motor(args.node)
    if args.enable:
        m.set_controller_mode(3, 9)  # POSITION + MIT (user manual 3.3.4)
        m.set_axis_state(8)
        time.sleep(0.3)
    try:
        fb = m.mit_control(args.pos, args.vel, args.kp, args.kd, args.torque)
        print(f"feedback: pos={fb[0]:.4f} rad  vel={fb[1]:.4f} rad/s  "
              f"torque={fb[2]:.4f} Nm  (output shaft)")
    except Timeout:
        print("no MIT echo — is the drive in input_mode 9 + closed loop?")
        return 1
    return 0


# ----------------------------------------------------------------- parser


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__)
    ap.add_argument("-c", "--channel", default="can0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--frame", choices=("motor", "output"), default="motor",
                    help="unit frame for pos/vel/torque (default motor=rotor)")
    ap.add_argument("--ratio", type=float, default=8.0,
                    help="gear ratio for --frame output (default 8)")
    ap.add_argument("--endpoints", help="endpoints JSON for sdo commands "
                    f"(default {DEFAULT_ENDPOINTS.name})")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("scan", help="list nodes broadcasting on the bus")
    p.add_argument("--time", type=float, default=1.5)
    p.set_defaults(fn=cmd_scan)

    p = sub.add_parser("status", help="one-shot full status of a node")
    p.add_argument("node", type=int)
    p.set_defaults(fn=cmd_status)

    p = sub.add_parser("errors", help="detailed decoded errors (0x03)")
    p.add_argument("node", type=int)
    p.add_argument("--clear", action="store_true")
    p.set_defaults(fn=cmd_errors)

    p = sub.add_parser("enable", help="set mode + enter closed loop")
    p.add_argument("node", type=int)
    p.add_argument("--mode", choices=sorted(MODES), default="torque")
    p.set_defaults(fn=cmd_enable)

    p = sub.add_parser("idle", help="request IDLE state")
    p.add_argument("node", type=int)
    p.set_defaults(fn=cmd_idle)

    p = sub.add_parser("estop", help="emergency stop one node or 'all'")
    p.add_argument("node", help="node id or 'all'")
    p.set_defaults(fn=cmd_estop)

    p = sub.add_parser("torque", help="stream a torque setpoint with feedback")
    p.add_argument("node", type=int)
    p.add_argument("nm", type=float, help="torque (Nm; rotor frame unless "
                   "--frame output)")
    p.add_argument("--rate", type=float, default=100.0, help="command Hz")
    p.add_argument("--duration", type=float, help="seconds (default: Ctrl-C)")
    p.add_argument("--poll", action="store_true",
                   help="RTR-poll pos/vel/torque every cycle instead of "
                        "using periodic broadcasts")
    p.add_argument("--enable", action="store_true",
                   help="enter torque mode + closed loop first")
    p.add_argument("--ramp", action="store_true",
                   help="with --enable: use torque-ramp input mode")
    p.add_argument("--hold", action="store_true",
                   help="stay in closed loop on exit (default: idle)")
    p.set_defaults(fn=cmd_torque)

    p = sub.add_parser("watch", help="live feedback line")
    p.add_argument("node", type=int)
    p.add_argument("--rate", type=float, default=5.0)
    p.set_defaults(fn=cmd_watch)

    p = sub.add_parser("sdo", help="read/write/search arbitrary parameters")
    p.add_argument("op", choices=("read", "write", "search"))
    p.add_argument("node", type=int)
    p.add_argument("path", help="endpoint path (suffix match) or search term")
    p.add_argument("value", nargs="?", help="value for write")
    p.set_defaults(fn=cmd_sdo)

    p = sub.add_parser("limits", help="set velocity/current limits (0x0F)")
    p.add_argument("node", type=int)
    p.add_argument("--vel", type=float, required=True, help="rev/s rotor")
    p.add_argument("--current", type=float, required=True, help="A")
    p.set_defaults(fn=cmd_limits)

    p = sub.add_parser("gains", help="set loop gains (0x1A/0x1B)")
    p.add_argument("node", type=int)
    p.add_argument("--pos-kp", type=float)
    p.add_argument("--vel-kp", type=float)
    p.add_argument("--vel-ki", type=float)
    p.set_defaults(fn=cmd_gains)

    p = sub.add_parser("commission",
                       help="set baud rate / 120R termination / node id, "
                            "then save to flash")
    p.add_argument("node", type=int)
    p.add_argument("--baud", type=int, choices=(125000, 250000, 500000, 1000000),
                   help="new CAN baud rate (all GIM6010 drives support 1M)")
    p.add_argument("--r120", choices=("on", "off", "keep"), default="keep",
                   help="built-in 120R terminator — 'on' only for the last "
                        "drive in the chain (default: keep)")
    p.add_argument("--node-id", type=int, help="assign a new node id (0-63)")
    p.add_argument("--deadman", type=int, metavar="MS",
                   help="also set break_timeout (comm-loss stop)")
    p.add_argument("--no-save", action="store_true",
                   help="apply to RAM only, skip save+reboot")
    p.set_defaults(fn=cmd_commission)

    p = sub.add_parser("rates", help="show/set periodic broadcast periods")
    p.add_argument("node", type=int)
    for key in RATE_ENDPOINTS:
        p.add_argument(f"--{key.replace('_', '-')}", type=int, dest=key,
                       metavar="MS", help=f"{key} period, 0 = off")
    p.set_defaults(fn=cmd_rates)

    p = sub.add_parser("deadman", help="CAN timeout stop (break_timeout)")
    p.add_argument("node", type=int)
    p.add_argument("ms", type=int, help="timeout in ms; 0 disables")
    p.set_defaults(fn=cmd_deadman)

    p = sub.add_parser("save", help="save config to flash + reboot (0x1F)")
    p.add_argument("node", type=int)
    p.set_defaults(fn=cmd_save)

    p = sub.add_parser("reboot", help="reboot drive (0x16)")
    p.add_argument("node", type=int)
    p.set_defaults(fn=cmd_reboot)

    p = sub.add_parser("mit", help="one MIT command (0x08) with echo decode")
    p.add_argument("node", type=int)
    p.add_argument("--pos", type=float, default=0.0, help="rad, output shaft")
    p.add_argument("--vel", type=float, default=0.0, help="rad/s")
    p.add_argument("--kp", type=float, default=0.0)
    p.add_argument("--kd", type=float, default=0.0)
    p.add_argument("--torque", type=float, default=0.0, help="Nm, output shaft")
    p.add_argument("--enable", action="store_true",
                   help="set input_mode 9 + closed loop first")
    p.set_defaults(fn=cmd_mit)

    args = ap.parse_args(argv)
    try:
        bus = make_bus(args)
    except OSError as e:
        print(f"cannot open CAN interface '{args.channel}': {e}", file=sys.stderr)
        print("is the adapter plugged in and the interface up?  "
              "(./canup.sh [bitrate])", file=sys.stderr)
        return 2
    with bus:
        try:
            return args.fn(bus, args)
        except Timeout as e:
            print(f"timeout: {e}", file=sys.stderr)
            return 2


if __name__ == "__main__":
    sys.exit(main())
