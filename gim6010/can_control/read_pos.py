#!/usr/bin/env python3
"""Read-only position/turn monitor for GIM6010 drives over CANSimple.

Prints the RAW rotor-side position (as it comes off the wire, no gear
conversion) plus the whole-turn count for any number of nodes selected by
CAN node id. The motor is never enabled: only RTR read requests are sent
(0x09 Get_Encoder_Estimates, 0x0A Get_Encoder_Count) — no Set_Axis_State,
no Set_Controller_Mode, no setpoints. With --passive nothing is transmitted
at all and only the drives' periodic broadcasts are decoded.

Examples:
    read_pos.py 1                    # node 1, 5 Hz, RTR polled
    read_pos.py 1 2 6                # three motors, one line each
    read_pos.py --scan               # every node heartbeating on the bus
    read_pos.py 1 2 --once           # single snapshot
    read_pos.py 1 --passive --rate 20   # decode broadcasts only, TX nothing
    read_pos.py 1 --zero             # count turns from the current position
    read_pos.py 1 2 --csv turns.csv --duration 60

Columns: pos = raw position in rotor revolutions, turns = whole turns +
fraction of a turn, out = output-shaft revolutions (pos / --ratio),
shadow/cpr = raw encoder counts, age = time since that pos sample.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time

from gim6010_can import GIM6010Bus, Timeout


def fmt(v, spec=".3f"):
    if v is not None:
        return format(v, spec)
    width = "".join(c for c in spec.split(".")[0] if c.isdigit())
    return "-".rjust(int(width)) if width else "-"


def sample(m, poll: bool, counts: bool, timeout: float):
    """(pos rev, vel rev/s, shadow_count, count_in_cpr, age s) — raw rotor."""
    pos = vel = None
    if poll:
        try:
            pos, vel = m.get_encoder_estimates(timeout=timeout)
            age = 0.0
        except Timeout:
            pass
    if pos is None:  # passive, or the poll timed out: use the last broadcast
        s = m.state
        pos, vel, age = s.pos, s.vel, s.encoder_age()
    sc = cpr = None
    if counts:
        try:
            sc, cpr = m.get_encoder_count(timeout=timeout)
        except Timeout:
            pass
    return pos, vel, sc, cpr, age


def line(node: int, m, pos, vel, sc, cpr, age, ratio: float) -> str:
    turns = "-" if pos is None else (
        f"{math.trunc(pos):+5d} {pos - math.trunc(pos):+.4f}")
    out = None if pos is None else pos / ratio
    return (f"node {node:2d}  pos={fmt(pos, '+10.4f')} rev  "
            f"turns={turns:>13}  vel={fmt(vel, '+8.4f')} rev/s  "
            f"out={fmt(out, '+9.4f')} rev  "
            f"shadow={fmt(sc, 'd'):>8} cpr={fmt(cpr, 'd'):>6}  "
            f"{m.state.axis_state_name():<18} "
            f"age={fmt(age * 1e3 if age != float('inf') else None, '5.0f')} ms")


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("nodes", type=int, nargs="*",
                    help="CAN node ids to read (0-63)")
    ap.add_argument("--scan", action="store_true",
                    help="read every node that heartbeats instead of a list")
    ap.add_argument("-c", "--channel", default="can0")
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--rate", type=float, default=5.0, help="samples per second")
    ap.add_argument("--once", action="store_true", help="one snapshot, then exit")
    ap.add_argument("--duration", type=float, help="seconds (default: Ctrl-C)")
    ap.add_argument("--passive", action="store_true",
                    help="transmit nothing; decode periodic broadcasts only")
    ap.add_argument("--no-counts", action="store_true",
                    help="skip the 0x0A raw-count poll (halves the traffic)")
    ap.add_argument("--zero", action="store_true",
                    help="subtract the first reading, so turns count from now")
    ap.add_argument("--ratio", type=float, default=8.0,
                    help="gear ratio for the output-shaft column (default 8)")
    ap.add_argument("--csv", help="also append samples to this CSV file")
    args = ap.parse_args(argv)

    if not args.nodes and not args.scan:
        ap.error("give at least one node id, or --scan")

    try:
        bus = GIM6010Bus(channel=args.channel, interface=args.interface)
    except OSError as e:
        print(f"cannot open CAN interface '{args.channel}': {e}", file=sys.stderr)
        print("is the adapter plugged in and the interface up?  "
              "(./canup.sh [bitrate])", file=sys.stderr)
        return 2

    poll = not args.passive
    counts = poll and not args.no_counts
    period = 1.0 / args.rate
    timeout = min(0.2, period)
    csv = open(args.csv, "a", buffering=1) if args.csv else None
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *a: stop.update(flag=True))

    with bus:
        if args.scan:
            print(f"listening 1.5s for heartbeats on {args.channel} ...")
            nodes = bus.scan(1.5)
            if not nodes:
                print("no nodes found — check baud rate (factory default 500k, "
                      "canup.sh sets 1M), wiring and termination")
                return 1
        else:
            nodes = args.nodes
            time.sleep(0.3)  # catch a heartbeat / a pos broadcast first
        motors = {n: bus.motor(n) for n in nodes}
        offset = {n: 0.0 for n in nodes}

        if csv and csv.tell() == 0:
            csv.write("t,node,pos_rev,turns,frac,vel_rev_s,out_rev,"
                      "shadow_count,count_in_cpr,axis_state\n")
        print(f"reading nodes {nodes} — {'passive (no TX)' if args.passive else 'RTR polled'}"
              f", {args.rate:g} Hz, motor NOT enabled"
              f"{'' if args.once else ' (Ctrl-C to stop)'}")
        redraw = sys.stdout.isatty() and not args.once

        t0 = time.monotonic()
        first = True
        while not stop["flag"]:
            now = time.monotonic() - t0
            rows = []
            for n, m in motors.items():
                pos, vel, sc, cpr, age = sample(m, poll, counts, timeout)
                if args.zero and first and pos is not None:
                    offset[n] = pos
                if pos is not None:
                    pos -= offset[n]
                rows.append(line(n, m, pos, vel, sc, cpr, age, args.ratio))
                if csv:
                    csv.write(
                        f"{now:.4f},{n},{fmt(pos, '.6f')},"
                        f"{'' if pos is None else math.trunc(pos)},"
                        f"{fmt(None if pos is None else pos - math.trunc(pos), '.6f')},"
                        f"{fmt(vel, '.6f')},"
                        f"{fmt(None if pos is None else pos / args.ratio, '.6f')},"
                        f"{fmt(sc, 'd')},{fmt(cpr, 'd')},"
                        f"{m.state.axis_state_name()}\n")
            if redraw and not first:
                sys.stdout.write(f"\033[{len(rows)}A")
            eol = "\033[K\n" if redraw else "\n"
            sys.stdout.write("".join(r + eol for r in rows))
            sys.stdout.flush()
            first = False
            if args.once or (args.duration and now >= args.duration):
                break
            time.sleep(max(0.0, period - ((time.monotonic() - t0) % period)))

        if all(m.state.pos is None for m in motors.values()):
            print("no position data — the drives broadcast pos/vel only once "
                  "the encoder is ready; check 'gim_cli.py status <node>' and, "
                  "in --passive mode, that encoder_rate_ms > 0 ('rates <node>')")
            return 1
    if csv:
        csv.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
