#!/usr/bin/env python3
"""BNO085 UART-SHTP diagnostic: what does the sensor actually stream?

  sniff  Passive. Writes nothing, bulk-reads the port and decodes SHTP, so it
         reports whatever reports were last enabled on the chip. Run it right
         after stopping app.py to see the configuration that session left
         behind, which separates "the sensor never sent it" from "the app never
         read it".

  app    Drives the real Bno085UartShtp class from imu.py and measures how
         often each published field changes.

Both modes need exclusive access to the port, so stop app.py first.
"""
from __future__ import annotations

import argparse
import collections
import pathlib
import statistics
import sys
import time

import serial

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from imu import (  # noqa: E402
    Bno085UartShtp,
    _REPORT_NAMES,
    _SENSOR_REPORTS,
    _ShtpFramer,
    _parse_report,
    _split_reports,
)

_CONTROL_NAMES = {
    0xF1: "command_response",
    0xF8: "product_id_response",
    0xFA: "timestamp_rebase",
    0xFB: "base_timestamp",
    0xFC: "get_feature_response",
}


def name_for(report_id: int) -> str:
    return _REPORT_NAMES.get(
        report_id, _CONTROL_NAMES.get(report_id, f"unknown_0x{report_id:02X}")
    )


class Stats:
    def __init__(self):
        self.counts = collections.Counter()
        self.changes = collections.Counter()
        self.gaps = collections.defaultdict(list)
        self.last_value = {}
        self.last_time = {}

    def observe(self, report_id: int, payload: bytes, now: float) -> None:
        self.counts[report_id] += 1
        previous = self.last_time.get(report_id)
        if previous is not None:
            self.gaps[report_id].append(now - previous)
        self.last_time[report_id] = now

        if report_id not in _SENSOR_REPORTS:
            return
        values = _parse_report(report_id, payload)
        if self.last_value.get(report_id) != values:
            self.changes[report_id] += 1
        self.last_value[report_id] = values


def run_sniff(args) -> int:
    framer = _ShtpFramer()
    stats = Stats()
    backlog = []
    total_bytes = 0

    with serial.Serial(args.port, args.baudrate, timeout=0.05) as port:
        port.reset_input_buffer()
        print(f"\nsniffing {args.seconds:.0f} s on {args.port} @ {args.baudrate} ...")
        started = time.monotonic()
        while time.monotonic() - started < args.seconds:
            waiting = port.in_waiting
            backlog.append(waiting)
            chunk = port.read(max(waiting, 1))
            if not chunk:
                continue
            total_bytes += len(chunk)
            now = time.monotonic()
            for packet in framer.feed(chunk):
                for report_id, payload in _split_reports(packet):
                    stats.observe(report_id, payload, now)
        duration = time.monotonic() - started

    print(f"\nbytes: {total_bytes} ({total_bytes / duration:.0f} B/s)")
    print(f"in_waiting backlog: median {statistics.median(backlog):.0f} B, "
          f"max {max(backlog)} B")
    print(f"\n{'report':<28}{'count':>8}{'rate Hz':>10}{'changes':>10}"
          f"{'median gap ms':>15}{'max gap ms':>13}")
    print("-" * 84)
    for report_id, count in stats.counts.most_common():
        gaps = stats.gaps.get(report_id, [])
        median = statistics.median(gaps) * 1e3 if gaps else float("nan")
        worst = max(gaps) * 1e3 if gaps else float("nan")
        print(f"{name_for(report_id):<28}{count:>8}{count / duration:>10.1f}"
              f"{stats.changes.get(report_id, 0):>10}{median:>15.2f}{worst:>13.2f}")
    for report_id, values in stats.last_value.items():
        print(f"last {name_for(report_id)}: "
              + "  ".join(f"{v:+.4f}" for v in values))
    return 0


def run_app(args) -> int:
    sensor = Bno085UartShtp(args.port, args.baudrate, 0.05, args.report_interval_us)
    sensor.start()
    print(f"\npolling imu.py for {args.seconds:.0f} s ...")

    fields = ("quaternion", "gyro", "accel")
    previous = {field: None for field in fields}
    changes = collections.Counter()
    ages = []
    samples = 0
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds:
            state = sensor.read()
            samples += 1
            ages.append(state.age_s)
            for field in fields:
                value = getattr(state, field)
                if previous[field] is not None and value != previous[field]:
                    changes[field] += 1
                previous[field] = value
            time.sleep(0.001)
    finally:
        sensor.close()
    duration = time.monotonic() - started

    print(f"\npolled {samples} times in {duration:.1f} s ({samples / duration:.0f} Hz)")
    print(f"orientation published: {sensor.samples} "
          f"({sensor.samples / duration:.1f} Hz)")
    print(f"age_s: median {statistics.median(ages) * 1e3:.2f} ms, "
          f"p99 {statistics.quantiles(ages, n=100)[-1] * 1e3:.2f} ms")
    print(f"dropped backlogs: {sensor.overruns}")
    # A still sensor repeats identical values, so this only means something
    # while the board is being moved.
    print(f"\n{'field':<14}{'value changes':>15}{'Hz':>8}")
    print("-" * 37)
    for field in fields:
        print(f"{field:<14}{changes[field]:>15}{changes[field] / duration:>8.1f}")
        print(f"{'  value':<14}" + "  ".join(f"{v:+.4f}" for v in previous[field]))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("sniff", "app"), default="sniff")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=3_000_000)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--report-interval-us", type=int, default=5000)
    args = parser.parse_args()
    return run_sniff(args) if args.mode == "sniff" else run_app(args)


if __name__ == "__main__":
    sys.exit(main())
