#!/usr/bin/env python3
"""Print normalized PS5 controller input; does not command SandroControl."""
from __future__ import annotations

import argparse
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from operator_input import (  # noqa: E402
    ControllerDisconnected,
    Ps5Controller,
)


def report_visible_devices() -> None:
    """Separate "no pad connected" from "pad connected but not readable"."""
    from evdev import InputDevice, list_devices

    paths = list_devices()
    if paths:
        print("\nReadable input devices:")
        for path in paths:
            device = InputDevice(path)
            try:
                print(f"  {path}: {device.name}")
            finally:
                device.close()
    else:
        print("\nNo input device is readable by this user at all.")

    kernel = pathlib.Path("/proc/bus/input/devices")
    hidden = []
    if kernel.exists():
        for block in kernel.read_text(errors="replace").split("\n\n"):
            if "dualsense" not in block.lower():
                continue
            name = next(
                (
                    line.split('"')[1]
                    for line in block.splitlines()
                    if line.startswith("N: Name=")
                ),
                "?",
            )
            for line in block.splitlines():
                if not line.startswith("H: Handlers="):
                    continue
                for handler in line.split("=", 1)[1].split():
                    if handler.startswith("event"):
                        node = f"/dev/input/{handler}"
                        if node not in paths:
                            hidden.append((node, name))
    if hidden:
        print("\nThe kernel sees a DualSense that this user cannot read:")
        for node, name in hidden:
            print(f"  {node}: {name}")
        print(
            "\nGrant access to the input devices, then reconnect the pad:\n"
            "  sudo usermod -aG input $USER   # then log out and back in"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device",
        help="Linux event device, e.g. /dev/input/event12 (auto-detected by default)",
    )
    parser.add_argument(
        "--list", action="store_true", help="list detected DualSense devices and exit"
    )
    parser.add_argument(
        "--rate", type=float, default=20.0, help="maximum printed samples per second"
    )
    args = parser.parse_args()

    if args.list:
        devices = Ps5Controller.discover()
        for path, name in devices:
            print(f"{path}: {name}")
        if devices:
            return 0
        print("No DualSense controller found.")
        report_visible_devices()
        return 1
    if args.rate <= 0:
        parser.error("--rate must be positive")

    controller = Ps5Controller(args.device)
    controller.connect()
    print(f"Connected to {controller.name} at {controller.device_path}")
    print("Reading only; Ctrl+C to stop.")
    period = 1.0 / args.rate
    next_print = time.monotonic()
    try:
        while True:
            state = controller.poll(max(0.0, next_print - time.monotonic()))
            now = time.monotonic()
            if now < next_print:
                continue
            buttons = ",".join(sorted(state.buttons)) or "-"
            print(
                f"\rleft=({state.left_x:+.3f},{state.left_y:+.3f}) "
                f"right=({state.right_x:+.3f},{state.right_y:+.3f}) "
                f"triggers=({state.l2:.3f},{state.r2:.3f}) "
                f"buttons={buttons:<40}",
                end="",
                flush=True,
            )
            next_print = now + period
    except KeyboardInterrupt:
        print()
        return 0
    except ControllerDisconnected as exc:
        print(f"\n{exc}", file=sys.stderr)
        return 1
    finally:
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
