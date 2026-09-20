#!/usr/bin/env python3
"""Zero the GIM6010-8 load-side encoder at the current position over USB.

Requires:
  pip install odrive

The Type-C port is data only. Power the drive from the XT30 (15–60 V) first,
then plug USB.

User-zero procedure from the CyberBeast / GIM6010-8 manuals
(section 3.4.3 / 3.1.7):

    index_offset = pos_estimate + index_offset

On dual-encoder motors (GIM6010-8) this offset is kept across power cycles
and the dual-encoder memory window shifts with it. save_configuration()
stores it and reboots the drive.
"""

import argparse
import sys
import time

import odrive
from odrive.utils import AxisState, dump_errors


def connect(serial_number=None, timeout=10):
    print("Looking for a drive on USB...")
    odrv = odrive.find_any(serial_number=serial_number, timeout=timeout)
    print(
        f"Connected  serial={odrv.serial_number}  "
        f"hw={odrv.hw_version_major}.{odrv.hw_version_minor}  "
        f"vbus={odrv.vbus_voltage:.1f} V"
    )
    return odrv


def _enc_attr(enc, name, default="n/a"):
    """Read an encoder field if this firmware exposes it."""
    try:
        return getattr(enc, name)
    except AttributeError:
        return default


def print_encoder(odrv, label):
    # hw 3.11 (this drive) exposes pos_estimate / index_offset / pos_abs.
    # sec_enc_pos_abs exists only on later firmware (3.12+), so do not assume it.
    enc = odrv.axis0.encoder
    extra = ""
    sec = _enc_attr(enc, "sec_enc_pos_abs")
    if sec != "n/a":
        extra = f"  sec_enc_pos_abs={sec}"
    print(
        f"{label}:  pos_estimate={enc.pos_estimate:.6f} turns  "
        f"index_offset={enc.config.index_offset:.6f} turns  "
        f"pos_abs={enc.pos_abs}{extra}"
    )


def idle(odrv):
    if odrv.axis0.current_state != AxisState.IDLE:
        print("Putting axis in IDLE...")
        odrv.axis0.requested_state = AxisState.IDLE
        deadline = time.time() + 3.0
        while odrv.axis0.current_state != AxisState.IDLE:
            if time.time() > deadline:
                raise RuntimeError("Timed out waiting for IDLE")
            time.sleep(0.05)


def reconnect_after_save(timeout=25):
    """save_configuration() reboots the drive and drops USB.

    Do not filter by serial here: odrv.serial_number is a decimal int, while
    find_any() matches the USB identity string, so a filter never finds it.
    """
    print("Waiting for the drive to come back after reboot...")
    time.sleep(4.0)
    last_error = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        remaining = max(2.0, deadline - time.time())
        try:
            return connect(timeout=min(8.0, remaining))
        except TimeoutError as exc:
            last_error = exc
            print("  still waiting...")
    raise TimeoutError(
        "Drive did not reappear on USB after save_configuration(). "
        "Power-cycle if needed, then re-run this script to confirm the zero."
    ) from last_error


def zero_load_encoder(odrv, persist):
    enc = odrv.axis0.encoder
    if not enc.config.use_index_offset:
        enc.config.use_index_offset = True

    before = enc.pos_estimate
    old_offset = enc.config.index_offset
    # Manual 3.4.3: current pose becomes user zero. pos_estimate does not
    # jump to 0 until after save_configuration() reboots the drive.
    new_offset = before + old_offset
    enc.config.index_offset = new_offset
    time.sleep(0.05)

    written = enc.config.index_offset
    print_encoder(odrv, "After offset write")
    if abs(written - new_offset) > 1e-3:
        raise RuntimeError(
            f"index_offset write failed: wanted {new_offset:.6f}, got {written:.6f}"
        )
    print(
        f"index_offset set to {written:.6f} turns "
        f"(was {old_offset:.6f}). pos_estimate stays at {enc.pos_estimate:.6f} "
        "until the drive reboots."
    )

    if not persist:
        print("Offset applied in RAM only (not saved).")
        return odrv

    print("Saving configuration (drive will reboot)...")
    try:
        odrv.save_configuration()
    except Exception:
        # USB drops when the drive reboots after save.
        pass

    odrv = reconnect_after_save()
    print_encoder(odrv, "After reboot")
    dump_errors(odrv)
    if abs(odrv.axis0.encoder.pos_estimate) > 0.05:
        print(
            "Warning: after reboot, pos_estimate is not near 0. "
            "The offset is saved; confirm the shaft did not move during reboot.",
            file=sys.stderr,
        )
    else:
        print("User zero is active: current pose reads as ~0 turns.")
    return odrv


def main():
    parser = argparse.ArgumentParser(
        description="Zero the GIM6010-8 load-side encoder at the current pose via USB."
    )
    parser.add_argument(
        "-s",
        "--serial",
        help="Drive serial number if more than one is connected",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="Apply the offset in RAM only; do not save or reboot",
    )
    parser.add_argument(
        "-y",
        "--yes",
        action="store_true",
        help="Skip the confirmation prompt",
    )
    args = parser.parse_args()

    odrv = connect(serial_number=args.serial)
    dump_errors(odrv)
    print_encoder(odrv, "Before zero")

    if not args.yes:
        answer = input(
            "Set the current load-side pose as user zero? [y/N] "
        ).strip().lower()
        if answer not in ("y", "yes"):
            print("Aborted.")
            return 1

    idle(odrv)
    zero_load_encoder(odrv, persist=not args.no_save)
    return 0


if __name__ == "__main__":
    sys.exit(main())
