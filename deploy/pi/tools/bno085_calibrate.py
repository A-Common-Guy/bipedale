#!/usr/bin/env python3
"""Re-calibrate the BNO085's accelerometer and gyro and save it in the chip's flash.

Run on the Pi with the robot stack stopped, since it needs the SPI bus:
`deploy/pi/pi imu-cal [--clear]`.

The chip calibrates itself continuously and keeps the result (its DCD) in its
own flash, loaded at every boot. A calibration learnt in bad conditions shows
up as an accelerometer offset, and so as a constant tilt in the attitude every
controller uses. This tool:

  1. with --clear, drops the stored calibration first (SH-2 Clear DCD and reset);
  2. enables accelerometer and gyro dynamic calibration (SH-2 Configure ME calibration);
  3. shows the calibrated acceleration in the body frame (x forward, y left, z up,
     after the holder's mount yaw) with the chip's accuracy flag, while you hold
     the robot still for two seconds on each of six faces: upright, upside down,
     nose up, nose down, lying on its left side, lying on its right side;
  4. saves the calibration (SH-2 Save DCD) once you confirm.

Each face is measured too: the six means give every axis's offset and gain,
printed before saving, so the result is checked rather than trusted.
"""
import argparse
import math
import struct
import sys
import time

from bno085_spi_probe import BNO085, CH_CONTROL, CH_REPORTS

G = 9.80665
ACCEL, GYRO, GAME = 0x01, 0x02, 0x08
SAVE_DCD, ME_CALIBRATION, CLEAR_DCD_AND_RESET = 0x06, 0x07, 0x0B
FACES = {(2, 1): "upright", (2, -1): "upside down", (0, 1): "nose up", (0, -1): "nose down",
         (1, 1): "on its right side", (1, -1): "on its left side"}
# Still means the acceleration varies by less than STILL_M_S2 over STILL_S. Not
# the gyro: after --clear its zero-rate offset is uncalibrated, and a robot at
# rest can read a few hundredths of rad/s.
HOLD_S, STILL_S, STILL_M_S2 = 2.0, 0.5, 0.3


class Calibrator:
    def __init__(self, imu: BNO085, mount_yaw: float) -> None:
        self.imu = imu
        self.cos, self.sin = math.cos(mount_yaw), math.sin(mount_yaw)
        self.half = (math.cos(mount_yaw / 2), math.sin(mount_yaw / 2))
        self.seq = 0
        self.accel = self.gyro = self.quat = None
        self.accuracy = 0
        self.responses = []

    def body(self, v):
        x, y, z = v
        return (self.cos * x - self.sin * y, self.sin * x + self.cos * y, z)

    def command(self, command: int, params=()) -> None:
        payload = list(params) + [0] * (9 - len(params))
        self.imu.send(CH_CONTROL, bytes([0xF2, self.seq & 0xFF, command] + payload))
        self.seq += 1

    def start(self, period_us: int = 20_000) -> None:
        # Like the driver: one boot packet, then every write while INT is still asserted.
        if not self.imu.wait_int(2.0):
            sys.exit("No INT from the BNO085: is the robot stack still running, or the wiring off?")
        self.imu.read_packet(1.0)
        self.command(ME_CALIBRATION, [1, 1, 0, 0])  # accel, gyro on; mag off (the game vector ignores it)
        for report in (ACCEL, GYRO, GAME):
            self.imu.enable(report, period_us)

    def poll(self) -> None:
        packet = self.imu.read_packet(0.2)
        if packet is None:
            return
        channel, _, body = packet
        if channel == CH_CONTROL and body and body[0] == 0xF1:
            self.responses.append(body)
            return
        if channel != CH_REPORTS:
            return
        i = 0
        while i < len(body):
            rid = body[i]
            if rid in (0xFA, 0xFB):
                i += 5
            elif rid == ACCEL and i + 10 <= len(body):
                self.accuracy = body[i + 2] & 0x03
                self.accel = self.body(v / 256.0 for v in struct.unpack_from("<3h", body, i + 4))
                i += 10
            elif rid == GYRO and i + 10 <= len(body):
                self.gyro = self.body(v / 512.0 for v in struct.unpack_from("<3h", body, i + 4))
                i += 10
            elif rid == GAME and i + 12 <= len(body):
                qi, qj, qk, qr = (v / 16384.0 for v in struct.unpack_from("<4h", body, i + 4))
                hc, hs = self.half  # body in world: chip quaternion times conj(mount yaw)
                self.quat = (qr * hc + qk * hs, qi * hc - qj * hs, qj * hc + qi * hs, qk * hc - qr * hs)
                i += 12
            else:
                return

    def response(self, command: int, timeout: float = 2.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for body in self.responses:
                if len(body) > 5 and body[2] == command:
                    return body
            self.poll()
        return None


def attitude(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def face_of(accel):
    norm = math.sqrt(sum(v * v for v in accel))
    if not 0.8 * G < norm < 1.2 * G:
        return None
    axis = max(range(3), key=lambda k: abs(accel[k]))
    if abs(accel[axis]) < 0.94 * norm:  # within about 20 degrees of a face
        return None
    return axis, 1 if accel[axis] > 0 else -1


def report(done) -> bool:
    print("\nPer axis, from the faces (m/s^2; a perfect chip reads offset 0, gain 1):")
    complete = True
    for axis, name in enumerate("xyz"):
        up, down = done.get((axis, 1)), done.get((axis, -1))
        if up is None or down is None:
            print(f"  {name}: needs both faces")
            complete = False
            continue
        offset, gain = (up + down) / 2.0, (up - down) / (2.0 * G)
        roll_hint = ""
        if name == "y":
            roll_hint = f"  (tilts an upright roll reading by {math.degrees(math.atan2(offset, G)):+.1f} deg)"
        if name == "x":
            roll_hint = f"  (tilts an upright pitch reading by {math.degrees(math.atan2(-offset, G)):+.1f} deg)"
        print(f"  {name}: offset {offset:+.3f}  gain {gain:.4f}{roll_hint}")
    return complete


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--clear", action="store_true", help="drop the stored calibration first")
    parser.add_argument("--mount-yaw", type=float, default=math.pi / 2,
                        help="chip yaw in the holder, rad (the URDF's imu_mount_yaw_rad)")
    parser.add_argument("--yes", action="store_true", help="save without asking")
    args = parser.parse_args()

    imu = BNO085()
    imu.reset()
    cal = Calibrator(imu, args.mount_yaw)
    if args.clear:
        cal.start()
        print("Clearing the stored calibration; the chip resets.")
        cal.command(CLEAR_DCD_AND_RESET)
        time.sleep(1.0)
    cal.start()

    print("Hold the robot still on each face for 2 s: upright, upside down, nose up, nose down,\n"
          "on its left side, on its right side. Any order. Ctrl-C when done.\n")
    done, holding, samples, recent = {}, None, [], []
    last_print = 0.0
    try:
        while len(done) < 6 or cal.accuracy < 3:
            cal.poll()
            if cal.accel is None:
                continue
            now = time.monotonic()
            recent.append((now, cal.accel))
            while recent and now - recent[0][0] > STILL_S:
                recent.pop(0)
            spread = max(max(a[k] for _, a in recent) - min(a[k] for _, a in recent)
                         for k in range(3))
            still = len(recent) >= 10 and spread < STILL_M_S2
            face = face_of(cal.accel) if still else None
            if face != holding:
                holding, samples, since = face, [], now
            elif face is not None and face not in done:
                samples.append(cal.accel[face[0]])
                if now - since >= HOLD_S:
                    done[face] = sum(samples) / len(samples)
                    print(f"\n  recorded {FACES[face]:18s} {done[face]:+.3f} m/s^2 along "
                          f"{'xyz'[face[0]]}  ({len(done)}/6)")
            if now - last_print > 0.25:
                last_print = now
                ax, ay, az = cal.accel
                roll, pitch = attitude(cal.quat) if cal.quat else (float("nan"),) * 2
                todo = " ".join(name.split()[-1] if " " in name else name
                                for key, name in FACES.items() if key not in done)
                sys.stdout.write(
                    f"\r  accel {ax:+6.2f} {ay:+6.2f} {az:+6.2f}  |a| "
                    f"{math.sqrt(ax * ax + ay * ay + az * az):5.2f}  accuracy {cal.accuracy}/3  "
                    f"roll {roll:+6.1f}  pitch {pitch:+6.1f}  "
                    f"{'holding ' + FACES[holding] if holding and holding not in done else 'move':24s}"
                    f" left: {todo or '-'}   ")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    print()
    complete = report(done)
    print(f"\nChip accuracy flag: {cal.accuracy}/3 (3 = calibrated)")
    if not complete or cal.accuracy < 3:
        print("Not every face is done or the chip is not calibrated yet; saving anyway keeps what it has.")
    if not args.yes:
        answer = input("Save this calibration in the chip's flash? [y/N] ").strip().lower()
        if answer != "y":
            print("Not saved. The chip goes back to its stored calibration at the next boot.")
            return
    cal.command(SAVE_DCD)
    response = cal.response(SAVE_DCD)
    if response is None:
        print("No answer to Save DCD; run again and check the result.")
    elif response[5] == 0:
        print("Saved. The chip loads this calibration at every boot.")
    else:
        print(f"Save DCD failed with status {response[5]}.")


if __name__ == "__main__":
    main()
