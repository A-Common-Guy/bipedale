#!/usr/bin/env python3
"""BNO085 over SPI1 on a Raspberry Pi 4 (spidev1.0, CS0=GPIO18, INT=GPIO25, RST=GPIO24).

Needs `dtoverlay=spi6-1cs` in /boot/firmware/config.txt. Prints the game/rotation
vector as quaternion + roll/pitch/yaw. Minimal SHTP implementation, no vendor library.
"""
import math
import struct
import sys
import time
from datetime import timedelta

import gpiod
import spidev
from gpiod.line import Bias, Direction, Edge, Value

INT_PIN, RST_PIN = 25, 24
CHIP = "/dev/gpiochip0"

CH_COMMAND, CH_EXEC, CH_CONTROL, CH_REPORTS = 0, 1, 2, 3
REPORT_IDS = {"rotation": 0x05, "game": 0x08, "accel": 0x01, "gyro": 0x02}


class BNO085:
    def __init__(self, bus=6, dev=0, speed_hz=3_000_000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, dev)
        self.spi.mode = 0b11  # CPOL=1, CPHA=1
        self.spi.max_speed_hz = speed_hz
        self.lines = gpiod.request_lines(
            CHIP,
            consumer="bno085",
            config={
                INT_PIN: gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP),
                RST_PIN: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.ACTIVE),
            },
        )
        self.tx_seq = [0] * 6

    def int_asserted(self):
        return self.lines.get_value(INT_PIN) == Value.INACTIVE  # active low

    def wait_int(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.int_asserted():
                return True
            self.lines.wait_edge_events(timedelta(milliseconds=2))
            if self.lines.wait_edge_events(timedelta(0)):
                self.lines.read_edge_events()
        return self.int_asserted()

    def reset(self):
        self.lines.set_value(RST_PIN, Value.INACTIVE)
        time.sleep(0.02)
        self.lines.set_value(RST_PIN, Value.ACTIVE)
        time.sleep(0.2)

    def read_packet(self, timeout=1.0):
        if not self.wait_int(timeout):
            return None
        hdr = self.spi.xfer2([0, 0, 0, 0])
        length = (hdr[0] | (hdr[1] << 8)) & 0x7FFF
        if length <= 4:
            return None
        # INT stays low until the full packet is clocked out; re-read it in one go.
        if not self.wait_int(0.1):
            return None
        data = self.spi.xfer2([0] * length)
        return data[2], data[3], bytes(data[4:length])

    def send(self, channel, payload):
        # The chip only takes a write while it is signalling with INT (WAKE is tied high).
        if not self.wait_int(1.0):
            print("warning: INT not asserted before write; chip may ignore it")
        length = len(payload) + 4
        pkt = [length & 0xFF, length >> 8, channel, self.tx_seq[channel]] + list(payload)
        self.tx_seq[channel] = (self.tx_seq[channel] + 1) & 0xFF
        self.spi.xfer2(pkt)

    def enable(self, report_id, period_us):
        self.send(CH_CONTROL, struct.pack("<BBBHIII", 0xFD, report_id, 0, 0, period_us, 0, 0))


def parse_reports(payload):
    i = 0
    while i < len(payload):
        rid = payload[i]
        if rid == 0xFB:  # base timestamp reference
            i += 5
        elif rid == 0xFA:  # timestamp rebase
            i += 5
        elif rid in (0x05, 0x08):  # rotation vector / game rotation vector
            qi, qj, qk, qr = (v / 16384.0 for v in struct.unpack_from("<4h", payload, i + 4))
            yield rid, (qr, qi, qj, qk)
            i += 14 if rid == 0x05 else 12
        else:
            return  # unknown report, stop parsing this packet


def quat_to_euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def main():
    kind = sys.argv[1] if len(sys.argv) > 1 else "game"
    imu = BNO085()
    imu.reset()

    # Like Adafruit's driver: read one boot packet, then write while INT is still asserted.
    # Draining everything first lets the chip go quiet, and it then ignores writes.
    if not imu.wait_int(2.0):
        sys.exit("No INT after reset: check INT/RST wiring and 3V3/GND.")
    first = imu.read_packet(1.0)
    print(f"first packet: {first[:2] if first else None}")
    imu.send(CH_CONTROL, bytes([0xF9, 0]))  # product ID request
    imu.enable(REPORT_IDS[kind], 10_000)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        pkt = imu.read_packet(0.2)
        if pkt and pkt[0] == CH_CONTROL and pkt[2][0] == 0xF8:
            b = pkt[2]
            print(f"Product ID: part {struct.unpack_from('<I', b, 4)[0]}, "
                  f"SW {b[2]}.{b[3]}.{struct.unpack_from('<H', b, 12)[0]}")
            break
    else:
        print("WARNING: no product ID response")

    print(f"Streaming {kind} vector at 100 Hz, Ctrl-C to stop")
    last = 0.0
    while True:
        pkt = imu.read_packet(1.0)
        if pkt is None:
            print("timeout waiting for data")
            continue
        ch, _, body = pkt
        if ch != CH_REPORTS:
            continue
        for rid, q in parse_reports(body):
            now = time.monotonic()
            if now - last > 0.1:
                r, p, y = quat_to_euler(q)
                print(f"q=({q[0]:+.3f} {q[1]:+.3f} {q[2]:+.3f} {q[3]:+.3f})  "
                      f"roll={r:+7.1f} pitch={p:+7.1f} yaw={y:+7.1f}")
                last = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
