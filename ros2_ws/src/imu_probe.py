"""Decode BNO085 SHTP-UART frames straight off the port, read-only.

Mirrors protocol.hpp framing so a mismatch shows up as a decode failure here.
"""
import collections
import os
import sys
import termios
import time

PORT = "/dev/ttyUSB0"
BAUD = termios.B3000000
DELIM = 0x7E
ESCAPE = 0x7D

SIZES = {
    0x01: 10, 0x02: 10, 0x03: 10, 0x04: 10, 0x05: 14, 0x06: 10,
    0x08: 12, 0x09: 14, 0xF1: 16, 0xF8: 16, 0xFA: 5, 0xFB: 5, 0xFC: 17,
}
NAMES = {
    0x01: "accelerometer", 0x02: "gyroscope", 0x03: "magnetic",
    0x04: "linear accel", 0x05: "rotation vector", 0x06: "gravity",
    0x08: "game rotation vector", 0x09: "geomag rotation",
    0xFB: "timebase", 0xFA: "timestamp rebase", 0xF1: "command response",
}


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
    iflag = 0
    oflag = 0
    lflag = 0
    cflag = termios.CS8 | termios.CREAD | termios.CLOCAL
    cc = list(cc)
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW,
                      [iflag, oflag, cflag, lflag, BAUD, BAUD, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def frames(fd, seconds):
    buf = bytearray()
    end = time.time() + seconds
    while time.time() < end:
        try:
            chunk = os.read(fd, 8192)
        except BlockingIOError:
            time.sleep(0.002)
            continue
        if not chunk:
            time.sleep(0.002)
            continue
        buf.extend(chunk)
        while True:
            first = buf.find(DELIM)
            if first < 0:
                buf.clear()
                break
            second = buf.find(DELIM, first + 1)
            if second < 0:
                del buf[:first]
                break
            raw = bytes(buf[first + 1:second])
            del buf[:second]
            decoded = bytearray()
            escaped = False
            for byte in raw:
                if escaped:
                    decoded.append(byte ^ 0x20)
                    escaped = False
                elif byte == ESCAPE:
                    escaped = True
                else:
                    decoded.append(byte)
            if len(decoded) > 5 and decoded[0] == 0x01:
                yield bytes(decoded[1:])


def main():
    fd = open_port(PORT)
    try:
        ids = collections.Counter()
        channels = collections.Counter()
        total = 0
        undecodable = 0
        for packet in frames(fd, 2.0):
            total += 1
            channels[packet[2]] += 1
            at = 4
            progressed = False
            while at < len(packet):
                size = SIZES.get(packet[at], 0)
                if size == 0 or at + size > len(packet):
                    break
                ids[packet[at]] += 1
                progressed = True
                at += size
            if not progressed:
                undecodable += 1
        print(f"decoded SHTP packets in 2 s : {total}")
        print(f"packets with no valid report: {undecodable}")
        print(f"channels seen               : {dict(channels)}")
        print("report ids seen:")
        for rid, count in sorted(ids.items()):
            print(f"  0x{rid:02x} {NAMES.get(rid, '?'):24s} {count}")
        need = {0x08: "orientation", 0x02: "gyro", 0x01: "accel"}
        missing = [n for r, n in need.items() if ids[r] == 0]
        print(f"\ndriver needs 0x08+0x02+0x01 -> missing: {missing or 'nothing'}")
    finally:
        os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
