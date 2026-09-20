"""Replicate BNO085Sensor::on_activate exactly and time the handshake.

Sends the same 8 set-feature commands with the same per-byte gap, then measures
how long each required report takes to reappear.
"""
import os
import termios
import time

PORT = "/dev/ttyUSB0"
BAUD = termios.B3000000
DELIM = 0x7E
ESCAPE = 0x7D
REPORT_INTERVAL_US = 5000
TX_BYTE_GAP_US = 1000
ONLINE_TIMEOUT_S = 0.15

SIZES = {
    0x01: 10, 0x02: 10, 0x03: 10, 0x04: 10, 0x05: 14, 0x06: 10,
    0x08: 12, 0x09: 14, 0xF1: 16, 0xF8: 16, 0xFA: 5, 0xFB: 5, 0xFC: 17,
}


def open_port():
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    cc = list(termios.tcgetattr(fd)[6])
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, [
        0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0, BAUD, BAUD, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def escape(data):
    out = bytearray()
    for byte in data:
        if byte in (DELIM, ESCAPE):
            out.append(ESCAPE)
            out.append(byte ^ 0x20)
        else:
            out.append(byte)
    return out


def uart_frame(channel, sequence, payload):
    length = 4 + len(payload)
    packet = bytearray([length & 0xFF, length >> 8, channel, sequence])
    packet.extend(payload)
    framed = bytearray([DELIM, 0x01])
    framed.extend(escape(packet))
    framed.append(DELIM)
    return framed


def send_report(fd, seq, report, interval):
    command = bytearray(17)
    command[0] = 0xFD
    command[1] = report
    command[5:9] = interval.to_bytes(4, "little")
    frame = uart_frame(2, seq, command)
    for byte in frame:
        os.write(fd, bytes([byte]))
        if TX_BYTE_GAP_US:
            time.sleep(TX_BYTE_GAP_US / 1e6)
    termios.tcdrain(fd)
    return len(frame)


def collect(fd, seconds, seen, start):
    buf = bytearray()
    end = time.time() + seconds
    while time.time() < end:
        try:
            chunk = os.read(fd, 8192)
        except BlockingIOError:
            chunk = b""
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
            if len(decoded) <= 5 or decoded[0] != 0x01:
                continue
            packet = bytes(decoded[1:])
            at = 4
            while at < len(packet):
                size = SIZES.get(packet[at], 0)
                if size == 0 or at + size > len(packet):
                    break
                if packet[at] in (0x08, 0x02, 0x01) and packet[at] not in seen:
                    seen[packet[at]] = time.time() - start
                at += size
        if all(r in seen for r in (0x08, 0x02, 0x01)):
            return


def main():
    fd = open_port()
    try:
        termios.tcflush(fd, termios.TCIFLUSH)
        seq = 0
        write_start = time.time()
        total_bytes = 0
        for report in (0x03, 0x04, 0x05, 0x06, 0x09):
            total_bytes += send_report(fd, seq, report, 0)
            seq += 1
        for report in (0x08, 0x02, 0x01):
            total_bytes += send_report(fd, seq, report, REPORT_INTERVAL_US)
            seq += 1
        write_s = time.time() - write_start
        print(f"set-feature bytes written : {total_bytes}")
        print(f"time spent writing        : {write_s:.3f} s"
              f"   (tx_byte_gap_us={TX_BYTE_GAP_US})")

        start = time.time()
        seen = {}
        collect(fd, 3.0, seen, start)
        names = {0x08: "orientation", 0x02: "gyro", 0x01: "accel"}
        print("\ntime from end-of-write until each report reappears:")
        for rid, label in names.items():
            when = seen.get(rid)
            print(f"  {label:12s} " +
                  (f"{when * 1000:7.1f} ms" if when is not None
                   else "   never (within 3 s)"))
        worst = max(seen.values()) if len(seen) == 3 else None
        print(f"\ndriver deadline is online_timeout_s = {ONLINE_TIMEOUT_S} s")
        if worst is None:
            print("VERDICT: sensor did not resume all three reports at all")
        elif worst > ONLINE_TIMEOUT_S:
            print(f"VERDICT: needs {worst:.3f} s, deadline is "
                  f"{ONLINE_TIMEOUT_S} s -> activation cannot succeed")
        else:
            print(f"VERDICT: all three within {worst:.3f} s, deadline is fine")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
