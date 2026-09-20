"""BNO085 standard UART-SHTP reader (/dev/ttyUSB0 by default).

UART-SHTP wraps every SHTP packet between 0x7E delimiters and escapes 0x7E and
0x7D with 0x7D. The sensor emits one report per packet behind a 5-byte timebase
prefix, so three reports at 200 Hz is roughly 17 kB/s -- far more than a
byte-at-a-time reader can consume, which is why the framing is decoded here
from bulk reads instead of through a CircuitPython transport.

Orientation comes from the game rotation vector: it fuses gyro and accelerometer
only, so it has no magnetometer settling delay and no heading jumps, which is
what a balance controller needs.
"""
from __future__ import annotations

import logging
import os
import threading
import time
from struct import pack_into, unpack_from
from typing import Dict, Iterator, Tuple

from models import ImuState

log = logging.getLogger("sandro.imu")

_DELIMITER = 0x7E
_ESCAPE = 0x7D
_PROTOCOL_ID = 0x01
_HEADER_LEN = 4

_CHANNEL_EXECUTABLE = 1
_CHANNEL_CONTROL = 2

_SET_FEATURE_COMMAND = 0xFD
_SET_FEATURE_LEN = 17

# report id -> (length on the wire, Q-point scalar, value count)
_SENSOR_REPORTS = {
    0x01: (10, 2**-8, 3),   # accelerometer, m/s^2
    0x02: (10, 2**-9, 3),   # calibrated gyroscope, rad/s
    0x03: (10, 2**-4, 3),   # magnetometer
    0x04: (10, 2**-8, 3),   # linear acceleration
    0x05: (14, 2**-14, 4),  # rotation vector (magnetometer fused)
    0x06: (10, 2**-8, 3),   # gravity
    0x08: (12, 2**-14, 4),  # game rotation vector
    0x09: (14, 2**-12, 4),  # geomagnetic rotation vector
}
_CONTROL_REPORTS = {
    0xF1: 16,  # command response
    0xF8: 16,  # product id response
    0xFA: 5,   # timestamp rebase
    0xFB: 5,   # base timestamp
    0xFC: 17,  # get feature response
}

_ACCELEROMETER = 0x01
_GYROSCOPE = 0x02
_GAME_ROTATION_VECTOR = 0x08

_REPORT_NAMES = {
    0x01: "accelerometer",
    0x02: "gyroscope",
    0x03: "magnetometer",
    0x04: "linear_acceleration",
    0x05: "rotation_vector",
    0x06: "gravity",
    0x08: "game_rotation_vector",
    0x09: "geomagnetic_rotation_vector",
}

# Reports this driver never consumes. A previous run may have left them
# streaming, and every unwanted report steals bandwidth from the ones we need.
_UNWANTED_REPORTS = (0x03, 0x04, 0x05, 0x06, 0x09)

_TX_BYTE_GAP_S = 0.001
_CONFIGURE_ATTEMPTS = 3
_VERIFY_WINDOW_S = 0.4
_VERIFY_MIN_REPORTS = 3
_SETTLE_S = 0.1
_RESET_SETTLE_S = 0.6
# Roughly half a second of stream. Anything beyond this is stale motion that is
# worth dropping outright rather than parsing and throwing away one sample at a
# time.
_MAX_BACKLOG_BYTES = 8192


class ImuConfigurationError(RuntimeError):
    """The sensor did not end up streaming exactly the requested reports."""


class Bno085UartShtp:
    """Read the game rotation vector, calibrated gyro and acceleration."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout_s: float,
        report_interval_us: int,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.report_interval_us = report_interval_us
        self._serial = None
        self._framer = _ShtpFramer()
        self._tx_sequence: Dict[int, int] = {}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="bno085")
        self._lock = threading.Lock()
        self._quaternion = (1.0, 0.0, 0.0, 0.0)
        self._gyro = (0.0, 0.0, 0.0)
        self._accel = (0.0, 0.0, 0.0)
        self._last = 0.0
        self.samples = 0
        self.overruns = 0

    def start(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required when imu.enabled is true"
            ) from exc

        self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)
        _tune_ftdi_latency(self.port)
        try:
            self._configure()
        except Exception:
            self._serial.close()
            self._serial = None
            raise
        self._thread.start()

    def _configure(self) -> None:
        """Enable exactly the wanted reports and prove they reach the host.

        The SH-2 feature response only acknowledges the command, so it is not
        evidence that samples are flowing. Verification therefore counts real
        reports on the wire; without it a silently ignored feature shows up much
        later as a field that never updates.
        """
        wanted = (_GAME_ROTATION_VECTOR, _GYROSCOPE, _ACCELEROMETER)
        last_error = None
        for attempt in range(_CONFIGURE_ATTEMPTS):
            if attempt:
                # The plain reconfigure did not take. Escalate to a reset so the
                # sensor hub drops whatever state is confusing it.
                self._soft_reset()
            self._serial.reset_input_buffer()
            for report_id in _UNWANTED_REPORTS:
                self._set_feature(report_id, 0)
            for report_id in wanted:
                self._set_feature(report_id, self.report_interval_us)

            # Reports queued while the commands were being paced out are not
            # evidence of the new configuration, so let them drain first.
            time.sleep(_SETTLE_S)
            self._serial.reset_input_buffer()
            counts = self._count_reports(_VERIFY_WINDOW_S)
            missing = [r for r in wanted if counts.get(r, 0) < _VERIFY_MIN_REPORTS]
            stray = [r for r in _UNWANTED_REPORTS if counts.get(r, 0)]
            if not missing and not stray:
                log.info(
                    "BNO085 streaming %s at %d us",
                    ", ".join(_REPORT_NAMES[r] for r in wanted),
                    self.report_interval_us,
                )
                self._framer.reset()
                self._serial.reset_input_buffer()
                return

            last_error = ImuConfigurationError(
                "BNO085 did not stream the requested reports "
                f"(missing: {_names(missing) or 'none'}; "
                f"unexpected: {_names(stray) or 'none'})"
            )
            log.warning("BNO085 configuration attempt %d failed: %s", attempt, last_error)
        raise last_error

    def _count_reports(self, seconds: float) -> Dict[int, int]:
        framer = _ShtpFramer()
        counts: Dict[int, int] = {}
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            chunk = self._read_available()
            if not chunk:
                continue
            for packet in framer.feed(chunk):
                for report_id, _payload in _split_reports(packet):
                    counts[report_id] = counts.get(report_id, 0) + 1
        return counts

    def _set_feature(self, report_id: int, interval_us: int) -> None:
        """Send a Set Feature Command; interval 0 disables the report."""
        report = bytearray(_SET_FEATURE_LEN)
        report[0] = _SET_FEATURE_COMMAND
        report[1] = report_id
        pack_into("<I", report, 5, interval_us)
        self._send(_CHANNEL_CONTROL, report)

    def _soft_reset(self) -> None:
        self._send(_CHANNEL_EXECUTABLE, bytearray([1]))
        time.sleep(_RESET_SETTLE_S)
        self._serial.reset_input_buffer()

    def _send(self, channel: int, data: bytearray) -> None:
        # SHTP numbers each channel's traffic per direction. Transmit numbering
        # is kept here rather than taken from received packets, which is what
        # made the CircuitPython transport's counter jump backwards.
        sequence = self._tx_sequence.get(channel, 0)
        self._tx_sequence[channel] = (sequence + 1) % 256

        packet = bytearray(_HEADER_LEN + len(data))
        pack_into("<H", packet, 0, len(packet))
        packet[2] = channel
        packet[3] = sequence
        packet[_HEADER_LEN:] = data
        frame = (
            bytes([_DELIMITER, _PROTOCOL_ID]) + _escape(packet) + bytes([_DELIMITER])
        )
        # The sensor hub's UART receiver cannot absorb a back-to-back burst at
        # 3 Mbps and silently drops the command, so host-to-sensor bytes are
        # paced. Only configuration is ever transmitted, so this costs nothing
        # in the steady state.
        for byte in frame:
            self._serial.write(bytes([byte]))
            time.sleep(_TX_BYTE_GAP_S)
        self._serial.flush()

    def _read_available(self) -> bytes:
        """Bulk-read whatever is queued, blocking briefly when nothing is."""
        waiting = self._serial.in_waiting
        if waiting > _MAX_BACKLOG_BYTES:
            self._serial.reset_input_buffer()
            self._framer.reset()
            self.overruns += 1
            log.warning(
                "BNO085 backlog of %d bytes dropped; the reader is falling behind",
                waiting,
            )
            return b""
        return self._serial.read(max(waiting, 1))

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = self._read_available()
            except Exception:
                log.exception("BNO085 read failed; the IMU panel will go offline")
                return
            if chunk:
                self._consume(chunk)

    def _consume(self, chunk: bytes) -> None:
        """Decode a whole chunk, then publish only its newest sample."""
        orientation = gyro = accel = None
        for packet in self._framer.feed(chunk):
            for report_id, payload in _split_reports(packet):
                if report_id == _GAME_ROTATION_VECTOR:
                    orientation = _parse_report(report_id, payload)
                elif report_id == _GYROSCOPE:
                    gyro = _parse_report(report_id, payload)
                elif report_id == _ACCELEROMETER:
                    accel = _parse_report(report_id, payload)
        if orientation is None and gyro is None and accel is None:
            return

        now = time.monotonic()
        with self._lock:
            if orientation is not None:
                i, j, k, real = orientation
                self._quaternion = (real, i, j, k)
                # Only a fresh orientation report may refresh the age, otherwise
                # a stalled sensor still looks online.
                self._last = now
                self.samples += 1
            if gyro is not None:
                self._gyro = gyro
            if accel is not None:
                self._accel = accel

    def read(self) -> ImuState:
        with self._lock:
            age = time.monotonic() - self._last if self._last else float("inf")
            return ImuState(
                quaternion=self._quaternion,
                gyro=self._gyro,
                accel=self._accel,
                online=age < 0.5,
                age_s=age,
            )

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._serial is not None:
            # Leave the sensor quiet so the next run starts from a known state.
            try:
                for report_id in (_GAME_ROTATION_VECTOR, _GYROSCOPE, _ACCELEROMETER):
                    self._set_feature(report_id, 0)
            except Exception:
                log.debug("BNO085 could not be quiesced on close", exc_info=True)
            self._serial.close()
            self._serial = None


class _ShtpFramer:
    """Turns the raw UART byte stream into unescaped SHTP packets."""

    def __init__(self):
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, chunk: bytes) -> list[bytes]:
        self._buffer += chunk
        packets = []
        while True:
            start = self._buffer.find(_DELIMITER)
            if start < 0:
                self._buffer.clear()
                return packets
            end = self._buffer.find(_DELIMITER, start + 1)
            if end < 0:
                del self._buffer[:start]
                return packets
            frame = bytes(self._buffer[start + 1 : end])
            # The closing delimiter also opens the next frame, so keep it.
            del self._buffer[:end]
            if frame:
                packet = _unescape(frame)
                if len(packet) > _HEADER_LEN and packet[0] == _PROTOCOL_ID:
                    packets.append(packet[1:])


def _escape(packet: bytearray) -> bytes:
    out = bytearray()
    for byte in packet:
        if byte in (_DELIMITER, _ESCAPE):
            out += bytes([_ESCAPE, byte ^ 0x20])
        else:
            out.append(byte)
    return bytes(out)


def _unescape(frame: bytes) -> bytes:
    if _ESCAPE not in frame:
        return frame
    out = bytearray()
    escaped = False
    for byte in frame:
        if escaped:
            out.append(byte ^ 0x20)
            escaped = False
        elif byte == _ESCAPE:
            escaped = True
        else:
            out.append(byte)
    return bytes(out)


def _split_reports(packet: bytes) -> Iterator[Tuple[int, bytes]]:
    """Walk the reports batched into one SHTP packet."""
    length = (packet[0] | (packet[1] << 8)) & 0x7FFF
    if length < _HEADER_LEN or length > len(packet):
        return
    data = packet[_HEADER_LEN:length]
    index = 0
    while index < len(data):
        report_id = data[index]
        size = _CONTROL_REPORTS.get(report_id)
        if size is None:
            entry = _SENSOR_REPORTS.get(report_id)
            if entry is None:
                # Unknown id: the rest of the batch cannot be located.
                return
            size = entry[0]
        if index + size > len(data):
            return
        yield report_id, data[index : index + size]
        index += size


def _parse_report(report_id: int, payload: bytes) -> Tuple[float, ...]:
    _length, scalar, count = _SENSOR_REPORTS[report_id]
    values = unpack_from("<" + "h" * count, payload, 4)
    return tuple(value * scalar for value in values)


def _names(report_ids) -> str:
    return ", ".join(_REPORT_NAMES.get(r, hex(r)) for r in report_ids)


def _tune_ftdi_latency(port: str) -> None:
    """Drop the FTDI read latency from its 16 ms default to 1 ms.

    The default holds a burst of samples in the adapter before the kernel sees
    them, which caps orientation freshness at roughly 16-33 ms no matter how
    fast the sensor or the parser is. Best effort: the write needs privileges
    that a normal session does not have, so the README documents a udev rule
    for making it stick.
    """
    node = f"/sys/bus/usb-serial/devices/{os.path.basename(port)}/latency_timer"
    try:
        with open(node, "w") as handle:
            handle.write("1")
    except OSError as exc:
        log.info(
            "FTDI latency_timer on %s left at its default (%s); "
            "orientation freshness is capped near 16 ms",
            port,
            exc,
        )
