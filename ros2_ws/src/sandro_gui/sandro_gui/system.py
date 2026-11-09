"""Host health: CPU temperature, throttling, load, memory, devices and the CAN link."""

from __future__ import annotations

import glob
import json
import os
import socket
import subprocess
import time

# get_throttled bits (Raspberry Pi firmware): now / since boot.
THROTTLE_BITS = {0: "under-voltage", 1: "frequency capped", 2: "throttled", 3: "soft temp limit"}


def _read(path: str) -> str | None:
    try:
        with open(path, encoding="ascii", errors="replace") as stream:
            return stream.read().strip()
    except OSError:
        return None


def throttled() -> dict | None:
    raw = _read("/sys/devices/platform/soc/soc:firmware/get_throttled")
    if raw is None:
        try:
            out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True,
                                 timeout=1.0).stdout
            raw = out.strip().split("=")[-1] if "=" in out else None
        except (OSError, subprocess.SubprocessError):
            raw = None
    if raw is None:
        return None
    try:
        value = int(raw, 16)
    except ValueError:
        return None
    return {"raw": hex(value),
            "now": [name for bit, name in THROTTLE_BITS.items() if value >> bit & 1],
            "since_boot": [name for bit, name in THROTTLE_BITS.items() if value >> (bit + 16) & 1]}


def can_link(interface: str) -> dict:
    """State of a SocketCAN interface from `ip -details -statistics -json`."""
    try:
        out = subprocess.run(["ip", "-details", "-statistics", "-json", "link", "show", interface],
                             capture_output=True, text=True, timeout=1.0)
    except (OSError, subprocess.SubprocessError) as error:
        return {"present": False, "error": str(error)}
    if out.returncode != 0:
        return {"present": False, "error": out.stderr.strip() or "not found"}
    try:
        link = json.loads(out.stdout)[0]
    except (ValueError, IndexError):
        return {"present": False, "error": "unparsable ip output"}
    info = (link.get("linkinfo") or {}).get("info_data") or {}
    stats = link.get("stats64") or link.get("stats") or {}
    rx, tx = stats.get("rx") or {}, stats.get("tx") or {}
    xstats = (link.get("linkinfo") or {}).get("info_xstats") or {}
    return {
        "present": True,
        "up": "UP" in link.get("flags", []) and link.get("operstate") != "DOWN",
        "operstate": link.get("operstate"),
        "state": info.get("state"),
        "bitrate": (info.get("bittiming") or {}).get("bitrate"),
        "sample_point": (info.get("bittiming") or {}).get("sample_point"),
        "berr_tx": (info.get("berr_counter") or {}).get("tx"),
        "berr_rx": (info.get("berr_counter") or {}).get("rx"),
        "rx_packets": rx.get("packets"), "tx_packets": tx.get("packets"),
        "rx_errors": rx.get("errors"), "tx_errors": tx.get("errors"),
        "rx_dropped": rx.get("dropped"), "tx_dropped": tx.get("dropped"),
        "bus_off": xstats.get("bus_off"), "restarts": xstats.get("restarts"),
        "bus_error": xstats.get("bus_error"),
    }


class SystemInfo:
    def __init__(self, can_interface: str, devices: list[str]) -> None:
        self.can_interface = can_interface
        self.devices = devices
        self._cpu = None
        self._throttled = (0.0, None)
        self._packets = None

    def _throttled_cached(self) -> dict | None:
        # vcgencmd costs a process start; the flags change rarely.
        stamp, value = self._throttled
        if time.monotonic() - stamp > 5.0:
            value = throttled()
            self._throttled = (time.monotonic(), value)
        return value

    def _cpu_percent(self) -> float | None:
        raw = _read("/proc/stat")
        if not raw:
            return None
        values = [int(v) for v in raw.splitlines()[0].split()[1:]]
        idle, total = values[3] + values[4], sum(values)
        previous, self._cpu = self._cpu, (idle, total)
        if previous is None or total == previous[1]:
            return None
        return 100.0 * (1.0 - (idle - previous[0]) / (total - previous[1]))

    def snapshot(self) -> dict:
        temp = _read("/sys/class/thermal/thermal_zone0/temp")
        load = (_read("/proc/loadavg") or "").split()
        mem = {}
        for line in (_read("/proc/meminfo") or "").splitlines():
            key, _, value = line.partition(":")
            if key in ("MemTotal", "MemAvailable"):
                mem[key] = int(value.split()[0]) * 1024
        uptime = (_read("/proc/uptime") or "0").split()[0]
        freq = _read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
        can = can_link(self.can_interface)
        if can.get("present") and can.get("rx_packets") is not None:
            now = time.monotonic()
            previous, self._packets = self._packets, (now, can["rx_packets"], can["tx_packets"])
            if previous and now > previous[0]:
                dt = now - previous[0]
                can["rx_hz"] = (can["rx_packets"] - previous[1]) / dt
                can["tx_hz"] = (can["tx_packets"] - previous[2]) / dt
        return {
            "host": socket.gethostname(),
            "time": time.time(),
            "cpu_temp_c": int(temp) / 1000.0 if temp and temp.lstrip("-").isdigit() else None,
            "cpu_percent": self._cpu_percent(),
            "cpu_mhz": int(freq) / 1000.0 if freq and freq.isdigit() else None,
            "governor": _read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
            "load": [float(v) for v in load[:3]] if len(load) >= 3 else None,
            "cpus": os.cpu_count(),
            "mem_total": mem.get("MemTotal"), "mem_available": mem.get("MemAvailable"),
            "uptime_s": float(uptime),
            "throttled": self._throttled_cached(),
            "devices": {path: os.path.exists(path) for path in self.devices},
            "joysticks": sorted(glob.glob("/dev/input/js*")),
            "can": can,
        }
