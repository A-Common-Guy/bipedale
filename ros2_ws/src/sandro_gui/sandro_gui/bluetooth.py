"""Gamepad pairing through bluetoothctl (BlueZ on the host, over the D-Bus socket).

What the Pi needed so far (deploy/pi/README.md): the radio can come up
rfkill soft-blocked, and BlueZ forgets a discovered device the moment the
scan stops, so the scan keeps running while pairing. The DualSense pairs with
the NoInputNoOutput agent.
"""

from __future__ import annotations

import glob
import os
import re
import shutil
import subprocess
import threading
import time

_MAC = re.compile(r"^[0-9A-F]{2}(:[0-9A-F]{2}){5}$")
PAD_NAMES = ("Wireless Controller", "DualSense")


def _run(args: list[str], timeout: float = 8.0) -> tuple[bool, str]:
    try:
        out = subprocess.run(["bluetoothctl", *args], capture_output=True, text=True,
                             timeout=timeout)
    except subprocess.TimeoutExpired as error:
        return False, (error.stdout or "") if isinstance(error.stdout, str) else "timed out"
    except OSError as error:
        return False, str(error)
    text = re.sub(r"\x1b\[[0-9;]*[A-Za-z]|\x01|\x02", "", out.stdout + out.stderr)
    return out.returncode == 0, text.strip()


def _fields(text: str) -> dict[str, str]:
    fields = {}
    for line in text.splitlines():
        key, sep, value = line.strip().partition(": ")
        if sep and key and key not in fields:
            fields[key] = value.strip()
    return fields


def rfkill() -> list[dict]:
    rows = []
    for path in sorted(glob.glob("/sys/class/rfkill/rfkill*")):
        try:
            kind = open(os.path.join(path, "type")).read().strip()
            soft = open(os.path.join(path, "soft")).read().strip() == "1"
            hard = open(os.path.join(path, "hard")).read().strip() == "1"
        except OSError:
            continue
        if kind == "bluetooth":
            rows.append({"path": path, "soft": soft, "hard": hard})
    return rows


class Bluetooth:
    def __init__(self) -> None:
        self.available = shutil.which("bluetoothctl") is not None
        self._lock = threading.Lock()
        self._scan: subprocess.Popen | None = None
        self._scan_until = 0.0
        self._status: dict = {"available": self.available}
        self._busy: str | None = None
        self._output: list[str] = []
        self._wake = threading.Event()
        if self.available:
            threading.Thread(target=self._poll, name="bluetooth", daemon=True).start()

    # Status -------------------------------------------------------------

    def _poll(self) -> None:
        while True:
            try:
                self._status = self._read_status()
            except Exception as error:  # noqa: BLE001 - keep polling
                self._status = {"available": True, "error": str(error)}
            # Every poll is five bluetoothctl processes and as many D-Bus
            # clients for bluetoothd: quick only while a scan or pairing runs.
            self._wake.wait(1.5 if self.scanning or self._busy else 10.0)
            self._wake.clear()

    @property
    def scanning(self) -> bool:
        return self._scan is not None and self._scan.poll() is None

    @staticmethod
    def _devices(*filters: str) -> dict[str, str] | None:
        ok, listing = _run(["devices", *filters], timeout=4.0)
        if not ok:
            return None
        found = {}
        for line in listing.splitlines():
            parts = line.strip().split(" ", 2)
            if len(parts) >= 2 and parts[0] == "Device" and _MAC.match(parts[1]):
                found[parts[1]] = parts[2] if len(parts) > 2 else ""
        return found

    def _read_status(self) -> dict:
        ok, text = _run(["show"], timeout=4.0)
        controller = _fields(text) if ok else {}
        known = self._devices() or {}
        # BlueZ 5.65+ filters the list; one call per state instead of one per device.
        sets = {state: self._devices(state) for state in ("Paired", "Trusted", "Connected")}
        devices = []
        for mac, name in known.items():
            device = {"mac": mac, "name": name,
                      "gamepad": any(n in name for n in PAD_NAMES)}
            fields = {}
            if device["gamepad"] or any(s is None for s in sets.values()):
                fields = _fields(_run(["info", mac], timeout=4.0)[1])
            for state, members in sets.items():
                value = mac in members if members is not None else fields.get(state) == "yes"
                device[state.lower()] = value
            device.update(battery=fields.get("Battery Percentage"), rssi=fields.get("RSSI"))
            devices.append(device)
        devices.sort(key=lambda d: (not d["connected"], not d["paired"], not d["gamepad"],
                                    d["name"] == "" or _MAC.match(d["name"].replace("-", ":"))
                                    is not None, d["name"]))
        return {
            "available": True,
            "error": None if ok else (text or "bluetoothctl show failed"),
            "controller": controller.get("Controller") or controller.get("Name"),
            "powered": controller.get("Powered") == "yes",
            "power_state": controller.get("PowerState"),
            "discovering": controller.get("Discovering") == "yes",
            "rfkill": rfkill(),
            "scanning": self.scanning,
            "scan_left_s": max(0.0, self._scan_until - time.monotonic()) if self.scanning else 0,
            "devices": devices,
            "busy": self._busy,
            "output": self._output[-30:],
        }

    def snapshot(self) -> dict:
        status = dict(self._status)
        status.update(busy=self._busy, output=self._output[-30:], scanning=self.scanning)
        return status

    # Actions ------------------------------------------------------------

    def _say(self, text: str) -> None:
        for line in text.splitlines():
            if line.strip():
                self._output.append(line.rstrip())
        del self._output[:-200]

    def _action(self, name: str, steps: list[tuple[list[str], float]]) -> dict:
        with self._lock:
            if self._busy:
                return {"ok": False, "message": f"busy: {self._busy}"}
            self._busy = name
        ok = True
        try:
            for args, timeout in steps:
                self._say("$ bluetoothctl " + " ".join(args))
                ok, text = _run(args, timeout)
                self._say(text)
                if not ok:
                    break
        finally:
            self._busy = None
        self._status = self._read_status()
        return {"ok": ok, "message": self._output[-1] if self._output else ""}

    def unblock(self) -> dict:
        done = []
        for row in rfkill():
            if row["soft"]:
                try:
                    with open(os.path.join(row["path"], "soft"), "w") as stream:
                        stream.write("0")
                    done.append(os.path.basename(row["path"]))
                except OSError as error:
                    self._say(f"rfkill {row['path']}: {error}")
                    return {"ok": False, "message": str(error)}
        if done:
            self._say("unblocked " + ", ".join(done))
            time.sleep(0.5)
        return self._action("power on", [(["power", "on"], 5.0)])

    def scan(self, seconds: float = 60.0) -> dict:
        if not self.available:
            return {"ok": False, "message": "bluetoothctl is not installed"}
        with self._lock:
            if self.scanning:
                self._scan.terminate()
            self._scan = subprocess.Popen(
                ["bluetoothctl", "--timeout", str(int(seconds)), "scan", "on"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._scan_until = time.monotonic() + seconds
        self._wake.set()
        self._say(f"scanning for {seconds:.0f} s: hold PS + Create until the light bar flashes")
        return {"ok": True, "message": "scanning"}

    def stop_scan(self) -> dict:
        with self._lock:
            if self.scanning:
                self._scan.terminate()
        return {"ok": True, "message": "scan stopped"}

    def _mac(self, mac: str) -> str:
        mac = (mac or "").upper()
        if not _MAC.match(mac):
            raise ValueError(f"invalid MAC address {mac!r}")
        return mac

    def pair(self, mac: str) -> dict:
        mac = self._mac(mac)
        return self._action(f"pairing {mac}", [
            (["--agent", "NoInputNoOutput", "--timeout", "20", "pair", mac], 25.0),
            (["trust", mac], 5.0),
            (["--timeout", "10", "connect", mac], 15.0)])

    def connect(self, mac: str) -> dict:
        mac = self._mac(mac)
        return self._action(f"connecting {mac}", [(["--timeout", "10", "connect", mac], 15.0)])

    def disconnect(self, mac: str) -> dict:
        mac = self._mac(mac)
        return self._action(f"disconnecting {mac}", [(["disconnect", mac], 8.0)])

    def remove(self, mac: str) -> dict:
        mac = self._mac(mac)
        return self._action(f"forgetting {mac}", [(["remove", mac], 8.0)])
