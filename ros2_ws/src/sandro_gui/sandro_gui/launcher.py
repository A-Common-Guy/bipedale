"""Starts, watches and stops the robot's `ros2 launch`.

The launch writes to a file in runs_dir rather than to a pipe, and runs in its
own session, so restarting the panel never takes the robot down with it: on
start the panel re-attaches to the run recorded in current.json. A launch
started elsewhere (deploy/pi/pi launch in another shell) is found in /proc
and shown as external; it can be stopped but its output is not here.

A real launch never arms the drives (arm_hardware:=false): the controller
manager comes up with the IMU active, the drives configured but inactive and
the chosen controller loaded inactive; arming and starting the controller are
the panel's own steps. A run is `starting` until the panel finds that state
(mark_ready), and a `process has died` line in its output is recorded at once.

Stopping is one SIGINT to the launch's process group, exactly a Ctrl-C: the
controller manager deactivates the drives (zero torque, idle, deadman off)
before it exits. SIGTERM follows only after stop_timeout_s.
"""

from __future__ import annotations

import json
import os
import re
import shlex
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from .errors import Busy
from .logbook import ERROR, INFO, WARN, Logbook, parse_line

# stack: the whole robot, drives not armed; imu: the BNO085 alone (diagnostic).
MODES = ("stack", "imu")
_ARGUMENT = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*:=\S*$")
_NAME = re.compile(r"^[A-Za-z0-9_]+$")
_IMU_ARGS = ("imu_transport:=", "imu_spi_device:=", "imu_port:=")
LAUNCH_FILES = ("sandro.launch.py", "imu_check.launch.py", "commissioning.launch.py")


@dataclass
class LaunchSettings:
    hardware: str = "real"
    args: list[str] = field(default_factory=list)
    simulation_args: list[str] = field(default_factory=list)
    overrides_path: str | None = None
    stop_timeout_s: float = 20.0
    runs_dir: str = "gui_runs"
    keep_runs: int = 30
    keep_bytes: int = 200 * 1024 ** 2
    controllers: list[str] = field(default_factory=list)
    available_hardware: tuple[str, ...] = ("real",)


def build_command(request: dict, settings: LaunchSettings) -> list[str]:
    """The `ros2 launch` command line for one Start request; ValueError when invalid."""
    mode = request.get("mode", "stack")
    hardware = request.get("hardware", settings.hardware)
    controller = request.get("controller", "none") or "none"
    if hardware not in ("real", "mujoco"):
        raise ValueError(f"hardware must be real or mujoco, not {hardware!r}")
    if hardware not in settings.available_hardware:
        raise ValueError(f"{hardware} is not installed here")
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}")
    if controller != "none" and not _NAME.match(controller):
        raise ValueError(f"invalid controller name {controller!r}")
    extra = shlex.split(request.get("extra", "") or "")
    for token in extra:
        if not _ARGUMENT.match(token):
            raise ValueError(f"extra arguments must be name:=value, not {token!r}")

    if hardware == "real" and mode == "imu":
        imu = [a for a in settings.args if a.startswith(_IMU_ARGS)]
        return ["ros2", "launch", "sandro_bringup", "imu_check.launch.py", *imu, *extra]

    command = ["ros2", "launch", "sandro_bringup", "sandro.launch.py", f"hardware:={hardware}"]
    if hardware == "real":
        command += list(settings.args)
        command.append("arm_hardware:=false")
        state_source = request.get("state_source", "encoder")
        if state_source not in ("encoder", "mit"):
            raise ValueError("state_source must be encoder or mit")
        period = int(request.get("encoder_period_ms", 5))
        if not 1 <= period <= 100:
            raise ValueError("encoder_period_ms must be within 1-100")
        command += [f"state_source:={state_source}", f"encoder_period_ms:={period}"]
    else:
        command += list(settings.simulation_args)
    teleop = bool(request.get("teleop", True))
    command += [f"controller:={controller}", f"teleop:={'true' if teleop else 'false'}"]
    if request.get("overrides", True) and settings.overrides_path:
        command.append(f"extra_params:={settings.overrides_path}")
    return command + extra


def died_process(line: str) -> str | None:
    """The process a `process has died` line of ros2 launch names, without its -N suffix."""
    if "process has died" not in line:
        return None
    return re.sub(r"-\d+$", "", parse_line(line)[1])


def prune(paths: list[Path], keep: int, keep_bytes: int, current: Path | None = None) -> list[Path]:
    """The oldest of `paths` (sorted by name, so by time) beyond `keep` files or `keep_bytes`."""
    old = [p for p in sorted(paths) if p != current]
    sizes = {}
    for path in old:
        try:
            sizes[path] = path.stat().st_size
        except OSError:
            sizes[path] = 0
    total = sum(sizes.values()) + (current.stat().st_size if current and current.exists() else 0)
    count = len(old) + (1 if current else 0)
    doomed = []
    for path in old:
        if count <= max(1, keep) and total <= keep_bytes:
            break
        doomed.append(path)
        count -= 1
        total -= sizes[path]
    return doomed


def _cmdline(pid: int) -> list[str]:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as stream:
            return [a.decode(errors="replace") for a in stream.read().split(b"\0") if a]
    except OSError:
        return []


def _start_ticks(pid: int) -> int | None:
    try:
        with open(f"/proc/{pid}/stat", encoding="ascii", errors="replace") as stream:
            fields = stream.read().rsplit(")", 1)[1].split()
        if fields[0] == "Z":
            return None
        return int(fields[19])
    except (OSError, IndexError, ValueError):
        return None


def is_launch(argv: list[str]) -> bool:
    """A `ros2 launch` of one of the Sandro launch files."""
    if "launch" not in argv or not any(a.endswith("ros2") for a in argv[:2]):
        return False
    return any(os.path.basename(a) in LAUNCH_FILES for a in argv)


def find_launches(exclude: set[int]) -> list[dict]:
    found = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) in exclude:
            continue
        pid = int(entry)
        argv = _cmdline(pid)
        if argv and is_launch(argv) and _start_ticks(pid) is not None:
            found.append({"pid": pid, "command": " ".join(argv[1:] if argv[0].endswith("python3") else argv)})
    return found


class Launcher:
    def __init__(self, logbook: Logbook, settings: LaunchSettings, cwd: str | None = None) -> None:
        self.log = logbook
        self.settings = settings
        self.cwd = cwd or os.getcwd()
        self.runs = Path(self.cwd, settings.runs_dir)
        self.runs.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None
        self._run: dict | None = None
        self._state = "stopped"
        self._external: list[dict] = []
        self._last_exit: dict | None = None
        self._tail = None
        self._partial = b""
        self._listeners = []
        self._restore()
        threading.Thread(target=self._watch, name="launch-watch", daemon=True).start()

    def on_start(self, callback) -> None:
        self._listeners.append(callback)

    # Status -------------------------------------------------------------

    def status(self) -> dict:
        with self._lock:
            run = dict(self._run) if self._run else None
            state = self._state
            if run is None and self._external:
                state = "external"
        if run:
            run["uptime_s"] = time.time() - run["started"]
            run["died"] = list(run.get("died", []))
        return {"state": state, "run": run, "external": list(self._external),
                "last_exit": self._last_exit}

    def running(self) -> bool:
        return self.status()["state"] in ("starting", "running", "stopping", "external")

    def mark_ready(self) -> None:
        with self._lock:
            if self._state != "starting" or self._run is None:
                return
            self._state = "running"
            took = time.time() - self._run["started"]
        self.log.note(f"stack ready after {took:.0f} s")

    # Start / stop -------------------------------------------------------

    def start(self, request: dict) -> dict:
        command = build_command(request, self.settings)
        with self._lock:
            if self._run is not None or self._external:
                raise Busy("a launch is already running; stop it first")
            stamp = time.strftime("%Y%m%d-%H%M%S")
            log_path = self.runs / f"run-{stamp}.log"
            env = dict(os.environ, PYTHONUNBUFFERED="1", RCUTILS_COLORIZED_OUTPUT="0",
                       RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}")
            with open(log_path, "wb") as stream:
                stream.write(f"# {time.strftime('%Y-%m-%d %H:%M:%S')} "
                             f"{shlex.join(command)}\n".encode())
                stream.flush()
                proc = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                        stdin=subprocess.DEVNULL, cwd=self.cwd, env=env,
                                        start_new_session=True)
            self._proc = proc
            self._run = {"pid": proc.pid, "pgid": proc.pid, "ticks": _start_ticks(proc.pid),
                         "log": log_path.name, "started": time.time(), "request": request,
                         "command": shlex.join(command), "died": []}
            self._state = "starting"
            self._open_tail(log_path, from_start=True)
            self._save()
        self._prune()
        self.log.note(f"started: {shlex.join(command)}")
        for callback in self._listeners:
            callback()
        return self.status()

    def stop(self) -> dict:
        with self._lock:
            if self._run is not None:
                pid, pgid, own = self._run["pid"], self._run["pgid"], True
            elif self._external:
                pid = self._external[0]["pid"]
                try:
                    pgid = os.getpgid(pid)
                except OSError:
                    return self.status()
                own = False
            else:
                return self.status()
            if self._state == "stopping":
                return self.status()
            self._state = "stopping" if own else self._state
        threading.Thread(target=self._stop, args=(pid, pgid), daemon=True).start()
        return self.status()

    def _stop(self, pid: int, pgid: int) -> None:
        self.log.note(f"stopping launch (SIGINT to process group {pgid})", WARN)
        steps = ((signal.SIGINT, self.settings.stop_timeout_s), (signal.SIGTERM, 5.0),
                 (signal.SIGKILL, 2.0))
        for number, wait in steps:
            try:
                os.killpg(pgid, number)
            except ProcessLookupError:
                return
            except PermissionError as error:
                self.log.note(f"cannot signal the launch: {error}", ERROR)
                return
            deadline = time.monotonic() + wait
            while time.monotonic() < deadline:
                if not self._alive(pid):
                    return
                time.sleep(0.1)
            if number != signal.SIGKILL:
                self.log.note(f"launch still running after {wait:.0f} s; sending "
                              f"{'SIGTERM' if number == signal.SIGINT else 'SIGKILL'}", ERROR)

    def _alive(self, pid: int) -> bool:
        with self._lock:
            if self._proc is not None and self._proc.pid == pid:
                return self._proc.poll() is None
        return _start_ticks(pid) is not None

    # Runs ---------------------------------------------------------------

    def runs_list(self) -> list[dict]:
        rows = []
        for path in sorted(self.runs.glob("run-*.log"), reverse=True):
            stat = path.stat()
            rows.append({"name": path.name, "bytes": stat.st_size, "modified": stat.st_mtime})
        return rows

    def run_path(self, name: str) -> Path | None:
        if not re.fullmatch(r"run-\d{8}-\d{6}\.log", name):
            return None
        path = self.runs / name
        return path if path.is_file() else None

    def _prune(self) -> None:
        current = self.runs / self._run["log"] if self._run else None
        for path in prune(list(self.runs.glob("run-*.log")), self.settings.keep_runs,
                          self.settings.keep_bytes, current):
            try:
                path.unlink()
            except OSError:
                pass

    # Internals ----------------------------------------------------------

    def _save(self) -> None:
        state = self.runs / "current.json"
        if self._run is None:
            state.unlink(missing_ok=True)
        else:
            state.write_text(json.dumps(self._run))

    def _restore(self) -> None:
        try:
            run = json.loads((self.runs / "current.json").read_text())
        except (OSError, ValueError):
            return
        if run.get("ticks") is not None and _start_ticks(run["pid"]) == run["ticks"]:
            run.setdefault("died", [])
            self._run, self._state = run, "starting"
            self._open_tail(self.runs / run["log"], from_start=True)
            self._read_tail(replay=True)
            self.log.note(f"re-attached to the running launch (pid {run['pid']})")
        else:
            (self.runs / "current.json").unlink(missing_ok=True)

    def _open_tail(self, path: Path, from_start: bool) -> None:
        if self._tail is not None:
            self._tail.close()
        try:
            self._tail = open(path, "rb")
            if not from_start:
                self._tail.seek(0, os.SEEK_END)
        except OSError:
            self._tail = None
        self._partial = b""

    def _read_tail(self, replay: bool = False) -> None:
        if self._tail is None:
            return
        chunk = self._tail.read()
        if not chunk:
            return
        lines = (self._partial + chunk).split(b"\n")
        self._partial = lines.pop()
        for line in lines:
            text = line.decode(errors="replace")
            self.log.add_line(text, replay=replay)
            process = died_process(text)
            if process and self._run is not None:
                self._run["died"].append(process)

    def _watch(self) -> None:
        next_scan = next_prune = 0.0
        while True:
            time.sleep(0.2)
            try:
                with self._lock:
                    self._read_tail()
                    run = self._run
                    if run is not None:
                        code = None
                        if self._proc is not None and self._proc.pid == run["pid"]:
                            code = self._proc.poll()
                            finished = code is not None
                        else:
                            finished = _start_ticks(run["pid"]) != run["ticks"]
                        if finished:
                            self._read_tail()
                            self._finish(run, code)
                now = time.monotonic()
                if now >= next_scan:
                    next_scan = now + 2.0
                    own = {self._run["pid"]} if self._run else set()
                    self._external = find_launches(own | {os.getpid()})
                if now >= next_prune:
                    next_prune = now + 60.0
                    with self._lock:
                        self._prune()
            except Exception as error:  # noqa: BLE001 - the watcher must survive
                self.log.note(f"launch watcher: {error}", ERROR)

    def _finish(self, run: dict, code: int | None) -> None:
        self._last_exit = {"code": code, "at": time.time(), "log": run["log"],
                           "command": run["command"]}
        self._run, self._proc, self._state = None, None, "stopped"
        self._save()
        if self._tail is not None:
            self._tail.close()
            self._tail = None
        level = INFO if code in (0, None, -signal.SIGINT) else ERROR
        self.log.note(f"launch exited{'' if code is None else f' with code {code}'}", level)
