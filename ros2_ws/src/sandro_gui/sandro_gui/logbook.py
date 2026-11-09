"""One log for everything the panel shows: launch output, /rosout and GUI actions.

Warnings and errors are also folded into a problem list keyed by their text
with the numbers masked, so a fault the drivers repeat once a second (DRIVE
FAULT, IMU FAULT) is one row with a count instead of a scrolling wall.
"""

from __future__ import annotations

import re
import threading
import time
from collections import deque

DEBUG, INFO, WARN, ERROR, FATAL = 10, 20, 30, 40, 50
LEVELS = {"DEBUG": DEBUG, "INFO": INFO, "WARN": WARN, "WARNING": WARN,
          "ERROR": ERROR, "FATAL": FATAL}
NAMES = {DEBUG: "debug", INFO: "info", WARN: "warn", ERROR: "error", FATAL: "fatal"}

_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
_LEVEL = r"(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)"
# [ERROR] [1789812592.262848452] [controller_manager]: message
_ROS = re.compile(r"^\[" + _LEVEL + r"\]\s+\[(\d+(?:\.\d+)?)\]\s+\[([^\]]+)\]:\s?(.*)$")
# [ERROR] [ros2_control_node-3]: process has died [pid 42, exit code -6, ...]
_LAUNCH = re.compile(r"^\[" + _LEVEL + r"\]\s+\[([^\]]+)\]:\s?(.*)$")
# [ros2_control_node-3] <anything the process printed>
_PROCESS = re.compile(r"^\[([A-Za-z0-9_.\-]+)\]\s+(.*)$")
_TROUBLE = re.compile(r"\b(error|exception|traceback|fatal|failed|fault)\b", re.I)
_NUMBERS = re.compile(r"0x[0-9a-fA-F]+|\d+(?:\.\d+)?")


def parse_line(line: str) -> tuple[int, str, str, float | None]:
    """(level, source, text, stamp) of one line of `ros2 launch` output.

    stamp is the node's own log time when the line carries one, so a log
    read back after a panel restart keeps its times.
    """
    line = _ANSI.sub("", line).rstrip()
    match = _ROS.match(line)
    if match:
        return LEVELS[match[1]], match[3], match[4], float(match[2])
    match = _LAUNCH.match(line)
    if match:
        return LEVELS[match[1]], match[2], match[3], None
    match = _PROCESS.match(line)
    if match:
        process, rest = match[1], match[2]
        inner = _ROS.match(rest)
        if inner:
            return LEVELS[inner[1]], inner[3], inner[4], float(inner[2])
        return (WARN if _TROUBLE.search(rest) else INFO), process, rest, None
    return (WARN if _TROUBLE.search(line) else INFO), "launch", line, None


def problem_key(source: str, text: str) -> str:
    return source + "|" + _NUMBERS.sub("#", text)[:200]


class Logbook:
    def __init__(self, size: int = 5000, ignore: list[str] | None = None) -> None:
        self._entries: deque[dict] = deque(maxlen=size)
        self._problems: dict[str, dict] = {}
        self._seq = 0
        self._lock = threading.Lock()
        self._last_stamp = 0.0
        self.ignore = list(ignore or [])

    def add(self, level: int, source: str, text: str, origin: str = "launch",
            stamp: float | None = None) -> dict:
        stamp = time.time() if stamp is None else stamp
        with self._lock:
            self._seq += 1
            entry = {"seq": self._seq, "t": stamp, "level": NAMES.get(level, "info"),
                     "source": source, "text": text, "origin": origin}
            self._entries.append(entry)
            if level >= WARN:
                key = problem_key(source, text)
                problem = self._problems.get(key)
                if problem is None:
                    if len(self._problems) > 300:
                        oldest = min(self._problems, key=lambda k: self._problems[k]["last"])
                        del self._problems[oldest]
                    problem = self._problems[key] = {
                        "key": key, "first": stamp, "count": 0, "source": source}
                problem.update(last=stamp, text=text, level=entry["level"], seq=self._seq)
                problem["count"] += 1
        return entry

    def add_line(self, line: str, origin: str = "launch", replay: bool = False) -> dict | None:
        """One line of launch output; `replay` when reading back an earlier part of the log."""
        if not line.strip() or any(pattern in line for pattern in self.ignore):
            return None
        level, source, text, stamp = parse_line(line)
        # Read back, a line without its own time (process output, a traceback)
        # takes the previous line's; live, it is simply now.
        if stamp is None:
            stamp = self._last_stamp if replay and self._last_stamp else None
        else:
            self._last_stamp = stamp
        return self.add(level, source, text, origin, stamp)

    def note(self, text: str, level: int = INFO) -> dict:
        """Something the panel itself did or refused, shown in the same log."""
        return self.add(level, "gui", text, origin="gui")

    @property
    def seq(self) -> int:
        return self._seq

    def since(self, seq: int, limit: int = 1000) -> list[dict]:
        with self._lock:
            entries = [e for e in self._entries if e["seq"] > seq]
        return entries[-limit:]

    def problems(self, limit: int = 60) -> list[dict]:
        with self._lock:
            rows = sorted(self._problems.values(), key=lambda p: p["last"], reverse=True)
            return [dict(p) for p in rows[:limit]]

    def clear_problems(self) -> None:
        with self._lock:
            self._problems.clear()
