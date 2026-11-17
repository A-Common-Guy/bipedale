"""The Sysid page: the measurements the RL training needs, one button each.

Each measurement is a sandro_bringup tool run as its own process with fixed
arguments (nothing to type, nothing to get wrong), after the panel has
checked that the robot is in the state the measurement needs:

  yaw   sysid_yaw.py: the WBC sweeps the opposed wheel torque to 45 Hz while
        it balances; the result is yaw_plant.json, the measured yaw response
        sandro_rl's train.loop_margin closes a policy's yaw loop on.
  buzz  buzz_record.py: records the running controller through stand, drive,
        stand, and says which loop its buzz is in.

A run lives in <dir>/<kind>_<time>/ with the tool's output in panel.log; the
page shows that log, the instruction of the moment (the tools' "NOW:" lines)
and afterwards the result with its downloads. Stop is one SIGINT, as Ctrl-C:
the yaw tool then switches the excitation off and puts the leg stiffness
back. The E-stop stops a running measurement too.
"""

from __future__ import annotations

import io
import json
import os
import signal
import subprocess
import sys
import tarfile
import threading
import time
from pathlib import Path

from .errors import Busy

WBC = "whole_body_controller"
# A yaw run is good when its response is coherent at least this far: past the
# 9 Hz yaw mode and the resonance near 12 Hz, with room above.
USABLE_HZ = 22.0
# What each run asks of the operator, shown before the start.
MEASUREMENTS = {
    "yaw": {
        "title": "Yaw response to 45 Hz",
        "tool": "sysid_yaw.py",
        "needs": "wbc",
        "seconds": 75,
        "purpose": "The robot's yaw answer to opposed wheel torque, 2 to 45 Hz, at 0.2 and 0.4 Nm per "
                   "wheel. Training holds a policy's yaw loop inside this measured response "
                   "(sandro_rl train.loop_margin).",
        "steps": ["Start the stack with whole_body_controller, arm and start it (Run page).",
                  "Let it stand on a flat floor with a metre of room around it; choose the height "
                  "with the pad (standing, crouched and tall are three separate runs).",
                  "Press Start and keep your hands off the sticks: the wheels twitch against each "
                  "other for about a minute, faster and faster. Hand on the E-stop."],
        "result": "yaw_plant.json",
    },
    "buzz": {
        "title": "Record a controller's buzz",
        "tool": "buzz_record.py",
        "needs": "balancer",
        "seconds": 95,
        "purpose": "Stand, drive, stand with the controller under test, recorded and split into the "
                   "yaw and pitch modes: where a buzz is, and at which frequency.",
        "steps": ["Start the controller under test from the Run page (for example "
                  "rl_flat_level_sysid) and let it stand.",
                  "Press Start and follow the instruction shown here: 30 s standing still, 30 s of "
                  "gentle driving and turning with the pad, 30 s standing still.",
                  "Nothing is commanded by this run; you drive."],
        "result": "summary.json",
    },
}


def command(kind: str, out: Path, tools: Path, label: str = "", python: str = sys.executable) -> list[str]:
    """The exact process of a measurement. The arguments are fixed here on purpose."""
    tool = str(tools / MEASUREMENTS[kind]["tool"])
    if kind == "yaw":
        return [python, "-u", tool, "--kp", "12", "--pulses", "0", "--chirp-nm", "0.2", "0.4",
                "--f0", "2", "--f1", "45", "--chirp-s", "30", "--settle", "3", "--yes", "--out", str(out)]
    return [python, "-u", tool, "--label", label, "--stand", "30", "--drive", "30", "--yes",
            "--out", str(out)]


def blocked(kind: str, ros: dict | None, launch: dict | None) -> str | None:
    """Why a measurement cannot start now (None: it can), from the panel's slow snapshot."""
    if (launch or {}).get("state") not in ("running", "external"):
        return "the stack is not running: start it on the Run page"
    if not ros or not ros.get("manager"):
        return "no controller manager yet"
    running = [c for c in ros.get("controllers", []) if c.get("owns_commands")]
    if not running:
        return "no controller is running: arm and start one on the Run page"
    controller = running[0]
    if controller.get("fault"):
        return f"{controller['name']} is {controller['fault']['state']}: restart it on the Run page"
    if MEASUREMENTS[kind]["needs"] == "wbc":
        if controller["name"] != WBC:
            return f"{controller['name']} is running: this run needs {WBC} (it makes the sweep)"
        values = ((ros.get("diagnostics") or {}).get(WBC) or {}).get("values") or {}
        if values.get("engaged") != 1.0:
            return f"{WBC} is not balancing yet"
    return None


def controller_under_test(ros: dict | None) -> str:
    running = [c["name"] for c in (ros or {}).get("controllers", []) if c.get("owns_commands")]
    return running[0] if running else ""


class Sysid:
    def __init__(self, cwd: Path, log, tools: Path, cfg: dict | None = None, runner=None) -> None:
        cfg = cfg or {}
        self.dir = Path(cwd) / cfg.get("dir", "experiments")
        self.tools = Path(tools)
        self.log = log
        # runner(kind, out, label) -> argv: the demo and the tests replace the tools.
        self.runner = runner or (lambda kind, out, label: command(kind, out, self.tools, label))
        self.lock = threading.Lock()
        self.job: dict | None = None
        self.process: subprocess.Popen | None = None
        self._restore()

    # Job ----------------------------------------------------------------

    def _state_file(self) -> Path:
        return self.dir / "job.json"

    def _save(self) -> None:
        try:
            self.dir.mkdir(parents=True, exist_ok=True)
            self._state_file().write_text(json.dumps(self.job))
        except OSError:
            pass

    def _restore(self) -> None:
        """A measurement outlives a panel restart: pick it up again by its pid."""
        try:
            job = json.loads(self._state_file().read_text())
        except (OSError, ValueError):
            return
        if job.get("state") == "running":
            try:
                os.kill(job["pid"], 0)
            except OSError:
                job.update(state="ended", ended=time.time())
        self.job = job

    def _poll(self) -> None:
        job = self.job
        if not job or job["state"] != "running":
            return
        if self.process is not None:
            code = self.process.poll()
            if code is None:
                return
            self.process = None
        else:
            try:
                os.kill(job["pid"], 0)
                return
            except OSError:
                code = None
        stopped = job.pop("stopping", False)
        job.update(ended=time.time(), code=code,
                   state="stopped" if stopped else "done" if code == 0 else
                   "ended" if code is None else "failed")
        self._save()
        self.log.note(f"sysid: {job['title']} {job['state']} ({job['name']})")

    def start(self, kind: str, ros: dict | None, launch: dict | None) -> dict:
        if kind not in MEASUREMENTS:
            raise ValueError(f"unknown measurement {kind!r}")
        with self.lock:
            self._poll()
            if self.job and self.job["state"] == "running":
                raise Busy(f"{self.job['title']} is still running")
            reason = blocked(kind, ros, launch)
            if reason:
                raise ValueError(reason)
            label = controller_under_test(ros)
            stamp = time.strftime("%Y%m%d-%H%M%S")
            name = f"{kind}_{stamp}" if kind == "yaw" else f"{kind}_{label}_{stamp}"
            out = self.dir / name
            out.mkdir(parents=True, exist_ok=True)
            argv = self.runner(kind, out, label)
            with open(out / "panel.log", "w") as log:
                log.write("$ " + " ".join(argv) + "\n")
                log.flush()
                self.process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                                                stdin=subprocess.DEVNULL, start_new_session=True)
            self.job = {"kind": kind, "title": MEASUREMENTS[kind]["title"], "name": name,
                        "controller": label, "pid": self.process.pid, "started": time.time(),
                        "seconds": MEASUREMENTS[kind]["seconds"], "state": "running"}
            self._save()
            self.log.note(f"sysid: {self.job['title']} started ({name})")
            return dict(self.job)

    def stop(self) -> dict | None:
        """One SIGINT to the tool, as Ctrl-C; it closes its recording and restores the WBC."""
        with self.lock:
            self._poll()
            job = self.job
            if not job or job["state"] != "running":
                return None
            job["stopping"] = True
            try:
                os.kill(job["pid"], signal.SIGINT)
            except OSError:
                pass
            return dict(job)

    # Views --------------------------------------------------------------

    def summary(self) -> dict | None:
        """The job for the slow snapshot."""
        with self.lock:
            self._poll()
            return dict(self.job) if self.job else None

    def _run_dir(self, name: str) -> Path | None:
        path = self.dir / name
        if not name or "/" in name or name.startswith(".") or not path.is_dir():
            return None
        return path

    def _tail(self, name: str, lines: int = 60) -> list[str]:
        try:
            text = (self.dir / name / "panel.log").read_text(errors="replace")
        except OSError:
            return []
        # The yaw tool rewrites one progress line with carriage returns.
        return [line.split("\r")[-1].rstrip() for line in text.splitlines()][-lines:]

    @staticmethod
    def _result(path: Path) -> dict:
        """What a finished run found, from its summary.json."""
        try:
            summary = json.loads((path / "summary.json").read_text())
        except (OSError, ValueError):
            return {}
        if isinstance(summary, dict):
            return {"text": summary.get("verdict", "")}
        rows = [r for r in summary if r.get("chirp_hz") == r.get("chirp_hz") and "chirp_hz" in r]
        if not rows:
            return {}
        text = "; ".join(f"{r.get('chirp_nm', 0):g} Nm: mode {r['chirp_hz']:.1f} Hz, "
                         f"coherence {r.get('coherence', 0):.2f}" for r in rows)
        result = {"text": text}
        try:
            plant = json.loads((path / "yaw_plant.json").read_text())
            f = plant["f_hz"]
            coherence = plant.get("coherence") or [1.0] * len(f)
            # The training trusts the table up to the first frequency whose
            # coherence is under 0.7 (sandro_rl loop_margin.usable).
            low = [i for i, c in enumerate(coherence) if c < 0.7]
            top = f[max(low[0] - 1, 0)] if low else f[-1]
            result.update(plant=True, f_hz=[f[0], f[-1]], usable_hz=top, weak=top < USABLE_HZ)
        except (OSError, ValueError, KeyError, IndexError):
            pass
        return result

    def runs(self, limit: int = 30) -> list[dict]:
        try:
            paths = sorted((p for p in self.dir.iterdir() if p.is_dir()),
                           key=lambda p: p.stat().st_mtime, reverse=True)
        except OSError:
            return []
        runs = []
        for path in paths[:limit]:
            kind = path.name.split("_", 1)[0]
            if kind == "sysid":            # runs of deploy/pi/pi sysid
                kind = "yaw"
            if kind not in MEASUREMENTS:
                continue
            size = sum(f.stat().st_size for f in path.rglob("*") if f.is_file())
            runs.append({"name": path.name, "kind": kind, "title": MEASUREMENTS[kind]["title"],
                         "time": path.stat().st_mtime, "bytes": size,
                         "files": sorted(f.name for f in path.iterdir() if f.is_file()),
                         "result": self._result(path)})
        return runs

    def status(self, ros: dict | None, launch: dict | None) -> dict:
        job = self.summary()
        tail = self._tail(job["name"]) if job else []
        now = next((line[5:] for line in reversed(tail) if line.startswith("NOW: ")), None)
        return {"job": job, "tail": tail, "now": now if job and job["state"] == "running" else None,
                "controller": controller_under_test(ros),
                "measurements": [dict(id=kind, **{k: m[k] for k in ("title", "purpose", "steps", "seconds")},
                                      blocked=blocked(kind, ros, launch))
                                 for kind, m in MEASUREMENTS.items()],
                "runs": self.runs(), "dir": str(self.dir)}

    def file(self, name: str, file: str) -> Path | None:
        """A result file of a run (never a path outside it)."""
        path = self._run_dir(name)
        if path is None or "/" in file or file.startswith(".") or not (path / file).is_file():
            return None
        return path / file

    def archive(self, name: str) -> bytes | None:
        """The whole run as a .tar, to copy the recording to the laptop in one click."""
        path = self._run_dir(name)
        if path is None:
            return None
        buffer = io.BytesIO()
        with tarfile.open(fileobj=buffer, mode="w") as tar:   # the bag is zstd already
            tar.add(path, arcname=name)
        return buffer.getvalue()


def demo_runner(kind: str, out: Path, label: str) -> list[str]:
    """A stand-in for the tools in --demo: the prompts and files of a short run."""
    script = (
        "import json, sys, time\n"
        "kind, out = sys.argv[1], sys.argv[2]\n"
        "steps = ['stand still, hands off the sticks (4 s)', 'drive and turn gently (4 s)',\n"
        "         'stand still again (4 s)'] if kind == 'buzz' else ['chirp 0.2 Nm (6 s)', 'chirp 0.4 Nm (6 s)']\n"
        "for step in steps:\n"
        "    print('NOW: ' + step, flush=True)\n"
        "    time.sleep(4 if kind == 'buzz' else 6)\n"
        "if kind == 'buzz':\n"
        "    json.dump({'verdict': 'quiet standing: at most 0.03 rad/s (demo)'}, open(out + '/summary.json', 'w'))\n"
        "else:\n"
        "    json.dump([{'kp': 12, 'chirp_nm': 0.2, 'chirp_hz': 12.4, 'coherence': 0.97}], open(out + '/summary.json', 'w'))\n"
        "    json.dump({'f_hz': [3.2, 30.0, 42.0], 'to_gyro': [[0, -2], [0, -0.2], [0, -0.1]],\n"
        "               'to_wheel': [[1, -1]] * 3, 'coherence': [0.95, 0.9, 0.5]}, open(out + '/yaw_plant.json', 'w'))\n"
        "print('NOW: done (demo data, not a measurement)', flush=True)\n")
    return [sys.executable, "-u", "-c", script, kind, str(out)]
