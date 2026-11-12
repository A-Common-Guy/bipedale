"""The robot's software: the version on disk, built and running, and the one-button update.

Under the workspace root (the robot container's /workspaces/sandro):

    repo.git/            bare repository: commits from the laptop (deploy/pi/pi push)
                         and from GitHub (Fetch, the `origin` remote)
    ros2_ws/src/         the source colcon builds: written by `pi sync` or by a deploy here
    version/source.json  the version ros2_ws/src holds
    version/build.json   the version install/ was built from

A deploy exports one commit's ros2_ws/src, writes only the files that differ
(so make stays incremental), builds what those files need (the rules of
deploy/pi/pi's change_scope), stamps both versions and restarts the panel onto
the new build. `pi sync` and `pi build` stamp the same two files, so the
version shown is right whichever way the code arrived.

Deploys, builds and fetches run as their own process (JobRunner), so a panel
that restarts or dies halfway does not end them; the panel reads their state
(gui_runs/software-job.json) and log back.

Also a script, for deploy/pi/pi and the panel (no ROS environment needed):

    python3 software.py stamp-build [ROOT]   version/build.json from source.json
    python3 -m sandro_gui.software job FILE  run the job FILE describes
"""

from __future__ import annotations

import io
import json
import os
import shutil
import signal
import subprocess
import sys
import tarfile
import threading
import time
from collections import deque
from pathlib import Path

INFO, WARN, ERROR = 20, 30, 40
SOURCE_PREFIX = "ros2_ws/src/"
SKIP_PARTS = {"__pycache__", ".pytest_cache"}
GIT_TIMEOUT_S = 20.0
# The first fetch brings the whole history, recordings included (130 MB in 2026-11).
FETCH_TIMEOUT_S = 900.0
CACHE_S = 3.0


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


def read_version(path: Path) -> dict | None:
    try:
        data = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) and data.get("commit") else None


def write_version(path: Path, data: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=1))
    os.replace(tmp, path)


def same_version(a: dict | None, b: dict | None) -> bool:
    """The same commit, and for uncommitted changes the same sync."""
    if not a or not b or a.get("commit") != b.get("commit"):
        return False
    if bool(a.get("dirty")) != bool(b.get("dirty")):
        return False
    return not a.get("dirty") or a.get("at") == b.get("at")


def label(version: dict | None) -> str:
    if not version:
        return "unknown"
    return (version.get("short") or version["commit"][:7]) + ("+" if version.get("dirty") else "")


def skipped(relpath: str, skip_packages: set[str]) -> bool:
    parts = relpath.split("/")
    return parts[0] in skip_packages or any(p in SKIP_PARTS for p in parts) or relpath.endswith(".pyc")


# Change scope ----------------------------------------------------------------

def build_scope(changes: list[tuple[str, str]], packages: set[str]) -> tuple[set[str], set[str]]:
    """(packages to build, packages to build with their dependents) for a list of
    (kind, path) changes under ros2_ws/src, kind new_dir / new / changed / deleted.

    deploy/pi/pi's change_scope: edits to existing files other than C++ and
    build files are live through --symlink-install, and so are new .py files in
    a Python package's module directory and anything under test/. Every other
    new file and any deletion re-runs its package; headers and CMakeLists.txt
    also rebuild the packages depending on it.
    """
    new_dirs = {path for kind, path in changes if kind == "new_dir"}
    select, above = set(), set()
    for kind, path in changes:
        pkg, _, rest = path.partition("/")
        if not rest or pkg not in packages:
            continue
        name = path.rsplit("/", 1)[-1]
        header = name.endswith((".hpp", ".h")) or name == "CMakeLists.txt"
        if kind == "deleted":
            select.add(pkg)
        elif kind == "new_dir":
            if not path.startswith(f"{pkg}/test/"):
                select.add(pkg)
        elif kind == "new":
            if path.rsplit("/", 1)[0] in new_dirs:
                select.add(pkg)
            elif (path.startswith(f"{pkg}/{pkg}/") and name.endswith(".py")) \
                    or path.startswith(f"{pkg}/test/"):
                pass
            elif header:
                above.add(pkg)
            else:
                select.add(pkg)
        elif kind == "changed":
            if header:
                above.add(pkg)
            elif name.endswith((".cpp", ".cc", ".c", "plugins.xml")) \
                    or name in ("package.xml", "setup.py", "setup.cfg"):
                select.add(pkg)
    return select, above


# Export and sync ---------------------------------------------------------------

def export_tree(archive: bytes, skip_packages: set[str]) -> dict[str, tuple[str, int, bytes]]:
    """{path under ros2_ws/src: (kind, mode, data)} from a `git archive` tar."""
    entries = {}
    with tarfile.open(fileobj=io.BytesIO(archive), mode="r:") as tar:
        for member in tar:
            if not member.name.startswith(SOURCE_PREFIX):
                continue
            rel = member.name[len(SOURCE_PREFIX):].rstrip("/")
            if not rel or rel.startswith("/") or ".." in rel.split("/") or skipped(rel, skip_packages):
                continue
            if member.isdir():
                entries[rel] = ("dir", member.mode, b"")
            elif member.issym():
                entries[rel] = ("link", member.mode, member.linkname.encode())
            elif member.isfile():
                entries[rel] = ("file", member.mode, tar.extractfile(member).read())
    for rel in list(entries):
        parts = rel.split("/")
        for i in range(1, len(parts)):
            entries.setdefault("/".join(parts[:i]), ("dir", 0o755, b""))
    return entries


def sync_tree(entries: dict[str, tuple[str, int, bytes]], target: Path, skip_packages: set[str],
              owner: tuple[int, int] | None = None) -> list[tuple[str, str]]:
    """Make `target` hold `entries`, touching only what differs; the changes made.

    Written files get the current time (not the commit's), so make rebuilds
    them; unchanged ones keep theirs. With `owner`, what is written is handed to
    that uid/gid, so `pi sync` from the laptop can still overwrite it.
    """
    target = Path(target)
    changes = []

    def own(path: Path) -> None:
        if owner is not None:
            try:
                os.lchown(path, *owner)
            except OSError:
                pass

    for rel in sorted(entries, key=lambda r: (r.count("/"), r)):
        kind, mode, data = entries[rel]
        path = target / rel
        if kind == "dir":
            if path.is_symlink() or (path.exists() and not path.is_dir()):
                path.unlink()
            if not path.is_dir():
                path.mkdir(parents=True)
                own(path)
                changes.append(("new_dir", rel))
            continue
        existed = path.exists() or path.is_symlink()
        if path.is_dir() and not path.is_symlink():
            shutil.rmtree(path)
            existed = True
        if kind == "link":
            if path.is_symlink() and os.readlink(path) == data.decode():
                continue
            if existed:
                path.unlink()
            os.symlink(data.decode(), path)
        else:
            if path.is_file() and not path.is_symlink() and path.stat().st_size == len(data) \
                    and path.read_bytes() == data:
                if (path.stat().st_mode & 0o111) != (mode & 0o111):
                    path.chmod(mode & 0o7777)
                continue
            if path.is_symlink():
                path.unlink()
            tmp = path.with_name(f".{path.name}.deploy")
            tmp.write_bytes(data)
            tmp.chmod(mode & 0o7777 or 0o644)
            os.replace(tmp, path)
        own(path)
        changes.append(("changed" if existed else "new", rel))

    for root, dirs, files in os.walk(target, topdown=False):
        for name in files + dirs:
            path = Path(root, name)
            rel = path.relative_to(target).as_posix()
            if rel in entries or skipped(rel, skip_packages):
                continue
            if path.is_dir() and not path.is_symlink():
                if any(path.iterdir()):
                    continue  # holds skipped files only
                path.rmdir()
            else:
                path.unlink()
            changes.append(("deleted", rel))
    return changes


# Jobs ----------------------------------------------------------------------------

def _alive(pid) -> bool:
    try:
        os.kill(int(pid), 0)
        return True
    except (OSError, TypeError, ValueError):
        return False


def _panel_pids(exclude: int) -> list[int]:
    """Running control panels (ros2 run sandro_gui sandro_gui), by command line."""
    pids = []
    for entry in Path("/proc").glob("[0-9]*"):
        try:
            argv = (entry / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        if int(entry.name) != exclude and any(a.endswith(b"lib/sandro_gui/sandro_gui") for a in argv):
            pids.append(int(entry.name))
    return pids


def _git(binary: str, repo: Path, *args: str, timeout: float = GIT_TIMEOUT_S, raw: bool = False):
    if not binary:
        raise RuntimeError("git is not installed in the robot container (deploy/pi/Dockerfile)")
    result = subprocess.run([binary, "--git-dir", str(repo), *args], capture_output=True,
                            timeout=timeout)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.decode(errors="replace").strip() or f"git {args[0]} failed")
    return result.stdout if raw else result.stdout.decode(errors="replace")


class JobRunner:
    """One deploy, build or fetch: state in gui_runs/software-job.json, output in its log.

    On the robot it runs in its own process (python3 -m sandro_gui.software job FILE,
    started by the panel in a new session), so a panel that restarts or dies halfway
    does not take the build with it; the panel reads the state and the log back.
    """

    def __init__(self, state_path: Path, restart=None) -> None:
        self.state_path = Path(state_path)
        self.state = json.loads(self.state_path.read_text())
        cfg = self.state["config"]
        self.root = Path(cfg["root"])
        self.repo = Path(cfg["repo"])
        self.src = self.root / "ros2_ws" / "src"
        self.versions_dir = self.root / "version"
        self.log_path = Path(cfg["runs"]) / self.state["log"]
        self.remote = cfg.get("remote", "")
        self.skip = set(cfg.get("skip_packages", []))
        self.colcon = list(cfg.get("colcon", ["colcon"]))
        self.colcon_args = list(cfg.get("colcon_args", []))
        self.git = cfg.get("git") or shutil.which("git")
        self.restart = restart if restart is not None else self._restart_panels
        self.changed_what_runs = False

    def save(self, **changes) -> None:
        self.state.update(changes)
        tmp = self.state_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.state, indent=1))
        os.replace(tmp, self.state_path)

    def line(self, text: str) -> None:
        with open(self.log_path, "a") as stream:
            stream.write(text + "\n")

    def step(self, step: str, message: str = "") -> None:
        self.save(step=step, message=message)
        if message:
            self.line(message)

    def run(self) -> None:
        self.save(pid=os.getpid(), state="running")
        try:
            work = {"deploy": self.deploy, "build": self.build, "fetch": self.fetch}[self.state["kind"]]
            message, state = work(), "done"
        except Exception as error:  # noqa: BLE001 - reported on the page
            message, state = str(error), "failed"
        self.line(message)
        self.save(state=state, step=state, message=message, ended=time.time())
        if state == "done" and self.changed_what_runs and self.state.get("restart"):
            self.line("restarting the panel on the new build")
            time.sleep(1.0)
            self.restart()

    def _restart_panels(self) -> None:
        for pid in _panel_pids(os.getpid()):
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass

    # The work ---------------------------------------------------------------------

    def fetch(self) -> str:
        self.step("fetch", f"fetching {self.remote}")
        refs = lambda: _git(self.git, self.repo, "for-each-ref", "--format=%(refname) %(objectname)")
        before = refs()
        proc = subprocess.Popen(
            [self.git, "--git-dir", str(self.repo), "fetch", "--progress", "--prune", "--no-tags",
             "origin", "+refs/heads/*:refs/remotes/origin/*"],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, stdin=subprocess.DEVNULL)
        timer = threading.Timer(FETCH_TIMEOUT_S, proc.kill)
        timer.start()
        last = 0.0
        try:
            # git redraws its progress with \r: log finished phases, show the rest as the step.
            for line in _lines(proc.stderr):
                if line.endswith("done.") or not line.split(":")[0].endswith(("objects", "deltas")):
                    self.line(line)
                elif time.monotonic() - last > 1.0:
                    last = time.monotonic()
                    self.save(message=line)
            code = proc.wait()
        finally:
            timer.cancel()
        if code != 0:
            raise RuntimeError(f"git fetch failed (exit {code}): is the robot online? "
                               "deploy/pi/pi push works without internet")
        after = refs()
        old = dict(line.split() for line in before.splitlines())
        changed = [ref.rsplit("/", 1)[-1] for ref, sha in (l.split() for l in after.splitlines())
                   if old.get(ref) != sha]
        return "fetched: " + (", ".join(changed) + " updated" if changed else "nothing new")

    def deploy(self) -> str:
        sha = self.state["params"]["commit"]
        short, subject, date, author = (self.state["params"]["info"] + [""] * 4)[:4]
        previous_source = read_version(self.versions_dir / "source.json")
        previous_build = read_version(self.versions_dir / "build.json")
        was_built = same_version(previous_source, previous_build)
        self.step("export", f"exporting {short} {subject}")
        archive = _git(self.git, self.repo, "archive", "--format=tar", sha, "--", "ros2_ws/src",
                       raw=True, timeout=120)
        entries = export_tree(archive, self.skip)
        self.step("sync", f"writing the files that differ ({len(entries)} in the commit)")
        self.src.mkdir(parents=True, exist_ok=True)
        stat = self.src.stat()
        owner = (stat.st_uid, stat.st_gid) if hasattr(os, "geteuid") and os.geteuid() == 0 else None
        packages = {rel for rel, (kind, _, _) in entries.items()
                    if kind == "dir" and f"{rel}/package.xml" in entries}
        packages |= {p.name for p in self.src.iterdir() if (p / "package.xml").is_file()}
        changes = sync_tree(entries, self.src, self.skip, owner)
        version = {"commit": sha, "short": short, "describe": short, "subject": subject,
                   "date": date, "author": author, "dirty": False, "changes": [],
                   "via": "panel", "at": now_iso()}
        write_version(self.versions_dir / "source.json", version)
        if changes:
            shown = ", ".join(f"{kind} {rel}" for kind, rel in changes[:8])
            more = f" and {len(changes) - 8} more" if len(changes) > 8 else ""
            self.line(f"{len(changes)} change(s): {shown}{more}")
        select, above = build_scope(changes, packages)
        installed = (self.root / "install" / "setup.bash").is_file()
        if installed and not was_built:
            self.line(f"the build ({label(previous_build)}) does not match what was on disk "
                      f"({label(previous_source)}), an update stopped halfway: building the workspace")
        full = not installed or not was_built
        if not full and not select and not above:
            self._stamp_build(version, [])
            self.changed_what_runs = bool(changes)
            return (f"{short} deployed: " + ("live through --symlink-install, nothing to compile"
                                             if changes else "already on disk and built"))
        built = self._colcon(select, above, full)
        self._stamp_build(version, built)
        self.changed_what_runs = True
        return f"{short} deployed and built ({', '.join(built) if built else 'the workspace'})"

    def build(self) -> str:
        source = read_version(self.versions_dir / "source.json")
        if source is None:
            raise RuntimeError("version/source.json is missing: deploy a commit, or deploy/pi/pi sync")
        built = self._colcon(set(), set(), full=True)
        self._stamp_build(source, built)
        self.changed_what_runs = True
        return f"{label(source)} built"

    def _stamp_build(self, version: dict, packages: list[str]) -> None:
        write_version(self.versions_dir / "build.json",
                      dict(version, built_at=now_iso(), packages=packages or "all"))

    def _colcon_list(self, *args: str) -> list[str]:
        result = subprocess.run([*self.colcon, "list", "--base-paths", "ros2_ws/src", "--names-only",
                                 *args], cwd=self.root, capture_output=True, text=True, timeout=60)
        return result.stdout.split()

    def _colcon(self, select: set[str], above: set[str], full: bool) -> list[str]:
        packages: list[str] = []
        if not full:
            names = set(select)
            if above:
                names |= set(self._colcon_list("--packages-above", *sorted(above)))
            new = [p for p in names if not (self.root / "install" / p).is_dir()]
            if new:
                names |= set(self._colcon_list("--packages-up-to", *sorted(new)))
            packages = sorted(names)
        self.step("build", "building " + (", ".join(packages) if packages else "the workspace"))
        command = [*self.colcon, "build", "--base-paths", "ros2_ws/src", "--symlink-install"]
        if packages:
            command += ["--packages-select", *packages]
        command += self.colcon_args
        proc = subprocess.Popen(command, cwd=self.root, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        for raw in proc.stdout:
            text = raw.decode(errors="replace").rstrip()
            if text:
                self.line(text)
        code = proc.wait()
        if code != 0:
            raise RuntimeError(f"colcon build failed (exit {code}): the log above names the package")
        return packages


# The panel's side -------------------------------------------------------------------

class Software:
    """Versions, the robot's repository, and the jobs: what the Software page shows and starts."""

    def __init__(self, root: str | Path, log, launcher, cfg: dict | None = None,
                 restart=None, detach: bool = True) -> None:
        cfg = cfg or {}
        self.root = Path(root)
        self.log = log
        self.launcher = launcher
        self.repo = self.root / cfg.get("repo", "repo.git")
        self.versions_dir = self.root / "version"
        self.runs = self.root / cfg.get("runs_dir", "gui_runs")
        self.job_path = self.runs / "software-job.json"
        self.remote = cfg.get("remote", "")
        self.branch = cfg.get("branch", "main")
        self.restart = restart
        self.detach = detach
        self.git = shutil.which("git")
        self.config = {
            "root": str(self.root), "repo": str(self.repo), "runs": str(self.runs),
            "remote": self.remote, "skip_packages": list(cfg.get("skip_packages", ["sandro_sim"])),
            "colcon": list(cfg.get("colcon", ["colcon"])),
            "colcon_args": list(cfg.get("colcon_args", [
                "--cmake-args", "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF",
                "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache", "--event-handlers", "console_cohesion+"])),
            "git": self.git}
        self._cache: tuple[float, dict] | None = None
        self._stamp_cache: dict[str, tuple[float, dict | None]] = {}
        self._job_cache: tuple[int | None, dict | None] = (None, None)
        self.panel = self.build_version()
        self.boot = time.time()
        job = self.job()
        self._seen = (job.get("id"), job.get("state")) if job else None

    # Versions -------------------------------------------------------------------

    def _stamp(self, name: str) -> dict | None:
        path = self.versions_dir / f"{name}.json"
        try:
            mtime = path.stat().st_mtime
        except OSError:
            return None
        cached = self._stamp_cache.get(name)
        if cached is None or cached[0] != mtime:
            cached = self._stamp_cache[name] = (mtime, read_version(path))
        return cached[1]

    def source_version(self) -> dict | None:
        return self._stamp("source")

    def build_version(self) -> dict | None:
        return self._stamp("build")

    def job(self) -> dict | None:
        try:
            mtime = self.job_path.stat().st_mtime_ns
        except OSError:
            return None
        if self._job_cache[0] != mtime:
            try:
                job = json.loads(self.job_path.read_text())
            except (OSError, ValueError):
                return self._job_cache[1]
            job.pop("config", None)
            self._job_cache = (mtime, job)
        job = dict(self._job_cache[1])
        if job.get("state") == "running" and time.time() - job.get("started", 0) > 10 \
                and not _alive(job.get("pid")):
            job.update(state="failed", step="failed", message="interrupted: its process is gone")
        return job

    def summary(self) -> dict:
        """The cheap part, for every slow snapshot; notes a job's start and end in the log."""
        job = self.job()
        key = (job.get("id"), job.get("state")) if job else None
        if key != self._seen and job:
            level = {"failed": ERROR, "done": INFO}.get(job["state"], INFO)
            text = (f"{job['kind']} {job['target']} started" if job["state"] == "running"
                    else f"{job['kind']} {job['target']} {job['state']}: {job.get('message', '')}")
            self.log.add(level, "software", text, origin="software")
        self._seen = key
        return {"source": self.source_version(), "build": self.build_version(),
                "panel": self.panel, "boot": self.boot, "job": job,
                "repo": self.repo.is_dir(), "git": bool(self.git)}

    def tail(self, lines: int = 400) -> list[str]:
        job = self.job()
        if not job:
            return []
        try:
            with open(self.runs / job["log"], errors="replace") as stream:
                return [line.rstrip("\n") for line in deque(stream, maxlen=lines)]
        except OSError:
            return []

    # The repository ---------------------------------------------------------------

    def _ensure_repo(self) -> None:
        if not self.git:
            raise RuntimeError("git is not installed in the robot container (deploy/pi/Dockerfile)")
        if not self.repo.is_dir():
            subprocess.run([self.git, "init", "--bare", "-q", str(self.repo)], check=True,
                           capture_output=True, timeout=GIT_TIMEOUT_S)
        if self.remote:
            remotes = _git(self.git, self.repo, "remote").split()
            if "origin" not in remotes:
                _git(self.git, self.repo, "remote", "add", "origin", self.remote)
            elif _git(self.git, self.repo, "remote", "get-url", "origin").strip() != self.remote:
                _git(self.git, self.repo, "remote", "set-url", "origin", self.remote)

    def refs(self) -> list[dict]:
        out = _git(self.git, self.repo, "for-each-ref", "--sort=-committerdate",
                   "--format=%(refname)%00%(objectname)%00%(committerdate:unix)%00%(subject)",
                   "refs/heads", "refs/remotes/origin")
        refs = []
        for line in out.splitlines():
            name, sha, when, subject = (line.split("\0") + ["", "", "", ""])[:4]
            if name.endswith("/HEAD"):
                continue
            if name.startswith("refs/remotes/"):
                short, origin = name[len("refs/remotes/"):], "GitHub"
            else:
                short, origin = name[len("refs/heads/"):], "pushed from the laptop"
            refs.append({"ref": name, "name": short, "origin": origin, "commit": sha,
                         "time": float(when or 0), "subject": subject})
        return refs

    def commits(self, ref: str, limit: int = 15) -> list[dict]:
        out = _git(self.git, self.repo, "log", f"-n{limit}",
                   "--format=%H%x00%h%x00%ct%x00%an%x00%s", ref, "--")
        rows = []
        for line in out.splitlines():
            sha, short, when, author, subject = (line.split("\0") + [""] * 5)[:5]
            rows.append({"commit": sha, "short": short, "time": float(when or 0),
                         "author": author, "subject": subject})
        return rows

    def _known(self, sha: str) -> bool:
        try:
            _git(self.git, self.repo, "cat-file", "-e", f"{sha}^{{commit}}")
            return True
        except RuntimeError:
            return False

    def status(self, ref: str | None = None) -> dict:
        """Everything the Software page shows; git answers are cached briefly."""
        result = self.summary()
        result.update(remote=self.remote, branch=self.branch, tail=self.tail())
        if not self.git or not self.repo.is_dir():
            return dict(result, refs=[], commits=[], ref=None)
        cached = self._cache
        key = ref or ""
        if cached is None or time.monotonic() - cached[0] > CACHE_S or cached[1].get("key") != key:
            try:
                refs = self.refs()
                chosen = ref or next((r["ref"] for r in refs if r["name"] in
                                      (f"origin/{self.branch}", self.branch)),
                                     refs[0]["ref"] if refs else None)
                commits = self.commits(chosen) if chosen else []
                source = (self.source_version() or {}).get("commit")
                behind = None
                if chosen and source and self._known(source):
                    behind = int(_git(self.git, self.repo, "rev-list", "--count", "--first-parent",
                                      f"{source}..{chosen}").strip())
                git = {"key": key, "refs": refs, "ref": chosen, "commits": commits,
                       "behind": behind, "error": None}
            except (RuntimeError, subprocess.TimeoutExpired) as error:
                git = {"key": key, "refs": [], "ref": None, "commits": [], "behind": None,
                       "error": str(error)}
            self._cache = cached = (time.monotonic(), git)
        return dict(result, **{k: v for k, v in cached[1].items() if k != "key"})

    # Starting jobs ------------------------------------------------------------------

    def _guard(self, stack_stopped: bool = True) -> None:
        job = self.job()
        if job and job["state"] == "running":
            raise RuntimeError(f"{job['kind']} {job['target']} is still running")
        state = self.launcher.status().get("state") if self.launcher else "stopped"
        if stack_stopped and state != "stopped":
            raise RuntimeError("stop the robot stack first: a build under a running stack can "
                               "crash it, and its CPU load disturbs the control loop")

    def _start(self, kind: str, target: str, params: dict | None = None,
               stack_stopped: bool = True) -> dict:
        self._guard(stack_stopped)
        self.runs.mkdir(parents=True, exist_ok=True)
        state = {"id": int(time.time() * 1000), "kind": kind, "target": target,
                 "state": "running", "step": "starting", "message": "", "started": time.time(),
                 "ended": None, "log": f"{kind}-{time.strftime('%Y%m%d-%H%M%S')}.log", "pid": None,
                 "restart": self.restart is not None, "params": params or {},
                 "config": self.config}
        tmp = self.job_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(state, indent=1))
        os.replace(tmp, self.job_path)
        if self.detach:
            with open(self.runs / state["log"], "ab") as log:
                proc = subprocess.Popen(
                    [sys.executable, "-m", "sandro_gui.software", "job", str(self.job_path)],
                    cwd=self.root, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                    start_new_session=True, env=dict(os.environ, PYTHONUNBUFFERED="1"))
            state["pid"] = proc.pid
        else:
            runner = JobRunner(self.job_path, restart=self.restart or (lambda: None))
            threading.Thread(target=runner.run, name=kind, daemon=True).start()
        self._cache = None
        return self.job() or state

    def fetch(self) -> dict:
        """The first fetch of a fresh repository takes minutes; the stack may run meanwhile."""
        if not self.remote:
            raise RuntimeError("no GitHub remote configured (gui.yaml software.remote)")
        self._ensure_repo()
        return {"job": self._start("fetch", "GitHub", stack_stopped=False)}

    def deploy(self, ref: str) -> dict:
        if not ref:
            raise ValueError("deploy needs a commit or branch")
        self._ensure_repo()
        sha = _git(self.git, self.repo, "rev-parse", "--verify", f"{ref}^{{commit}}").strip()
        info = _git(self.git, self.repo, "log", "-1", "--format=%h%x00%s%x00%cI%x00%an",
                    sha).strip().split("\0")
        return {"job": self._start("deploy", info[0], {"commit": sha, "info": info})}

    def build(self) -> dict:
        return {"job": self._start("build", label(self.source_version()))}


def _lines(stream):
    """Lines of a byte stream split at \\n and \\r, decoded and stripped."""
    pending = b""
    while True:
        chunk = stream.read1(4096) if hasattr(stream, "read1") else stream.read(4096)
        if not chunk:
            break
        pending += chunk
        parts = pending.replace(b"\r", b"\n").split(b"\n")
        pending = parts.pop()
        for part in parts:
            text = part.decode(errors="replace").strip()
            if text:
                yield text
    if pending.strip():
        yield pending.decode(errors="replace").strip()


def check(software: dict | None, launch: dict | None) -> tuple[str, str, str, str]:
    """(level, value, detail, short) of the Software pre-flight row."""
    if not software:
        return "off", "unknown", "", "?"
    source, build, panel, job = (software.get(k) for k in ("source", "build", "panel", "job"))
    value = label(build or source)
    if job and job.get("state") == "running":
        return "warn", f"{job['kind']} {job['target']}", job.get("message", ""), "busy"
    if not source and not build:
        return ("off", "unknown", "no version stamp yet: deploy from the Software page, or "
                "deploy/pi/pi sync and build", "?")
    if job and job.get("state") == "failed":
        return "error", value, f"the last {job['kind']} failed: {job.get('message', '')}", value
    notes = []
    level = "ok"
    if not same_version(source, build):
        level = "warn"
        notes.append(f"source on disk is {label(source)}, the build {label(build)}: build it")
    elif not same_version(panel, build):
        level = "warn"
        notes.append(f"the panel runs {label(panel)}: restart it (Software page)")
    run = (launch or {}).get("run") or {}
    if run.get("version") and build and not same_version(run["version"], build):
        level = "warn"
        notes.append(f"the stack was launched from {label(run['version'])}: restart it to run "
                     f"{label(build)}")
    current = build or source
    if current.get("dirty"):
        notes.append(f"uncommitted changes synced at {current.get('at', '?')}")
    elif current.get("subject"):
        notes.insert(0, current["subject"])
    return level, value, " · ".join(notes), value


def main(argv: list[str]) -> int:
    if len(argv) >= 2 and argv[0] == "job":
        JobRunner(Path(argv[1])).run()
        return 0
    if len(argv) >= 1 and argv[0] == "stamp-build":
        root = Path(argv[1] if len(argv) > 1 else ".")
        source = read_version(root / "version" / "source.json")
        if source is None:
            print("no version/source.json: nothing stamped", file=sys.stderr)
            return 0
        write_version(root / "version" / "build.json", dict(source, built_at=now_iso()))
        print(f"build version {label(source)}")
        return 0
    print(__doc__.strip().splitlines()[-1], file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
