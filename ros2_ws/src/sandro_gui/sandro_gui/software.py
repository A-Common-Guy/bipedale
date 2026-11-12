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

Also a script, for deploy/pi/pi (no ROS environment needed):

    python3 software.py stamp-build [ROOT]   version/build.json from source.json
"""

from __future__ import annotations

import io
import json
import os
import shutil
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


# The software service ------------------------------------------------------------

class Software:
    def __init__(self, root: str | Path, log, launcher, cfg: dict | None = None,
                 restart=None) -> None:
        cfg = cfg or {}
        self.root = Path(root)
        self.log = log
        self.launcher = launcher
        self.repo = self.root / cfg.get("repo", "repo.git")
        self.src = self.root / "ros2_ws" / "src"
        self.versions_dir = self.root / "version"
        self.runs = self.root / cfg.get("runs_dir", "gui_runs")
        self.remote = cfg.get("remote", "")
        self.branch = cfg.get("branch", "main")
        self.skip = set(cfg.get("skip_packages", ["sandro_sim"]))
        self.colcon_args = list(cfg.get("colcon_args", [
            "--cmake-args", "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF",
            "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache", "--event-handlers", "console_cohesion+"]))
        self.restart = restart
        self.git = shutil.which("git")
        self._lock = threading.Lock()
        self._job: dict | None = None
        self._tail: deque[str] = deque(maxlen=400)
        self._cache: tuple[float, dict] | None = None
        self._stamp_cache: dict[str, tuple[float, dict | None]] = {}
        self._job_seq = 0
        self._restart_after = False
        self.panel = self.build_version()
        self.boot = time.time()

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

    def summary(self) -> dict:
        """The cheap part, for every slow snapshot."""
        with self._lock:
            job = dict(self._job) if self._job else None
        return {"source": self.source_version(), "build": self.build_version(),
                "panel": self.panel, "boot": self.boot, "job": job,
                "repo": self.repo.is_dir(), "git": bool(self.git)}

    # Git ------------------------------------------------------------------------

    def _git(self, *args: str, timeout: float = GIT_TIMEOUT_S, binary: bool = False):
        if not self.git:
            raise RuntimeError("git is not installed in the robot container (deploy/pi/Dockerfile)")
        result = subprocess.run([self.git, "--git-dir", str(self.repo), *args],
                                capture_output=True, timeout=timeout)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.decode(errors="replace").strip()
                               or f"git {args[0]} failed")
        return result.stdout if binary else result.stdout.decode(errors="replace")

    def _ensure_repo(self) -> None:
        if not self.git:
            raise RuntimeError("git is not installed in the robot container (deploy/pi/Dockerfile)")
        if not self.repo.is_dir():
            subprocess.run([self.git, "init", "--bare", "-q", str(self.repo)], check=True,
                           capture_output=True, timeout=GIT_TIMEOUT_S)
        if self.remote:
            remotes = self._git("remote").split()
            if "origin" not in remotes:
                self._git("remote", "add", "origin", self.remote)
            elif self._git("remote", "get-url", "origin").strip() != self.remote:
                self._git("remote", "set-url", "origin", self.remote)

    def refs(self) -> list[dict]:
        out = self._git("for-each-ref", "--sort=-committerdate",
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
        out = self._git("log", f"-n{limit}", "--format=%H%x00%h%x00%ct%x00%an%x00%s", ref, "--")
        rows = []
        for line in out.splitlines():
            sha, short, when, author, subject = (line.split("\0") + [""] * 5)[:5]
            rows.append({"commit": sha, "short": short, "time": float(when or 0),
                         "author": author, "subject": subject})
        return rows

    def _known(self, sha: str) -> bool:
        try:
            self._git("cat-file", "-e", f"{sha}^{{commit}}")
            return True
        except RuntimeError:
            return False

    def status(self, ref: str | None = None) -> dict:
        """Everything the Software page shows; git answers are cached briefly."""
        result = self.summary()
        result.update(remote=self.remote, branch=self.branch, tail=list(self._tail))
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
                    behind = int(self._git("rev-list", "--count", "--first-parent",
                                           f"{source}..{chosen}").strip())
                git = {"key": key, "refs": refs, "ref": chosen, "commits": commits,
                       "behind": behind, "error": None}
            except (RuntimeError, subprocess.TimeoutExpired) as error:
                git = {"key": key, "refs": [], "ref": None, "commits": [], "behind": None,
                       "error": str(error)}
            self._cache = cached = (time.monotonic(), git)
        return dict(result, **{k: v for k, v in cached[1].items() if k != "key"})

    # Jobs -----------------------------------------------------------------------

    def _guard(self, stack_stopped: bool = True) -> None:
        with self._lock:
            if self._job and self._job["state"] == "running":
                raise RuntimeError(f"{self._job['kind']} {self._job['target']} is still running")
        state = self.launcher.status().get("state") if self.launcher else "stopped"
        if stack_stopped and state != "stopped":
            raise RuntimeError("stop the robot stack first: a build under a running stack can "
                               "crash it, and its CPU load disturbs the control loop")

    def _start(self, kind: str, target: str, work, *args, stack_stopped: bool = True) -> dict:
        self._guard(stack_stopped)
        self.runs.mkdir(parents=True, exist_ok=True)
        with self._lock:
            self._job_seq += 1
            log_name = f"{kind}-{time.strftime('%Y%m%d-%H%M%S')}.log"
            self._job = {"id": self._job_seq, "kind": kind, "target": target, "state": "running",
                         "step": "starting", "message": "", "started": time.time(),
                         "ended": None, "log": log_name}
            self._tail.clear()
            job = dict(self._job)
        threading.Thread(target=self._run, args=(work, args), name=kind, daemon=True).start()
        return job

    def _step(self, step: str, message: str = "") -> None:
        with self._lock:
            if self._job:
                self._job.update(step=step, message=message)
        if message:
            self._line(message, INFO, "software")

    def _line(self, text: str, level: int = INFO, source: str = "colcon") -> None:
        self._tail.append(text)
        with self._lock:
            name = self._job["log"] if self._job else None
        if name:
            with open(self.runs / name, "a") as stream:
                stream.write(text + "\n")
        self.log.add(level, source, text, origin="software")

    def _run(self, work, args) -> None:
        try:
            message = work(*args)
            state, level = "done", INFO
        except Exception as error:  # noqa: BLE001 - reported on the page
            message, state, level = str(error), "failed", ERROR
        with self._lock:
            if self._job:
                self._job.update(state=state, step=state, message=message, ended=time.time())
        self._cache = None
        self._line(message, level, "software")
        if state == "done" and self.restart is not None and getattr(self, "_restart_after", False):
            self._restart_after = False
            self._line("restarting the panel on the new build", WARN, "software")
            threading.Timer(1.5, self.restart).start()

    def job(self) -> dict | None:
        with self._lock:
            return dict(self._job) if self._job else None

    # Actions --------------------------------------------------------------------

    def fetch(self) -> dict:
        """A job: the first fetch of a fresh repository takes minutes; the stack may run."""
        if not self.remote:
            raise RuntimeError("no GitHub remote configured (gui.yaml software.remote)")
        self._ensure_repo()
        return {"job": self._start("fetch", "GitHub", self._fetch, stack_stopped=False)}

    def _fetch(self) -> str:
        self._step("fetch", f"fetching {self.remote}")
        before = {r["ref"]: r["commit"] for r in self.refs()}
        proc = subprocess.Popen(
            [self.git, "--git-dir", str(self.repo), "fetch", "--progress", "--prune", "--no-tags",
             "origin", "+refs/heads/*:refs/remotes/origin/*"],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, stdin=subprocess.DEVNULL,
            start_new_session=True)
        timer = threading.Timer(FETCH_TIMEOUT_S, proc.kill)
        timer.start()
        last = 0.0
        try:
            # git redraws its progress with \r: log finished phases, show the rest as the step.
            for line in _lines(proc.stderr):
                if line.endswith("done.") or not line.split(":")[0].endswith(("objects", "deltas")):
                    self._line(line, INFO, "git")
                elif time.monotonic() - last > 1.0:
                    last = time.monotonic()
                    self._step("fetch", line)
            code = proc.wait()
        finally:
            timer.cancel()
        if code != 0:
            raise RuntimeError(f"git fetch failed (exit {code}): is the robot online? "
                               "deploy/pi/pi push works without internet")
        self._cache = None
        after = {r["ref"]: r["commit"] for r in self.refs()}
        changed = [ref.rsplit("/", 1)[-1] for ref, sha in after.items() if before.get(ref) != sha]
        return "fetched: " + (", ".join(changed) + " updated" if changed else "nothing new")

    def deploy(self, ref: str) -> dict:
        if not ref:
            raise ValueError("deploy needs a commit or branch")
        self._ensure_repo()
        sha = self._git("rev-parse", "--verify", f"{ref}^{{commit}}").strip()
        info = self._git("log", "-1", "--format=%h%x00%s%x00%cI%x00%an", sha).strip().split("\0")
        return self._start("deploy", info[0], self._deploy, sha, info)

    def build(self) -> dict:
        source = self.source_version()
        return self._start("build", label(source), self._build_synced)

    def _deploy(self, sha: str, info: list[str]) -> str:
        short, subject, date, author = (info + [""] * 4)[:4]
        self._step("export", f"exporting {short} {subject}")
        archive = self._git("archive", "--format=tar", sha, "--", "ros2_ws/src", binary=True,
                            timeout=120)
        entries = export_tree(archive, self.skip)
        self._step("sync", f"writing the files that differ ({len(entries)} in the commit)")
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
            self._line(f"{len(changes)} change(s): {shown}{more}", INFO, "software")
        select, above = build_scope(changes, packages)
        full = not (self.root / "install" / "setup.bash").is_file()
        if not full and not select and not above:
            self._stamp_build(version, [])
            self._restart_after = bool(changes)
            return (f"{short} deployed: " + ("live through --symlink-install, nothing to compile"
                                             if changes else "already on disk"))
        built = self._colcon(select, above, full)
        self._stamp_build(version, built)
        self._restart_after = True
        return f"{short} deployed and built ({', '.join(built) if built else 'everything'})"

    def _build_synced(self) -> str:
        source = self.source_version()
        if source is None:
            raise RuntimeError("version/source.json is missing: deploy a commit, or deploy/pi/pi sync")
        built = self._colcon(set(), set(), full=True)
        self._stamp_build(source, built)
        self._restart_after = True
        return f"{label(source)} built"

    def _stamp_build(self, version: dict, packages: list[str]) -> None:
        write_version(self.versions_dir / "build.json",
                      dict(version, built_at=now_iso(), packages=packages or "all"))

    def _colcon_list(self, *args: str) -> list[str]:
        result = subprocess.run(["colcon", "list", "--base-paths", "ros2_ws/src", "--names-only",
                                 *args], cwd=self.root, capture_output=True, text=True,
                                timeout=60)
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
        self._step("build", "building " + (", ".join(packages) if packages else "the workspace"))
        command = ["colcon", "build", "--base-paths", "ros2_ws/src", "--symlink-install"]
        if packages:
            command += ["--packages-select", *packages]
        command += self.colcon_args
        code = self._stream(command)
        if code != 0:
            raise RuntimeError(f"colcon build failed (exit {code}): the log above names the package")
        return packages

    def _stream(self, command: list[str]) -> int:
        proc = subprocess.Popen(command, cwd=self.root, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True)
        for raw in proc.stdout:
            text = raw.decode(errors="replace").rstrip()
            if text:
                failed = "Failed" in text or "error:" in text or "Error" in text
                self._line(text, ERROR if failed else INFO)
        return proc.wait()


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
