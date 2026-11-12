import os
import shutil
import subprocess
import time
from pathlib import Path

import pytest

from sandro_gui import software
from sandro_gui.logbook import Logbook
from sandro_gui.software import Software, build_scope, check, same_version, sync_tree

GIT = shutil.which("git")
needs_git = pytest.mark.skipif(GIT is None, reason="git not installed")


def test_build_scope_follows_the_pi_scripts_rules():
    packages = {"sandro_gui", "sandro_controllers"}
    assert build_scope([("changed", "sandro_gui/sandro_gui/panel.py"),
                        ("changed", "sandro_gui/web/run.js"),
                        ("new", "sandro_gui/sandro_gui/software.py"),
                        ("new", "sandro_gui/test/test_software.py")], packages) == (set(), set())
    assert build_scope([("changed", "sandro_controllers/src/rl_policy_core.cpp")], packages) \
        == ({"sandro_controllers"}, set())
    assert build_scope([("changed", "sandro_controllers/include/sandro_controllers/x.hpp")],
                       packages) == (set(), {"sandro_controllers"})
    assert build_scope([("new", "sandro_controllers/policies/new.bin")], packages) \
        == ({"sandro_controllers"}, set())
    assert build_scope([("deleted", "sandro_gui/web/old.js")], packages) == ({"sandro_gui"}, set())
    # A new directory's files re-run the package, a new test directory does not.
    assert build_scope([("new_dir", "sandro_gui/sandro_gui/sub"),
                        ("new", "sandro_gui/sandro_gui/sub/a.py")], packages) == ({"sandro_gui"}, set())
    assert build_scope([("new_dir", "sandro_gui/test/data")], packages) == (set(), set())
    assert build_scope([("changed", "README.md"), ("changed", "other/x.cpp")], packages) == (set(), set())


def test_sync_writes_only_what_differs(tmp_path):
    target = tmp_path / "src"
    (target / "pkg" / "__pycache__").mkdir(parents=True)
    (target / "pkg" / "same.py").write_text("same")
    (target / "pkg" / "edit.py").write_text("old")
    (target / "pkg" / "gone.py").write_text("gone")
    (target / "pkg" / "__pycache__" / "x.pyc").write_bytes(b"cache")
    old = time.time() - 1000
    os.utime(target / "pkg" / "same.py", (old, old))
    entries = {"pkg": ("dir", 0o755, b""),
               "pkg/same.py": ("file", 0o644, b"same"),
               "pkg/edit.py": ("file", 0o644, b"new"),
               "pkg/tool.sh": ("file", 0o755, b"#!/bin/sh\n"),
               "pkg/sub": ("dir", 0o755, b""),
               "pkg/sub/a.txt": ("file", 0o644, b"a")}
    changes = sync_tree(entries, target, {"sandro_sim"})
    assert sorted(changes) == sorted([("changed", "pkg/edit.py"), ("new", "pkg/tool.sh"),
                                      ("new_dir", "pkg/sub"), ("new", "pkg/sub/a.txt"),
                                      ("deleted", "pkg/gone.py")])
    assert (target / "pkg" / "same.py").stat().st_mtime == pytest.approx(old)
    assert (target / "pkg" / "edit.py").read_text() == "new"
    assert os.access(target / "pkg" / "tool.sh", os.X_OK)
    assert (target / "pkg" / "__pycache__" / "x.pyc").exists()  # skipped, left alone
    assert sync_tree(entries, target, {"sandro_sim"}) == []


def test_same_version_tells_syncs_of_uncommitted_changes_apart():
    a = {"commit": "abc", "dirty": False}
    assert same_version(a, dict(a, built_at="x"))
    assert not same_version(a, {"commit": "abd"})
    dirty = {"commit": "abc", "dirty": True, "at": "t1"}
    assert same_version(dirty, dict(dirty))
    assert not same_version(dirty, dict(dirty, at="t2"))
    assert not same_version(a, None)


def test_check_names_what_to_do():
    v = {"commit": "a" * 40, "short": "aaaaaaa", "dirty": False, "subject": "fix"}
    level, value, detail, _ = check({"source": v, "build": v, "panel": v}, {})
    assert (level, value) == ("ok", "aaaaaaa") and detail == "fix"
    w = dict(v, commit="b" * 40, short="bbbbbbb")
    assert "build it" in check({"source": w, "build": v, "panel": v}, {})[2]
    assert "restart it" in check({"source": v, "build": v, "panel": w}, {})[2]
    stack = check({"source": v, "build": v, "panel": v}, {"run": {"version": w}})
    assert stack[0] == "warn" and "stack was launched from bbbbbbb" in stack[2]
    assert check({"source": None, "build": None}, {})[0] == "off"
    failed = check({"source": v, "build": v, "panel": v,
                    "job": {"state": "failed", "kind": "deploy", "message": "boom"}}, {})
    assert failed[0] == "error" and "boom" in failed[2]


class Launcher:
    def __init__(self, state="stopped"):
        self.state = state

    def status(self):
        return {"state": self.state}


def git(cwd, *args):
    subprocess.run([GIT, "-C", str(cwd), *args], check=True, capture_output=True,
                   env=dict(os.environ, GIT_AUTHOR_NAME="t", GIT_AUTHOR_EMAIL="t@t",
                            GIT_COMMITTER_NAME="t", GIT_COMMITTER_EMAIL="t@t"))


def wait(service, timeout=10.0):
    end = time.time() + timeout
    while time.time() < end:
        job = service.job()
        if job and job["state"] != "running":
            return job
        time.sleep(0.05)
    raise AssertionError("job did not finish")


FAKE_COLCON = """#!/bin/sh
echo "$@" >> "$(dirname "$0")/colcon_calls"
if [ "$1" = list ]; then echo sandro_controllers; exit 0; fi
echo "Starting >>> sandro_controllers"; echo "Finished <<< sandro_controllers"
"""


def fake_colcon(tmp_path):
    script = tmp_path / "colcon"
    script.write_text(FAKE_COLCON)
    script.chmod(0o755)
    calls = tmp_path / "colcon_calls"
    return script, lambda: [line.split() for line in calls.read_text().splitlines()
                            if line.startswith("build")] if calls.exists() else []


def robot_with_commit(tmp_path):
    work = tmp_path / "work"
    pkg = work / "ros2_ws" / "src" / "sandro_controllers"
    (pkg / "src").mkdir(parents=True)
    (pkg / "package.xml").write_text("<package/>")
    (pkg / "src" / "core.cpp").write_text("int a;")
    (pkg / "sandro_controllers").mkdir()
    (pkg / "sandro_controllers" / "tools.py").write_text("x = 0")
    (work / "ros2_ws" / "src" / "sandro_sim").mkdir()
    (work / "ros2_ws" / "src" / "sandro_sim" / "package.xml").write_text("<package/>")
    (work / "README.md").write_text("outside the source tree")
    git(tmp_path, "init", "-q", str(work))
    git(work, "add", "-A")
    git(work, "commit", "-qm", "first")
    root = tmp_path / "robot"
    (root / "install").mkdir(parents=True)
    (root / "install" / "setup.bash").write_text("")
    (root / "install" / "sandro_controllers").mkdir()
    return work, pkg, root


@needs_git
def test_deploy_writes_the_commit_builds_what_changed_and_restarts(tmp_path):
    work, pkg, root = robot_with_commit(tmp_path)
    colcon, builds = fake_colcon(tmp_path)
    restarts = []
    service = Software(root, Logbook(), Launcher(), {"remote": "", "colcon": [str(colcon)]},
                       restart=lambda: restarts.append(1), detach=False)
    git(work, "push", "-q", _bare(service), "HEAD:refs/heads/main")

    service.deploy("main")
    job = wait(service)
    assert job["state"] == "done", job["message"]
    src = root / "ros2_ws" / "src"
    assert (src / "sandro_controllers" / "src" / "core.cpp").read_text() == "int a;"
    assert not (src / "sandro_sim").exists() and not (root / "ros2_ws" / "README.md").exists()
    # The build state was unknown before the first deploy: the whole workspace.
    assert builds() and "--packages-select" not in builds()[0]
    source, build = service.source_version(), service.build_version()
    assert source["subject"] == "first" and same_version(source, build)
    time.sleep(1.5)
    assert restarts == [1]
    assert "colcon" in " ".join(service.tail()) or "Finished" in " ".join(service.tail())

    # The same commit again, built: nothing written, nothing built, no restart.
    service.deploy("main")
    assert "already on disk and built" in wait(service)["message"] and len(builds()) == 1

    # An edit to a module is live through --symlink-install: written, not built.
    (pkg / "sandro_controllers" / "tools.py").write_text("x = 1")
    git(work, "commit", "-qam", "python only")
    git(work, "push", "-q", str(service.repo), "HEAD:refs/heads/main")
    service.deploy("main")
    assert "nothing to compile" in wait(service)["message"] and len(builds()) == 1
    assert (src / "sandro_controllers" / "sandro_controllers" / "tools.py").read_text() == "x = 1"
    status = service.status()
    assert status["commits"][0]["subject"] == "python only" and status["behind"] == 0

    # A C++ edit builds just that package.
    (pkg / "src" / "core.cpp").write_text("int b;")
    git(work, "commit", "-qam", "cpp")
    git(work, "push", "-q", str(service.repo), "HEAD:refs/heads/main")
    service.deploy("main")
    assert wait(service)["state"] == "done"
    assert builds()[-1][builds()[-1].index("--packages-select") + 1] == "sandro_controllers"


@needs_git
def test_a_deploy_after_an_interrupted_build_builds_the_workspace(tmp_path):
    work, _, root = robot_with_commit(tmp_path)
    colcon, builds = fake_colcon(tmp_path)
    service = Software(root, Logbook(), Launcher(), {"remote": "", "colcon": [str(colcon)]},
                       detach=False)
    git(work, "push", "-q", _bare(service), "HEAD:refs/heads/main")
    service.deploy("main")
    wait(service)
    # The files of the commit are on disk, but install/ is from an older build.
    software.write_version(root / "version" / "build.json", {"commit": "0" * 40, "short": "0000000"})
    service.deploy("main")
    job = wait(service)
    assert job["state"] == "done" and "built" in job["message"], job
    assert len(builds()) == 2 and "--packages-select" not in builds()[1]
    assert any("stopped halfway" in line for line in service.tail())
    assert same_version(service.source_version(), service.build_version())


@needs_git
def test_a_detached_deploy_outlives_the_panel_and_reports_back(tmp_path, monkeypatch):
    work, _, root = robot_with_commit(tmp_path)
    colcon, builds = fake_colcon(tmp_path)
    monkeypatch.setenv("PYTHONPATH", str(Path(software.__file__).resolve().parents[1]))
    panel = Software(root, Logbook(), Launcher(), {"remote": "", "colcon": [str(colcon)]})
    git(work, "push", "-q", _bare(panel), "HEAD:refs/heads/main")
    started = panel.deploy("main")["job"]
    assert started["state"] == "running"
    # A new panel process (after a restart) sees the same job from its files.
    later = Software(root, Logbook(), Launcher(), {"remote": "", "colcon": [str(colcon)]})
    job = wait(later, timeout=30)
    assert job["state"] == "done", (job, later.tail())
    assert job["id"] == started["id"] and builds()
    assert same_version(later.source_version(), later.build_version())


def _bare(service):
    subprocess.run([GIT, "init", "--bare", "-q", str(service.repo)], check=True)
    return str(service.repo)


@needs_git
def test_fetch_is_a_job_that_runs_beside_the_stack(tmp_path):
    upstream = tmp_path / "upstream"
    (upstream / "ros2_ws" / "src").mkdir(parents=True)
    (upstream / "ros2_ws" / "src" / "a.txt").write_text("a")
    git(tmp_path, "init", "-q", "-b", "main", str(upstream))
    git(upstream, "add", "-A")
    git(upstream, "commit", "-qm", "upstream commit")
    service = Software(tmp_path / "robot", Logbook(), Launcher("running"), {"remote": str(upstream)},
                       detach=False)
    service.fetch()
    job = wait(service)
    assert job["state"] == "done" and "main updated" in job["message"], job
    status = service.status()
    assert [r["name"] for r in status["refs"]] == ["origin/main"]
    assert status["commits"][0]["subject"] == "upstream commit"
    service.fetch()
    assert wait(service)["message"] == "fetched: nothing new"


@needs_git
def test_deploy_is_refused_while_the_stack_runs(tmp_path):
    service = Software(tmp_path, Logbook(), Launcher("running"), {"remote": ""})
    _bare(service)
    with pytest.raises(RuntimeError, match="stop the robot stack first"):
        service._guard()


def test_the_panel_remembers_the_build_it_started_from(tmp_path):
    software.write_version(tmp_path / "version" / "build.json", {"commit": "d" * 40, "short": "ddddddd"})
    service = Software(tmp_path, Logbook(), Launcher(), {"remote": ""})
    software.write_version(tmp_path / "version" / "build.json", {"commit": "e" * 40, "short": "eeeeeee"})
    summary = service.summary()
    assert summary["panel"]["short"] == "ddddddd"
    assert summary["build"]["short"] in ("ddddddd", "eeeeeee")  # mtime resolution


def test_stamp_build_script(tmp_path):
    software.write_version(tmp_path / "version" / "source.json",
                           {"commit": "c" * 40, "short": "ccccccc"})
    assert software.main(["stamp-build", str(tmp_path)]) == 0
    assert software.read_version(tmp_path / "version" / "build.json")["short"] == "ccccccc"
