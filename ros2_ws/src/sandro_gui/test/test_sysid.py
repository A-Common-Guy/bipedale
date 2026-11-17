"""The Sysid page's jobs: fixed commands, the checks before a start, a run's life and its files."""

import io
import json
import sys
import tarfile
import time
from pathlib import Path

import pytest

from sandro_gui import logbook, sysid
from sandro_gui.errors import Busy

BRINGUP = Path(__file__).resolve().parents[2] / "sandro_bringup"
RUNNING = {"state": "running"}


def ros(name="whole_body_controller", engaged=1.0, fault=None):
    return {"manager": True,
            "controllers": [{"name": "imu_broadcaster", "owns_commands": False},
                            {"name": name, "owns_commands": True, "fault": fault}],
            "diagnostics": {name: {"values": {"engaged": engaged}}}}


def test_the_commands_are_fixed_and_the_tools_exist():
    tools = BRINGUP / "tools"
    yaw = sysid.command("yaw", Path("/x/yaw_1"), tools, python="python3")
    assert yaw[2] == str(tools / "sysid_yaw.py") and Path(yaw[2]).is_file()
    # 2-45 Hz at 0.2 and 0.4 Nm per wheel, the stock leg stiffness, no prompt.
    assert yaw[3:] == ["--kp", "12", "--pulses", "0", "--chirp-nm", "0.2", "0.4", "--f0", "2",
                       "--f1", "45", "--chirp-s", "30", "--settle", "3", "--yes", "--out", "/x/yaw_1"]
    buzz = sysid.command("buzz", Path("/x/b"), tools, "rl_flat_level_sysid", python="python3")
    assert Path(buzz[2]).is_file() and buzz[3:5] == ["--label", "rl_flat_level_sysid"]
    assert "--yes" in buzz
    # Both stay inside what sysid_yaw.py itself refuses (0.5 Nm per wheel, kp 6-25).
    assert max(float(v) for v in yaw[yaw.index("--chirp-nm") + 1:yaw.index("--f0")]) <= 0.5


def test_a_measurement_starts_only_on_the_state_it_needs():
    assert "stack is not running" in sysid.blocked("yaw", ros(), {"state": "stopped"})
    assert "controller manager" in sysid.blocked("yaw", {"manager": False}, RUNNING)
    idle = {"manager": True, "controllers": [{"name": "imu_broadcaster", "owns_commands": False}]}
    assert "no controller is running" in sysid.blocked("buzz", idle, RUNNING)
    # The sweep is the whole-body controller's: a policy cannot make it.
    assert "needs whole_body_controller" in sysid.blocked("yaw", ros("rl_agile_terrain"), RUNNING)
    assert "not balancing" in sysid.blocked("yaw", ros(engaged=0.0), RUNNING)
    fallen = ros("rl_flat_level_sysid", fault={"state": "fallen", "text": "x"})
    assert "fallen" in sysid.blocked("buzz", fallen, RUNNING)
    assert sysid.blocked("yaw", ros(), RUNNING) is None
    assert sysid.blocked("buzz", ros("rl_flat_level_sysid"), RUNNING) is None
    assert sysid.blocked("buzz", ros(), {"state": "external"}) is None


def fake(seconds=0.2, code=0):
    script = ("import json, sys, time\n"
              "print('NOW: stand still', flush=True)\n"
              f"time.sleep({seconds})\n"
              "json.dump({'verdict': 'quiet standing: at most 0.02 rad/s'}, open(sys.argv[1] + '/summary.json', 'w'))\n"
              f"sys.exit({code})\n")
    return lambda kind, out, label: [sys.executable, "-u", "-c", script, str(out)]


def wait(jobs, state, seconds=5.0):
    end = time.time() + seconds
    while time.time() < end:
        if (jobs.summary() or {}).get("state") == state:
            return jobs.summary()
        time.sleep(0.02)
    raise AssertionError(f"job never {state}: {jobs.summary()}")


def test_a_run_from_start_to_its_files(tmp_path):
    jobs = sysid.Sysid(tmp_path, logbook.Logbook(), BRINGUP / "tools", runner=fake())
    with pytest.raises(ValueError, match="needs whole_body_controller"):
        jobs.start("yaw", ros("rl_agile_terrain"), RUNNING)
    with pytest.raises(ValueError, match="unknown measurement"):
        jobs.start("pitch", ros(), RUNNING)
    job = jobs.start("buzz", ros("rl_flat_level_sysid"), RUNNING)
    assert job["name"].startswith("buzz_rl_flat_level_sysid_") and job["controller"] == "rl_flat_level_sysid"
    with pytest.raises(Busy):
        jobs.start("yaw", ros(), RUNNING)
    done = wait(jobs, "done")
    assert done["code"] == 0
    status = jobs.status(ros(), RUNNING)
    assert status["now"] is None and "NOW: stand still" in status["tail"]
    run = status["runs"][0]
    assert run["name"] == job["name"] and run["kind"] == "buzz"
    assert run["result"] == {"text": "quiet standing: at most 0.02 rad/s"}
    assert {"panel.log", "summary.json"} <= set(run["files"])
    # Downloads: one file, or the run as a tar; never a path outside the run.
    assert jobs.file(job["name"], "summary.json").is_file()
    assert jobs.file(job["name"], "../job.json") is None and jobs.file("..", "job.json") is None
    assert jobs.archive("../" + job["name"]) is None
    names = tarfile.open(fileobj=io.BytesIO(jobs.archive(job["name"]))).getnames()
    assert f"{job['name']}/summary.json" in names
    # A panel started later finds the finished job again.
    again = sysid.Sysid(tmp_path, logbook.Logbook(), BRINGUP / "tools", runner=fake())
    assert again.summary()["state"] == "done"


def test_stop_is_one_sigint_and_a_failure_is_reported(tmp_path):
    jobs = sysid.Sysid(tmp_path, logbook.Logbook(), BRINGUP / "tools", runner=fake(seconds=30))
    assert jobs.stop() is None
    jobs.start("buzz", ros(), RUNNING)
    time.sleep(0.3)
    assert jobs.status(ros(), RUNNING)["now"] == "stand still"
    assert jobs.stop()["kind"] == "buzz"
    assert wait(jobs, "stopped")["code"] != 0
    failing = sysid.Sysid(tmp_path / "b", logbook.Logbook(), BRINGUP / "tools", runner=fake(code=1))
    failing.start("yaw", ros(), RUNNING)
    assert wait(failing, "failed")["code"] == 1


def test_a_yaw_result_reports_its_band_and_coherence(tmp_path):
    run = tmp_path / "experiments" / "yaw_20261117-120000"
    run.mkdir(parents=True)
    (run / "summary.json").write_text(json.dumps(
        [{"kp": 12.0, "chirp_nm": 0.2, "chirp_hz": 12.4, "coherence": 0.97},
         {"kp": 12.0, "chirp_nm": 0.4, "chirp_hz": 12.7, "coherence": 0.98}]))
    (run / "yaw_plant.json").write_text(json.dumps(
        {"f_hz": [3.2, 20.0, 30.0, 42.0], "to_gyro": [[0, -2]] * 4, "to_wheel": [[1, 0]] * 4,
         "coherence": [0.95, 0.9, 0.8, 0.6]}))
    # deploy/pi/pi sysid's runs are listed too.
    (tmp_path / "experiments" / "sysid_20261112-225514").mkdir()
    (tmp_path / "experiments" / "something_else").mkdir()
    jobs = sysid.Sysid(tmp_path, logbook.Logbook(), BRINGUP / "tools")
    runs = {r["name"]: r for r in jobs.runs()}
    assert set(runs) == {"yaw_20261117-120000", "sysid_20261112-225514"}
    result = runs["yaw_20261117-120000"]["result"]
    # Clean up to 30 Hz of the 42 swept: enough for the training.
    assert result["plant"] and result["f_hz"] == [3.2, 42.0]
    assert result["usable_hz"] == 30.0 and result["weak"] is False
    assert "0.4 Nm: mode 12.7 Hz" in result["text"]
    assert runs["sysid_20261112-225514"]["result"] == {}
