"""policy_record.py: the recording cut at the controller changes, and what each stretch did."""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
spec = importlib.util.spec_from_file_location("policy_record", TOOLS / "policy_record.py")
record = importlib.util.module_from_spec(spec)
spec.loader.exec_module(record)
FS = record.buzz.FS


def test_a_stretch_is_one_stay_of_a_controller():
    events = [{"kind": "start", "t": 100.0},
              {"kind": "controller", "name": None, "t": 100.1},
              {"kind": "controller", "name": "rl_agile_terrain_v6", "t": 110.0},
              {"kind": "controller", "name": "rl_agile_step", "t": 150.0},
              {"kind": "controller", "name": None, "t": 190.0},           # the E-stop
              {"kind": "controller", "name": "rl_agile_terrain_v6", "t": 200.0},
              {"kind": "end", "t": 230.0}]
    assert record.stretches(events) == [("rl_agile_terrain_v6", 110.0, 150.0),
                                        ("rl_agile_step", 150.0, 190.0),
                                        ("rl_agile_terrain_v6", 200.0, 230.0)]
    assert record.stretches([{"kind": "start", "t": 1.0}, {"kind": "end", "t": 9.0}]) == []


def signals(seconds=30.0):
    """10 s standing with a 13 Hz roll buzz, 10 s driving at 0.5 m/s, 10 s on its side."""
    t = np.arange(0.0, seconds, 1 / FS)
    speed = np.where((t >= 10) & (t < 20), 0.5 / record.buzz.RADIUS, 0.0)
    wheels = np.stack([t, speed, speed], 1)
    roll = np.where(t < 10, 0.3 * np.sin(2 * np.pi * 13.0 * t), 0.0)
    tilt = np.where(t >= 20, 1.4, 0.02)
    imu = np.stack([t, roll, 0 * t, 0 * t, tilt], 1)
    return wheels, imu


def test_the_windows_tell_standing_driving_and_lying_apart():
    wheels, imu = signals()
    found = record.windows(wheels, imu, 0.0, 30.0)
    assert [w["state"] for w in found] == ["standing"] * 5 + ["driving"] * 5 + ["down"] * 5
    rms, hz = found[1]["bands"]["5-15 Hz"]["gyro roll"]
    assert rms == pytest.approx(0.3 / math.sqrt(2), rel=0.05) and hz == pytest.approx(13.0, abs=0.5)
    assert found[6]["speed"] == pytest.approx(0.5, rel=0.01)
    assert record.windows(wheels, imu, 0.0, 1.5) == []


def test_the_analysis_gives_one_row_per_stretch_with_its_policy(tmp_path, monkeypatch):
    wheels, imu = signals()
    monkeypatch.setattr(record, "read_bag", lambda path: (wheels, imu))
    plan = {"version": {"short": "abc1234"},
            "events": [{"kind": "start", "t": 0.0},
                       {"kind": "controller", "name": "rl_agile_step", "t": 0.0},
                       {"kind": "controller", "name": "rl_stand_up_v2", "t": 20.0},
                       {"kind": "end", "t": 30.0}],
            "controllers": {"rl_agile_step": {"policy_file": "package://x/agile_step.bin",
                                              "policy_fingerprint": "f00d"},
                            "rl_stand_up_v2": None}}
    (tmp_path / "plan.json").write_text(json.dumps(plan))
    verdict = record.analyse(tmp_path)
    first, second = verdict.split("; ")
    assert first.startswith("rl_agile_step 20 s, buzz standing 0.21 rad/s of gyro roll at 13")
    assert "drove 10 s to 0.5 m/s" in first
    assert second.startswith("rl_stand_up_v2 10 s, never stood still") and "lying 8 s" in second
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["version"]["short"] == "abc1234"
    row = summary["stretches"][0]
    assert row["policy_file"].endswith("agile_step.bin") and row["policy_fingerprint"] == "f00d"
    assert row["standing"]["seconds"] == 8.0 and summary["stretches"][1]["policy_file"] is None


def test_what_is_recorded():
    assert {"/joint_states", "/imu_broadcaster/imu", "/cmd_vel", "/terrain_profile",
            "/controller_manager/activity"} <= set(record.TOPICS)
