"""The consistency check passes on the current model and fails on altered copies."""

import re

import pytest

from sandro_rl import consistency
from sandro_rl.robot import ModelContractError, RobotSpec


def test_contract_holds_for_the_current_model():
    RobotSpec.load()


@pytest.mark.gpu
def test_current_model_is_consistent():
    report = consistency.check()
    assert report.passed, report.text()


def _edit(path, pattern, replacement, count=1):
    text = path.read_text()
    new, n = re.subn(pattern, replacement, text, count=count)
    assert n, f"pattern {pattern!r} not found"
    path.write_text(new)


ALTERATIONS = {
    "renamed joint": (r'"right_wheel"', '"right_wheel_renamed"', "joint 'right_wheel' missing"),
    "actuator gear": (r'(<motor name="left_motor_inner" joint="left_motor_inner" forcerange="-10 10") gear="1"',
                      r'\1 gear="2"', "gear 2"),
    "missing actuator": (r'\s*<motor name="left_wheel"[^>]*/>', "", "ctrl size"),
    "removed keyframe": (r'<key name="stand"', '<key name="crouch"', "stand"),
    "tyre condim 6": (r'(right_motor_shaft_2_3_tire_cyl"[^>]*?)condim="4"', r'\1condim="6"', "condim"),
    "tyre friction not in sim_identified.yaml": (
        r'(left_motor_shaft_2_2_tire_cyl"[^>]*?)friction="0.8 ', r'\1friction="0.6 ', "friction"),
    "physics step": (r'<option ', '<option timestep="0.001" ', "physics step"),
}


@pytest.mark.parametrize("name", sorted(ALTERATIONS))
def test_altered_model_fails(model_copy, name):
    scene, robot = model_copy
    pattern, replacement, reason = ALTERATIONS[name]
    if name == "physics step":
        text = robot.read_text()
        robot.write_text(re.sub(r'timestep="[^"]*"', "", text))
    _edit(robot, pattern, replacement, count=0 if name == "renamed joint" else 1)
    with pytest.raises(ModelContractError, match=reason):
        RobotSpec.load(scene)
    report = consistency.check(scene)
    assert not report.passed
    assert "model contract" in report.checks[0]["check"] and not report.checks[0]["pass"]
