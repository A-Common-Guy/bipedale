"""buzz_record.py's analysis on synthetic signals, and sysid_yaw.py's plant band."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools"


def tool(name):
    spec = importlib.util.spec_from_file_location(name, TOOLS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


buzz = tool("buzz_record")


def test_band_content_gives_the_rms_and_the_frequency_of_a_tone():
    t = np.arange(0, 10, 1 / buzz.FS)
    x = 0.5 * np.sin(2 * np.pi * 9.0 * t) + 0.1 * np.sin(2 * np.pi * 30.0 * t) + 2.0
    rms, hz = buzz.band_content(x, 5.0, 15.0)
    assert rms == pytest.approx(0.5 / math.sqrt(2), rel=0.02) and hz == pytest.approx(9.0, abs=0.15)
    rms, hz = buzz.band_content(x, 15.0, 45.0)
    assert rms == pytest.approx(0.1 / math.sqrt(2), rel=0.03) and hz == pytest.approx(30.0, abs=0.15)
    assert math.isnan(buzz.band_content(x[:100], 5.0, 15.0)[0])


def modes(yaw_hz=0.0, yaw=0.0, pitch_hz=0.0, pitch=0.0):
    t = np.arange(0, 12, 1 / buzz.FS)
    opposed = yaw * np.sin(2 * np.pi * yaw_hz * t) * buzz.TRACK / (2 * buzz.RADIUS)   # wheel rad/s
    together = pitch * 4.0 * np.sin(2 * np.pi * pitch_hz * t)
    gyro = np.stack([np.zeros_like(t), pitch * np.sin(2 * np.pi * pitch_hz * t),
                     yaw * np.sin(2 * np.pi * yaw_hz * t)], 1)
    return buzz.modes(t, together + opposed, together - opposed, t, gyro, 0.5, 11.5)


def test_the_yaw_and_pitch_modes_are_told_apart():
    result = modes(yaw_hz=9.0, yaw=0.6)
    low = result["5-15 Hz"]
    assert low["gyro yaw"][0] == pytest.approx(0.6 / math.sqrt(2), rel=0.03)
    # Opposed wheels are given as the yaw rate they roll: the same number as the gyro here.
    assert low["wheels opposed (as yaw)"][0] == pytest.approx(low["gyro yaw"][0], rel=0.03)
    assert low["wheels together"][0] < 0.01 and low["gyro pitch"][0] < 0.01
    line = buzz.verdict(result)
    assert line.startswith("buzz standing") and "9.0 Hz" in line and "yaw loop inside the measured band" in line
    assert "above the first chirp" in buzz.verdict(modes(yaw_hz=28.0, yaw=0.4))
    assert "pitch loop" in buzz.verdict(modes(pitch_hz=7.5, pitch=0.5))
    assert buzz.verdict(modes(yaw_hz=9.0, yaw=0.03)).startswith("quiet standing")


def leg_modes(roll_hz=14.0, roll=0.0, legs_opposed=0.0, legs_together=0.0):
    """The wheels still, the gyro rolling, and the legs (LEGS order) moving in height:
    opposed is the right leg up while the left goes down, per joint."""
    t = np.arange(0, 12, 1 / buzz.FS)
    wave = np.sin(2 * np.pi * roll_hz * t)
    gyro = np.stack([roll * wave, np.zeros_like(t), np.zeros_like(t)], 1)
    right, left = (legs_together + legs_opposed) * wave, (legs_together - legs_opposed) * wave
    legs = np.stack([right, right, left, left], 1)   # right inner, outer, left inner, outer
    return buzz.modes(t, np.zeros_like(t), np.zeros_like(t), t, gyro, 0.5, 11.5, legs=legs)


def test_the_legs_roll_mode_is_told_apart_from_their_height_mode():
    low = leg_modes(roll=0.3, legs_opposed=0.4)["5-15 Hz"]
    assert low["legs opposed (per joint)"][0] == pytest.approx(0.4 / math.sqrt(2), rel=0.03)
    assert low["legs opposed (per joint)"][1] == pytest.approx(14.0, abs=0.15)
    assert low["legs together (per joint)"][0] < 0.01
    low = leg_modes(legs_together=0.4)["5-15 Hz"]
    assert low["legs together (per joint)"][0] == pytest.approx(0.4 / math.sqrt(2), rel=0.03)
    assert low["legs opposed (per joint)"][0] < 0.01
    # Without the legs, the modes are the gyro's and the wheels' only.
    assert not any(name.startswith("legs") for name in modes(yaw_hz=9.0, yaw=0.6)["5-15 Hz"])


def test_a_roll_buzz_names_the_legs():
    line = buzz.verdict(leg_modes(roll=0.3, legs_opposed=0.4))
    assert line.startswith("buzz standing") and "gyro roll at 14.0 Hz" in line
    assert "legs opposed 0.28 rad/s per joint at 14.0 Hz" in line
    # A roll buzz analysed without the legs still gets its verdict.
    gyro_only = {band: {k: v for k, v in values.items() if not k.startswith("legs")}
                 for band, values in leg_modes(roll=0.3).items()}
    assert "roll, the legs: no measured response yet" in buzz.verdict(gyro_only)


def test_the_plant_band_stays_clear_of_the_sweep_ends():
    sysid = tool("sysid_yaw")
    chirp = lambda f0, f1: {"kind": "chirp", "f0_hz": f0, "f1_hz": f1}
    assert sysid.plant_band({"events": [chirp(2.0, 25.0)]}) == (3.0, 22.0)
    assert sysid.plant_band({"events": [{"kind": "kp"}, chirp(2.0, 45.0), chirp(2.0, 45.0)]}) == (3.0, 42.0)
    with pytest.raises(ValueError):
        sysid.plant_band({"events": [{"kind": "pulses"}]})


def test_opposed_signals_pair_rear_with_rear_and_the_chirp_is_the_controllers():
    sysid = tool("sysid_yaw")
    t = np.arange(0, 6, 1 / sysid.FS)
    event = {"t": 1.0, "f0_hz": 2.0, "f1_hz": 20.0, "duration_s": 4.0}
    chirp = sysid.chirp_reference(event, t)
    assert abs(chirp[t < 1.0]).max() == 0.0 and abs(chirp[t > 5.0]).max() == 0.0
    assert abs(chirp).max() == pytest.approx(1.0, abs=0.01)
    # A roll sweep: the left leg longer, the right shorter, i.e. both motors of
    # a side the same way, the two sides against each other.
    joints = np.zeros((len(t), 19))
    joints[:, 0] = t
    column = lambda name, block: 1 + 6 * block + sysid.JOINTS.index(name)
    for name, sign in (("right_motor_inner", 1), ("right_motor_outer", 1),
                       ("left_motor_inner", -1), ("left_motor_outer", -1)):
        joints[:, column(name, 2)] = sign * 0.3 * chirp
    gyro = np.stack([t, 0 * t, 0.2 * chirp, 0 * t], 1)          # t, z, x, y
    _, measured = sysid.opposed_signals(joints, gyro, 1.0, 5.0)
    grid = np.arange(1.0, 5.0, 1 / sysid.FS)
    reference = sysid.chirp_reference(event, grid)
    np.testing.assert_allclose(measured["rear_torque"], 0.3 * reference, atol=1e-6)
    np.testing.assert_allclose(measured["front_torque"], 0.3 * reference, atol=1e-6)
    np.testing.assert_allclose(measured["roll_rate"], 0.2 * reference, atol=1e-6)
    assert abs(measured["wheel_torque"]).max() == 0.0 and abs(measured["yaw_rate"]).max() == 0.0
    # Fore-aft against each other is the right rear and the left FRONT motor the
    # same way: with rear paired to rear it shows as rear against front.
    joints[:] = 0.0
    joints[:, 0] = t
    for name, sign in (("right_motor_inner", 1), ("right_motor_outer", -1),
                       ("left_motor_inner", 1), ("left_motor_outer", -1)):
        joints[:, column(name, 2)] = sign * 0.3 * chirp
    _, measured = sysid.opposed_signals(joints, gyro, 1.0, 5.0)
    np.testing.assert_allclose(measured["rear_torque"], 0.3 * reference, atol=1e-6)
    np.testing.assert_allclose(measured["front_torque"], -0.3 * reference, atol=1e-6)
    # Runs from before the axes are yaw runs.
    plan = {"events": [{"kind": "chirp", "f0_hz": 2.0, "f1_hz": 25.0},
                       {"kind": "chirp", "axis": "roll", "f0_hz": 2.0, "f1_hz": 25.0}]}
    assert len(sysid.chirps_of(plan, "yaw")) == 1 and len(sysid.chirps_of(plan, "roll")) == 1
    assert len(sysid.chirps_of(plan)) == 2
