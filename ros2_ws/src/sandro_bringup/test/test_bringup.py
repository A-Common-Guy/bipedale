import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_controller_limits_and_commissioning_watchdogs_are_explicit():
    config = yaml.safe_load((ROOT / "config" / "controllers.yaml").read_text())
    manager = config["controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 300
    assert manager["driving_balancer"]["type"].endswith("DrivingBalancerController")
    assert manager["whole_body_controller"]["type"].endswith(
        "WholeBodyController"
    )

    balance = config["driving_balancer"]["ros__parameters"]
    assert balance["legs"]["torque_limit_nm"] == 3.0
    assert balance["wheels"]["torque_limit_nm"] == 6.0
    assert balance["drive"]["command_timeout_s"] == 0.3

    whole_body = config["whole_body_controller"]["ros__parameters"]
    assert whole_body["limits"]["leg_torque_nm"] == 3.0
    assert whole_body["limits"]["wheel_torque_nm"] == 6.0
    assert whole_body["safety"]["command_timeout_s"] == 0.3

    commissioning = [
        name for name in manager if name.startswith("commission_")
    ]
    assert len(commissioning) == 8
    for name in commissioning:
        parameters = config[name]["ros__parameters"]
        assert parameters["command_timeout_s"] == 0.1
        assert parameters["effort_limit_nm"] == 0.5


def test_teleop_maps_height_and_estop_chord():
    config = yaml.safe_load((ROOT / "config" / "teleop.yaml").read_text())
    teleop = config["teleop_twist_joy_node"]["ros__parameters"]
    assert teleop["require_enable_button"] is True
    assert teleop["axis_linear"]["z"] != teleop["axis_linear"]["x"]
    assert teleop["axis_linear"]["z"] != teleop["axis_angular"]["yaw"]
    assert 0.0 < teleop["scale_linear"]["z"] <= 0.05
    estop = config["joy_estop"]["ros__parameters"]
    assert len(estop["axes"]) >= 2
    assert estop["axes"][0] != estop["axes"][1]
    assert -1.0 < estop["axis_pressed_below"] < 0.0
    motion_axes = {teleop["axis_linear"]["x"], teleop["axis_linear"]["z"],
                   teleop["axis_angular"]["yaw"]}
    assert not motion_axes & set(estop["axes"])
    assert len(estop["rearm_buttons"]) >= 2
    assert teleop["enable_button"] not in estop["rearm_buttons"]
    assert estop["rearm_hold_s"] >= 1.0
    launch = (ROOT / "launch" / "sandro.launch.py").read_text()
    assert 'executable="joy_estop"' in launch


def test_joy_estop_chord_and_controller_selection():
    from types import SimpleNamespace
    from sandro_bringup.joy_estop import (
        buttons_held, chord_pressed, controllers_to_stop, rearm_step)

    assert buttons_held([0, 0, 0, 0, 1, 0, 1], [4, 6])
    assert not buttons_held([0, 0, 0, 0, 1, 0, 0], [4, 6])
    assert not buttons_held([1, 1], [4, 6])
    # Two separate holds: never from disarmed drives straight into a balancer.
    assert rearm_step(False, [], True) == "arm_hardware"
    assert rearm_step(True, [], True) == "activate"
    assert rearm_step(True, [], False) == "load_and_activate"
    assert rearm_step(True, ["whole_body_controller"], True) == "nothing"
    assert rearm_step(None, [], True) == "activate"

    assert chord_pressed([0, 0, 0, 0, -1.0, -0.9], [4, 5], -0.5)
    assert not chord_pressed([0, 0, 0, 0, -1.0, 1.0], [4, 5], -0.5)  # one trigger only
    assert not chord_pressed([0, 0, 0, 0, 0, 0], [4, 5], -0.5)  # no event yet reads 0
    assert not chord_pressed([-1, -1], [4, 5], -0.5)  # short message never triggers
    assert not chord_pressed([-1, -1], [], -0.5)
    states = [
        SimpleNamespace(name="joint_state_broadcaster", state="active", claimed_interfaces=[]),
        SimpleNamespace(name="whole_body_controller", state="active",
                        claimed_interfaces=["right_wheel/effort"]),
        SimpleNamespace(name="driving_balancer", state="inactive",
                        claimed_interfaces=[]),
    ]
    assert controllers_to_stop(states) == ["whole_body_controller"]


def test_real_robot_overrides_only_touch_known_parameters():
    """The per-robot override file must stay a subset of the shipped config."""
    shipped = yaml.safe_load((ROOT / "config" / "controllers.yaml").read_text())
    overrides = yaml.safe_load((ROOT / "config" / "sandro_real_overrides.yaml").read_text())
    assert set(overrides) == {"whole_body_controller"}
    base = shipped["whole_body_controller"]["ros__parameters"]

    def check(subset, reference, path=""):
        for key, value in subset.items():
            assert key in reference, f"{path}{key} is not a shipped parameter"
            if isinstance(value, dict):
                check(value, reference[key], f"{path}{key}.")
            else:
                assert type(value) is type(reference[key]), f"{path}{key} type"

    check(overrides["whole_body_controller"]["ros__parameters"], base)
    tasks = overrides["whole_body_controller"]["ros__parameters"]["tasks"]
    # A rearward neutral shift eats the front pivot's stretch margin at height.
    assert tasks["fore_aft"]["neutral_offset_m"] < 0.0
    assert tasks["height"]["max_m"] <= 0.19


def test_cyclone_configuration_is_well_formed():
    root = ET.parse(ROOT / "config" / "cyclonedds.xml").getroot()
    assert root.tag.endswith("CycloneDDS")


def test_launch_modules_import():
    for path in sorted((ROOT / "launch").glob("*.launch.py")):
        spec = importlib.util.spec_from_file_location(path.stem, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        assert module.generate_launch_description() is not None


def test_real_hardware_never_defaults_to_arm():
    main_launch = (ROOT / "launch" / "sandro.launch.py").read_text()
    commissioning = (ROOT / "launch" / "commissioning.launch.py").read_text()
    assert '"arm_hardware", default_value="false"' in main_launch
    assert '"activate", default_value="false"' in commissioning
