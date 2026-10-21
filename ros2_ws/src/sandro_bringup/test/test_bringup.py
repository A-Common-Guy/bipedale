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


def test_teleop_maps_sticks_switch_and_estop_chord():
    config = yaml.safe_load((ROOT / "config" / "teleop.yaml").read_text())
    teleop = config["joy_teleop"]["ros__parameters"]
    estop = config["joy_estop"]["ros__parameters"]
    assert len(estop["axes"]) >= 2
    assert estop["axes"][0] != estop["axes"][1]
    assert -1.0 < estop["axis_pressed_below"] < 0.0
    # Throttle and turning on the left stick, height and roll on the right.
    assert {teleop["forward_axis"], teleop["yaw_axis"]} == {0, 1}
    assert {teleop["height_axis"], teleop["roll_axis"]} == {2, 3}
    motion_axes = [teleop[k] for k in ("forward_axis", "yaw_axis", "height_axis", "roll_axis")]
    assert not set(motion_axes) & set(estop["axes"])
    assert teleop["estop_axes"] == estop["axes"]
    buttons = [teleop["deadman_button"], teleop["switch_button"], teleop["announce_button"],
               teleop["recentre_button"]]
    assert len(set(buttons)) == len(buttons)
    assert not set(buttons) & set(estop["rearm_buttons"])
    assert len(estop["rearm_buttons"]) >= 2
    assert estop["rearm_hold_s"] >= 1.0
    policies = yaml.safe_load((ROOT / "config" / "rl_policies.yaml").read_text())
    assert set(teleop["policies"]) <= set(policies["controller_manager"]["ros__parameters"])
    for name in teleop["policies"]:
        commands = policies[name]["ros__parameters"]["commands"]
        assert commands["height_min_m"] <= teleop["handover_height_m"] <= commands["height_max_m"]
    launch = (ROOT / "launch" / "sandro.launch.py").read_text()
    assert 'executable="joy_estop"' in launch
    assert 'executable="joy_teleop"' in launch


def test_joy_teleop_mapping_and_handover():
    from sandro_bringup.joy_teleop import (
        Limits, handover_ready, height_rate_towards, next_policy, stick_command)

    mapping = {"forward_axis": 1, "yaw_axis": 0, "height_axis": 3, "roll_axis": 2,
               "forward_sign": 1.0, "yaw_sign": 1.0, "height_sign": 1.0, "roll_sign": -1.0}
    limits = Limits(1.0, 3.0, 0.15, 0.3)
    # Left stick up and left: forward and turning left; right stick right: +roll.
    assert stick_command([1.0, 1.0, -1.0, 0.0], mapping, limits, 0.5) == (0.5, 1.5, 0.0, 0.3)
    assert stick_command([], mapping, limits, 1.0) == (0.0, 0.0, 0.0, -0.0)
    assert abs(height_rate_towards(0.17, 0.18, 0.05, 3.0) - 0.03) < 1e-12
    assert height_rate_towards(0.10, 0.18, 0.05, 3.0) == 0.05

    policies = ["rl_flat_level", "rl_agile_terrain"]
    assert next_policy(policies, "rl_flat_level") == "rl_agile_terrain"
    assert next_policy(policies, "rl_agile_terrain") == "rl_flat_level"
    assert next_policy(policies, "whole_body_controller") is None

    target = Limits(0.5, 1.5, 0.05, 0.0, 0.17, 0.15, 2.0)
    diag = {"message": "policy", "velocity_reference_m_s": 0.0, "height_reference_m": 0.17,
            "roll_reference_rad": 0.0, "tilt_rad": 0.03, "rate_rad_s": 0.1}
    still = [0.0] * 6
    assert handover_ready(diag, still, target, 0.17, 0.6)[0]
    assert not handover_ready(diag, [0, 0, 1.5, 0, 0, 0], target, 0.17, 0.6)[0]
    assert not handover_ready(diag, None, target, 0.17, 0.6)[0]
    assert not handover_ready({**diag, "message": "standing up"}, still, target, 0.17, 0.6)[0]
    assert not handover_ready({**diag, "height_reference_m": 0.15}, still, target, 0.17, 0.6)[0]
    assert not handover_ready({**diag, "roll_reference_rad": 0.1}, still, target, 0.17, 0.6)[0]
    assert not handover_ready({**diag, "tilt_rad": 0.1}, still, target, 0.17, 0.6)[0]


def test_joy_teleop_leds_show_policy_and_switch(tmp_path):
    from sandro_bringup.joy_teleop import PadLeds, led_state

    policies = ["rl_flat_level", "rl_agile_terrain"]
    colors = [0, 255, 0, 255, 100, 0]
    assert led_state(policies, colors, "rl_agile_terrain", None, True) == ((255, 100, 0), 2)
    assert led_state(policies, colors, None, None, False) == ((255, 0, 0), 0)
    assert led_state(policies, colors, "whole_body_controller", None, False) == (
        (255, 255, 255), 0)
    # A switch blinks between the outgoing and the incoming policy.
    assert led_state(policies, colors, "rl_flat_level", "rl_agile_terrain", False) == (
        (0, 255, 0), 1)
    assert led_state(policies, colors, "rl_flat_level", "rl_agile_terrain", True) == (
        (255, 100, 0), 2)

    leaves = ["rgb:indicator/multi_intensity", "rgb:indicator/brightness"] + [
        f"white:player-{k}/brightness" for k in range(1, 6)]
    for leaf in leaves:
        path = tmp_path / f"input7:{leaf}"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("0")
    PadLeds(str(tmp_path)).show((255, 100, 0), 2)
    read = {leaf: (tmp_path / f"input7:{leaf}").read_text() for leaf in leaves}
    assert read["rgb:indicator/multi_intensity"] == "255 100 0"
    assert [read[f"white:player-{k}/brightness"] for k in range(1, 6)] == [
        "1", "1", "0", "0", "0"]
    PadLeds(str(tmp_path / "absent")).show((255, 0, 0), 0)  # no pad: silently nothing


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
    shipped.update(yaml.safe_load((ROOT / "config" / "rl_policies.yaml").read_text()))
    overrides = yaml.safe_load((ROOT / "config" / "sandro_real_overrides.yaml").read_text())
    assert set(overrides) == {"whole_body_controller", "rl_flat_level", "rl_agile_terrain"}
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
    for name in ("rl_flat_level", "rl_agile_terrain"):
        check(overrides[name]["ros__parameters"], shipped[name]["ros__parameters"], f"{name}.")
        # One physical IMU mounting for every controller.
        assert (overrides[name]["ros__parameters"]["observation"]["imu_offset_rad"]
                == tasks["roll"]["imu_offset_rad"])


def test_rl_policies_are_complete():
    config = yaml.safe_load((ROOT / "config" / "rl_policies.yaml").read_text())
    manager = config["controller_manager"]["ros__parameters"]
    controllers = ROOT.parent / "sandro_controllers"
    assert set(manager) == {"rl_flat_level", "rl_agile_terrain"}
    for name, entry in manager.items():
        assert entry["type"] == "sandro_controllers/RlPolicyController"
        parameters = config[name]["ros__parameters"]
        policy = parameters["policy_file"].removeprefix("package://sandro_controllers/")
        assert (controllers / policy).is_file(), policy
        assert parameters["command_topic"] == "/cmd_vel"
        assert parameters["observation"]["command_size"] == (
            4 if parameters["commands"]["roll_rad"] > 0.0 else 3)


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
