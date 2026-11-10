import importlib.util
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[2]
JOINTS = ("right_motor_inner", "right_motor_outer", "right_wheel",
          "left_motor_inner", "left_motor_outer", "left_wheel")


def _load(path):
    spec = importlib.util.spec_from_file_location(path.stem.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_controller_limits_and_commissioning_watchdogs_are_explicit():
    config = yaml.safe_load((ROOT / "config" / "controllers.yaml").read_text())
    manager = config["controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 300
    assert manager["driving_balancer"]["type"].endswith("DrivingBalancerController")
    assert manager["whole_body_controller"]["type"].endswith(
        "WholeBodyController"
    )

    balance = config["driving_balancer"]["ros__parameters"]
    # The legs may use the drives' envelope peak (hard_effort_limit_nm); the
    # hardware folds it back to the rated torque as a drive heats.
    xacro = (ROOT.parent / "sandro_description" / "urdf" / "sandro.ros2_control.xacro").read_text()
    peak = float(re.search(r'name="hard_effort_limit_nm">([0-9.]+)<', xacro).group(1))
    rated = float(re.search(r'name="rated_effort_nm">([0-9.]+)<', xacro).group(1))
    assert (peak, rated) == (9.0, 5.0)
    assert balance["legs"]["torque_limit_nm"] == peak
    assert balance["wheels"]["torque_limit_nm"] == 6.0
    assert balance["drive"]["command_timeout_s"] == 0.3

    whole_body = config["whole_body_controller"]["ros__parameters"]
    assert whole_body["limits"]["leg_torque_nm"] == peak
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


def test_broadcasters_keep_joint_states_global_and_health_slow():
    config = yaml.safe_load((ROOT / "config" / "controllers.yaml").read_text())
    manager = config["controller_manager"]["ros__parameters"]
    joint_state = config["joint_state_broadcaster"]["ros__parameters"]
    # use_local_topics would move /joint_states as well; only the dynamic topic goes.
    assert joint_state.get("use_local_topics", False) is False
    assert joint_state["publish_dynamic_joint_states"] is False
    assert manager["health_broadcaster"]["type"] == "joint_state_broadcaster/JointStateBroadcaster"
    health = config["health_broadcaster"]["ros__parameters"]
    assert health["use_local_topics"] is True
    assert isinstance(health["update_rate"], int)
    assert manager["update_rate"] % health["update_rate"] == 0
    assert health["update_rate"] == 10
    whole_body = config["whole_body_controller"]["ros__parameters"]
    # compare_runs.py's leg-lag and pitch-peak metrics need about 50 Hz.
    assert whole_body["diagnostics.publish_rate_hz"] >= 50.0


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
    # Re-arm pre-loads the same switch partners the launch does.
    assert estop["switch_policies"] == teleop["policies"]
    assert estop["rearm_timeout_s"] > estop["service_timeout_s"]
    assert 0.0 < teleop["handover_wheel_velocity_rad_s"]
    controllers = yaml.safe_load((ROOT / "config" / "controllers.yaml").read_text())
    manager = controllers["controller_manager"]["ros__parameters"]
    assert set(estop["rearm_broadcasters"]) == {"joint_state_broadcaster", "health_broadcaster"}
    assert set(estop["rearm_broadcasters"]) <= set(manager)


def test_joy_teleop_mapping_and_handover(ros_stubs):
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
    still = dict.fromkeys(JOINTS, 0.0)

    def ready(diag=diag, velocity=still):
        return handover_ready(diag, velocity, target, 0.17, 0.6, 1.5)

    assert ready() == (True, "")
    # Balancing wheels have their own margin; the legs keep 0.6 x 2.0 rad/s.
    assert ready(velocity={**still, "right_wheel": 1.0, "left_wheel": -1.4}) == (True, "")
    assert ready(velocity={**still, "left_wheel": 1.6}) == (False, "left_wheel moving")
    assert ready(velocity={**still, "right_motor_outer": 1.5}) == (
        False, "right_motor_outer moving")
    assert ready(velocity=None) == (False, "no joint state")
    assert ready({**diag, "message": "standing up"}) == (
        False, "running controller is 'standing up'")
    assert not ready({**diag, "height_reference_m": 0.15})[0]
    assert not ready({**diag, "roll_reference_rad": 0.1})[0]
    assert ready({**diag, "tilt_rad": 0.1}) == (False, "body tilted")


def test_joy_teleop_decodes_joint_states_only_during_a_switch():
    source = (ROOT / "sandro_bringup" / "joy_teleop.py").read_text()
    assert "MultiThreadedExecutor" not in source and "wait_for_service" not in source
    constructor = source.split("def __init__", 1)[1].split("\n    def ", 1)[0]
    assert '"/joint_states"' not in constructor
    assert '"/joint_states"' in source.split("def request_switch", 1)[1].split("\n    def ", 1)[0]
    assert "destroy_subscription(self.joint_subscription)" in source


def test_joy_teleop_leds_show_policy_and_switch(tmp_path, ros_stubs):
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


def test_joy_estop_chord_and_controller_selection(ros_stubs):
    from types import SimpleNamespace
    from sandro_bringup.joy_estop import (
        buttons_held, chord_pressed, controllers_to_stop, rearm_step, reports_error,
        switch_partners)

    assert buttons_held([0, 0, 0, 0, 1, 0, 1], [4, 6])
    assert not buttons_held([0, 0, 0, 0, 1, 0, 0], [4, 6])
    assert not buttons_held([1, 1], [4, 6])
    # Two separate holds: never from disarmed drives straight into a balancer.
    assert rearm_step(False, [], "inactive") == "arm_hardware"
    assert rearm_step(True, [], "inactive") == "activate"
    assert rearm_step(True, [], "unconfigured") == "configure_and_activate"
    assert rearm_step(True, [], None) == "load_and_activate"
    assert rearm_step(True, ["whole_body_controller"], "active") == "nothing"
    assert rearm_step(None, [], "inactive") == "activate"
    # The first arming of a session is the operator's, never the gamepad's.
    assert rearm_step(False, [], "inactive", armed_once=False) == "not_armed_yet"
    assert rearm_step(None, [], "inactive", armed_once=False) == "activate"
    # A controller active on disarmed drives would act the moment they arm.
    assert rearm_step(False, ["whole_body_controller"], "active") == "blocked"
    # A fallen (ERROR) controller restarts with one hold; a healthy one does not.
    assert rearm_step(True, ["rl_flat_level"], "active", failed=["rl_flat_level"]) == "restart"
    assert rearm_step(True, ["rl_flat_level"], "active", failed=[]) == "nothing"
    assert switch_partners(["rl_flat_level", "rl_agile_terrain"], "rl_agile_terrain") == [
        "rl_flat_level"]
    assert switch_partners(["rl_flat_level", "rl_agile_terrain"], "whole_body_controller") == []
    error = b"\x02"
    assert reports_error([SimpleNamespace(level=b"\x01"), SimpleNamespace(level=error)], error)
    assert not reports_error([SimpleNamespace(level=b"\x01")], error)
    assert not reports_error([], error)

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
    assert set(overrides) == {"whole_body_controller", "rl_flat_level", "rl_agile_terrain",
                              "rl_stand_up", "rl_flat_level_fast", "rl_flat_level_drivetrain",
                              "rl_flat_level_fast_drivetrain"}
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
    for name in ("rl_flat_level", "rl_agile_terrain", "rl_stand_up", "rl_flat_level_fast",
                 "rl_flat_level_drivetrain", "rl_flat_level_fast_drivetrain"):
        check(overrides[name]["ros__parameters"], shipped[name]["ros__parameters"], f"{name}.")
        # One physical IMU mounting for every controller.
        assert (overrides[name]["ros__parameters"]["observation"]["imu_offset_rad"]
                == tasks["roll"]["imu_offset_rad"])


def test_rl_policies_are_complete():
    config = yaml.safe_load((ROOT / "config" / "rl_policies.yaml").read_text())
    manager = config["controller_manager"]["ros__parameters"]
    controllers = ROOT.parent / "sandro_controllers"
    assert set(manager) == {"rl_flat_level", "rl_agile_terrain", "rl_stand_up", "rl_flat_level_fast",
                            "rl_flat_level_drivetrain", "rl_flat_level_fast_drivetrain"}
    # Only the stand-up policy engages lying down.
    assert {name for name in manager
            if (config[name]["ros__parameters"].get("safety") or {}).get("start_lying")} == {"rl_stand_up"}
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


def test_launch_modules_import(ros_stubs):
    for path in sorted((ROOT / "launch").glob("*.launch.py")):
        assert _load(path).generate_launch_description() is not None


def test_joint_timing_follows_the_state_source(ros_stubs, monkeypatch):
    module = _load(ROOT / "launch" / "sandro.launch.py")
    assert set(module.JOINT_TIMING["mit"]) == set(module.JOINT_TIMING["encoder"])

    def configuration(values):
        class Configuration:
            def __init__(self, name):
                self.name = name

            def perform(self, _context):
                return values[self.name]
        return Configuration

    monkeypatch.setattr(module, "LaunchConfiguration",
                        configuration({"state_source": "mit", "encoder_period_ms": "auto",
                                       "sim_joint_rate_hz": "250"}))
    assert module._joint_timing(None, "encoder_period_ms") == "20"
    assert module._joint_timing(None, "sim_joint_rate_hz") == "250"
    monkeypatch.setattr(module, "LaunchConfiguration",
                        configuration({"state_source": "can", "encoder_period_ms": "auto"}))
    with pytest.raises(ValueError):
        module._joint_timing(None, "encoder_period_ms")


def test_real_hardware_never_defaults_to_arm():
    main_launch = (ROOT / "launch" / "sandro.launch.py").read_text()
    commissioning = (ROOT / "launch" / "commissioning.launch.py").read_text()
    assert '"arm_hardware", default_value="false"' in main_launch
    assert '"activate", default_value="false"' in commissioning


class _Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


def _expand_launch(monkeypatch, **arguments):
    """sandro.launch.py's actions for these arguments, with recording stand-ins for launch."""
    module = _load(ROOT / "launch" / "sandro.launch.py")
    for name in ("Node", "LogInfo", "RegisterEventHandler", "OnProcessExit",
                 "DeclareLaunchArgument", "OpaqueFunction"):
        monkeypatch.setattr(module, name, type(name, (_Action,), {}))
    monkeypatch.setattr(module, "LaunchDescription", list)

    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    monkeypatch.setattr(module, "LaunchConfiguration", Configuration)
    shares = {"sandro_bringup": str(ROOT), "sandro_description": str(ROOT.parent / "sandro_description"),
              "sandro_sim": str(ROOT.parent / "sandro_sim"), "mujoco_ros2_control": "/opt/mujoco",
              "joy": "/opt/joy", "rviz2": "/opt/rviz2"}
    monkeypatch.setattr(module, "_package", shares.get)
    monkeypatch.setattr(module.subprocess, "check_output", lambda *a, **k: "<robot/>")
    declared = {a.args[0]: a.kwargs["default_value"] for a in module.generate_launch_description()
                if type(a).__name__ == "DeclareLaunchArgument"}
    actions = module._setup({**declared, "rviz": "false", **arguments})
    nodes = [a for a in actions if type(a).__name__ == "Node"]
    spawners = []
    for node in nodes:
        if node.kwargs["executable"] == "spawner":
            arguments_ = node.kwargs["arguments"]
            names = arguments_[:arguments_.index("--controller-manager")]
            spawners.append(([n for n in names if not n.startswith("--")],
                             "--inactive" in names, node))
    manager = next(n for n in nodes if n.kwargs["executable"] == "ros2_control_node")
    logs = " ".join(a.kwargs["msg"] for a in actions if type(a).__name__ == "LogInfo")
    return module, actions, nodes, spawners, manager, logs


def _initial_state(manager):
    return [p for p in manager.kwargs["parameters"]
            if isinstance(p, dict) and "hardware_components_initial_state.inactive" in p]


def test_real_launch_starts_one_session_with_the_drives_inactive(monkeypatch, ros_stubs):
    module, _, nodes, spawners, manager, logs = _expand_launch(
        monkeypatch, hardware="real", controller="rl_flat_level",
        extra_params="/tmp/overrides.yaml")
    assert manager.kwargs["package"] == "controller_manager"
    parameters = manager.kwargs["parameters"]
    # Drives inactive, IMU active (unlisted); applied after any override file.
    assert _initial_state(manager) == [parameters[-1]]
    assert parameters[-1]["hardware_components_initial_state.inactive"] == ["SandroSystem"]
    assert parameters.index("/tmp/overrides.yaml") < len(parameters) - 1
    assert module.REALTIME in parameters
    assert set(module.REALTIME) == {"thread_priority", "cpu_affinity", "lock_memory"}
    assert 0 < module.REALTIME["thread_priority"] < 99
    # One spawner activates every broadcaster, the IMU's last (the spawner stops at the
    # first failure); one loads the controller and its switch partner inactive.
    assert [(names, inactive) for names, inactive, _ in spawners] == [
        (list(module.BROADCASTERS), False),
        (["rl_flat_level", "rl_agile_terrain", "rl_flat_level_fast", "rl_flat_level_drivetrain",
          "rl_flat_level_fast_drivetrain"], True)]
    assert module.BROADCASTERS[-1] == "imu_broadcaster"
    assert "REAL SAFE STATE" in logs and "not armed" in logs
    # Teleop starts too: nothing moves before the operator arms.
    executables = {n.kwargs["executable"]: n for n in nodes}
    assert {"joy_node", "joy_teleop", "joy_estop"} <= set(executables)
    assert executables["joy_node"].kwargs["arguments"] == ["--ros-args", "--log-level", "warn"]
    assert {"rearm_controller": "rl_flat_level"} in executables["joy_estop"].kwargs["parameters"]


def test_real_launch_without_controller_spawns_broadcasters_only(monkeypatch, ros_stubs):
    _, _, nodes, spawners, manager, _ = _expand_launch(
        monkeypatch, hardware="real", controller="none")
    assert _initial_state(manager)
    assert [names for names, _, _ in spawners] == [
        ["joint_state_broadcaster", "health_broadcaster", "imu_broadcaster"]]
    estop = next(n for n in nodes if n.kwargs["executable"] == "joy_estop")
    assert not any(isinstance(p, dict) and "rearm_controller" in p
                   for p in estop.kwargs["parameters"])


def test_armed_real_launch_keeps_everything_active(monkeypatch, ros_stubs):
    module, _, _, spawners, manager, logs = _expand_launch(
        monkeypatch, hardware="real", arm_hardware="true", controller="rl_agile_terrain")
    assert not _initial_state(manager)
    assert module.REALTIME in manager.kwargs["parameters"]
    assert [(names, inactive) for names, inactive, _ in spawners] == [
        (list(module.BROADCASTERS), False), (["rl_agile_terrain"], False),
        (["rl_flat_level", "rl_flat_level_fast", "rl_flat_level_drivetrain", "rl_flat_level_fast_drivetrain"], True)]
    assert "REAL SAFE STATE" not in logs


def test_simulation_resets_after_the_controller_spawner(monkeypatch, ros_stubs):
    module, actions, _, spawners, manager, _ = _expand_launch(
        monkeypatch, hardware="mujoco", controller="whole_body_controller")
    assert manager.kwargs["package"] == "mujoco_ros2_control"
    assert module.REALTIME not in manager.kwargs["parameters"]
    assert not _initial_state(manager)
    assert [(names, inactive) for names, inactive, _ in spawners] == [
        (list(module.BROADCASTERS), False), (["whole_body_controller"], False)]
    handler = next(a for a in actions if type(a).__name__ == "RegisterEventHandler")
    assert handler.args[0].kwargs["target_action"] is spawners[1][2]


def test_launches_list_the_health_broadcaster():
    for name in ("commissioning.launch.py", "imu_check.launch.py"):
        text = (ROOT / "launch" / name).read_text()
        assert '"health_broadcaster"' in text, name
        assert text.count('executable="spawner"') <= 2, name


def test_imu_check_keeps_only_the_imu_component(ros_stubs):
    module = _load(ROOT / "launch" / "imu_check.launch.py")
    urdf = ('<robot><link name="base_link"/>'
            '<ros2_control name="SandroSystem" type="system"><hardware/></ros2_control>'
            '<ros2_control name="SandroImu" type="sensor">\n<hardware/>\n</ros2_control></robot>')
    stripped = module.imu_only(urdf)
    assert "SandroSystem" not in stripped and 'name="SandroImu"' in stripped
    assert '<link name="base_link"/>' in stripped


def _deploy(name):
    path = REPO / "deploy" / "pi" / name
    if not path.is_file():
        pytest.skip("deploy/ is not part of this checkout")
    return path.read_text()


def test_pi_sync_builds_only_the_packages_a_change_needs():
    script = _deploy("pi")
    function = re.search(r"^change_scope\(\) \{.*?^\}", script, re.S | re.M).group(0)

    def scope(flags, path, *new_dirs):
        return subprocess.run(["bash", "-c", function + '\nchange_scope "$@"', "_",
                               flags, path, *new_dirs],
                              capture_output=True, text=True, check=True).stdout.strip()

    edited, new = ">f.st......", ">f+++++++++"
    assert scope(edited, "sandro_bringup/launch/sandro.launch.py") == "live"
    assert scope(edited, "sandro_bringup/config/controllers.yaml") == "live"
    assert scope(new, "sandro_bringup/sandro_bringup/helper.py") == "live"
    assert scope(new, "sandro_bringup/test/test_more.py") == "live"
    assert scope(new, "sandro_gui/sandro_gui/sub/x.py", "sandro_gui/sandro_gui/sub/") == "select"
    assert scope(new, "sandro_bringup/launch/other.launch.py") == "select"
    assert scope(new, "sandro_bringup/config/new.yaml") == "select"
    assert scope("*deleting", "sandro_bringup/config/old.yaml") == "select"
    assert scope("cd+++++++++", "sandro_gui/web/icons/") == "select"
    assert scope(edited, "sandro_hardware/src/gim6010_system.cpp") == "select"
    assert scope(edited, "sandro_gui/setup.py") == "select"
    assert scope(edited, "sandro_hardware/sandro_hardware_plugins.xml") == "select"
    assert scope(edited, "sandro_hardware/include/sandro_hardware/protocol.hpp") == "above"
    assert scope(edited, "sandro_controllers/CMakeLists.txt") == "above"
    assert scope(".d..t......", "sandro_bringup/") == "live"
    assert "-DBUILD_TESTING=OFF" in script and "-DCMAKE_BUILD_TYPE=Release" in script
    assert "--packages-select" in script and "--packages-up-to" in script


def test_pi_container_stops_gracefully_and_keeps_logs_off_the_sd_card():
    script, panel = _deploy("pi"), _deploy("panel.sh")
    assert "--stop-timeout 30" in script
    assert "--tmpfs /var/log/ros" in script and "ROS_LOG_DIR=/var/log/ros" in script
    assert "ROS_LOG_DIR=/var/log/ros" in _deploy("Dockerfile")
    stop = panel.split("stop() {", 1)[1].split("\n}", 1)[0]
    # Waits for the launch to exit, within docker's 30 s, instead of a fixed sleep.
    assert "pgrep -f" in stop and "sleep 5" not in stop
    assert "max_log_bytes" in panel
    assert subprocess.run(["bash", "-n", REPO / "deploy" / "pi" / "pi"]).returncode == 0
    assert subprocess.run(["bash", "-n", REPO / "deploy" / "pi" / "panel.sh"]).returncode == 0


def test_dockerignore_keeps_the_heavy_directories_out():
    path = REPO / ".dockerignore"
    if not path.is_file():
        pytest.skip(".dockerignore is not part of this checkout")
    ignored = {line.strip() for line in path.read_text().splitlines()}
    assert {"sandro_rl/runs", "sandro_rl/policies", "recordings", "onshape_export",
            "3d_models", "**/.venv"} <= ignored
    assert not {"ros2_ws", "ros2_ws/src", "deploy"} & ignored
    assert (REPO / "Dockerfile").read_text().count("-DCMAKE_BUILD_TYPE=Release") == 2
