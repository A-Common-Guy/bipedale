"""The panel's pure parts: parsing, command building, decoding, the model and the checks."""

import math
import re
import struct
from pathlib import Path

import pytest
import yaml

from sandro_gui import can_monitor, gamepad, logbook, model
from sandro_gui.launcher import LaunchSettings, build_command, is_launch
from sandro_gui.panel import health

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent
BRINGUP = SRC / "sandro_bringup"
DESCRIPTION = SRC / "sandro_description"


def settings(**extra):
    values = dict(hardware="real", args=["imu_transport:=spi", "imu_spi_device:=/dev/spidev6.0",
                                         "rviz:=false"],
                  simulation_args=["headless:=true"], overrides_path="/cfg/overrides.yaml",
                  available_hardware=("real", "mujoco"))
    values.update(extra)
    return LaunchSettings(**values)


def test_static_files_follow_symlinks_but_not_parent_paths(tmp_path):
    import io
    from types import SimpleNamespace
    from sandro_gui.server import Handler

    source, web = tmp_path / "src", tmp_path / "install" / "web"
    source.mkdir()
    web.mkdir(parents=True)
    (source / "index.html").write_text("<html>panel</html>")
    (web / "index.html").symlink_to(source / "index.html")
    (tmp_path / "secret.txt").write_text("no")

    def get(path):
        handler = Handler.__new__(Handler)
        handler.web, handler._cache, handler.command = web, {}, "GET"
        handler.headers = {}
        handler.wfile = io.BytesIO()
        sent = SimpleNamespace(status=None)
        handler.send_response = lambda status: setattr(sent, "status", status)
        handler.send_header = lambda *_: None
        handler.end_headers = lambda: None
        handler._static(path)
        return sent.status, handler.wfile.getvalue()

    assert get("/") == (200, b"<html>panel</html>")
    assert get("/../../secret.txt")[0] == 404
    assert get("/%2e%2e/%2e%2e/secret.txt")[0] == 404


def test_launch_lines_parse_into_level_source_and_text():
    p = logbook.parse_line
    assert p("[ros2_control_node-3] [ERROR] [1789812592.26] [GIM6010System]: DRIVE FAULT: x") == \
        (logbook.ERROR, "GIM6010System", "DRIVE FAULT: x", 1789812592.26)
    assert p("[ERROR] [ros2_control_node-3]: process has died [pid 42, exit code -6]") == \
        (logbook.ERROR, "ros2_control_node-3", "process has died [pid 42, exit code -6]", None)
    assert p("[INFO] [launch]: All log files can be found below /root/.ros/log")[0] == logbook.INFO
    assert p("\x1b[31m[WARN] [1.5] [joy_teleop]: running controller: none\x1b[0m") == \
        (logbook.WARN, "joy_teleop", "running controller: none", 1.5)
    assert p("[spawner-5] Traceback (most recent call last):")[0] == logbook.WARN
    assert p("[robot_state_publisher-1] plain output")[:2] == (logbook.INFO, "robot_state_publisher-1")


def test_log_drops_noise_and_keeps_node_times_on_replay():
    book = logbook.Logbook(ignore=["Unknown event type"])
    assert book.add_line("[joy_node-6] [INFO] [5.0] [joy_node]: Unknown event type 1623") is None
    first = book.add_line("[ros2_control_node-3] [ERROR] [100.5] [x]: fault", replay=True)
    after = book.add_line("[ros2_control_node-3] terminate called after throwing", replay=True)
    assert first["t"] == 100.5 and after["t"] == 100.5
    live = book.add_line("[ros2_control_node-3] more output")
    assert live["t"] > 1e9


def test_repeated_faults_fold_into_one_problem():
    book = logbook.Logbook()
    for age in (12, 13, 14):
        book.add_line(f"[ros2_control_node-3] [ERROR] [1.0] [BNO085Sensor]: IMU FAULT: no BNO085 "
                      f"report for {age} ms")
    book.add_line("[ros2_control_node-3] [INFO] [1.0] [x]: fine")
    problems = book.problems()
    assert len(problems) == 1 and problems[0]["count"] == 3
    assert problems[0]["text"].endswith("14 ms")
    assert [e["seq"] for e in book.since(2)] == [3, 4]
    book.clear_problems()
    assert book.problems() == []


def test_armed_command_matches_the_pi_launch():
    command = build_command({"mode": "armed", "controller": "whole_body_controller",
                             "teleop": True, "overrides": True}, settings())
    assert command[:5] == ["ros2", "launch", "sandro_bringup", "sandro.launch.py", "hardware:=real"]
    assert "arm_hardware:=true" in command and "controller:=whole_body_controller" in command
    assert "teleop:=true" in command and "extra_params:=/cfg/overrides.yaml" in command
    assert "imu_spi_device:=/dev/spidev6.0" in command


def test_unarmed_launch_never_starts_a_controller_or_teleop():
    command = build_command({"mode": "drives_off", "controller": "rl_flat_level", "teleop": True},
                            settings())
    assert "arm_hardware:=false" in command
    assert "controller:=none" in command and "teleop:=false" in command


def test_imu_check_simulation_and_validation():
    imu = build_command({"mode": "imu", "extra": "imu_mount_yaw_rad:=0"}, settings())
    assert imu == ["ros2", "launch", "sandro_bringup", "imu_check.launch.py",
                   "imu_transport:=spi", "imu_spi_device:=/dev/spidev6.0", "imu_mount_yaw_rad:=0"]
    sim = build_command({"hardware": "mujoco", "mode": "drives_off", "controller": "lqr_balancer",
                         "overrides": False}, settings())
    assert "hardware:=mujoco" in sim and "controller:=lqr_balancer" in sim
    assert not any(a.startswith(("arm_hardware", "extra_params")) for a in sim)
    for bad in ({"extra": "rm -rf"}, {"controller": "a;b"}, {"mode": "fly"},
                {"hardware": "mujoco"}):
        with pytest.raises(ValueError):
            build_command(bad, settings(available_hardware=("real",)) if bad.get("hardware")
                          else settings())


def test_only_sandro_launches_are_recognised():
    assert is_launch(["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "launch", "sandro_bringup",
                      "sandro.launch.py", "hardware:=real"])
    assert is_launch(["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "launch", "sandro_bringup",
                      "imu_check.launch.py"])
    assert not is_launch(["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "run", "sandro_gui",
                          "sandro_gui"])
    assert not is_launch(["/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "launch", "other",
                          "thing.launch.py"])


def test_can_frames_decode_like_the_hardware_plugin():
    heartbeat = struct.pack("<IBBbB", 0x104000, 8, 0, 37, 0)
    node, command, fields = can_monitor.decode((3 << 5) | 0x01, heartbeat)
    assert (node, command) == (3, can_monitor.HEARTBEAT)
    assert fields == {"axis_error": 0x104000, "axis_state": 8, "fault_flags": 0, "temperature": 37}
    assert can_monitor.axis_error_text(0x104000, 0) == "0x104000 estop_requested can_watchdog"
    node, command, fields = can_monitor.decode((5 << 5) | 0x09, struct.pack("<ff", 1.5, -2.0))
    assert (node, command, fields) == (5, can_monitor.ENCODER,
                                       {"position_rev": 1.5, "velocity_rev_s": -2.0})


def test_joint_mapping_comes_from_the_ros2_control_description():
    mapping = can_monitor.joint_mapping(DESCRIPTION / "urdf" / "sandro.ros2_control.xacro")
    assert {n: m["name"] for n, m in mapping.items()} == {
        1: "right_motor_inner", 2: "right_motor_outer", 3: "right_wheel",
        5: "left_motor_inner", 4: "left_motor_outer", 6: "left_wheel"}
    assert all(m["ratio"] == 8.0 for m in mapping.values())
    assert mapping[2]["sign"] == -1.0 and mapping[6]["sign"] == -1.0


def test_pad_sources_agree_on_named_state():
    # SDL via joy: L1 is button 9, triggers +1 released; left stick up is +1.
    buttons = [0] * 16
    buttons[9] = 1
    joy = gamepad.from_joy([0.0, 1.0, 0.0, 0.0, -1.0, 1.0], buttons)
    assert joy["buttons"]["l1"] and not joy["buttons"]["cross"]
    assert joy["ly"] == 1.0 and joy["l2"] == 1.0 and joy["r2"] == 0.0
    # joydev: L1 is button 4, y down positive, triggers -32767 released.
    js = gamepad.from_js([0, -32767, 32767, 0, 0, -32767, 0, -32767], [0, 0, 0, 0, 1])
    assert js["buttons"]["l1"] and js["buttons"]["up"]
    assert js["ly"] == pytest.approx(1.0) and js["l2"] == pytest.approx(1.0) and js["r2"] == 0.0


def test_guide_lists_every_configured_combination():
    teleop = yaml.safe_load((BRINGUP / "config" / "teleop.yaml").read_text())
    data = gamepad.guide(teleop)
    keys = {tuple(e["keys"]) for e in data["entries"]}
    t = teleop["joy_teleop"]["ros__parameters"]
    deadman = gamepad.SDL_BUTTONS[t["deadman_button"]]
    assert (deadman,) in keys
    assert (deadman, gamepad.SDL_BUTTONS[t["switch_button"]]) in keys
    assert (deadman, gamepad.SDL_BUTTONS[t["recentre_button"]]) in keys
    assert (gamepad.SDL_BUTTONS[t["announce_button"]],) in keys
    assert tuple(gamepad.SDL_AXES[i] for i in teleop["joy_estop"]["ros__parameters"]["axes"]) in keys
    assert [item["policy"] for item in data["lightbar"]][:len(t["policies"])] == t["policies"]
    assert "| hold L1 |" in gamepad.guide_markdown(data)


def test_user_guide_table_is_current():
    teleop = yaml.safe_load((BRINGUP / "config" / "teleop.yaml").read_text())
    table = gamepad.guide_markdown(gamepad.guide(teleop))
    assert table in (ROOT / "docs" / "USER_GUIDE.md").read_text(), \
        "regenerate the table: python3 -m sandro_gui.main --print-guide"


@pytest.fixture(scope="module")
def robot_model():
    return model.build(DESCRIPTION / "mjcf" / "robot_full_floating.xml")


def test_model_cad_angles_match_the_ros2_joint_references(robot_model):
    description, blob = robot_model
    references = dict(re.findall(r'<joint name="(\w+_motor_\w+)"[^>]*ref="([-\d.e]+)"',
                                 (DESCRIPTION / "mjcf" / "robot_ros2.xml").read_text()))
    for side in ("right", "left"):
        leg = description["legs"][side]
        assert leg["theta_cad"][0] == pytest.approx(float(references[f"{side}_motor_inner"]), abs=1e-4)
        assert leg["theta_cad"][1] == pytest.approx(float(references[f"{side}_motor_outer"]), abs=1e-4)
        axle = model.forward_kinematics(*leg["theta_cad"])
        body = (leg["inner_pivot"][0] + leg["direction"] * axle[0], leg["inner_pivot"][2] - axle[1])
        assert math.dist(body, (leg["wheel_cad"][0], leg["wheel_cad"][2])) < 1e-3
    assert description["legs"]["right"]["wheel_on"] == "outer"
    assert description["legs"]["left"]["wheel_on"] == "inner"
    assert len(blob) == description["bytes"] and description["triangles"] > 10000
    kinds = {g["kind"] for g in description["groups"].values()}
    assert kinds == {"torso", "crank", "lower", "wheel"}


def test_model_chip_is_yawed_in_its_holder(robot_model):
    w, x, y, z = robot_model[0]["imu_chip_quat"]
    assert abs(x) < 1e-3 and abs(y) < 1e-3
    assert math.degrees(2 * math.atan2(z, w)) == pytest.approx(90.0, abs=0.5)


def _snapshot(**slow):
    base = {"launch": {"state": "stopped"}, "can_link": {"present": True, "up": True,
                                                         "state": "ERROR-ACTIVE", "bitrate": 1000000},
            "can": {"frames_hz": 2400.0, "nodes": []}, "ros": {"manager": False}, "pad": {},
            "system": {"cpu_temp_c": 50.0, "throttled": {"now": [], "since_boot": []}}}
    base.update(slow)
    return base


def test_checks_flag_a_silent_drive_and_a_bus_off():
    nodes = [{"node": n, "joint": f"j{n}", "heartbeat_age_s": 0.05, "error_text": ""} for n in range(5)]
    nodes.append({"node": 6, "joint": "left_wheel", "heartbeat_age_s": None, "error_text": ""})
    checks = {c["id"]: c for c in health(_snapshot(can={"frames_hz": 2000.0, "nodes": nodes}), {},
                                         {}, False)}
    assert checks["drives"]["level"] == "error" and "left_wheel (node 6)" in checks["drives"]["detail"]
    assert checks["can"]["level"] == "ok"
    bus_off = health(_snapshot(can_link={"present": True, "up": True, "state": "BUS-OFF"}), {}, {},
                     False)
    assert {c["id"]: c["level"] for c in bus_off}["can"] == "error"


def test_checks_flag_a_controller_manager_that_died_inside_the_launch():
    slow = _snapshot(launch={"state": "running",
                             "run": {"uptime_s": 60, "request": {"mode": "armed"}}})
    controller = {c["id"]: c for c in health(slow, {}, {}, False)}["controller"]
    assert controller["level"] == "error" and controller["value"] == "controller manager down"
    early = _snapshot(launch={"state": "running", "run": {"uptime_s": 5, "request": {"mode": "armed"}}})
    assert {c["id"]: c for c in health(early, {}, {}, False)}["controller"]["level"] == "off"


def test_checks_accept_a_healthy_running_stack():
    nodes = [{"node": n, "joint": f"j{n}", "heartbeat_age_s": 0.05, "error_text": "",
              "temperature": 35} for n in range(6)]
    slow = _snapshot(launch={"state": "running", "run": {"uptime_s": 30, "request": {"mode": "armed"}}},
                     can={"frames_hz": 6000.0, "nodes": nodes},
                     ros={"manager": True, "hardware": [{"name": "SandroSystem", "state": "active"}],
                          "controllers": [{"name": "whole_body_controller", "owns_commands": True}]},
                     pad={"device": "/dev/input/js0", "battery": {"percent": 80}})
    fast = {"ros": {"imu": {"q": (0, 0, 0, 1)}, "imu_age_s": 0.004, "imu_sensor_age_s": 0.003,
                    "joints_age_s": 0.003, "joints": {"a": {"sensor_age_s": 0.004}},
                    "rates": {"/imu_broadcaster/imu": {"hz": 298.0},
                              "/joint_states": {"hz": 300.0}, "/joy": {"age_s": 0.05}}}}
    expect = {"imu_max_age_ms": 25, "encoder_max_age_ms": 25}
    topic_hz = {"/imu_broadcaster/imu": 300, "/joint_states": 300}
    levels = {c["id"]: c["level"]
              for c in health(slow, fast, expect, False, topic_hz=topic_hz)}
    assert set(levels.values()) == {"ok"}, levels
    # A stale encoder at the driver is a warning even with the topic at full rate.
    fast["ros"]["joints"]["a"]["sensor_age_s"] = 0.08
    checks = {c["id"]: c for c in health(slow, fast, expect, False, topic_hz=topic_hz)}
    assert checks["encoders"]["level"] == "warn" and "80 ms" in checks["encoders"]["detail"]
