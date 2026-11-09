"""The panel's pure parts: parsing, command building, decoding, the model and the checks."""

import math
import re
import struct
import threading
from pathlib import Path

import pytest
import yaml

from sandro_gui import can_monitor, control, gamepad, logbook, model, system
from sandro_gui.errors import Busy, Conflict
from sandro_gui.launcher import LaunchSettings, build_command, died_process, is_launch, prune
from sandro_gui.panel import controller_catalog, health, readiness, stack_state

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


def test_stack_command_preloads_the_controller_and_never_arms():
    command = build_command({"mode": "stack", "controller": "whole_body_controller",
                             "teleop": True, "overrides": True}, settings())
    assert command[:5] == ["ros2", "launch", "sandro_bringup", "sandro.launch.py", "hardware:=real"]
    assert "arm_hardware:=false" in command and "controller:=whole_body_controller" in command
    assert "teleop:=true" in command and "extra_params:=/cfg/overrides.yaml" in command
    assert "imu_spi_device:=/dev/spidev6.0" in command
    assert build_command({}, settings()) == build_command({"mode": "stack"}, settings())
    quiet = build_command({"controller": "rl_flat_level", "teleop": False}, settings())
    assert "controller:=rl_flat_level" in quiet and "teleop:=false" in quiet


def test_imu_check_simulation_and_validation():
    imu = build_command({"mode": "imu", "extra": "imu_mount_yaw_rad:=0"}, settings())
    assert imu == ["ros2", "launch", "sandro_bringup", "imu_check.launch.py",
                   "imu_transport:=spi", "imu_spi_device:=/dev/spidev6.0", "imu_mount_yaw_rad:=0"]
    sim = build_command({"hardware": "mujoco", "controller": "lqr_balancer",
                         "overrides": False}, settings())
    assert "hardware:=mujoco" in sim and "controller:=lqr_balancer" in sim
    assert not any(a.startswith(("arm_hardware", "extra_params")) for a in sim)
    for bad in ({"extra": "rm -rf"}, {"controller": "a;b"}, {"mode": "fly"}, {"mode": "armed"},
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
                             "run": {"uptime_s": 60, "request": {"mode": "stack"}}})
    controller = {c["id"]: c for c in health(slow, {}, {}, False)}["controller"]
    assert controller["level"] == "error" and controller["value"] == "controller manager down"
    early = _snapshot(launch={"state": "running", "run": {"uptime_s": 5, "request": {"mode": "stack"}}})
    assert {c["id"]: c for c in health(early, {}, {}, False)}["controller"]["level"] == "off"
    # A `process has died` line in the launch output says so at once, not after 20 s.
    died = _snapshot(launch={"state": "starting", "run": {
        "uptime_s": 3, "request": {"mode": "stack"}, "died": ["ros2_control_node"]}})
    checks = {c["id"]: c for c in health(died, {}, {}, False)}
    assert checks["controller"]["level"] == "error"
    assert checks["stack"]["phase"] == "died" and checks["stack"]["level"] == "error"


def test_died_lines_name_the_process():
    assert died_process("[ERROR] [ros2_control_node-3]: process has died [pid 42, exit code -6, "
                        "cmd '...']") == "ros2_control_node"
    assert died_process("[INFO] [spawner-5]: process has finished cleanly [pid 50]") is None


def test_checks_accept_a_healthy_running_stack():
    nodes = [{"node": n, "joint": f"j{n}", "heartbeat_age_s": 0.05, "error_text": "",
              "temperature": 35} for n in range(6)]
    slow = _snapshot(launch={"state": "running", "run": {"uptime_s": 30, "request": {"mode": "stack"}}},
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


# Launch readiness and the stack's state -----------------------------------

BROADCASTERS = ["joint_state_broadcaster", "imu_broadcaster", "health_broadcaster"]


def _ros(hardware: dict, controllers: dict, manager: bool = True) -> dict:
    return {"manager": manager,
            "hardware": [{"name": n, "state": s} for n, s in hardware.items()],
            "controllers": [{"name": n, "state": s, "type": "", "claimed": ["x/effort"]
                             if s == "active" and "broadcaster" not in n else [],
                             "owns_commands": s == "active" and "broadcaster" not in n}
                            for n, s in controllers.items()]}


def test_a_stack_is_ready_with_drives_configured_broadcasters_up_and_the_controller_loaded():
    request = {"mode": "stack", "controller": "rl_flat_level"}
    up = {b: "active" for b in BROADCASTERS}
    assert readiness(request, None, broadcasters=BROADCASTERS) == ["the controller manager"]
    waiting = readiness(request, _ros({"SandroImu": "active", "SandroSystem": "unconfigured"},
                                      {"imu_broadcaster": "active"}), broadcasters=BROADCASTERS)
    assert waiting == ["SandroSystem configured (it is unconfigured)",
                       "joint_state_broadcaster active", "health_broadcaster active",
                       "rl_flat_level loaded"]
    ready = _ros({"SandroImu": "active", "SandroSystem": "inactive"},
                 dict(up, rl_flat_level="inactive"))
    assert readiness(request, ready, broadcasters=BROADCASTERS) == []
    # Armed at the start (arm_hardware:=true) is as ready.
    armed = _ros({"SandroImu": "active", "SandroSystem": "active"}, dict(up, rl_flat_level="active"))
    assert readiness(request, armed, broadcasters=BROADCASTERS) == []
    imu = _ros({"SandroImu": "active"}, {"imu_broadcaster": "active"})
    assert readiness({"mode": "imu"}, imu, broadcasters=BROADCASTERS) == []
    sim = _ros({"SandroSystem": "active"}, dict(up, lqr_balancer="active"))
    assert readiness({"hardware": "mujoco", "controller": "lqr_balancer"}, sim,
                     broadcasters=BROADCASTERS) == []


def test_stack_state_never_says_armed_after_an_estop():
    launch = {"state": "running", "run": {"uptime_s": 30, "request": {"mode": "stack"}}}
    down = _ros({"SandroImu": "active", "SandroSystem": "unconfigured"}, {})
    state = stack_state(launch, down, {"estopped": True}, False)
    assert state["phase"] == "estopped" and "armed" not in state["value"].replace("E-stopped", "")
    assert state["short"] == "E-stopped"
    idle = stack_state(launch, _ros({"SandroSystem": "inactive"}, {}), {}, False)
    assert idle["phase"] == "not_armed" and idle["level"] == "ok"
    assert stack_state(launch, _ros({"SandroSystem": "active"}, {}), {}, False)["phase"] == "armed"
    broken = stack_state(launch, down, {}, False)
    assert broken["phase"] == "unconfigured" and broken["level"] == "warn"
    starting = stack_state({"state": "starting", "run": {"uptime_s": 2, "request": {}}}, None, {},
                           False, waiting=["SandroImu active"])
    assert starting["phase"] == "starting" and "SandroImu active" in starting["detail"]


# Controllers ----------------------------------------------------------------

def test_fallen_controllers_are_read_from_their_diagnostics():
    rl = "sandro_controllers/RlPolicyController"
    assert control.condition(rl, {"level": 1, "values": {"phase": 0}}) is None
    assert control.condition(rl, {"level": 2, "values": {"phase": 2.0, "status_mask": 2.0}}
                             )["state"] == "fallen"
    oscillation = control.condition(rl, {"level": 2, "values": {"phase": 2, "status_mask": 66}})
    assert oscillation["state"] == "fallen" and "RL_OSCILLATION" in oscillation["text"]
    wbc = "sandro_controllers/WholeBodyController"
    assert control.condition(wbc, {"level": 2, "values": {"status_mask": 4.0}})["state"] == "fallen"
    stale = control.condition(wbc, {"level": 2, "message": "imu_stale",
                                    "values": {"status_mask": 2.0}})
    assert stale == {"state": "error", "text": "imu_stale"}
    lqr = "sandro_controllers/LqrBalancerController"
    assert control.condition(lqr, {"level": 2, "values": {"saturation_mask": 512}})["state"] == \
        "fallen"
    # And the Controller check turns red with it.
    slow = _snapshot(launch={"state": "running", "run": {"uptime_s": 30, "request": {}}},
                     ros=_ros({"SandroSystem": "active"}, {"rl_flat_level": "active"}))
    slow["ros"]["controllers"][0]["fault"] = {"state": "fallen", "text": "fallen"}
    check = {c["id"]: c for c in health(slow, {}, {}, False)}["controller"]
    assert check["level"] == "error" and check["short"] == "fallen"


class FakeManager:
    """A controller manager that records the calls control.py makes."""

    estop_service, clear_service = "/gim6010_estop/trigger", "/gim6010_estop/clear_errors"

    def __init__(self, controllers: dict, hardware: str = "active") -> None:
        self.states = dict(controllers)
        self.types = {"whole_body_controller": "sandro_controllers/WholeBodyController",
                      "rl_flat_level": "sandro_controllers/RlPolicyController",
                      "rl_agile_terrain": "sandro_controllers/RlPolicyController",
                      "timed_effort": "sandro_controllers/TimedEffortController"}
        self.hardware = {"SandroSystem": hardware}
        self.calls = []
        self.handover_ok = True

    def controllers(self, timeout=5.0):
        self.calls.append(("controllers",))
        return {n: {"name": n, "type": self.types.get(n, "broadcaster"), "state": s,
                    "claimed": ["x/effort"] if s == "active" and n in self.types else []}
                for n, s in self.states.items()}

    def hardware_states(self, timeout=5.0):
        return dict(self.hardware)

    def load(self, name):
        self.calls.append(("load", name))
        self.states[name] = "unconfigured"

    def configure(self, name):
        self.calls.append(("configure", name))
        self.states[name] = "inactive"

    def switch(self, activate, deactivate, strict=True, timeout=5.0):
        self.calls.append(("switch", list(activate), list(deactivate)))
        for name in deactivate:
            self.states[name] = "inactive"
        for name in activate:
            self.states[name] = "active"

    def set_hardware(self, name, active):
        self.calls.append(("set_hardware", name, active))
        self.hardware[name] = "active" if active else "inactive"
        return self.hardware[name]

    def trigger(self, service, timeout=3.0, wait=1.0):
        self.calls.append(("trigger", service))
        return (self.handover_ok if service.endswith("arm_handover") else True), "ok"


def test_swapping_a_balancer_for_a_policy_arms_its_handover_right_before_the_switch():
    manager = FakeManager({"whole_body_controller": "active", "rl_flat_level": "inactive"})
    ctl = control.Control(manager, logbook.Logbook())
    with pytest.raises(Conflict):
        ctl.activate("rl_flat_level")
    ctl.activate("rl_flat_level", swap=True)
    assert manager.calls[-2:] == [("trigger", "/rl_flat_level/arm_handover"),
                                  ("switch", ["rl_flat_level"], ["whole_body_controller"])]
    # Policy to balancer, or nothing running: no handover.
    ctl.activate("whole_body_controller", swap=True)
    assert ("trigger", "/whole_body_controller/arm_handover") not in manager.calls
    fresh = FakeManager({"rl_agile_terrain": "inactive"})
    control.Control(fresh, logbook.Logbook()).activate("rl_agile_terrain")
    assert not any(c[0] == "trigger" for c in fresh.calls)
    # A refused handover switches nothing: the running balancer keeps the robot.
    refused = FakeManager({"whole_body_controller": "active"})
    refused.handover_ok = False
    with pytest.raises(RuntimeError, match="keeps the robot"):
        control.Control(refused, logbook.Logbook()).activate("rl_flat_level", swap=True)
    assert ("load", "rl_flat_level") in refused.calls
    assert not any(c[0] == "switch" for c in refused.calls)


def test_a_controller_never_starts_on_disarmed_drives():
    manager = FakeManager({"whole_body_controller": "inactive"}, hardware="inactive")
    ctl = control.Control(manager, logbook.Logbook())
    with pytest.raises(RuntimeError, match="arm the drives first"):
        ctl.activate("whole_body_controller")
    assert not any(c[0] == "switch" for c in manager.calls)
    ctl.arm_and_start("whole_body_controller")
    assert [c for c in manager.calls if c[0] in ("set_hardware", "switch")] == [
        ("set_hardware", "SandroSystem", True), ("switch", ["whole_body_controller"], [])]


def test_estop_fires_the_can_frame_first_and_the_service_in_parallel_with_the_controllers():
    order = []

    class Slow(FakeManager):
        fired = threading.Event()

        def controllers(self, timeout=5.0):
            # Deactivation must not hold the drives' E-stop back.
            assert self.fired.wait(2.0), "the CAN E-stop waited for the controller listing"
            order.append("controllers")
            return super().controllers(timeout)

        def trigger(self, service, timeout=3.0, wait=1.0):
            order.append("service")
            self.fired.set()
            return super().trigger(service, timeout, wait)

    manager = Slow({"whole_body_controller": "active", "imu_broadcaster": "active"})
    ctl = control.Control(manager, logbook.Logbook())
    message = ctl.emergency_stop(lambda: order.append("frame") or "frame sent")
    assert order[0] == "frame" and order.index("service") < order.index("controllers")
    assert manager.states["whole_body_controller"] == "inactive"
    assert manager.states["imu_broadcaster"] == "active"
    assert message.startswith("frame sent") and "deactivated whole_body_controller" in message
    assert ctl.last_controller == "whole_body_controller"


def test_an_estop_during_a_start_aborts_it():
    manager = FakeManager({"rl_flat_level": "inactive"})
    ctl = control.Control(manager, logbook.Logbook())
    original = manager.configure

    def configure_then_estop(name):
        original(name)
        ctl.emergency_stop()

    manager.states.pop("rl_flat_level")
    manager.configure = configure_then_estop
    with pytest.raises(RuntimeError, match="E-STOP"):
        ctl.activate("rl_flat_level")
    assert ("switch", ["rl_flat_level"], []) not in manager.calls


def test_arm_restarts_broadcasters_and_disarm_stops_controllers_first():
    manager = FakeManager({"joint_state_broadcaster": "inactive", "imu_broadcaster": "active",
                           "whole_body_controller": "inactive"}, hardware="unconfigured")
    ctl = control.Control(manager, logbook.Logbook(),
                          broadcasters=["joint_state_broadcaster", "imu_broadcaster"])
    ctl.arm_and_start("whole_body_controller")
    assert manager.calls.index(("set_hardware", "SandroSystem", True)) < \
        manager.calls.index(("switch", ["joint_state_broadcaster"], [])) < \
        manager.calls.index(("switch", ["whole_body_controller"], []))
    ctl.disarm()
    assert manager.calls[-2:] == [("switch", [], ["whole_body_controller"]),
                                  ("set_hardware", "SandroSystem", False)]

    class Refusing(FakeManager):
        def set_hardware(self, name, active):
            raise RuntimeError(f"{name} stays unconfigured")

    with pytest.raises(control.ArmFailed, match="did not arm"):
        control.Control(Refusing({}), logbook.Logbook()).arm()


def test_conflicts_and_busy_actions_answer_409():
    assert Busy("x").http_status == 409 and Conflict(["a"]).http_status == 409


def test_controller_catalog_knows_which_policy_starts_lying():
    catalog = controller_catalog(BRINGUP / "config" / "controllers.yaml")
    assert catalog["rl_stand_up"] == {"type": "RlPolicyController", "start_lying": True}
    assert catalog["rl_flat_level"]["start_lying"] is False
    assert catalog["whole_body_controller"]["type"] == "WholeBodyController"


# CAN E-stop and drive faults --------------------------------------------------

def test_estop_frame_matches_the_estop_node():
    can_id, length, data = struct.unpack("=IB3x8s", can_monitor.estop_frame(3))
    assert (can_id, length, data) == ((3 << 5) | 0x02, 8, bytes(8))
    with pytest.raises(ValueError):
        can_monitor.estop_frame(64)
    bus = can_monitor.EstopBus("can-that-does-not-exist", [1, 2])
    assert "failed" in bus.send()


def test_arm_failures_say_which_drive_and_whether_it_clears():
    nodes = [{"node": 3, "joint": "right_wheel", "heartbeat_age_s": 0.05,
              "axis_error": 0x104000, "fault_flags": 0},
             {"node": 4, "joint": "left_motor_outer", "heartbeat_age_s": 0.05,
              "axis_error": 0x800, "fault_flags": 0},
             {"node": 6, "joint": "left_wheel", "heartbeat_age_s": None},
             {"node": 1, "joint": "right_motor_inner", "heartbeat_age_s": 0.05, "axis_error": 0}]
    faults = can_monitor.drive_faults(nodes)
    assert len(faults) == 3
    assert faults[0].startswith("right_wheel (node 3) 0x104000") and ": clearable" in faults[0]
    assert "left_motor_outer (node 4) 0x800" in faults[1] and "not clearable" in faults[1]
    assert "left_wheel (node 6) is silent" in faults[2]
    assert can_monitor.operator_clearable(0x4000, 0) and not can_monitor.operator_clearable(0x4000, 1)


# Housekeeping ---------------------------------------------------------------

def test_run_logs_are_capped_by_count_and_size(tmp_path):
    paths = []
    for i in range(6):
        path = tmp_path / f"run-20261109-10000{i}.log"
        path.write_bytes(b"x" * 100)
        paths.append(path)
    assert prune(paths, keep=4, keep_bytes=10_000) == paths[:2]
    assert prune(paths, keep=30, keep_bytes=250) == paths[:4]
    # The running launch's log is never deleted, whatever its size.
    assert paths[-1] not in prune(paths, keep=1, keep_bytes=0, current=paths[-1])


def test_link_counters_come_from_sysfs(tmp_path):
    base = tmp_path / "can0"
    (base / "statistics").mkdir(parents=True)
    (base / "operstate").write_text("up\n")
    (base / "flags").write_text("0x40c1\n")
    for name in system.COUNTERS:
        (base / "statistics" / name).write_text("12\n")
    counters = system.link_counters("can0", str(tmp_path))
    assert counters["operstate"] == "UP" and counters["admin_up"] and counters["rx_packets"] == 12
    assert system.link_counters("can1", str(tmp_path)) is None


def test_model_cache_depends_on_the_meshes():
    files = model.sources(DESCRIPTION / "mjcf" / "robot_full_floating.xml")
    assert files[0].name == "robot_full_floating.xml"
    assert any(f.suffix.lower() == ".stl" for f in files[1:])
