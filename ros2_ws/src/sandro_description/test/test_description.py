from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "tools"))
import sync_model  # noqa: E402
STABLE_JOINTS = {
    "right_motor_inner",
    "right_motor_outer",
    "right_wheel",
    "left_motor_inner",
    "left_motor_outer",
    "left_wheel",
}


def test_packaged_model_matches_export():
    result = subprocess.run(
        [sys.executable, str(PACKAGE / "tools" / "sync_model.py"), "check"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_canonical_mjcf_contract():
    root = ET.parse(PACKAGE / "mjcf" / "robot_full_floating.xml").getroot()
    actuator_joints = {node.attrib["joint"] for node in root.findall("./actuator/*")}
    assert actuator_joints == STABLE_JOINTS
    assert len(root.findall(".//site[@name='imu']")) == 1
    assert len(root.findall("./equality/connect")) == 4


def test_ros2_mjcf_uses_bounded_unit_gain_effort_actuators():
    root = ET.parse(PACKAGE / "mjcf" / "robot_ros2.xml").getroot()
    motors = root.findall("./actuator/motor")
    assert {motor.attrib["joint"] for motor in motors} == STABLE_JOINTS
    assert all(motor.attrib["forcerange"] == "-10 10" for motor in motors)
    assert len(root.findall("./equality/connect")) == 4
    # Canonical ctrl values are affine-actuator position setpoints; here ctrl is
    # torque, so a non-zero keyframe would inject effort before any controller.
    for key in root.findall("./keyframe/key"):
        assert set(key.attrib["ctrl"].split()) == {"0"}
    # qpos0 is the CAD pose, which is below the floor. Simulation starts from
    # this keyframe instead, so it has to survive model regeneration.
    assert [key.attrib["name"] for key in root.findall("./keyframe/key")] == ["stand"]
    scene = ET.parse(PACKAGE / "mjcf" / "scene_ros2.xml").getroot()
    assert scene.find("./include").attrib["file"] == "robot_ros2.xml"


def test_ros2_mjcf_timestep_divides_the_control_period():
    # The controller manager wakes on the first /clock tick past each period,
    # so any remainder slows the loop (2 ms steps give 250 Hz instead of 300).
    root = ET.parse(PACKAGE / "mjcf" / "robot_ros2.xml").getroot()
    timestep = float(root.find("./option").attrib["timestep"])
    steps = (1.0 / sync_model.CONTROL_RATE_HZ) / timestep
    assert abs(steps - round(steps)) < 1e-6
    assert round(steps) >= 2


def test_identified_payload_is_a_body_with_inertia():
    """A geom mass on the torso would be ignored: it carries an <inertial>."""
    identified = sync_model.load_identified()
    root = ET.parse(PACKAGE / "mjcf" / "robot_ros2.xml").getroot()
    body = root.find(".//body[@name='identified_payload']")
    mass = float((identified.get("payload") or {}).get("mass_kg", 0.0))
    if mass <= 0.0:
        assert body is None
        return
    assert body is not None
    inertial = body.find("./inertial")
    assert inertial is not None
    assert abs(float(inertial.attrib["mass"]) - mass) < 1e-9
    assert all(float(v) > 0.0 for v in inertial.attrib["diaginertia"].split())
    assert body.find("./joint") is None  # welded to the torso
    assert root.find(".//geom[@name='identified_payload']").attrib["contype"] == "0"


def _worldbody_joints(root):
    return {
        joint.attrib.get("name"): joint
        for joint in root.find("./worldbody").iter("joint")
        if joint.attrib.get("name")
    }


def test_ros2_mjcf_actuated_joints_use_the_logical_convention():
    """Both backends must report the same joint angles.

    The physical drives apply sign and zero offset in sandro_hardware; the
    simulated hinges carry the same convention through a flipped axis and a
    `ref`, so a controller sees zero-down five-bar angles and forward-positive
    wheels on either backend. The expected values follow from JOINT_FRAMES and
    the canonical export, so a regenerated export cannot silently change them.
    """
    canonical = ET.parse(PACKAGE / "mjcf" / "robot_full_floating.xml").getroot()
    ros2 = ET.parse(PACKAGE / "mjcf" / "robot_ros2.xml").getroot()
    canonical_joints = _worldbody_joints(canonical)
    ros2_joints = _worldbody_joints(ros2)
    assert set(sync_model.JOINT_FRAMES) == set(STABLE_JOINTS)
    for name, (sign, zero) in sync_model.JOINT_FRAMES.items():
        assert abs(sign) == 1.0
        raw_axis = [float(v) for v in canonical_joints[name].attrib.get("axis", "0 0 1").split()]
        axis = [float(v) for v in ros2_joints[name].attrib["axis"].split()]
        assert axis == [sign * v for v in raw_axis]
        expected_ref = -zero if sign > 0 else zero
        assert abs(float(ros2_joints[name].attrib["ref"]) - expected_ref) < 1e-7
        assert "ref" not in canonical_joints[name].attrib
    # The left wheel is the one hinge the export points the other way.
    assert sync_model.JOINT_FRAMES["right_wheel"][0] == 1.0
    assert sync_model.JOINT_FRAMES["left_wheel"][0] == -1.0

    canonical_qpos = [float(v) for v in canonical.find("./keyframe/key").attrib["qpos"].split()]
    ros2_qpos = [float(v) for v in ros2.find("./keyframe/key").attrib["qpos"].split()]
    assert len(canonical_qpos) == len(ros2_qpos)
    layout = dict(sync_model._qpos_layout(canonical))
    converted = set()
    for name in STABLE_JOINTS:
        index = layout[name]
        expected = sync_model.logical_joint_value(name, canonical_qpos[index])
        assert abs(ros2_qpos[index] - expected) < 1e-7, name
        converted.add(index)
    for index, (raw, generated) in enumerate(zip(canonical_qpos, ros2_qpos)):
        if index not in converted:
            assert raw == generated
    # In the stand pose both legs are close to symmetric in the logical frame.
    right = (ros2_qpos[layout["right_motor_inner"]], ros2_qpos[layout["right_motor_outer"]])
    left = (ros2_qpos[layout["left_motor_inner"]], ros2_qpos[layout["left_motor_outer"]])
    assert all(0.1 < angle < 0.6 for angle in right + left)


def test_xacro_files_are_well_formed_and_document_tree_limit():
    files = sorted((PACKAGE / "urdf").glob("*.xacro"))
    assert len(files) == 4
    for path in files:
        ET.parse(path)

    description = (PACKAGE / "urdf" / "sandro.urdf.xacro").read_text()
    assert "tree visualization" in description
    assert "source of physics" in description
    assert "<inertial>" not in description
    for joint in STABLE_JOINTS:
        # Macro-expanded side names are represented by the stable suffixes.
        assert joint.split("_", 1)[1] in description


def _expand(backend: str, *extra: str) -> ET.Element:
    result = subprocess.run(
        [
            "xacro", str(PACKAGE / "urdf" / "sandro.urdf.xacro"),
            f"control_backend:={backend}", "simulation_headless:=true", *extra,
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout)


def test_xacro_expands_for_visualization_and_both_backends():
    assert not _expand("none").findall("./ros2_control")

    mujoco = _expand("mujoco")
    assert len(mujoco.findall("./ros2_control")) == 1
    # The simulation backend is the MuJoCo system behind the latency decorator.
    assert mujoco.find("./ros2_control/hardware/plugin").text == "sandro_sim/LatencySystem"
    assert (
        mujoco.find(".//param[@name='inner_plugin']").text
        == "mujoco_ros2_control/MujocoSystemInterface"
    )
    for key in ("imu_delay_s", "imu_rate_hz", "joint_delay_s", "joint_rate_hz", "command_delay_s"):
        assert float(mujoco.find(f".//param[@name='{key}']").text) >= 0.0
    assert mujoco.find(".//param[@name='headless']").text == "true"
    assert mujoco.find(".//param[@name='initial_keyframe']").text == "stand"

    # Without latency emulation the wrapper stays (it also emulates the
    # drives' MIT loop, which the bare backend cannot) with every delay zero.
    direct = _expand("mujoco", "simulation_latency:=false")
    assert direct.find("./ros2_control/hardware/plugin").text == "sandro_sim/LatencySystem"
    for key in ("imu_delay_s", "imu_rate_hz", "joint_delay_s", "joint_rate_hz", "command_delay_s"):
        assert float(direct.find(f".//param[@name='{key}']").text) == 0.0
    # Every joint exposes the full MIT frame to the controllers.
    for backend in (mujoco, _expand("real")):
        for joint in backend.findall("./ros2_control/joint"):
            names = {node.get("name") for node in joint.findall("command_interface")}
            assert names == {"effort", "position", "velocity", "kp", "kd"}, joint.get("name")

    real = _expand("real")
    assert len(real.findall("./ros2_control")) == 2
    plugins = {node.text for node in real.findall("./ros2_control/hardware/plugin")}
    assert plugins == {
        "sandro_hardware/GIM6010System",
        "sandro_hardware/BNO085Sensor",
    }

    def imu_params(root):
        hardware = root.find("./ros2_control[@name='SandroImu']/hardware")
        return {node.get("name"): node.text for node in hardware.findall("param")}

    spi = imu_params(real)
    assert spi["transport"] == "spi"
    assert spi["spi_device"] == "/dev/spidev6.0"
    assert "port" not in spi
    uart = imu_params(_expand("real", "imu_transport:=uart", "imu_port:=/dev/ttyUSB1"))
    assert uart["transport"] == "uart"
    assert uart["port"] == "/dev/ttyUSB1"
    assert "spi_device" not in uart
