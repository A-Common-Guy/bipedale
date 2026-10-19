#!/usr/bin/env bash
# Run the real BNO085 plugin alone, with no drives in the picture, and print the IMU topic.
# The URDF is rendered for hardware:=real and every ros2_control block except SandroImu is
# stripped, so the GIM6010 plugin is never loaded and nothing can be armed.
#
# usage: imu_only.sh [xacro args...]     default: imu_transport:=spi imu_spi_device:=/dev/spidev6.0
# stop with Ctrl-C.
set -euo pipefail
ARGS=("$@")
[ ${#ARGS[@]} -gt 0 ] || ARGS=(imu_transport:=spi imu_spi_device:=/dev/spidev6.0)

share=$(ros2 pkg prefix sandro_bringup)/share
urdf=$(mktemp /tmp/sandro_imu_only.XXXXXX.urdf)
params=$(mktemp /tmp/sandro_imu_only.XXXXXX.yaml)
trap 'rm -f "$urdf" "$params"; trap - TERM INT EXIT; kill 0' EXIT TERM INT

xacro "$(ros2 pkg prefix sandro_description)/share/sandro_description/urdf/sandro.urdf.xacro" \
  control_backend:=real "${ARGS[@]}" |
python3 -c '
import re, sys
text = sys.stdin.read()
blocks = re.findall(r"<ros2_control\b.*?</ros2_control>", text, re.S)
for block in blocks:
    if "SandroImu" not in block.split(">", 1)[0]:
        text = text.replace(block, "")
sys.stdout.write(text)
' >"$urdf"
grep -c '<ros2_control' "$urdf" | sed 's/^/ros2_control blocks kept: /'

# A multi-line URDF does not survive `-p`; a JSON string is a valid YAML scalar.
python3 -c '
import json, sys
urdf = json.dumps(open(sys.argv[1]).read())
for node in ("controller_manager", "robot_state_publisher"):
    print(f"{node}:\n  ros__parameters:\n    robot_description: {urdf}")
' "$urdf" >"$params"

# Jazzy's controller manager waits for the /robot_description topic, not the parameter.
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$params" &
ros2 run controller_manager ros2_control_node --ros-args \
  --params-file "$share/sandro_bringup/config/controllers.yaml" \
  --params-file "$params" &
sleep 3
ros2 run controller_manager spawner imu_broadcaster \
  --controller-manager /controller_manager --controller-manager-timeout 20
ros2 topic echo /imu_broadcaster/imu --field orientation --qos-reliability best_effort
