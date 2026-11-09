# Robot-side image for the Raspberry Pi (arm64): ros2_control + the Sandro packages.
# No MuJoCo, RViz or sandro_sim here; those stay on the development machine.
# Built once on the Pi by `deploy/pi/pi setup`. Sources are bind-mounted at run time,
# so day-to-day development never rebuilds this image.
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CCACHE_DIR=/workspaces/sandro/.ccache \
    CCACHE_MAXSIZE=2G

# bluez only for bluetoothctl: the panel pairs the gamepad through the host's
# bluetoothd over the D-Bus socket the container mounts; no daemon runs here.
RUN apt-get update && apt-get install -y --no-install-recommends \
      bluez \
      build-essential \
      ccache \
      can-utils \
      iproute2 \
      libeigen3-dev \
      python3-colcon-common-extensions \
      python3-numpy \
      python3-yaml \
      ros-jazzy-controller-manager \
      ros-jazzy-diagnostic-updater \
      ros-jazzy-joint-state-broadcaster \
      ros-jazzy-joy \
      ros-jazzy-realtime-tools \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-teleop-twist-joy \
      ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

COPY panel.sh /usr/local/bin/sandro-panel
RUN chmod +x /usr/local/bin/sandro-panel

WORKDIR /workspaces/sandro
ENTRYPOINT ["/ros_entrypoint.sh"]
# The control panel on port 8080 (deploy/pi/panel.sh); it waits for the first build.
CMD ["sandro-panel"]
