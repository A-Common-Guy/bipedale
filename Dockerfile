FROM ros:jazzy-ros-base-noble AS base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      can-utils \
      git \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-jazzy-controller-manager \
      ros-jazzy-diagnostic-updater \
      ros-jazzy-joy \
      ros-jazzy-joint-state-broadcaster \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-rviz2 \
      ros-jazzy-teleop-twist-joy \
      ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Jazzy has no binary in the configured apt repository. Use the maintained
# ros-controls implementation, pinned for reproducible builds.
# Third-party sources stay outside ros2_ws/src: compose bind-mounts that
# directory for live development, which would hide the clone and leave every
# --symlink-install link in install/ dangling.
ARG MUJOCO_ROS2_CONTROL_REV=58f465b1b4634a6f366e71919189574f5d95c1e8
WORKDIR /workspaces/sandro
COPY ros2_ws/src/ ros2_ws/src/
RUN git clone https://github.com/ros-controls/mujoco_ros2_control.git \
      /opt/sandro_deps/mujoco_ros2_control \
    && git -C /opt/sandro_deps/mujoco_ros2_control checkout "${MUJOCO_ROS2_CONTROL_REV}" \
    && touch /opt/sandro_deps/mujoco_ros2_control/mujoco_ros2_control_demos/COLCON_IGNORE \
             /opt/sandro_deps/mujoco_ros2_control/mujoco_ros2_control_tests/COLCON_IGNORE
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths ros2_ws/src /opt/sandro_deps --ignore-src -r -y \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --base-paths ros2_ws/src /opt/sandro_deps --symlink-install \
    && rm -rf /var/lib/apt/lists/*

FROM base AS development
COPY . .
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --base-paths ros2_ws/src --symlink-install

RUN printf '%s\n' \
    'source /opt/ros/jazzy/setup.bash' \
    '[ -f /workspaces/sandro/install/setup.bash ] && source /workspaces/sandro/install/setup.bash' \
    >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
