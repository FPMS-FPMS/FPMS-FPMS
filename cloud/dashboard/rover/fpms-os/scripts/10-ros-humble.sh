#!/usr/bin/env bash
# Stage 10 - ROS 2 Humble, Nav2, SLAM, bridges, and the micro-ROS agent.
#
# Humble is pinned, and not by preference:
#   - its rclpy extensions are cpython-310-aarch64 binaries, so the base must
#     be Ubuntu 22.04
#   - nav2_params.yaml uses progress_checker_plugin (SINGULAR), which Iron
#     renamed to progress_checker_plugins. The rename is silent - the old key
#     is simply ignored.
#   - Humble's controller_server has no odom_topic parameter; setting one on a
#     newer distro would change behaviour invisibly.
set -euo pipefail
echo "--- 10-ros-humble"

export DEBIAN_FRONTEND=noninteractive

curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list
apt-get update -qq

apt-get install -y -qq --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rclpy \
    ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-nav-msgs \
    ros-humble-geometry-msgs ros-humble-diagnostic-msgs \
    ros-humble-tf2-ros ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs ros-humble-tf2-msgs \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions python3-rosdep python3-vcstool

# Nav2. Installed and tuned, but NOT started at boot -- see fpms-nav2.service
# for the three preconditions that are unmet.
#
# Some of these are shipped only so the metapackage is complete and must never
# be launched:
#   nav2-velocity-smoother  "the single worst thing you could add to this stack"
#   nav2-dwb-controller, nav2-smac-planner, nav2-mppi-controller  all rejected
#     with reasons in nav2_params.yaml
#   robot-localization      "NOT RUN, and should not be"
apt-get install -y -qq --no-install-recommends \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander ros-humble-nav2-msgs \
    ros-humble-slam-toolbox

# Bridges. rosbridge is the browser's only way into ROS, and its params file
# whitelist is the safety mechanism (drive topics absent by construction).
apt-get install -y -qq --no-install-recommends \
    ros-humble-rosbridge-suite \
    ros-humble-foxglove-bridge

# Joystick packages: installed for characterisation, but NO UNIT LAUNCHES THEM.
# teleop_twist_joy publishes /cmd_vel DIRECTLY, so running it means two writers
# on /cmd_vel -- stop fpms-teleop first or you will have exactly the race the
# masked units exist to prevent.
apt-get install -y -qq --no-install-recommends \
    ros-humble-joy ros-humble-teleop-twist-joy

# NOT installed: ros-humble-robot-state-publisher / xacro. There is no URDF in
# this project; the whole tree below base_footprint is static_transform_publisher.
# Adding it would create a SECOND publisher on base_footprint->base_link->laser_frame,
# which TF_TREE.md forbids absolutely.
#
# NOT installed: python3-transforms3d / tf_transformations. It is deliberately
# avoided in fpms_missions.py because it was installed-and-broken on the old
# Pi. The rover computes quaternions inline, in two lines.

# --- micro-ROS agent --------------------------------------------------------
#
# NOT an apt package. micro-ros-agent.service sources
# /home/ubuntu/uros_ws/install/setup.bash under `set -e`, so if this workspace
# is not built the wrapper dies instantly and the drive link never comes up.
echo "--- building the micro-ROS agent workspace"
UROS_WS="${FPMS_HOME}/uros_ws"
rm -rf "$UROS_WS"; mkdir -p "$UROS_WS/src"

set +u; source /opt/ros/humble/setup.bash; set -u

git clone -q --depth 1 -b humble \
    https://github.com/micro-ROS/micro_ros_setup.git "$UROS_WS/src/micro_ros_setup"

rosdep init >/dev/null 2>&1 || true
rosdep update --rosdistro humble >/dev/null 2>&1 || true
(cd "$UROS_WS" && rosdep install --from-paths src --ignore-src -y -q >/dev/null 2>&1 || true)

(cd "$UROS_WS" && colcon build --symlink-install >/dev/null)
set +u; source "$UROS_WS/install/setup.bash"; set -u

(cd "$UROS_WS" && ros2 run micro_ros_setup create_agent_ws.sh >/dev/null)
(cd "$UROS_WS" && ros2 run micro_ros_setup build_agent.sh >/dev/null)

chown -R "${FPMS_USER}:${FPMS_USER}" "$UROS_WS"

# Verify, because the wrapper's `set -e` makes a missing setup.bash an
# instant, silent death of the entire drive link.
[ -f "$UROS_WS/install/setup.bash" ] \
    || { echo "FATAL: micro-ROS workspace did not build" >&2; exit 1; }

# --- shell environment ------------------------------------------------------
#
# The units set all of this explicitly and do NOT rely on the profile -- an
# interactive shell's environment must never be what makes a service work.
# This is for the operator SSHing in to debug, so that an ad-hoc
# `ros2 topic list` sees what the services see.
#
# Every ad-hoc `ros2 topic list` in this project's history that forgot
# ROS_DOMAIN_ID saw an EMPTY LIST and concluded the link was dead.
cat > /etc/profile.d/fpms-ros.sh <<'EOF'
# FPMS-OS interactive ROS environment.
# The services do NOT depend on this file -- they set everything in their unit.
# This exists so an operator's ad-hoc `ros2 topic list` sees the same world.
[ -f /opt/ros/humble/setup.bash ] && . /opt/ros/humble/setup.bash
[ -f /home/ubuntu/uros_ws/install/setup.bash ] && . /home/ubuntu/uros_ws/install/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fpms/fastdds_udp_only.xml
export ROS_LOCALHOST_ONLY=1
EOF

echo "--- 10-ros-humble OK"
