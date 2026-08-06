#!/bin/bash
# Reboot loop, or connected-but-silent? Repeated create_client events mean the
# board keeps restarting its session; exactly one means it connects and stalls.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }

S systemctl restart micro-ros-agent
echo "=== WATCHING 90s ==="
sleep 90

echo "=== SESSION EVENTS IN WINDOW ==="
journalctl -u micro-ros-agent --since '-95 sec' --no-pager 2>&1 \
  | grep -cE 'create_client' | sed 's/^/create_client count: /'
journalctl -u micro-ros-agent --since '-95 sec' --no-pager 2>&1 \
  | grep -cE 'create_datawriter' | sed 's/^/create_datawriter count: /'
journalctl -u micro-ros-agent --since '-95 sec' --no-pager 2>&1 \
  | grep -iE 'delete|destroy' | tail -5

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "=== TOPIC INFO ==="
timeout 15 ros2 topic info /odom_raw 2>&1 | head -4
timeout 15 ros2 topic info /imu 2>&1 | head -4
echo "=== HZ ATTEMPT ==="
timeout 20 ros2 topic hz /odom_raw 2>&1 | head -3
echo "=== LOOP DIAG DONE ==="
