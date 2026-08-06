#!/bin/bash
# Run the agent in the foreground at max verbosity and classify what the board
# actually sends. WRITE_DATA submessages mean it is publishing; only HEARTBEAT
# and session traffic means the control timer never fires.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }
S systemctl stop micro-ros-agent
sleep 2

source /opt/ros/humble/setup.bash
source /home/ubuntu/uros_ws/install/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
DEV=/dev/serial/by-path/platform-fc880000.usb-usb-0:1.3:1.0-port0

timeout 40 ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEV" -b 921600 -v6 \
    > /home/ubuntu/agent_v6.log 2>&1

echo "=== MESSAGE TYPE COUNTS ==="
for k in WRITE_DATA DATA HEARTBEAT ACKNACK CREATE GET_INFO DELETE; do
    printf "%-12s %s\n" "$k" "$(grep -c "$k" /home/ubuntu/agent_v6.log)"
done
echo "=== SAMPLE (last 25 lines) ==="
tail -25 /home/ubuntu/agent_v6.log | cut -c1-160
echo "=== VERBOSE DONE ==="
