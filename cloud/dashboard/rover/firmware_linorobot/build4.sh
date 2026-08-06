#!/bin/bash
# Domain fix: rebuild, reflash, verify in domain 20, then full pre-flight.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }
PENV="$HOME/.platformio/penv"
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble

python3 /home/ubuntu/patch4.py || exit 1

S systemctl stop fpms-teleop
S systemctl stop micro-ros-agent
sleep 2

cd /home/ubuntu/lino/firmware || exit 1
echo "=== BUILD4 ==="
"$PENV/bin/pio" run -e fpms 2>&1 | tail -12
echo "=== UPLOAD4 ==="
"$PENV/bin/pio" run -e fpms -t upload 2>&1 | tail -8

S systemctl start micro-ros-agent
echo "=== WAIT 60s ==="
sleep 60

export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "=== DOMAIN 20 GRAPH ==="
timeout 45 python3 /home/ubuntu/graph.py 2>&1 | tail -20
echo "=== PREFLIGHT ==="
timeout 90 python3 /home/ubuntu/preflight.py 2>&1 | tail -40
echo "=== BUILD4 DONE ==="
