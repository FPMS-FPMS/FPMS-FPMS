#!/bin/bash
# Clear the board's partial hang, then re-verify WITHOUT moving the rover.
#
# Deliberately does NOT run a turn test: heading has no absolute reference on
# this rover and a rotation self-test destroys the start pose that the static
# map->odom transform is pinned to.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }

S systemctl stop micro-ros-agent
sleep 2

python3 - <<'EOF'
import time, serial
ser = serial.Serial("/dev/ttyUSB1", 921600, timeout=0.2)
ser.setDTR(False); ser.setRTS(True); time.sleep(0.2); ser.setRTS(False)
time.sleep(0.05); ser.close()
print("DTR/RTS reset pulsed")
EOF

S systemctl start micro-ros-agent
echo "=== WAIT 60s - ROVER MUST STAY STILL (gyro bias calibration) ==="
sleep 60

# The odom->base_footprint publisher holds stale subscriptions across a board
# session change and will sit frozen otherwise.
S systemctl restart fpms-odom-tf
sleep 8

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /home/ubuntu
timeout 90 python3 mission_preflight.py 2>&1 | tail -16
echo "=== RESET DONE ==="
