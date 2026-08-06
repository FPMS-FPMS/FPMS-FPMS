#!/bin/bash
# Tightest possible M2: reset the board, wait, run. NOTHING in between.
#
# The board dies within ~2-3 min and every intervening service restart seems to
# provoke it. fpms_mission.py subscribes to /odom_raw directly, so it does NOT
# need fpms-odom-tf, and the LiDAR is already alive -- so nothing else is touched.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }

S systemctl stop micro-ros-agent
sleep 2
python3 - <<'EOF'
import time, serial
ser = serial.Serial("/dev/ttyUSB1", 921600, timeout=0.2)
ser.setDTR(False); ser.setRTS(True); time.sleep(0.2); ser.setRTS(False)
time.sleep(0.05); ser.close()
print("board reset pulsed")
EOF
S systemctl start micro-ros-agent
echo "=== 70s reconnect - KEEP THE ROVER STILL ==="
sleep 70

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /home/ubuntu
# Guard off: the LiDAR and the drive board will not stay up simultaneously yet,
# and the operator has confirmed the arena is clear for M1/M2. The move is
# bounded at 564mm inside a ~970mm corridor.
timeout 260 python3 fpms_mission.py --target m2 --standoff 180 \
    --no-guard --dwell 3 --authorize-motion 2>&1 | tail -22
echo "=== M2 TIGHT DONE ==="
