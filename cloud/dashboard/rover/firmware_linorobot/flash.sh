#!/bin/bash
# Flash the new firmware.
#
# Order matters: fpms-teleop owns /cmd_vel, so it is stopped BEFORE the new
# firmware boots. Otherwise the first thing a freshly flashed board could see is
# a stale velocity command, with the rover on the floor. It is left stopped
# afterwards on purpose -- nothing commands motion until direction and encoder
# sign have been verified per wheel.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }
PENV="$HOME/.platformio/penv"
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble

echo "=== STOP CONSUMERS ==="
S systemctl stop fpms-teleop
S systemctl stop micro-ros-agent
sleep 2

echo "=== BACK UP STOCK FLASH BEFORE OVERWRITING ==="
python3 -m esptool --port /dev/ttyUSB1 --baud 460800 read_flash 0 0x400000 \
    /home/ubuntu/stock_backup_preflash.bin 2>&1 | tail -3
md5sum /home/ubuntu/stock_backup_preflash.bin

echo "=== UPLOAD ==="
cd /home/ubuntu/lino/firmware || exit 1
"$PENV/bin/pio" run -e fpms -t upload 2>&1 | tail -22

echo "=== RESTART AGENT ==="
S systemctl start micro-ros-agent
echo "=== WAIT 75s ==="
sleep 75
journalctl -u micro-ros-agent -n 6 --no-pager 2>&1 | tail -6

export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "=== TOPICS ==="
timeout 25 ros2 topic list 2>&1
echo "=== OBSERVE (no motion) ==="
timeout 40 python3 /home/ubuntu/crawl_test.py --mode observe 2>&1 | tail -6
echo "=== fpms-teleop LEFT STOPPED DELIBERATELY ==="
echo "=== FLASH DONE ==="
