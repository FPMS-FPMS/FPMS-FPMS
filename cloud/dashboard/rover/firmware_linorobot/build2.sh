#!/bin/bash
# Fix the penv, add the REAL ICM42670P gyro, rebuild.
PW="$1"
set -x
echo "$PW" | sudo -S apt-get install -y python3-venv 2>&1 | tail -3

PENV="$HOME/.platformio/penv"
if [ ! -x "$PENV/bin/pio" ]; then
    rm -rf "$PENV"
    python3 -m venv "$PENV" || exit 1
    "$PENV/bin/python" -m pip install -U pip 2>&1 | tail -2
    "$PENV/bin/python" -m pip install -U \
        platformio catkin_pkg lark lark-parser "empy==3.3.4" \
        colcon-common-extensions pyyaml 2>&1 | tail -4
fi
"$PENV/bin/pio" --version || exit 1

source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble

cp /home/ubuntu/icm42670_imu.h /home/ubuntu/lino/firmware/lib/imu/ || exit 1
python3 /home/ubuntu/patch2.py || exit 1
grep -n "USE_ICM42670_IMU\|ICM42670IMU" /home/ubuntu/lino/firmware/lib/imu/imu.h

cd /home/ubuntu/lino/firmware || exit 1
echo "=== BUILD2 START ==="
"$PENV/bin/pio" run -e fpms 2>&1 | tail -45
echo "=== BUILD2 RC=${PIPESTATUS[0]} ==="
ls -la .pio/build/fpms/firmware.bin 2>&1
echo "=== BUILD2 DONE ==="
