#!/bin/bash
# Gyro fix + diagnostic: rebuild, reflash, read raw registers during a turn.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }
PENV="$HOME/.platformio/penv"
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble

cp /home/ubuntu/icm42670_imu.h /home/ubuntu/lino/firmware/lib/imu/ || exit 1
# fpms_config.h now carries the IMU_TWEAK macro, so it must be copied too --
# the macro has to exist before imu_interface.h is compiled.
cp /home/ubuntu/fpms_config.h /home/ubuntu/lino/config/custom/ || exit 1

S systemctl stop fpms-teleop
S systemctl stop micro-ros-agent
sleep 2

cd /home/ubuntu/lino/firmware || exit 1
echo "=== BUILD5 ==="
"$PENV/bin/pio" run -e fpms 2>&1 | tail -14
echo "=== UPLOAD5 ==="
"$PENV/bin/pio" run -e fpms -t upload 2>&1 | tail -8

S systemctl start micro-ros-agent
echo "=== WAIT 60s (rover must stay STILL: 40-sample gyro bias calibration) ==="
sleep 60

export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "=== GYRO DIAGNOSTIC ==="
timeout 90 python3 /home/ubuntu/gyro_diag.py --turn 2>&1 | tail -25
echo "=== BUILD5 DONE ==="
