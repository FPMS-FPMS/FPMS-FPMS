#!/bin/bash
# Make the TF tree survive a reboot.
#
# map->odom is a STATIC transform at the rover's known start pose rather than
# AMCL/SLAM output: the arena is a featureless 1200mm box, which gives a scan
# matcher almost nothing to lock onto, and a Nav2 planning against a bad
# map->odom fails in ways that are very hard to debug. The cost is that drift is
# uncorrected, so accuracy rests on the measured-retrace and the final approach
# nudge -- which is exactly how the golden phase6 driver reached 0.6%.
#
# CONSEQUENCE: the rover MUST start each run at arena (972, 228) mm facing 90deg,
# because that pose is baked into this transform.
set -e
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }

S bash -c "cat > /etc/systemd/system/fpms-map-odom.service <<'EOF'
[Unit]
Description=FPMS static map->odom at the arena start pose
After=network.target

[Service]
Type=simple
User=ubuntu
Environment=ROS_DOMAIN_ID=20
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=HOME=/home/ubuntu
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && exec ros2 run tf2_ros static_transform_publisher --x 0.972 --y 0.228 --z 0.0 --yaw 1.5707963 --pitch 0.0 --roll 0.0 --frame-id map --child-frame-id odom'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF"

S bash -c "cat > /etc/systemd/system/fpms-tf.service <<'EOF'
[Unit]
Description=FPMS static TF tree (base_footprint->base_link->laser_frame/imu_frame)
After=network.target

[Service]
Type=simple
User=ubuntu
Environment=ROS_DOMAIN_ID=20
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=HOME=/home/ubuntu
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && exec ros2 launch /home/ubuntu/nav2/fpms_tf.launch.py'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF"

# Replace the ad-hoc processes started by hand earlier, or they will fight the
# units for ownership of the same transforms.
pkill -f 'static_transform_publisher --x 0.972' || true
pkill -f 'fpms_tf.launch.py' || true
sleep 2

S systemctl daemon-reload
S systemctl enable --now fpms-map-odom fpms-tf 2>&1 | tail -2
sleep 12
systemctl is-active fpms-map-odom fpms-tf mosquitto

echo "=== BRIDGE STATUS ==="
journalctl -u mosquitto -n 12 --no-pager 2>&1 | grep -iE 'bridge|connect|error' | tail -6

echo "=== TF UNITS DONE ==="
