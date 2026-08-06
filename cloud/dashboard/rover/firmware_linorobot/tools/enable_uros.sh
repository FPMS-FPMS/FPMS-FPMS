#!/bin/bash
# Restore the micro-ROS link end to end, and get battery voltage onto the
# dashboard on every boot.
#
# Flashes the MISSION firmware over the open-loop test sketch, with the
# operator-verified motor/encoder inversions compiled in. Nothing commands
# motion here -- this is link-up and telemetry only.
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }
PENV="$HOME/.platformio/penv"
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble

S systemctl stop micro-ros-agent fpms-missions fpms-teleop 2>/dev/null
sleep 2

cp /home/ubuntu/fpms_config.h /home/ubuntu/lino/config/custom/fpms_config.h || exit 1
echo "=== inversions compiled in ==="
grep -E '^#define MOTOR[0-9]_(INV|ENCODER_INV)' /home/ubuntu/lino/config/custom/fpms_config.h

cd /home/ubuntu/lino/firmware || exit 1
echo "=== BUILD MISSION FIRMWARE ==="
"$PENV/bin/pio" run -e fpms 2>&1 | tail -4
echo "=== FLASH (replaces the open-loop test sketch) ==="
"$PENV/bin/pio" run -e fpms -t upload 2>&1 | tail -4

echo "=== ENABLE AT BOOT ==="
S systemctl enable micro-ros-agent fpms-teleop fpms-rover-agent fpms-lidar-ros \
    fpms-odom-tf fpms-tf fpms-map-odom mosquitto 2>&1 | tail -2
# fpms-missions stays DISABLED at boot on purpose: it is dashboard-triggerable
# and must not be able to launch a mission before the drivetrain is trusted.
S systemctl disable fpms-missions 2>&1 | tail -1

S systemctl start micro-ros-agent
echo "=== 75s for the board to establish its micro-ROS session ==="
sleep 75

export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "=== ROS SIDE: is the board publishing? ==="
cd /home/ubuntu
timeout 40 python3 - <<'EOF'
import time, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
rclpy.init(); n = Node("linkcheck")
st = {"odom": 0, "imu": 0, "batt": 0, "v": None}
n.create_subscription(Odometry, "/odom_raw", lambda m: st.__setitem__("odom", st["odom"]+1), 20)
n.create_subscription(Imu, "/imu", lambda m: st.__setitem__("imu", st["imu"]+1), 30)
def onb(m):
    st["batt"] += 1; st["v"] = m.voltage
n.create_subscription(BatteryState, "/battery", onb, 10)
t0 = time.time()
while time.time()-t0 < 15.0: rclpy.spin_once(n, timeout_sec=0.05)
print(f"  /odom_raw {st['odom']} msgs   /imu {st['imu']} msgs   /battery {st['batt']} msgs")
print(f"  BATTERY = {st['v']} V")
n.destroy_node(); rclpy.shutdown()
EOF

echo "=== START TELEOP (bridges ROS telemetry -> MQTT -> dashboard) ==="
S systemctl start fpms-teleop
sleep 12

echo "=== MQTT SIDE: what the dashboard will receive ==="
set -a; . /etc/fpms/config.env; set +a
T="${FPMS_THING_NAME:-rover2}"
timeout 25 mosquitto_sub -h 127.0.0.1 -u "$FPMS_MQTT_USER" -P "$FPMS_MQTT_PASS" \
    -t "fpms/$T/telemetry/drive" -C 1 \
  | python3 -c "
import sys, json
d = json.load(sys.stdin)
for k in ('battery_v','batt_v','voltage','link_ok','micro_ros'):
    if k in d: print(f'  {k} = {d[k]}')
print('  keys:', ','.join(sorted(d)[:14]))
" || echo "  (no drive telemetry within 25s)"

echo "=== SERVICE STATE ==="
systemctl is-active micro-ros-agent fpms-teleop fpms-rover-agent mosquitto | tr '\n' ' '; echo
echo "=== boot-enabled ==="
systemctl is-enabled micro-ros-agent fpms-teleop fpms-rover-agent fpms-lidar-ros fpms-odom-tf fpms-tf fpms-map-odom mosquitto 2>&1 | tr '\n' ' '; echo
echo "=== UROS DONE ==="
