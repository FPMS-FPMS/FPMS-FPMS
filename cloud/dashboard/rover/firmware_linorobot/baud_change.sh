#!/bin/bash
# Drop the Pi<->ESP32 serial link from 921600 to 230400, then prove it holds.
#
# Both ends must change together or the link simply will not come up:
#   firmware  : BAUDRATE in fpms_config.h  (already edited, copied below)
#   agent     : the -b flag on micro-ros-agent.service
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }
PENV="$HOME/.platformio/penv"
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble

cp /home/ubuntu/fpms_config.h /home/ubuntu/lino/config/custom/fpms_config.h || exit 1
grep -n 'define BAUDRATE' /home/ubuntu/lino/config/custom/fpms_config.h

S systemctl stop fpms-missions 2>/dev/null
S systemctl stop fpms-teleop 2>/dev/null
S systemctl stop micro-ros-agent

echo "=== BUILD ==="
cd /home/ubuntu/lino/firmware || exit 1
"$PENV/bin/pio" run -e fpms 2>&1 | tail -8
echo "=== FLASH ==="
"$PENV/bin/pio" run -e fpms -t upload 2>&1 | tail -6

echo "=== AGENT UNIT -> 230400 ==="
UNIT=$(systemctl show micro-ros-agent -p FragmentPath --value)
echo "unit: $UNIT"
S cp "$UNIT" "$UNIT.bak.$(date +%s)"
S sed -i 's/-b 921600/-b 230400/' "$UNIT"
grep -o '\-b [0-9]*' "$UNIT"
S systemctl daemon-reload
S systemctl start micro-ros-agent

echo "=== 70s reconnect - KEEP THE ROVER STILL ==="
sleep 70

export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /home/ubuntu
echo "=== SURVIVAL: 3 min at the new baud ==="
python3 -u - <<'EOF'
import time, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
rclpy.init(); n = Node("surv"); s = {"c": 0}
n.create_subscription(Odometry, "/odom_raw", lambda m: s.__setitem__("c", s["c"]+1), 20)
prev = 0; ok = True
for i in range(9):
    t0 = time.time()
    while time.time()-t0 < 20.0: rclpy.spin_once(n, timeout_sec=0.05)
    got = s["c"]-prev; prev = s["c"]
    alive = got > 100
    ok = ok and alive
    print(f"  t={20*(i+1):3d}s {got:5d} msgs  {'ALIVE' if alive else '*** SILENT ***'}")
    if not alive: break
n.destroy_node(); rclpy.shutdown()
raise SystemExit(0 if ok else 1)
EOF
echo "SURVIVAL_RC=$?"
echo "=== BAUD CHANGE DONE ==="
