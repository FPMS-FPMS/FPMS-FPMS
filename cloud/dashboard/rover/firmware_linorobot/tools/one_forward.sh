#!/bin/bash
# ONE pure-forward command. No mission, no retrace, no interpretation.
# The operator watches; their eyes are the ground truth here, because the
# on-board sensors may be reporting a consistent lie (inverted encoder signs on
# one side make a spin read as straight-line travel).
PW="$1"
S() { echo "$PW" | sudo -S "$@"; }

S systemctl stop micro-ros-agent
sleep 2
python3 - <<'EOF'
import time, serial
ser = serial.Serial("/dev/ttyUSB1", 230400, timeout=0.2)
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

echo ""
echo ">>> WATCH THE ROVER NOW <<<"
echo ""
python3 -u - <<'EOF'
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

rclpy.init()
n = Node("one_forward")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
st = {"p": None, "gz": 0.0, "acc": 0.0}
def on_o(m):
    p = m.pose.pose.position
    st["p"] = (p.x*1000.0, p.y*1000.0)
def on_i(m): st["gz"] = m.angular_velocity.z
n.create_subscription(Odometry, "/odom_raw", on_o, 20)
n.create_subscription(Imu, "/imu", on_i, 30)

t0 = time.time()
while time.time()-t0 < 3.0: rclpy.spin_once(n, timeout_sec=0.05)
if st["p"] is None:
    print("BOARD SILENT - nothing commanded"); raise SystemExit(1)
x0, y0 = st["p"]

print("COMMANDING: linear.x = +0.08 m/s, angular.z = 0.0, for 3.0 seconds")
t = Twist(); t.linear.x = 0.08
t0 = last = time.time()
while time.time()-t0 < 3.0:
    pub.publish(t)
    rclpy.spin_once(n, timeout_sec=0.02)
    now = time.time(); dt = now-last; last = now
    if 0 < dt < 0.5: st["acc"] += st["gz"]*dt
for _ in range(6):
    pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.03)
t0 = time.time()
while time.time()-t0 < 1.0: rclpy.spin_once(n, timeout_sec=0.05)

x1, y1 = st["p"]
print("")
print("WHAT THE SENSORS CLAIM:")
print(f"  odometry says it moved {math.hypot(x1-x0, y1-y0):.0f} mm")
print(f"  gyro says it rotated   {math.degrees(st['acc']):+.1f} deg")
print("")
print("If the rover physically SPUN, both of those numbers are lies and the")
print("encoder signs are inverted on one side.")
EOF
echo "=== ONE FORWARD DONE ==="
