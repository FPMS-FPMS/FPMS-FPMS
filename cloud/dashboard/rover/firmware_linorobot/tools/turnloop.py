"""Repeating in-place turn so the operator can watch which wheels spin which way.

Encoders report a perfect differential while the chassis does not rotate, which
means the left/right grouping in the config does not match the physical wiring.
No instrument on the robot can resolve that -- only looking at it can.

In-place turns only, alternating, always followed by a zero Twist.
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

rclpy.init()
node = Node("fpms_turnloop")
pub = node.create_publisher(Twist, "/cmd_vel", 10)


def hold(lin, ang, secs):
    t = Twist()
    t.linear.x = float(lin)
    t.angular.z = float(ang)
    t0 = time.time()
    while time.time() - t0 < secs:
        pub.publish(t)
        time.sleep(0.05)


time.sleep(1.0)
for i in range(6):
    print(f"cycle {i + 1}/6: LEFT turn (+1.2 rad/s) 3s", flush=True)
    hold(0.0, 1.2, 3.0)
    hold(0.0, 0.0, 1.0)
    print(f"cycle {i + 1}/6: RIGHT turn (-1.2 rad/s) 3s", flush=True)
    hold(0.0, -1.2, 3.0)
    hold(0.0, 0.0, 1.0)

for _ in range(5):
    pub.publish(Twist())
    time.sleep(0.05)
print("stopped", flush=True)
node.destroy_node()
rclpy.shutdown()
