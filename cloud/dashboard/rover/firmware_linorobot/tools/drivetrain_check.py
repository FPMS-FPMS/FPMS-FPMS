"""Determine what the drivetrain actually does with linear vs angular commands.

Operator reports: a FORWARD command spins the rover, a TURN command drives it
straight. That is a linear/angular swap.

A left/right GROUPING error cannot cause it: a forward command sends both groups
the same sign, so the rover goes straight however the wheels are grouped. The
swap requires ONE SIDE to be electrically reversed -- then:
    forward  -> left fwd, right rev  -> rotation
    turn     -> left rev, right fwd, one side flipped -> both same -> translation

This measures both axes and prints which case holds. Every burst is small and
immediately reversed, so the rover ends roughly where it started.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Imu, LaserScan

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                        durability=QoSDurabilityPolicy.VOLATILE)


class Check(Node):
    def __init__(self):
        super().__init__("fpms_drivetrain_check")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom_raw", self._odom, 20)
        self.create_subscription(Imu, "/imu", self._imu, 30)
        self.create_subscription(LaserScan, "/scan_lidar", self._scan, SENSOR_QOS)
        self.pose = None
        self.gz = 0.0
        self.front = None
        self.n_odom = self.n_imu = 0

    def _odom(self, m):
        self.n_odom += 1
        p = m.pose.pose.position
        self.pose = (p.x * 1000.0, p.y * 1000.0)

    def _imu(self, m):
        self.n_imu += 1
        self.gz = m.angular_velocity.z

    def _scan(self, m):
        best = None
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            a = math.degrees(m.angle_min + i * m.angle_increment)
            a = (a + 180.0) % 360.0 - 180.0
            if abs(a) <= 25.0:
                best = r if best is None else min(best, r)
        self.front = None if best is None else best * 1000.0

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())
            self.spin(0.04)

    def burst(self, lin, ang, secs):
        """Return (degrees rotated, mm translated) for one commanded burst."""
        self.spin(0.3)
        x0, y0 = self.pose
        acc = 0.0
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        t0 = last = time.time()
        while time.time() - t0 < secs:
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - last
            last = now
            if 0 < dt < 0.5:
                acc += self.gz * dt
        self.stop()
        t0 = last = time.time()
        while time.time() - t0 < 0.6:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - last
            last = now
            if 0 < dt < 0.5:
                acc += self.gz * dt
        x1, y1 = self.pose
        return math.degrees(acc), math.hypot(x1 - x0, y1 - y0)


rclpy.init()
node = Check()
node.spin(4.0)
print(f"feeds: odom={node.n_odom} imu={node.n_imu} front={node.front}")
if node.n_odom == 0 or node.n_imu == 0:
    print("ABORT: board silent")
    raise SystemExit(1)
if node.front is not None and node.front < 500:
    print(f"ABORT: only {node.front:.0f}mm ahead - move the rover to open floor")
    raise SystemExit(1)

print("\n--- TEST A: pure LINEAR, +0.08 m/s for 1.5s (expect translation) ---")
rot_a, mm_a = node.burst(0.08, 0.0, 1.5)
print(f"  rotated {rot_a:+.1f} deg   translated {mm_a:.1f} mm")
node.spin(1.0)
print("  returning ...")
node.burst(-0.08, 0.0, 1.5)
node.spin(1.0)

print("\n--- TEST B: pure ANGULAR, +0.9 rad/s for 1.2s (expect rotation) ---")
rot_b, mm_b = node.burst(0.0, 0.9, 1.2)
print(f"  rotated {rot_b:+.1f} deg   translated {mm_b:.1f} mm")
node.spin(1.0)
print("  returning ...")
node.burst(0.0, -0.9, 1.2)
node.stop()

print("\n" + "=" * 58)
lin_ok = mm_a > 40 and abs(rot_a) < 15
ang_ok = abs(rot_b) > 20 and mm_b < 60
if lin_ok and ang_ok:
    print("DRIVETRAIN CORRECT: linear translates, angular rotates.")
elif (not lin_ok) and (not ang_ok) and abs(rot_a) > 20 and mm_b > 40:
    print("SWAPPED: linear command ROTATES and angular command TRANSLATES.")
    print("  -> one side is electrically reversed.")
    print("  -> fix by inverting that side in fpms_config.h:")
    print("     MOTOR2_INV/MOTOR4_INV (right) or MOTOR1_INV/MOTOR3_INV (left),")
    print("     then rebuild and reflash. Do NOT patch around it in mission code.")
else:
    print("MIXED/UNCLEAR result - report these four numbers:")
    print(f"  linear:  rot={rot_a:+.1f}deg  mm={mm_a:.1f}")
    print(f"  angular: rot={rot_b:+.1f}deg  mm={mm_b:.1f}")
print("=" * 58)

node.destroy_node()
rclpy.shutdown()
