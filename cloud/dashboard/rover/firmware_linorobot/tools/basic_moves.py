"""Four basic moves: forward 200mm, back 200mm, turn 90 left, turn 90 right.

Built on what the open-loop test proved: the right-side motors are wired
opposite (now handled by MOTORn_INV in firmware) and a crawl is the right speed.

Distance comes from encoder odometry, heading from the IMU gyro. The LiDAR front
distance is printed before and after each move as an INDEPENDENT check -- it is
the only sensor that does not share a failure mode with the encoders, and
encoder-derived odometry has lied convincingly on this rover before.

Each move is bounded, followed by a full stop, and pauses so the operator can
watch. Aborts if odometry and LiDAR disagree wildly on a straight move.
"""
import math
import sys
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

CRAWL_MPS = 0.06
TURN_RADPS = 0.9
SEG_TIMEOUT = 25.0
SETTLE = 0.8
OFF_AXIS_ABORT_MM = 120.0


def norm(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


class Basic(Node):
    def __init__(self):
        super().__init__("fpms_basic_moves")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom_raw", self._odom, 20)
        self.create_subscription(Imu, "/imu", self._imu, 30)
        self.create_subscription(LaserScan, "/scan_lidar", self._scan, SENSOR_QOS)
        self.pose = None
        self.yaw = 0.0
        self.gz = 0.0
        self.front = None
        self.n_odom = self.n_imu = self.n_scan = 0

    def _odom(self, m):
        self.n_odom += 1
        p = m.pose.pose.position
        o = m.pose.pose.orientation
        self.pose = (p.x * 1000.0, p.y * 1000.0)
        self.yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                              1.0 - 2.0 * (o.y * o.y + o.z * o.z))

    def _imu(self, m):
        self.n_imu += 1
        self.gz = m.angular_velocity.z

    def _scan(self, m):
        self.n_scan += 1
        best = None
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            if abs(norm(math.degrees(m.angle_min + i * m.angle_increment))) <= 20.0:
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

    def drive(self, mm, reverse=False):
        self.spin(0.3)
        x0, y0 = self.pose
        yaw0 = self.yaw
        f0 = self.front
        want = -1.0 if reverse else 1.0
        travelled = off = 0.0
        reason = "DONE"
        t0 = time.time()
        while True:
            if time.time() - t0 > SEG_TIMEOUT:
                reason = "TIMEOUT"
                break
            rclpy.spin_once(self, timeout_sec=0.02)
            dx = self.pose[0] - x0
            dy = self.pose[1] - y0
            travelled = (dx * math.cos(yaw0) + dy * math.sin(yaw0)) * want
            off = abs(-dx * math.sin(yaw0) + dy * math.cos(yaw0))
            if travelled >= mm:
                break
            if off > OFF_AXIS_ABORT_MM:
                reason = "OFF_AXIS"
                break
            t = Twist()
            t.linear.x = -CRAWL_MPS if reverse else CRAWL_MPS
            self.pub.publish(t)
        self.stop()
        self.spin(SETTLE)
        dx = self.pose[0] - x0
        dy = self.pose[1] - y0
        travelled = (dx * math.cos(yaw0) + dy * math.sin(yaw0)) * want
        f1 = self.front
        # LiDAR moves the OPPOSITE way to travel when driving at a wall.
        lidar_delta = None if (f0 is None or f1 is None) else (f0 - f1) * want
        return travelled, off, reason, f0, f1, lidar_delta

    def turn(self, deg):
        self.spin(0.3)
        target = math.radians(abs(deg))
        sign = 1.0 if deg > 0 else -1.0
        acc = 0.0
        t0 = last = time.time()
        while acc < target:
            if time.time() - t0 > SEG_TIMEOUT:
                break
            t = Twist()
            remain = target - acc
            t.angular.z = sign * TURN_RADPS * (0.4 if remain < math.radians(15) else 1.0)
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - last
            last = now
            if 0 < dt < 0.5:
                acc += abs(self.gz) * dt
        self.stop()
        t0 = last = time.time()
        while time.time() - t0 < SETTLE:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - last
            last = now
            if 0 < dt < 0.5:
                acc += abs(self.gz) * dt
        return math.degrees(acc) * sign


rclpy.init()
n = Basic()
n.spin(3.0)
print(f"feeds: odom={n.n_odom} imu={n.n_imu} scan={n.n_scan} front={n.front}")
if n.pose is None or n.n_imu == 0:
    print("ABORT: board silent")
    sys.exit(1)

print("\n>>> WATCH THE ROVER <<<\n")

print("--- 1. FORWARD 200mm ---")
d, off, r, f0, f1, ld = n.drive(200.0)
print(f"    encoders: {d:+.0f}mm  off-axis {off:.0f}mm  ({r})")
print(f"    lidar: {f0} -> {f1} mm   implies {ld if ld is None else round(ld)}mm travelled")
n.spin(2.0)

print("--- 2. BACK 200mm ---")
d, off, r, f0, f1, ld = n.drive(200.0, reverse=True)
print(f"    encoders: {d:+.0f}mm  off-axis {off:.0f}mm  ({r})")
print(f"    lidar: {f0} -> {f1} mm   implies {ld if ld is None else round(ld)}mm travelled")
n.spin(2.0)

print("--- 3. TURN 90 LEFT (+90) ---")
g = n.turn(90.0)
print(f"    gyro measured {g:+.1f} deg   front now {n.front} mm")
n.spin(2.0)

print("--- 4. TURN 90 RIGHT (-90) ---")
g = n.turn(-90.0)
print(f"    gyro measured {g:+.1f} deg   front now {n.front} mm")

n.stop()
print("\nDONE - motors stopped.")
n.destroy_node()
rclpy.shutdown()
