"""Measure TURN_WIRE_SIGN on the current firmware, then undo the turn.

fpms_missions.py carries TURN_WIRE_SIGN = -1 with an explicit note that it was
DERIVED FROM THE REWIRING, NEVER OBSERVED. On the new firmware an inverted turn
sign makes a closed-loop turn chase a target it is moving away from, so it
rotates forever -- which is what happened when the mission service was triggered.

ROS REP-103: positive angular.z is counter-clockwise, and an IMU whose Z axis
points up reports a positive rate for that. So:
    raw +angular.z -> positive gyro  =>  TURN_WIRE_SIGN = +1 (no inversion)
    raw +angular.z -> negative gyro  =>  TURN_WIRE_SIGN = -1

Deliberately small, and it turns back afterwards: heading has no absolute
reference on this rover and the static map->odom transform is pinned to the
start pose.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu

RATE = 0.9          # above the stiction threshold for turning
SECS = 1.2          # ~40 deg, small enough to undo cleanly


class TurnSign(Node):
    def __init__(self):
        super().__init__("fpms_turn_sign")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Imu, "/imu", self._imu, 30)
        self.gz = 0.0
        self.n = 0

    def _imu(self, m):
        self.n += 1
        self.gz = m.angular_velocity.z

    def spin(self, s):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())
            self.spin(0.04)

    def burst(self, rate, secs):
        """Return gyro-integrated degrees for a commanded angular rate."""
        acc = 0.0
        t = Twist()
        t.angular.z = float(rate)
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
        # keep integrating through the coast-down
        t0 = last = time.time()
        while time.time() - t0 < 0.6:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - last
            last = now
            if 0 < dt < 0.5:
                acc += self.gz * dt
        return math.degrees(acc)


rclpy.init()
node = TurnSign()
node.spin(3.0)
if node.n == 0:
    print("ABORT: no /imu data")
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(1)

print(f"commanding RAW angular.z = +{RATE} rad/s for {SECS}s ...")
got = node.burst(+RATE, SECS)
print(f"  measured gyro = {got:+.1f} deg")

sign = 1 if got > 0 else -1
print()
print(f"RESULT: raw +angular.z produced a {'POSITIVE' if got > 0 else 'NEGATIVE'} gyro rate")
print(f"  => firmware {'follows' if got > 0 else 'INVERTS'} the ROS convention")
print(f"  => TURN_WIRE_SIGN should be {sign:+d}")
print(f"  (fpms_missions.py currently ships -1, derived not measured)")
print(f"  set with: FPMS_MISSION_TURN_WIRE_SIGN={sign}")

print()
print("undoing the turn to restore the start heading ...")
back = node.burst(-RATE, SECS)
print(f"  measured gyro = {back:+.1f} deg")
print(f"  net rotation over both bursts = {got + back:+.1f} deg (want ~0)")

node.stop()
node.destroy_node()
rclpy.shutdown()
