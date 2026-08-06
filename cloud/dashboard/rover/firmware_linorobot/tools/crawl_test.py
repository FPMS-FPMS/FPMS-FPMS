"""Measure the real minimum commandable move on the new firmware.

The vendor firmware could not move less than ~230 mm: PWM_MOTOR_DEAD_ZONE (200
of 400) was added as feed-forward, so every non-zero setpoint slammed ~50% duty.
This sweeps small linear.x setpoints and reports the distance actually travelled
for each, which is the direct test of whether that floor is gone.

Distance comes from /odom_raw POSE, never twist: the vendor board reported
twist.linear.x sign-inverted relative to its own pose, and that artefact cost
this project two multi-hour debugging sessions. Sign is taken by projecting the
pose delta onto the board's reported yaw from the same message, which
degenerates to dx when the orientation is identity -- the measured case here.

Motion is refused unless --authorize-motion is passed. Every burst is bounded
and followed by an explicit zero Twist.
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class Crawl(Node):
    def __init__(self):
        super().__init__("fpms_crawl_test")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom_raw", self.on_odom, 20)
        self.create_subscription(Imu, "/imu", self.on_imu, 20)
        self.pose = None
        self.yaw = 0.0
        self.n_odom = 0
        self.gyro_z = 0.0
        self.n_imu = 0
        self.twist_lin = 0.0
        self.twist_ang = 0.0

    def on_odom(self, m):
        self.n_odom += 1
        p, o = m.pose.pose.position, m.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                              1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self.pose = (p.x, p.y)
        # Firmware derives these from measured wheel RPM. For an in-place turn
        # the pose cannot show anything (identity quaternion), so wheel-derived
        # angular velocity is the only evidence the motors are counter-rotating.
        self.twist_lin = m.twist.twist.linear.x
        self.twist_ang = m.twist.twist.angular.z

    def on_imu(self, m):
        self.n_imu += 1
        self.gyro_z = m.angular_velocity.z

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        for _ in range(3):
            self.pub.publish(Twist())
            self.spin(0.05)

    def burst(self, lin, ang, secs):
        """Command a bounded burst, return signed mm travelled and deg turned."""
        self.spin(0.3)
        if self.pose is None:
            return None
        x0, y0, yaw0 = self.pose[0], self.pose[1], self.yaw
        gyro_int = 0.0
        wheel_ang_int = 0.0
        peak_ang = 0.0
        peak_lin = 0.0
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        t0 = time.time()
        last = t0
        while time.time() - t0 < secs:
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - last
            last = now
            if 0 < dt < 0.5:
                gyro_int += self.gyro_z * dt
                wheel_ang_int += self.twist_ang * dt
            peak_ang = max(peak_ang, abs(self.twist_ang))
            peak_lin = max(peak_lin, abs(self.twist_lin))
        self.last_wheel_deg = math.degrees(wheel_ang_int)
        self.last_peak_ang = peak_ang
        self.last_peak_lin = peak_lin
        self.stop()
        self.spin(0.7)          # let it settle before measuring
        x1, y1 = self.pose
        dx, dy = x1 - x0, y1 - y0
        # Signed travel along the heading the board itself reported.
        along = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        return along * 1000.0, math.degrees(gyro_int), math.hypot(dx, dy) * 1000.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--authorize-motion", action="store_true")
    ap.add_argument("--mode", default="observe",
                    choices=["observe", "direction", "sweep", "turn"])
    args = ap.parse_args()

    rclpy.init()
    node = Crawl()
    node.spin(3.0)

    print(f"odom msgs={node.n_odom}  imu msgs={node.n_imu}  pose={node.pose}")
    if node.pose is None:
        print("NO /odom_raw -- board not publishing. Aborting.")
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    if args.mode == "observe" or not args.authorize_motion:
        if args.mode != "observe":
            print("REFUSED: motion requires --authorize-motion")
        node.destroy_node(); rclpy.shutdown(); return

    if args.mode == "direction":
        # Smallest useful probe: is +linear.x actually forward, and is the
        # magnitude sane? Deliberately tiny in case a side is mirrored.
        r = node.burst(0.05, 0.0, 1.0)
        print(f"DIRECTION +0.05 m/s x 1.0s -> along={r[0]:+.1f} mm  "
              f"gyro={r[1]:+.1f} deg  |disp|={r[2]:.1f} mm")

    elif args.mode == "sweep":
        print(f"{'cmd m/s':>8} {'secs':>5} {'along mm':>10} {'|disp| mm':>10} "
              f"{'implied m/s':>12}")
        for lin in (0.01, 0.02, 0.03, 0.05, 0.08, 0.12):
            r = node.burst(lin, 0.0, 2.0)
            if r is None:
                print("lost odom"); break
            print(f"{lin:8.3f} {2.0:5.1f} {r[0]:10.1f} {r[2]:10.1f} "
                  f"{r[2] / 1000.0 / 2.0:12.4f}")
            node.spin(1.0)

    elif args.mode == "turn":
        # 0.3 rad/s puts each side at only omega*track/2 = 0.0255 m/s, under the
        # measured 0.03 m/s stiction floor, so nothing moves. Skid-steer also
        # scrubs all four tyres sideways, so its threshold is higher than
        # straight-line. Sweep upward to find where rotation actually starts.
        for ang in (0.6, 0.9, 1.2, -1.2):
            r = node.burst(0.0, ang, 2.0)
            print(f"TURN {ang:+.2f} -> gyro={r[1]:+7.1f} deg | "
                  f"wheel_derived={node.last_wheel_deg:+7.1f} deg | "
                  f"peak_ang={node.last_peak_ang:.3f} rad/s | "
                  f"peak_lin={node.last_peak_lin:.3f} m/s | disp={r[2]:.1f} mm")
            node.spin(1.2)

    node.stop()
    node.destroy_node()
    rclpy.shutdown()


main()
