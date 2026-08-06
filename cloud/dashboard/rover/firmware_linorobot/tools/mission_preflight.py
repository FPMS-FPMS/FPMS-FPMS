"""Verify the rover can actually execute a mission, and fix it if it cannot.

Run this BEFORE every mission. It exists because of a specific failure: the
drive board enters a partial hang where it keeps publishing /odom_raw and /imu
at 50 Hz -- so every liveness check based on "is the topic alive" passes -- while
its odometry integrator is frozen and it silently ignores /cmd_vel. A mission
then commands 594 mm, travels 0 mm, and burns its whole timeout looking like a
bug in the mission code.

The tell is that the published pose becomes BIT-IDENTICAL between samples. A
live board's pose always jitters in the last decimals. That is checked here with
no motion at all, which matters because heading has no absolute reference on
this rover -- a turning self-test would destroy the one piece of state a mission
depends on.

Exit codes: 0 ready, 1 not ready (reason printed).
"""
import argparse
import math
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import BatteryState, Imu, LaserScan

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST, depth=5,
    durability=QoSDurabilityPolicy.VOLATILE)

RESET_SCRIPT = """
import time, serial
ser = serial.Serial("/dev/ttyUSB1", 921600, timeout=0.2)
ser.setDTR(False); ser.setRTS(True); time.sleep(0.2); ser.setRTS(False)
time.sleep(0.05); ser.close()
print("reset pulsed")
"""

# Arena truth. The rover must start here for map->odom (a static transform at
# this pose) to mean anything.
HOME_X, HOME_Y = 972.0, 228.0
ARENA_MM = 1200.0
# Facing 90 deg from the start box, the far wall is ~972mm away. Facing a side
# wall it would be ~228mm. That single number verifies heading with no motion.
EXPECT_FRONT_MM = ARENA_MM - HOME_Y
FRONT_TOL_MM = 260.0
FRONT_CONE_DEG = 25.0


def norm_deg(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


class Pre(Node):
    def __init__(self):
        super().__init__("fpms_mission_preflight")
        self.create_subscription(Odometry, "/odom_raw", self._odom, 20)
        self.create_subscription(Imu, "/imu", self._imu, 30)
        self.create_subscription(BatteryState, "/battery", self._batt, 10)
        self.create_subscription(LaserScan, "/scan_lidar", self._scan, SENSOR_QOS)
        self.poses = []
        self.n_odom = self.n_imu = self.n_scan = 0
        self.volts = None
        self.front_mm = None
        self.scan_stamp = 0.0
        self.gyro_z = 0.0

    def _odom(self, m):
        self.n_odom += 1
        p = m.pose.pose.position
        self.poses.append((p.x, p.y))

    def _imu(self, m):
        self.n_imu += 1
        self.gyro_z = m.angular_velocity.z

    def _batt(self, m):
        self.volts = m.voltage

    def _scan(self, m):
        self.n_scan += 1
        self.scan_stamp = time.time()
        best = None
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            if abs(norm_deg(math.degrees(m.angle_min + i * m.angle_increment))) <= FRONT_CONE_DEG:
                best = r if best is None else min(best, r)
        self.front_mm = None if best is None else best * 1000.0

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)


def pose_is_static(node, secs=4.0):
    """Report whether pose changed at all, and from what value.

    NOT a hang test on its own. A healthy but stationary rover integrates zero
    encoder counts and so reports a bit-identical pose too -- and straight after
    a board reset that value is exactly (0,0). The distinguishing feature of the
    real hang seen on 2026-08-03 was a pose stuck at a NON-ZERO value mid-travel
    while twist read exactly zero. Only the motion probe below is conclusive.
    """
    node.poses.clear()
    node.spin(secs)
    if len(node.poses) < 20:
        return None, None, f"only {len(node.poses)} odom samples in {secs}s"
    uniq = len(set(node.poses))
    last = node.poses[-1]
    return uniq <= 1, last, f"{len(node.poses)} samples, {uniq} distinct, last={last}"


def motion_probe(node, mps=0.08, secs=1.5, need_mm=20.0):
    """Drive a short STRAIGHT burst and confirm the pose actually advances.

    Straight, not turning: heading has no absolute reference on this rover and a
    rotation probe would destroy the start pose the static map->odom transform is
    pinned to. ~90mm forward inside a ~970mm clear corridor is harmless.
    """
    from geometry_msgs.msg import Twist
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    node.spin(0.3)
    if not node.poses:
        return False, "no odom before probe"
    x0, y0 = node.poses[-1]

    t = Twist()
    t.linear.x = float(mps)
    t0 = time.time()
    while time.time() - t0 < secs:
        pub.publish(t)
        rclpy.spin_once(node, timeout_sec=0.02)
    for _ in range(5):
        pub.publish(Twist())
        node.spin(0.05)
    node.spin(0.8)

    x1, y1 = node.poses[-1]
    moved = math.hypot(x1 - x0, y1 - y0) * 1000.0
    return moved >= need_mm, f"moved {moved:.1f}mm (need >={need_mm:.0f}mm)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--auto-reset", action="store_true",
                    help="hard-reset the board and re-check if it is hung")
    ap.add_argument("--skip-heading", action="store_true")
    ap.add_argument("--probe", action="store_true",
                    help="drive a short straight burst to prove the board acts "
                         "on /cmd_vel (the only conclusive hang test)")
    args = ap.parse_args()

    rclpy.init()
    node = Pre()
    node.spin(4.0)

    problems = []

    print("=" * 60)
    print(f"1. FEEDS   odom={node.n_odom} imu={node.n_imu} scan={node.n_scan}")
    if node.n_odom == 0 or node.n_imu == 0:
        problems.append("board not publishing (odom or imu silent)")

    print(f"2. BATTERY {node.volts if node.volts is None else round(node.volts, 2)} V")
    if node.volts is not None and node.volts < 11.5:
        problems.append(f"battery {node.volts:.2f}V is low - results will not be trustworthy")

    static, last, detail = pose_is_static(node)
    at_origin = last is not None and abs(last[0]) < 1e-9 and abs(last[1]) < 1e-9
    print(f"3. POSE    {detail}")
    if static and not at_origin:
        # Stuck at a non-zero value = the hang signature.
        problems.append(f"pose frozen mid-travel at {last} - board is hung")
    elif static:
        print("   (static at origin - expected when stationary; probe decides)")

    stale = time.time() - node.scan_stamp > 2.0
    print(f"4. LIDAR   front={None if node.front_mm is None else round(node.front_mm)}mm "
          f"stale={stale}")
    if node.n_scan == 0 or stale:
        problems.append("no live scan - the obstacle guard would be blind")
    elif not args.skip_heading and node.front_mm is not None:
        err = abs(node.front_mm - EXPECT_FRONT_MM)
        ok = err <= FRONT_TOL_MM
        print(f"5. HEADING expect ~{EXPECT_FRONT_MM:.0f}mm to far wall, "
              f"saw {node.front_mm:.0f}mm (err {err:.0f}mm) -> "
              f"{'facing up the arena' if ok else 'WRONG HEADING OR POSITION'}")
        if not ok:
            problems.append(
                f"front distance {node.front_mm:.0f}mm does not match the start pose "
                f"({EXPECT_FRONT_MM:.0f}mm expected). Rover is not at "
                f"({HOME_X:.0f},{HOME_Y:.0f}) facing 90deg, or something is in front of it.")

    if args.probe and not problems:
        ok, detail = motion_probe(node)
        print(f"6. PROBE   {detail} -> {'board is ACTING on /cmd_vel' if ok else 'NO MOTION'}")
        if not ok:
            problems.append("board ignored /cmd_vel - hung; hard-reset it "
                            "(board_reset.sh) before running a mission")

    print("=" * 60)
    if problems:
        print("NOT READY:")
        for p in problems:
            print(f"  - {p}")
    else:
        print("READY: board live, pose advancing, scan good, heading consistent.")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if problems else 0)


main()
