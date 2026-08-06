"""FPMS mission executor: crawl to a zone, dwell, retrace home.

Built on what the golden phase6 driver proved, NOT on its constants -- those
were tuned for a firmware with no usable velocity loop, where the only lever was
pulse width. This firmware closes the loop on float RPM, so speed is commanded
directly and the structure carries over rather than the numbers:

  - segments are turn-then-drive, and each records what ACTUALLY happened
    (gyro-measured degrees, odometry-measured millimetres), never what was asked
  - heading is held by a P-controller whose error resets per segment, so error
    does not compound across a route
  - the return leg replays the recorded segments in reverse, driving BACKWARD
    first and then undoing each turn. Outbound error is then traversed in the
    opposite sense and cancels instead of accumulating. That is why the old
    rover came home to 0.6%.

Distance comes from /odom_raw POSE, never twist. Heading comes from the IMU gyro
(verified against wheel odometry to within 0.8 deg).
"""
import argparse
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

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST, depth=5,
    durability=QoSDurabilityPolicy.VOLATILE)

# Arena truth, from frontend/src/lib/arena.ts (single source of truth).
ARENA_MM = 1200.0
HOME = (972.0, 228.0)
HOME_HEADING_DEG = 90.0            # facing up the arena, +y
TARGETS = {
    "m1": (228.0, 972.0),          # zone-a, top-left
    "m2": (972.0, 972.0),          # zone-b, top-right
    "water": (228.0, 228.0),       # water station, bottom-left
}

# Measured on this rover 2026-08-03, wheels on hardwood, 13.4V pack:
#   0.010 / 0.020 m/s -> no motion (mechanical stiction)
#   0.030 m/s -> 9.3mm in 2s   0.050 -> 53.6mm   0.080 -> 118.8mm
# So usable crawl starts around 0.03; 0.06 gives margin over stiction while
# still being a genuine crawl.
CRAWL_MPS = 0.06
TURN_RADPS = 0.9                   # below ~0.6 the wheels do not break stiction

# The PID needs time to overcome stiction, so a burst delivers less than
# speed*time. Both loops therefore measure rather than integrate the command.
HEADING_KP = 1.8                   # rad/s of correction per rad of error
HEADING_CLAMP = 0.5                # rad/s
LOOP_HZ = 20.0

DIST_TOL_MM = 8.0
TURN_TOL_DEG = 2.0
SETTLE_S = 0.6                     # let coast finish before measuring

FRONT_CONE_DEG = 25.0
FRONT_STOP_MM = 400.0              # measured FROM THE LIDAR, which sits behind
                                   # the nose. 120 was a never-fires backstop.
SEG_TIMEOUT_S = 60.0
MAX_SEGMENT_MM = 1400.0            # nothing legal in a 1200mm arena exceeds this


def norm_deg(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


class Mission(Node):
    def __init__(self, use_guard=True, front_stop_mm=FRONT_STOP_MM):
        super().__init__("fpms_mission")
        self.front_stop_mm = front_stop_mm
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom_raw", self._odom, 20)
        self.create_subscription(Imu, "/imu", self._imu, 30)
        self.create_subscription(LaserScan, "/scan_lidar", self._scan, SENSOR_QOS)
        self.pose = None
        self.odom_yaw = 0.0
        self.gyro_z = 0.0
        self.front_mm = None
        self.scan_stamp = 0.0
        self.use_guard = use_guard
        self.n_odom = self.n_imu = self.n_scan = 0

    def _odom(self, m):
        self.n_odom += 1
        p = m.pose.pose.position
        self.pose = (p.x * 1000.0, p.y * 1000.0)
        # The replacement firmware publishes a REAL wheel-derived yaw here; the
        # vendor firmware published an identity quaternion, which is why older
        # code in this project treats odom orientation as useless.
        o = m.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                                   1.0 - 2.0 * (o.y * o.y + o.z * o.z))

    def _imu(self, m):
        self.n_imu += 1
        self.gyro_z = m.angular_velocity.z

    def _scan(self, m):
        self.n_scan += 1
        self.scan_stamp = time.time()
        best = None
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            ang = math.degrees(m.angle_min + i * m.angle_increment)
            if abs(norm_deg(ang)) <= FRONT_CONE_DEG:
                best = r if best is None else min(best, r)
        self.front_mm = None if best is None else best * 1000.0

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self):
        for _ in range(4):
            self.pub.publish(Twist())
            self.spin(0.03)

    def blocked(self):
        """Forward guard. A stale scan is treated as blind, not as clear."""
        if not self.use_guard:
            return False
        if time.time() - self.scan_stamp > 2.0:
            return True
        return self.front_mm is not None and self.front_mm < self.front_stop_mm

    # -- primitives ---------------------------------------------------------

    def turn(self, deg):
        """Turn by deg. Returns the gyro-measured degrees ACTUALLY rotated."""
        if abs(deg) < TURN_TOL_DEG:
            return 0.0
        target = math.radians(abs(deg))
        sign = 1.0 if deg > 0 else -1.0
        acc = 0.0
        t0 = last = time.time()
        while acc < target:
            if time.time() - t0 > SEG_TIMEOUT_S:
                break
            t = Twist()
            # Ease off near the goal so the coast overshoot stays small; the old
            # open-loop driver bought this with a fixed 0.93 coast factor, which
            # only worked because its duty was constant.
            remain = target - acc
            rate = TURN_RADPS * (0.35 if remain < math.radians(12.0) else 1.0)
            t.angular.z = sign * rate
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - last
            last = now
            if 0 < dt < 0.5:
                acc += abs(self.gyro_z) * dt
        self.stop()

        # Keep integrating through the coast-down, then report the truth.
        t0 = last = time.time()
        while time.time() - t0 < SETTLE_S:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - last
            last = now
            if 0 < dt < 0.5:
                acc += abs(self.gyro_z) * dt
        return math.degrees(acc) * sign

    def drive(self, dist_mm, reverse=False):
        """Drive dist_mm holding heading. Returns measured mm travelled."""
        if dist_mm <= DIST_TOL_MM:
            return 0.0, "SKIP"
        if dist_mm > MAX_SEGMENT_MM:
            raise ValueError(f"segment {dist_mm:.0f}mm exceeds arena bounds")
        self.spin(0.25)
        if self.pose is None:
            raise RuntimeError("no /odom_raw")
        x0, y0 = self.pose
        yaw0 = self.odom_yaw    # heading this segment is measured along
        head_err = 0.0          # gyro-integrated drift since this segment began
        travelled = 0.0
        want = -1.0 if reverse else 1.0
        off_axis = 0.0
        reason = "DONE"
        t0 = last = time.time()
        while True:
            if time.time() - t0 > SEG_TIMEOUT_S:
                reason = "TIMEOUT"
                break
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.time()
            dt = now - last
            last = now
            if 0 < dt < 0.5:
                head_err += self.gyro_z * dt

            if self.pose is not None:
                dx = self.pose[0] - x0
                dy = self.pose[1] - y0
                # Signed along the segment's own heading, NOT hypot(): an
                # unsigned magnitude counts a rover curving away as progress,
                # which is exactly how a bad reverse leg burned its full timeout
                # while driving ~200mm sideways.
                along = (dx * math.cos(yaw0) + dy * math.sin(yaw0)) * want
                off_axis = abs(-dx * math.sin(yaw0) + dy * math.cos(yaw0))
                travelled = along
            if travelled >= dist_mm:
                break
            if off_axis > 150.0:
                reason = "OFF_AXIS"
                break
            if not reverse and self.blocked():
                reason = "BLOCKED"
                break

            remain = dist_mm - travelled
            speed = CRAWL_MPS * (0.5 if remain < 60.0 else 1.0)
            t = Twist()
            t.linear.x = -speed if reverse else speed
            # Correction sign flips in reverse: "left wheel faster" means the
            # opposite thing to the chassis when travelling backwards. Omitting
            # this is what makes a retrace diverge instead of converge.
            # NO reverse sign flip. The golden driver flipped this because it
            # corrected WHEEL POWERS, where the geometric effect of "left wheel
            # faster" genuinely inverts when travelling backwards. Here the
            # correction is a direct angular.z against an absolute
            # gyro-integrated heading error, and heading is heading whichever
            # way the rover is moving. Flipping it made the loop positive
            # feedback in reverse: measured 2026-08-03, the return leg curved
            # ~200mm sideways instead of retracing, and timed out.
            corr = -HEADING_KP * head_err
            t.angular.z = max(-HEADING_CLAMP, min(HEADING_CLAMP, corr))
            self.pub.publish(t)
        self.stop()
        self.spin(SETTLE_S)
        if self.pose is not None:
            dx = self.pose[0] - x0
            dy = self.pose[1] - y0
            travelled = (dx * math.cos(yaw0) + dy * math.sin(yaw0)) * want
            off_axis = abs(-dx * math.sin(yaw0) + dy * math.cos(yaw0))
        if off_axis > 20.0:
            print(f"    (drifted {off_axis:.0f}mm off the segment axis)")
        return travelled, reason


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", default="m1", choices=sorted(TARGETS))
    ap.add_argument("--dwell", type=float, default=3.0)
    ap.add_argument("--standoff", type=float, default=0.0,
                    help="stop this many mm short of the zone centre")
    ap.add_argument("--no-guard", action="store_true")
    # The arena wall is a known static boundary, not an obstacle. Approaching a
    # zone whose centre sits 228mm from a wall trips a 400mm guard at the goal,
    # so it is loosened per-run rather than switched off -- a disabled guard is
    # how this rover previously drove into something with the guard "working".
    ap.add_argument("--front-stop", type=float, default=FRONT_STOP_MM)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--authorize-motion", action="store_true")
    # Square the rover back up after something rotated it, without asking the
    # operator to physically re-place it. Closed-loop on the gyro, same
    # primitive the mission uses.
    ap.add_argument("--turn-only", type=float, default=None,
                    help="just turn this many degrees and exit")
    args = ap.parse_args()

    if args.turn_only is not None:
        if not args.authorize_motion:
            print("REFUSED: motion requires --authorize-motion")
            return
        rclpy.init()
        node = Mission(use_guard=False)
        node.spin(3.0)
        if node.n_imu == 0:
            print("ABORT: no /imu")
        else:
            got = node.turn(args.turn_only)
            print(f"turn-only: asked {args.turn_only:+.1f}deg, "
                  f"measured {got:+.1f}deg")
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        return

    tx, ty = TARGETS[args.target]
    dx, dy = tx - HOME[0], ty - HOME[1]
    dist = math.hypot(dx, dy) - args.standoff
    bearing = math.degrees(math.atan2(dy, dx))
    turn_deg = norm_deg(bearing - HOME_HEADING_DEG)

    print(f"PLAN {args.target}: home{HOME} heading {HOME_HEADING_DEG:.0f}deg "
          f"-> ({tx:.0f}, {ty:.0f})")
    print(f"  bearing {bearing:+.1f}deg -> turn {turn_deg:+.1f}deg, "
          f"drive {dist:.0f}mm at {CRAWL_MPS} m/s "
          f"(~{dist / 1000.0 / CRAWL_MPS:.0f}s)")
    print(f"  then dwell {args.dwell:.0f}s, then retrace home")
    if args.dry_run or not args.authorize_motion:
        if not args.dry_run:
            print("REFUSED: motion requires --authorize-motion")
        return

    rclpy.init()
    node = Mission(use_guard=not args.no_guard, front_stop_mm=args.front_stop)
    node.spin(3.0)
    print(f"feeds: odom={node.n_odom} imu={node.n_imu} scan={node.n_scan} "
          f"front={node.front_mm}")
    if node.pose is None or node.n_imu == 0:
        print("ABORT: missing odom or imu")
        node.stop(); node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    if node.blocked():
        print(f"ABORT: front blocked at start ({node.front_mm}mm) or scan stale")
        node.stop(); node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    retrace = []
    try:
        print("\n--- OUTBOUND ---")
        actual_turn = node.turn(turn_deg)
        print(f"turn: asked {turn_deg:+.1f}deg, measured {actual_turn:+.1f}deg")
        node.spin(0.4)
        moved, reason = node.drive(dist)
        print(f"drive: asked {dist:.0f}mm, measured {moved:.0f}mm ({reason})")
        retrace.append((actual_turn, moved))

        print(f"\n--- ARRIVED, dwelling {args.dwell:.0f}s ---")
        node.stop()
        node.spin(args.dwell)

        print("\n--- RETURN (retrace) ---")
        for (td, dd) in reversed(retrace):
            back, r2 = node.drive(dd, reverse=True)
            print(f"reverse: asked {dd:.0f}mm, measured {back:.0f}mm ({r2})")
            node.spin(0.4)
            undo = node.turn(-td)
            print(f"undo turn: asked {-td:+.1f}deg, measured {undo:+.1f}deg")

        print("\n--- HOME ---")
        if node.pose is not None:
            print(f"odom residual from start: ({node.pose[0]:+.1f}, "
                  f"{node.pose[1]:+.1f}) mm  |err|="
                  f"{math.hypot(*node.pose):.1f} mm")
    except KeyboardInterrupt:
        print("INTERRUPTED")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


main()
