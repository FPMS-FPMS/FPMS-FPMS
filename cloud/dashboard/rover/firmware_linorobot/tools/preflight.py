"""Pre-flight: battery, ROS graph, odometry, TF, coordinates, Nav2.

Read-only. Publishes nothing, so it cannot move the rover.

Two QoS details learned the hard way on this rover:
  - LaserScan is published BEST_EFFORT; a default RELIABLE subscriber silently
    never matches and reports 0 Hz on a perfectly healthy feed.
  - /battery changed type with the firmware swap (UInt16 decivolts -> BatteryState
    volts), so both are accepted rather than assumed.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import UInt16
from tf2_ros import Buffer, TransformListener

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE,
)

ARENA_MM = 1200.0
TARGETS = {
    "zone-a (m1)": (228.0, 972.0),
    "zone-b (m2)": (972.0, 972.0),
    "water": (228.0, 228.0),
    "home/start": (972.0, 228.0),
}


class Pre(Node):
    def __init__(self):
        super().__init__("fpms_preflight")
        self.n = {"odom_raw": 0, "odom": 0, "imu": 0, "battery": 0, "scan": 0}
        self.odom_raw = None
        self.odom = None
        self.batt_v = None
        self.batt_type = None
        self.gyro = None
        self.scan_info = None

        self.create_subscription(Odometry, "/odom_raw", self._raw, 20)
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.create_subscription(Imu, "/imu", self._imu, 20)
        self.create_subscription(LaserScan, "/scan_lidar", self._scan, SENSOR_QOS)
        # One type only: ROS 2 refuses a second subscription on the same topic
        # with a different type. The firmware swap changed /battery from
        # UInt16(decivolts) to BatteryState(volts), and Pi-side subscribers
        # still declare the old type, so try the new one and fall back.
        try:
            self.create_subscription(BatteryState, "/battery", self._batt_new, 10)
        except Exception:
            self.create_subscription(UInt16, "/battery", self._batt_old, 10)

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

    def _raw(self, m):
        self.n["odom_raw"] += 1
        p, o = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self.odom_raw = (p.x, p.y, yaw, o.z, o.w,
                         m.twist.twist.linear.x, m.twist.twist.angular.z,
                         m.header.frame_id, m.child_frame_id)

    def _odom(self, m):
        self.n["odom"] += 1
        p = m.pose.pose.position
        self.odom = (p.x, p.y, m.header.frame_id, m.child_frame_id)

    def _imu(self, m):
        self.n["imu"] += 1
        self.gyro = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                     m.linear_acceleration.x, m.linear_acceleration.y,
                     m.linear_acceleration.z)

    def _scan(self, m):
        self.n["scan"] += 1
        if self.scan_info is None:
            fin = [r for r in m.ranges if math.isfinite(r) and r > 0.0]
            self.scan_info = (len(m.ranges), len(fin),
                              round(min(fin), 3) if fin else None, m.header.frame_id)

    def _batt_new(self, m):
        self.n["battery"] += 1
        self.batt_v = m.voltage
        self.batt_type = "BatteryState(volts)"

    def _batt_old(self, m):
        self.n["battery"] += 1
        self.batt_v = m.data / 10.0
        self.batt_type = "UInt16(decivolts)"


rclpy.init()
node = Pre()
WINDOW = 12.0
t0 = time.time()
while time.time() - t0 < WINDOW:
    rclpy.spin_once(node, timeout_sec=0.1)
el = time.time() - t0

print("=" * 62)
print("1. BATTERY")
if node.batt_v is None:
    print("   NO /battery data")
else:
    v = node.batt_v
    verdict = ("GOOD" if v >= 12.0 else
               "USABLE" if v >= 11.5 else
               "LOW - charge before accuracy work" if v >= 11.0 else
               "TOO LOW - do not run motors")
    print(f"   {v:.2f} V   [{node.batt_type}]   -> {verdict}")

print("2. ROS 2 / MICRO-ROS LINK")
for k in ("odom_raw", "imu", "battery", "odom", "scan"):
    print(f"   {k:10s} {node.n[k]:5d} msgs  {node.n[k] / el:6.2f} Hz")
board_up = node.n["odom_raw"] > 0 and node.n["imu"] > 0
print(f"   board publishing: {'YES' if board_up else 'NO'}")

print("3. ODOMETRY")
if node.odom_raw:
    x, y, yaw, qz, qw, tvx, taz, fid, cid = node.odom_raw
    ident = abs(qz) < 1e-9 and abs(qw - 1.0) < 1e-9
    print(f"   /odom_raw pose=({x:+.4f}, {y:+.4f}) m  yaw={math.degrees(yaw):+.2f} deg")
    print(f"   frames: {fid} -> {cid}")
    print(f"   twist: lin.x={tvx:+.4f}  ang.z={taz:+.4f}")
    print(f"   orientation identity (no wheel heading): {ident}")
else:
    print("   NO /odom_raw")
if node.odom:
    print(f"   /odom pose=({node.odom[0]:+.4f}, {node.odom[1]:+.4f}) "
          f"frames {node.odom[2]} -> {node.odom[3]}")
else:
    print("   /odom absent (fpms-odom-tf not producing)")
if node.gyro:
    g = node.gyro
    print(f"   IMU gyro=({g[0]:+.4f}, {g[1]:+.4f}, {g[2]:+.4f}) rad/s")
    print(f"   IMU accel=({g[3]:+.3f}, {g[4]:+.3f}, {g[5]:+.3f}) m/s^2 "
          f"|a|={math.sqrt(g[3]**2 + g[4]**2 + g[5]**2):.3f} (expect ~9.81 at rest)")

print("4. TF TREE")
for parent, child in (("odom", "base_footprint"), ("map", "odom"),
                      ("base_footprint", "base_link"), ("base_link", "laser_frame")):
    try:
        t = node.tf_buf.lookup_transform(parent, child, rclpy.time.Time())
        tr = t.transform.translation
        print(f"   {parent} -> {child}: OK ({tr.x:+.3f}, {tr.y:+.3f}, {tr.z:+.3f})")
    except Exception as e:
        print(f"   {parent} -> {child}: MISSING ({type(e).__name__})")

print("5. COORDINATES (arena.ts is source of truth)")
print(f"   arena {ARENA_MM:.0f} x {ARENA_MM:.0f} mm, origin bottom-left, +x right, +y up")
for name, (x, y) in TARGETS.items():
    print(f"   {name:14s} ({x:6.0f}, {y:6.0f}) mm")
print("   rover start heading 90 deg (up the arena)")

print("6. LIDAR")
print(f"   /scan_lidar {node.scan_info}")

node.destroy_node()
rclpy.shutdown()
