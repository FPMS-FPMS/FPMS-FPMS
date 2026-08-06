"""Read the IMU diagnostic smuggled through imu_msg.orientation.

  orientation.x = raw signed gyro Z register (pre-scale, pre-bias, pre-deadband)
  orientation.y = raw signed accel Z register  (control: proves reads work)
  orientation.z = pwr | gyro_cfg<<8 | accel_cfg<<16
  orientation.w = WHO_AM_I | i2c_addr<<8 | read_ok<<16

The control channel matters: the first attempt at this diagnostic reported all
zeros, which looked like "gyro is off" but was actually IMU_TWEAK never being
compiled in. If accel Z moves and WHO_AM_I reads 0x67, the transport is proven
and any zero on the gyro channel is real.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu

MODES = {0: "OFF", 1: "STANDBY", 2: "reserved", 3: "LOW-NOISE"}


class GyroDiag(Node):
    def __init__(self):
        super().__init__("fpms_gyro_diag")
        self.create_subscription(Imu, "/imu", self.on_imu, 30)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.gz = self.az = self.cfg = self.who = None
        self.wz = 0.0
        self.n = 0
        self.reset_ranges()

    def reset_ranges(self):
        self.gz_min = self.gz_max = None
        self.az_min = self.az_max = None
        self.wz_peak = 0.0

    def on_imu(self, m):
        self.n += 1
        o = m.orientation
        self.gz, self.az = int(o.x), int(o.y)
        self.cfg, self.who = int(o.z), int(o.w)
        self.wz = m.angular_velocity.z
        self.wz_peak = max(self.wz_peak, abs(self.wz))
        self.gz_min = self.gz if self.gz_min is None else min(self.gz_min, self.gz)
        self.gz_max = self.gz if self.gz_max is None else max(self.gz_max, self.gz)
        self.az_min = self.az if self.az_min is None else min(self.az_min, self.az)
        self.az_max = self.az if self.az_max is None else max(self.az_max, self.az)

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)


ap = argparse.ArgumentParser()
ap.add_argument("--turn", action="store_true")
args = ap.parse_args()

rclpy.init()
node = GyroDiag()
node.spin(4.0)

if node.who is None:
    print("NO /imu DATA")
    raise SystemExit(1)

who = node.who & 0xFF
addr = (node.who >> 8) & 0xFF
okf = (node.who >> 16) & 0xFF
pwr = node.cfg & 0xFF
gcfg = (node.cfg >> 8) & 0xFF
acfg = (node.cfg >> 16) & 0xFF

print(f"TRANSPORT: WHO_AM_I=0x{who:02X} (expect 0x67)  addr=0x{addr:02X}  read_ok={okf}")
print(f"CONFIG   : PWR_MGMT0=0x{pwr:02X} (expect 0x0F)  "
      f"GYRO_CONFIG0=0x{gcfg:02X} (expect 0x06)  ACCEL_CONFIG0=0x{acfg:02X} (expect 0x06)")
print(f"           gyro mode bits = {(pwr >> 2) & 3} ({MODES.get((pwr >> 2) & 3)})   "
      f"accel mode bits = {pwr & 3} ({MODES.get(pwr & 3)})")
print(f"AT REST  : raw gyroZ={node.gz}  raw accelZ={node.az}  "
      f"angular.z={node.wz:+.5f} rad/s   ({node.n} msgs)")

if args.turn:
    print("\n--- commanding +1.2 rad/s for 3s ---")
    node.reset_ranges()
    gyro_int = 0.0
    t = Twist()
    t.angular.z = 1.2
    t0 = time.time()
    last = t0
    while time.time() - t0 < 3.0:
        node.pub.publish(t)
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.time()
        dt = now - last
        last = now
        if 0 < dt < 0.5:
            gyro_int += node.wz * dt
    for _ in range(5):
        node.pub.publish(Twist())
        node.spin(0.05)
    node.spin(1.0)

    gz_span = (node.gz_max or 0) - (node.gz_min or 0)
    az_span = (node.az_max or 0) - (node.az_min or 0)
    print(f"raw gyroZ  range [{node.gz_min}..{node.gz_max}]  span={gz_span}")
    print(f"raw accelZ range [{node.az_min}..{node.az_max}]  span={az_span} (control)")
    print(f"peak |angular.z| = {node.wz_peak:.4f} rad/s")
    print(f"integrated gyro  = {math.degrees(gyro_int):+.1f} deg "
          f"(expect ~+180 to +200 if tracking)")

    print()
    if who != 0x67:
        print("VERDICT: I2C/transport problem - WHO_AM_I wrong")
    elif gz_span > 200:
        print("VERDICT: gyro registers ARE moving -> fault is in the ROS-layer "
              "scaling/bias/deadband, not the sensor")
    elif pwr != 0x0F:
        print(f"VERDICT: config did not stick (PWR_MGMT0=0x{pwr:02X}) -> gyro not "
              "enabled; wrong register address or a required init step is missing")
    else:
        print("VERDICT: config correct but gyro registers static -> sensor enabled "
              "and silent; suspect register map or a missing start-up step")

node.destroy_node()
rclpy.shutdown()
