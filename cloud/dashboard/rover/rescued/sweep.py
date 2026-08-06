#!/usr/bin/env python3
"""Motor deadband sweep. Wheels MUST be off the ground.

Ramps a single axis in small steps, holding each step for 1.5s and sampling the
board's own reported wheel velocity over the last 0.5s, with a 1.0s zero-Twist
discharge between steps so integrator windup cannot accumulate across steps.

Hard caps: |linear.x| <= 0.10, |angular.z| <= 0.60, and a zero Twist is sent on
every exit path including exceptions.
"""
import math
import signal
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

LIN_CAP = 0.10
ANG_CAP = 0.60
HOLD_S = 1.5
SAMPLE_S = 0.5
DISCHARGE_S = 1.0

qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=10)

rclpy.init()
node = rclpy.create_node("deadband_sweep")
pub = node.create_publisher(Twist, "/cmd_vel", qos)

S = {"vx": 0.0, "wz_odom": 0.0, "gz": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0,
     "batt": None, "n": 0}


def on_odom(m):
    S["vx"] = m.twist.twist.linear.x
    S["wz_odom"] = m.twist.twist.angular.z
    S["x"] = m.pose.pose.position.x
    S["y"] = m.pose.pose.position.y
    o = m.pose.pose.orientation
    S["yaw"] = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
    S["n"] += 1


def on_imu(m):
    S["gz"] = m.angular_velocity.z


def on_batt(m):
    S["batt"] = m.data


node.create_subscription(Odometry, "/odom_raw", on_odom, qos)
node.create_subscription(Imu, "/imu", on_imu, qos)
node.create_subscription(UInt16, "/battery", on_batt, qos)


# The Wi-Fi link to this Pi is dropping every ~45s. If SSH dies mid-sweep the
# default SIGHUP would kill Python outright, skipping the `finally` that stops
# the motors and leaving the wheels spinning. Catch the signals and zero first.
def _panic(signum, _frame):
    try:
        for _ in range(15):
            t = Twist()
            pub.publish(t)
            time.sleep(0.01)
        print(f"\n!! signal {signum} — zero Twist sent x15, aborting sweep",
              flush=True)
    finally:
        import os
        os._exit(1)


for _s in (signal.SIGHUP, signal.SIGTERM, signal.SIGINT):
    signal.signal(_s, _panic)


def spin(dt):
    t = time.monotonic()
    while time.monotonic() - t < dt:
        rclpy.spin_once(node, timeout_sec=0.02)


def send(vx=0.0, wz=0.0):
    vx = max(-LIN_CAP, min(LIN_CAP, vx))
    wz = max(-ANG_CAP, min(ANG_CAP, wz))
    t = Twist()
    t.linear.x = float(vx)
    t.angular.z = float(wz)
    pub.publish(t)


def zero(dur):
    t = time.monotonic()
    while time.monotonic() - t < dur:
        send(0.0, 0.0)
        spin(0.05)


def step(axis, cmd):
    """Hold `cmd` on `axis` for HOLD_S, sampling the last SAMPLE_S."""
    samples = []
    gz = []
    t0 = time.monotonic()
    x0, y0, yaw0 = S["x"], S["y"], S["yaw"]
    while True:
        el = time.monotonic() - t0
        if el >= HOLD_S:
            break
        send(cmd, 0.0) if axis == "lin" else send(0.0, cmd)
        spin(0.05)
        if el >= (HOLD_S - SAMPLE_S):
            samples.append(S["vx"] if axis == "lin" else S["wz_odom"])
            gz.append(S["gz"])
    dx, dy = S["x"] - x0, S["y"] - y0
    dist = math.hypot(dx, dy)
    dyaw = math.degrees(math.atan2(math.sin(S["yaw"] - yaw0), math.cos(S["yaw"] - yaw0)))
    obs = statistics.mean(samples) if samples else 0.0
    sd = statistics.pstdev(samples) if len(samples) > 1 else 0.0
    gzm = statistics.mean(gz) if gz else 0.0
    zero(DISCHARGE_S)
    return {"cmd": cmd, "obs": obs, "sd": sd, "dist_mm": dist * 1000,
            "dyaw_deg": dyaw, "gyro_dps": math.degrees(gzm),
            "batt": S["batt"], "n": len(samples)}


def sweep(axis, sign, label, cap, stepsize):
    print(f"\n{'='*78}\n{label}\n{'='*78}")
    if axis == "lin":
        print(f"{'cmd':>8} {'obs m/s':>10} {'sd':>8} {'moved mm':>10} "
              f"{'dyaw deg':>9} {'batt':>6}")
    else:
        print(f"{'cmd':>8} {'obs rad/s':>10} {'sd':>8} {'gyro dps':>10} "
              f"{'dyaw deg':>9} {'batt':>6}")
    rows = []
    c = 0.0
    while abs(c) <= cap + 1e-9:
        r = step(axis, c * sign if c != 0 else 0.0)
        rows.append(r)
        m = r["dist_mm"] if axis == "lin" else abs(r["dyaw_deg"])
        print(f"{r['cmd']:>8.3f} {r['obs']:>10.4f} {r['sd']:>8.4f} "
              f"{r['dist_mm']:>10.1f} {r['dyaw_deg']:>9.2f} "
              f"{str(r['batt']):>6}"
              if axis == "lin" else
              f"{r['cmd']:>8.3f} {r['obs']:>10.4f} {r['sd']:>8.4f} "
              f"{r['gyro_dps']:>10.2f} {r['dyaw_deg']:>9.2f} {str(r['batt']):>6}")
        sys.stdout.flush()
        # stop early once clearly and steadily moving
        moving = [x for x in rows[-3:] if abs(x["obs"]) > 0.005]
        if len(rows) >= 4 and len(moving) == 3:
            print("  -> 3 consecutive moving steps; stopping this sweep early")
            break
        c = round(c + stepsize, 4)
    return rows


results = {}
try:
    print("settling, wheels should be free...")
    zero(1.5)
    print("battery at start:", S["batt"], " odom msgs:", S["n"])
    if S["n"] == 0:
        raise SystemExit("NO ODOM - aborting, refusing to drive blind")

    results["lin_fwd"] = sweep("lin", +1, "LINEAR FORWARD  (+linear.x)", LIN_CAP, 0.002)
    results["lin_rev"] = sweep("lin", -1, "LINEAR REVERSE  (-linear.x)", LIN_CAP, 0.002)
    results["ang_ccw"] = sweep("ang", +1, "ANGULAR LEFT/CCW (+angular.z)", ANG_CAP, 0.01)
    results["ang_cw"] = sweep("ang", -1, "ANGULAR RIGHT/CW (-angular.z)", ANG_CAP, 0.01)
finally:
    for _ in range(15):
        send(0.0, 0.0)
        spin(0.02)
    print("\nFINAL: zero Twist sent x15. battery:", S["batt"])
    rclpy.shutdown()

print("\n\n############ SUMMARY ############")
for k, rows in results.items():
    unit = "m/s" if k.startswith("lin") else "rad/s"
    first_any = next((r for r in rows if abs(r["obs"]) > 0.003), None)
    # "smooth" = moving, and the next step also moves with a larger magnitude
    smooth = None
    for i, r in enumerate(rows):
        if abs(r["obs"]) > 0.005 and r["sd"] < abs(r["obs"]) * 0.5:
            nxt = rows[i + 1] if i + 1 < len(rows) else None
            if nxt is None or abs(nxt["obs"]) >= abs(r["obs"]) * 0.8:
                smooth = r
                break
    print(f"\n{k}:")
    print(f"  first ANY motion : cmd={first_any['cmd']:.3f} obs={first_any['obs']:.4f} {unit}"
          if first_any else "  first ANY motion : NONE up to cap")
    print(f"  first SMOOTH     : cmd={smooth['cmd']:.3f} obs={smooth['obs']:.4f} {unit} "
          f"(sd {smooth['sd']:.4f})" if smooth else "  first SMOOTH     : NONE")
    for r in rows:
        if abs(r["obs"]) > 0.003:
            ratio = abs(r["obs"] / r["cmd"]) if r["cmd"] else 0
            print(f"    cmd {r['cmd']:+.3f} -> obs {r['obs']:+.4f}  ratio {ratio:.2f}")
