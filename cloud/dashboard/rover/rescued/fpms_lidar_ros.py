#!/usr/bin/env python3
"""
fpms_lidar_ros.py - publishes the rover's REAL LiDAR into ROS 2 as
sensor_msgs/msg/LaserScan on /scan_lidar.

WHY THIS NODE EXISTS
====================
Nav2 cannot localise without a laser scan in ROS, and on this rover there was
none. NAV2_BRIEF.md section 7 calls this the first link in the blocking chain:
LiDAR->ROS -> TF tree -> map -> AMCL -> costmaps -> behaviour tree.

Two things were already true and neither of them was a usable scan source:

  * The drive board publishes `/scan`, but it is DEAD - every range reads 0.0.
    Nothing here touches it, and this node deliberately does NOT publish to
    `/scan` so the two can never be confused. See TOPIC below.
  * The real D500/LD-series scanner is owned by `fpms-rover-agent`, which
    decodes it and publishes a 360-bin summary to MQTT only. That MQTT feed is
    what the dashboard's radar view draws, and it must not break.

WHY THIS SUBSCRIBES TO MQTT INSTEAD OF OPENING THE SERIAL PORT
==============================================================
Only one process can hold /dev/serial/by-path/...usb-0:1.2:1.0-port0. Three
options existed:

  (a) this node takes the port          -> kills the dashboard LiDAR view
  (b) add rclpy to fpms-rover-agent     -> breaks a deliberate design rule
  (c) re-publish the existing MQTT feed -> chosen

(b) was rejected on the strongest grounds: fpms-rover-agent is deliberately
ROS-free so that it keeps a camera and a LiDAR alive when ROS is broken. Making
the rover's fire/wildlife/obstacle telemetry depend on rclpy importing cleanly
would trade a robust subsystem for a convenience.

(a) was rejected because it breaks a working, in-use feature to gain resolution
we do not currently need.

(c) costs a round trip through the off-board MQTT broker. That cost was measured
rather than assumed - see THE RATE and THE LATENCY below.

THE RATE - measured, not assumed
================================
`fpms-rover-agent` accumulates points from 230400-baud packets and emits a
summary every 1/FPMS_LIDAR_HZ seconds. FPMS_LIDAR_HZ defaulted to 2, which is
far too slow for Nav2. The raw sensor rate is much higher than that publish
rate, so the publish rate was the only limit:

    FPMS_LIDAR_HZ=2   ->  1.90 Hz, 2529 points/msg, 353/360 bins filled (98%)
    FPMS_LIDAR_HZ=10  ->  9.83 Hz,  515 points/msg, 349/360 bins filled (97%)

That is ~5060 raw points/second off the wire either way; only the binning window
changed. At 10 Hz each message carries almost exactly one full revolution
(515 points over 360 one-degree bins), and bin coverage barely drops - 97% vs
98% - because the scanner emits points at a near-uniform angular spacing rather
than randomly.

10 Hz is therefore the natural ceiling for this approach and also the right
answer: the scanner itself spins at ~9.8 Hz, so asking for more would only slice
single revolutions into partial scans with holes in them. Nav2/AMCL is
comfortable from about 5 Hz upward, so ~9.8 Hz is comfortably adequate and this
node did not need to fall back to option (a) or (b).

FPMS_LIDAR_HZ=10 is now set in /etc/fpms/config.env. If it is ever turned back
down, this node keeps working but Nav2's costmap update rate degrades with it.

THE LATENCY - the known cost of option (c)
==========================================
The scan travels Pi -> broker (off-board, 192.168.137.1) -> Pi. Measured
round trip with a LiDAR-sized 2.5 KB payload:

    min 13.7 ms | median 99.9 ms | p90 178.6 ms | max 207.3 ms

So a scan is roughly one scan-period old by the time it is published to ROS.
That is acceptable *for this rover specifically* because it cannot move slowly
and cannot move fast: NAV2_BRIEF.md section 3b puts the usable speed near
0.18 m/s, at which 200 ms of staleness is 3.6 cm - inside one costmap cell at
the usual 5 cm resolution.

It is recorded here because it is a real cost and it is the thing to revisit
first if Nav2 ever shows lag-shaped misbehaviour (walls smearing in the
direction of travel, costmap obstacles trailing the robot). The fixes, in
increasing order of disruption: move the MQTT broker onto the Pi (removes the
WiFi round trip entirely and needs no change to this file), then option (a).

The second cost is a dependency on WiFi for a Nav2-critical topic. This node
does not paper over it: if the MQTT feed goes quiet it STOPS publishing rather
than repeating the last scan (see SCAN_STALE_SEC). A frozen scan is far more
dangerous than an absent one, because Nav2 will keep planning against a world
that is quietly no longer being observed.

WHAT THIS NODE DOES NOT DO
==========================
  * It does not publish TF. `rover/nav2/fpms_tf.launch.py` owns
    base_link -> laser_frame. Two publishers for one frame is a broken
    (non-tree) TF graph.
  * It does not touch the serial port, /cmd_vel, or any motor.
  * It has no systemd dependency on micro-ros-agent or fpms-rover-agent, and
    restarting it can never disturb either.

This module is importable WITHOUT ROS installed (the rclpy imports are guarded)
so the pure conversion helpers can be unit-tested off-robot, the same trick
fpms_odom_tf.py and deadband_sweep.py use.
"""

import json
import math
import os
import signal
import sys
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import LaserScan
    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised only off-robot
    _ROS_AVAILABLE = False
    Node = object


# ============================================================================
# !! MOUNTING ORIENTATION - NEEDS PHYSICAL VERIFICATION                     !!
# ============================================================================
# These two constants are the ENTIRE mapping between the scanner's own degree
# frame and the ROS angle convention. Every obstacle Nav2 sees is placed by
# them. If they are wrong the costmap is rotated or mirrored, which does not
# look like a bug - it looks like a robot that mysteriously refuses to localise.
#
# HOW TO VERIFY (do this before trusting any Nav2 run):
#   1. Park the rover with a flat wall roughly 1 m directly in FRONT of it and
#      nothing else within ~3 m.
#   2. ros2 topic echo /scan_lidar --once
#   3. The short returns must land at index 180 (ROS angle 0 = straight ahead),
#      NOT at index 0 or 90 or 270. Index 0 is directly BEHIND the robot.
#   4. Then put the wall only on the rover's LEFT. Short returns must appear
#      around index 270 (ROS +90 deg = left). If they appear near index 90
#      instead, the scan is mirrored: flip LIDAR_ROTATION_SIGN.
#
# EVIDENCE BEHIND THE CURRENT VALUES (good, but not a substitute for step 1-4):
# `/home/ubuntu/fpms_phase6_LATEST.py` is prior autonomy that actually worked on
# this exact hardware - 0.6% distance error, +/-1-4 degree turns. It treated the
# scanner's frame as:
#
#     front = [p for p in pts if p["deg"] <= 25 or p["deg"] >= 335]
#     def deg_xy(deg, dist): return sin(deg)*dist, cos(deg)*dist   # x right, y fwd
#
# ...which says two things. Scanner 0 degrees is straight AHEAD (hence
# LIDAR_ZERO_OFFSET_DEG = 0). And scanner degrees increase CLOCKWISE seen from
# above - deg 90 maps to x=+dist, its "right" - whereas ROS (REP-103) angles
# increase COUNTER-CLOCKWISE, positive to the left. Hence the sign is -1.
#
# The zero offset is well evidenced; phase6 aimed at markers and stopped at
# walls using it. The SIGN is the weaker of the two, because phase6's front
# check is symmetric (+/-25 deg) and so carries no left/right information on its
# own - it rests on that deg_xy convention being right. Step 4 above is what
# actually settles it. Treat both as unverified until someone has run the wall
# test and changed this comment to say so.
# ============================================================================

# Scanner bearing, in its own degrees, that points straight out the rover's nose.
LIDAR_ZERO_OFFSET_DEG = 0.0

# +1 if the scanner's degrees increase counter-clockwise (same sense as ROS),
# -1 if they increase clockwise (mirrored relative to ROS).
LIDAR_ROTATION_SIGN = -1

# ============================================================================
# TOPIC
# ============================================================================
# NOT "/scan". The drive board already publishes a dead /scan (every range 0.0,
# NAV2_BRIEF.md section 2). Publishing here as well would put a live and a dead
# publisher on one topic and hand Nav2 an interleaved stream, half of which
# claims an obstacle is touching the sensor. This name is already what
# rover/nav2/nav2_params.yaml expects in amcl/scan_topic and in both costmaps'
# observation source, so all four must be changed together or not at all.
SCAN_TOPIC = os.environ.get("FPMS_SCAN_TOPIC", "/scan_lidar")

# Set by rover/nav2/fpms_tf.launch.py as a child of base_link. This node only
# stamps the name; it does not broadcast the transform.
LASER_FRAME = "laser_frame"

# ============================================================================
# SCAN GEOMETRY
# ============================================================================
# fpms-rover-agent emits exactly 360 bins, one per degree, index 0 = scanner
# bearing 0. Anything else means the agent's BINS constant moved and the
# geometry below is a lie, so it is checked at runtime rather than trusted.
EXPECTED_BINS = 360

ANGLE_MIN = -math.pi
ANGLE_MAX = math.pi
ANGLE_INCREMENT = 2.0 * math.pi / EXPECTED_BINS

RANGE_MIN_M = 0.12
RANGE_MAX_M = 6.0

# fpms-rover-agent CLAMPS its output with min(mm/1000, 6.0), so a bin reading
# exactly 6.0 means "at or beyond 6 m", not "there is an obstacle at 6 m".
# Passing the clamp through verbatim would paint a false wall in a 6 m ring
# around the rover. Anything at or above this becomes a no-return instead. The
# cost is genuine detections in the last centimetre of range, which is nothing:
# Nav2's obstacle_range is far shorter than 6 m anyway.
RANGE_CLAMP_M = 6.0

# ============================================================================
# TIMING
# ============================================================================
# If the MQTT feed goes quiet for this long, stop publishing and say so. Sized
# at several scan periods so ordinary WiFi jitter (p90 179 ms round trip) never
# trips it, while a real outage is caught in well under a second.
SCAN_STALE_SEC = 1.0

# Nominal, used only until the real inter-scan interval has been observed.
NOMINAL_SCAN_HZ = 10.0

# Backdating the stamp by the measured transport delay is available but is NOT
# enabled, because the delay is variable (median 100 ms, p90 179 ms) and a fixed
# correction would be right only on average. 0.0 means "stamp on arrival", which
# is honest: the scan is as old as the transport made it. If Nav2 ever shows
# lag-shaped error, measure the delay again and set this - do not guess at it.
SCAN_LATENCY_COMPENSATION_S = 0.0


def log(msg):
    sys.stdout.write("%s %s\n" % (time.strftime("%H:%M:%S"), msg))
    sys.stdout.flush()


# ============================================================================
# PURE HELPERS - no ROS, no MQTT, no clock. Unit-testable off-robot.
# ============================================================================

def bin_index_for_ros_index(j, n_bins=EXPECTED_BINS,
                            zero_offset_deg=LIDAR_ZERO_OFFSET_DEG,
                            rotation_sign=LIDAR_ROTATION_SIGN):
    """Scanner bin that supplies ROS LaserScan.ranges[j].

    ROS index j sits at angle ANGLE_MIN + j*ANGLE_INCREMENT, i.e. (j - 180)
    degrees, so j=180 is straight ahead and j=0 is directly behind. Mapping back
    into the scanner's frame is that angle taken through the mount sign and
    offset. This is the single place the two conventions meet.
    """
    ros_angle_deg = -180.0 + j * (360.0 / n_bins)
    return int(round(rotation_sign * ros_angle_deg + zero_offset_deg)) % n_bins


# Built once. The mapping is fixed at import time, so doing this per scan would
# be 360 modulo operations 10 times a second for an answer that never changes.
_BIN_LUT = [bin_index_for_ros_index(j) for j in range(EXPECTED_BINS)]


def mqtt_ranges_to_laserscan_ranges(ranges_m, lut=_BIN_LUT,
                                    clamp_m=RANGE_CLAMP_M):
    """Convert the agent's 360-bin MQTT array into LaserScan.ranges.

    The critical rule, and the reason this is its own function: a bin with no
    return arrives from MQTT as 0.0 and MUST leave as inf. LaserScan has no
    "unknown" value other than inf/nan, and Nav2 reads a literal 0.0 as an
    obstacle in contact with the sensor - a rover ringed by 360 phantom
    obstacles at range zero, which reads as a lidar fault rather than a units
    bug. NAV2_BRIEF.md section 7 calls this out explicitly.
    """
    inf = float("inf")
    out = [inf] * len(lut)
    n = len(ranges_m)
    for j, b in enumerate(lut):
        if b >= n:
            continue
        r = ranges_m[b]
        # 0.0, None and negatives all mean "nothing came back from this bearing".
        if not r or r <= 0.0:
            continue
        r = float(r)
        if not math.isfinite(r) or r >= clamp_m:
            continue
        out[j] = r
    return out


# ============================================================================
# NODE
# ============================================================================

class LidarBridge(Node):
    """MQTT telemetry/lidar -> sensor_msgs/LaserScan on SCAN_TOPIC."""

    def __init__(self, cfg):
        super().__init__("fpms_lidar_ros")
        self.cfg = cfg
        self.thing = cfg.get("FPMS_THING_NAME", "rover2")
        self.topic_in = "fpms/%s/telemetry/lidar" % self.thing

        # RELIABLE so this publisher satisfies both kinds of subscriber. Nav2's
        # costmap layers subscribe with sensor-data QoS (BEST_EFFORT) and rviz2
        # commonly defaults to RELIABLE; a reliable publisher matches both,
        # whereas a best-effort one is silently incompatible with the second and
        # shows up as "the topic exists but rviz displays nothing".
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(LaserScan, SCAN_TOPIC, qos)

        self._lock = threading.Lock()
        self._last_rx = 0.0
        self._last_interval = 1.0 / NOMINAL_SCAN_HZ
        self._published = 0
        self._dropped = 0
        self._warned_bins = False
        self._alive = False

        self.create_timer(1.0, self._watchdog)
        self.create_timer(30.0, self._report)

        log("publishing %s frame_id=%s (from MQTT %s)"
            % (SCAN_TOPIC, LASER_FRAME, self.topic_in))
        log("mount: zero_offset=%.1f deg sign=%+d  <- UNVERIFIED, run the wall test"
            % (LIDAR_ZERO_OFFSET_DEG, LIDAR_ROTATION_SIGN))

    # -- MQTT side ----------------------------------------------------------

    def on_lidar_payload(self, payload):
        """Called from the paho network thread. Must never raise."""
        try:
            data = json.loads(payload)
        except Exception as e:
            self._dropped += 1
            return
        ranges_m = data.get("ranges_m")
        if not isinstance(ranges_m, list) or not ranges_m:
            self._dropped += 1
            return

        if len(ranges_m) != EXPECTED_BINS and not self._warned_bins:
            # Not fatal - the conversion indexes defensively - but the geometry
            # published below assumes 360 one-degree bins, so a change here
            # silently rotates or squashes every scan. Say it once, loudly.
            self._warned_bins = True
            log("WARNING: agent sent %d bins, expected %d. angle_increment is "
                "now wrong; check BINS in fpms-rover-agent."
                % (len(ranges_m), EXPECTED_BINS))

        now = time.time()
        with self._lock:
            if self._last_rx:
                dt = now - self._last_rx
                # Exponential average, and only over plausible intervals, so one
                # WiFi hiccup does not drag scan_time somewhere unbelievable.
                if 0.005 < dt < 1.0:
                    self._last_interval = 0.8 * self._last_interval + 0.2 * dt
            self._last_rx = now
            interval = self._last_interval

        try:
            self._publish(ranges_m, interval)
        except Exception as e:
            self._dropped += 1
            log("publish error: %s" % (e,))

    def _publish(self, ranges_m, interval):
        msg = LaserScan()

        # The Pi's ROS clock, NOT any timestamp carried in the payload.
        # fpms_odom_tf.py learned this the hard way: a stamp from another clock
        # falls outside tf2's buffer and Nav2 rejects the scan with an
        # extrapolation error that points at TF rather than at the timestamp.
        stamp = self.get_clock().now()
        if SCAN_LATENCY_COMPENSATION_S:
            stamp = stamp - rclpy.duration.Duration(
                seconds=SCAN_LATENCY_COMPENSATION_S)
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = LASER_FRAME

        msg.angle_min = ANGLE_MIN
        msg.angle_max = ANGLE_MAX
        msg.angle_increment = ANGLE_INCREMENT

        msg.scan_time = float(interval)
        # Nominal only. The agent's bins are a min-hold over the whole
        # accumulation window rather than instantaneous samples, so no exact
        # per-ray time exists. Nav2's costmap uses projectLaser(), which ignores
        # this field; it is filled in for consumers that do read it.
        msg.time_increment = float(interval) / EXPECTED_BINS

        msg.range_min = RANGE_MIN_M
        msg.range_max = RANGE_MAX_M
        msg.ranges = mqtt_ranges_to_laserscan_ranges(ranges_m)
        msg.intensities = []      # the agent does not forward per-bin intensity

        self.pub.publish(msg)
        self._published += 1
        if not self._alive:
            self._alive = True
            hits = sum(1 for r in msg.ranges if math.isfinite(r))
            log("first scan published: %d/%d bearings returned"
                % (hits, len(msg.ranges)))

    # -- health -------------------------------------------------------------

    def _watchdog(self):
        with self._lock:
            last = self._last_rx
        if not last:
            return
        age = time.time() - last
        if age > SCAN_STALE_SEC and self._alive:
            self._alive = False
            # Deliberately NOT republishing the last scan. Nav2 must see the
            # scan stop, not a frozen world it will happily keep planning in.
            log("MQTT lidar feed stale (%.1fs) - not publishing. Check "
                "fpms-rover-agent and the WiFi link." % age)
        elif age <= SCAN_STALE_SEC and not self._alive and self._published:
            self._alive = True
            log("MQTT lidar feed recovered")

    def _report(self):
        with self._lock:
            interval = self._last_interval
        log("scans=%d dropped=%d rate=%.2f Hz"
            % (self._published, self._dropped,
               (1.0 / interval) if interval > 0 else 0.0))


# ============================================================================
# CONFIG / MQTT WIRING
# ============================================================================

def load_config(path="/etc/fpms/config.env"):
    """Read the same file fpms-rover-agent reads.

    Credentials live here and only here - never in this file, which is pushed to
    a public repo.
    """
    cfg = {}
    try:
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                cfg[k.strip()] = v.strip()
    except Exception as e:
        log("config read failed (%s); falling back to environment" % (e,))
    for k in ("FPMS_THING_NAME", "FPMS_MQTT_HOST", "FPMS_MQTT_PORT",
              "FPMS_MQTT_USER", "FPMS_MQTT_PASS"):
        if os.environ.get(k):
            cfg[k] = os.environ[k]
    return cfg


def main():
    if not _ROS_AVAILABLE:
        log("rclpy not importable - source /opt/ros/humble/setup.bash")
        return 1

    import paho.mqtt.client as mqtt
    from paho.mqtt.client import CallbackAPIVersion

    cfg = load_config()
    rclpy.init()
    node = LidarBridge(cfg)

    def on_connect(client, userdata, flags, rc, properties=None):
        # QoS 0 on purpose. A queue of scans replayed after a reconnect is a
        # queue of lies about where the walls are; the freshest scan is the only
        # one with any value, and dropping the rest is the correct behaviour.
        client.subscribe(node.topic_in, qos=0)
        log("MQTT connected to %s:%s (%s), subscribed %s"
            % (cfg.get("FPMS_MQTT_HOST"), cfg.get("FPMS_MQTT_PORT"), rc,
               node.topic_in))

    def on_message(client, userdata, msg):
        node.on_lidar_payload(msg.payload)

    def on_disconnect(client, userdata, flags, rc, properties=None):
        log("MQTT disconnected (%s); paho will retry" % (rc,))

    client = mqtt.Client(client_id="%s-lidar-ros" % cfg.get("FPMS_THING_NAME", "rover"),
                         callback_api_version=CallbackAPIVersion.VERSION2)
    user = cfg.get("FPMS_MQTT_USER")
    if user:
        client.username_pw_set(user, cfg.get("FPMS_MQTT_PASS", ""))
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    host = cfg.get("FPMS_MQTT_HOST", "192.168.137.1")
    port = int(cfg.get("FPMS_MQTT_PORT", "1883"))
    try:
        client.connect(host, port, keepalive=30)
    except Exception as e:
        # Do not die - systemd would just restart us into the same broker
        # outage. paho's reconnect loop is the better waiting room.
        log("initial MQTT connect failed (%s); will keep retrying" % (e,))
    client.loop_start()

    stopping = threading.Event()

    def stop(*_):
        stopping.set()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    try:
        while not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        client.loop_stop()
        try:
            client.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        log("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
