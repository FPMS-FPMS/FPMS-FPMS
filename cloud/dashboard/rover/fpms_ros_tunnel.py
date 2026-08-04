#!/usr/bin/env python3
"""FPMS ROS tunnel - one honest path between the ESP32's micro-ROS topics and MQTT.

Runs alongside (never instead of) micro-ros-agent, fpms-teleop, fpms-missions and
fpms-rover-agent. It owns NO actuator by default: it is a listener that mirrors
the board's topics onto MQTT and, far more importantly, publishes a link-health
verdict that cannot be faked by a cached number.

WHY THIS FILE EXISTS AT ALL
---------------------------
On 2026-08-04 the dashboard showed `odom_hz = 11.1` while the micro-ROS session
was already gone. The rate had been computed once, cached, and re-published
forever; only `battery_age` was climbing, and nobody was looking at it. A dead
link therefore read as a healthy one for a long time.

The specific bug is subtle and worth naming, because the naive implementation
reproduces it every time: a Hz figure derived from a ring buffer of arrival
timestamps STAYS NON-ZERO after the topic dies, because the buffer still holds
the last few good arrivals and `(n-1)/span` over those arrivals is still ~11.
The number is not stale in the sense of "old"; it is arithmetically correct and
semantically a lie.

The fix, implemented in TopicHealth.hz() below:

  * Hz is recomputed from scratch on every read, never cached.
  * Hz is FORCED to exactly 0.0 whenever the newest arrival is older than
    STALE_S. A topic that is not currently delivering has no rate. Full stop.
  * The health verdict is produced by a WALL-CLOCK WATCHDOG TIMER, not by a
    message callback. A callback-driven health check cannot fire when the
    problem is that no callbacks are firing - which is precisely the failure
    mode we are trying to detect.
  * Every published field that describes freshness ships next to its own
    `age_s`, so a consumer can second-guess us. `hz` alone is never sufficient
    evidence of life and this file never presents it as such.

SAFETY MODEL
------------
  * This node creates NO publisher on /cmd_vel unless FPMS_TUNNEL_ALLOW_CMDVEL
    is truthy in the environment. Not "creates one and refuses to use it" - it
    does not exist as a Python object. There is nothing to accidentally call.
  * The env flag is the ONLY way to arm the /cmd_vel path. The runtime MQTT
    control topic can DISABLE it but can never ENABLE it. Safety gates that can
    be opened remotely are not gates.
  * The command topic is `fpms/<thing>/control/tunnel`, deliberately NOT under
    `fpms/<thing>/commands/`: fpms-rover-agent subscribes to `commands/#` and
    nacks every verb it does not know. Publishing a new verb there would make
    the dashboard show "unknown command" racing our real reply. Using a
    different prefix avoids that without editing anybody else's file.
  * /cmd_vel is SUBSCRIBED to unconditionally. That is read-only and it is how a
    second writer to the wire gets caught.

WHAT IT KNOWS ABOUT THIS BOARD (measured previously, trusted here)
-----------------------------------------------------------------
  * /odom_raw ~10-11 Hz, cumulative pose. twist.linear.x is sign-inverted with
    respect to its own pose.position - a firmware reporting bug. This file
    mirrors both verbatim and flags the disagreement rather than silently
    "fixing" it, because a repair applied inside a diagnostic tool is how you
    lose the ability to diagnose.
  * /imu ~25 Hz, orientation is NOT fused (identity quaternion). Heading must be
    integrated from angular_velocity.z. We report the integrated yaw as a
    convenience but label it `gyro_yaw_deg` so nobody mistakes it for a fused
    heading with an absolute zero.
  * /battery is std_msgs/UInt16 in DECIVOLTS at ~1 Hz.
  * /scan from the board is dead (all ranges 0.0). Not touched here; the live
    LiDAR is fpms-rover-agent's `telemetry/lidar`.
  * ROS_DOMAIN_ID must be 20. On any other domain this node sees nothing and
    would report DEAD forever, which is a very confusing way to discover an
    environment problem - hence the explicit warning in main().
"""

import json
import math
import os
import signal
import threading
import time
from collections import deque

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                           HistoryPolicy)
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import UInt16
    HAVE_ROS = True
    ROS_IMPORT_ERROR = ""
except Exception as _e:                                    # pragma: no cover
    HAVE_ROS = False
    ROS_IMPORT_ERROR = str(_e)
    # Fallback base class so this module can still be IMPORTED without ROS.
    # That is what lets TopicHealth - the class that exists to stop a stale rate
    # being mistaken for a live link - be unit-tested off the rover. main()
    # refuses to run in that state.
    Node = object

try:
    import paho.mqtt.client as mqtt
    from paho.mqtt.client import CallbackAPIVersion
    HAVE_MQTT = True
    MQTT_IMPORT_ERROR = ""
except Exception as _e:                                    # pragma: no cover
    HAVE_MQTT = False
    MQTT_IMPORT_ERROR = str(_e)


# ===================================================================== CONFIG
def load_config(path="/etc/fpms/config.env"):
    """Same loader shape as fpms_teleop.py / fpms_missions.py.

    Deliberately tolerant: a missing or unreadable config must not stop the
    service from starting, because the defaults below are all safe (localhost
    broker, no motion).
    """
    cfg = {}
    try:
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    cfg[k] = v.strip()
    except Exception:
        pass
    return cfg


CFG = load_config()


def _cfg(key, default):
    """Environment wins over config.env wins over the built-in default.

    Environment first so that a one-off foreground run can override a setting
    without editing a file that four other services also read.
    """
    v = os.environ.get(key)
    if v is None:
        v = CFG.get(key)
    return default if v is None else v


def _cfg_float(key, default, lo, hi):
    try:
        v = float(_cfg(key, default))
    except Exception:
        return default
    if not math.isfinite(v):
        return default
    return max(lo, min(hi, v))


def _cfg_bool(key, default=False):
    v = str(_cfg(key, "1" if default else "0")).strip().lower()
    return v in ("1", "true", "yes", "on")


THING = _cfg("FPMS_THING_NAME", "rover2")
BROKER = _cfg("FPMS_MQTT_HOST", "127.0.0.1")
PORT = int(_cfg("FPMS_MQTT_PORT", "1883"))
USER = _cfg("FPMS_MQTT_USER", "")
PASS = _cfg("FPMS_MQTT_PASS", "")

# --------------------------------------------------------------- HEALTH TUNING
# STALE_S: a topic quiet for longer than this has NO rate (hz is forced to 0.0)
# and is flagged stale. Chosen at 1.0 s because the SLOWEST topic we watch is
# /battery at ~1 Hz - anything tighter would false-positive on battery alone,
# which is why the per-topic override below exists.
STALE_S = _cfg_float("FPMS_TUNNEL_STALE_S", 1.0, 0.2, 30.0)
# DEAD_S: the whole micro-ROS link is declared DEAD after this much silence on
# the primary topic. 3.0 s matches ROS_DEAD_S in fpms_teleop.py and
# fpms_missions.py on purpose - three services disagreeing about when the link
# died is worse than any particular value.
DEAD_S = _cfg_float("FPMS_TUNNEL_DEAD_S", 3.0, 0.5, 60.0)
# How often the health snapshot goes out on MQTT.
TELEM_HZ = _cfg_float("FPMS_TUNNEL_TELEM_HZ", 5.0, 0.2, 50.0)
# The watchdog runs FASTER than the telemetry so a transition is detected and
# logged at its real time, not rounded up to the next telemetry tick.
WATCHDOG_HZ = _cfg_float("FPMS_TUNNEL_WATCHDOG_HZ", 10.0, 1.0, 50.0)
# Mirror cap. Mirrors are published the instant a message arrives (that is the
# whole point - minimum added latency), but never faster than this, because
# /imu at 25 Hz x JSON over the loopback broker is pure waste when nothing is
# consuming it that fast.
MIRROR_HZ = _cfg_float("FPMS_TUNNEL_MIRROR_HZ", 20.0, 0.0, 200.0)
BATTERY_MIRROR_HZ = _cfg_float("FPMS_TUNNEL_BATTERY_MIRROR_HZ", 2.0, 0.0, 50.0)

# ------------------------------------------------------------------ MOTION GATE
# THE gate. Default OFF. If this is false no Twist publisher is ever created.
ALLOW_CMDVEL = _cfg_bool("FPMS_TUNNEL_ALLOW_CMDVEL", False)
# Even when armed, a bridged velocity is clamped in REAL m/s before the
# CMD_SCALE division, exactly like fpms_teleop.py does it. The clamp is the last
# thing that happens; no command path can get a number past it.
CMD_SCALE = _cfg_float("FPMS_CMD_SCALE", 6.1, 0.1, 100.0)
MAX_LIN_MPS = _cfg_float("FPMS_TUNNEL_MAX_LIN_MPS", 0.25, 0.0, 1.0)
MAX_ANG_RPS = _cfg_float("FPMS_TUNNEL_MAX_ANG_RPS", 0.8, 0.0, 3.0)
# A bridged velocity expires. If the sender stops talking the wire goes to zero
# without anybody having to send a stop.
CMDVEL_DEADMAN_S = _cfg_float("FPMS_TUNNEL_CMDVEL_DEADMAN_S", 0.3, 0.05, 5.0)

# Foreground-verification aid: exit cleanly after N seconds. 0 = run forever
# (the systemd case). This exists so a smoke test does not need `timeout`, which
# would SIGKILL us mid-shutdown and prove nothing about clean exit.
RUN_S = _cfg_float("FPMS_TUNNEL_RUN_S", 0.0, 0.0, 86400.0)

STARTED = time.time()


# ==================================================================== HELPERS
def jnum(v, nd=None):
    """JSON-safe number: NaN/inf become null instead of poisoning the payload."""
    try:
        f = float(v)
    except Exception:
        return None
    if not math.isfinite(f):
        return None
    return round(f, nd) if nd is not None else f


def yaw_from_quat(q):
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def log(*a):
    print(time.strftime("[%H:%M:%S]"), *a, flush=True)


# =============================================================== TOPIC HEALTH
class TopicHealth:
    """Arrival bookkeeping for ONE topic, with a rate that cannot lie.

    The contract, and the reason this class exists as its own object rather than
    three loose dicts on the node:

        hz(now) is 0.0 whenever age(now) > stale_s.

    That single rule is what stops "odom_hz 11.1 forever" from happening again.
    The windowed average is still used while the topic IS live, because a rate
    computed from the last two samples reports a topic as dead every time one
    message arrives late on a WiFi link that swings -47 to -78 dBm.
    """

    def __init__(self, name, stale_s=None, window_s=5.0):
        self.name = name
        self.stale_s = STALE_S if stale_s is None else stale_s
        self.window_s = window_s
        self.times = deque(maxlen=512)
        self.count = 0
        self.last_mono = 0.0        # 0.0 means "never seen", not "seen at t=0"
        self.last_wall = 0.0
        self.first_mono = 0.0

    def mark(self, now=None):
        now = time.monotonic() if now is None else now
        if self.count == 0:
            self.first_mono = now
        self.count += 1
        self.last_mono = now
        self.last_wall = time.time()
        self.times.append(now)

    def seen(self):
        return self.count > 0

    def age_s(self, now=None):
        """Seconds since the last message, or None if never seen.

        None and 'a very large number' are different facts and the dashboard
        must be able to tell them apart: never-seen usually means wrong domain
        or wrong topic name, while old-but-seen means the link died.
        """
        if not self.seen():
            return None
        now = time.monotonic() if now is None else now
        return max(0.0, now - self.last_mono)

    def stale(self, now=None):
        age = self.age_s(now)
        return True if age is None else (age > self.stale_s)

    def hz(self, now=None):
        """Current publish rate. ZERO when stale. Recomputed every call.

        Note the ordering: staleness is checked BEFORE the arithmetic. It is not
        a post-hoc correction applied to a cached figure - there is no cached
        figure to correct. Anyone reading this value is reading a claim about
        RIGHT NOW.
        """
        now = time.monotonic() if now is None else now
        if not self.seen():
            return None                     # never seen != rate of zero
        if (now - self.last_mono) > self.stale_s:
            return 0.0                      # <-- the whole point of this file
        recent = [t for t in self.times if now - t <= self.window_s]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        if span <= 0:
            return 0.0
        return (len(recent) - 1) / span

    def state(self, now=None, dead_s=None):
        dead_s = DEAD_S if dead_s is None else dead_s
        age = self.age_s(now)
        if age is None:
            return "never"
        if age > dead_s:
            return "dead"
        if age > self.stale_s:
            return "stale"
        return "live"

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        return {
            "state": self.state(now),
            "hz": jnum(self.hz(now), 2),
            "age_s": jnum(self.age_s(now), 3),
            "count": self.count,
            "stale": self.stale(now),
            # Wall-clock stamp of the last message, so a human reading the JSON
            # can compare it against their own watch. Monotonic values are
            # meaningless outside this process.
            "last_wall": jnum(self.last_wall, 3) if self.seen() else None,
        }


# ======================================================================== BUS
class Bus:
    """MQTT wrapper that never raises into the caller and reconnects forever.

    Same shape as fpms_teleop.py's and fpms_missions.py's: connect_async plus
    loop_start, so a broker that is down at boot delays telemetry rather than
    preventing the node from starting.
    """

    def __init__(self, client_suffix="tunnel"):
        self.client = mqtt.Client(client_id=f"{THING}-{client_suffix}",
                                  callback_api_version=CallbackAPIVersion.VERSION2)
        if USER:
            self.client.username_pw_set(USER, PASS or None)
        self.client.reconnect_delay_set(min_delay=1, max_delay=15)
        self.client.will_set(f"fpms/{THING}/events/offline",
                             json.dumps({"thing": THING, "status": "offline",
                                         "svc": client_suffix,
                                         "reason": "unexpected disconnect"}),
                             qos=1, retain=False)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.connected = False
        self.on_control = None
        self.sub_topics = []
        self.online_payload = {}

    def start(self):
        try:
            self.client.connect_async(BROKER, PORT, keepalive=30)
            self.client.loop_start()
            log(f"MQTT: connecting to {BROKER}:{PORT} (async, retries forever)")
        except Exception as e:
            log(f"MQTT: start failed ({e}); node continues without MQTT")

    def stop(self):
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass

    def _on_connect(self, client, _u, _f, reason_code, _p=None):
        try:
            ok = (reason_code == 0 or getattr(reason_code, "value", 1) == 0)
            self.connected = bool(ok)
            log(f"MQTT: connected rc={reason_code}")
            if not ok:
                return
            for t in self.sub_topics:
                client.subscribe(t, qos=1)
                log(f"MQTT: subscribed {t}")
            if self.online_payload:
                self.publish("events/online", dict(self.online_payload), qos=1)
        except Exception as e:
            log(f"MQTT: on_connect failed: {e}")

    def _on_disconnect(self, client, _u, *a):
        self.connected = False
        log("MQTT: disconnected (will retry)")

    def _on_message(self, client, _u, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8") or "{}")
            if not isinstance(payload, dict):
                payload = {"value": payload}
        except Exception:
            payload = {"raw": msg.payload.decode("utf-8", "replace")}
        try:
            if self.on_control:
                self.on_control(msg.topic, payload)
        except Exception as e:
            log(f"control handler failed: {e}")

    def publish(self, suffix, payload, qos=0):
        if not isinstance(payload, dict):
            payload = {"value": payload}
        try:
            payload.setdefault("ts", time.time())
            payload.setdefault("thing", THING)
            self.client.publish(f"fpms/{THING}/{suffix}", json.dumps(payload),
                                qos=qos)
        except Exception as e:
            log(f"publish failed on {suffix}: {e}")


# ================================================================ TUNNEL NODE
class TunnelNode(Node):
    """Subscribes the board's topics, mirrors them to MQTT, judges the link.

    ROS callbacks WRITE state under self.lock; the timers READ it. There is no
    other thread that touches ROS state - paho's network thread only enqueues
    control requests, which are handled under the same lock.
    """

    def __init__(self, bus):
        super().__init__("fpms_ros_tunnel")
        self.bus = bus
        self.lock = threading.RLock()
        self.shutdown = threading.Event()
        # Set HERE, not from main(), so the watchdog can never read it before it
        # exists. "Assigned later by the caller" is how an AttributeError ends
        # up inside the one timer that is supposed to notice things are broken.
        self._boot_mono = time.monotonic()

        # RELIABLE/VOLATILE/KEEP_LAST(10) matches what fpms_missions.py and
        # fpms_teleop.py already use against this board. A QoS mismatch here
        # would silently produce zero messages, which this node would then
        # correctly-but-uselessly report as DEAD.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.h_odom = TopicHealth("/odom_raw")
        self.h_imu = TopicHealth("/imu")
        # /battery arrives at ~1 Hz, so a 1 s staleness window would flap. Give
        # it its own, wider window; the numbers still have to be justified per
        # topic rather than shared out of tidiness.
        self.h_batt = TopicHealth("/battery", stale_s=max(3.0, STALE_S * 3.0),
                                  window_s=20.0)
        self.h_cmdvel = TopicHealth("/cmd_vel")
        self.topics = {"odom_raw": self.h_odom, "imu": self.h_imu,
                       "battery": self.h_batt, "cmd_vel": self.h_cmdvel}

        self.create_subscription(Odometry, "/odom_raw", self._on_odom, qos)
        self.create_subscription(Imu, "/imu", self._on_imu, qos)
        self.create_subscription(UInt16, "/battery", self._on_battery, qos)
        # Read-only. Listening to the actuator topic is how a second writer is
        # detected; it is NOT a step towards writing to it.
        self.create_subscription(Twist, "/cmd_vel", self._on_cmdvel, qos)

        # ---------------------------------------------------------- THE GATE
        # The publisher object only exists if the environment armed it. There is
        # deliberately no `self.pub_cmd = None` fallback that later code could
        # be tempted to fill in.
        self.cmdvel_armed = bool(ALLOW_CMDVEL)
        if self.cmdvel_armed:
            self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", qos)
            log("WARNING: /cmd_vel bridge is ARMED "
                "(FPMS_TUNNEL_ALLOW_CMDVEL is set). This node can move the "
                "rover.")
        else:
            log("/cmd_vel bridge DISARMED (default). No Twist publisher was "
                "created; this node is read-only with respect to motion.")

        # Latest values, mirrored verbatim.
        self.odom = None
        self.imu = None
        self.batt_dv = None
        self.cmdvel_seen = None
        self.gyro_yaw_deg = 0.0     # integrated from /imu wz; NO absolute zero
        self._imu_last_t = 0.0

        # Bridged velocity request (only meaningful when armed).
        self.req_lin = 0.0
        self.req_ang = 0.0
        self.req_t = 0.0

        # Mirror rate limiting.
        self._mirror_last = {}
        self.mirror_lat_ms = deque(maxlen=200)

        # Link verdict, owned by the watchdog timer.
        self.link_state = "INIT"
        self.link_since = time.monotonic()
        self.link_reason = "waiting for first message"
        self.transitions = 0

        self.create_timer(1.0 / WATCHDOG_HZ, self._watchdog_tick)
        self.create_timer(1.0 / TELEM_HZ, self._telem_tick)
        if RUN_S > 0:
            self.create_timer(0.25, self._run_limit_tick)

    # ------------------------------------------------------------ CALLBACKS
    def _mirror_ok(self, key, cap_hz, now):
        """True if enough time has passed to mirror this topic again."""
        if cap_hz <= 0:
            return False
        last = self._mirror_last.get(key, 0.0)
        if (now - last) < (1.0 / cap_hz):
            return False
        self._mirror_last[key] = now
        return True

    def _on_odom(self, msg):
        t0 = time.monotonic()
        with self.lock:
            self.h_odom.mark(t0)
            p = msg.pose.pose.position
            yaw = math.degrees(yaw_from_quat(msg.pose.pose.orientation))
            tw = msg.twist.twist
            self.odom = {
                "x_m": jnum(p.x, 4), "y_m": jnum(p.y, 4),
                "yaw_deg": jnum(wrap180(yaw), 2),
                # Mirrored VERBATIM including the known sign bug. See module
                # docstring: twist.linear.x is sign-inverted with respect to
                # this same message's pose.position. Repairing it here would
                # make this diagnostic unable to show the fault it exists to
                # expose. Consumers should differentiate pose instead.
                "vx_mps_raw": jnum(tw.linear.x, 4),
                "wz_rps_raw": jnum(tw.angular.z, 4),
                "twist_sign_warning": "twist.linear.x is inverted vs pose; "
                                      "prefer differentiated pose",
            }
            snap = dict(self.odom)
        if self._mirror_ok("odom", MIRROR_HZ, t0):
            self.bus.publish("telemetry/tunnel/odom", snap)
            self.mirror_lat_ms.append((time.monotonic() - t0) * 1000.0)

    def _on_imu(self, msg):
        t0 = time.monotonic()
        with self.lock:
            self.h_imu.mark(t0)
            wz = float(msg.angular_velocity.z)
            # Integrate heading here because the board publishes an IDENTITY
            # quaternion - there is no fused orientation to read. dt is guarded
            # so that a stall followed by a burst cannot inject a huge step.
            if self._imu_last_t > 0:
                dt = t0 - self._imu_last_t
                if 0.0 < dt < 0.5:
                    self.gyro_yaw_deg = wrap180(
                        self.gyro_yaw_deg + math.degrees(wz) * dt)
            self._imu_last_t = t0
            self.imu = {
                "wz_rps": jnum(wz, 4),
                "ax_mps2": jnum(msg.linear_acceleration.x, 3),
                "ay_mps2": jnum(msg.linear_acceleration.y, 3),
                "az_mps2": jnum(msg.linear_acceleration.z, 3),
                # Named gyro_* on purpose: this is a RELATIVE heading with no
                # absolute reference. Calling it "heading" invites somebody to
                # navigate with it across a power cycle.
                "gyro_yaw_deg": jnum(self.gyro_yaw_deg, 2),
                "orientation_fused": False,
            }
            snap = dict(self.imu)
        if self._mirror_ok("imu", MIRROR_HZ, t0):
            self.bus.publish("telemetry/tunnel/imu", snap)
            self.mirror_lat_ms.append((time.monotonic() - t0) * 1000.0)

    def _on_battery(self, msg):
        t0 = time.monotonic()
        with self.lock:
            self.h_batt.mark(t0)
            # std_msgs/UInt16 in DECIVOLTS. Not sensor_msgs/BatteryState.
            self.batt_dv = int(msg.data)
            snap = {"decivolts": self.batt_dv,
                    "volts": jnum(self.batt_dv / 10.0, 2)}
        if self._mirror_ok("battery", BATTERY_MIRROR_HZ, t0):
            self.bus.publish("telemetry/tunnel/battery", snap)

    def _on_cmdvel(self, msg):
        t0 = time.monotonic()
        with self.lock:
            self.h_cmdvel.mark(t0)
            self.cmdvel_seen = {"lin_x": jnum(msg.linear.x, 4),
                                "ang_z": jnum(msg.angular.z, 4)}
            snap = dict(self.cmdvel_seen)
        if self._mirror_ok("cmd_vel", MIRROR_HZ, t0):
            self.bus.publish("telemetry/tunnel/cmd_vel", snap)

    # ------------------------------------------------------------- WATCHDOG
    def _judge_link(self, now):
        """Return (state, reason) purely from message AGE. No cached rates.

        /odom_raw is the primary witness: it is the highest-rate topic the board
        publishes and the one every other service already keys off. /imu is a
        secondary witness so that an odometry-specific firmware fault is not
        mistaken for the whole link being gone.
        """
        odom_age = self.h_odom.age_s(now)
        imu_age = self.h_imu.age_s(now)
        batt_age = self.h_batt.age_s(now)
        ages = [a for a in (odom_age, imu_age, batt_age) if a is not None]

        if not ages:
            up = now - self._boot_mono
            if up < DEAD_S:
                return "INIT", f"no message yet ({up:.1f}s since start)"
            return "DEAD", ("no message on /odom_raw, /imu or /battery since "
                            f"start ({up:.1f}s) - micro-ROS session absent, or "
                            "ROS_DOMAIN_ID is wrong")

        newest = min(ages)
        if newest > DEAD_S:
            return "DEAD", (f"newest board message is {newest:.1f}s old "
                            f"(> {DEAD_S:.1f}s)")
        if odom_age is None:
            return "PARTIAL", "/odom_raw never seen; other board topics are live"
        if odom_age > DEAD_S:
            return "PARTIAL", (f"/odom_raw silent {odom_age:.1f}s while another "
                               "board topic is live")
        if odom_age > self.h_odom.stale_s:
            return "STALE", f"/odom_raw {odom_age:.2f}s old"
        return "ALIVE", "ok"

    def _watchdog_tick(self):
        """Wall-clock health judgement. Runs whether or not messages arrive.

        This is the structural fix for the cached-Hz bug: a health check driven
        by a message callback goes quiet at exactly the moment it is needed. A
        timer keeps ticking and keeps re-deciding.
        """
        now = time.monotonic()
        with self.lock:
            state, reason = self._judge_link(now)
            if state != self.link_state:
                prev = self.link_state
                self.link_state = state
                self.link_reason = reason
                self.link_since = now
                self.transitions += 1
                log(f"LINK {prev} -> {state}: {reason}")
                self.bus.publish("events/tunnel",
                                 {"svc": "tunnel", "event": "link_state",
                                  "from": prev, "to": state, "reason": reason},
                                 qos=1)
            else:
                self.link_reason = reason

            # Bridged-velocity deadman. Even when armed, a request that stopped
            # being refreshed must decay to zero on its own; a stop that has to
            # be sent is not a stop.
            if self.cmdvel_armed and self.req_t > 0:
                if (now - self.req_t) > CMDVEL_DEADMAN_S:
                    if self.req_lin != 0.0 or self.req_ang != 0.0:
                        log("cmd_vel bridge: request stale, zeroing")
                    self.req_lin = 0.0
                    self.req_ang = 0.0

        if self.cmdvel_armed:
            self._publish_bridged()

    def _publish_bridged(self):
        """Only ever reached when ALLOW_CMDVEL armed the publisher."""
        with self.lock:
            lin = clamp(self.req_lin, -MAX_LIN_MPS, MAX_LIN_MPS)
            ang = clamp(self.req_ang, -MAX_ANG_RPS, MAX_ANG_RPS)
            link_ok = self.link_state in ("ALIVE", "STALE")
        if not link_ok:
            lin = 0.0
            ang = 0.0
        t = Twist()
        # Clamp in REAL m/s first, then divide by CMD_SCALE, exactly the order
        # fpms_teleop.py uses. Scaling before clamping would let the scale
        # constant silently widen the limit.
        t.linear.x = lin / CMD_SCALE
        t.angular.z = ang / CMD_SCALE
        try:
            self.pub_cmd.publish(t)
        except Exception as e:
            log(f"cmd_vel publish failed: {e}")

    # ------------------------------------------------------------ TELEMETRY
    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            topics = {k: v.snapshot(now) for k, v in self.topics.items()}
            lat = list(self.mirror_lat_ms)
            snap = {
                "svc": "tunnel",
                "uptime_s": jnum(time.time() - STARTED, 1),
                # THE headline field. A consumer that reads only this and
                # link_reason is reading the truth.
                "link_state": self.link_state,
                "link_reason": self.link_reason,
                "link_for_s": jnum(now - self.link_since, 1),
                "link_transitions": self.transitions,
                # Kept for compatibility with existing dashboard code, and
                # derived from link_state rather than from any rate.
                "ros_ok": self.link_state in ("ALIVE", "STALE"),
                "stale": self.link_state not in ("ALIVE",),
                "topics": topics,
                "odom": self.odom,
                "imu": self.imu,
                "battery": (None if self.batt_dv is None else
                            {"decivolts": self.batt_dv,
                             "volts": jnum(self.batt_dv / 10.0, 2)}),
                "cmd_vel_last_seen": self.cmdvel_seen,
                "cmd_vel_bridge": {
                    "armed": self.cmdvel_armed,
                    "env_flag": "FPMS_TUNNEL_ALLOW_CMDVEL",
                    "publisher_exists": self.cmdvel_armed,
                    "max_lin_mps": MAX_LIN_MPS,
                    "max_ang_rps": MAX_ANG_RPS,
                    "deadman_s": CMDVEL_DEADMAN_S,
                },
                "thresholds": {"stale_s": STALE_S, "dead_s": DEAD_S},
                "domain_id": os.environ.get("ROS_DOMAIN_ID"),
            }
        if lat:
            snap["mirror_latency_ms"] = {
                "mean": jnum(sum(lat) / len(lat), 3),
                "max": jnum(max(lat), 3),
                "n": len(lat),
                "note": "ROS callback entry to MQTT publish, Pi-internal only",
            }
        return snap

    def _telem_tick(self):
        self.bus.publish("telemetry/tunnel", self.snapshot())

    def _run_limit_tick(self):
        if (time.time() - STARTED) >= RUN_S:
            log(f"FPMS_TUNNEL_RUN_S={RUN_S:.0f}s elapsed; exiting cleanly")
            # Set the event and NOTHING else. Calling rclpy.shutdown() from
            # inside a callback the executor is currently running hangs the
            # process - measured on this Pi 2026-08-04: telemetry stopped at the
            # deadline and the process then sat alive for minutes. main() spins
            # in slices and watches this event instead.
            self.shutdown.set()

    # -------------------------------------------------------------- CONTROL
    def on_control(self, topic, payload):
        """`fpms/<thing>/control/tunnel`. Never under commands/ - see docstring."""
        cmd = str(payload.get("cmd") or payload.get("action") or "status").lower()
        if cmd in ("status", "query"):
            self.bus.publish("telemetry/tunnel", self.snapshot())
            return
        if cmd == "disarm":
            # Runtime can only ever move the gate towards SAFE.
            with self.lock:
                was = self.cmdvel_armed
                self.cmdvel_armed = False
                self.req_lin = 0.0
                self.req_ang = 0.0
            log(f"control: cmd_vel bridge disarmed (was armed={was})")
            self.bus.publish("events/tunnel", {"svc": "tunnel", "ack": "disarm",
                                               "armed": False}, qos=1)
            return
        if cmd == "arm":
            # Deliberately refused. Arming is an environment/systemd decision,
            # made by a person with physical access to the rover, not a message
            # that anything on the LAN can publish.
            log("control: 'arm' REFUSED - arming is env-only "
                "(FPMS_TUNNEL_ALLOW_CMDVEL) by design")
            self.bus.publish("events/tunnel",
                             {"svc": "tunnel", "nack": "arm",
                              "error": "cmd_vel bridge can only be armed via "
                                       "FPMS_TUNNEL_ALLOW_CMDVEL in the unit "
                                       "environment; runtime arming is refused "
                                       "by design"}, qos=1)
            return
        if cmd == "cmd_vel":
            if not self.cmdvel_armed:
                self.bus.publish("events/tunnel",
                                 {"svc": "tunnel", "nack": "cmd_vel",
                                  "error": "bridge disarmed; no /cmd_vel "
                                           "publisher exists in this process"},
                                 qos=1)
                return
            try:
                lin = float(payload.get("lin_x", payload.get("linear", 0.0)))
                ang = float(payload.get("ang_z", payload.get("angular", 0.0)))
            except Exception:
                lin = ang = 0.0
            with self.lock:
                self.req_lin = clamp(lin, -MAX_LIN_MPS, MAX_LIN_MPS)
                self.req_ang = clamp(ang, -MAX_ANG_RPS, MAX_ANG_RPS)
                self.req_t = time.monotonic()
            return
        self.bus.publish("events/tunnel", {"svc": "tunnel", "nack": cmd,
                                           "error": "unknown tunnel command"},
                         qos=1)


# ======================================================================= MAIN
def main():
    if not HAVE_ROS:
        raise SystemExit(f"rclpy is required to run this service "
                         f"({ROS_IMPORT_ERROR}). Source "
                         "/opt/ros/humble/setup.bash and set ROS_DOMAIN_ID=20.")
    if not HAVE_MQTT:
        raise SystemExit(f"paho-mqtt is required to run this service "
                         f"({MQTT_IMPORT_ERROR}).")

    dom = os.environ.get("ROS_DOMAIN_ID")
    if dom != "20":
        # Not fatal, but on the wrong domain this node sees NOTHING and would
        # report DEAD forever - which looks exactly like a dead board. Two
        # sessions have already lost time to that confusion.
        log(f"WARNING: ROS_DOMAIN_ID={dom!r}, expected '20' - this rover's "
            "board publishes on domain 20 only; on any other domain this "
            "node will report DEAD regardless of the hardware")

    bus = Bus("tunnel")
    bus.sub_topics = [f"fpms/{THING}/control/tunnel"]
    bus.online_payload = {
        "svc": "tunnel", "status": "online",
        "control_topic": f"fpms/{THING}/control/tunnel",
        "telemetry": [f"fpms/{THING}/telemetry/tunnel",
                      f"fpms/{THING}/telemetry/tunnel/odom",
                      f"fpms/{THING}/telemetry/tunnel/imu",
                      f"fpms/{THING}/telemetry/tunnel/battery",
                      f"fpms/{THING}/telemetry/tunnel/cmd_vel"],
        "commands": ["status", "disarm", "cmd_vel(armed only)"],
        "cmd_vel_armed": bool(ALLOW_CMDVEL),
    }
    bus.start()

    rclpy.init()
    node = TunnelNode(bus)
    bus.on_control = lambda _t, p: node.on_control(_t, p)

    def _sig(_s, _f):
        # Same rule as the run-limit timer: set the event, do not touch rclpy
        # from inside the handler. SIGTERM arrives on the main thread, which may
        # be mid-callback; shutting the context down from there deadlocks.
        log("signal: shutting down tunnel")
        node.shutdown.set()

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    log(f"fpms_ros_tunnel up: thing={THING} broker={BROKER}:{PORT} "
        f"stale={STALE_S:.1f}s dead={DEAD_S:.1f}s telem={TELEM_HZ:.1f}Hz "
        f"cmd_vel_armed={bool(ALLOW_CMDVEL)} run_s={RUN_S:.0f}")
    # NOT rclpy.spin(). Two things need to end this loop - the run-limit timer
    # and SIGTERM - and both of them run on the executor's own thread, where
    # calling rclpy.shutdown() hangs the process instead of ending it. Spinning
    # in 0.1 s slices makes a plain threading.Event the single stop signal, and
    # bounds how long systemd waits for us at TimeoutStopSec.
    try:
        while rclpy.ok() and not node.shutdown.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        log("interrupted")
    except Exception as e:
        log(f"spin ended: {e}")
    finally:
        # If and only if this process ever armed the wire, leave a stop behind.
        # A node that never published must NOT publish on the way out: on this
        # board an unexpected Twist at teardown is a real hazard.
        try:
            if getattr(node, "cmdvel_armed", False):
                z = Twist()
                for _ in range(10):
                    node.pub_cmd.publish(z)
                    time.sleep(0.02)
        except Exception:
            pass
        bus.publish("events/tunnel", {"svc": "tunnel", "status": "offline",
                                      "reason": "clean shutdown"}, qos=1)
        time.sleep(0.2)
        bus.stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        log("fpms_ros_tunnel exited cleanly")


if __name__ == "__main__":
    main()
