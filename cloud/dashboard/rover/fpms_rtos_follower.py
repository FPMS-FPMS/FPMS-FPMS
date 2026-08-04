#!/usr/bin/env python3
"""FPMS RTOS follower - the Pi-side fixed-rate path follower that pairs with an
RTOS-style firmware.

READ THIS FIRST - WHAT "RTOS" DOES AND DOES NOT MEAN HERE
--------------------------------------------------------
This file is a HARD-REAL-TIME-SHAPED LOOP ON THE PI. It is named for the
firmware architecture it is designed to pair with, not for anything currently
running on the ESP32. The board runs Yahboom factory firmware v2.0.0 and is not
to be reflashed. There is NO FreeRTOS control task on the board today, and
nothing in this file puts one there. If somebody tells you the rover "runs an
RTOS", the honest answer is: the Pi runs a fixed-rate loop, the board runs
vendor firmware, and the loop is written so that a future firmware-side task
could take it over without changing the geometry.

WHERE THE MOTION MATH COMES FROM
--------------------------------
Everything about the control law is lifted from the golden "B8B"/phase6 driver
(`fpms_phase6_LATEST.py`, live path `_p5_navdrive` at 1307-1516). That driver
measured 0.6% distance error and +/-1-4 degree turns on this exact chassis. It
is the asset; this file EXTENDS its structure and invents no new motion math.

What is carried over verbatim:

    _DRV  = 26      drive duty out of 100      -> DRV_DUTY
    _KPH  = 10      heading P-gain             -> KPH   (duty counts per RADIAN)
    clamp = +/-6    heading trim clamp         -> TRIM_CLAMP
    _HZ   = 20      Pi-side control rate       -> HZ
    hd    = integral of gyro wz, guarded 0<dt<0.5

    c = clamp(hd * _KPH, -6, +6)
    bot.set_motor(pwr-c, pwr-c, pwr+c, pwr+c)

Note what that gain actually means, because it is easy to misread: `hd` is in
RADIANS (B8B compares it against `math.radians(deg)` in `_spin`), so KPH=10
duty-counts-per-radian saturates the +/-6 clamp at 0.6 rad = 34 degrees of
heading error. The clamp is not a safety afterthought, it IS the control
authority: +/-6 on a base of 26 is a +/-23% differential.

What is IMPROVED over B8B, and why:

  1. B8B drives one straight segment at a time towards a heading of zero. This
     follower does PURE PURSUIT along a polyline, so the same P-trim now servos
     towards the bearing of a lookahead point instead of towards "straight
     ahead". The trim law is unchanged; only its setpoint got smarter.
  2. B8B's loop is `time.sleep(1.0/_HZ)` at the bottom, which accumulates every
     millisecond the body took. This loop schedules against an absolute
     monotonic deadline and MEASURES the jitter it failed to remove. An
     unmeasured control period is a lie about the gains.
  3. B8B has no deadman at all: `_spin` with a dead gyro spins forever. Here a
     stale pose or a stale path zeroes the output within DEADMAN_MS.
  4. B8B has an implicit state machine spread across returned strings
     ("DONE"/"BLOCKED"/"TIMEOUT"). Here it is explicit and published.

THE DUTY -> VELOCITY TRANSLATION, AND WHY IT IS THE WEAK POINT
--------------------------------------------------------------
B8B commanded RAW DUTY. This node emits geometry_msgs/Twist, because /cmd_vel is
the only actuator interface the factory firmware exposes. The trim therefore has
to be converted from duty counts into an angular velocity:

    f     = c / DRV_DUTY                 differential as a fraction of base
    v_l   = v * (1 - f),  v_r = v * (1 + f)
    omega = (v_r - v_l) / WHEELBASE_M = 2 * v * f / WHEELBASE_M

This conversion is GEOMETRY, not measurement. The duty->speed curve of these
motors is not linear and has a dead zone; the mapping is honest about its
assumptions and is reported in telemetry as `trim_model` so a reader can see
exactly what was assumed. It is UNVERIFIED on hardware. Treat any live run as a
first-time calibration, not as a repeat of B8B's measured accuracy.

WHEELBASE_M defaults to 0.105 - the MEASURED 105 mm. The config file still says
170 mm; that value is wrong and is not read here.

SAFETY MODEL
------------
  * DRY RUN IS THE DEFAULT. In dry run no /cmd_vel publisher object is created.
    The velocities the loop computes are published on MQTT and nowhere else.
  * Leaving dry run requires FPMS_FOLLOW_DRY_RUN=0 in the unit environment AND
    an explicit {"cmd":"start","live":true}. Two independent acts, one of which
    needs a shell on the rover.
  * The deadman is a property of the tick, not of a command: every tick that
    cannot prove it has a fresh pose AND a fresh path emits zero. It does not
    matter whether any message ever arrives again.
  * Pose comes from /odom_raw ONLY. fpms_teleop.py anchors its origin from
    /odom_raw, and /odom carries an independent accumulated origin; mixing them
    produced a stable 4.93 m offset on 2026-08-04 that looked like a correct
    pose because it never drifted. Do not add an /odom subscription here
    without also changing the anchor.
  * The control topic is `fpms/<thing>/control/follower`, NOT under
    `fpms/<thing>/commands/`, because fpms-rover-agent subscribes to
    `commands/#` and nacks every verb it does not know.
  * This node never sends `arm` or a `mission`. It does not talk to
    fpms-missions at all; it can OBSERVE `telemetry/mission_plan` as a path
    source, read-only, when explicitly configured to.
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
    # That is what lets the pure geometry (seg_project, Path, _parse_path) be
    # unit-tested off the rover, on a laptop, without a ROS install. main()
    # refuses to run in that state, so nothing can accidentally drive from a
    # half-initialised environment.
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

# ------------------------------------------------------- B8B CONSTANTS (LIVE)
# Do not "tidy" these. Each one was measured on this chassis; see the module
# docstring and GOLDEN_B8B_STUDY.md.
HZ = _cfg_float("FPMS_FOLLOW_HZ", 20.0, 2.0, 100.0)     # B8B _HZ
DRV_DUTY = _cfg_float("FPMS_FOLLOW_DRV_DUTY", 26.0, 1.0, 100.0)   # B8B _DRV
KPH = _cfg_float("FPMS_FOLLOW_KPH", 10.0, 0.0, 200.0)   # B8B _KPH, per RADIAN
TRIM_CLAMP = _cfg_float("FPMS_FOLLOW_TRIM_CLAMP", 6.0, 0.0, 26.0)  # B8B +/-6
# Measured 105 mm. config.env's 170 is wrong and is deliberately not read.
WHEELBASE_M = _cfg_float("FPMS_FOLLOW_WHEELBASE_M", 0.105, 0.02, 1.0)

# ------------------------------------------------------------- PURE PURSUIT
# Lookahead. Larger = smoother and lazier about cross-track; smaller = twitchier
# and more likely to oscillate at 20 Hz. 250 mm is roughly two chassis lengths
# of travel at CRUISE_MPS per second, which keeps the correction gentle enough
# for the +/-6 clamp to be the binding constraint rather than the geometry.
LOOKAHEAD_MM = _cfg_float("FPMS_FOLLOW_LOOKAHEAD_MM", 250.0, 20.0, 3000.0)
ARRIVE_MM = _cfg_float("FPMS_FOLLOW_ARRIVE_MM", 40.0, 1.0, 1000.0)
# Cross-track distance beyond which the follower refuses to keep driving. B8B
# had no such check because it drove blind straight lines; a follower that can
# be handed an arbitrary path needs one, or a bad path drives the rover away.
MAX_XTRACK_MM = _cfg_float("FPMS_FOLLOW_MAX_XTRACK_MM", 600.0, 50.0, 10000.0)
# Heading error beyond which forward speed is withheld and only the trim runs -
# B8B's `_face_and_drive` spins first when |err| > 15 deg. Same idea, expressed
# continuously so there is no separate spin state to get stuck in.
FACE_FIRST_DEG = _cfg_float("FPMS_FOLLOW_FACE_FIRST_DEG", 25.0, 1.0, 180.0)

# --------------------------------------------------------------- SPEED LIMITS
CRUISE_MPS = _cfg_float("FPMS_MISSION_CRUISE_MPS", 0.18, 0.01, 1.0)
MAX_LIN_MPS = _cfg_float("FPMS_FOLLOW_MAX_LIN_MPS", 0.25, 0.0, 1.0)
MAX_ANG_RPS = _cfg_float("FPMS_FOLLOW_MAX_ANG_RPS", 0.8, 0.0, 3.0)
# fpms_teleop.py's measured fudge: commanding 0.10 produced ~0.61 m/s of real
# ground speed on stock firmware. Everything above is REAL m/s; this is the only
# place it becomes a wire number, and it happens AFTER every clamp.
CMD_SCALE = _cfg_float("FPMS_CMD_SCALE", 6.1, 0.1, 100.0)

# ------------------------------------------------------------------- DEADMAN
DEADMAN_MS = _cfg_float("FPMS_FOLLOW_DEADMAN_MS", 300.0, 50.0, 5000.0)
# Path staleness. A path is a one-shot object, so "fresh path" means the
# commander is still asserting it via `keepalive`. Set to 0 to require only a
# fresh POSE - the pose deadman can never be switched off.
PATH_DEADMAN_MS = _cfg_float("FPMS_FOLLOW_PATH_DEADMAN_MS", 300.0, 0.0, 60000.0)
# Whole-link death, same 3.0 s as fpms_teleop.py / fpms_missions.py.
ROS_DEAD_S = _cfg_float("FPMS_FOLLOW_ROS_DEAD_S", 3.0, 0.5, 60.0)

# ------------------------------------------------------------------- DRY RUN
# THE default. Changing it is an act performed on the rover, by a person.
DRY_RUN = _cfg_bool("FPMS_FOLLOW_DRY_RUN", True)
TELEM_HZ = _cfg_float("FPMS_FOLLOW_TELEM_HZ", 5.0, 0.2, 50.0)
RUN_S = _cfg_float("FPMS_FOLLOW_RUN_S", 0.0, 0.0, 86400.0)

STARTED = time.time()


# ===================================================================== STATES
# Explicit, published, and the ONLY thing that decides whether a velocity is
# non-zero. `S_FOLLOW` is the single state in which output may be non-zero.
S_INIT = "INIT"          # node up, nothing known yet
S_IDLE = "IDLE"          # ready, no path, output zero
S_FOLLOW = "FOLLOW"      # actively tracking a path
S_DEADMAN = "DEADMAN"    # had a path, lost pose or path freshness -> zero
S_ARRIVED = "ARRIVED"    # end of path within ARRIVE_MM -> zero
S_FAULT = "FAULT"        # path rejected, cross-track blown, or link dead
S_STOPPED = "STOPPED"    # explicit stop; needs a new start


# ==================================================================== HELPERS
def jnum(v, nd=None):
    try:
        f = float(v)
    except Exception:
        return None
    if not math.isfinite(f):
        return None
    return round(f, nd) if nd is not None else f


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def wrap_pi(rad):
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q):
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)


def log(*a):
    print(time.strftime("[%H:%M:%S]"), *a, flush=True)


def percentile(vals, p):
    if not vals:
        return None
    s = sorted(vals)
    i = int(round((len(s) - 1) * p))
    return s[max(0, min(len(s) - 1, i))]


# =================================================================== GEOMETRY
def seg_project(px, py, ax, ay, bx, by):
    """Project P onto segment AB. Returns (t, cx, cy, perp_signed).

    perp_signed is positive when P lies to the LEFT of A->B, matching the
    right-hand convention used for yaw everywhere else in this codebase. Sign
    consistency matters: a cross-track term with the wrong sign is positive
    feedback, which is exactly the class of bug that made the velocity-PID
    firmware drive into a wall.
    """
    dx = bx - ax
    dy = by - ay
    den = dx * dx + dy * dy
    if den <= 1e-9:
        return 0.0, ax, ay, math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / den
    t = clamp(t, 0.0, 1.0)
    cx = ax + t * dx
    cy = ay + t * dy
    cross = dx * (py - ay) - dy * (px - ax)
    perp = math.copysign(math.hypot(px - cx, py - cy), cross)
    return t, cx, cy, perp


class Path:
    """A polyline in millimetres, in the /odom_raw frame, plus its freshness."""

    def __init__(self, pts, frame="odom_raw"):
        self.pts = [(float(a), float(b)) for a, b in pts]
        self.frame = frame
        self.stamp = time.monotonic()
        self.idx = 0                # index of the segment currently tracked
        self.seq = 0

    def touch(self):
        self.stamp = time.monotonic()

    def age_s(self, now=None):
        now = time.monotonic() if now is None else now
        return max(0.0, now - self.stamp)

    def length_mm(self):
        return sum(math.hypot(self.pts[i + 1][0] - self.pts[i][0],
                              self.pts[i + 1][1] - self.pts[i][1])
                   for i in range(len(self.pts) - 1))

    def advance(self, px, py):
        """Walk `idx` forward past segments already behind the rover.

        Only ever moves FORWARD. A follower allowed to re-select an earlier
        segment can lock into a loop on a path that doubles back, which every
        retrace route in this project does.
        """
        while self.idx < len(self.pts) - 2:
            ax, ay = self.pts[self.idx]
            bx, by = self.pts[self.idx + 1]
            t, _, _, _ = seg_project(px, py, ax, ay, bx, by)
            if t < 0.999:
                break
            self.idx += 1

    def lookahead(self, px, py, ld_mm):
        """Pure-pursuit goal point: first intersection of the ld circle ahead.

        Implemented by walking forward along the polyline accumulating arc
        length from the projection foot. That is cheaper and far more robust
        than solving the circle-segment quadratic, and it degrades sensibly when
        the rover is further than ld from the whole path (it aims at the foot).
        """
        if len(self.pts) < 2:
            return None, None, 0.0
        ax, ay = self.pts[self.idx]
        bx, by = self.pts[self.idx + 1]
        t, cx, cy, perp = seg_project(px, py, ax, ay, bx, by)
        remain = ld_mm
        i = self.idx
        gx, gy = cx, cy
        while i < len(self.pts) - 1:
            sx, sy = (gx, gy)
            ex, ey = self.pts[i + 1]
            seg = math.hypot(ex - sx, ey - sy)
            if seg >= remain:
                if seg <= 1e-9:
                    break
                k = remain / seg
                gx = sx + (ex - sx) * k
                gy = sy + (ey - sy) * k
                remain = 0.0
                break
            remain -= seg
            gx, gy = ex, ey
            i += 1
        return gx, gy, perp

    def dist_to_end_mm(self, px, py):
        ex, ey = self.pts[-1]
        return math.hypot(ex - px, ey - py)


# ======================================================================== BUS
class Bus:
    """Same shape as fpms_missions.py's Bus. Never raises into the caller."""

    def __init__(self, client_suffix="follower"):
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


# ============================================================== FOLLOWER NODE
class FollowerNode(Node):
    """Sensor state plus the fixed-rate control loop.

    Threading: ROS callbacks write state under self.lock; the control thread
    reads it under the same lock and is the ONLY thing that may publish a Twist.
    paho's network thread only mutates path/state, never publishes. One writer,
    one rate, no interleaving - the same discipline fpms_teleop.py uses, and for
    the same reason: 2 Hz zeros interleaved into a 20 Hz setpoint is the
    stall-then-lurch this firmware's encoder quantisation produces.
    """

    def __init__(self, bus):
        super().__init__("fpms_rtos_follower")
        self.bus = bus
        self.lock = threading.RLock()
        self.shutdown = threading.Event()

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        # /odom_raw ONLY. See the module docstring - /odom has an independent
        # accumulated origin and mixing the two produced a stable 4.93 m offset.
        self.create_subscription(Odometry, "/odom_raw", self._on_odom, qos)
        self.create_subscription(Imu, "/imu", self._on_imu, qos)
        self.create_subscription(UInt16, "/battery", self._on_battery, qos)

        # ------------------------------------------------------------- GATE
        self.dry_run = bool(DRY_RUN)
        self.live_confirmed = False    # second act, set by an explicit start
        if self.dry_run:
            log("DRY RUN (default): no /cmd_vel publisher created. Computed "
                "velocities go to MQTT only.")
        else:
            self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", qos)
            log("WARNING: FPMS_FOLLOW_DRY_RUN=0 - a /cmd_vel publisher EXISTS. "
                "Motion still requires an explicit {\"cmd\":\"start\","
                "\"live\":true}.")

        # Pose / heading state.
        self.odom_x_mm = None
        self.odom_y_mm = None
        self.odom_yaw_deg = 0.0
        self.odom_last = 0.0
        self.imu_last = 0.0
        self.batt_dv = None
        self.batt_last = 0.0
        # Integrated gyro heading. B8B trusts the gyro for heading and never the
        # quaternion, because the board publishes identity orientation. It has
        # NO absolute zero, so it is latched against odom yaw at engage time.
        self.gyro_yaw_deg = 0.0
        self.gyro_ref_deg = None     # gyro_yaw - odom_yaw, latched at start
        self._imu_t = 0.0

        # Path + state machine.
        self.path = None
        self.state = S_INIT
        self.state_since = time.monotonic()
        self.state_reason = "starting"
        self.stop_latch = True       # start latched: nothing moves until told

        # Last computed command, published whether or not it was sent.
        self.out_lin = 0.0
        self.out_ang = 0.0
        self.out_trim_c = 0.0
        self.out_wire_lin = 0.0
        self.out_wire_ang = 0.0
        self.out_sent = False
        self.xtrack_mm = 0.0
        self.head_err_deg = 0.0
        self.goal = None

        # Jitter bookkeeping. An unmeasured control period is a lie about gains.
        self.period = 1.0 / HZ
        self.jitter_ms = deque(maxlen=int(max(20, HZ * 10)))
        self.tick_count = 0
        self.overruns = 0
        self.loop_ms = deque(maxlen=int(max(20, HZ * 10)))

        self.create_timer(1.0 / TELEM_HZ, self._telem_tick)
        if RUN_S > 0:
            self.create_timer(0.25, self._run_limit_tick)

    # ------------------------------------------------------------ CALLBACKS
    def _on_odom(self, msg):
        now = time.monotonic()
        with self.lock:
            p = msg.pose.pose.position
            # /odom_raw is metres; everything in this file is millimetres,
            # because every route, marker and arena coordinate in this project
            # is in millimetres and a unit boundary in the middle of the control
            # loop is where sign and scale bugs hide.
            self.odom_x_mm = p.x * 1000.0
            self.odom_y_mm = p.y * 1000.0
            self.odom_yaw_deg = wrap180(
                math.degrees(yaw_from_quat(msg.pose.pose.orientation)))
            self.odom_last = now

    def _on_imu(self, msg):
        now = time.monotonic()
        with self.lock:
            wz = float(msg.angular_velocity.z)
            if self._imu_t > 0:
                dt = now - self._imu_t
                # Exactly B8B's guard: `if 0<dt<0.5`. A gap longer than that is
                # a stall, and integrating across it injects a fictitious turn.
                if 0.0 < dt < 0.5:
                    self.gyro_yaw_deg = wrap180(
                        self.gyro_yaw_deg + math.degrees(wz) * dt)
            self._imu_t = now
            self.imu_last = now

    def _on_battery(self, msg):
        with self.lock:
            self.batt_dv = int(msg.data)
            self.batt_last = time.monotonic()

    # -------------------------------------------------------- STATE MACHINE
    def _set_state(self, state, reason):
        """Caller must hold self.lock."""
        if state == self.state:
            self.state_reason = reason
            return
        prev = self.state
        self.state = state
        self.state_since = time.monotonic()
        self.state_reason = reason
        log(f"STATE {prev} -> {state}: {reason}")
        self.bus.publish("events/follower",
                         {"svc": "follower", "event": "state",
                          "from": prev, "to": state, "reason": reason,
                          "dry_run": self.dry_run}, qos=1)

    def _pose_fresh(self, now):
        return (self.odom_x_mm is not None
                and (now - self.odom_last) * 1000.0 <= DEADMAN_MS)

    def _path_fresh(self, now):
        if self.path is None:
            return False
        if PATH_DEADMAN_MS <= 0:
            return True
        return (self.path.age_s(now) * 1000.0) <= PATH_DEADMAN_MS

    def _link_dead(self, now):
        """True when the board has been silent long enough to call it dead.

        Deliberately AGE-based, never rate-based. A cached Hz stays non-zero
        after a link dies; that exact confusion cost a session on 2026-08-04.
        """
        if self.odom_last == 0.0 and self.imu_last == 0.0:
            return True
        newest = max(self.odom_last, self.imu_last)
        return (now - newest) > ROS_DEAD_S

    # ----------------------------------------------------------- CONTROL LAW
    def _compute(self, now):
        """One tick of the B8B trim law aimed at a pure-pursuit goal.

        Returns (lin_mps, ang_rps, trim_counts). Caller holds the lock.
        """
        px, py = self.odom_x_mm, self.odom_y_mm
        self.path.advance(px, py)
        gx, gy, perp = self.path.lookahead(px, py, LOOKAHEAD_MM)
        self.goal = (jnum(gx, 1), jnum(gy, 1))
        self.xtrack_mm = perp

        # Heading: gyro, expressed in the odom frame via the latch. B8B trusts
        # the gyro over anything the board calls "orientation"; the latch is
        # what gives that relative signal an absolute zero without pretending
        # the board fused anything.
        if self.gyro_ref_deg is None:
            self.gyro_ref_deg = self.gyro_yaw_deg - self.odom_yaw_deg
        heading_deg = wrap180(self.gyro_yaw_deg - self.gyro_ref_deg)

        bearing_deg = math.degrees(math.atan2(gy - py, gx - px))
        err_deg = wrap180(bearing_deg - heading_deg)
        self.head_err_deg = err_deg
        err_rad = math.radians(err_deg)

        # ---- B8B, unchanged: duty counts, gain 10 per radian, clamp +/-6 ----
        c = clamp(err_rad * KPH, -TRIM_CLAMP, TRIM_CLAMP)

        # ---- duty -> velocity. GEOMETRY, NOT MEASUREMENT. See docstring. ----
        f = c / DRV_DUTY
        lin = CRUISE_MPS
        # Withhold forward speed while badly mis-aimed, the continuous analogue
        # of B8B's "spin first if |err| > 15 deg". Cosine taper rather than a
        # branch, so there is no separate spin state that can get stuck.
        if abs(err_deg) > FACE_FIRST_DEG:
            lin = 0.0
        else:
            lin *= max(0.0, math.cos(err_rad))
        omega = 2.0 * max(lin, CRUISE_MPS * 0.35) * f / WHEELBASE_M

        # Ease off near the end so the last centimetres are slow, which is what
        # the operator asked for and what B8B got structurally from its bounded
        # bursts and mandatory stops.
        d_end = self.path.dist_to_end_mm(px, py)
        if d_end < LOOKAHEAD_MM:
            lin *= max(0.25, d_end / LOOKAHEAD_MM)

        lin = clamp(lin, -MAX_LIN_MPS, MAX_LIN_MPS)
        omega = clamp(omega, -MAX_ANG_RPS, MAX_ANG_RPS)
        return lin, omega, c

    # ------------------------------------------------------------ THE LOOP
    def control_loop(self):
        """Fixed-rate loop scheduled against an absolute monotonic deadline.

        B8B ends its loop with `time.sleep(1.0/_HZ)`, which makes the true
        period 50 ms PLUS however long the body took - so the real rate sagged
        below 20 Hz whenever the LiDAR copy was slow, and the gains were tuned
        against a rate nobody measured. Here the deadline advances by exactly
        one period regardless of body duration, and the residual error is
        recorded as jitter. If the body ever exceeds a period the deadline is
        re-based rather than allowed to spiral into a catch-up burst - a burst
        of back-to-back ticks would multiply the trim's effective gain.
        """
        next_t = time.monotonic()
        while not self.shutdown.is_set():
            next_t += self.period
            now = time.monotonic()
            sleep_s = next_t - now
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                # Overrun: we are already late. Re-base so we do not fire a
                # catch-up burst.
                self.overruns += 1
                next_t = time.monotonic()

            t_tick = time.monotonic()
            self.jitter_ms.append((t_tick - next_t) * 1000.0)
            self.tick_count += 1
            self._tick(t_tick)
            self.loop_ms.append((time.monotonic() - t_tick) * 1000.0)

    def _tick(self, now):
        lin = 0.0
        ang = 0.0
        c = 0.0
        with self.lock:
            pose_ok = self._pose_fresh(now)
            path_ok = self._path_fresh(now)
            dead = self._link_dead(now)

            if self.stop_latch:
                # Two distinct latched conditions, and they are worth telling
                # apart on the dashboard: nothing to do, versus something to do
                # that is deliberately not being done. Collapsing them made the
                # state flap IDLE<->STOPPED on consecutive ticks.
                if self.path is None:
                    self._set_state(S_IDLE, "no path; stop latch held")
                else:
                    self._set_state(S_STOPPED, "path loaded; awaiting start")
            elif dead:
                # The board is not talking. This is the state the rover is in
                # right now with the ESP32 link down, and the correct output is
                # zero, not a best guess from the last known pose.
                self._set_state(S_FAULT, "micro-ROS link dead (no /odom_raw or "
                                         f"/imu for >{ROS_DEAD_S:.1f}s)")
            elif self.path is None:
                self._set_state(S_IDLE, "no path")
            elif not pose_ok:
                self._set_state(S_DEADMAN,
                                f"pose older than {DEADMAN_MS:.0f}ms")
            elif not path_ok:
                self._set_state(S_DEADMAN,
                                f"path not re-asserted within "
                                f"{PATH_DEADMAN_MS:.0f}ms")
            elif self.path.dist_to_end_mm(self.odom_x_mm,
                                          self.odom_y_mm) <= ARRIVE_MM:
                self._set_state(S_ARRIVED, "within ARRIVE_MM of path end")
            else:
                try:
                    lin, ang, c = self._compute(now)
                except Exception as e:
                    lin = ang = c = 0.0
                    self._set_state(S_FAULT, f"control law raised: {e}")
                else:
                    if abs(self.xtrack_mm) > MAX_XTRACK_MM:
                        lin = ang = c = 0.0
                        self._set_state(S_FAULT,
                                        f"cross-track {self.xtrack_mm:.0f}mm "
                                        f"> {MAX_XTRACK_MM:.0f}mm - refusing "
                                        "to chase a path the rover is not on")
                    else:
                        self._set_state(S_FOLLOW, "tracking")

            # ONE place decides whether the output may be non-zero. Anything
            # not FOLLOW is a zero, unconditionally, no exceptions bolted on
            # later.
            if self.state != S_FOLLOW:
                lin = ang = c = 0.0

            self.out_lin = lin
            self.out_ang = ang
            self.out_trim_c = c
            self.out_wire_lin = lin / CMD_SCALE
            self.out_wire_ang = ang / CMD_SCALE
            may_send = (not self.dry_run) and self.live_confirmed
            self.out_sent = bool(may_send)

        if may_send:
            t = Twist()
            t.linear.x = self.out_wire_lin
            t.angular.z = self.out_wire_ang
            try:
                self.pub_cmd.publish(t)
            except Exception as e:
                log(f"cmd_vel publish failed: {e}")

    # ------------------------------------------------------------ TELEMETRY
    def _jitter_stats(self):
        j = [abs(v) for v in self.jitter_ms]
        if not j:
            return None
        return {
            "period_ms": jnum(self.period * 1000.0, 2),
            "abs_mean_ms": jnum(sum(j) / len(j), 3),
            "abs_max_ms": jnum(max(j), 3),
            "p95_ms": jnum(percentile(j, 0.95), 3),
            "ticks": self.tick_count,
            "overruns": self.overruns,
            "body_mean_ms": jnum(sum(self.loop_ms) / len(self.loop_ms), 3)
                            if self.loop_ms else None,
            "body_max_ms": jnum(max(self.loop_ms), 3) if self.loop_ms else None,
        }

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            odom_age = None if self.odom_last == 0 else (now - self.odom_last)
            imu_age = None if self.imu_last == 0 else (now - self.imu_last)
            batt_age = None if self.batt_last == 0 else (now - self.batt_last)
            snap = {
                "svc": "follower",
                "uptime_s": jnum(time.time() - STARTED, 1),
                "state": self.state,
                "state_reason": self.state_reason,
                "state_for_s": jnum(now - self.state_since, 2),
                "dry_run": self.dry_run,
                "live_confirmed": self.live_confirmed,
                "publisher_exists": (not self.dry_run),
                "stop_latch": self.stop_latch,
                # What it WOULD send, always, in dry run and live alike. In dry
                # run `sent` is false and these are the whole product.
                "cmd": {
                    "lin_mps": jnum(self.out_lin, 4),
                    "ang_rps": jnum(self.out_ang, 4),
                    "wire_lin": jnum(self.out_wire_lin, 5),
                    "wire_ang": jnum(self.out_wire_ang, 5),
                    "trim_counts": jnum(self.out_trim_c, 3),
                    "sent": self.out_sent,
                },
                "err": {
                    "heading_deg": jnum(self.head_err_deg, 2),
                    "xtrack_mm": jnum(self.xtrack_mm, 1),
                    "goal_mm": self.goal,
                },
                "pose": {
                    "x_mm": jnum(self.odom_x_mm, 1),
                    "y_mm": jnum(self.odom_y_mm, 1),
                    "odom_yaw_deg": jnum(self.odom_yaw_deg, 2),
                    "gyro_yaw_deg": jnum(self.gyro_yaw_deg, 2),
                    "gyro_ref_deg": jnum(self.gyro_ref_deg, 2),
                    "source": "/odom_raw only (see module docstring)",
                },
                "freshness": {
                    "odom_age_s": jnum(odom_age, 3),
                    "imu_age_s": jnum(imu_age, 3),
                    "battery_age_s": jnum(batt_age, 3),
                    "pose_fresh": self._pose_fresh(now),
                    "path_fresh": self._path_fresh(now),
                    "link_dead": self._link_dead(now),
                    "deadman_ms": DEADMAN_MS,
                    "path_deadman_ms": PATH_DEADMAN_MS,
                    "ros_dead_s": ROS_DEAD_S,
                },
                "battery": (None if self.batt_dv is None else
                            {"decivolts": self.batt_dv,
                             "volts": jnum(self.batt_dv / 10.0, 2)}),
                "path": (None if self.path is None else {
                    "points": len(self.path.pts),
                    "seg_idx": self.path.idx,
                    "length_mm": jnum(self.path.length_mm(), 1),
                    "age_s": jnum(self.path.age_s(now), 3),
                    "frame": self.path.frame,
                    "seq": self.path.seq,
                }),
                "loop": self._jitter_stats(),
                # The assumptions, published, so nobody has to read the source
                # to know what the numbers mean.
                "trim_model": {
                    "source": "B8B/phase6 _p5_navdrive",
                    "drv_duty": DRV_DUTY,
                    "kph_counts_per_rad": KPH,
                    "clamp_counts": TRIM_CLAMP,
                    "wheelbase_m": WHEELBASE_M,
                    "cruise_mps": CRUISE_MPS,
                    "cmd_scale": CMD_SCALE,
                    "lookahead_mm": LOOKAHEAD_MM,
                    "hz": HZ,
                    "caveat": "duty->omega conversion is geometry, not a "
                              "measured motor curve; UNVERIFIED on hardware",
                },
                "domain_id": os.environ.get("ROS_DOMAIN_ID"),
            }
        return snap

    def _telem_tick(self):
        self.bus.publish("telemetry/follower", self.snapshot())

    def _run_limit_tick(self):
        if (time.time() - STARTED) >= RUN_S:
            log(f"FPMS_FOLLOW_RUN_S={RUN_S:.0f}s elapsed; exiting cleanly")
            # Set the event and NOTHING else. Calling rclpy.shutdown() from
            # inside a callback the executor is currently running hangs the
            # process - measured on this Pi 2026-08-04. main() spins in slices
            # and watches this event instead. The control thread watches the
            # same event, so both loops end from one flag.
            self.shutdown.set()

    # -------------------------------------------------------------- CONTROL
    def on_control(self, topic, payload):
        """`fpms/<thing>/control/follower`. Not under commands/ - see docstring."""
        cmd = str(payload.get("cmd") or payload.get("action") or "status").lower()

        if cmd in ("status", "query"):
            self.bus.publish("telemetry/follower", self.snapshot())
            return

        if cmd in ("stop", "estop", "clear"):
            with self.lock:
                self.stop_latch = True
                self.live_confirmed = False
                if cmd == "clear":
                    self.path = None
                self._set_state(S_STOPPED, f"{cmd} received")
                self.out_lin = self.out_ang = 0.0
            # In live mode leave a stop on the wire immediately rather than
            # waiting for the next tick. A stop that waits for a tick is not a
            # stop.
            if not self.dry_run:
                try:
                    z = Twist()
                    for _ in range(3):
                        self.pub_cmd.publish(z)
                except Exception:
                    pass
            self.bus.publish("events/follower", {"svc": "follower",
                                                 "ack": cmd}, qos=1)
            return

        if cmd in ("path", "keepalive"):
            pts = payload.get("points_mm") or payload.get("path")
            if cmd == "keepalive" and not pts:
                with self.lock:
                    if self.path is not None:
                        self.path.touch()
                return
            ok, err, parsed = self._parse_path(pts)
            if not ok:
                # Nack and stop. Deliberately NOT a state change: rejecting an
                # input is not a fault of the follower, and any previously
                # accepted path is still valid. Faulting here also fought the
                # control tick for ownership of the state machine.
                log(f"path REJECTED: {err}")
                self.bus.publish("events/follower",
                                 {"svc": "follower", "nack": "path",
                                  "error": err}, qos=1)
                return
            with self.lock:
                seq = 0 if self.path is None else self.path.seq + 1
                self.path = Path(parsed, payload.get("frame", "odom_raw"))
                self.path.seq = seq
                # A new path re-latches the gyro reference: the heading zero
                # must belong to the path that is about to be followed, not to
                # one abandoned ten minutes ago.
                self.gyro_ref_deg = None
                # No _set_state here. The 20 Hz tick is the SINGLE authority on
                # state; it will pick this up within one period. Two writers to
                # the state machine produced a visible IDLE<->STOPPED flap and,
                # worse, a window where the published state disagreed with the
                # state the control law had actually used.
            log(f"path accepted: {len(parsed)} points, "
                f"{self.path.length_mm():.0f}mm, frame="
                f"{payload.get('frame', 'odom_raw')}")
            self.bus.publish("events/follower",
                             {"svc": "follower", "ack": "path",
                              "points": len(parsed)}, qos=1)
            return

        if cmd == "start":
            want_live = bool(payload.get("live", False))
            with self.lock:
                if want_live and self.dry_run:
                    # Refused, and the refusal names the only thing that can
                    # change it. Dry run is an environment decision.
                    self.bus.publish("events/follower",
                                     {"svc": "follower", "nack": "start",
                                      "error": "node is in DRY RUN "
                                               "(FPMS_FOLLOW_DRY_RUN=1); "
                                               "live motion cannot be enabled "
                                               "over MQTT. Set the env flag in "
                                               "the unit and restart."}, qos=1)
                    log("start(live=true) REFUSED - node is in dry run")
                    return
                if self.path is None:
                    self.bus.publish("events/follower",
                                     {"svc": "follower", "nack": "start",
                                      "error": "no path loaded"}, qos=1)
                    return
                self.live_confirmed = bool(want_live and not self.dry_run)
                self.stop_latch = False
                self.path.touch()
                self._set_state(S_FOLLOW,
                                "started (LIVE)" if self.live_confirmed
                                else "started (dry run - computing only)")
            self.bus.publish("events/follower",
                             {"svc": "follower", "ack": "start",
                              "live": self.live_confirmed,
                              "dry_run": self.dry_run}, qos=1)
            return

        self.bus.publish("events/follower", {"svc": "follower", "nack": cmd,
                                             "error": "unknown follower "
                                                      "command"}, qos=1)

    @staticmethod
    def _parse_path(pts):
        """Validate hard. A path is the one input that can drive the rover."""
        if not isinstance(pts, (list, tuple)) or len(pts) < 2:
            return False, "path needs at least 2 points", None
        if len(pts) > 500:
            return False, "path too long (>500 points)", None
        out = []
        for p in pts:
            if isinstance(p, dict):
                x, y = p.get("x"), p.get("y")
            elif isinstance(p, (list, tuple)) and len(p) >= 2:
                x, y = p[0], p[1]
            else:
                return False, f"bad point {p!r}", None
            try:
                x = float(x)
                y = float(y)
            except Exception:
                return False, f"non-numeric point {p!r}", None
            if not (math.isfinite(x) and math.isfinite(y)):
                return False, "non-finite point", None
            # 20 m box. Anything outside is a unit error (metres passed as
            # millimetres is the classic one) and must be refused, not driven.
            if abs(x) > 20000.0 or abs(y) > 20000.0:
                return False, (f"point ({x:.0f},{y:.0f}) outside +/-20000mm - "
                               "check units, this field is MILLIMETRES"), None
            out.append((x, y))
        return True, "", out


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
        log(f"WARNING: ROS_DOMAIN_ID={dom!r}, expected '20' - this rover's "
            "board publishes on domain 20 only; on any other domain this node "
            "sees no pose and will sit in FAULT regardless of the hardware")

    bus = Bus("follower")
    bus.sub_topics = [f"fpms/{THING}/control/follower"]
    bus.online_payload = {
        "svc": "follower", "status": "online",
        "control_topic": f"fpms/{THING}/control/follower",
        "telemetry": [f"fpms/{THING}/telemetry/follower"],
        "commands": ["status", "path", "keepalive", "start", "stop", "clear"],
        "dry_run": bool(DRY_RUN),
        "hz": HZ,
        "note": "Pi-side follower. The ESP32 runs Yahboom factory firmware; "
                "there is no RTOS task on the board.",
    }
    bus.start()

    rclpy.init()
    node = FollowerNode(bus)
    bus.on_control = lambda _t, p: node.on_control(_t, p)

    loop_thread = threading.Thread(target=node.control_loop, daemon=True,
                                   name="fpms-follow-loop")
    loop_thread.start()

    def _sig(_s, _f):
        # Latch stop FIRST, then set the event. Order matters: the control
        # thread may be mid-tick, and it must find the latch already set rather
        # than compute one more non-zero command on the way out.
        # rclpy is deliberately NOT touched here - see _run_limit_tick.
        log("signal: stopping follower")
        with node.lock:
            node.stop_latch = True
            node.live_confirmed = False
        node.shutdown.set()

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    log(f"fpms_rtos_follower up: thing={THING} hz={HZ:.0f} "
        f"dry_run={bool(DRY_RUN)} cruise={CRUISE_MPS:.3f}m/s "
        f"(wire {CRUISE_MPS / CMD_SCALE:.4f}) kph={KPH:.0f} "
        f"clamp=+/-{TRIM_CLAMP:.0f} lookahead={LOOKAHEAD_MM:.0f}mm "
        f"deadman={DEADMAN_MS:.0f}ms run_s={RUN_S:.0f}")
    # NOT rclpy.spin(). Two things need to end this loop - the run-limit timer
    # and SIGTERM - and both run on the executor's own thread, where calling
    # rclpy.shutdown() hangs the process instead of ending it. Spinning in
    # 0.1 s slices makes a plain threading.Event the single stop signal, and
    # bounds how long systemd waits for us at TimeoutStopSec.
    try:
        while rclpy.ok() and not node.shutdown.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        log("interrupted")
    except Exception as e:
        log(f"spin ended: {e}")
    finally:
        node.shutdown.set()
        try:
            loop_thread.join(timeout=2.0)
        except Exception:
            pass
        # Ten zeros on the way out IF AND ONLY IF this process ever owned the
        # wire. A dry-run process publishes nothing on exit - on this board an
        # unexpected Twist at teardown is a real hazard, and a node that never
        # wrote must not start writing during shutdown.
        try:
            if not node.dry_run:
                z = Twist()
                for _ in range(10):
                    node.pub_cmd.publish(z)
                    time.sleep(0.02)
        except Exception:
            pass
        bus.publish("events/follower", {"svc": "follower", "status": "offline",
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
        log("fpms_rtos_follower exited cleanly")


if __name__ == "__main__":
    main()
