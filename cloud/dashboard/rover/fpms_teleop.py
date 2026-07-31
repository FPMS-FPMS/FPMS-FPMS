#!/usr/bin/env python3
"""FPMS teleop bridge — MQTT commands in, /cmd_vel out, drive telemetry back.

Runs alongside (never instead of) micro-ros-agent and fpms-rover-agent. It owns
exactly one resource: the /cmd_vel topic. It does not touch a serial port, it
does not restart anything, and it holds no reference to the other services.

SAFETY MODEL — the reason this file is shaped the way it is:

  * ONE writer to /cmd_vel. Commands arriving on paho's network thread never
    publish; they only set state under a lock. A single 20 Hz control tick on
    the ROS thread reads that state, ramps, clamps and publishes. There is no
    interleaving of two code paths both driving the motors.
  * The clamp is the LAST thing that happens before a Twist is built, and it is
    expressed in real-world m/s. No command path, present or future, can get a
    number past it.
  * Every way of stopping converges on the same place: mode goes idle and the
    ramp pulls to zero. `stop` additionally slams a hard zero out immediately,
    bypassing the ramp, because a stop that waits for a ramp is not a stop.
  * The deadman is not a feature of jog, it is a property of the control tick:
    if a jog is stale the tick refuses to honour it, whether or not any message
    ever arrives again.
  * Motion is GATED, not merely limited: a dead micro-ROS link or a flat pack
    blocks new motion and halts running motion. `stop` is never gated.
"""

import json
import math
import os
import signal
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16

import paho.mqtt.client as mqtt
from paho.mqtt.client import CallbackAPIVersion


# ===================================================================== CONFIG
def load_config(path="/etc/fpms/config.env"):
    cfg = {}
    try:
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    cfg[k] = v
    except Exception:
        pass
    return cfg


CFG = load_config()
THING = CFG.get("FPMS_THING_NAME", "rover2")
BROKER = CFG.get("FPMS_MQTT_HOST", "192.168.137.1")
PORT = int(CFG.get("FPMS_MQTT_PORT", "1883"))
USER = CFG.get("FPMS_MQTT_USER", "")
PASS = CFG.get("FPMS_MQTT_PASS", "")

ORIGIN_FILE = "/home/ubuntu/.fpms_teleop_origin.json"
STARTED = time.time()


# =============================================================== CALIBRATION
# The single place where "what I ask for" is turned into "what the board does".
#
# MEASURED: commanding linear.x = 0.10 produced roughly 0.61 m/s of actual
# ground speed — about 6x. The operator judged that far too fast. Everything
# below is therefore expressed in REAL m/s and divided by CMD_SCALE on the way
# out. If the rover is ever re-measured, this constant is the only edit.
CMD_SCALE = 6.1


def to_cmd(desired_mps):
    """Real-world m/s -> the number the firmware wants."""
    return desired_mps / CMD_SCALE


# The same 6x fudge is applied to rotation. The 6x was measured on the linear
# axis only, so this is an assumption — but it is an assumption that can only
# make the rover turn SLOWER than asked, which is the correct direction to be
# wrong in with a person standing next to it. The `turn` command is closed-loop
# on the gyro, so it self-corrects for this being wrong either way.
def to_cmd_ang(desired_radps):
    return desired_radps / CMD_SCALE


# Target ACTUAL speeds, in real m/s and rad/s.
JOG_MAX_MPS = 0.05        # full joystick deflection
NUDGE_MPS = 0.04          # F/B 10cm buttons
DOCK_MPS = 0.025          # slow docking near a waypoint
TURN_MAX_RADPS = 0.4      # slow turning
DOCK_TURN_RADPS = 0.2     # closed-loop `turn` command

# Ramps, in real units per second. Deceleration is deliberately far more
# aggressive than acceleration: taking off gently is comfort, stopping promptly
# is safety.
ACCEL_MPS2 = 0.08
DECEL_MPS2 = 0.30
ANG_ACCEL = 0.60
ANG_DECEL = 2.00

CONTROL_HZ = 20.0
CONTROL_DT = 1.0 / CONTROL_HZ

JOG_DEADMAN_S = 0.6       # no jog for this long -> zero Twist
TELEM_HZ = 2.0
MAX_NUDGE_MM = 1000       # refuse anything longer than a metre

# --- turn tuning ---
# Stop commanding rotation at this fraction of the target and let the rover
# coast the rest. Carried over from the previous working rover, which held
# +/-1-4 degrees with exactly this trick.
TURN_COAST_FACTOR = 0.93
TURN_SETTLE_S = 1.2       # how long to let it coast before measuring
TURN_TIMEOUT_S = 15.0
MAX_TURN_DEG = 360.0

# --- stall / wrong-way watchdogs -------------------------------------------
# MEASURED 2026-07-31: a nudge commanded at DOCK_MPS (0.0041 on the wire after
# CMD_SCALE) produced NO motion for ~3s and then a lurch of ~290mm BACKWARD
# plus 24 degrees of unintended rotation. The board firmware appears to run a
# closed-loop wheel controller whose integrator winds up while the commanded
# velocity sits under the motor deadband, then releases all at once in a
# direction that is not reliably the commanded one.
#
# These two watchdogs exist so that condition can never build up again: if the
# rover is being commanded to move and the odometry says it is not moving, we
# stop commanding LONG before the integrator has anything to release.
STALL_CHECK_S = 2.0       # commanded this long with no progress => abort
STALL_MIN_MM = 5.0        # ...where "progress" is at least this much
WRONG_WAY_MM = 50.0       # moving this far opposite the command => abort
TURN_STALL_CHECK_S = 2.5
TURN_STALL_MIN_DEG = 2.0

# --- health thresholds ---
# 3S LiPo assumption: 3 cells x 3.7V nominal = 11.1V. Below that the pack is
# into the knee of its discharge curve and motor current will sag it further,
# so new motion is refused there rather than at a true-empty 9.9V.
BATT_LOW_V = 11.1
ROS_DEAD_S = 3.0          # no /odom_raw for this long => link considered dead
BATT_STALE_S = 10.0


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def ramp(cur, target, accel_rate, decel_rate, dt):
    """Move `cur` toward `target`, using the decel rate whenever the step
    shrinks the magnitude or crosses zero."""
    if target == cur:
        return target
    shrinking = abs(target) < abs(cur) or (cur * target) < 0
    rate = decel_rate if shrinking else accel_rate
    step = rate * dt
    if target > cur:
        return min(cur + step, target)
    return max(cur - step, target)


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def jnum(v, nd=None):
    """JSON-safe number. NaN and Infinity are not valid JSON and would break
    the dashboard's parser, so they become null."""
    try:
        if v is None:
            return None
        f = float(v)
        if not math.isfinite(f):
            return None
        return round(f, nd) if nd is not None else f
    except Exception:
        return None


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def hz_from(times):
    if len(times) < 2:
        return 0.0
    span = times[-1] - times[0]
    if span <= 0:
        return 0.0
    return (len(times) - 1) / span


# ======================================================================= BUS
class Bus:
    """MQTT wrapper that never raises into the caller and reconnects forever.

    connect_async + loop_start is used rather than connect(): a broker that is
    down at boot must delay the rover's telemetry, not prevent the node from
    starting. Until it comes up, publishes are dropped and the ROS side — which
    is the side that actually stops the motors — runs completely unaffected.
    """

    def __init__(self):
        self.client = mqtt.Client(client_id=f"{THING}-teleop",
                                  callback_api_version=CallbackAPIVersion.VERSION2)
        if USER:
            self.client.username_pw_set(USER, PASS or None)
        self.client.reconnect_delay_set(min_delay=1, max_delay=15)
        self.client.will_set(f"fpms/{THING}/events/offline",
                             json.dumps({"thing": THING, "status": "offline",
                                         "svc": "teleop",
                                         "reason": "unexpected disconnect"}),
                             qos=1, retain=False)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.connected = False
        self.on_command = None

    # -- lifecycle ---------------------------------------------------------
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

    # -- callbacks ---------------------------------------------------------
    def _on_connect(self, client, _u, _f, reason_code, _p=None):
        try:
            if getattr(reason_code, "is_failure", False):
                log(f"MQTT: connect refused ({reason_code}); will retry")
                return
            self.connected = True
            log(f"MQTT: connected to {BROKER}:{PORT} ({reason_code})")
            # Subscribed by name, not with a '#' wildcard. fpms-rover-agent is
            # already on fpms/<thing>/commands/# and answers anything it does
            # not know with a nack; taking only the verbs this node implements
            # keeps the two agents from both claiming the same message.
            for action in ("jog", "stop", "estop", "auto_off", "nudge", "turn",
                           "mission", "set_coordinate"):
                client.subscribe(f"fpms/{THING}/commands/{action}", qos=1)
            self.publish("events/online",
                         {"svc": "teleop", "status": "online",
                          "cmd_scale": CMD_SCALE,
                          "limits": {"jog_max_mps": JOG_MAX_MPS,
                                     "turn_max_radps": TURN_MAX_RADPS,
                                     "dock_mps": DOCK_MPS,
                                     "dock_turn_radps": DOCK_TURN_RADPS,
                                     "batt_low_v": BATT_LOW_V}}, qos=1)
        except Exception as e:
            log(f"MQTT: on_connect error {e}")

    def _on_disconnect(self, _c, _u, *args):
        self.connected = False
        log("MQTT: disconnected; paho will retry")

    def _on_message(self, _c, _u, msg):
        # Runs on paho's network thread. It must return fast and must never
        # raise: an exception here would kill the MQTT loop and with it the
        # only channel a `stop` can arrive on.
        try:
            action = msg.topic.rsplit("/", 1)[-1]
            try:
                payload = json.loads(msg.payload.decode() or "{}")
            except Exception:
                payload = {}
            if not isinstance(payload, dict):
                payload = {}
            if self.on_command:
                self.on_command(action, payload)
        except Exception as e:
            log(f"MQTT: message handler error {e}")

    # -- publish -----------------------------------------------------------
    def publish(self, suffix, payload, qos=0):
        try:
            payload.setdefault("ts", time.time())
            payload.setdefault("thing", THING)
        except Exception:
            pass
        if not self.connected:
            return
        try:
            # allow_nan=False turns an accidental NaN into an exception here
            # rather than into invalid JSON on the dashboard's parser.
            body = json.dumps(payload, allow_nan=False)
        except Exception as e:
            log(f"MQTT: payload not JSON-safe on {suffix}: {e}")
            return
        try:
            self.client.publish(f"fpms/{THING}/{suffix}", body, qos=qos)
        except Exception as e:
            log(f"MQTT: publish failed on {suffix}: {e}")


# ====================================================================== NODE
class TeleopNode(Node):

    def __init__(self, bus):
        super().__init__("fpms_teleop")
        self.bus = bus
        self.lock = threading.RLock()

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", qos)
        self.create_subscription(Odometry, "/odom_raw", self._on_odom, qos)
        self.create_subscription(Imu, "/imu", self._on_imu, qos)
        self.create_subscription(UInt16, "/battery", self._on_battery, qos)

        # --- motion state (all guarded by self.lock) ---
        self.mode = "idle"           # idle | jog | nudge | turn
        self.cur_vx = 0.0            # real m/s, post-ramp
        self.cur_wz = 0.0            # real rad/s, post-ramp
        self.jog_vx = 0.0            # normalized -1..1
        self.jog_wz = 0.0
        self.jog_ts = 0.0            # monotonic, last jog message
        self.jog_warned = False
        self.nudge = None
        self.turn = None

        # --- odom ---
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = 0.0
        self.odom_last = 0.0
        self.odom_times = deque(maxlen=40)

        # --- imu / integrated heading ---
        # The firmware does NOT fuse orientation (it publishes identity), so
        # heading for the turn controller is integrated from the z gyro here.
        self.imu_last = 0.0
        self.imu_times = deque(maxlen=60)
        self.gyro_z = 0.0            # bias-corrected, rad/s
        self.gyro_bias = 0.0         # estimated while stationary
        self.gyro_bias_n = 0
        self.yaw_int = 0.0           # integrated yaw, rad, free-running
        self._imu_prev_t = None

        # --- battery ---
        self.battery_raw = None
        self.battery_last = 0.0
        self.battery_times = deque(maxlen=20)

        # --- bookkeeping ---
        self.origin = None           # (ref_x_m, ref_y_m, set_x_mm, set_y_mm)
        self._load_origin()
        self.last_cmd = None
        self.last_cmd_at = None
        self.last_error = None
        self._last_zero_pub = 0.0
        self._link_warned = False
        self._jog_nack_at = 0.0

        self.create_timer(CONTROL_DT, self._control_tick)
        self.create_timer(1.0 / TELEM_HZ, self._telemetry_tick)
        log("teleop node up: /cmd_vel publisher; /odom_raw + /imu + /battery subs")

    # ------------------------------------------------------------ ROS input
    def _on_odom(self, msg):
        try:
            with self.lock:
                self.odom_x = msg.pose.pose.position.x
                self.odom_y = msg.pose.pose.position.y
                self.odom_yaw = yaw_from_quat(msg.pose.pose.orientation)
                now = time.monotonic()
                self.odom_last = now
                self.odom_times.append(now)
        except Exception as e:
            log(f"odom callback error {e}")

    def _on_imu(self, msg):
        try:
            now = time.monotonic()
            with self.lock:
                self.imu_last = now
                self.imu_times.append(now)
                raw = float(msg.angular_velocity.z)
                if not math.isfinite(raw):
                    return

                # Estimate zero-rate bias only while genuinely parked, so a
                # slow real rotation is never mistaken for drift.
                if self.mode == "idle" and abs(self.cur_wz) < 1e-6:
                    if self.gyro_bias_n < 2000:
                        self.gyro_bias_n += 1
                    a = 1.0 / min(self.gyro_bias_n, 200)
                    self.gyro_bias = (1 - a) * self.gyro_bias + a * raw

                self.gyro_z = raw - self.gyro_bias

                dt = 0.0 if self._imu_prev_t is None else (now - self._imu_prev_t)
                self._imu_prev_t = now
                # Ignore absurd gaps (a stalled link resuming) so one hiccup
                # cannot inject a huge bogus angle into a running turn.
                if 0.0 < dt < 0.5:
                    self.yaw_int += self.gyro_z * dt
        except Exception as e:
            log(f"imu callback error {e}")

    def _on_battery(self, msg):
        try:
            with self.lock:
                self.battery_raw = int(msg.data)
                now = time.monotonic()
                self.battery_last = now
                self.battery_times.append(now)
        except Exception as e:
            log(f"battery callback error {e}")

    # --------------------------------------------------------- health gates
    def _battery_v_unlocked(self):
        if self.battery_raw is None:
            return None
        return self.battery_raw / 10.0

    def _link_ok_unlocked(self, now=None):
        now = now or time.monotonic()
        return self.odom_x is not None and (now - self.odom_last) < ROS_DEAD_S

    def _motion_block_unlocked(self):
        """Reason new motion must not start, or None. Caller holds lock."""
        if not self._link_ok_unlocked():
            return ("micro-ROS link down: no /odom_raw for >%.0fs "
                    "(not restarting the agent)" % ROS_DEAD_S)
        v = self._battery_v_unlocked()
        if v is not None and v < BATT_LOW_V:
            return f"battery low: {v:.1f}V < {BATT_LOW_V}V threshold"
        return None

    # -------------------------------------------------------- control tick
    def _control_tick(self):
        """The only place a Twist is ever published."""
        try:
            with self.lock:
                now = time.monotonic()

                # Link watchdog. A dead micro-ROS session means we have no idea
                # what the rover is doing, so nothing may keep driving.
                if not self._link_ok_unlocked(now):
                    if self.mode != "idle":
                        log("micro-ROS link DOWN — halting motion "
                            f"(mode was {self.mode})")
                        self.last_error = "micro-ROS link down; motion halted"
                        self._event("events/nack",
                                    {"action": self.mode,
                                     "error": "micro-ROS link down; motion halted"})
                        self.mode = "idle"
                        self.nudge = None
                        self.turn = None
                        self.cur_vx = self.cur_wz = 0.0
                    if not self._link_warned:
                        self._link_warned = True
                        log("micro-ROS link watchdog tripped (no /odom_raw)")
                elif self._link_warned:
                    self._link_warned = False
                    log("micro-ROS link restored")

                des_vx, des_wz = self._desired_unlocked(now)

                self.cur_vx = ramp(self.cur_vx, des_vx, ACCEL_MPS2, DECEL_MPS2, CONTROL_DT)
                self.cur_wz = ramp(self.cur_wz, des_wz, ANG_ACCEL, ANG_DECEL, CONTROL_DT)

                # HARD CLAMP, in real units, immediately before publish. This is
                # the backstop for every code path above it.
                self.cur_vx = clamp(self.cur_vx, -JOG_MAX_MPS, JOG_MAX_MPS)
                self.cur_wz = clamp(self.cur_wz, -TURN_MAX_RADPS, TURN_MAX_RADPS)

                vx, wz, mode = self.cur_vx, self.cur_wz, self.mode

            idle_zero = (mode == "idle" and abs(vx) < 1e-6 and abs(wz) < 1e-6)
            if idle_zero:
                # Parked. Keep a slow zero heartbeat instead of 20 Hz of nothing.
                if now - self._last_zero_pub < 0.5:
                    return
                self._last_zero_pub = now
            self._publish_twist(vx, wz)
        except Exception as e:
            log(f"control tick error {e}")
            self._safe_zero()

    def _desired_unlocked(self, now):
        """Target speed in REAL units for the current mode. Caller holds lock."""
        if self.mode == "jog":
            if now - self.jog_ts > JOG_DEADMAN_S:
                # Deadman. The controller went quiet; assume the worst.
                if not self.jog_warned:
                    self.jog_warned = True
                    log(f"jog DEADMAN tripped — no command for {JOG_DEADMAN_S}s; stopping")
                    self._event("events/ack", {"action": "jog", "deadman": True,
                                               "stopped": True})
                self.mode = "idle"
                self.jog_vx = self.jog_wz = 0.0
                return 0.0, 0.0
            return (self.jog_vx * JOG_MAX_MPS, self.jog_wz * TURN_MAX_RADPS)

        if self.mode == "nudge":
            return self._nudge_step_unlocked(now)

        if self.mode == "turn":
            return self._turn_step_unlocked(now)

        return 0.0, 0.0

    # -------------------------------------------------------------- nudge
    def _nudge_progress_unlocked(self, n):
        """(signed progress, lateral drift, total displacement) in metres.

        Progress is the displacement PROJECTED ONTO THE HEADING THE NUDGE
        STARTED FROM, then signed by the commanded direction, so it is positive
        only when the rover is going the way it was told to.

        This replaces a straight hypot(), which was direction-blind: on
        2026-07-31 that bug let a nudge commanded FORWARD report "done" after
        the rover had actually travelled 290mm BACKWARD. Distance is not
        progress unless it points the right way.
        """
        dx = self.odom_x - n["x0"]
        dy = self.odom_y - n["y0"]
        h = n["heading0"]
        along = dx * math.cos(h) + dy * math.sin(h)
        lateral = -dx * math.sin(h) + dy * math.cos(h)
        return along * n["sign"], lateral, math.hypot(dx, dy)

    def _nudge_step_unlocked(self, now):
        n = self.nudge
        if not n:
            self.mode = "idle"
            return 0.0, 0.0
        if self.odom_x is None:
            self._finish_nudge_unlocked("no odometry", 0.0, 0.0, 0.0, now)
            return 0.0, 0.0

        progress, lateral, total = self._nudge_progress_unlocked(n)
        elapsed = now - n["t0"]

        if progress >= n["target_m"]:
            self._finish_nudge_unlocked("done", progress, lateral, total, now)
            return 0.0, 0.0
        if progress < -(WRONG_WAY_MM / 1000.0):
            # Going the opposite way to the command. Something is wrong with
            # the drive; do not keep feeding it velocity.
            self._finish_nudge_unlocked("wrong-way abort", progress, lateral, total, now)
            return 0.0, 0.0
        if elapsed > STALL_CHECK_S and abs(total) < (STALL_MIN_MM / 1000.0):
            self._finish_nudge_unlocked("stalled (no motion; firmware deadband?)",
                                        progress, lateral, total, now)
            return 0.0, 0.0
        if elapsed > n["timeout_s"]:
            self._finish_nudge_unlocked("timeout", progress, lateral, total, now)
            return 0.0, 0.0
        return (n["sign"] * DOCK_MPS, 0.0)

    def _finish_nudge_unlocked(self, reason, progress, lateral, total, now):
        n = self.nudge or {}
        self.nudge = None
        self.mode = "idle"
        heading_change = wrap180(math.degrees(self.odom_yaw) -
                                 math.degrees(n.get("heading0", self.odom_yaw)))
        log(f"nudge {reason}: progress {progress*1000:+.0f}mm of "
            f"{n.get('target_m', 0)*1000:.0f}mm (total displacement "
            f"{total*1000:.0f}mm, lateral {lateral*1000:+.0f}mm, "
            f"heading {heading_change:+.1f}deg) in {now - n.get('t0', now):.2f}s")
        ok = reason == "done"
        self._event("events/ack", {"action": "nudge", "state": "finished",
                                   "reason": reason, "ok": ok,
                                   "requested_mm": jnum(n.get("target_m", 0) * 1000, 1),
                                   # signed, along the commanded direction
                                   "measured_mm": jnum(progress * 1000, 1),
                                   "displacement_mm": jnum(total * 1000, 1),
                                   "lateral_mm": jnum(lateral * 1000, 1),
                                   "heading_change_deg": jnum(heading_change, 1),
                                   "elapsed_s": jnum(now - n.get("t0", now), 2),
                                   "dir": n.get("dir")})

    # --------------------------------------------------------------- turn
    def _turn_step_unlocked(self, now):
        t = self.turn
        if not t:
            self.mode = "idle"
            return 0.0, 0.0

        turned = abs(self.yaw_int - t["yaw0"])
        elapsed = now - t["t0"]

        if t["phase"] == "driving":
            if elapsed > TURN_TIMEOUT_S:
                t["phase"] = "coasting"
                t["coast_t0"] = now
                t["reason"] = "timeout"
                return 0.0, 0.0
            if (elapsed > TURN_STALL_CHECK_S
                    and math.degrees(turned) < TURN_STALL_MIN_DEG):
                # Same deadband/windup guard as the nudge: stop commanding
                # rotation the rover is visibly not performing.
                t["phase"] = "coasting"
                t["coast_t0"] = now
                t["reason"] = "stalled (no rotation; firmware deadband?)"
                return 0.0, 0.0
            if turned >= t["stop_at_rad"]:
                # Cut drive early and let momentum carry the rest.
                t["phase"] = "coasting"
                t["coast_t0"] = now
                t["reason"] = "done"
                return 0.0, 0.0
            return (0.0, t["sign"] * DOCK_TURN_RADPS)

        # coasting: hold zero, then measure what actually happened
        if now - t["coast_t0"] >= TURN_SETTLE_S:
            self._finish_turn_unlocked(t.get("reason", "done"), now)
        return 0.0, 0.0

    def _finish_turn_unlocked(self, reason, now):
        t = self.turn or {}
        self.turn = None
        self.mode = "idle"
        gyro_deg = math.degrees(self.yaw_int - t.get("yaw0", self.yaw_int))
        odom_deg = wrap180(math.degrees(self.odom_yaw - t.get("odom_yaw0", self.odom_yaw)))
        req = t.get("deg", 0.0) * t.get("sign", 1.0)
        err = gyro_deg - req
        log(f"turn {reason}: requested {req:+.1f}deg, gyro-measured {gyro_deg:+.1f}deg "
            f"(err {err:+.1f}), odom-measured {odom_deg:+.1f}deg, "
            f"{now - t.get('t0', now):.2f}s")
        self._event("events/ack", {
            "action": "turn", "state": "finished", "reason": reason,
            "dir": t.get("dir"),
            "requested_deg": jnum(abs(req), 1),
            "signed_requested_deg": jnum(req, 1),
            # The number the operator asked for: what it ACTUALLY turned.
            "measured_deg": jnum(gyro_deg, 1),
            "measured_deg_odom": jnum(odom_deg, 1),
            "error_deg": jnum(err, 1),
            "coast_factor": TURN_COAST_FACTOR,
            "elapsed_s": jnum(now - t.get("t0", now), 2)})

    # ------------------------------------------------------------- publish
    def _publish_twist(self, vx_real, wz_real):
        try:
            t = Twist()
            t.linear.x = float(to_cmd(vx_real))
            t.linear.y = 0.0
            t.linear.z = 0.0
            t.angular.x = 0.0
            t.angular.y = 0.0
            t.angular.z = float(to_cmd_ang(wz_real))
            self.pub_cmd.publish(t)
        except Exception as e:
            log(f"cmd_vel publish failed {e}")

    def _safe_zero(self):
        try:
            self.pub_cmd.publish(Twist())
        except Exception:
            pass

    # ------------------------------------------------------------- commands
    def handle_command(self, action, payload):
        """Called on paho's thread. Sets state; never publishes a Twist
        (except `stop`, which is allowed to slam zeros out immediately)."""
        try:
            log(f"command: {action} {payload}")
            with self.lock:
                self.last_cmd = action
                self.last_cmd_at = time.time()
            if action == "jog":
                self._cmd_jog(payload)
            elif action in ("stop", "estop", "auto_off"):
                self._cmd_stop(action)
            elif action == "nudge":
                self._cmd_nudge(payload)
            elif action == "turn":
                self._cmd_turn(payload)
            elif action == "mission":
                self._cmd_mission(payload)
            elif action == "set_coordinate":
                self._cmd_set_coordinate(payload)
            else:
                self._nack(action, "unknown command")
        except Exception as e:
            log(f"command {action} failed: {e}")
            # A command that blew up must not leave the rover driving.
            try:
                with self.lock:
                    self.mode = "idle"
                    self.nudge = None
                    self.turn = None
            except Exception:
                pass
            self._nack(action, str(e)[:200])

    def _nack(self, action, error):
        with self.lock:
            self.last_error = error
        self._event("events/nack", {"action": action, "error": error})

    def _cmd_jog(self, p):
        try:
            vx = float(p.get("vx", 0.0))
            wz = float(p.get("wz", 0.0))
        except Exception:
            self._nack("jog", "vx/wz not numeric")
            return
        if not (math.isfinite(vx) and math.isfinite(wz)):
            self._nack("jog", "vx/wz not finite")
            return
        vx = clamp(vx, -1.0, 1.0)
        wz = clamp(wz, -1.0, 1.0)

        with self.lock:
            block = self._motion_block_unlocked()
            if block:
                # A joystick streams at 10-20 Hz; nacking every frame would
                # flood the broker. One complaint every 2s is enough.
                now = time.monotonic()
                if now - self._jog_nack_at > 2.0:
                    self._jog_nack_at = now
                    self.last_error = block
                    self._event("events/nack", {"action": "jog", "error": block})
                self.mode = "idle"
                self.jog_vx = self.jog_wz = 0.0
                return
            if self.mode in ("nudge", "turn"):
                # An operator grabbing the stick outranks a running manoeuvre.
                if self.mode == "nudge":
                    try:
                        prog, lat, tot = self._nudge_progress_unlocked(self.nudge)
                    except Exception:
                        prog = lat = tot = 0.0
                    self._finish_nudge_unlocked("superseded by jog", prog, lat, tot,
                                                time.monotonic())
                else:
                    self._finish_turn_unlocked("superseded by jog", time.monotonic())
            self.jog_vx, self.jog_wz = vx, wz
            self.jog_ts = time.monotonic()
            self.jog_warned = False
            self.mode = "jog"
        self._event("events/ack", {"action": "jog", "vx": jnum(vx, 3), "wz": jnum(wz, 3),
                                   "target_mps": jnum(vx * JOG_MAX_MPS, 4),
                                   "target_radps": jnum(wz * TURN_MAX_RADPS, 4)})

    def _cmd_stop(self, action):
        # NEVER gated. Not on battery, not on link state, not on anything.
        with self.lock:
            self.mode = "idle"
            self.nudge = None
            self.turn = None
            self.jog_vx = self.jog_wz = 0.0
            self.jog_ts = 0.0
            # Bypass the ramp. A stop is not negotiable.
            self.cur_vx = 0.0
            self.cur_wz = 0.0
        for _ in range(8):
            self._safe_zero()
            time.sleep(0.02)
        log(f"{action}: hard zero sent x8, mode=idle")
        self._event("events/ack", {"action": action, "estop": True,
                                   "stopped": True, "mode": "idle"})

    def _cmd_nudge(self, p):
        d = str(p.get("dir", "fwd")).lower()
        if d not in ("fwd", "forward", "back", "backward", "rev"):
            self._nack("nudge", f"bad dir {d!r}")
            return
        sign = 1.0 if d in ("fwd", "forward") else -1.0
        try:
            mm = float(p.get("mm", 100))
        except Exception:
            self._nack("nudge", "mm not numeric")
            return
        if not math.isfinite(mm) or mm <= 0:
            self._nack("nudge", "mm must be > 0")
            return
        if mm > MAX_NUDGE_MM:
            self._nack("nudge", f"mm {mm} exceeds cap {MAX_NUDGE_MM}")
            return

        with self.lock:
            block = self._motion_block_unlocked()
            if block:
                self.last_error = block
                self._event("events/nack", {"action": "nudge", "error": block})
                return
            target_m = mm / 1000.0
            # Generous but bounded: 3x the ideal time plus ramp allowance, so a
            # slipping wheel ends in a reported timeout, not an endless drive.
            timeout_s = min(60.0, (target_m / DOCK_MPS) * 3.0 + 4.0)
            self.nudge = {"x0": self.odom_x, "y0": self.odom_y,
                          "heading0": self.odom_yaw,
                          "target_m": target_m, "sign": sign,
                          "t0": time.monotonic(), "timeout_s": timeout_s,
                          "dir": d}
            self.mode = "nudge"
        log(f"nudge start: {d} {mm}mm at {DOCK_MPS} m/s (timeout {timeout_s:.1f}s)")
        self._event("events/ack", {"action": "nudge", "state": "started",
                                   "dir": d, "mm": jnum(mm, 1),
                                   "speed_mps": DOCK_MPS,
                                   "timeout_s": jnum(timeout_s, 1)})

    def _cmd_turn(self, p):
        d = str(p.get("dir", "left")).lower()
        if d not in ("left", "ccw", "right", "cw"):
            self._nack("turn", f"bad dir {d!r}")
            return
        # REP-103: +z is counter-clockwise, i.e. left.
        sign = 1.0 if d in ("left", "ccw") else -1.0
        try:
            deg = float(p.get("deg", 15))
        except Exception:
            self._nack("turn", "deg not numeric")
            return
        if not math.isfinite(deg) or deg <= 0:
            self._nack("turn", "deg must be > 0")
            return
        if deg > MAX_TURN_DEG:
            self._nack("turn", f"deg {deg} exceeds cap {MAX_TURN_DEG}")
            return

        with self.lock:
            block = self._motion_block_unlocked()
            if block:
                self.last_error = block
                self._event("events/nack", {"action": "turn", "error": block})
                return
            if (time.monotonic() - self.imu_last) > ROS_DEAD_S or self.imu_last == 0.0:
                err = "no fresh /imu; refusing to turn open-loop"
                self.last_error = err
                self._event("events/nack", {"action": "turn", "error": err})
                return
            target_rad = math.radians(deg)
            self.turn = {"yaw0": self.yaw_int, "odom_yaw0": self.odom_yaw,
                         "deg": deg, "sign": sign, "dir": d,
                         "stop_at_rad": target_rad * TURN_COAST_FACTOR,
                         "t0": time.monotonic(), "phase": "driving",
                         "coast_t0": 0.0, "reason": "done"}
            self.mode = "turn"
        log(f"turn start: {d} {deg}deg at {DOCK_TURN_RADPS} rad/s "
            f"(cut drive at {deg*TURN_COAST_FACTOR:.1f}deg, coast {TURN_SETTLE_S}s)")
        self._event("events/ack", {"action": "turn", "state": "started",
                                   "dir": d, "deg": jnum(deg, 1),
                                   "rate_radps": DOCK_TURN_RADPS,
                                   "coast_factor": TURN_COAST_FACTOR,
                                   "timeout_s": TURN_TIMEOUT_S})

    def _cmd_mission(self, p):
        name = str(p.get("name", ""))
        if name not in ("m1", "m2", "water", "home"):
            self._nack("mission", f"unknown mission {name!r}")
            return
        # Deliberately does not drive. Navigation is not implemented and
        # inventing it under a person's feet is not an option.
        log(f"mission '{name}' received — stub, no motion commanded")
        self._event("events/ack", {"action": "mission", "name": name,
                                   "accepted": True, "started": False,
                                   "note": "stub: logged only, no navigation implemented"})

    def _cmd_set_coordinate(self, p):
        try:
            x_mm = float(p.get("x_mm", 0.0))
            y_mm = float(p.get("y_mm", 0.0))
        except Exception:
            self._nack("set_coordinate", "x_mm/y_mm not numeric")
            return
        if not (math.isfinite(x_mm) and math.isfinite(y_mm)):
            self._nack("set_coordinate", "x_mm/y_mm not finite")
            return
        with self.lock:
            if self.odom_x is None:
                self._nack("set_coordinate", "no odometry yet")
                return
            self.origin = (self.odom_x, self.odom_y, x_mm, y_mm)
        self._save_origin()
        log(f"set_coordinate: current pose is now ({x_mm}, {y_mm}) mm")
        self._event("events/ack", {"action": "set_coordinate",
                                   "x_mm": jnum(x_mm, 1), "y_mm": jnum(y_mm, 1),
                                   "moved": False})

    # -------------------------------------------------------------- origin
    def _load_origin(self):
        try:
            with open(ORIGIN_FILE) as fh:
                d = json.load(fh)
            self.origin = (d["ref_x"], d["ref_y"], d["x_mm"], d["y_mm"])
            log(f"origin loaded from {ORIGIN_FILE}: {self.origin}")
        except Exception:
            self.origin = None

    def _save_origin(self):
        try:
            rx, ry, xm, ym = self.origin
            tmp = ORIGIN_FILE + ".tmp"
            with open(tmp, "w") as fh:
                json.dump({"ref_x": rx, "ref_y": ry, "x_mm": xm, "y_mm": ym,
                           "ts": time.time()}, fh)
            os.replace(tmp, ORIGIN_FILE)
        except Exception as e:
            log(f"origin save failed {e}")

    def _arena_mm_unlocked(self):
        if self.odom_x is None:
            return None, None
        if self.origin is None:
            return self.odom_x * 1000.0, self.odom_y * 1000.0
        rx, ry, xm, ym = self.origin
        return (xm + (self.odom_x - rx) * 1000.0,
                ym + (self.odom_y - ry) * 1000.0)

    # ------------------------------------------------------------ telemetry
    def _event(self, suffix, payload):
        try:
            self.bus.publish(suffix, payload, qos=1)
        except Exception as e:
            log(f"event publish failed {e}")

    def _telemetry_tick(self):
        try:
            now = time.monotonic()
            with self.lock:
                x_mm, y_mm = self._arena_mm_unlocked()
                heading = wrap180(math.degrees(self.odom_yaw))
                batt_raw = self.battery_raw
                batt_v = self._battery_v_unlocked()
                batt_age = (now - self.battery_last) if self.battery_last else None
                vx, wz, mode = self.cur_vx, self.cur_wz, self.mode
                link_ok = self._link_ok_unlocked(now)
                imu_fresh = self.imu_last > 0 and (now - self.imu_last) < ROS_DEAD_S
                odom_hz = hz_from(list(self.odom_times))
                imu_hz = hz_from(list(self.imu_times))
                batt_hz = hz_from(list(self.battery_times))
                jog_age = (now - self.jog_ts) if self.jog_ts else None
                deadman_ok = (mode != "jog") or (jog_age is not None
                                                 and jog_age <= JOG_DEADMAN_S)
                last_cmd, last_cmd_at = self.last_cmd, self.last_cmd_at
                last_error = self.last_error
                gyro_deg_s = math.degrees(self.gyro_z)

            payload = {
                # --- power ---
                "battery_v": jnum(batt_v, 2),
                "battery_raw": batt_raw if isinstance(batt_raw, int) else None,
                # Threshold assumption: 3S pack, 3 x 3.7V nominal = 11.1V.
                "battery_low": (batt_v is not None and batt_v < BATT_LOW_V),
                "battery_age_s": jnum(batt_age, 2),
                "battery_stale": (batt_age is None or batt_age > BATT_STALE_S),

                # --- link health ---
                "ros_ok": bool(link_ok),
                # No direct handle on the micro-ROS session, so this is inferred
                # from the board still producing data: if /odom_raw or /imu are
                # arriving, the session is by definition alive.
                "uros_session": bool(link_ok or imu_fresh),
                "odom_hz": jnum(odom_hz, 2),
                "imu_hz": jnum(imu_hz, 2),
                "batt_hz": jnum(batt_hz, 2),

                # --- motion ---
                "mode": mode,
                "moving": bool(abs(vx) > 1e-4 or abs(wz) > 1e-4),
                # Reported as REAL m/s — what the rover is actually doing, not
                # the scaled number on the wire.
                "cmd_vel": {"vx": jnum(vx, 4), "wz": jnum(wz, 4)},
                "deadman_ok": bool(deadman_ok),
                "ms_since_jog": int(jog_age * 1000) if jog_age is not None else -1,

                # --- pose ---
                "x_mm": jnum(x_mm, 1),
                "y_mm": jnum(y_mm, 1),
                "heading_deg": jnum(heading, 2),
                "yaw_int_deg": jnum(wrap180(math.degrees(self.yaw_int)), 2),
                "gyro_z_dps": jnum(gyro_deg_s, 2),

                # --- bookkeeping ---
                "last_cmd": last_cmd,
                "last_cmd_at": jnum(last_cmd_at, 3),
                "last_error": last_error,
                "uptime_s": jnum(time.time() - STARTED, 1),
                "cmd_scale": CMD_SCALE,
                "origin_set": self.origin is not None,
            }
            self.bus.publish("telemetry/drive", payload, qos=0)
        except Exception as e:
            log(f"telemetry tick error {e}")

    # ------------------------------------------------------------- shutdown
    def shutdown_stop(self):
        """Zero the motors, loudly and repeatedly, before anything is torn down."""
        try:
            with self.lock:
                self.mode = "idle"
                self.nudge = None
                self.turn = None
                self.cur_vx = self.cur_wz = 0.0
        except Exception:
            pass
        for _ in range(10):
            self._safe_zero()
            time.sleep(0.03)
        log("shutdown: zero Twist published x10")


# ====================================================================== MAIN
STOP_FLAG = threading.Event()


def main():
    signal.signal(signal.SIGTERM, lambda *_: STOP_FLAG.set())
    signal.signal(signal.SIGINT, lambda *_: STOP_FLAG.set())

    bus = Bus()
    bus.start()

    node = None
    try:
        rclpy.init(args=None)
        node = TeleopNode(bus)
        bus.on_command = node.handle_command

        # Own spin loop rather than rclpy.spin(): the shutdown path has to run
        # BEFORE the context is destroyed, because it needs a live publisher to
        # send the zero Twist.
        while not STOP_FLAG.is_set():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except Exception as e:
                log(f"spin error {e}")
                time.sleep(0.1)
    except Exception as e:
        log(f"fatal: {e}")
    finally:
        try:
            if node is not None:
                node.shutdown_stop()
                bus.publish("events/offline", {"svc": "teleop", "status": "offline",
                                               "reason": "clean shutdown"}, qos=1)
                time.sleep(0.2)
                node.destroy_node()
        except Exception as e:
            log(f"shutdown error {e}")
        try:
            bus.stop()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        log("exited")


if __name__ == "__main__":
    main()
