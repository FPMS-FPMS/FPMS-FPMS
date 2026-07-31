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
DOCK_MPS = 0.025          # slow docking near a waypoint — see MIN_CMD_LIN: this
                          # is BELOW the deadband floor and gets raised to it
TURN_MAX_RADPS = 0.4      # slow turning
DOCK_TURN_RADPS = 0.2     # closed-loop `turn` command — likewise floored


# ============================================================ DEADBAND FLOOR
# MEASURED 2026-07-31: a nudge asked for DOCK_MPS = 0.025 m/s, which leaves this
# file as linear.x = 0.025 / 6.1 = 0.0041 on the wire. The rover sat perfectly
# still for ~3 seconds and then LURCHED ~290mm BACKWARD with 24 degrees of
# unintended rotation. That is the signature of a wheel-velocity controller in
# the ESP32 firmware integrating error while the motors are stalled under their
# own deadband, then dumping the accumulated integral the instant static
# friction breaks — in whichever direction the two wheels happen to break first.
#
# The conclusion is counter-intuitive and it is the whole point of this section:
# asking this chassis to go SLOWER THAN IT CAN GO makes it MORE dangerous, not
# less. "Very very slow" below the deadband is not slow motion, it is stored
# energy waiting for a release the operator cannot predict. So the fix is a
# FLOOR — every non-zero command is raised until it clears the deadband. There
# is no path in this file that may scale a command down toward 0.0041 again.
#
# ---- THESE TWO NUMBERS ARE ESTIMATES PENDING A MEASURED SWEEP -------------
# The sweep that would establish the real deadband (walk linear.x up from 0.002
# in 0.001 steps, note where the wheels first turn smoothly rather than lurch)
# was ABORTED and has NOT been run. What is actually known:
#   * 0.0041 on the wire is BELOW the deadband  (measured: stall then lurch)
#   * 0.10   on the wire is comfortably above   (measured: 0.61 m/s ground)
# The truth is somewhere in a factor-of-24 gap, and these defaults are a guess
# inside it. Re-run the sweep and then set the real values in config.env.
#
# MIN_CMD_LIN = 0.035 real m/s (0.0057 on the wire) was chosen as follows:
#   * it is 1.4x the value measured to be dead, so it is a genuine raise;
#   * it is 70% of JOG_MAX_MPS, which leaves a usable band between "floor" and
#     "full stick" — a floor at the cap would delete speed control entirely;
#   * it errs LOW rather than high because a person stands next to this rover,
#     and a floor that is too low fails visibly (stall watchdog aborts the move)
#     while a floor that is too high fails by driving faster than anyone asked.
#     Failing toward "aborts and tells you" is the correct direction.
#
# MIN_CMD_ANG = 0.30 real rad/s (0.049 on the wire) is weaker still — the
# angular deadband has never been measured at all, not even a failing point.
# Reasoning by geometry: in a pure spin each wheel runs at w * track/2, so with
# an assumed ~0.20 m track a wheel-speed floor of 0.035 m/s implies w >= 0.35
# rad/s. 0.30 is deliberately set just under that, because in a spin the two
# wheels drive against each other and break static friction more easily than
# one wheel does in a straight line. If turns still stall, 0.35 via config.env
# is the next thing to try — that is what the override exists for.
#
# ---- WHAT THIS COSTS: the slowest speed that is now ACHIEVABLE ------------
# Linear:  0.035 m/s = 35 mm/s. The operator asked for 25 mm/s. This is 40%
#          FASTER than requested, and that is stated plainly rather than hidden:
#          25 mm/s was never actually available on this chassis. What 25 mm/s
#          produced was three seconds of nothing followed by 290mm of backward
#          lurch — an average that flatters the number and a peak that does not.
#          A 100mm nudge now takes ~2.9s of real motion instead of ~4s of
#          stall-then-jump. A lurch is not slow. This is the slower option.
# Angular: 0.30 rad/s = 17.2 deg/s, up from the 0.2 rad/s (11.5 deg/s) the
#          `turn` command asked for. `turn` is closed-loop on the gyro and cuts
#          drive at TURN_COAST_FACTOR, so it absorbs the higher rate by stopping
#          sooner; the reported measured_deg is unaffected.
#
# Note also that the acceleration ramp no longer exists BELOW the floor: a start
# is now a step straight to MIN_CMD_LIN rather than a glide up through 0.004.
# That is intentional. Gliding up through the deadband is precisely how the
# integrator gets fed, so a "gentle" ramp through it was never gentle.
#
# Overridable from /etc/fpms/config.env without editing this file:
#   FPMS_MIN_CMD_LIN=0.04
#   FPMS_MIN_CMD_ANG=0.35
# Setting either to 0 disables that floor and restores the old (lurching)
# behaviour; that is allowed only because a measurement rig may need it.
CFG_NOTES = []


def _cfg_float(key, default, lo, hi):
    """Read a tuning float from config.env, falling back to `default`.

    Never raises. A rover that refuses to boot because someone fat-fingered a
    tuning constant is worse than a rover running the documented default, and
    this node is the only thing that can stop the motors. The range check is not
    cosmetic either: these constants exist to MAKE the rover move, so a typo'd
    FPMS_MIN_CMD_LIN=5 has to be caught here rather than on the wire.

    Notes are queued instead of logged because log() does not exist yet at
    import time; TeleopNode prints them once at startup.
    """
    raw = CFG.get(key)
    if raw is None:
        return default
    try:
        v = float(raw)
        if not math.isfinite(v) or not (lo <= v <= hi):
            raise ValueError(f"outside [{lo}, {hi}]")
        CFG_NOTES.append(f"config: {key}={v} overrides default {default}")
        return v
    except Exception as e:
        CFG_NOTES.append(f"config: {key}={raw!r} rejected ({e}); using {default}")
        return default


# Upper bound is the motion envelope itself: a floor above the clamp would make
# the floor the only speed the rover has, which is a worse bug than the one this
# is fixing. snap_up() re-clamps regardless, so this is belt and braces.
MIN_CMD_LIN = _cfg_float("FPMS_MIN_CMD_LIN", 0.035, 0.0, JOG_MAX_MPS)
MIN_CMD_ANG = _cfg_float("FPMS_MIN_CMD_ANG", 0.30, 0.0, TURN_MAX_RADPS)

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

# --- lurch guard ------------------------------------------------------------
# The stall/wrong-way checks above live inside the nudge and turn state machines
# and therefore protect exactly two of the four ways this node can drive. The
# lurch guard sits in the control tick instead, so it covers EVERY motion path —
# jog included, which is the one an operator is most likely to be standing next
# to. It answers one question 20 times a second: is the chassis doing something
# the command does not explain?
#
# Two shapes, both observed on 2026-07-31:
#   * driving the wrong way    — commanded forward, odometry says backward
#   * moving with no command   — cmd_vel is zero and the rover is still going,
#                                i.e. the integrator releasing after the command
#                                that wound it up has already been withdrawn
LURCH_MIN_MPS = 0.02      # below this, odometry is noise, not motion
LURCH_CONFIRM_S = 0.25    # must persist this long — one bad frame must not stop
                          # the rover, and one good frame must not excuse a lurch
LURCH_COAST_GRACE_S = 1.0  # after cmd_vel goes to zero, real momentum carries the
                           # rover for a moment; that is coasting, not a lurch
LURCH_NACK_COOLDOWN_S = 2.0  # rate-limits the COMPLAINT only, never the stop

# --- health thresholds ---
# 3S LiPo assumption: 3 cells x 3.7V nominal = 11.1V. Below that the pack is
# into the knee of its discharge curve and motor current will sag it further,
# so new motion is refused there rather than at a true-empty 9.9V.
BATT_LOW_V = 11.1
ROS_DEAD_S = 3.0          # no /odom_raw for this long => link considered dead
BATT_STALE_S = 10.0


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def snap_up(v, floor, cap):
    """Raise a non-zero magnitude to `floor`, preserving sign; clamp to `cap`.

    EXACTLY ZERO STAYS EXACTLY ZERO. That is the one property this function must
    never lose: every stop path in this file converges on a zero, and a floor
    that turned 0.0 into 0.035 would turn `stop` into a creep. The test is `v ==
    0.0`, not a tolerance, because a ramp that has genuinely reached zero lands
    on it exactly and anything else is real (if tiny) commanded motion that the
    firmware cannot execute smoothly and therefore must be raised.

    The cap is re-applied afterwards so that the floor can never be used to
    smuggle a value past the motion envelope — if a floor is ever configured
    above its cap, the cap wins and the rover ends up at its normal maximum
    rather than beyond it.
    """
    if v == 0.0 or floor <= 0.0:
        return clamp(v, -cap, cap)
    if abs(v) < floor:
        v = math.copysign(floor, v)
    return clamp(v, -cap, cap)


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
                                     # The floors are limits too — they are the
                                     # bottom of the envelope, not the top.
                                     "min_cmd_lin": MIN_CMD_LIN,
                                     "min_cmd_ang": MIN_CMD_ANG,
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
        self.odom_vx = 0.0           # signed real m/s along heading, from deltas
        self._odom_prev = None       # (x, y, yaw, t) of the previous frame

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
        self._deadband_snapped = False
        self._lurch_since = 0.0      # when the current suspicion started
        self._lurch_nack_at = 0.0
        self._cmd_zero_since = 0.0   # when cmd_vel last became zero
        self.lurch_trips = 0

        for note in CFG_NOTES:
            log(note)
        log(f"deadband floor: lin {MIN_CMD_LIN} m/s, ang {MIN_CMD_ANG} rad/s "
            f"(ESTIMATES — sweep not yet run)")

        self.create_timer(CONTROL_DT, self._control_tick)
        self.create_timer(1.0 / TELEM_HZ, self._telemetry_tick)
        log("teleop node up: /cmd_vel publisher; /odom_raw + /imu + /battery subs")

    # ------------------------------------------------------------ ROS input
    def _on_odom(self, msg):
        try:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = yaw_from_quat(msg.pose.pose.orientation)
            now = time.monotonic()
            with self.lock:
                # Signed ground speed along the PREVIOUS heading, differentiated
                # from position rather than read out of msg.twist. The board's
                # own state estimate is demonstrably partial — it publishes an
                # identity orientation — and the lurch guard exists precisely to
                # contradict the firmware, so it cannot be built on the
                # firmware's opinion of its own velocity.
                if self._odom_prev is not None:
                    px, py, pyaw, pt = self._odom_prev
                    dt = now - pt
                    # Same absurd-gap rule as the IMU integrator: a stalled link
                    # resuming must not manufacture a huge apparent velocity.
                    if 0.0 < dt < 0.5:
                        dx, dy = x - px, y - py
                        along = dx * math.cos(pyaw) + dy * math.sin(pyaw)
                        # Light smoothing. One noisy frame must not be able to
                        # halt the rover, and the guard's confirm window means
                        # the lag this adds costs nothing.
                        self.odom_vx = 0.5 * self.odom_vx + 0.5 * (along / dt)
                self._odom_prev = (x, y, yaw, now)
                self.odom_x = x
                self.odom_y = y
                self.odom_yaw = yaw
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

                # Track how long the command has been zero, for the coast window
                # the lurch guard needs. Done after the clamp so it reflects what
                # is actually about to go out, not what some mode wanted.
                if abs(self.cur_vx) <= 1e-6 and abs(self.cur_wz) <= 1e-6:
                    if not self._cmd_zero_since:
                        self._cmd_zero_since = now
                else:
                    self._cmd_zero_since = 0.0

                # LURCH GUARD — every motion path, not just nudge and turn.
                lurch = self._lurch_guard_unlocked(now)
                if lurch:
                    self._halt_for_lurch_unlocked(lurch, now)

                vx, wz, mode = self.cur_vx, self.cur_wz, self.mode

            if lurch:
                # Out of the lock and straight to zero. Not via the ramp, not
                # via the idle heartbeat's rate limit: the whole point is that
                # the rover is already moving in a way nobody commanded.
                self._safe_zero()
                return

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

    # --------------------------------------------------------- lurch guard
    def _lurch_guard_unlocked(self, now):
        """Reason the chassis is contradicting the command, or None.

        Caller holds the lock. Deliberately reads only odometry and the
        post-clamp command, so it is blind to which mode produced that command
        and therefore cannot be bypassed by adding a new one.
        """
        if not self._link_ok_unlocked(now):
            # No trustworthy odometry to judge with. The link watchdog above
            # already halts motion in this case; a second opinion built on stale
            # data would only produce false trips.
            self._lurch_since = 0.0
            return None

        speed = self.odom_vx
        vx_cmd, wz_cmd = self.cur_vx, self.cur_wz
        if abs(speed) < LURCH_MIN_MPS:
            self._lurch_since = 0.0
            return None

        reason = None
        if abs(vx_cmd) > 1e-6:
            if (speed * vx_cmd) < 0.0:
                reason = (f"commanded {vx_cmd:+.3f} m/s but odometry reports "
                          f"{speed:+.3f} m/s")
        elif abs(wz_cmd) <= 1e-6:
            # Zero command, non-zero motion. Real momentum explains the first
            # moment of this, so only complain once the coast window has passed.
            if (self._cmd_zero_since
                    and (now - self._cmd_zero_since) > LURCH_COAST_GRACE_S):
                reason = (f"cmd_vel has been zero for "
                          f"{now - self._cmd_zero_since:.1f}s but odometry "
                          f"reports {speed:+.3f} m/s")

        if reason is None:
            self._lurch_since = 0.0
            return None

        # Confirm before acting. A single frame of odometry noise stopping the
        # rover mid-manoeuvre would train the operator to ignore this guard.
        if not self._lurch_since:
            self._lurch_since = now
            return None
        if (now - self._lurch_since) < LURCH_CONFIRM_S:
            return None
        return reason

    def _halt_for_lurch_unlocked(self, reason, now):
        """Stop everything. Caller holds the lock; the zero Twist goes out in
        the control tick as soon as the lock is released."""
        self.lurch_trips += 1
        self._lurch_since = 0.0
        mode_was = self.mode
        self.mode = "idle"
        self.nudge = None
        self.turn = None
        self.jog_vx = self.jog_wz = 0.0
        self.cur_vx = self.cur_wz = 0.0
        self._cmd_zero_since = now
        err = f"lurch guard: {reason} (mode was {mode_was})"
        self.last_error = err
        log(f"LURCH GUARD tripped — {err}; halting")
        # The stop is unconditional; only the complaint is rate-limited, so a
        # persistent fault cannot flood the broker but also cannot go unnoticed.
        if (now - self._lurch_nack_at) > LURCH_NACK_COOLDOWN_S:
            self._lurch_nack_at = now
            self._event("events/nack", {"action": mode_was, "error": err,
                                        "lurch_guard": True,
                                        "odom_vx": jnum(self.odom_vx, 4),
                                        "trips": self.lurch_trips})

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
            # LAST STEP BEFORE THE WIRE, and the mirror image of the clamp: the
            # clamp is the ceiling nothing may exceed, this is the floor nothing
            # non-zero may sit under. Every command path converges here, so no
            # future mode can reintroduce a sub-deadband creep the way DOCK_MPS
            # did. Zero is untouched — see snap_up().
            snapped = (0.0 < abs(vx_real) < MIN_CMD_LIN
                       or 0.0 < abs(wz_real) < MIN_CMD_ANG)
            vx_out = snap_up(vx_real, MIN_CMD_LIN, JOG_MAX_MPS)
            wz_out = snap_up(wz_real, MIN_CMD_ANG, TURN_MAX_RADPS)
            with self.lock:
                self._deadband_snapped = snapped

            t = Twist()
            t.linear.x = float(to_cmd(vx_out))
            t.linear.y = 0.0
            t.linear.z = 0.0
            t.angular.x = 0.0
            t.angular.y = 0.0
            t.angular.z = float(to_cmd_ang(wz_out))
            self.pub_cmd.publish(t)
        except Exception as e:
            log(f"cmd_vel publish failed {e}")

    def _safe_zero(self):
        # Bypasses _publish_twist entirely: a default Twist is already all
        # zeros, and a stop must not pass through the floor logic at all.
        try:
            self._deadband_snapped = False
        except Exception:
            pass
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
        # Report the speed it will ACTUALLY drive at, not the one requested.
        # DOCK_MPS is below MIN_CMD_LIN and will be floored on the way out; an
        # ack claiming 0.025 m/s would be advertising the speed that lurched.
        eff_mps = snap_up(DOCK_MPS, MIN_CMD_LIN, JOG_MAX_MPS)
        log(f"nudge start: {d} {mm}mm at {eff_mps} m/s "
            f"(requested {DOCK_MPS}, raised to clear deadband; "
            f"timeout {timeout_s:.1f}s)")
        self._event("events/ack", {"action": "nudge", "state": "started",
                                   "dir": d, "mm": jnum(mm, 1),
                                   "speed_mps": jnum(eff_mps, 4),
                                   "requested_mps": DOCK_MPS,
                                   "deadband_floored": eff_mps != DOCK_MPS,
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
        # As with nudge: DOCK_TURN_RADPS is under the angular floor and will be
        # raised on the way out, so report the rate that will actually be used.
        # The closed loop absorbs the difference by cutting drive sooner.
        eff_radps = snap_up(DOCK_TURN_RADPS, MIN_CMD_ANG, TURN_MAX_RADPS)
        log(f"turn start: {d} {deg}deg at {eff_radps} rad/s "
            f"(requested {DOCK_TURN_RADPS}, raised to clear deadband; "
            f"cut drive at {deg*TURN_COAST_FACTOR:.1f}deg, coast {TURN_SETTLE_S}s)")
        self._event("events/ack", {"action": "turn", "state": "started",
                                   "dir": d, "deg": jnum(deg, 1),
                                   "rate_radps": jnum(eff_radps, 4),
                                   "requested_radps": DOCK_TURN_RADPS,
                                   "deadband_floored": eff_radps != DOCK_TURN_RADPS,
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
                snapped = self._deadband_snapped
                odom_vx = self.odom_vx
                lurch_trips = self.lurch_trips

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
                # True while the last published command was raised to the floor,
                # so the operator can SEE the rover refusing to creep rather than
                # wondering why "slower" did nothing.
                "deadband_snapped": bool(snapped),
                # What odometry says the rover is doing, as opposed to what it
                # was told to do. The lurch guard compares exactly these two.
                "odom_vx": jnum(odom_vx, 4),
                "lurch_trips": int(lurch_trips),

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
                # The calibration the dashboard should show next to the speeds:
                # cmd_scale is what divides them, the floors are what they can
                # never go below. Both are real-world units.
                "min_cmd_lin": jnum(MIN_CMD_LIN, 4),
                "min_cmd_ang": jnum(MIN_CMD_ANG, 4),
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
