#!/usr/bin/env python3
"""
FPMS STM32 bridge -- Yahboom Rosmaster board (CH340, 115200) -> the FPMS ROS 2 contract.

WHY THIS FILE EXISTS
====================
Rover 2's controller was swapped on 2026-08-07 from the Yahboom MicroROS Board
V2.0 (ESP32-S3, CP2102, micro-ROS/XRCE at 230400) to a Yahboom STM32 ROS board
(CH340, Rosmaster protocol at 115200). The STM32 board is NOT a micro-ROS
client -- there is no 0x7E begin-flag anywhere in its stream -- so
`micro_ros_agent` can never establish a session with it, and every topic the
mission executor needs had publisher count 0.

This node replaces micro_ros_agent as the transport. It speaks Rosmaster over
serial (the oldest, proven path for this board -- same library the stock
yahboomcar_bringup driver uses) and republishes it as EXACTLY the topics
fpms_missions.py and fpms_odom_tf.py already subscribe to. Nothing downstream
is rewritten; B8B is untouched.

    /odom_raw   nav_msgs/Odometry          <- integrated from board velocity
    /imu        sensor_msgs/Imu            <- board accel + gyro
    /battery    sensor_msgs/BatteryState   <- board voltage
    /wheel_ticks std_msgs/Int32MultiArray  <- raw 4x encoder counts (for calibration)
    /cmd_vel    geometry_msgs/Twist        -> set_car_motion()  [GATED, see below]
    /cmd_duty   std_msgs/Int32MultiArray   -> set_motor()       [GATED, see below]

TWO MOTION INPUTS -- AND WHY THE RAW ONE EXISTS
===============================================
/cmd_vel is a VELOCITY setpoint the board closes its own PID around. That path
has a dead zone and an acceleration ramp on this chassis: a short burst ends
before it ever reaches cruise, so healthy segments abort measuring ~0 mm. This
is the documented "/cmd_vel cannot crawl" behaviour.

/cmd_duty is the RAW per-wheel duty path, ported from
/home/ubuntu/fpms_phase6_M1_M2_WORKING.py -- the program that actually completed
missions M1 and M2 on this same Yahboom board. It drove every metre of those
missions through `bot.set_motor(p1, p2, p3, p4)` with values around 17-50, e.g.

    b6_forward:      bot.set_motor(p, p, p, p)      with B6_FORWARD_POWER = 26
    p5_drive_to_marker: bot.set_motor(l, l, r, r)   # M1,M2=left; M3,M4=right
    _p5_navdrive._fwd:  bot.set_motor(pwr-c, pwr-c, pwr+c, pwr+c)   # _DRV = 26

WHEEL ORDER: data[0], data[1] = LEFT pair (M1, M2); data[2], data[3] = RIGHT
pair (M3, M4) -- stated verbatim at fpms_phase6_M1_M2_WORKING.py:2469 and
consistent with _fwd()'s heading correction, which SUBTRACTS the correction
from the first two and ADDS it to the last two.

SIGN: this node applies NO sign transform. Duty goes to the board exactly as
received, so the caller owns polarity exactly as the golden program did. For
the record, in the golden file the paths that ran M1/M2 use POSITIVE duty for
forward (`B6_FORWARD_POWER = 26  # PHASE5: flipped -- new chassis is opposite
sign`, and `pwr = -_DRV if rev else _DRV`, i.e. reverse is the negated one).
The stale header comment at its line 14 ("set_motor negative values drive
forward") predates that PHASE5 flip and is contradicted by the constants below
it. Do not encode either claim here -- the executor decides, and it can be
checked against /wheel_ticks.

SIGN CONVENTION -- READ BEFORE CHANGING ANYTHING
================================================
This node publishes the CORRECT convention: forward is +x, CCW is +yaw. The old
ESP32 board reported inverted, which is why fpms_odom_tf.py carried
ODOM_TWIST_SIGN = -1 and STACK.md 7 lists it as "must become +1 with v3".
Because this bridge emits correct signs, that constant MUST be +1. Publishing
correct data into a stack still compensating for a lie is the exact failure
this project has hit before: every number stays plausible and the rover drives
backwards.

THE ARM GATE -- AND WHY IT FOLLOWS THE EXECUTOR
==============================================
The node boots disarmed: it publishes every sensor topic normally but IGNORES
/cmd_vel, so the wheels cannot turn. That exists because none of this
drivetrain's calibration survived the board swap.

It ARMS ITSELF FROM `/fpms/mission/armed`. That is the executor's own arm latch,
mirrored as a read-out by fpms_foxglove_cmd. The operator's single deliberate
ARM press therefore opens both gates at once, and the executor's disarm --
on stop, on abort, on mission end, on its 120 s timeout -- closes both.

This is deliberate. A second, independent arm gate that the UI did not know
about was worse than no gate: the console armed the executor, the executor
published a perfectly good /cmd_vel, this node silently dropped it, and the run
died with `burst cap ... measured +0mm` -- a message that names neither the real
cause nor this node. One operator action, one arm state, and the failure is
visible in this log either way.

Still enforced regardless of arm state: the /estop latch, the CMD_TIMEOUT_S
deadman, and the MAX_V/MAX_W refusal. Manual override remains available:

    ros2 topic pub --once /bridge_arm std_msgs/Bool '{data: true}'   # or false
    FPMS_BRIDGE_ARMED=1   # force-armed at boot, ignores the executor

An ARM (from either source) also clears a latched /estop, because arming is the
operator's explicit consent to motion and a stale latch from a previous run
would otherwise leave the rover silently dead.
"""

import json
import math
import os
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Int32MultiArray, Bool

from Rosmaster_Lib import Rosmaster

import paho.mqtt.client as mqtt
from paho.mqtt.client import CallbackAPIVersion

PORT = os.environ.get("FPMS_STM32_PORT", "/dev/ttyUSB0")
ODOM_FRAME = os.environ.get("FPMS_ODOM_FRAME", "odom")
BASE_FRAME = os.environ.get("FPMS_BASE_FRAME", "base_footprint")
IMU_FRAME = os.environ.get("FPMS_IMU_FRAME", "imu_link")

ODOM_HZ = float(os.environ.get("FPMS_BRIDGE_ODOM_HZ", "20"))
IMU_HZ = float(os.environ.get("FPMS_BRIDGE_IMU_HZ", "25"))
SLOW_HZ = float(os.environ.get("FPMS_BRIDGE_SLOW_HZ", "2"))

# No /cmd_vel for this long -> zero the wire. A deadman, not a policy.
CMD_TIMEOUT_S = float(os.environ.get("FPMS_BRIDGE_CMD_TIMEOUT_S", "0.7"))

# Velocities above these are refused outright as nonsense/typos.
MAX_V = float(os.environ.get("FPMS_BRIDGE_MAX_V", "0.6"))
MAX_W = float(os.environ.get("FPMS_BRIDGE_MAX_W", "2.0"))

# Per-wheel duty above this is refused outright, the same way MAX_V/MAX_W
# refuse a nonsense velocity. 40 is comfortably above every value the golden
# program used (drive 17-26, turn 50 is deliberately NOT reachable by default)
# and far below the board's +/-100 full scale, so a typo cannot bolt the rover
# across the floor. Raise it with FPMS_BRIDGE_MAX_DUTY only deliberately.
MAX_DUTY = int(float(os.environ.get("FPMS_BRIDGE_MAX_DUTY", "40")))


def _truthy(v):
    return str(v).strip().lower() in ("1", "true", "yes", "on")


# ============================================================ MQTT TELEMETRY
# WHY THIS NODE ALSO SPEAKS MQTT
# ------------------------------
# The operator's dashboards are MQTT clients, not ROS ones. Everything this
# node reads off the board -- IMU, the four raw encoder counts, battery volts,
# and its own health -- existed ONLY as ROS topics, so none of it could ever
# reach a dashboard. The rest of this stack (fpms_teleop.py, fpms_cored.py)
# already publishes to Mosquitto with the idiom copied below; this is the same
# idiom, nothing new.
#
# THE CONTROL PATH MUST NOT BE ABLE TO NOTICE THIS EXISTS. Two guarantees:
#   1. connect_async + loop_start, exactly as fpms_teleop.py's Bus does. paho
#      owns a background network thread, retries a down broker forever, and
#      publish() only enqueues -- it never writes the socket on the caller's
#      thread.
#   2. The payload is assembled and published on a DEDICATED daemon thread, not
#      on a ROS timer. Even a pathologically slow paho could not then delay
#      /cmd_vel relay, the deadman, or the sensor timers, because it does not
#      share their executor.
# The publisher only ever READS cached values that the ROS timers already
# produced. It performs no serial I/O of its own, so it cannot add a single
# byte of traffic to a link this project has repeatedly found fragile.
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


def _cfg(key, default):
    """Environment wins over /etc/fpms/config.env wins over the default."""
    return os.environ.get(key, CFG.get(key, default))


THING = _cfg("FPMS_THING_NAME", "rover2")
MQTT_HOST = _cfg("FPMS_MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(float(_cfg("FPMS_MQTT_PORT", "1883")))
MQTT_USER = _cfg("FPMS_MQTT_USER", "")
MQTT_PASS = _cfg("FPMS_MQTT_PASS", "")

# 1 = publish board telemetry to MQTT (default). 0 = ROS only, exactly the
# behaviour before this existed.
MQTT_ENABLE = _truthy(_cfg("FPMS_BRIDGE_MQTT", "1"))
# Publish rate. Deliberately decoupled from every ROS rate: this is a dashboard
# heartbeat, not a control feed.
MQTT_HZ = float(_cfg("FPMS_BRIDGE_MQTT_HZ", "5"))
MQTT_TOPIC = "telemetry/board"

# TICKS -> MILLIMETRES, per wheel.
# 0.16657 mm/tick = 6.00 counts/mm -- the value fpms_phase6_M1_M2_WORKING.py
# (the program that actually completed M1 and M2 on this board) used, and which
# matches the 6.12 counts/mm measured on this chassis. It is NOT 14.8: that
# figure is 2.7x wrong and is the single cause behind this project's history of
# distance overshoot. Publishing raw counts alone was what let that error hide.
MM_PER_TICK = float(_cfg("FPMS_BRIDGE_MM_PER_TICK", "0.16657"))

# Window over which the published rates are measured. Long enough that the
# 2 Hz encoder feed is not quantised into nonsense (a 1 s window can only ever
# report 1.0 or 2.0 for it), short enough to notice a feed dying.
RATE_WINDOW_S = 4.0


class BoardBus:
    """MQTT publisher that never raises into the caller and reconnects forever.

    Lifted from fpms_teleop.py's Bus. connect_async rather than connect: a
    broker that is down at boot must delay the dashboards, not prevent this
    node -- which is the thing that stops the motors -- from starting.
    """

    def __init__(self, client_id, log):
        self.log = log
        self.connected = False
        self.published = 0
        self.errors = 0
        self.last_error = None
        self.client = mqtt.Client(client_id=client_id,
                                  callback_api_version=CallbackAPIVersion.VERSION2)
        if MQTT_USER:
            self.client.username_pw_set(MQTT_USER, MQTT_PASS or None)
        self.client.reconnect_delay_set(min_delay=1, max_delay=15)
        # A dashboard must be able to tell "the bridge stopped publishing"
        # from "the bridge is fine and the rover is parked". The will is the
        # only signal that survives this process being killed.
        self.client.will_set("fpms/%s/events/offline" % THING,
                             json.dumps({"thing": THING, "status": "offline",
                                         "svc": "stm32_bridge",
                                         "reason": "unexpected disconnect"}),
                             qos=1, retain=False)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect

    def start(self):
        try:
            self.client.connect_async(MQTT_HOST, MQTT_PORT, keepalive=30)
            self.client.loop_start()
            self.log("MQTT: connecting to %s:%d as %s (async, retries forever); "
                     "publishing fpms/%s/%s at %.1f Hz"
                     % (MQTT_HOST, MQTT_PORT, MQTT_USER or "<anonymous>",
                        THING, MQTT_TOPIC, MQTT_HZ))
        except Exception as e:
            self.log("MQTT: start failed (%s); node continues without MQTT" % e)

    def stop(self):
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass

    def _on_connect(self, _c, _u, _f, reason_code, _p=None):
        try:
            if getattr(reason_code, "is_failure", False):
                self.connected = False
                self.log("MQTT: connect refused (%s); will retry" % reason_code)
                return
            self.connected = True
            self.log("MQTT: connected to %s:%d (%s)"
                     % (MQTT_HOST, MQTT_PORT, reason_code))
            self.publish("events/online",
                         {"svc": "stm32_bridge", "status": "online",
                          "topic": "fpms/%s/%s" % (THING, MQTT_TOPIC),
                          "hz": MQTT_HZ, "mm_per_tick": MM_PER_TICK,
                          "port": PORT}, qos=1)
        except Exception as e:
            self.log("MQTT: on_connect error %s" % e)

    def _on_disconnect(self, _c, _u, *args):
        self.connected = False

    def publish(self, suffix, payload, qos=0):
        """Enqueue one message. Total: it swallows everything."""
        if not self.connected:
            # Dropped, not queued: a dashboard wants the CURRENT sample, and a
            # burst of stale ones on reconnect would be worse than a gap.
            return
        try:
            # allow_nan=False turns an accidental NaN into an exception here
            # rather than into invalid JSON on the dashboard's parser.
            body = json.dumps(payload, allow_nan=False)
        except Exception as e:
            self.errors += 1
            self.last_error = "payload not JSON-safe: %s" % e
            return
        try:
            self.client.publish("fpms/%s/%s" % (THING, suffix), body, qos=qos)
            self.published += 1
        except Exception as e:
            self.errors += 1
            self.last_error = "publish failed: %s" % e


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Stm32Bridge(Node):
    def __init__(self):
        super().__init__("fpms_stm32_bridge")

        # FPMS_BRIDGE_ARMED=1 forces armed and makes the node ignore the
        # executor's latch entirely. Default 0 = follow /fpms/mission/armed.
        self.force_armed = _truthy(os.environ.get("FPMS_BRIDGE_ARMED", "0"))
        self.armed = self.force_armed
        self._exec_armed = False
        self.estop = False

        self.bot = Rosmaster(com=PORT, debug=False)
        self.bot.create_receive_threading()
        time.sleep(1.0)

        # Hold a reference to the library's receive thread. It is a bare
        # `while True` with no try/except around a pyserial read that has no
        # timeout, so ONE SerialException kills it silently -- and every getter
        # is a plain cached-field read, so /odom_raw, /imu and /battery would go
        # right on publishing at 20/25/2 Hz with FROZEN values. If the frozen vx
        # is non-zero the bridge would integrate phantom distance forever and
        # every burst would end physically short. Liveness is checked in
        # _tick_slow; this is the single most dangerous failure this node has.
        self._rx_thread = getattr(self.bot, "_Rosmaster__receive_thread", None)
        self._link_dead = False

        try:
            self.fw = self.bot.get_version()
        except Exception:
            self.fw = None

        # Dead-reckoned pose, integrated from the board's own velocity report.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # MONOTONIC everywhere. time.time() is NTP-disciplined and this box
        # syncs after boot: a forward step of >=CMD_TIMEOUT_S would fire the
        # deadman mid-burst (a visible stutter), and any step corrupts dt.
        self._last_odom_t = time.monotonic()

        self._last_cmd_t = 0.0
        self._cmd = (0.0, 0.0)      # last /cmd_vel  (v, w)
        self._duty = [0, 0, 0, 0]   # last /cmd_duty (M1, M2, M3, M4)
        # LAST WRITER WINS: whichever of /cmd_vel and /cmd_duty arrived most
        # recently sets this, and ONLY that input's payload is re-asserted by
        # the deadman. A stale /cmd_vel publisher therefore cannot fight a live
        # /cmd_duty one (or the reverse) -- the older topic simply stops being
        # written the moment the newer one is received.
        self._mode = "vel"
        # Which path the BOARD last saw. Switching paths zeroes the abandoned
        # one first: set_car_motion leaves a velocity setpoint latched that the
        # board's own PID keeps chasing, so raw duty must not be layered on top
        # of a standing setpoint.
        self._wire_mode = "vel"
        self._lock = threading.Lock()
        self._warned_disarmed = 0.0

        # --- MQTT telemetry state -------------------------------------------
        # Snapshots of what the ROS timers last published, each stamped with the
        # MONOTONIC instant it was read off the board. The MQTT thread only ever
        # reads these, and each is replaced as a whole dict, so a reader can
        # never see a half-updated sample.
        self._boot_t = time.monotonic()
        self._last_imu = None
        self._last_enc = None
        self._last_volt = None
        self._n_odom = 0
        self._n_imu = 0
        self._n_ticks = 0
        self._n_mqtt = 0
        self._rate_t0 = time.monotonic()
        self._rates = None          # None until the first window closes
        self._mqtt_stop = threading.Event()
        self._mqtt_thread = None
        self.bus = None

        self.pub_odom = self.create_publisher(Odometry, "/odom_raw", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu", 10)
        self.pub_batt = self.create_publisher(BatteryState, "/battery", 10)
        self.pub_ticks = self.create_publisher(Int32MultiArray, "/wheel_ticks", 10)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        # Raw per-wheel duty: Int32MultiArray, exactly 4 elements,
        # [M1, M2, M3, M4] = [left, left, right, right], each |duty| <= MAX_DUTY.
        self.create_subscription(Int32MultiArray, "/cmd_duty",
                                 self._on_cmd_duty, 10)
        self.create_subscription(Bool, "/estop", self._on_estop, 10)
        self.create_subscription(Bool, "/bridge_arm", self._on_arm, 10)
        # The executor's own arm latch, mirrored by fpms_foxglove_cmd. Following
        # it is what makes the console's single ARM press open this gate too.
        self.create_subscription(Bool, "/fpms/mission/armed",
                                 self._on_exec_armed, 10)

        self.create_timer(1.0 / ODOM_HZ, self._tick_odom)
        self.create_timer(1.0 / IMU_HZ, self._tick_imu)
        self.create_timer(1.0 / SLOW_HZ, self._tick_slow)
        self.create_timer(0.1, self._tick_deadman)

        # Never ASSUME the wire is zero at boot. A crashed predecessor can leave
        # a non-zero setpoint latched on the board; booting disarmed and simply
        # believing it is stopped would leave the rover driving.
        self._stop_wire()

        self.get_logger().info(
            "fpms_stm32_bridge up: port=%s fw=%s frames=%s->%s  ARMED=%s "
            "max_duty=%d"
            % (PORT, self.fw, ODOM_FRAME, BASE_FRAME, self.armed, MAX_DUTY))
        if not self.armed:
            self.get_logger().warn(
                "DISARMED: /cmd_vel AND /cmd_duty are ignored, the wheels "
                "cannot turn. "
                "This gate now FOLLOWS the executor: press ARM on the console "
                "(or publish /fpms/mission/armed or /bridge_arm true).")

    # ---------------------------------------------------------------- inputs
    def _clear_estop_on_arm(self, source):
        if self.estop:
            self.estop = False
            self.get_logger().warn(
                "ESTOP latch CLEARED by ARM from %s -- arming is explicit "
                "operator consent to motion" % source)

    def _on_arm(self, msg):
        self.armed = bool(msg.data)
        self.get_logger().warn("ARM state changed by /bridge_arm -> armed=%s"
                               % self.armed)
        if self.armed:
            self._clear_estop_on_arm("/bridge_arm")
        else:
            self._stop_wire()

    def _on_exec_armed(self, msg):
        """Follow the mission executor's arm latch (mirrored by foxglove_cmd)."""
        want = bool(msg.data)
        if want == self._exec_armed:
            return
        self._exec_armed = want
        if self.force_armed:
            self.get_logger().info(
                "executor armed=%s (ignored: FPMS_BRIDGE_ARMED=1 forces armed)"
                % want)
            return
        self.armed = want
        self.get_logger().warn("ARM follows executor -> armed=%s" % want)
        if want:
            self._clear_estop_on_arm("the executor")
        else:
            self._stop_wire()

    def _on_estop(self, msg):
        if bool(msg.data):
            if not self.estop:
                self.get_logger().error("ESTOP latched -- zeroing the wire")
            self.estop = True
            self._stop_wire()
        else:
            if self.estop:
                self.get_logger().warn("ESTOP released")
            self.estop = False

    def _on_cmd_vel(self, msg):
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        if abs(v) > MAX_V or abs(w) > MAX_W:
            # A refusal must ZERO the wire, not coast on the last accepted
            # setpoint for up to CMD_TIMEOUT_S (126 mm at cruise) while the
            # operator reads "refusing" and assumes nothing is moving.
            self.get_logger().warn(
                "refusing out-of-range /cmd_vel v=%.3f w=%.3f -- zeroing" % (v, w))
            with self._lock:
                self._mode = "vel"
                self._cmd = (0.0, 0.0)
                self._last_cmd_t = time.monotonic()
            self._stop_wire()
            return
        with self._lock:
            self._mode = "vel"
            self._cmd = (v, w)
            self._last_cmd_t = time.monotonic()
        # Write the wire ON RECEIPT, not on the next 10 Hz tick. The executor
        # publishes at 20 Hz, so relaying only from the timer discarded half of
        # every burst's commands and delayed motion 0-100 ms (mean ~9 mm at
        # cruise), eating the burst-cap margin and halving the heading-hold rate.
        self._write_wire("vel", (v, w))

    def _refuse_duty(self, why):
        """Mirror of the /cmd_vel refusal: log, ZERO, and take the mode.

        Refusing must not leave the previously accepted duty coasting on the
        wire for up to CMD_TIMEOUT_S while the operator reads "refusing" and
        assumes nothing is moving -- the same trap _on_cmd_vel documents.
        """
        self.get_logger().warn("refusing /cmd_duty: %s -- zeroing" % why)
        with self._lock:
            self._mode = "duty"
            self._duty = [0, 0, 0, 0]
            self._last_cmd_t = time.monotonic()
        self._stop_wire()

    def _on_cmd_duty(self, msg):
        """Raw per-wheel duty -> bot.set_motor(M1, M2, M3, M4).

        Message: std_msgs/Int32MultiArray, data = exactly 4 ints,
        [M1, M2, M3, M4] = [left, left, right, right], each in
        [-MAX_DUTY, +MAX_DUTY]. Order and sign are passed through UNCHANGED,
        in the golden program's own order (see the module docstring).

        Same gates as /cmd_vel, no exceptions: it goes through _write_wire
        (arm gate + /estop latch + _link_dead latch) and is re-asserted by
        _tick_deadman, so a duty command that stops being published is zeroed
        after CMD_TIMEOUT_S just like a velocity one.
        """
        try:
            d = [int(x) for x in msg.data]
        except (TypeError, ValueError):
            self._refuse_duty("unparseable data %r" % (list(msg.data),))
            return
        if len(d) != 4:
            self._refuse_duty(
                "%d elements, need exactly 4 [M1,M2 left, M3,M4 right]" % len(d))
            return
        if any(abs(x) > MAX_DUTY for x in d):
            self._refuse_duty(
                "%s exceeds +/-%d (FPMS_BRIDGE_MAX_DUTY)" % (d, MAX_DUTY))
            return
        with self._lock:
            self._mode = "duty"
            self._duty = d
            self._last_cmd_t = time.monotonic()
        # Write ON RECEIPT for the same reason /cmd_vel does: relaying only
        # from the 10 Hz timer would discard half of a 20 Hz burst.
        self._write_wire("duty", d)

    # ----------------------------------------------------------------- wire
    def _write_wire(self, mode, cmd):
        """The ONE gated writer, shared by both inputs.

        mode "vel":  cmd = (v, w)              -> set_car_motion(v, 0, w)
        mode "duty": cmd = [m1, m2, m3, m4]    -> set_motor(m1, m2, m3, m4)

        Every gate is enforced here, so neither input can reach the board
        while disarmed, e-stopped or link-dead. Nothing else calls set_motor
        or set_car_motion except _stop_wire/_zero_path, which only send zeros.
        """
        if self.estop or not self.armed or self._link_dead:
            return
        if mode != self._wire_mode:
            self._zero_path(self._wire_mode)
            self._wire_mode = mode
        try:
            if mode == "duty":
                # Hard clamp as well as the refusal above: the wire itself must
                # be incapable of emitting more than MAX_DUTY, whatever state
                # got in by another route.
                c = [max(-MAX_DUTY, min(MAX_DUTY, int(x))) for x in cmd]
                self.bot.set_motor(c[0], c[1], c[2], c[3])
            else:
                self.bot.set_car_motion(float(cmd[0]), 0.0, float(cmd[1]))
        except Exception as e:
            self.get_logger().error("wire write (%s) failed: %s" % (mode, e))

    def _zero_path(self, mode):
        """Zero ONE path once. Only ever sends zeros."""
        try:
            if mode == "duty":
                self.bot.set_motor(0, 0, 0, 0)
            else:
                self.bot.set_car_motion(0.0, 0.0, 0.0)
        except Exception as e:
            self.get_logger().error("zeroing the %s path failed: %s" % (mode, e))

    def _stop_wire(self):
        """Zero the wire. Sent THREE times, unconditionally.

        The board never ACKs and the library only checksums on receive, so a
        single stop frame is not evidence the rover stopped. The previous code
        wrote one frame and then set a `_zeroed` flag that suppressed all
        further stops -- so one dropped frame left the rover cruising at
        0.18 m/s with this node convinced it was halted. The executor already
        sends its own zero three times for exactly this reason; matching it
        costs three 20-byte frames.

        BOTH PATHS, always. set_car_motion(0) does not cancel a standing raw
        duty and set_motor(0) does not cancel a standing velocity setpoint, so
        a stop that zeroed only the path we THINK is live would leave the other
        one driving. Stopping is never conditional on bookkeeping.
        """
        for _ in range(3):
            try:
                self.bot.set_car_motion(0.0, 0.0, 0.0)
            except Exception as e:
                self.get_logger().error("set_car_motion(0) failed: %s" % e)
            try:
                self.bot.set_motor(0, 0, 0, 0)
            except Exception as e:
                self.get_logger().error("set_motor(0) failed: %s" % e)

    def _tick_deadman(self):
        # Only the most recently received input is re-asserted. The other one's
        # last value is deliberately left un-written, so it cannot fight.
        with self._lock:
            mode = self._mode
            cmd = list(self._duty) if mode == "duty" else self._cmd
            age = time.monotonic() - self._last_cmd_t

        if self._link_dead:
            self._stop_wire()
            return

        nonzero = any(c for c in cmd)
        if self.estop or not self.armed:
            self._stop_wire()
            now = time.monotonic()
            if not self.armed and nonzero and now - self._warned_disarmed > 2.0:
                self._warned_disarmed = now
                self.get_logger().warn(
                    "/cmd_%s %s IGNORED -- bridge is DISARMED"
                    % ("duty" if mode == "duty" else "vel", list(cmd)))
            return

        if age > CMD_TIMEOUT_S:
            # DEADMAN: applies identically to /cmd_duty. A duty command must be
            # re-asserted faster than CMD_TIMEOUT_S or the wire is zeroed.
            self._stop_wire()
            return

        # Re-assert the CURRENT command every tick, including zero. Skipping the
        # write when we believe the wire is already zero is what made a single
        # lost stop frame unrecoverable.
        self._write_wire(mode, cmd)

    # ------------------------------------------------------------- publish
    def _tick_odom(self):
        now = time.monotonic()
        dt = now - self._last_odom_t
        self._last_odom_t = now
        if dt <= 0.0:
            return
        # CLAMP, do not discard. The old code committed the timestamp and then
        # returned on dt > 1.0, so a scheduling gap deleted that travel from
        # odometry permanently and silently -- 180 mm at cruise for a 1 s gap.
        # Integrating a clamped interval is wrong by at most the clamp; dropping
        # it is wrong by the whole gap and leaves no trace.
        dt_max = 3.0 / ODOM_HZ
        if dt > dt_max:
            self.get_logger().warn(
                "odom timer gap %.3fs (>%.3fs) -- clamping; %.0f mm may be "
                "unaccounted" % (dt, dt_max, (dt - dt_max) * 180.0))
            dt = dt_max
        if self._link_dead:
            return
        try:
            vx, vy, w = self.bot.get_motion_data()
        except Exception as e:
            self.get_logger().error("get_motion_data failed: %s" % e)
            return

        vx = float(vx); vy = float(vy); w = float(w)
        # Rotate the interval's velocity by the MIDPOINT yaw, then advance.
        # Using the end-of-interval yaw biases direction by w*dt/2.
        yaw_mid = self.yaw + 0.5 * w * dt
        self.yaw = math.atan2(math.sin(self.yaw + w * dt),
                              math.cos(self.yaw + w * dt))
        self.x += (vx * math.cos(yaw_mid) - vy * math.sin(yaw_mid)) * dt
        self.y += (vx * math.sin(yaw_mid) + vy * math.cos(yaw_mid)) * dt

        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = ODOM_FRAME
        m.child_frame_id = BASE_FRAME
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        # A REAL yaw, not the identity quaternion -- fpms_odom_tf's open
        # question M1 is answered "real" for this board.
        m.pose.pose.orientation = yaw_to_quat(self.yaw)
        m.twist.twist.linear.x = vx
        m.twist.twist.linear.y = vy
        m.twist.twist.angular.z = w
        self.pub_odom.publish(m)
        self._n_odom += 1

    def _tick_imu(self):
        # Publish NOTHING rather than a frozen value. A topic that stops is a
        # failure ROS handles correctly; a plausible frozen number is the one
        # this project has been fooled by repeatedly.
        if self._link_dead:
            return
        try:
            ax, ay, az = self.bot.get_accelerometer_data()
            gx, gy, gz = self.bot.get_gyroscope_data()
        except Exception as e:
            self.get_logger().error("imu read failed: %s" % e)
            return
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = IMU_FRAME
        m.linear_acceleration.x = float(ax)
        m.linear_acceleration.y = float(ay)
        m.linear_acceleration.z = float(az)
        m.angular_velocity.x = float(gx)
        m.angular_velocity.y = float(gy)
        m.angular_velocity.z = float(gz)
        # No absolute orientation is claimed: -1 in [0] is the ROS convention
        # for "this field is not produced", which is honest and keeps consumers
        # from trusting a yaw this node did not measure.
        m.orientation_covariance[0] = -1.0
        self.pub_imu.publish(m)
        self._n_imu += 1
        # Snapshot for the MQTT publisher. Whole-dict replacement, so the other
        # thread reads a consistent sample or the previous one, never a mix.
        self._last_imu = {"ax": float(ax), "ay": float(ay), "az": float(az),
                          "gx": float(gx), "gy": float(gy), "gz": float(gz),
                          "t": time.monotonic()}

    def _check_link(self):
        """Detect a silently dead receive thread.

        Every Rosmaster getter is a cached-field read, so if the library's
        receive thread dies this node keeps publishing perfect-looking frozen
        data forever. Latch the fault, zero the wire, and say so.
        """
        if self._rx_thread is None or self._link_dead:
            return
        if not self._rx_thread.is_alive():
            self._link_dead = True
            self.armed = False
            self.get_logger().error(
                "BOARD LINK DEAD: the Rosmaster receive thread has exited. "
                "Odometry/IMU/battery are now FROZEN and must not be trusted. "
                "Disarming and zeroing the wire. Restart fpms-stm32-bridge.")
            self._stop_wire()

    def _tick_slow(self):
        self._check_link()
        try:
            v = float(self.bot.get_battery_voltage())
        except Exception:
            v = float("nan")
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.voltage = v
        b.present = True
        self.pub_batt.publish(b)
        # NaN is a read failure, not a voltage. Kept out of the snapshot so it
        # can never reach the dashboard's JSON parser as a bare NaN token.
        self._last_volt = {"v": v, "t": time.monotonic()} if math.isfinite(v) else None

        try:
            enc = self.bot.get_motor_encoder()
            t = Int32MultiArray()
            t.data = [int(e) for e in enc]
            self.pub_ticks.publish(t)
            self._n_ticks += 1
            self._last_enc = {"ticks": list(t.data), "t": time.monotonic()}
        except Exception as e:
            self.get_logger().error("encoder read failed: %s" % e)


    # ---------------------------------------------------------- MQTT publish
    def _board_payload(self):
        """One consolidated telemetry sample. Pure: reads snapshots, no I/O.

        Every section carries its own `age_s` and the payload carries `fresh`.
        That is not decoration. This project has repeatedly been fooled by a
        feed that kept arriving with frozen values -- the Rosmaster receive
        thread dies silently and every getter is a cached-field read, so the
        numbers stay perfectly plausible forever. A consumer that trusts a
        value here without reading `fresh`/`age_s` is making exactly that
        mistake, so both are unmissable and the values are never omitted (a
        blank panel is indistinguishable from a parked rover).
        """
        now_m = time.monotonic()
        self._n_mqtt += 1

        dt = now_m - self._rate_t0
        if dt >= RATE_WINDOW_S:
            self._rates = {"odom": round(self._n_odom / dt, 2),
                           "imu": round(self._n_imu / dt, 2),
                           "ticks": round(self._n_ticks / dt, 2),
                           "mqtt": round(self._n_mqtt / dt, 2)}
            self._rate_t0 = now_m
            self._n_odom = self._n_imu = self._n_ticks = self._n_mqtt = 0

        dead = bool(self._link_dead)

        imu = self._last_imu
        if imu is None:
            imu_out = {"accel": None, "gyro": None, "age_s": None,
                       "units": {"accel": "m/s^2", "gyro": "rad/s"}}
        else:
            imu_out = {"accel": {"x": imu["ax"], "y": imu["ay"], "z": imu["az"]},
                       "gyro": {"x": imu["gx"], "y": imu["gy"], "z": imu["gz"]},
                       "age_s": round(now_m - imu["t"], 3),
                       "units": {"accel": "m/s^2", "gyro": "rad/s"}}

        enc = self._last_enc
        if enc is None:
            enc_out = {"ticks": None, "mm": None, "mm_per_tick": MM_PER_TICK,
                       "counts_per_mm": round(1.0 / MM_PER_TICK, 3),
                       "order": ["M1_left", "M2_left", "M3_right", "M4_right"],
                       "age_s": None}
        else:
            enc_out = {"ticks": list(enc["ticks"]),
                       "mm": [round(t * MM_PER_TICK, 2) for t in enc["ticks"]],
                       "mm_per_tick": MM_PER_TICK,
                       "counts_per_mm": round(1.0 / MM_PER_TICK, 3),
                       "order": ["M1_left", "M2_left", "M3_right", "M4_right"],
                       "age_s": round(now_m - enc["t"], 3)}

        volt = self._last_volt
        volt_out = None if volt is None else round(volt["v"], 3)
        volt_age = None if volt is None else round(now_m - volt["t"], 3)

        # Freshest thing we have actually read off the board. Once the link is
        # dead nothing advances it, so it is the age of the whole payload.
        ages = [a for a in (imu_out["age_s"], enc_out["age_s"], volt_age)
                if a is not None]
        data_age = min(ages) if ages else None

        payload = {
            "thing": THING,
            "ts": time.time(),
            "svc": "stm32_bridge",
            # Read these two before any number above them.
            "fresh": (not dead) and data_age is not None and data_age < 3.0,
            "link_dead": dead,
            "data_age_s": data_age,
            "imu": imu_out,
            "encoders": enc_out,
            "voltage": volt_out,
            "voltage_age_s": volt_age,
            "health": {
                "fw": self.fw,
                "armed": bool(self.armed),
                "estop": bool(self.estop),
                "link_dead": dead,
                "port": PORT,
                "rates_hz": self._rates,
                "target_rates_hz": {"odom": ODOM_HZ, "imu": IMU_HZ,
                                    "ticks": SLOW_HZ, "mqtt": MQTT_HZ},
                "uptime_s": round(now_m - self._boot_t, 1),
                "mqtt_published": self.bus.published if self.bus else 0,
                "mqtt_errors": self.bus.errors if self.bus else 0,
            },
        }
        if dead:
            payload["stale_reason"] = (
                "BOARD LINK DEAD: the Rosmaster receive thread has exited. "
                "Every imu/encoder/voltage value in this payload is FROZEN at "
                "its last-read value and must not be trusted. Restart "
                "fpms-stm32-bridge.")
        elif not payload["fresh"]:
            payload["stale_reason"] = (
                "No board sample in the last 3 s -- values below are stale.")
        return payload

    def _mqtt_run(self):
        """Publish on a fixed period, forever, on our own daemon thread.

        ON A TIMER, NOT ON CHANGE. A parked rover produces identical samples
        for minutes, and a feed that only speaks when something moves is
        indistinguishable from a dead one -- which is precisely how this stack
        has previously mistaken a crashed publisher for a stationary rover.
        Silence on this topic therefore means one thing: the process stopped.
        """
        period = 1.0 / MQTT_HZ if MQTT_HZ > 0 else 0.2
        nxt = time.monotonic()
        while not self._mqtt_stop.is_set():
            try:
                self.bus.publish(MQTT_TOPIC, self._board_payload())
            except Exception as e:
                # This thread must outlive anything it observes: if it dies the
                # dashboards go dark with no other symptom.
                self.get_logger().warn("MQTT board publish failed: %s" % e)
            nxt += period
            delay = nxt - time.monotonic()
            if delay < 0.0:
                nxt = time.monotonic()
                delay = 0.0
            if self._mqtt_stop.wait(delay):
                break

    def start_mqtt(self):
        if not MQTT_ENABLE:
            self.get_logger().warn(
                "MQTT board telemetry DISABLED (FPMS_BRIDGE_MQTT=0) -- the "
                "operator dashboards will show no board data.")
            return
        self.bus = BoardBus("%s-stm32-bridge" % THING,
                            lambda s: self.get_logger().info(s))
        self.bus.start()
        self._mqtt_thread = threading.Thread(target=self._mqtt_run, daemon=True,
                                             name="fpms-bridge-mqtt")
        self._mqtt_thread.start()

    def stop_mqtt(self):
        self._mqtt_stop.set()
        if self._mqtt_thread is not None:
            try:
                self._mqtt_thread.join(timeout=2.0)
            except Exception:
                pass
        if self.bus is not None:
            self.bus.stop()


def main():
    os.environ.setdefault("ROS_DOMAIN_ID", "20")
    rclpy.init()
    node = Stm32Bridge()
    # Started AFTER the node is fully constructed, so the publisher thread can
    # never observe a half-built node.
    node.start_mqtt()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException means the context is ALREADY down (this is
        # what systemd's SIGTERM produces). Calling rclpy.shutdown() again below
        # raised RCLError "rcl_shutdown already called", so every clean stop
        # exited 1 and looked like a crash in systemd. Guard on rclpy.ok().
        pass
    finally:
        try:
            node._stop_wire()
        except Exception:
            pass
        try:
            node.stop_mqtt()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
