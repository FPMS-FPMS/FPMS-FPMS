#!/usr/bin/env python3
"""
fpms_odom_tf.py - odometry + TF provider for Nav2 on the FPMS rover.

WHY THIS NODE EXISTS
====================
Nav2 requires two things the rover board does not give us directly:

  1. a TF tree with `odom` -> `base_footprint`, and
  2. a trustworthy `nav_msgs/Odometry` on `/odom`.

The Yahboom MicroROS board publishes `/odom_raw` and `/imu`, but neither is
usable as-is:

  * `/odom_raw` `twist.linear.x` is SIGN-INVERTED relative to its own
    `pose.position`. Measured on hardware (wheels off the ground, nothing else
    publishing):

        cmd +0.012  ->  twist mean -0.842,  pose displacement +1.395  (FORWARD)
        cmd +0.100  ->  twist mean -1.225,  pose displacement +3.505  (FORWARD)
        cmd -0.012  ->  twist mean +0.574,  pose displacement -1.506  (BACKWARD)

    The pose integrates in the direction the robot actually travels; the twist
    reports the opposite sign. TRUST POSE, NOT TWIST. We take the *magnitude*
    of travel from consecutive `pose.position` deltas and use the twist only to
    recover the sign, corrected exactly once by the single named constant
    ODOM_TWIST_SIGN below. There are deliberately no scattered minus signs
    anywhere else in this file. If the firmware is ever fixed, flipping that one
    value to +1 is the entire fix.

  * `/imu` orientation is NOT fused by the board firmware - the quaternion is
    identity (0, 0, 0, 1) on every message. Reading it and expecting a bearing
    yields a robot that believes it is permanently facing +x. Heading is
    therefore INTEGRATED from `angular_velocity.z` at ~25 Hz.

  * Rates: `/odom_raw` ~10-11 Hz, `/imu` ~25 Hz.

FUSION STRATEGY
===============
Position from wheel odometry (pose deltas), heading from integrated gyro Z.
This is exactly the combination the rover's own prior working code used, and it
measured 0.6% distance error and +/-1-4 degrees on turns. It is deliberately not
more clever than that.

The dominant error term in a gyro-integrated heading is the gyro Z BIAS, so it
is estimated while stationary (a settling window at startup, then continuously
re-estimated whenever commanded velocity is zero and wheel motion is ~0) and
subtracted before integration.

SAFETY POSTURE
==============
  * No callback is allowed to raise. Every incoming field is finite-checked; a
    single NaN reaching TF poisons the entire transform tree for every consumer.
  * If `/odom_raw` goes silent for more than ODOM_STALE_SEC, we STOP
    broadcasting TF and say so in the log. A frozen TF is strictly worse than no
    TF, because Nav2 will happily keep planning against a pose that is quietly
    no longer being updated.

COEXISTENCE WITH fpms_teleop.py
===============================
`fpms_teleop.py` already subscribes to `/imu` and maintains its own `yaw_int`
for its closed-loop turn command. That is fine - both are read-only consumers of
the same topic. The two headings are independent and will drift apart; neither
is authoritative over the other. Note that teleop's bias estimator gates on its
own internal `mode == "idle"` state, which this node cannot see, so the
stationary detector here is built from `/cmd_vel` plus observed wheel motion
instead.

MOTOR LAYOUT (per the operator - record for anyone doing direct motor control)
=============================================================================
    M1 = front-left     M3 = front-right
    M2 = back-left      M4 = back-right

This node drives nothing and consumes /cmd_vel only as an "are we commanded to
be stationary?" hint, so the layout does not affect the math here. It is
recorded because any future code that bypasses /cmd_vel and pokes motors
directly will need it, and it is otherwise undocumented outside the operator's
head.

This module is importable WITHOUT ROS installed (the rclpy imports are guarded)
so that the pure-math helpers can be unit-tested off-robot.
"""

import math
import os
import signal
import sys
import threading
import time

# ROS imports are guarded so the pure math can be imported and unit-tested on a
# machine with no ROS installation. deadband_sweep.py uses the same trick to
# keep its --dry-run path working off-robot.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Twist, TransformStamped
    from tf2_ros import TransformBroadcaster
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised only off-robot
    _ROS_AVAILABLE = False
    Node = object


# ============================================================================
# CHASSIS CONSTANTS
# Recovered from the rover's own prior working code. REUSE THESE - do not
# re-derive them, because the prior code's empirical numbers already absorb the
# real gear ratio and tyre behaviour that a clean derivation from wheel diameter
# would miss.
#
# Note that this node does not currently need any of them: the board integrates
# ticks into `pose.position` for us and never exposes raw tick counts (there is
# no /wheel_ticks topic on this hardware). They are recorded here so that a
# future node doing its own tick math has a single authoritative source.
# ============================================================================
WHEELBASE_M = 0.170          # 170 mm; wheelbase and track are equal on this chassis
TRACK_M = 0.170              # 170 mm
WHEEL_DIAMETER_M = 0.070     # 70 mm
TICKS_PER_REV = 1320         # encoder ticks per wheel revolution
MM_PER_TICK = 0.16657        # mm of ground travel per tick
M_PER_TICK = MM_PER_TICK / 1000.0

# ============================================================================
# THE ONE SIGN CORRECTION - applied in exactly one place, signed_travel().
# Matches the constant of the same name in fpms_teleop.py. Keep them in step.
# ============================================================================
ODOM_TWIST_SIGN = -1

# ============================================================================
# FRAMES
# ============================================================================
ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"
LASER_FRAME = "laser_frame"

# ============================================================================
# !! LIDAR MOUNT OFFSETS - THESE ARE PLACEHOLDERS AND ARE ALMOST CERTAINLY  !!
# !! WRONG. THEY MUST BE PHYSICALLY MEASURED ON THE ROBOT WITH A RULER -    !!
# !! NOT GUESSED, NOT COPIED FROM A CAD DRAWING, NOT LEFT AS THEY ARE.      !!
#
# base_footprint sits on the ground directly below the centre of the drive axes.
# These describe where the LiDAR's optical centre is relative to that point:
#   X forward (+) / backward (-), Y left (+) / right (-), Z up (+) from ground.
# LASER_YAW_RAD is the mount rotation: if the LiDAR's zero-degree ray does not
# point straight forward, every obstacle Nav2 sees is rotated by that error and
# the costmap is subtly, dangerously wrong.
#
# A 2 cm or 5 degree error here shows up as walls that refuse to line up during
# SLAM and as phantom obstacles during navigation. Measure it.
#
# Set PUBLISH_LASER_STATIC_TF=False if a URDF / robot_state_publisher is ever
# introduced, so this node does not fight it for ownership of the transform.
#
# THAT HAS NOW HAPPENED: rover/nav2/fpms_tf.launch.py introduces base_link and
# owns base_footprint -> base_link -> laser_frame (plus base_link ->
# imu_frame) as static transforms. If this flag were still True, that launch
# file and this node would both broadcast a transform ending in laser_frame -
# one via base_footprint directly, one via base_link - giving laser_frame two
# competing parents and an actively broken (non-tree) TF graph. See
# rover/nav2/TF_TREE.md for the expected tree and how to verify it.
# ============================================================================
PUBLISH_LASER_STATIC_TF = False
LASER_X_OFFSET_M = 0.0       # MEASURE ME
LASER_Y_OFFSET_M = 0.0       # MEASURE ME
LASER_Z_OFFSET_M = 0.10      # MEASURE ME
LASER_YAW_RAD = 0.0          # MEASURE ME

# ============================================================================
# TUNING
# ============================================================================
ODOM_STALE_SEC = 2.0         # no /odom_raw for this long -> stop TF and complain
WATCHDOG_HZ = 5.0            # how often staleness is checked

BIAS_SETTLE_SEC = 3.0        # startup window spent averaging gyro Z at rest
BIAS_MIN_SAMPLES = 20        # refuse to trust a bias built from fewer than this
BIAS_REEST_SEC = 1.5         # must be still this long before re-estimating
BIAS_EMA_ALPHA = 0.02        # slow blend for the running re-estimate
GYRO_STILL_RAD_S = 0.35      # |gyro| above this means we are definitely moving
CMD_STILL_EPS = 1e-3         # |cmd_vel| below this counts as a commanded stop
WHEEL_STILL_M = 0.002        # pose delta below this counts as wheels stopped
CMD_FRESH_SEC = 1.0          # a /cmd_vel older than this is evidence of nothing

MAX_PLAUSIBLE_STEP_M = 0.5   # a single pose delta larger than this is a glitch
MAX_PLAUSIBLE_DT = 0.5       # ignore integration gaps longer than this
MIN_PLAUSIBLE_DT = 1e-6

# ============================================================================
# COVARIANCES - deliberately pessimistic and honest.
#
# This node dead-reckons. There is no absolute reference anywhere in it, so true
# uncertainty grows without bound. Publishing small numbers would tell Nav2's
# filters that this pose deserves to dominate a scan match, which is precisely
# backwards. Heading in particular is integrated OPEN-LOOP from a rate gyro - it
# is the least trustworthy number emitted here and is labelled as such.
#
# Unmeasured degrees of freedom (z, roll, pitch) get a huge variance rather than
# zero. A zero would claim perfect knowledge of something never sampled.
# ============================================================================
POSE_XY_VAR = 0.05 ** 2      # ~5 cm 1-sigma, wheel odom on a reasonable floor
POSE_YAW_VAR = 0.20 ** 2     # ~11 deg 1-sigma, open-loop integrated gyro
UNMEASURED_VAR = 1e6         # z / roll / pitch: simply not known
TWIST_LINEAR_VAR = 0.10 ** 2
TWIST_ANGULAR_VAR = 0.15 ** 2


# ============================================================================
# LOGGING - matches the helper defined identically in fpms_teleop.py,
# fpms_rover_agent.py, deadband_sweep.py and fpms_cloud_uplink.py. systemd
# captures stdout into the journal, hence the unconditional flush.
# ============================================================================

def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


_LOG_LAST = {}


def log_every(key, interval, *a):
    """Rate-limited log, keyed by call site.

    Callbacks here run at 10-25 Hz. An unthrottled error log inside one of them
    turns a single recurring fault into a journal flood that buries everything
    else, so repeated messages are collapsed to one per `interval` seconds.
    """
    now = time.monotonic()
    if now - _LOG_LAST.get(key, -1e9) >= interval:
        _LOG_LAST[key] = now
        log(*a)


def _enforce_domain_id():
    """This rover lives on ROS_DOMAIN_ID=20 and nowhere else.

    Copied from deadband_sweep.py. Running on the wrong domain produces a node
    that starts cleanly, subscribes successfully and receives absolutely
    nothing - a failure mode that costs far more to diagnose than to prevent.
    """
    cur = os.environ.get("ROS_DOMAIN_ID")
    if cur is None:
        os.environ["ROS_DOMAIN_ID"] = "20"
        log("ROS_DOMAIN_ID was unset; defaulting it to 20 (mandatory for this rover)")
    elif cur != "20":
        raise SystemExit(
            "refuse: ROS_DOMAIN_ID=%r in this shell but this rover requires 20. "
            "Fix the environment and re-run." % (cur,))


# ============================================================================
# PURE MATH - no ROS types, no node state. Unit-testable off-robot.
# ============================================================================

def is_finite(*values):
    """True only if every argument is a real, finite number.

    The gate on every value that comes off the wire. inf and NaN both fail, and
    a non-numeric value fails rather than raising, because this runs inside
    callbacks that are never permitted to throw.
    """
    for v in values:
        try:
            f = float(v)
        except (TypeError, ValueError):
            return False
        if math.isnan(f) or math.isinf(f):
            return False
    return True


def wrap_to_pi(angle):
    """Wrap an angle into the CLOSED range [-pi, pi].

    Heading is integrated indefinitely, so without this it grows unbounded and
    eventually loses float precision. atan2(sin, cos) is used rather than a
    modulo because it is branch-free and cannot land a hair outside the range
    through rounding.

    The range is closed, not half-open: atan2 returns [-pi, pi], so an input of
    exactly -pi comes back as -pi rather than +pi. Both endpoints denote the
    same heading (pointing backwards along -x), so no consumer can tell them
    apart, and forcing one endpoint would mean adding a branch to a function
    whose whole appeal is not having one. Do not write a comparison anywhere
    that assumes the +pi endpoint is the canonical one.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_from_yaw(yaw):
    """Yaw (radians about Z) -> (x, y, z, w) unit quaternion.

    Flat-ground robot, so roll and pitch are identically zero and this reduces
    to a half-angle rotation about Z. Returned in ROS (x, y, z, w) order - note
    this is NOT the (w, x, y, z) order used by most maths texts.

    The repo had no yaw->quaternion helper before this one; fpms_teleop.py only
    ever needed the inverse.
    """
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def yaw_from_quat(x, y, z, w):
    """(x, y, z, w) -> yaw in radians. Inverse of quat_from_yaw for flat poses.

    Same formula as fpms_teleop.py's yaw_from_quat, unpacked into scalars so it
    can be tested without a ROS message type. Not used in the fusion path - the
    board's quaternion is identity always - but kept for diagnostics.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def signed_travel(dx, dy, twist_linear_x):
    """Distance travelled this step, signed for direction of travel.

    Magnitude comes from the POSE delta, which is the field the board integrates
    correctly. Direction comes from the twist, which is inverted - hence
    ODOM_TWIST_SIGN, applied here and nowhere else in this file.

    When the twist is essentially zero there is no directional evidence, so the
    sign stays positive. The magnitude is negligible at that point anyway, and
    guessing backwards would be worse than guessing forwards.
    """
    magnitude = math.hypot(dx, dy)
    if ODOM_TWIST_SIGN * twist_linear_x < 0.0:
        return -magnitude
    return magnitude


def remove_bias(raw_rate, bias):
    """Bias-corrected gyro rate. Trivial, but named so the intent is greppable."""
    return raw_rate - bias


def integrate_heading(theta, rate, dt):
    """Advance heading by a bias-corrected rate over dt, wrapped to (-pi, pi]."""
    return wrap_to_pi(theta + rate * dt)


def advance_pose(x, y, theta, distance):
    """Move `distance` along the CURRENT heading.

    Heading from the gyro, distance from the wheels - this one line is the
    actual fusion. The board's own reported heading is never consulted.
    """
    return (x + distance * math.cos(theta), y + distance * math.sin(theta))


def build_covariance(xy_var, yaw_var, unmeasured=UNMEASURED_VAR):
    """A 6x6 row-major covariance as a flat 36-list, for pose or twist.

    Only the x, y and yaw diagonals carry real numbers. z, roll and pitch get
    `unmeasured` because a planar robot never observes them; off-diagonal terms
    stay zero because the correlations are not tracked.
    """
    cov = [0.0] * 36
    cov[0] = float(xy_var)          # x
    cov[7] = float(xy_var)          # y
    cov[14] = float(unmeasured)     # z
    cov[21] = float(unmeasured)     # roll
    cov[28] = float(unmeasured)     # pitch
    cov[35] = float(yaw_var)        # yaw
    return cov


class GyroBiasEstimator(object):
    """Estimates the constant offset on the gyro's Z axis.

    A MEMS rate gyro reads a non-zero rate while perfectly still. Integrated
    over a few minutes, even 0.01 rad/s of bias becomes tens of degrees of
    heading error - the single largest source of drift in this node, far larger
    than wheel slip.

    Two phases:

      SETTLING - at startup the robot is assumed stationary and everything is
      averaged for `settle_sec`. Until `min_samples` have arrived the bias is
      not considered valid and heading integration is held off, so a garbage
      offset is never baked into the first metres of a run.

      RUNNING - thereafter, any time the robot is commanded to stop AND the
      wheels confirm it is not moving, samples fold into a slow EMA. This tracks
      the thermal drift that makes a one-shot startup calibration go stale over
      a long mission.

    A sample whose magnitude exceeds `still_limit` is discarded regardless of
    what anything else claims: that much rotation means the robot is moving, and
    whatever said otherwise was wrong.
    """

    def __init__(self, settle_sec=BIAS_SETTLE_SEC, min_samples=BIAS_MIN_SAMPLES,
                 alpha=BIAS_EMA_ALPHA, still_limit=GYRO_STILL_RAD_S):
        self.settle_sec = float(settle_sec)
        self.min_samples = int(min_samples)
        self.alpha = float(alpha)
        self.still_limit = float(still_limit)

        self.bias = 0.0
        self.valid = False
        self.settling = True
        self.samples = 0
        self.updates = 0
        self._start_time = None
        self._sum = 0.0

    def add_settling_sample(self, rate, now):
        """Feed a sample during the startup window. Returns True once settled."""
        if not is_finite(rate, now):
            return self.valid
        if self._start_time is None:
            self._start_time = now
        if abs(rate) <= self.still_limit:
            self._sum += rate
            self.samples += 1
        if (now - self._start_time) >= self.settle_sec and self.samples >= self.min_samples:
            self.bias = self._sum / float(self.samples)
            self.valid = True
            self.settling = False
        return self.valid

    def add_still_sample(self, rate):
        """Fold a confirmed-stationary sample into the running estimate."""
        if not is_finite(rate):
            return
        if abs(rate) > self.still_limit:
            return
        if not self.valid:
            # Not settled yet - treat it as more settling evidence.
            self._sum += rate
            self.samples += 1
            return
        self.bias = (1.0 - self.alpha) * self.bias + self.alpha * rate
        self.updates += 1

    def correct(self, rate):
        """Apply the current estimate. Returns 0.0 while the bias is untrusted."""
        if not is_finite(rate) or not self.valid:
            return 0.0
        return remove_bias(rate, self.bias)


# ============================================================================
# THE NODE
# ============================================================================

class FpmsOdomTf(Node):
    """Fuses /odom_raw position with /imu gyro heading -> /odom + TF."""

    def __init__(self):
        super().__init__("fpms_odom_tf")

        self.lock = threading.RLock()

        # Fused state, in the odom frame.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.bias_est = GyroBiasEstimator()

        # Last raw board pose, used to form deltas.
        self._last_raw_x = None
        self._last_raw_y = None

        # Timing is monotonic throughout, matching fpms_teleop.py. The wall
        # clock can step; monotonic cannot, and a backwards step would produce a
        # negative dt and a bogus rotation.
        self._last_imu_t = None
        self._last_odom_t = None
        self._last_cmd_t = None
        self._still_since = None
        self._start_t = time.monotonic()

        # Motion evidence.
        self._cmd_is_zero = True      # nothing commanded yet == stopped
        self._wheels_still = True
        self._last_linear_v = 0.0
        self._last_angular_v = 0.0

        self._tf_suppressed = False
        self._odom_msgs = 0
        self._imu_msgs = 0
        self._rejected = 0

        # QoS: RELIABLE / VOLATILE / KEEP_LAST(10), identical to the profile
        # fpms_teleop.py and deadband_sweep.py use. The board publishes
        # everything RELIABLE; a BEST_EFFORT subscription here would still match,
        # but matching the rest of the fleet keeps one profile to reason about.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(Odometry, "/odom_raw", self._on_odom_raw, qos)
        self.create_subscription(Imu, "/imu", self._on_imu, qos)
        # /cmd_vel feeds only the bias estimator: it says when the robot is
        # *supposed* to be still, which together with wheel evidence is when a
        # gyro sample is worth learning from.
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, qos)

        self.pub_odom = self.create_publisher(Odometry, "/odom", qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = None

        if PUBLISH_LASER_STATIC_TF:
            self._publish_laser_static_tf()

        self.create_timer(1.0 / WATCHDOG_HZ, self._watchdog)

        log("odom_tf node up: %s -> %s; position from /odom_raw pose deltas, "
            "heading from integrated /imu gyro z (ODOM_TWIST_SIGN=%d)"
            % (ODOM_FRAME, BASE_FRAME, ODOM_TWIST_SIGN))
        log("settling gyro bias for %.1fs - keep the robot COMPLETELY STILL"
            % BIAS_SETTLE_SEC)
        if PUBLISH_LASER_STATIC_TF:
            log("WARNING static %s -> %s uses PLACEHOLDER offsets "
                "x=%.3f y=%.3f z=%.3f yaw=%.3frad - MEASURE THESE ON THE "
                "PHYSICAL ROBOT before trusting any map built with them"
                % (BASE_FRAME, LASER_FRAME, LASER_X_OFFSET_M, LASER_Y_OFFSET_M,
                   LASER_Z_OFFSET_M, LASER_YAW_RAD))

    # ---------------------------------------------------------------- helpers

    def _stamp(self):
        """Header stamp for outgoing messages, from the Pi's ROS clock.

        Deliberately NOT `msg.header.stamp` off `/odom_raw`. The board's clock
        is not synchronised to the Pi, and a transform stamped in the board's
        time frame lands outside tf2's buffer window - Nav2 then rejects every
        lookup with an extrapolation error that looks nothing like a clock
        problem. Everything published here is stamped locally.
        """
        return self.get_clock().now().to_msg()

    def _publish_laser_static_tf(self):
        """One-shot static base_footprint -> laser_frame."""
        try:
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self._stamp()
            t.header.frame_id = BASE_FRAME
            t.child_frame_id = LASER_FRAME
            t.transform.translation.x = float(LASER_X_OFFSET_M)
            t.transform.translation.y = float(LASER_Y_OFFSET_M)
            t.transform.translation.z = float(LASER_Z_OFFSET_M)
            qx, qy, qz, qw = quat_from_yaw(float(LASER_YAW_RAD))
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.static_tf_broadcaster.sendTransform(t)
        except Exception as e:
            # A missing laser transform is survivable; a dead node is not.
            log("static laser tf error %s" % (e,))

    def _mark_still_or_moving_unlocked(self):
        """Track how long the robot has been continuously stationary."""
        now = time.monotonic()
        cmd_fresh = (self._last_cmd_t is not None
                     and (now - self._last_cmd_t) < CMD_FRESH_SEC)
        if cmd_fresh:
            commanded_still = self._cmd_is_zero
        else:
            # Once a /cmd_vel has been seen, its absence means "unknown", which
            # is treated as moving. Before the first one ever arrives, nothing
            # is driving the robot, so stillness is a safe assumption.
            commanded_still = (self._last_cmd_t is None)
        if commanded_still and self._wheels_still:
            if self._still_since is None:
                self._still_since = now
        else:
            self._still_since = None

    def _is_confirmed_still_unlocked(self):
        if self._still_since is None:
            return False
        return (time.monotonic() - self._still_since) >= BIAS_REEST_SEC

    # -------------------------------------------------------------- callbacks

    def _on_cmd_vel(self, msg):
        """Record whether motion is being commanded. Never raises."""
        try:
            lx = msg.linear.x
            az = msg.angular.z
            if not is_finite(lx, az):
                return
            with self.lock:
                self._cmd_is_zero = (abs(lx) < CMD_STILL_EPS and abs(az) < CMD_STILL_EPS)
                self._last_cmd_t = time.monotonic()
                self._mark_still_or_moving_unlocked()
        except Exception as e:
            log_every("cmd_cb", 5.0, "cmd_vel callback error %s" % (e,))

    def _on_imu(self, msg):
        """Integrate heading from gyro Z. Never raises.

        msg.orientation is deliberately ignored: the board publishes an identity
        quaternion on every message because it performs no fusion, so reading it
        would peg the heading at zero forever.
        """
        try:
            raw = msg.angular_velocity.z
            if not is_finite(raw):
                with self.lock:
                    self._rejected += 1
                return
            raw = float(raw)
            now = time.monotonic()

            with self.lock:
                self._imu_msgs += 1

                # Phase 1: startup settling window.
                if self.bias_est.settling:
                    became_valid = self.bias_est.add_settling_sample(raw, now)
                    self._last_imu_t = now
                    if became_valid:
                        log("gyro z bias settled: %.6f rad/s (%.4f deg/s) "
                            "from %d samples"
                            % (self.bias_est.bias, math.degrees(self.bias_est.bias),
                               self.bias_est.samples))
                    return

                # Phase 2: keep learning whenever genuinely stopped.
                if self._is_confirmed_still_unlocked():
                    self.bias_est.add_still_sample(raw)

                if self._last_imu_t is None:
                    self._last_imu_t = now
                    return

                dt = now - self._last_imu_t
                self._last_imu_t = now
                if dt <= MIN_PLAUSIBLE_DT or dt > MAX_PLAUSIBLE_DT:
                    # A large dt means starvation or a stalled link resuming.
                    # Integrating across it injects a bogus rotation.
                    return

                corrected = self.bias_est.correct(raw)
                if not is_finite(corrected):
                    self._rejected += 1
                    return

                new_theta = integrate_heading(self.theta, corrected, dt)
                if not is_finite(new_theta):
                    self._rejected += 1
                    return
                self.theta = new_theta
                self._last_angular_v = corrected
        except Exception as e:
            log_every("imu_cb", 5.0, "imu callback error %s" % (e,))

    def _on_odom_raw(self, msg):
        """Take translation from board pose deltas, then publish /odom + TF."""
        try:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            tx = msg.twist.twist.linear.x

            if not is_finite(px, py):
                with self.lock:
                    self._rejected += 1
                log_every("odom_nan", 5.0, "non-finite /odom_raw pose, dropped")
                return
            if not is_finite(tx):
                tx = 0.0  # lose the direction hint, keep the magnitude
            px, py, tx = float(px), float(py), float(tx)

            now = time.monotonic()

            with self.lock:
                self._odom_msgs += 1
                self._last_odom_t = now

                if self._last_raw_x is None:
                    # The first message only establishes the reference point.
                    self._last_raw_x = px
                    self._last_raw_y = py
                    return

                dx = px - self._last_raw_x
                dy = py - self._last_raw_y
                step = math.hypot(dx, dy)

                if step > MAX_PLAUSIBLE_STEP_M:
                    # Half a metre in one 10 Hz tick is not physically possible
                    # for this chassis - it is an encoder or transport glitch.
                    # Re-anchor to the new value so the jump is not replayed
                    # forever, but do not integrate it.
                    log_every("odom_jump", 5.0,
                              "implausible /odom_raw jump %.3fm in one step - "
                              "re-anchoring, not integrating" % step)
                    self._last_raw_x = px
                    self._last_raw_y = py
                    self._rejected += 1
                    return

                distance = signed_travel(dx, dy, tx)
                self._last_raw_x = px
                self._last_raw_y = py
                self._wheels_still = (step < WHEEL_STILL_M)
                self._mark_still_or_moving_unlocked()

                # THE FUSION: distance from the wheels, direction from the gyro.
                nx, ny = advance_pose(self.x, self.y, self.theta, distance)
                if not is_finite(nx, ny):
                    self._rejected += 1
                    log_every("fuse_nan", 5.0, "fused pose went non-finite, dropped")
                    return
                self.x = nx
                self.y = ny

                # Report a corrected linear velocity so downstream consumers
                # never see the board's inverted twist.
                self._last_linear_v = ODOM_TWIST_SIGN * tx

                if self._tf_suppressed:
                    log("/odom_raw recovered - resuming %s -> %s tf broadcast"
                        % (ODOM_FRAME, BASE_FRAME))
                    self._tf_suppressed = False

                x, y, theta = self.x, self.y, self.theta
                lin, ang = self._last_linear_v, self._last_angular_v
                bias_ok = self.bias_est.valid

            self._publish(x, y, theta, lin, ang, bias_ok)
        except Exception as e:
            log_every("odom_cb", 5.0, "odom_raw callback error %s" % (e,))

    # ------------------------------------------------------------- publishing

    def _publish(self, x, y, theta, lin, ang, bias_ok):
        """Emit /odom and the odom -> base_footprint transform."""
        try:
            qx, qy, qz, qw = quat_from_yaw(theta)
            if not is_finite(x, y, qz, qw):
                log_every("pub_nan", 5.0, "refusing to publish non-finite pose")
                return

            stamp = self._stamp()

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = ODOM_FRAME
            odom.child_frame_id = BASE_FRAME
            odom.pose.pose.position.x = float(x)
            odom.pose.pose.position.y = float(y)
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw

            # Before the bias settles the heading is not merely uncertain, it is
            # unmodelled. Say so, rather than quietly implying otherwise.
            yaw_var = POSE_YAW_VAR if bias_ok else UNMEASURED_VAR
            odom.pose.covariance = build_covariance(POSE_XY_VAR, yaw_var)

            odom.twist.twist.linear.x = float(lin)
            odom.twist.twist.angular.z = float(ang)
            odom.twist.covariance = build_covariance(TWIST_LINEAR_VAR, TWIST_ANGULAR_VAR)

            self.pub_odom.publish(odom)

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = ODOM_FRAME
            t.child_frame_id = BASE_FRAME
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            log_every("publish", 5.0, "publish error %s" % (e,))

    # --------------------------------------------------------------- watchdog

    def _watchdog(self):
        """Stop broadcasting TF if the board goes quiet. Never raises.

        The last transform is NOT re-sent with a fresh timestamp. That would
        look healthy to Nav2 while being a lie, and Nav2 would plan and drive
        against a pose that stopped updating. Silence makes the failure loud:
        tf lookups start failing and Nav2 refuses to move, which is the correct
        response to no longer knowing where the robot is.
        """
        try:
            now = time.monotonic()
            with self.lock:
                last = self._last_odom_t
                suppressed = self._tf_suppressed
                odom_n, imu_n, rej = self._odom_msgs, self._imu_msgs, self._rejected

                if last is None:
                    if (now - self._start_t) > ODOM_STALE_SEC and not suppressed:
                        self._tf_suppressed = True
                        log("ERROR no /odom_raw since startup (imu msgs=%d) - is "
                            "micro-ros-agent up? NOT broadcasting %s -> %s"
                            % (imu_n, ODOM_FRAME, BASE_FRAME))
                    return

                age = now - last
                if age > ODOM_STALE_SEC and not suppressed:
                    self._tf_suppressed = True
                    self._last_linear_v = 0.0
                    log("ERROR /odom_raw stale for %.1fs - SUSPENDING %s -> %s tf. "
                        "A frozen transform would let Nav2 plan against a pose "
                        "that is no longer updating. (odom=%d imu=%d rejected=%d)"
                        % (age, ODOM_FRAME, BASE_FRAME, odom_n, imu_n, rej))
        except Exception as e:
            log_every("watchdog", 5.0, "watchdog error %s" % (e,))


STOP_FLAG = threading.Event()


def main():
    if not _ROS_AVAILABLE:
        sys.stderr.write(
            "fpms_odom_tf: rclpy / ROS 2 message packages not importable.\n"
            "Source the ROS environment first: source /opt/ros/humble/setup.bash\n")
        return 1

    signal.signal(signal.SIGTERM, lambda *_: STOP_FLAG.set())
    signal.signal(signal.SIGINT, lambda *_: STOP_FLAG.set())

    _enforce_domain_id()

    node = None
    try:
        rclpy.init(args=None)
        node = FpmsOdomTf()
        # Hand-rolled spin, matching fpms_teleop.py: a raised exception inside
        # rclpy.spin() would tear the process down, whereas here one bad
        # iteration is logged and the node keeps running.
        while not STOP_FLAG.is_set():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except Exception as e:
                log_every("spin", 5.0, "spin error %s" % (e,))
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        log("fatal %s" % (e,))
        return 1
    finally:
        log("odom_tf shutting down")
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
