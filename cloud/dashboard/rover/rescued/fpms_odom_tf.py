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
    reports the opposite sign. TRUST POSE, NOT TWIST - and note that "trust
    pose" has to mean the SIGN as well as the magnitude, which an earlier
    version of this file did not do: it took the magnitude from the pose delta
    and the direction from the twist. That is a twist read in front of a
    direction decision, which is the exact shape of the mistake this project
    lost hours to twice. The sign now comes from the pose too, by PROJECTING
    the pose delta onto the board's own reported heading, the way
    fpms_teleop.py's `_on_odom` and `summarize_leg` already do:

        along = dx*cos(board_yaw) + dy*sin(board_yaw)

    Both fields are from the SAME message, so the projection is internally
    consistent no matter what the board's frame is doing. Twist is retained
    only as a corroborating witness with no authority: it is compared against
    the pose verdict and disagreements are COUNTED and logged, because that
    counter is the cheapest test that would catch the firmware being fixed (or
    breaking differently) without anybody re-running a bench sweep. If the
    projection is genuinely ambiguous, twist is consulted as a last resort and
    the event is counted separately - see `travel_sign()`.

    ODOM_TWIST_SIGN survives as the single point of correction for that last
    resort and for the velocity republished on /odom. If the firmware is ever
    fixed, flipping that one value to +1 is still the entire fix.

  * `/imu` orientation is NOT fused by the board firmware - the quaternion is
    identity (0, 0, 0, 1) on every message. Reading it and expecting a bearing
    yields a robot that believes it is permanently facing +x. Heading is
    therefore INTEGRATED from `angular_velocity.z` at ~25 Hz.

  * `/odom_raw`'s OWN `pose.pose.orientation` is a separate question from
    `/imu`'s and the repo currently contradicts itself about it:
    fpms_teleop.py's `_on_odom` says "the board publishes an identity
    orientation", while its `summarize_leg` says "the board does not fuse
    orientation, so odom yaw is dead-reckoned from the wheels" - i.e. that it is
    a real, encoder-derived yaw. Those cannot both be true and NOBODY HAS
    CHECKED WHICH. This node therefore checks at runtime, says which it found in
    the log exactly once, and is correct either way (see `travel_sign()` and
    `_note_board_yaw_unlocked()`). Resolving this is prerequisite #1 for the
    complementary filter described below.

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

WHICH HEADING IS TRUSTED, AND WHY - THE GYRO, UNCONDITIONALLY, FOR NOW
======================================================================
There are two candidate headings on this rover and they fail in different ways:

  GYRO Z, integrated      drifts without bound at the residual bias rate.
                          Has NO slip mechanism at all - a rate gyro measures
                          the body's actual rotation whatever the wheels do.

  ENCODER / WHEEL YAW     bounded error per turn, no unbounded drift, but on
  (the board's own        THIS chassis it is systematically wrong by an
  pose.pose.orientation,  UNMEASURED SCALE FACTOR, and the error is large.
  if it is real at all)

The scale factor is the reason the gyro wins outright today, and it is a
property of the chassis rather than of the firmware. This is a 4WD SKID-STEER
rover (NAV2_BRIEF.md section 2): it has no steering, so every degree of yaw is
produced by dragging four wheels sideways across the floor. The yaw a wheel
encoder infers is (v_right - v_left) / TRACK, and for a skid-steer machine the
value of TRACK that makes that equation true is NOT the geometric track
(170 mm) - it is an "effective track" that is larger by a slip-dependent factor,
commonly 1.2-1.8x on hard floors and different again on carpet. Nobody has
measured it for this rover, so an encoder heading here carries an unknown
multiplicative error of tens of percent on every turn. A gyro bias of even
0.01 rad/s takes about a minute to do that much damage, and this rover's
missions are bounded at MISSION_TIMEOUT_S = 240 s with stops throughout that
re-estimate the bias.

The empirical half of the argument agrees: the prior working autonomy
(/home/ubuntu/fpms_phase6_LATEST.py) used the integrated gyro for heading, and
measured +/-1-4 degrees per turn. Nothing has ever measured this chassis'
encoder heading at all.

So the rule this node implements:

    HEADING IS THE GYRO. The encoder yaw is a WITNESS, not a source. It is
    differenced against the gyro over the same interval and the ratio and
    residual are reported - because those two numbers are exactly what a
    complementary filter would need in order to be defensible, and neither
    exists yet.

WHY THE COMPLEMENTARY FILTER IS WRITTEN, DISABLED, AND PROBABLY NOT THE ANSWER
==============================================================================
`complementary_heading()` below implements the standard form

    theta <- (1 - a) * (theta + omega*dt) + a * theta_reference,  a = dt/(tau+dt)

a first-order crossover: below `tau` the gyro dominates, above it the reference
does. There are TWO independent reasons it is off, and the second one is the
interesting one.

REASON 1 - `tau` has no defensible value yet. It is not a free parameter to be
set to a nice-looking number. The crossover belongs where the two error sources
are equal in size:

    tau = sigma_reference / bias_residual                       [seconds]

  sigma_reference  1-sigma heading error of the reference, radians.
  bias_residual    1-sigma of the gyro bias that SURVIVES the estimator here,
                   rad/s - not the raw datasheet bias, the leftover after
                   `GyroBiasEstimator` has done its job.

Read it as "how long does the gyro's leftover bias take to do as much damage as
the reference has already done?", and weight them equally there. Neither number
is known for this rover (M2, M3 below), so any value written today would be a
guess wearing the costume of a measurement - which is the exact failure this
project has already paid for twice.

REASON 2 - A COMPLEMENTARY FILTER NEEDS A BOUNDED REFERENCE, AND THIS ROVER HAS
NONE. The filter's premise is that the reference has bounded error and no drift,
so that the blend inherits the gyro's short-term smoothness and the reference's
long-term stability. The encoder yaw does not satisfy that. It is not an
absolute heading; it is a SECOND DEAD-RECKONED heading, integrated from wheel
differences, and its error accumulates with total rotation exactly as the gyro's
accumulates with time. Blending two unbounded signals does not bound anything -
it only produces a mixture whose drift is a weighted sum of two drifts. It can
look better on a short bench run and cannot be better over a mission.

So the encoder yaw is not the reference this filter wants, and no value of `tau`
makes it one. Wiring it up would be tuning a mechanism that cannot deliver what
it is being asked for.

THE ACTUAL ABSOLUTE HEADING SOURCE IS THE LIDAR. A rectangular arena's walls
give heading modulo 90 degrees with no drift term at all - see the LiDAR section
below and `heading_from_manhattan()`. That is a genuinely bounded reference, and
it is the thing worth spending the effort on. `complementary_heading()` is kept
because it is the correct blend to use ONCE such a reference exists, whether it
comes from the LiDAR fit or from a board firmware that starts fusing
orientation.

`HeadingCrossCheck` below is therefore not a step towards enabling the filter.
It exists to MEASURE the encoder heading - the skid-steer effective-track factor
in particular - so that the claim above is a number in a log rather than an
argument in a comment, and so the next person can disagree with evidence.

LIDAR-BASED ABSOLUTE CORRECTION - PURE MATH SHIPPED, WIRING OFF
================================================================
Dead reckoning has no absolute reference, so its error grows without bound and
no amount of filtering fixes that. The arena is a KNOWN 1200 x 1200 mm box
(frontend/src/lib/arena.ts is the source of truth, mirrored in
fpms_missions.py), which is a very strong prior: in a rectangular room a single
360-degree scan pins all three degrees of freedom outright, with no map server,
no AMCL and no particle filter.

The maths for that is implemented here as pure functions - `manhattan_offset()`,
`wall_fit()`, `arena_fix_from_scan()` - and is exercised off-robot by
`--selftest`, which builds synthetic scans of a known box at a known pose and
checks that the fit recovers it. What is NOT enabled is the wiring that would
let a fix move the published pose (LIDAR_FIX_ENABLED = False), because every
remaining prerequisite is a physical measurement nobody has taken. They are
listed under MEASUREMENTS REQUIRED and enforced by `lidar_fix_blockers()`,
which returns the list of reasons the correction is refused so that the refusal
names its cause instead of being a silent False.

MEASUREMENTS REQUIRED - NOTHING BELOW HAS BEEN MEASURED ON THIS ROVER
=====================================================================
Every item here gates something this file already contains. None of them can be
obtained from the repo; all of them need the rover powered, on the floor, with
one agent connected to it. Until then the affected feature stays off and says
so at startup rather than running on a guess.

  M1  Is `/odom_raw`'s pose.pose.orientation a REAL yaw or the identity
      quaternion? One `ros2 topic echo /odom_raw --once` after driving a turn
      settles it. Gates: the encoder-heading witness, and therefore M2 and M3.
      This node logs which it observed, once, at INFO.

  M2  The skid-steer effective-track factor. Drive the calibration pattern
      (four 90-degree turns back to the start heading, repeated 5x) and read
      the `scale` from the 30 s report line - it is encoder rotation over gyro
      rotation. It is NOT expected to be 1.0, and how far it sits from 1.0 is
      the size of the argument for ignoring the encoder yaw. Turns nothing on;
      it converts the heading-source argument above from prose into a number.

  M3  bias_residual - the gyro bias left after `GyroBiasEstimator`. Park the
      rover, let the bias settle, then leave it parked for 10 minutes and read
      the total heading change. bias_residual = |dtheta| / 600 s. This is the
      number that says how long dead reckoning stays usable, and it is what
      POSE_YAW_VAR should be derived from instead of the guess it is now.

  M4  The LiDAR mount transform - LASER_X/Y/Z_OFFSET_M and above all
      LASER_YAW_RAD, which live in rover/nav2/fpms_tf.launch.py, NOT here (see
      the frames block below). These are PLACEHOLDERS. A mount-yaw error of e
      makes every LiDAR heading fix wrong by exactly e, with no symptom other
      than a heading that is confidently wrong. Gates: LIDAR_FIX_ENABLED.

  M5  fpms_lidar_ros.py's LIDAR_ZERO_OFFSET_DEG and LIDAR_ROTATION_SIGN, which
      are marked UNVERIFIED in that file and have a wall test written out step
      by step in its header. A wrong sign MIRRORS the scan - and a mirrored
      scan of a SQUARE arena still fits a square perfectly, so the wall fit
      below returns a confident, wrong, REFLECTED pose. This is the single most
      dangerous unmeasured item on the list because it is the one that fails
      silently. `--selftest` demonstrates it rather than asserting it, and
      pins down the blind spot exactly: the reflection is about the arena
      centre line, so the error is 2*|coord - 600 mm| and the sanity check
      against dead reckoning only catches it beyond LIDAR_MAX_JUMP_M. Within
      150 mm of the centre line a mirrored scan is ACCEPTED with up to 300 mm
      of error and no complaint. Gates: LIDAR_FIX_ENABLED.

  M6  Does the arena have PHYSICAL WALLS the scanner can see, at the scanner's
      mounting height? If the 1200 x 1200 boundary is tape on the floor, or the
      wall is shorter than the LiDAR is tall, there is nothing to fit and the
      entire correction design is void. Nobody has recorded this either way.
      Gates: LIDAR_FIX_ENABLED, and gates whether M4/M5 are worth taking.

  M7  The D500's range bias. `wall_fit()` returns `span_error_m` - opposite
      walls must fit a separation of 1.200 m, so the error in that separation
      is an indicator of range bias, measured for free every time a fix is
      computed. It is NOT exactly twice the bias: `--selftest` measures
      0.72-0.86 of 2b, because a ray meeting a wall at angle alpha to its
      normal projects only b*cos(alpha) of its bias onto that wall's axis. So
      it is reported, never used to correct anything, until somebody has
      characterised it against a target at a known distance.

The arena ZONE rectangles are deliberately NOT used by the fit. They are
painted floor regions in arena.ts, not obstacles, and there is no evidence
anywhere in this repo that they present anything for a scanner to range
against. Fitting to them would be inventing geometry.

SAFETY POSTURE
==============
  * No callback is allowed to raise. Every incoming field is finite-checked; a
    single NaN reaching TF poisons the entire transform tree for every consumer.
  * If `/odom_raw` goes silent for more than ODOM_STALE_SEC, we STOP
    broadcasting TF and say so in the log. A frozen TF is strictly worse than no
    TF, because Nav2 will happily keep planning against a pose that is quietly
    no longer being updated.

  * When `/odom_raw` COMES BACK, the first message re-anchors without
    integrating. The board's pose is cumulative, so that message carries every
    centimetre travelled during the outage in a single delta, and this node has
    no record of what the heading was doing across it. Integrating that lump
    along the current heading would draw a straight line where an unknown path
    was - and it would do it at precisely the moment the operator has just been
    told the transform was suspended, i.e. when they are least likely to be
    watching for a second, quieter fault.

  * The heading integrator never starts on an uncalibrated bias, but neither is
    it allowed to never start. See BIAS_SETTLE_DEADLINE_SEC.

COEXISTENCE WITH fpms_teleop.py
===============================
`fpms_teleop.py` already subscribes to `/imu` and maintains its own `yaw_int`
for its closed-loop turn command. That is fine - both are read-only consumers of
the same topic. The two headings are independent and will drift apart; neither
is authoritative over the other. Note that teleop's bias estimator gates on its
own internal `mode == "idle"` state, which this node cannot see, so the
stationary detector here is built from `/cmd_vel` plus observed wheel motion
instead.

That difference has a consequence worth knowing before somebody "fixes" the
stationary detector to be more eager. BIAS_REEST_SEC (1.5 s) is longer than
fpms_missions.py's longest post-motion settle (TURN_SETTLE_S = 1.2 s), so a
mission's between-segment stops do NOT feed the bias estimator. That is
deliberate: during those stops the chassis is still coasting through the tail of
a commanded turn, and a real rotation learned as bias would then be subtracted
from every subsequent turn. Re-estimation happens between MISSIONS, not between
segments. Shortening BIAS_REEST_SEC to "get more samples" would trade a correct
estimator for a busier one.

This node's stationary detector also depends on somebody publishing `/cmd_vel`
zeros while idle - teleop's 2 Hz idle heartbeat is what normally supplies them.
fpms_missions.py's docstring requires that heartbeat to be SUPPRESSED while a
mission owns the wire. When it is, `_last_cmd_t` goes stale between segments and
`_mark_still_or_moving_unlocked` correctly reports "unknown, assume moving". No
bias is learned during a mission, which is the safe direction to be wrong in.

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
so that the pure-math helpers can be unit-tested off-robot:

    python3 fpms_odom_tf.py --selftest

That covers the sign resolution, the heading helpers, the bias estimator's
failure paths and the whole LiDAR arena fit against synthetic scans. It proves
the maths. It proves NOTHING about this rover's physical constants, and it
prints the list of blockers at the end so that distinction cannot be mislaid.
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
# The board integrates ticks into `pose.position` for us and never exposes raw
# tick counts (there is no /wheel_ticks topic on this hardware), so most of
# these are recorded for a future node rather than used here. M_PER_TICK is the
# exception - it is the resolution of the only position signal this node has,
# and POSE_NOISE_FLOOR_M is derived from it below.
#
# TRACK_M is the GEOMETRIC track. It is NOT the effective track that would make
# an encoder-derived yaw correct on this skid-steer chassis - see the heading
# discussion in the module docstring. Do not use it to compute a yaw.
# ============================================================================
WHEELBASE_M = 0.170          # 170 mm; wheelbase and track are equal on this chassis
TRACK_M = 0.170              # 170 mm
WHEEL_DIAMETER_M = 0.070     # 70 mm
TICKS_PER_REV = 1320         # encoder ticks per wheel revolution
MM_PER_TICK = 0.16657        # mm of ground travel per tick
M_PER_TICK = MM_PER_TICK / 1000.0

# ============================================================================
# THE ONE SIGN CORRECTION.
# Matches the constant of the same name in fpms_teleop.py and deadband_sweep.py.
# Keep all three in step.
#
# It is applied at exactly two places, and BOTH are places where twist is being
# reported or used as a last resort - never where the pose can answer:
#   1. `travel_sign()`, only after the pose projection has declared itself
#      ambiguous, and the event is counted when it happens;
#   2. the linear velocity republished on /odom, so downstream consumers never
#      see the board's inverted number.
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
REPORT_SEC = 30.0            # periodic health line; matches fpms_lidar_ros.py

BIAS_SETTLE_SEC = 3.0        # startup window spent averaging gyro Z at rest
BIAS_MIN_SAMPLES = 20        # refuse to trust a bias built from fewer than this

# HARD DEADLINE on the startup settle. Without one, a node that starts while the
# rover is being driven (a service restart mid-mission is the obvious case)
# never collects `BIAS_MIN_SAMPLES` of quiet gyro, never leaves the settling
# phase, and therefore NEVER INTEGRATES HEADING AT ALL - it publishes theta = 0
# forever while the rover drives around, silently, with only the one startup log
# line to show for it. That is the worst failure this file could have: a
# confident straight-line pose for a robot that is turning.
#
# On the deadline the estimator gives up on a clean calibration, adopts whatever
# it has (the partial mean, or zero if it has nothing), and marks itself
# PROVISIONAL: heading integration starts, the yaw covariance stays at
# UNMEASURED_VAR, and the log says so repeatedly rather than once. A drifting
# heading is recoverable - the running re-estimate fixes it the moment the rover
# genuinely stops - whereas a frozen heading is not recoverable at all.
#
# 20 s is 6.7x the settle window: long enough that a slow /imu start or a rover
# that is merely being nudged still calibrates properly, short enough that the
# rover cannot cross this 1.2 m arena before the fallback engages (at the
# mission cruise speed of 0.18 m/s, 20 s is 3.6 m).
BIAS_SETTLE_DEADLINE_SEC = 20.0

# Must be LONGER than the longest post-motion settle in fpms_missions.py
# (TURN_SETTLE_S = 1.2 s). During that settle the wire is commanded to zero and
# the pose has stopped moving, but the chassis is still COASTING through the
# tail of a turn - real rotation, below GYRO_STILL_RAD_S, that would be learned
# as bias and then subtracted from every future turn. This inequality is
# load-bearing and cross-file; if TURN_SETTLE_S ever grows, this must grow with
# it.
BIAS_REEST_SEC = 1.5

BIAS_EMA_ALPHA = 0.02        # slow blend for the running re-estimate

# |gyro| above this means we are definitely moving, whatever anything else says.
# fpms_missions.py turns at TURN_RADPS = 0.45 rad/s, so a commanded turn is
# always rejected outright; this threshold's real job is catching a rover that
# is being rotated by hand or by a collision while /cmd_vel says zero.
GYRO_STILL_RAD_S = 0.35

CMD_STILL_EPS = 1e-3         # |cmd_vel| below this counts as a commanded stop
CMD_FRESH_SEC = 1.0          # a /cmd_vel older than this is evidence of nothing

# Wheels-stopped test, as a SPEED not a distance. The previous form compared a
# raw per-message pose delta against a fixed 2 mm, which silently meant
# different speeds at different message rates - /odom_raw is only nominally
# 10-11 Hz, and the threshold moved with the jitter. 0.02 m/s is comfortably
# below the firmware's expressible floor: NAV2_BRIEF.md section 3b puts one
# encoder count per 10 ms PID period at 0.0145 m/s on the wire, which the
# measured ~6x discrepancy puts near 0.09 m/s of real ground speed. Anything
# the velocity loop can actually hold is more than 4x this.
WHEEL_STILL_MPS = 0.02

# Below one encoder tick of ground travel the board CANNOT have observed
# motion, so a delta smaller than this is float and transport noise. It matters
# because hypot() is unsigned: a parked rover whose pose jitters by a few
# microns would otherwise have every jitter integrated as travel, and the old
# "no directional evidence -> assume forward" rule turned that random walk into
# a monotonic forward creep. Derived from the firmware's own tick constant
# rather than picked - and note NAV2_BRIEF.md section 3b says the fitted
# hardware does not match those constants (roughly 6x out), which makes the real
# tick larger than this and therefore makes this deadband conservatively small.
# Being too small only means integrating a little noise; too large would mean
# discarding real motion.
POSE_NOISE_FLOOR_M = M_PER_TICK

# For the pose-projection sign test: the along-heading component must be the
# MAJORITY component of the delta for its sign to count as evidence. cos(45 deg)
# is exactly the point at which the along-track and across-track components are
# equal; past it, "the robot moved along its heading" has stopped being the
# leading explanation of the delta and the projection's sign is noise.
SIGN_PROJECTION_MIN_FRAC = 0.70710678118654752  # 1/sqrt(2)

# A quaternion within this of identity is treated as "the board is not
# reporting a yaw". Sized well below the smallest turn this rover can express
# so a genuinely small heading is never mistaken for identity.
IDENTITY_QUAT_EPS = 1e-6

MAX_PLAUSIBLE_STEP_M = 0.5   # a single pose delta larger than this is a glitch
# Belt to that braces, and the one that survives a change of message rate.
# Commanding wire 0.10 measured ~0.61 m/s of ground speed (NAV2_BRIEF.md section
# 3b), and every writer on this rover clamps well under that - fpms_missions.py
# at HARD_MAX_LIN_MPS = 0.25 real, teleop lower still. 1.5 m/s is over 2x
# anything commandable, so it only ever fires on a transport or encoder glitch.
MAX_PLAUSIBLE_SPEED_MPS = 1.5
MAX_PLAUSIBLE_DT = 0.5       # ignore integration gaps longer than this
MIN_PLAUSIBLE_DT = 1e-6

# ============================================================================
# HEADING FUSION - None, for the two reasons in the module docstring.
#
# The arithmetic, once there is something to put in it:
#
#     HEADING_COMPLEMENTARY_TAU_S = sigma_reference / bias_residual
#
# with sigma_reference in radians and bias_residual in rad/s (M3). But note
# REASON 2 before reaching for the encoder yaw as that reference: it is a second
# dead-reckoned heading, not a bounded one, and blending two drifting signals
# bounds nothing. The reference this wants is the LiDAR wall fit.
#
# While this is None the heading is the pure integrated gyro. Nothing in the
# node calls `complementary_heading()` - it is deliberately not wired, because a
# filter that cannot bound the error should not be in the live path pretending
# to.
# ============================================================================
HEADING_COMPLEMENTARY_TAU_S = None

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
# These four are STATED GUESSES, not measurements. They are round numbers
# because there is nothing to round from - saying "0.05 m" is at least legible
# as an assertion, whereas 0.0473 would imply a measurement that never happened.
# POSE_YAW_VAR in particular is gated on M3 in the docstring: once the residual
# bias is known, the honest yaw variance is (bias_residual * t_since_fix)^2 plus
# the gyro's random walk, and this constant should be replaced by that.
#
# They are NOT time-varying. nav_msgs/Odometry covariance is a per-message
# statement, and robot_localization reads /odom differentially - feeding it a
# variance that grows without bound would misreport a per-step uncertainty as an
# accumulated one. The accumulation is real, but it belongs to whatever holds
# map -> odom, which today is nothing.
POSE_XY_VAR = 0.05 ** 2      # ~5 cm 1-sigma, wheel odom on a reasonable floor
POSE_YAW_VAR = 0.20 ** 2     # ~11 deg 1-sigma, open-loop integrated gyro
UNMEASURED_VAR = 1e6         # z / roll / pitch: simply not known
TWIST_LINEAR_VAR = 0.10 ** 2
TWIST_ANGULAR_VAR = 0.15 ** 2

# ============================================================================
# LIDAR ARENA FIX - the maths is implemented and self-tested; the wiring is OFF.
#
# LIDAR_FIX_ENABLED is not a convenience switch. Turning it on without M4, M5
# and M6 from the docstring produces a pose correction that is CONFIDENT and
# WRONG, which is strictly worse than the dead reckoning it would replace,
# because dead-reckoning error is smooth and recognisable while a bad absolute
# fix teleports the robot. `lidar_fix_blockers()` lists what is missing and the
# node refuses with those reasons at startup rather than just doing nothing.
# ============================================================================
LIDAR_FIX_ENABLED = False
SCAN_TOPIC = "/scan_lidar"   # published by fpms_lidar_ros.py; NOT the dead /scan

# The arena, mirrored from frontend/src/lib/arena.ts via fpms_missions.py.
# Mirrored as the same one number that file uses, so a rescale there stays a
# one-number change here.
ARENA_MM = 1200.0
ARENA_M = ARENA_MM / 1000.0

# A fix is only ever ACCEPTED while the rover is confirmed stationary. The scan
# arrives via an off-board MQTT round trip measured at median 100 ms / p90
# 179 ms (fpms_lidar_ros.py). At the mission cruise speed that is under 4 cm of
# translation, which would be tolerable - but at the mission turn rate of
# 0.45 rad/s it is up to 4.6 degrees of heading, which is larger than the whole
# error the fix is trying to remove. Standing still makes the latency free.
# fpms_missions.py already stops between every segment, so this costs nothing.
LIDAR_FIX_REQUIRE_STILL = True

# Rejection thresholds for a candidate fix. All of them are structural (they
# describe the geometry of a 1200 mm box) rather than tuned, because there is no
# data to tune against.
LIDAR_MIN_WALL_POINTS = 12        # per wall, before a distance is believed
LIDAR_MIN_MANHATTAN_FRAC = 0.35   # of returns aligned to the dominant grid
LIDAR_MAX_SPAN_ERROR_M = 0.10     # |d_near + d_far - 1.200|; see M7
LIDAR_MAX_JUMP_M = 0.30           # a fix further than this from dead reckoning
LIDAR_MAX_JUMP_RAD = math.radians(25.0)
LIDAR_WALL_BAND_M = 0.06          # inlier band about a fitted wall line


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


def quat_is_identity(x, y, z, w, eps=IDENTITY_QUAT_EPS):
    """True when a quaternion carries no rotation worth reading.

    Used to answer M1 - whether /odom_raw's orientation is a real encoder yaw or
    the same unfused identity the /imu topic publishes - at runtime, once,
    instead of trusting either of fpms_teleop.py's two contradictory comments
    about it.
    """
    if not is_finite(x, y, z, w):
        return True
    return (abs(float(x)) <= eps and abs(float(y)) <= eps
            and abs(float(z)) <= eps and abs(abs(float(w)) - 1.0) <= eps)


def travel_sign(dx, dy, board_yaw, twist_linear_x,
                min_frac=SIGN_PROJECTION_MIN_FRAC,
                noise_floor=POSE_NOISE_FLOOR_M):
    """Direction of travel for one pose delta. Returns (sign, evidence).

    `sign` is -1.0, 0.0 or +1.0. `evidence` is one of:

        "noise"      the delta is under one encoder tick; there was no motion
        "pose"       the delta projects cleanly onto the board's own heading
        "twist"      the projection was ambiguous, twist broke the tie
        "none"       neither said anything; the step is dropped

    THE POINT OF THIS FUNCTION is that "pose" is the normal answer and "twist"
    is an exception that gets counted. Direction is taken from the SAME message
    that supplied the magnitude, by projecting the delta onto the board's
    reported forward vector - which is exactly what fpms_teleop.py's `_on_odom`
    and `summarize_leg` already do, and it is internally consistent whatever the
    board's frame is doing, because both fields describe the same frame.

    If /odom_raw's orientation turns out to be identity (M1), board_yaw is 0 and
    the projection degenerates to `dx` - which is precisely the signed quantity
    the NAV2_BRIEF.md section 3a bench measurements were read from, and which
    was correct in every one of those trials. So the identity case is not a
    degradation; it is the measured case.

    The ambiguous case is real and worth naming: if the board reports identity
    while still integrating position through an internal heading it does not
    publish, then after a 90 degree turn the travel is along +/-y, `dx` is
    approximately zero, and the projection has nothing to say. Only then is the
    twist consulted, sign-corrected once by ODOM_TWIST_SIGN, and the caller
    counts the event so that "how often are we falling back to the field we do
    not trust?" is a number somebody can read rather than a guess.

    Returning 0.0 rather than +1.0 when there is no evidence is deliberate. The
    magnitude is an unsigned hypot(), so a parked rover's position noise used to
    be integrated as a monotonic FORWARD creep - every jitter, always positive,
    accumulating. Dropping the step instead is correct: with no evidence of
    direction, the honest displacement is zero.
    """
    magnitude = math.hypot(dx, dy)
    if magnitude < noise_floor:
        return 0.0, "noise"

    if is_finite(board_yaw):
        along = dx * math.cos(board_yaw) + dy * math.sin(board_yaw)
        if abs(along) >= min_frac * magnitude:
            return (1.0 if along > 0.0 else -1.0), "pose"

    if is_finite(twist_linear_x) and twist_linear_x != 0.0:
        corrected = ODOM_TWIST_SIGN * float(twist_linear_x)
        return (1.0 if corrected > 0.0 else -1.0), "twist"

    return 0.0, "none"


def signed_travel(dx, dy, board_yaw, twist_linear_x):
    """(distance, evidence) for one pose delta. Magnitude from pose, always."""
    sign, evidence = travel_sign(dx, dy, board_yaw, twist_linear_x)
    return sign * math.hypot(dx, dy), evidence


def remove_bias(raw_rate, bias):
    """Bias-corrected gyro rate. Trivial, but named so the intent is greppable."""
    return raw_rate - bias


def integrate_heading(theta, rate, dt):
    """Advance heading by a bias-corrected rate over dt, wrapped to [-pi, pi]."""
    return wrap_to_pi(theta + rate * dt)


def midpoint_heading(theta_prev, theta_now):
    """The heading halfway between two samples, taken the short way round.

    A pose delta accrued over the last ~100 ms while the heading was CHANGING.
    Advancing it along the heading measured at the END of that interval biases
    every arc outward by half the turn - small per step, but it is a bias, and a
    bias is the one error a retrace cannot cancel (fpms_missions.py makes the
    same argument for its turn-coast factor). The midpoint is second-order exact
    for a constant turn rate, which is what a 100 ms window is.

    Wrapping the DIFFERENCE rather than averaging the two angles is what keeps
    this correct across the +/-pi seam: the naive mean of -179 and +179 degrees
    is 0, i.e. exactly backwards.
    """
    return wrap_to_pi(theta_prev + 0.5 * wrap_to_pi(theta_now - theta_prev))


def complementary_heading(theta_gyro, theta_reference, dt, tau):
    """Blend an integrated gyro heading towards an ABSOLUTE reference heading.

    theta <- (1 - a) * theta_gyro + a * theta_reference,   a = dt / (tau + dt)

    NOT CALLED ANYWHERE IN THE LIVE PATH, and `tau` is None. Two reasons, both
    in the module docstring: `tau` has no measured value, and - more
    fundamentally - the only reference currently on offer (the board's encoder
    yaw) is itself dead-reckoned, so blending it in cannot bound the drift. This
    function is correct and ready for a reference that IS bounded, which on this
    rover means the LiDAR wall fit.

    `theta_reference` must be expressed in the SAME frame as `theta_gyro`. The
    board's odom yaw is not - it has been free-running since the board booted,
    while this node's theta starts at zero - so it cannot be passed in here
    without first being re-based, which is a second reason not to.

    The blend is done on the WRAPPED DIFFERENCE, not on the two angles, for the
    same seam reason as `midpoint_heading`.
    """
    if tau is None or not is_finite(theta_reference, dt, tau):
        return theta_gyro
    if dt <= 0.0 or tau <= 0.0:
        return theta_gyro
    a = dt / (tau + dt)
    return wrap_to_pi(theta_gyro + a * wrap_to_pi(theta_reference - theta_gyro))


def advance_pose(x, y, theta, distance):
    """Move `distance` along the CURRENT heading.

    Heading from the gyro, distance from the wheels - this one line is the
    actual fusion. The board's own reported heading is never consulted for the
    world-frame direction; it is used only to sign the step, inside the same
    message that produced it (see `travel_sign`).
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


# ============================================================================
# LIDAR ARENA FIX - pure geometry, no ROS, no node state, no hardware.
#
# WHAT THIS IS FOR
# ----------------
# Dead reckoning has no absolute reference; its error grows without bound and
# filtering cannot fix that, only slow it. The arena is a known 1200 x 1200 mm
# box, and a rectangular room is an unusually strong prior: one 360-degree scan
# determines all three degrees of freedom, with no map server, no particle
# filter and no AMCL.
#
# WHERE A FIX MUST AND MUST NOT BE APPLIED - READ THIS BEFORE WIRING IT UP
# ------------------------------------------------------------------------
# A fix from this code MUST NOT be written into this node's x/y/theta, and MUST
# NOT move the odom -> base_footprint transform. REP-105 requires `odom` to be
# CONTINUOUS - smooth, drift-permitted, jump-forbidden - precisely so that local
# planners and controllers can differentiate it. Teleporting it is how you get a
# controller that reacts to a correction as though the robot were flung across
# the arena. Absolute corrections belong on the map -> odom edge, which is
# AMCL's edge in a normal stack and would be this node's only if it ever
# published one. `map_to_odom_from_fix()` below computes exactly that transform
# and nothing else.
#
# The second, cheaper consumer is fpms_missions.py's `Anchor`, whose whole job
# is mapping odom onto the arena and which today holds an ASSUMED start pose
# behind a "POSE: SIMULATED" badge. A validated fix is what would let it set
# `assumed = False` honestly.
#
# METHOD
# ------
# 1. Convert the scan into base_footprint cartesian points.
# 2. MANHATTAN FRAME: every wall of a rectangular arena runs along one of two
#    perpendicular directions, so the directions of short segments between
#    neighbouring returns cluster modulo 90 degrees. `manhattan_offset()`
#    recovers that grid rotation, and its concentration measure doubles as
#    "does this scan look like a box at all?".
# 3. HEADING: the grid rotation gives heading modulo 90 degrees. Dead reckoning
#    is already good to far better than +/-45 degrees, so it picks which of the
#    four candidates is right - the prior resolves the ambiguity, it does not
#    contribute to the answer. The result is an ABSOLUTE heading with no drift
#    term at all, which is the single most valuable thing the LiDAR can give
#    this rover, because gyro bias drift is its largest error source.
# 4. POSITION: de-rotate the points into world axes and fit the four walls.
#    Each axis is measured TWICE, from opposite walls, and the two must sum to
#    1.200 m. That disagreement is a free, continuous self-test (see M7).
#
# WHY OPPOSITE-WALL AVERAGING IS WORTH THE EXTRA WALL
# ----------------------------------------------------
# A constant range bias b lengthens every ray. The near-wall estimate moves one
# way, the far-wall estimate moves the other way by the same amount, so their
# MEAN is exempt from b entirely while their disagreement is an indicator of it.
# One geometric arrangement gives both a bias-immune position and a standing
# measurement of the bias. `--selftest` confirms the cancellation holds to
# under 2 mm for a 30 mm bias, and that the disagreement is 0.72-0.86 of 2b
# rather than exactly 2b - see M7 for why.
#
# The zone rectangles from arena.ts are deliberately not used. They are painted
# floor regions, and nothing in this repo says they present a surface a scanner
# can range against. Fitting to them would be inventing geometry.
# ============================================================================

# fpms_lidar_ros.py's published geometry. Mirrored, not re-derived: if those
# change, these must change with them, and the node checks the incoming message
# rather than assuming.
SCAN_ANGLE_MIN = -math.pi
SCAN_ANGLE_INCREMENT = 2.0 * math.pi / 360.0


def scan_points_base(ranges, angle_min=SCAN_ANGLE_MIN,
                     angle_increment=SCAN_ANGLE_INCREMENT,
                     laser_x=0.0, laser_y=0.0, laser_yaw=0.0,
                     range_min=0.12, range_max=6.0):
    """LaserScan ranges -> [(x, y), ...] in the base_footprint frame.

    Non-returns arrive as inf or nan (fpms_lidar_ros.py converts the agent's 0.0
    "no return" into inf for exactly this reason) and are dropped here. A 0.0
    that ever reaches this function is a bug upstream, not an obstacle touching
    the sensor, and is dropped too.

    `laser_x/y/yaw` are the mount transform. THEY ARE PLACEHOLDERS EVERYWHERE IN
    THIS REPO (M4). A yaw error rotates every point by that error and therefore
    rotates the heading fix by exactly it, with no other symptom.
    """
    pts = []
    if not ranges:
        return pts
    c, s = math.cos(laser_yaw), math.sin(laser_yaw)
    for j, r in enumerate(ranges):
        try:
            rv = float(r)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(rv) or rv < range_min or rv >= range_max:
            continue
        a = angle_min + j * angle_increment
        lx, ly = rv * math.cos(a), rv * math.sin(a)
        pts.append((laser_x + c * lx - s * ly, laser_y + s * lx + c * ly))
    return pts


def manhattan_offset(points, max_gap_m=0.15):
    """Dominant orthogonal grid rotation of a point set. Returns (rho, conc, n).

    `rho` is in [0, pi/2): the angle, in the input frame, of the grid the points
    lie on. `conc` is the concentration in [0, 1] - 1.0 means every segment is
    aligned to that grid, 0.0 means the directions are isotropic and there is no
    box here. `n` is the number of segments that contributed.

    The 90-degree periodicity is handled by mapping each segment direction psi
    onto the unit circle as angle 4*psi and taking a length-weighted circular
    mean. Four times the angle because a direction is already only defined
    modulo pi (a wall has no near or far side), and the grid adds a second
    factor of two. This is the standard "Manhattan frame" estimator and it needs
    no histogram, no bin width to argue about, and no RANSAC: the resultant
    length IS the goodness of fit.

    Segments longer than `max_gap_m` are dropped - those span the jump from one
    surface to another (an occlusion edge, or the gap where a wall ends) and
    their direction describes nothing physical. 0.15 m is a little over twice
    the 6 cm inlier band used for the walls themselves, so a genuinely rough
    wall still contributes while a corner jump does not.
    """
    if len(points) < 3:
        return 0.0, 0.0, 0
    cs = sn = wsum = 0.0
    n = 0
    for i in range(len(points)):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % len(points)]
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length <= 1e-9 or length > max_gap_m:
            continue
        psi = math.atan2(dy, dx)
        cs += length * math.cos(4.0 * psi)
        sn += length * math.sin(4.0 * psi)
        wsum += length
        n += 1
    if wsum <= 0.0 or n == 0:
        return 0.0, 0.0, 0
    conc = math.hypot(cs, sn) / wsum
    rho = math.atan2(sn, cs) / 4.0
    return rho % (math.pi / 2.0), conc, n


def heading_from_manhattan(rho, prior_heading):
    """Resolve the grid rotation's 4-fold ambiguity against dead reckoning.

    `rho` fixes the heading only modulo 90 degrees. The four candidates are
    -rho + k*90 degrees (negated because rho is the world grid's angle seen from
    the body, and the heading is the body's angle seen from the world). Dead
    reckoning chooses among them and contributes nothing else - so a prior that
    is wrong by up to 45 degrees still yields an exactly correct heading, and a
    prior wrong by more than that yields one that is wrong by a whole multiple
    of 90 degrees, which is loud rather than subtle.

    Returns (heading, margin_rad): `margin` is how far the prior sat from the
    chosen candidate. A margin near 45 degrees means the choice was nearly a
    coin toss and the caller should reject the fix.
    """
    best, best_err = None, None
    for k in range(4):
        cand = wrap_to_pi(-rho + k * (math.pi / 2.0))
        err = abs(wrap_to_pi(cand - prior_heading))
        if best_err is None or err < best_err:
            best, best_err = cand, err
    return best, best_err


def _wall_fit_1d(values, band_m, min_points):
    """Fit the two extreme surfaces along one axis. Returns a dict.

    `values` are the de-rotated coordinates of every return along one world
    axis. The near wall is at the minimum and the far wall at the maximum,
    because obstacles inside the arena can only SHORTEN a ray, never lengthen
    it - anything between the rover and a wall steals that bearing's return and
    contributes a value closer to zero, so clutter can remove wall support but
    can never fake a wall further out than the real one.

    The extreme itself is a single noisy sample, so it is only used to seed a
    band; the reported coordinate is the mean of every return inside that band.
    """
    out = {"low": None, "high": None, "n_low": 0, "n_high": 0}
    if not values:
        return out
    lo_seed, hi_seed = min(values), max(values)
    lo = [v for v in values if v <= lo_seed + band_m]
    hi = [v for v in values if v >= hi_seed - band_m]
    out["n_low"], out["n_high"] = len(lo), len(hi)
    if len(lo) >= min_points:
        out["low"] = sum(lo) / len(lo)
    if len(hi) >= min_points:
        out["high"] = sum(hi) / len(hi)
    return out


def wall_fit(points, heading, arena_m=ARENA_M,
             band_m=LIDAR_WALL_BAND_M, min_points=LIDAR_MIN_WALL_POINTS):
    """Rover position in the arena from four walls. Returns a dict.

    `points` are in the base frame, `heading` is the rover's world heading. Each
    point is rotated into world axes, giving its offset from the rover in world
    x/y; a wall at world coordinate W and offset q then says rover = W - q.

    Keys: x_m, y_m (None if that axis could not be fitted), span_error_x_m,
    span_error_y_m (measured wall separation minus the known 1.200 m - an
    indicator of the scanner's range bias, a little under 2b; see M7), and the
    four inlier counts.
    """
    c, s = math.cos(heading), math.sin(heading)
    qx = [c * px - s * py for px, py in points]
    qy = [s * px + c * py for px, py in points]

    fx = _wall_fit_1d(qx, band_m, min_points)
    fy = _wall_fit_1d(qy, band_m, min_points)

    out = {"x_m": None, "y_m": None,
           "span_error_x_m": None, "span_error_y_m": None,
           "n_x_low": fx["n_low"], "n_x_high": fx["n_high"],
           "n_y_low": fy["n_low"], "n_y_high": fy["n_high"]}

    for axis, fit in (("x", fx), ("y", fy)):
        if fit["low"] is None or fit["high"] is None:
            continue
        # Wall at 0 says rover = -q_low; wall at arena_m says rover = arena_m -
        # q_high. Averaging the two cancels a constant range bias exactly, while
        # their disagreement measures twice that bias.
        near = -fit["low"]
        far = arena_m - fit["high"]
        out["%s_m" % axis] = 0.5 * (near + far)
        out["span_error_%s_m" % axis] = (fit["high"] - fit["low"]) - arena_m
    return out


def arena_fix_from_scan(ranges, prior_x, prior_y, prior_heading,
                        angle_min=SCAN_ANGLE_MIN,
                        angle_increment=SCAN_ANGLE_INCREMENT,
                        laser_x=LASER_X_OFFSET_M, laser_y=LASER_Y_OFFSET_M,
                        laser_yaw=LASER_YAW_RAD, arena_m=ARENA_M,
                        range_min=0.12, range_max=6.0):
    """One scan + a dead-reckoned prior -> an arena-frame pose fix, or a refusal.

    Returns a dict that ALWAYS has `accepted` and `rejects`. `rejects` is a list
    of strings and is the whole point of the return shape: a correction that
    declines to fire must say which of its preconditions failed, or the next
    person debugging it has nothing to go on but silence.

    The prior is used for exactly two things - resolving the heading's 4-fold
    ambiguity, and bounding how far a fix is allowed to move the pose. It never
    contributes to the fitted numbers, so a fix is not a filtered version of
    dead reckoning; it is independent evidence.
    """
    result = {"accepted": False, "rejects": [],
              "x_m": None, "y_m": None, "heading_rad": None,
              "manhattan_conc": 0.0, "n_points": 0,
              "span_error_x_m": None, "span_error_y_m": None,
              "heading_margin_rad": None}

    pts = scan_points_base(ranges, angle_min, angle_increment,
                           laser_x, laser_y, laser_yaw, range_min, range_max)
    result["n_points"] = len(pts)
    if len(pts) < 4 * LIDAR_MIN_WALL_POINTS:
        result["rejects"].append(
            "only %d usable returns; need %d" % (len(pts), 4 * LIDAR_MIN_WALL_POINTS))
        return result

    rho, conc, _n_seg = manhattan_offset(pts)
    result["manhattan_conc"] = conc
    if conc < LIDAR_MIN_MANHATTAN_FRAC:
        result["rejects"].append(
            "scan is not box-shaped (manhattan concentration %.2f < %.2f)"
            % (conc, LIDAR_MIN_MANHATTAN_FRAC))
        return result

    heading, margin = heading_from_manhattan(rho, prior_heading)
    result["heading_rad"] = heading
    result["heading_margin_rad"] = margin
    if margin > LIDAR_MAX_JUMP_RAD:
        result["rejects"].append(
            "heading fix %.1f deg is %.1f deg from dead reckoning (max %.1f)"
            % (math.degrees(heading), math.degrees(margin),
               math.degrees(LIDAR_MAX_JUMP_RAD)))

    fit = wall_fit(pts, heading, arena_m)
    result["x_m"], result["y_m"] = fit["x_m"], fit["y_m"]
    result["span_error_x_m"] = fit["span_error_x_m"]
    result["span_error_y_m"] = fit["span_error_y_m"]

    for axis in ("x", "y"):
        if fit["%s_m" % axis] is None:
            result["rejects"].append(
                "%s axis not bracketed by two walls (inliers %d / %d, need %d each)"
                % (axis, fit["n_%s_low" % axis], fit["n_%s_high" % axis],
                   LIDAR_MIN_WALL_POINTS))
            continue
        span_err = fit["span_error_%s_m" % axis]
        if abs(span_err) > LIDAR_MAX_SPAN_ERROR_M:
            result["rejects"].append(
                "%s walls %.3f m apart, arena is %.3f (error %+.3f m > %.3f); "
                "range bias or the wrong walls"
                % (axis, span_err + arena_m, arena_m, span_err,
                   LIDAR_MAX_SPAN_ERROR_M))

    if result["x_m"] is not None and result["y_m"] is not None:
        jump = math.hypot(result["x_m"] - prior_x, result["y_m"] - prior_y)
        if jump > LIDAR_MAX_JUMP_M:
            result["rejects"].append(
                "fix is %.3f m from dead reckoning (max %.3f)"
                % (jump, LIDAR_MAX_JUMP_M))

    result["accepted"] = (not result["rejects"]
                          and result["x_m"] is not None
                          and result["y_m"] is not None)
    return result


def map_to_odom_from_fix(fix_x, fix_y, fix_heading, odom_x, odom_y, odom_heading):
    """The map -> odom transform implied by an absolute fix. (x, y, yaw).

    T_map_odom = T_map_base * inverse(T_odom_base). This is the ONLY correct
    place to apply an absolute correction: odom -> base_footprint stays
    continuous and keeps drifting, and every jump lands on this edge instead,
    which is exactly the contract REP-105 and Nav2 expect (it is the edge AMCL
    would own). Nothing in this file publishes it yet.
    """
    dtheta = wrap_to_pi(fix_heading - odom_heading)
    c, s = math.cos(dtheta), math.sin(dtheta)
    return (fix_x - (c * odom_x - s * odom_y),
            fix_y - (s * odom_x + c * odom_y),
            dtheta)


def lidar_fix_wiring_plan():
    """Exactly what turning the correction on would mean. One place, so that
    "it is designed but not built" is a specific claim somebody can check.

    1. Subscribe SCAN_TOPIC (sensor-data QoS - fpms_lidar_ros.py publishes
       RELIABLE, which satisfies a BEST_EFFORT subscriber).
    2. On each scan, and ONLY when `_is_confirmed_still_unlocked()` agrees
       (LIDAR_FIX_REQUIRE_STILL): call arena_fix_from_scan() with the current
       dead-reckoned arena pose as the prior. The stillness gate is what makes
       the MQTT bridge's median-100 ms / p90-179 ms transport delay free rather
       than a 4.6 degree heading smear at the mission turn rate.
    3. If `accepted`, feed `map_to_odom_from_fix()` and broadcast map -> odom.
       NEVER touch self.x / self.y / self.theta and never move
       odom -> base_footprint - `odom` must stay continuous (REP-105).
    4. Publish the rejects and the span errors as telemetry, so a correction
       that stops firing says why without anyone opening a terminal.

    Step 3 is also where fpms_missions.py's `Anchor` would be updated, which is
    what would let its `pose_assumed` flag - and the dashboard's "POSE:
    SIMULATED" badge - become false honestly for the first time.
    """
    return "subscribe %s, fix only while still=%s, publish map -> odom" % (
        SCAN_TOPIC, LIDAR_FIX_REQUIRE_STILL)


def lidar_fix_blockers():
    """Why the LiDAR correction is not running. A list of strings, newest last.

    Deliberately a function rather than a constant: it is printed at startup so
    the refusal is visible in the journal on every boot, instead of being a
    False that somebody has to go read the source to understand.
    """
    blockers = []
    if not LIDAR_FIX_ENABLED:
        blockers.append(
            "LIDAR_FIX_ENABLED is False - the fit maths is shipped and "
            "self-tested, the wiring (%s) is deliberately not built; see "
            "lidar_fix_wiring_plan()" % lidar_fix_wiring_plan())
    blockers.append(
        "M4: base_link -> laser_frame is a PLACEHOLDER in "
        "rover/nav2/fpms_tf.launch.py; LASER_YAW_RAD in particular rotates "
        "every heading fix by exactly its error")
    blockers.append(
        "M5: fpms_lidar_ros.py LIDAR_ZERO_OFFSET_DEG / LIDAR_ROTATION_SIGN are "
        "UNVERIFIED; a mirrored scan of a SQUARE arena still fits perfectly and "
        "returns a confidently reflected pose")
    blockers.append(
        "M6: nobody has recorded whether the arena has physical walls the "
        "scanner can see at its mounting height")
    return blockers


class GyroBiasEstimator(object):
    """Estimates the constant offset on the gyro's Z axis.

    A MEMS rate gyro reads a non-zero rate while perfectly still. Integrated
    over a few minutes, even 0.01 rad/s of bias becomes tens of degrees of
    heading error - the single largest source of drift in this node, far larger
    than wheel slip.

    Three phases:

      SETTLING - at startup the robot is assumed stationary and everything is
      averaged for `settle_sec`. Until `min_samples` have arrived the bias is
      not considered valid and heading integration is held off, so a garbage
      offset is never baked into the first metres of a run.

      PROVISIONAL - reached only if `deadline_sec` passes without a clean
      settle, which happens whenever this node starts while the rover is
      already moving (a service restart mid-mission is the obvious case). See
      BIAS_SETTLE_DEADLINE_SEC for why this phase has to exist: without it the
      node would sit in SETTLING forever and NEVER INTEGRATE HEADING, publishing
      theta = 0 for a robot that is turning. It adopts whatever partial mean it
      has - or zero - starts integrating, and keeps `provisional` True so the
      caller can keep publishing an honest yaw covariance and keep complaining.

      RUNNING - any time the robot is commanded to stop AND the wheels confirm
      it is not moving, samples fold into a slow EMA. This tracks the thermal
      drift that makes a one-shot startup calibration go stale over a long
      mission, and it is also what promotes a PROVISIONAL estimate to a real one
      the first time the rover genuinely stops.

    A sample whose magnitude exceeds `still_limit` is discarded regardless of
    what anything else claims: that much rotation means the robot is moving, and
    whatever said otherwise was wrong.
    """

    def __init__(self, settle_sec=BIAS_SETTLE_SEC, min_samples=BIAS_MIN_SAMPLES,
                 alpha=BIAS_EMA_ALPHA, still_limit=GYRO_STILL_RAD_S,
                 deadline_sec=BIAS_SETTLE_DEADLINE_SEC):
        self.settle_sec = float(settle_sec)
        self.min_samples = int(min_samples)
        self.alpha = float(alpha)
        self.still_limit = float(still_limit)
        self.deadline_sec = float(deadline_sec)

        self.bias = 0.0
        self.valid = False
        self.settling = True
        self.provisional = False
        self.samples = 0
        self.updates = 0
        self.promotions = 0
        self._start_time = None
        self._sum = 0.0

    def add_settling_sample(self, rate, now):
        """Feed a sample during the startup window. Returns True once usable.

        "Usable" is not the same as "good": on the deadline path this returns
        True with `provisional` set, meaning heading integration may start but
        nobody should believe the number yet.
        """
        if not is_finite(rate, now):
            return self.valid
        if self._start_time is None:
            self._start_time = now
        if abs(rate) <= self.still_limit:
            self._sum += rate
            self.samples += 1

        elapsed = now - self._start_time
        if elapsed >= self.settle_sec and self.samples >= self.min_samples:
            self.bias = self._sum / float(self.samples)
            self.valid = True
            self.settling = False
            self.provisional = False
        elif elapsed >= self.deadline_sec:
            # Give up on a clean calibration rather than never integrating.
            # Whatever quiet samples arrived are better than nothing, and
            # nothing is better than a permanently frozen heading.
            self.bias = (self._sum / float(self.samples)) if self.samples else 0.0
            self.valid = True
            self.settling = False
            self.provisional = True
        return self.valid

    def add_still_sample(self, rate):
        """Fold a confirmed-stationary sample into the running estimate.

        This is also the path that repairs a PROVISIONAL estimate. The first
        confirmed-still sample after a deadline fallback REPLACES the guess
        outright instead of easing towards it through a 0.02 EMA - that EMA has
        a time constant of ~50 samples (~2 s at 25 Hz) which is fine for
        tracking thermal drift and far too slow to walk back a bad startup.
        """
        if not is_finite(rate):
            return
        if abs(rate) > self.still_limit:
            return
        if not self.valid:
            # Not settled yet - treat it as more settling evidence.
            self._sum += rate
            self.samples += 1
            return
        if self.provisional:
            self.bias = float(rate)
            self.provisional = False
            self.promotions += 1
            return
        self.bias = (1.0 - self.alpha) * self.bias + self.alpha * rate
        self.updates += 1

    def correct(self, rate):
        """Apply the current estimate. Returns 0.0 while the bias is untrusted."""
        if not is_finite(rate) or not self.valid:
            return 0.0
        return remove_bias(rate, self.bias)

    def trusted(self):
        """True only for a bias that came from a clean stationary calibration."""
        return self.valid and not self.provisional


class HeadingCrossCheck(object):
    """Accumulates gyro heading against the board's encoder heading.

    This class produces measurements M2 and M3 from the module docstring, and it
    is the only reason the encoder yaw is read at all. It votes on nothing.

    `scale` is the ratio of encoder-reported rotation to gyro-reported rotation
    over the same interval. On a skid-steer chassis that number is NOT 1.0 and
    is not supposed to be: it is the effective-track factor, the amount by which
    wheels dragged sideways over-report or under-report a turn. Measuring it is
    the entire prerequisite for ever trusting an encoder heading here.

    `drift_rate` is the two headings' disagreement divided by elapsed time,
    which while the rover is PARKED is the gyro bias that survived the
    estimator - i.e. M3 - and while it is DRIVING is dominated by slip instead.
    The caller knows which regime it is in; this class does not, so it reports
    both raw and lets the operator read it in context.

    Accumulating deltas rather than comparing absolute headings is deliberate:
    the two frames have an unknown constant offset (the board's odom yaw has
    been running since the board booted, this node's theta starts at zero), and
    that offset is exactly the thing a delta cancels.
    """

    def __init__(self):
        self.gyro_total = 0.0
        self.enc_total = 0.0
        self.samples = 0
        self.started = None
        self.max_disagreement = 0.0

    def add(self, d_gyro, d_encoder, now):
        if not is_finite(d_gyro, d_encoder, now):
            return
        if self.started is None:
            self.started = now
        self.gyro_total += float(d_gyro)
        self.enc_total += float(d_encoder)
        self.samples += 1
        disagreement = abs(self.gyro_total - self.enc_total)
        if disagreement > self.max_disagreement:
            self.max_disagreement = disagreement

    def report(self, now):
        """(scale, drift_rate_rad_s, gyro_deg, enc_deg) - any may be None."""
        if self.samples == 0 or self.started is None:
            return None, None, 0.0, 0.0
        elapsed = now - self.started
        scale = (self.enc_total / self.gyro_total
                 if abs(self.gyro_total) > math.radians(30.0) else None)
        drift = ((self.gyro_total - self.enc_total) / elapsed
                 if elapsed > 1.0 else None)
        return (scale, drift, math.degrees(self.gyro_total),
                math.degrees(self.enc_total))


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
        self.cross_check = HeadingCrossCheck()

        # Last raw board pose, used to form deltas.
        self._last_raw_x = None
        self._last_raw_y = None
        self._last_raw_yaw = None

        # Heading at the previous /odom_raw message. The pose delta accrued
        # between then and now, so the step is advanced along the MIDPOINT of
        # the two - see midpoint_heading().
        self._theta_at_last_odom = 0.0

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

        # Counters that exist to make the twist question ANSWERABLE later
        # without another bench sweep. `_sign_twist` is how often the pose
        # projection was too ambiguous to sign a step and the untrusted field
        # had to break the tie; `_sign_disagree` is how often pose and
        # sign-corrected twist gave OPPOSITE answers when both had an opinion.
        # A rising `_sign_disagree` on a firmware that has not changed means one
        # of the two is now lying; a `_sign_disagree` that suddenly hits every
        # step means somebody fixed the firmware and ODOM_TWIST_SIGN needs to
        # become +1.
        self._sign_pose = 0
        self._sign_twist = 0
        self._sign_noise = 0
        self._sign_none = 0
        self._sign_disagree = 0

        # M1: is /odom_raw's orientation a real encoder yaw, or identity? The
        # repo contradicts itself (see the module docstring), so observe it.
        self._board_yaw_is_identity = None

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
        self.create_timer(REPORT_SEC, self._report)

        log("odom_tf node up: %s -> %s; position from /odom_raw pose deltas "
            "(sign from the pose projection, not the twist), heading from "
            "integrated /imu gyro z" % (ODOM_FRAME, BASE_FRAME))
        log("settling gyro bias for %.1fs - keep the robot COMPLETELY STILL "
            "(hard deadline %.0fs, after which heading integrates on a "
            "PROVISIONAL bias rather than not at all)"
            % (BIAS_SETTLE_SEC, BIAS_SETTLE_DEADLINE_SEC))
        log("heading source: GYRO ONLY. encoder yaw is a witness, not a vote "
            "(complementary tau=%s)" % (HEADING_COMPLEMENTARY_TAU_S,))
        for reason in lidar_fix_blockers():
            log("lidar arena fix DISABLED - %s" % reason)
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
                    if became_valid and self.bias_est.provisional:
                        log("WARNING gyro z bias did NOT settle within %.0fs - "
                            "the rover was not still. Adopting a PROVISIONAL "
                            "bias %.6f rad/s from %d samples and starting to "
                            "integrate anyway: a drifting heading is "
                            "recoverable, a frozen one is not. Yaw covariance "
                            "stays at 'unmeasured' until the rover stops."
                            % (BIAS_SETTLE_DEADLINE_SEC, self.bias_est.bias,
                               self.bias_est.samples))
                    elif became_valid:
                        log("gyro z bias settled: %.6f rad/s (%.4f deg/s) "
                            "from %d samples"
                            % (self.bias_est.bias, math.degrees(self.bias_est.bias),
                               self.bias_est.samples))
                    return

                # Phase 2: keep learning whenever genuinely stopped.
                if self._is_confirmed_still_unlocked():
                    was_provisional = self.bias_est.provisional
                    self.bias_est.add_still_sample(raw)
                    if was_provisional and not self.bias_est.provisional:
                        log("gyro z bias promoted from PROVISIONAL to measured: "
                            "%.6f rad/s (%.4f deg/s)"
                            % (self.bias_est.bias, math.degrees(self.bias_est.bias)))

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

    def _note_board_yaw_unlocked(self, q):
        """Record, once, whether /odom_raw carries a real yaw (M1).

        fpms_teleop.py asserts both that the board "publishes an identity
        orientation" and that "odom yaw is dead-reckoned from the wheels". They
        cannot both be true. This settles it from the wire instead of from the
        comments, and the answer is what makes `travel_sign`'s pose projection
        either trivially correct (identity: the projection is dx, exactly the
        quantity the section 3a bench measurements were read from) or strictly
        better (real yaw: the projection follows the robot round a turn).

        Only the FIRST non-identity observation flips it, because a real yaw
        genuinely passes through identity every time the board's frame heading
        crosses zero.
        """
        is_id = quat_is_identity(q.x, q.y, q.z, q.w)
        if self._board_yaw_is_identity is None:
            self._board_yaw_is_identity = is_id
            log("M1: /odom_raw pose.pose.orientation is %s on the first "
                "message. %s"
                % ("IDENTITY" if is_id else "a REAL quaternion",
                   "If it stays identity, the board publishes no encoder yaw "
                   "and travel_sign's projection degenerates to dx - which is "
                   "the measured case."
                   if is_id else
                   "The board DOES publish an encoder-derived yaw; "
                   "HeadingCrossCheck will now accumulate M2."))
        elif self._board_yaw_is_identity and not is_id:
            self._board_yaw_is_identity = False
            log("M1 CORRECTION: /odom_raw orientation is NOT always identity - "
                "the board does publish an encoder yaw. It merely started at "
                "zero.")

    def _on_odom_raw(self, msg):
        """Take translation from board pose deltas, then publish /odom + TF."""
        try:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            tx = msg.twist.twist.linear.x

            if not is_finite(px, py):
                with self.lock:
                    self._rejected += 1
                log_every("odom_nan", 5.0, "non-finite /odom_raw pose, dropped")
                return
            if not is_finite(tx):
                # Only the last-resort tie-breaker is lost. The pose projection,
                # which is the primary source, is untouched by this.
                tx = 0.0
            px, py, tx = float(px), float(py), float(tx)
            board_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            if not is_finite(board_yaw):
                board_yaw = 0.0

            now = time.monotonic()

            with self.lock:
                self._odom_msgs += 1
                prev_t = self._last_odom_t
                self._last_odom_t = now
                self._note_board_yaw_unlocked(q)

                # Re-anchor without integrating whenever the previous sample is
                # not a usable reference: the first message ever, or the first
                # after a gap. THE GAP CASE IS NOT COSMETIC. The board's pose is
                # cumulative, so the message that arrives after a 2 s dropout
                # carries every centimetre travelled during the dropout in one
                # delta - and this node has no record of what the heading was
                # doing across it. Integrating that lump along the CURRENT
                # heading writes a straight line where an unknown path was, and
                # it happens exactly when the watchdog has just suspended TF for
                # staleness, i.e. at the least convenient moment.
                gap = None if prev_t is None else (now - prev_t)
                if (self._last_raw_x is None or gap is None
                        or gap > MAX_PLAUSIBLE_DT or gap <= MIN_PLAUSIBLE_DT):
                    if self._last_raw_x is not None and gap is not None:
                        log_every("odom_gap", 5.0,
                                  "/odom_raw gap of %.2fs - re-anchoring "
                                  "without integrating; the heading history "
                                  "across the gap is unknown" % gap)
                        self._rejected += 1
                    self._last_raw_x = px
                    self._last_raw_y = py
                    self._last_raw_yaw = board_yaw
                    self._theta_at_last_odom = self.theta
                    if self._tf_suppressed:
                        log("/odom_raw recovered - resuming %s -> %s tf "
                            "broadcast" % (ODOM_FRAME, BASE_FRAME))
                        self._tf_suppressed = False
                    return

                dx = px - self._last_raw_x
                dy = py - self._last_raw_y
                step = math.hypot(dx, dy)

                if step > MAX_PLAUSIBLE_STEP_M or (step / gap) > MAX_PLAUSIBLE_SPEED_MPS:
                    # Neither of these is physically possible for this chassis -
                    # it is an encoder or transport glitch. Re-anchor to the new
                    # value so the jump is not replayed forever, but do not
                    # integrate it. The speed form is the one that survives a
                    # change of message rate; the absolute form is the backstop
                    # for a gap that slipped through above.
                    log_every("odom_jump", 5.0,
                              "implausible /odom_raw jump %.3fm in %.3fs "
                              "(%.2f m/s) - re-anchoring, not integrating"
                              % (step, gap, step / gap))
                    self._last_raw_x = px
                    self._last_raw_y = py
                    self._last_raw_yaw = board_yaw
                    self._theta_at_last_odom = self.theta
                    self._rejected += 1
                    return

                # Sign the step from the POSE, in the frame of the message that
                # produced it. Twist is consulted only if that is ambiguous, and
                # the outcome is counted either way.
                distance, evidence = signed_travel(dx, dy, board_yaw, tx)
                self._count_sign_evidence_unlocked(evidence, dx, dy, board_yaw, tx)

                # M2/M3: the encoder yaw's opinion of this interval, against the
                # gyro's, accumulated. Neither steers anything - see the heading
                # discussion in the module docstring.
                if self._last_raw_yaw is not None and not self._board_yaw_is_identity:
                    self.cross_check.add(
                        wrap_to_pi(self.theta - self._theta_at_last_odom),
                        wrap_to_pi(board_yaw - self._last_raw_yaw), now)

                self._last_raw_x = px
                self._last_raw_y = py
                self._last_raw_yaw = board_yaw
                self._wheels_still = ((step / gap) < WHEEL_STILL_MPS)
                self._mark_still_or_moving_unlocked()

                # THE FUSION: distance from the wheels, direction from the gyro,
                # taken at the MIDPOINT of the interval the distance accrued
                # over rather than at its end - see midpoint_heading().
                heading = midpoint_heading(self._theta_at_last_odom, self.theta)
                self._theta_at_last_odom = self.theta
                nx, ny = advance_pose(self.x, self.y, heading, distance)
                if not is_finite(nx, ny):
                    self._rejected += 1
                    log_every("fuse_nan", 5.0, "fused pose went non-finite, dropped")
                    return
                self.x = nx
                self.y = ny

                # Report a corrected linear velocity so downstream consumers
                # never see the board's inverted twist. This is a REPORT of the
                # board's own claim, not an input to anything here.
                self._last_linear_v = ODOM_TWIST_SIGN * tx

                if self._tf_suppressed:
                    log("/odom_raw recovered - resuming %s -> %s tf broadcast"
                        % (ODOM_FRAME, BASE_FRAME))
                    self._tf_suppressed = False

                x, y, theta = self.x, self.y, self.theta
                lin, ang = self._last_linear_v, self._last_angular_v
                bias_ok = self.bias_est.trusted()

            self._publish(x, y, theta, lin, ang, bias_ok)
        except Exception as e:
            log_every("odom_cb", 5.0, "odom_raw callback error %s" % (e,))

    def _count_sign_evidence_unlocked(self, evidence, dx, dy, board_yaw, tx):
        """Tally which field signed this step, and whether the two agreed.

        The disagreement counter is the cheap standing test for the section 3a
        fault changing under us. Nobody will re-run a bench sweep to notice a
        firmware update; a counter that has been zero for a thousand steps and
        then is not will notice for free.
        """
        if evidence == "pose":
            self._sign_pose += 1
        elif evidence == "twist":
            self._sign_twist += 1
        elif evidence == "noise":
            self._sign_noise += 1
            return
        else:
            self._sign_none += 1
            return

        if evidence != "pose" or tx == 0.0:
            return
        along = dx * math.cos(board_yaw) + dy * math.sin(board_yaw)
        if (along > 0.0) != (ODOM_TWIST_SIGN * tx > 0.0):
            self._sign_disagree += 1
            log_every("sign_disagree", 30.0,
                      "pose projection and sign-corrected twist DISAGREE on "
                      "direction (along=%+.4f m, twist=%+.4f). Pose wins, as "
                      "measured. If this counter climbs steadily, re-check "
                      "ODOM_TWIST_SIGN against NAV2_BRIEF.md section 3a - the "
                      "firmware may have changed." % (along, tx))

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

            # Before the bias settles CLEANLY the heading is not merely
            # uncertain, it is unmodelled. Say so, rather than quietly implying
            # otherwise. `bias_ok` is GyroBiasEstimator.trusted(), so a
            # PROVISIONAL bias adopted on the settle deadline also lands here -
            # the heading is being integrated (better than frozen) but nothing
            # downstream should weight it.
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

    # ----------------------------------------------------------------- report

    def _report(self):
        """Periodic health line. Never raises.

        This exists because everything interesting about this node is a
        RATIO or a COUNTER, not an instantaneous value, and none of it is
        visible from `ros2 topic echo /odom`: which field is signing the steps,
        whether the bias is real or provisional, and what the encoder heading
        thinks of the gyro's work. It is also where measurements M2 and M3 come
        out - a session that drove the calibration pattern only has to read this
        line to have the numbers the complementary filter needs.

        Same 30 s cadence as fpms_lidar_ros.py's `_report`, for one habit.
        """
        try:
            now = time.monotonic()
            with self.lock:
                bias = self.bias_est.bias
                provisional = self.bias_est.provisional
                valid = self.bias_est.valid
                settling = self.bias_est.settling
                updates = self.bias_est.updates
                samples = self.bias_est.samples
                theta = self.theta
                x, y = self.x, self.y
                odom_n, imu_n, rej = self._odom_msgs, self._imu_msgs, self._rejected
                sp, st, sn, s0, sd = (self._sign_pose, self._sign_twist,
                                      self._sign_noise, self._sign_none,
                                      self._sign_disagree)
                scale, drift, gyro_deg, enc_deg = self.cross_check.report(now)
                identity = self._board_yaw_is_identity

            log("odom=%d imu=%d rejected=%d | pose (%.3f, %.3f) heading %.1f deg"
                % (odom_n, imu_n, rej, x, y, math.degrees(theta)))
            log("sign source: pose=%d twist=%d noise=%d none=%d disagree=%d"
                % (sp, st, sn, s0, sd))
            if settling:
                log("gyro bias STILL SETTLING - heading is NOT being integrated "
                    "yet (%d/%d samples)" % (samples, BIAS_MIN_SAMPLES))
            elif provisional:
                log("gyro bias %.6f rad/s is PROVISIONAL - the rover has not "
                    "been still long enough (%.1fs) to calibrate. Heading is "
                    "integrating but its covariance is published as unmeasured."
                    % (bias, BIAS_REEST_SEC))
            elif valid:
                log("gyro bias %.6f rad/s (%.4f deg/s), %d running updates"
                    % (bias, math.degrees(bias), updates))

            if identity is False:
                log("M2/M3 heading cross-check: gyro %+.1f deg, encoder "
                    "%+.1f deg, scale %s, drift %s rad/s "
                    "(scale is the skid-steer effective-track factor; it is "
                    "NOT expected to be 1.0)"
                    % (gyro_deg, enc_deg,
                       ("%.3f" % scale) if scale is not None else
                       "n/a (need >30 deg of turning)",
                       ("%+.6f" % drift) if drift is not None else "n/a"))
        except Exception as e:
            log_every("report", 60.0, "report error %s" % (e,))


STOP_FLAG = threading.Event()


# ============================================================================
# SELF-TEST - `python3 fpms_odom_tf.py --selftest`, no ROS and no rover.
#
# Everything checked here is pure geometry or pure bookkeeping, so it is
# genuinely verifiable off-robot and the results mean what they say. Nothing
# here validates a physical constant: a synthetic arena proves the fit recovers
# a pose from a scan, and proves NOTHING about whether this rover's scanner is
# mounted the way the constants claim. That distinction is the whole reason the
# LiDAR correction ships disabled.
# ============================================================================

def _synth_arena_scan(rover_x, rover_y, heading, arena_m=ARENA_M, n=360,
                      range_max=6.0, mirror=False):
    """Ray-cast a square box: the exact LaserScan a perfect scanner would send.

    `mirror=True` reverses the angular order, which is what a wrong
    LIDAR_ROTATION_SIGN does (M5). It exists so the self-test can demonstrate
    the failure mode rather than merely assert it: a mirrored scan of a SQUARE
    room still fits perfectly and returns a reflected pose with no complaint.
    """
    ranges = []
    for j in range(n):
        a = SCAN_ANGLE_MIN + j * (2.0 * math.pi / n) + heading
        dx, dy = math.cos(a), math.sin(a)
        best = float("inf")
        for coord, d0, p0 in ((0.0, dx, rover_x), (arena_m, dx, rover_x),
                              (0.0, dy, rover_y), (arena_m, dy, rover_y)):
            if abs(d0) < 1e-12:
                continue
            t = (coord - p0) / d0
            if 0.0 < t < best:
                # Only accept the hit if it lands inside the wall's extent.
                hx, hy = rover_x + dx * t, rover_y + dy * t
                if (-1e-6 <= hx <= arena_m + 1e-6
                        and -1e-6 <= hy <= arena_m + 1e-6):
                    best = t
        ranges.append(best if best < range_max else float("inf"))
    return list(reversed(ranges)) if mirror else ranges


def _selftest():
    failures = []

    def check(name, ok, detail=""):
        print("  %-46s %s%s" % (name, "PASS" if ok else "FAIL",
                                ("  " + detail) if detail else ""))
        if not ok:
            failures.append(name)

    print("travel_sign - direction comes from the pose, twist is the exception")
    # Board reporting identity yaw: the projection is dx, which is exactly the
    # quantity NAV2_BRIEF.md section 3a's bench table was read from.
    s, e = travel_sign(0.05, 0.0, 0.0, -0.842)
    check("forward, identity yaw, twist says backward", (s, e) == (1.0, "pose"),
          "got %r" % ((s, e),))
    s, e = travel_sign(-0.05, 0.0, 0.0, +0.574)
    check("backward, identity yaw, twist says forward", (s, e) == (-1.0, "pose"),
          "got %r" % ((s, e),))
    # Board reporting a real yaw: the projection follows it round a turn.
    s, e = travel_sign(0.0, 0.05, math.pi / 2, 0.0)
    check("forward at 90 deg with a real board yaw", (s, e) == (1.0, "pose"),
          "got %r" % ((s, e),))
    # Identity yaw while the board integrates through an unpublished heading:
    # the projection genuinely cannot answer, so twist breaks the tie and the
    # caller counts it.
    s, e = travel_sign(0.0, 0.05, 0.0, -0.842)
    check("ambiguous projection falls back to twist", (s, e) == (1.0, "twist"),
          "got %r" % ((s, e),))
    # THE CREEP BUG: at rest, hypot() is unsigned, so the old "no evidence ->
    # forward" rule turned position noise into a monotonic forward ratchet.
    s, e = travel_sign(1e-6, 1e-6, 0.0, 0.0)
    check("sub-tick noise contributes nothing", (s, e) == (0.0, "noise"),
          "got %r" % ((s, e),))
    creep = 0.0
    for i in range(2000):
        d, _ = signed_travel(1e-5 * (1 if i % 2 else -1), 0.0, 0.0, 0.0)
        creep += d
    check("2000 jitter samples do not accumulate", abs(creep) < 1e-9,
          "creep %.3e m" % creep)

    print("heading helpers")
    check("midpoint across the +/-pi seam",
          abs(wrap_to_pi(midpoint_heading(math.radians(179),
                                          math.radians(-179)) - math.pi)) < 1e-9,
          "got %.6f" % midpoint_heading(math.radians(179), math.radians(-179)))
    check("midpoint of 0 and 90 is 45",
          abs(midpoint_heading(0.0, math.pi / 2) - math.pi / 4) < 1e-12)
    check("complementary filter is a no-op while tau is None",
          complementary_heading(0.5, 1.5, 0.04, HEADING_COMPLEMENTARY_TAU_S) == 0.5)
    blended = complementary_heading(0.0, 1.0, 0.1, 0.9)
    check("complementary filter blends by dt/(tau+dt)",
          abs(blended - 0.1) < 1e-12, "got %.6f" % blended)

    print("gyro bias estimator")
    est = GyroBiasEstimator(settle_sec=3.0, min_samples=20, deadline_sec=20.0)
    for i in range(100):
        est.add_settling_sample(0.01, i * 0.04)
    check("clean settle finds the bias",
          est.valid and not est.provisional and abs(est.bias - 0.01) < 1e-9,
          "bias %.6f" % est.bias)
    # The failure this guard exists for: a node that starts while the rover is
    # already turning never collects a quiet sample.
    est = GyroBiasEstimator(settle_sec=3.0, min_samples=20, deadline_sec=20.0)
    for i in range(600):                         # 24 s at 25 Hz, past the deadline
        est.add_settling_sample(0.9, i * 0.04)   # 0.9 rad/s: always "moving"
    check("never-still startup hits the deadline, does not hang",
          est.valid and est.provisional and est.bias == 0.0,
          "valid=%s provisional=%s bias=%.6f" % (est.valid, est.provisional, est.bias))
    check("provisional bias is not 'trusted'", not est.trusted())
    est.add_still_sample(0.012)
    check("first genuine still sample replaces the guess outright",
          est.trusted() and abs(est.bias - 0.012) < 1e-12, "bias %.6f" % est.bias)

    print("lidar arena fit - synthetic 1200x1200 box, exact ray casts")
    for (tx, ty, th_deg) in ((0.300, 0.300, 90.0), (0.600, 0.600, 0.0),
                             (0.950, 0.250, 37.0), (0.150, 1.050, -128.0)):
        th = math.radians(th_deg)
        fix = arena_fix_from_scan(_synth_arena_scan(tx, ty, th),
                                  tx + 0.05, ty - 0.05, th + math.radians(6.0))
        ex = abs(fix["x_m"] - tx) if fix["x_m"] is not None else 9.9
        ey = abs(fix["y_m"] - ty) if fix["y_m"] is not None else 9.9
        eh = (abs(wrap_to_pi(fix["heading_rad"] - th))
              if fix["heading_rad"] is not None else 9.9)
        check("fit recovers (%.2f, %.2f, %+.0f deg)" % (tx, ty, th_deg),
              fix["accepted"] and ex < 0.02 and ey < 0.02
              and eh < math.radians(2.0),
              "dx=%.4f dy=%.4f dheading=%.3f deg conc=%.2f rejects=%s"
              % (ex, ey, math.degrees(eh), fix["manhattan_conc"],
                 fix["rejects"] or "none"))

    # M7: a constant range bias must cancel in the position (that is the whole
    # reason both opposite walls are fitted) and show up in the span error.
    #
    # NOTE the span error is NOT exactly 2b - this test was written asserting
    # that it was, and the code disproved it. A ray meeting a wall at angle
    # alpha to its normal has length d/cos(alpha), so adding b to the RANGE
    # moves the point only b*cos(alpha) along the wall's axis. The band mean is
    # therefore 2*b*E[cos alpha], which measures 0.72-0.86 of 2b for a 6 cm
    # band. It is a monotone INDICATOR of range bias, not a calibration of it,
    # and it is left uncorrected for exactly that reason.
    tx, ty, th = 0.400, 0.700, math.radians(20.0)
    prev_span = 0.0
    monotone = True
    for b in (0.01, 0.02, 0.03, 0.05):
        biased = [(r + b) if math.isfinite(r) else r
                  for r in _synth_arena_scan(tx, ty, th)]
        fix = arena_fix_from_scan(biased, tx, ty, th)
        if b == 0.03:
            check("a 30 mm range bias cancels in position",
                  fix["x_m"] is not None and abs(fix["x_m"] - tx) < 0.005
                  and abs(fix["y_m"] - ty) < 0.005,
                  "position error %.4f m"
                  % (math.hypot(fix["x_m"] - tx, fix["y_m"] - ty)
                     if fix["x_m"] is not None else -1))
        se = fix["span_error_x_m"]
        if se is None or se <= prev_span or not (1.2 * b <= se <= 2.0 * b):
            monotone = False
        prev_span = se or 0.0
    check("span error tracks range bias, monotonically, near 2b", monotone,
          "last: bias 0.050 -> span %.4f (2b = 0.100)" % prev_span)

    # A scan that is not a box must be refused, with a reason.
    import random
    random.seed(7)
    noise = [random.uniform(0.3, 2.0) for _ in range(360)]
    fix = arena_fix_from_scan(noise, 0.6, 0.6, 0.0)
    check("random scan is refused with a stated reason",
          not fix["accepted"] and fix["rejects"], "rejects=%s" % fix["rejects"])

    # M5, DEMONSTRATED rather than asserted. A mirrored scan of a SQUARE room
    # still fits a square perfectly - the fit has nothing to complain about -
    # and returns the pose REFLECTED about the arena centre line, heading
    # essentially untouched. The only thing that catches it is the sanity check
    # against dead reckoning, and that check has a blind spot with a size this
    # test pins down exactly.
    tx, ty, th = 0.300, 0.850, math.radians(90.0)
    far = arena_fix_from_scan(_synth_arena_scan(tx, ty, th, mirror=True), tx, ty, th)
    check("M5: mirrored scan far from the centre line is caught",
          not far["accepted"] and far["x_m"] is not None
          and math.hypot(far["x_m"] - tx, far["y_m"] - ty) > LIDAR_MAX_JUMP_M,
          "fit is %.3f m from truth; rejects=%s"
          % (math.hypot(far["x_m"] - tx, far["y_m"] - ty), far["rejects"]))
    # The blind spot: the reflection distance is 2*|coord - 600 mm|, so within
    # 150 mm of the arena centre line it is under LIDAR_MAX_JUMP_M and is
    # ACCEPTED, wrong, silently. That is up to 300 mm of confident error in a
    # 1200 mm arena, which is why M5 is a hard blocker and not a nice-to-have.
    tx, ty, th = 0.700, 0.500, math.radians(90.0)
    near = arena_fix_from_scan(_synth_arena_scan(tx, ty, th, mirror=True), tx, ty, th)
    moved = (math.hypot(near["x_m"] - tx, near["y_m"] - ty)
             if near["x_m"] is not None else 0.0)
    check("M5: near the centre line it is ACCEPTED and wrong (the danger)",
          near["accepted"] and moved > 0.10,
          "accepted with a %.3f m error and no rejects" % moved)

    print("map -> odom, the only correct home for an absolute fix")
    mx, my, myaw = map_to_odom_from_fix(0.6, 0.6, math.radians(90.0),
                                        0.1, 0.0, 0.0)
    check("fix - odom composes back to the fix",
          abs((mx + (math.cos(myaw) * 0.1 - math.sin(myaw) * 0.0)) - 0.6) < 1e-12
          and abs((my + (math.sin(myaw) * 0.1 + math.cos(myaw) * 0.0)) - 0.6) < 1e-12
          and abs(myaw - math.radians(90.0)) < 1e-12)

    print("")
    if failures:
        print("SELFTEST FAILED: %d of the checks above" % len(failures))
        for f in failures:
            print("  - %s" % f)
        return 1
    print("SELFTEST PASSED - pure maths only. Nothing here validates a "
          "physical constant on the rover.")
    print("")
    print("The LiDAR correction remains disabled for these reasons:")
    for reason in lidar_fix_blockers():
        print("  - %s" % reason)
    return 0


def main():
    if "--selftest" in sys.argv[1:]:
        return _selftest()

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
