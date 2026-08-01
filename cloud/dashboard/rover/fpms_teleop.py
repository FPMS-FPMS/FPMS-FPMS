#!/usr/bin/env python3
"""FPMS teleop bridge — MQTT commands in, /cmd_vel out, drive telemetry back.

Runs alongside (never instead of) micro-ros-agent and fpms-rover-agent. It owns
the board's actuator topics — /cmd_vel, /beep, /servo_s1, /servo_s2 — and
nothing else. It does not touch a serial port, it does not restart anything, and
it holds no reference to the other services.

WHY EVERY MOTION COMMAND LIVES IN THIS FILE:

  The rover board is a Yahboom MicroROS Board V2.0 (ESP32-S3) speaking micro-ROS
  on ROS_DOMAIN_ID=20. There is NO Rosmaster_Lib serial protocol on it and there
  never will be — the ESP32 is the ROS node, not a slave to one. Any motor
  handler written against a Rosmaster-style driver can only ever answer "no
  motor interface", which is worse than not answering at all because it reads
  like a hardware fault rather than a category error. Motion, beeper and servos
  are therefore implemented HERE, in the process that already holds the rclpy
  node and the single /cmd_vel publisher, and are not implemented anywhere else.

  Sensor reality on this board, which shapes several replies below:
    * /odom_raw ~10-11 Hz, cumulative pose. No raw encoder ticks are published,
      so `read_encoders` reports pose and twist and says so plainly.
    * /odom_raw's twist.linear.x is SIGN-INVERTED relative to its own
      pose.position — a measured firmware reporting bug, not a wiring fault.
      See ODOM_TWIST_SIGN. Prefer differentiated pose over twist everywhere;
      every guard in this file already does, which is the only reason a day of
      "backward lurches" was a misreading rather than a runaway.
    * /imu ~25 Hz but orientation is NOT fused (identity quaternion), so heading
      is integrated from angular_velocity.z.
    * /battery is UInt16 DECIVOLTS at 1 Hz.
    * /scan is DEAD on this board (every range 0.0). Nothing here reads it.

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
from std_msgs.msg import UInt16, Int32

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


# ==================================================== ODOM TWIST SIGN FIX
# MEASURED on live hardware, wheels off the ground, with teleop stopped so the
# test rig was the ONLY publisher on /cmd_vel:
#
#   commanded linear.x   /odom_raw twist.linear.x   /odom_raw pose displacement
#   ------------------   ------------------------   ---------------------------
#         +0.012              mean -0.842                 +1.395  (FORWARD)
#         +0.100              mean -1.225                 +3.505  (FORWARD)
#         -0.012              mean +0.574                 -1.506  (BACKWARD)
#          0.000                    0                      0      (agree)
#
# /odom_raw's twist.linear.x is SIGN-INVERTED with respect to /odom_raw's own
# pose.position. The two fields of the same message disagree with each other.
# Pose was right in every trial; twist was wrong in every trial; at zero they
# agree, which is why this went unnoticed for so long.
#
# THIS IS A FIRMWARE REPORTING BUG, NOT A MECHANICAL ONE. The drive itself is
# correct and proportional: a positive command moves the rover forward, a
# negative command moves it backward, and the magnitude scales sensibly. Nothing
# is wired backwards, no motor is reversed, and NOTHING ABOUT THE COMMAND PATH
# NEEDS INVERTING. Only the reading of twist.linear.x does. Do not "fix" this by
# negating a command anywhere — that would break a drive that works.
#
# ONE constant, applied at the ONE place twist.linear.x is read (_on_odom).
# Scattering -1 through the readers is how half of them end up corrected and the
# other half do not, which is a worse bug than the original because it is
# intermittent by code path rather than constant.
ODOM_TWIST_SIGN = -1

# Angular twist is NOT corrected: no trial above exercised rotation, so whether
# twist.angular.z shares the inversion is unknown. Assuming it does would be
# guessing, and this file has just spent a day paying for a guess. It is
# reported raw and labelled as uncorrected.
ODOM_TWIST_ANG_SIGN = 1


# Target ACTUAL speeds, in real m/s and rad/s.
JOG_MAX_MPS = 0.05        # full joystick deflection
NUDGE_MPS = 0.04          # F/B 10cm buttons
DOCK_MPS = 0.025          # slow docking near a waypoint
TURN_MAX_RADPS = 0.4      # slow turning
DOCK_TURN_RADPS = 0.2     # closed-loop `turn` command


# ============================================================ DEADBAND FLOOR
# ---- RETRACTION: THE LURCH THIS SECTION WAS BUILT AROUND DID NOT HAPPEN ----
#
# This section used to assert, at length, that a nudge commanded at 0.0041 on
# the wire made the rover sit still for three seconds and then LURCH ~290mm
# BACKWARD, and it concluded that this chassis CANNOT CREEP — that asking it to
# go slower than some deadband made it more dangerous rather than less.
#
# That is now known to be wrong, and it was wrong for an embarrassing reason:
# the "backward" came from /odom_raw's twist.linear.x, which is SIGN-INVERTED
# relative to its own pose (see ODOM_TWIST_SIGN above). The rover moved FORWARD,
# as commanded, the whole time. A clean bench measurement — wheels off, teleop
# stopped, one publisher — showed the drive is correct and PROPORTIONAL down to
# the smallest speed tried:
#
#     commanded +0.012 on the wire -> smooth forward motion, pose +1.395
#     commanded -0.012 on the wire -> smooth backward motion, pose -1.506
#
# There is no evidence of a motor deadband, no evidence of PID windup, and no
# evidence of a stall-then-release. Every claim of the form "this chassis cannot
# creep" is withdrawn. It creeps.
#
# ---- 2026-08-01: THE ABOVE IS TRUE BUT INCOMPLETE, AND LOAD IS WHY -----------
# Those readings were taken under one load condition that was never written
# down, and that omission made them look more general than they are. Measured
# since, on this rover:
#
#     ON THE FLOOR, wheels loaded:  0.00295 on the wire -> NO motion at all
#     WHEELS OFF, free spinning:    0.0010  on the wire -> fast rotation
#
# A smaller command produced motion where a larger one produced none, because
# the two runs differed in load, not in amplitude. So the response is not
# proportional in any way a caller can rely on, and the honest summary is:
#
#     amplitude alone does not predict motion; load has to be stated with it.
#
# Practical consequence for anything wanting SLOW: short bounded pulses with
# full stops between them (~0.35 s was the shortest that moved anything, free
# spinning) rather than a smaller setpoint. That number is ALSO free-spinning
# and must be re-measured under load before any distance is computed from it.
# See SESSION_HANDOFF.md, 2026-08-01.
#
# ---- WHY THE MECHANISM STAYS ANYWAY ----------------------------------------
# The floor is kept, and it is kept DISABLED (default 0.0). Deleting it outright
# would be overcorrecting in the other direction: no test has yet gone below
# 0.012 on the wire, so a real deadband may still exist somewhere underneath
# that, and if a clean sweep ever finds one, the fix should be a config.env line
# and not a code change made in a hurry. What is NOT justified is a floor
# switched on by default on the strength of an artefact.
#
# ---- UNMEASURED — PENDING A CLEAN SWEEP ------------------------------------
# Nothing below the smallest tested command has been characterised. The sweep
# that would settle it: with the wheels off the ground and nothing else
# publishing to /cmd_vel, walk linear.x up from 0.001 in 0.001 steps and record
# POSE DISPLACEMENT (never twist) at each step; the first step with smooth
# proportional pose motion is the true floor. Until that is run:
#
#   MIN_CMD_LIN = 0.0  (floor OFF — no linear deadband has been demonstrated)
#   MIN_CMD_ANG = 0.0  (floor OFF — rotation has never been characterised at
#                       all, in either direction, so a floor here would be a
#                       guess layered on a guess)
#
# With both at 0.0, snap_up() is an identity apart from its clamp and commands
# pass through at the speed the operator actually asked for. The acceleration
# ramp is once again continuous from zero, as it should be.
#
# Overridable from /etc/fpms/config.env without editing this file, which is the
# intended way to act on a sweep result:
#   FPMS_MIN_CMD_LIN=0.02
#   FPMS_MIN_CMD_ANG=0.15
# Set either non-zero and every command below it is raised to it (sign
# preserved; exact zero always stays exact zero — see snap_up).
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


# Defaults are 0.0 — floor OFF, pending a clean sweep. See the retraction above.
# The upper bound is the motion envelope itself: a floor above the clamp would
# make the floor the only speed the rover has. snap_up() re-clamps regardless,
# so this is belt and braces.
MIN_CMD_LIN = _cfg_float("FPMS_MIN_CMD_LIN", 0.0, 0.0, JOG_MAX_MPS)
MIN_CMD_ANG = _cfg_float("FPMS_MIN_CMD_ANG", 0.0, 0.0, TURN_MAX_RADPS)

# `set_speed` still needs a lower bound to nack against, and with the deadband
# floor at 0.0 it cannot use that. This is NOT a deadband claim: it is a
# usability bound. Below 5 mm/s a 100mm nudge takes over twenty seconds and
# every timeout in this file would need re-deriving, so a value under it is
# almost certainly a units mistake and is refused with an explanation. If a
# sweep ever establishes a real deadband above this, MIN_CMD_LIN wins — the
# effective bound is the larger of the two.
SET_SPEED_MIN_MPS = 0.005

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
# These were written in response to a "stall then lurch backward" that has since
# been shown to be a sign-inversion artefact (see ODOM_TWIST_SIGN). The fault
# they were built for does not exist.
#
# They are kept, unchanged, because what they actually check is still worth
# checking and is independent of that story: "I am commanding motion and the
# POSE is not changing" means a jammed wheel, a lifted chassis, or a dead motor
# driver, and "the pose is moving opposite to the command" means something is
# genuinely wrong. Both read differentiated pose, never twist, so neither is
# affected by the inversion — verified by inspection when the inversion was
# found, and the reason nothing here had to be re-signed.
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
# Two shapes:
#   * driving the wrong way    — commanded forward, pose says backward
#   * moving with no command   — cmd_vel is zero and the rover is still going
#
# NEITHER SHAPE HAS EVER ACTUALLY BEEN OBSERVED. The episode that motivated this
# guard was an inverted twist reading, not a runaway. The guard stays because
# the failure it describes is real in principle and cheap to check for — but it
# MUST be fed from differentiated pose (self.odom_vx) and never from
# twist.linear.x. Fed from twist, it would read every correct forward move as a
# reverse one and abort all normal motion within LURCH_CONFIRM_S. That is not
# hypothetical: it is precisely what the inverted field would have caused.
LURCH_MIN_MPS = 0.02      # below this, odometry is noise, not motion
LURCH_CONFIRM_S = 0.25    # must persist this long — one bad frame must not stop
                          # the rover, and one good frame must not excuse a trip
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

# --- wire ownership ---
# fpms-missions publishes telemetry/mission at TELEM_HZ while it exists. Seeing
# one that recently means it is running, so teleop stops writing to /cmd_vel and
# stops answering the `mission` verb.
#
# This exists because BOTH nodes publish Twist. fpms_missions.py explains it at
# length: interleaving teleop's 2 Hz idle zeros into a mission's 20 Hz setpoint
# reproduces exactly the stall-then-lurch this project spent hours misdiagnosing,
# so the mission node REFUSES TO START while it hears a foreign writer. Without
# this constant a mission can never begin — the refusal is correct, and teleop is
# the one that has to yield.
#
# THIS VALUE IS COUPLED TO fpms_missions.py's IDLE_TELEM_S (5.0 s) AND MUST STAY
# COMFORTABLY LARGER THAN IT. That coupling is not obvious and it is not
# cosmetic, so it is spelled out here rather than left to be rediscovered:
#
# While no mission is running, the executor deliberately slows its telemetry to
# one message every IDLE_TELEM_S. If this window is shorter than that interval,
# teleop reclaims the wire between heartbeats and emits an idle zero — and the
# executor, which refuses to start whenever it hears a foreign writer on
# /cmd_vel, then refuses EVERY mission. The failure is a permanent deadlock that
# reads as "the rover ignores the button", with both services healthy and
# nothing in either log that looks like an error.
#
# Set at 12 s: long enough to ride out a dropped idle heartbeat (a 10 s gap),
# short enough that teleop resumes its own heartbeat promptly if the executor
# actually dies. The asymmetry is deliberate — being slow to reclaim an idle
# wire costs nothing, while reclaiming it early puts two writers on a moving
# rover, which is the stall-then-lurch failure this whole mechanism exists to
# prevent.
MISSION_OWNS_WIRE_S = 12.0


# ======================================================= RUNTIME TUNING CAPS
# JOG_MAX_MPS and NUDGE/DOCK_MPS above are DEFAULTS. `set_speed` lets an
# operator retune them at runtime without editing this file or restarting the
# service — which is the point, because the sweep that would characterise the
# low end of this drive is a field procedure, not a code change.
#
# What set_speed may NOT do is leave the envelope this file was reviewed
# against, so every runtime value is clamped into [floor, hard cap] here:
#   * the floor is max(MIN_CMD_LIN, SET_SPEED_MIN_MPS). Below it, a nack rather
#     than a silent clamp — silently raising a value teaches the operator that
#     the dial does nothing. With MIN_CMD_LIN now defaulting to 0.0, in practice
#     the bound is the SET_SPEED_MIN_MPS sanity limit.
#   * the hard cap is fixed here and is not itself tunable from anywhere. It is
#     the number that makes "set_speed cannot make the rover dangerous" a
#     property of the code rather than of the operator's typing.
HARD_MAX_LIN_MPS = 0.12   # ~2.4x the default jog max; still a walking pace
HARD_MAX_ANG_RADPS = TURN_MAX_RADPS   # angular is not runtime-tunable at all

# --- motor self-test --------------------------------------------------------
# A BOUNDED self-test: four short legs (fwd, rev, spin left, spin right) with
# odometry recorded either side of each so the reply states what the chassis
# actually did, not what it was asked to do. Everything about it is capped:
# there is no payload that makes it run long or fast.
TEST_MAX_LEG_S = 3.0      # per leg, hard
TEST_MIN_LEG_S = 0.3
TEST_DEFAULT_LEG_S = 1.0
TEST_SETTLE_S = 0.8       # zero-command coast+measure window between legs
TEST_POLL_S = 0.02        # abort responsiveness: <=20ms, well inside the 100ms
                          # requirement, and the worker never holds the lock
                          # across a sleep so the control tick is never delayed
TEST_MAX_TOTAL_S = 30.0   # belt and braces: worker self-destructs past this

# --- beeper -----------------------------------------------------------------
# Yahboom firmware convention on /beep (UInt16): 0 = off, 1 = on until told
# otherwise, and any value >= 10 = beep for roughly that many milliseconds.
# A latched-on beeper is an annoyance nobody can silence from the dashboard if
# the link then drops, so `beep` never sends 1: it sends 0 or a bounded ms.
BEEP_MIN_MS = 10
BEEP_MAX_MS = 2000
BEEP_DEFAULT_MS = 200

# --- servos -----------------------------------------------------------------
# /servo_s1 and /servo_s2 are Int32 degrees. The firmware accepts 0..180, but
# 0 and 180 are the mechanical end stops: a hobby servo parked hard against its
# stop stalls, draws locked-rotor current continuously and cooks itself, and on
# this rover it would be doing that off the same pack that has to stop the
# motors. The range below is therefore deliberately INSET from the firmware's:
#   SERVO_MIN_DEG = 10, SERVO_MAX_DEG = 170  (centre 90)
# 10 degrees of margin at each end is enough to guarantee the horn never loads
# the stop while giving up almost none of the useful travel. Out-of-range values
# are CLAMPED (not refused) and the ack states both what was asked and what was
# sent, because a servo request is not a motion command and refusing it outright
# would be more surprising than honouring it at the limit.
# Overridable from config.env: FPMS_SERVO_MIN_DEG / FPMS_SERVO_MAX_DEG.
SERVO_MIN_DEG = int(_cfg_float("FPMS_SERVO_MIN_DEG", 10.0, 0.0, 90.0))
SERVO_MAX_DEG = int(_cfg_float("FPMS_SERVO_MAX_DEG", 170.0, 90.0, 180.0))

# --- ping -------------------------------------------------------------------
# A client clock more than an hour off is not a latency measurement, it is a
# wrong clock; say so rather than reporting a 3-week round trip.
PING_MAX_SKEW_S = 3600.0


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


# ======================================================== ARGUMENT VALIDATION
# Everything below is PURE: no ROS, no MQTT, no self, no clock except one that
# is passed in. That is not tidiness, it is so the parts of this file that decide
# how fast and how far the rover moves can be exercised on a laptop while the
# hardware is busy, which is the only way they get tested at all.
#
# They raise ValueError with a message written for the operator, and the command
# handlers turn that message straight into the `error` field of a nack. There is
# no path where a bad argument is silently defaulted.

class ArgError(ValueError):
    """A payload the operator can fix. The message becomes the nack reason."""


def num_arg(payload, key, default=None):
    """Finite float from a JSON payload, or ArgError naming the offender.

    bool is rejected explicitly: True would otherwise sail through float() as
    1.0, and {"speed": true} meaning "0.05 m/s" is not a reading anyone intends.
    """
    raw = payload.get(key, default)
    if raw is None:
        raw = default
    if raw is None:
        raise ArgError(f"{key} is required")
    if isinstance(raw, bool):
        raise ArgError(f"{key} must be a number, not a boolean")
    if isinstance(raw, str):
        raw = raw.strip()
    try:
        v = float(raw)
    except Exception:
        raise ArgError(f"{key}={payload.get(key)!r} is not numeric")
    if not math.isfinite(v):
        raise ArgError(f"{key}={payload.get(key)!r} is not a finite number")
    return v


def clamp_note(name, asked, lo, hi, unit=""):
    """(clamped value, note or None). The note is what the ack says out loud.

    Clamping quietly is how an operator ends up believing a dial works. Every
    caller of this puts the returned note in its reply.
    """
    v = clamp(asked, lo, hi)
    if v != asked:
        return v, (f"{name} {asked:g}{unit} clamped to {v:g}{unit} "
                   f"(allowed {lo:g}..{hi:g}{unit})")
    return v, None


def parse_test_motors(payload, floor_lin, cap_lin, floor_ang, cap_ang,
                      min_leg_s=TEST_MIN_LEG_S, max_leg_s=TEST_MAX_LEG_S,
                      default_leg_s=TEST_DEFAULT_LEG_S):
    """Validate `test_motors` {"speed":.., "duration_s":..}.

    Returns {"speed_mps", "ang_radps", "duration_s", "notes"}.

    speed is taken as a magnitude — a self-test drives all four directions by
    construction, so a negative "speed" is a unit confusion, not a request to
    run the legs backwards, and it is treated as its magnitude rather than
    quietly inverting the test. Zero is refused outright because a self-test
    that does not move proves nothing while still taking the motors.

    The angular leg speed is derived, not accepted from the payload: there is no
    argument by which the operator can spin this test faster than the same
    fraction of the angular envelope that `speed` is of the linear one.
    """
    speed = abs(num_arg(payload, "speed", floor_lin))
    if speed <= 0.0:
        raise ArgError("speed must be > 0 (a self-test that does not move "
                       "tells you nothing)")
    dur = num_arg(payload, "duration_s", default_leg_s)
    if dur <= 0.0:
        raise ArgError("duration_s must be > 0")

    notes = []
    speed, n = clamp_note("speed", speed, max(floor_lin, 1e-9), cap_lin, " m/s")
    if n:
        notes.append(n)
    dur, n = clamp_note("duration_s", dur, min_leg_s, max_leg_s, "s")
    if n:
        notes.append(n)

    # Angular leg: same fraction of the angular envelope as the linear leg is of
    # the linear envelope, then floored/capped in angular units on its own terms.
    frac = speed / cap_lin if cap_lin > 0 else 1.0
    ang = clamp(frac * cap_ang, max(floor_ang, 1e-9), cap_ang)
    return {"speed_mps": speed, "ang_radps": ang, "duration_s": dur,
            "notes": notes}


def parse_beep(payload, min_ms=BEEP_MIN_MS, max_ms=BEEP_MAX_MS,
               default_ms=BEEP_DEFAULT_MS):
    """Validate `beep` {"ms":..}. Returns (ms:int, notes:list).

    0 is legal and means "silence now". Anything else is pulled up to min_ms and
    down to max_ms: below ~10ms the firmware's own convention reads the value as
    a mode flag rather than a duration, and 1 latches the beeper on forever.
    """
    ms = num_arg(payload, "ms", default_ms)
    if ms < 0:
        raise ArgError("ms must be >= 0 (0 means silence)")
    notes = []
    ms = int(round(ms))
    if ms == 0:
        return 0, notes
    if ms < min_ms:
        notes.append(f"ms {ms} raised to {min_ms} (values 1..{min_ms - 1} are "
                     f"mode flags to this firmware, not durations; 1 would "
                     f"latch the beeper on)")
        ms = min_ms
    elif ms > max_ms:
        notes.append(f"ms {ms} clamped to {max_ms}")
        ms = max_ms
    return ms, notes


def parse_servo(payload, lo_deg=SERVO_MIN_DEG, hi_deg=SERVO_MAX_DEG):
    """Validate `servo` {"which":1|2, "angle":..}. Returns (which, angle, notes)."""
    which_raw = payload.get("which", 1)
    if isinstance(which_raw, bool):
        raise ArgError("which must be 1 or 2, not a boolean")
    if isinstance(which_raw, str):
        which_raw = which_raw.strip().lower().lstrip("s")
    try:
        which = int(float(which_raw))
    except Exception:
        raise ArgError(f"which={payload.get('which')!r} is not 1 or 2")
    if which not in (1, 2):
        raise ArgError(f"which={payload.get('which')!r} is not 1 or 2 "
                       f"(this board exposes /servo_s1 and /servo_s2 only)")
    angle = num_arg(payload, "angle")
    notes = []
    ang_i = int(round(angle))
    ang_c, n = clamp_note("angle", ang_i, lo_deg, hi_deg, " deg")
    if n:
        notes.append(n + "; the range is inset from the firmware's 0..180 so "
                         "the horn never parks against a mechanical end stop "
                         "and stalls")
    return which, int(ang_c), notes


def parse_set_speed(payload, floor_lin, hard_lin):
    """Validate `set_speed` {"jog_max":.., "nudge":..}. Returns (updates, notes).

    Both values are REAL m/s, the same units the acks and telemetry report, so
    an operator can copy a number straight out of telemetry into a set_speed.

    Below `floor_lin` is a NACK, never a clamp. That asymmetry is deliberate:
    if a deadband floor is configured, `snap_up` would raise such a value on the
    way to the wire anyway, so accepting it would show the operator a setting of
    0.02 m/s while the rover ran at something else. A dial that reads back a
    number the rover is not using is how a whole day gets spent chasing a fault
    that is really a units mismatch. Above the cap IS clamped (with a note),
    because "as fast as you'll let me" is an intelligible request while "a speed
    the configuration forbids" is not.
    """
    if not any(k in payload for k in ("jog_max", "nudge")):
        raise ArgError("nothing to set: expected jog_max and/or nudge "
                       "(real m/s)")
    updates, notes = {}, []
    for key, label in (("jog_max", "jog_max"), ("nudge", "nudge")):
        if key not in payload:
            continue
        v = num_arg(payload, key)
        if v <= 0.0:
            raise ArgError(f"{label} must be > 0 m/s (use `stop` to stop)")
        if v < floor_lin:
            raise ArgError(
                f"{label} {v:g} m/s is below the minimum this bridge will "
                f"accept ({floor_lin:g} m/s). Refused rather than silently "
                f"raised, so the setting you read back is the speed the rover "
                f"uses. The bound is the larger of the configured deadband "
                f"floor (FPMS_MIN_CMD_LIN) and a sanity limit; check for a "
                f"units mistake — these are m/s, not mm/s.")
        v, n = clamp_note(label, v, floor_lin, hard_lin, " m/s")
        if n:
            notes.append(n)
        updates[key] = v
    if "jog_max" in updates and "nudge" in updates:
        if updates["nudge"] > updates["jog_max"]:
            raise ArgError(
                f"nudge {updates['nudge']:g} m/s exceeds jog_max "
                f"{updates['jog_max']:g} m/s; the hard clamp is jog_max, so "
                f"the nudge would be clipped to it anyway")
    return updates, notes


def summarize_leg(name, vx_cmd, wz_cmd, before, after,
                  min_mm=STALL_MIN_MM, min_deg=TURN_STALL_MIN_DEG):
    """Turn a before/after odometry pair into what a self-test leg ACTUALLY did.

    `before` and `after` are the snapshots taken either side of the leg:
    {"t","x","y","yaw","yaw_int","odom_seq"}. Any of x/y may be None if odometry
    was missing, in which case this says so rather than reporting 0.0mm — 0.0mm
    and "no idea" are very different self-test results.

    Displacement is decomposed against the heading the leg STARTED from, so
    `forward_mm` is signed by the direction the leg was commanded in — a plain
    hypot() would report a healthy distance for a leg that ran backwards, and a
    self-test that can be fooled is worse than no self-test because its output
    is what someone will trust. It is computed from POSE, never from
    twist.linear.x, which on this board is sign-inverted (ODOM_TWIST_SIGN).

    Rotation is reported from BOTH the integrated gyro and the odometry yaw.
    They are independent (the board does not fuse orientation, so odom yaw is
    dead-reckoned from the wheels while yaw_int comes off the IMU), and a
    disagreement between them is exactly what wheel slip looks like.
    """
    out = {
        "leg": name,
        "cmd_vx_mps": jnum(vx_cmd, 4),
        "cmd_wz_radps": jnum(wz_cmd, 4),
        "elapsed_s": jnum(after.get("t", 0.0) - before.get("t", 0.0), 2),
        "odom_frames": int(after.get("odom_seq", 0) - before.get("odom_seq", 0)),
    }
    if before.get("x") is None or after.get("x") is None:
        out.update({"distance_mm": None, "forward_mm": None, "lateral_mm": None,
                    "gyro_deg": None, "odom_yaw_deg": None, "ok": False,
                    "verdict": "no odometry recorded for this leg"})
        return out

    dx = after["x"] - before["x"]
    dy = after["y"] - before["y"]
    h = before.get("yaw", 0.0)
    along = dx * math.cos(h) + dy * math.sin(h)
    lateral = -dx * math.sin(h) + dy * math.cos(h)
    dist = math.hypot(dx, dy)
    gyro_deg = math.degrees(after.get("yaw_int", 0.0) - before.get("yaw_int", 0.0))
    odom_deg = wrap180(math.degrees(after.get("yaw", 0.0) - before.get("yaw", 0.0)))

    spin = abs(wz_cmd) > abs(vx_cmd) * 10.0 or (vx_cmd == 0.0 and wz_cmd != 0.0)
    if spin:
        signed = gyro_deg * (1.0 if wz_cmd >= 0 else -1.0)
        if abs(gyro_deg) < min_deg:
            ok, verdict = False, (f"no rotation ({gyro_deg:+.1f}deg < "
                                  f"{min_deg:g}deg); firmware deadband or a "
                                  f"stalled motor")
        elif signed < 0:
            ok, verdict = False, (f"rotated the WRONG WAY: commanded "
                                  f"{wz_cmd:+.3f} rad/s, measured "
                                  f"{gyro_deg:+.1f}deg")
        else:
            ok, verdict = True, "ok"
    else:
        signed_mm = along * 1000.0 * (1.0 if vx_cmd >= 0 else -1.0)
        if dist * 1000.0 < min_mm:
            ok, verdict = False, (f"no motion ({dist*1000:.0f}mm < {min_mm:g}mm); "
                                  f"firmware deadband or a stalled motor")
        elif signed_mm < 0:
            ok, verdict = False, (f"drove the WRONG WAY: commanded "
                                  f"{vx_cmd:+.3f} m/s, measured "
                                  f"{along*1000:+.0f}mm along the start heading")
        else:
            ok, verdict = True, "ok"

    out.update({
        "distance_mm": jnum(dist * 1000.0, 1),
        # signed along the heading the leg started from, not an absolute
        "forward_mm": jnum(along * 1000.0, 1),
        "lateral_mm": jnum(lateral * 1000.0, 1),
        "gyro_deg": jnum(gyro_deg, 1),
        "odom_yaw_deg": jnum(odom_deg, 1),
        "ok": bool(ok),
        "verdict": verdict,
    })
    return out


def ping_timing(t_raw, now_s, max_skew_s=PING_MAX_SKEW_S):
    """Interpret a client timestamp. Returns (t_echo, units, uplink_ms).

    Accepts epoch seconds or epoch milliseconds and works out which by seeing
    which one lands near now. If neither does, the value is echoed verbatim with
    units "unknown" and uplink_ms None — the client's clock is wrong, and
    reporting a 3-week uplink latency because of it would be worse than
    reporting nothing. `t` may also be absent entirely, which is valid.
    """
    if t_raw is None:
        return None, None, None
    if isinstance(t_raw, bool):
        return None, "unknown", None
    try:
        t = float(t_raw)
    except Exception:
        return t_raw, "unknown", None
    if not math.isfinite(t):
        return None, "unknown", None
    if abs(now_s - t) <= max_skew_s:
        return t, "s", (now_s - t) * 1000.0
    if abs(now_s * 1000.0 - t) <= max_skew_s * 1000.0:
        return t, "ms", (now_s * 1000.0 - t)
    return t, "unknown", None


# =================================================== THE COMMAND SET, ONE LIST
# Subscribed by exact name, never with a '#' wildcard, and this is the single
# list that both the subscriber and the `status`/`online` replies are built from
# — so a verb cannot end up implemented-but-unsubscribed or advertised-but-
# missing. fpms-rover-agent is on fpms/<thing>/commands/# and nacks whatever it
# does not know; taking only these verbs is what stops the two agents from both
# claiming the same message.
#
# NOTE ON EVENT SUBTYPES: the backend's email_alerts matcher fires a FIRE
# WARNING on any event subtype containing "fire" or "alert". Nothing here may
# use either word — hence "motor_test", not "motor_alert".
#
# ---- VERBS DELIBERATELY NOT CLAIMED HERE -----------------------------------
# `ping`, `status`, `connect` and `disconnect` belong to fpms-rover-agent and
# are NOT in this map. Both processes subscribe the same broker; if both
# answered, Drive.tsx resolves a command on the FIRST reply for a (thing,
# action) pair, so which of two different answers the operator saw would be a
# race. rover-agent wins those four because it has no ROS dependency and keeps
# answering when the micro-ROS link is down — which is exactly when an operator
# most needs ping and status to work. A teleop bridge that goes quiet during a
# ROS outage is the worst possible owner of "is anything alive?".
#
# The teleop-specific equivalents use DISTINCT verbs so they can coexist:
#   drive_status                  -> this bridge's motion/envelope snapshot
#   drive_connect/drive_disconnect -> this bridge's telemetry stream only
#
# `auto_on` is also not claimed. rover-agent nacks it and points at teleop's
# `mission`; there is no autonomy loop in this file to enable, and an ack that
# set a latch while nothing autonomous existed is exactly the bug that was just
# removed from rover-agent. Claiming it here would either re-create that lie or
# double-answer the nack.
#
# rover/test_commands.py parses this dict to assert the two processes never
# claim the same verb. Keep it a plain literal of string keys.
#
# ---- COMPANION CHANGE REQUIRED IN fpms_rover_agent.py ----------------------
# That agent subscribes fpms/<thing>/commands/# and nacks anything it does not
# recognise, staying silent only for verbs listed in its SILENT_VERBS tuple.
# The three drive_* verbs below are new and are NOT yet in that tuple, so until
# they are added, rover-agent answers them with "unknown command" and races this
# bridge's real reply. test_commands.py fails on exactly this ("every teleop
# verb is silenced here or owned here"), which is the check doing its job.
#
# The fix is one line in fpms_rover_agent.py — a file this change was scoped out
# of — appending to SILENT_VERBS:
#     "drive_status", "drive_connect", "drive_disconnect",
# Nothing else needs to move: jog/nudge/turn/stop/estop/auto_off/mission/
# set_coordinate/set_speed/test_motors/read_encoders/beep/servo are already
# silenced there, and ping/status/connect/disconnect are deliberately NOT
# claimed here precisely so rover-agent can keep answering them.
TELEOP_ACTIONS = {
    # motion (gated: link + battery)
    "jog": "events/ack",
    "nudge": "events/ack",
    "turn": "events/ack",
    "test_motors": "events/motor_test",
    # stopping (never gated, ever)
    "stop": "events/ack",
    "estop": "events/ack",
    # auto_off is a stop by another name, so it is claimed here: it must work
    # through the same ungated path as stop.
    "auto_off": "events/ack",
    # actuators
    "beep": "events/ack",
    "servo": "events/ack",
    # reads and admin
    "drive_status": "events/drive_status",
    "read_encoders": "events/encoders",
    "set_speed": "events/ack",
    "drive_connect": "events/ack",
    "drive_disconnect": "events/ack",
    "mission": "events/ack",
    "set_coordinate": "events/ack",
}


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
        # Monotonic stamp of the last telemetry/mission seen. This is how teleop
        # knows fpms-missions is alive and therefore owns the wire. See
        # mission_active(), the heartbeat suppression in _control_tick, and
        # _cmd_mission.
        self._mission_seen = 0.0

    def mission_active(self):
        """True if fpms-missions published recently enough to own /cmd_vel.

        Deliberately a freshness check rather than a flag: if the mission node
        dies mid-run its telemetry simply stops, and teleop resumes ownership on
        its own a few seconds later. A latched boolean would leave the wire
        orphaned until something thought to clear it.
        """
        return (time.monotonic() - self._mission_seen) < MISSION_OWNS_WIRE_S

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
            # Subscribed by name from TELEOP_ACTIONS, not with a '#' wildcard.
            # See the comment on that dict for why.
            for action in TELEOP_ACTIONS:
                client.subscribe(f"fpms/{THING}/commands/{action}", qos=1)
            # Not a command — this is how teleop notices fpms-missions running
            # and yields /cmd_vel to it. qos=0: a dropped sample only delays the
            # handover by one tick, and MISSION_OWNS_WIRE_S covers that.
            client.subscribe(f"fpms/{THING}/telemetry/mission", qos=0)
            self.publish("events/online",
                         {"svc": "teleop", "status": "online",
                          "cmd_scale": CMD_SCALE,
                          # The dashboard should build its button set from this
                          # rather than from a hardcoded list that can drift.
                          "actions": sorted(TELEOP_ACTIONS),
                          "reply_topics": dict(TELEOP_ACTIONS),
                          "board": "Yahboom MicroROS Board V2.0 (ESP32-S3)",
                          "limits": {"jog_max_mps": JOG_MAX_MPS,
                                     "turn_max_radps": TURN_MAX_RADPS,
                                     "dock_mps": DOCK_MPS,
                                     "dock_turn_radps": DOCK_TURN_RADPS,
                                     "hard_max_lin_mps": HARD_MAX_LIN_MPS,
                                     "hard_max_ang_radps": HARD_MAX_ANG_RADPS,
                                     # The floors are limits too — they are the
                                     # bottom of the envelope, not the top.
                                     "min_cmd_lin": MIN_CMD_LIN,
                                     "min_cmd_ang": MIN_CMD_ANG,
                                     "servo_deg": [SERVO_MIN_DEG, SERVO_MAX_DEG],
                                     "beep_ms": [BEEP_MIN_MS, BEEP_MAX_MS],
                                     "test_leg_s": [TEST_MIN_LEG_S, TEST_MAX_LEG_S],
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
            # MUST be tested on the full topic, before the action is derived.
            # `telemetry/mission` and `commands/mission` both end in "mission",
            # so the rsplit below cannot tell them apart — routing on it would
            # feed every mission telemetry sample back in as a mission COMMAND.
            if msg.topic.endswith("/telemetry/mission"):
                self._mission_seen = time.monotonic()
                return
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
        # The other two actuators this board exposes. Created up front rather
        # than lazily inside a command handler so that a `beep` or `servo`
        # arriving on paho's thread never constructs ROS entities from a
        # non-ROS thread.
        self.pub_beep = self.create_publisher(UInt16, "/beep", qos)
        self.pub_servo = {
            1: self.create_publisher(Int32, "/servo_s1", qos),
            2: self.create_publisher(Int32, "/servo_s2", qos),
        }
        self.create_subscription(Odometry, "/odom_raw", self._on_odom, qos)
        self.create_subscription(Imu, "/imu", self._on_imu, qos)
        self.create_subscription(UInt16, "/battery", self._on_battery, qos)
        # /scan is deliberately NOT subscribed: every range on this board reads
        # 0.0, so a subscription would only produce a plausible-looking stream of
        # "obstacle at 0m" that some future guard would act on.

        # --- motion state (all guarded by self.lock) ---
        self.mode = "idle"           # idle | jog | nudge | turn | test
        self.cur_vx = 0.0            # real m/s, post-ramp
        self.cur_wz = 0.0            # real rad/s, post-ramp
        self.jog_vx = 0.0            # normalized -1..1
        self.jog_wz = 0.0
        self.jog_ts = 0.0            # monotonic, last jog message
        self.jog_warned = False
        self.nudge = None
        self.turn = None

        # --- runtime tuning (set_speed) ---
        # Live copies of the two speeds an operator may retune. The module
        # constants remain the DEFAULTS and are still what the file documents;
        # these are what the control tick actually reads. Both are re-clamped
        # into [MIN_CMD_LIN, HARD_MAX_LIN_MPS] on every write, so there is no
        # state either can be in that the hard clamp has not already approved.
        self.jog_max_mps = clamp(JOG_MAX_MPS, 0.0, HARD_MAX_LIN_MPS)
        self.nudge_mps = clamp(DOCK_MPS, 0.0, HARD_MAX_LIN_MPS)
        self.tuned_by_operator = False

        # --- motor self-test ---
        # Runs on its own thread. It NEVER publishes a Twist and never touches
        # cur_vx/cur_wz: it sets a target the control tick picks up, exactly
        # like every other mode, so the ramp, the clamp, the floor, the lurch
        # guard and the link watchdog all still stand between it and the wheels.
        self.selftest = None         # {"vx","wz","leg","token","until"}
        self._test_thread = None
        self._test_abort = threading.Event()
        self._test_abort_reason = None
        self._test_token = 0
        self.last_test = None        # summary of the most recent run

        # --- telemetry streaming (connect / disconnect) ---
        # Gates ONLY this node's telemetry/drive stream. It does not stop
        # commands being accepted, does not stop acks/nacks (an operator who has
        # muted telemetry must still be told his stop worked), and does not
        # touch micro-ros-agent or fpms-rover-agent in any way.
        self.telemetry_enabled = True
        self.telemetry_suppressed = 0

        # --- odom ---
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = 0.0
        self.odom_last = 0.0
        self.odom_times = deque(maxlen=40)
        self.odom_vx = 0.0           # signed real m/s along heading, from deltas
        self._odom_prev = None       # (x, y, yaw, t) of the previous frame
        # The board's OWN twist estimate off /odom_raw, SIGN-CORRECTED by
        # ODOM_TWIST_SIGN on the way in. Not used by any guard (see _on_odom for
        # why, and note that the reason is now measured rather than merely
        # cautious), but it is the closest thing to an encoder readout this
        # firmware publishes, so read_encoders reports it. The raw field is kept
        # alongside so the inversion stays visible to anyone re-measuring it.
        self.odom_twist_vx = 0.0
        self.odom_twist_vx_raw = 0.0
        self.odom_twist_wz = 0.0
        self.odom_seq = 0            # frames seen since boot

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
        self._yielded_wire = False   # log the handover once, not 20x/second
        self._link_warned = False
        self._jog_nack_at = 0.0
        self._deadband_snapped = False
        self._lurch_since = 0.0      # when the current suspicion started
        self._lurch_nack_at = 0.0
        self._cmd_zero_since = 0.0   # when cmd_vel last became zero
        self.lurch_trips = 0

        for note in CFG_NOTES:
            log(note)
        log(f"deadband floor: lin {MIN_CMD_LIN} m/s, ang {MIN_CMD_ANG} rad/s"
            + (" (both OFF — no deadband demonstrated; clean sweep not yet run)"
               if (MIN_CMD_LIN <= 0.0 and MIN_CMD_ANG <= 0.0) else
               " (configured via config.env)"))
        log(f"odom twist sign correction: linear x{ODOM_TWIST_SIGN} "
            f"(firmware reports twist inverted vs its own pose), "
            f"angular x{ODOM_TWIST_ANG_SIGN} (uncorrected, unmeasured)")

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
                # Signed ground speed along the PREVIOUS heading, DIFFERENTIATED
                # FROM POSITION rather than read out of msg.twist.
                #
                # That choice was made defensively — the board publishes an
                # identity orientation, so its state estimate was already known
                # to be partial, and a guard whose job is to contradict the
                # firmware cannot be built on the firmware's opinion of its own
                # velocity. It turned out to be load-bearing: msg.twist.linear.x
                # is sign-inverted relative to pose (see ODOM_TWIST_SIGN), so
                # every guard fed from twist would have read a correct forward
                # move as a reverse one and aborted it. Every guard in this file
                # is fed from THIS number, and pose was correct in every trial.
                # Keep it that way: prefer differentiated pose over twist.
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
                self.odom_seq += 1
                # THE ONLY PLACE twist.linear.x IS READ IN THIS FILE, and
                # therefore the only place ODOM_TWIST_SIGN is applied. Every
                # consumer downstream (read_encoders, telemetry) sees the
                # corrected value and does not know a correction happened —
                # which is the point: a reader that has to remember to negate is
                # a reader that will forget.
                try:
                    tw = msg.twist.twist
                    if math.isfinite(tw.linear.x):
                        self.odom_twist_vx = ODOM_TWIST_SIGN * float(tw.linear.x)
                        self.odom_twist_vx_raw = float(tw.linear.x)
                    if math.isfinite(tw.angular.z):
                        # Uncorrected on purpose — see ODOM_TWIST_ANG_SIGN.
                        self.odom_twist_wz = (ODOM_TWIST_ANG_SIGN
                                              * float(tw.angular.z))
                except Exception:
                    pass
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
                        # A running self-test is halted like anything else. The
                        # worker notices the token change on its next 20ms poll
                        # and reports the run as aborted rather than finished.
                        self._abort_test_unlocked("micro-ROS link down")
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
                #
                # Two stages since set_speed exists: the runtime limit first
                # (what the operator currently asked the envelope to be), then
                # the fixed ceiling this file was reviewed against. The second
                # clamp does not trust the first — jog_max_mps is clamped on
                # write too, so this is the third independent place a runaway
                # value would have to survive.
                lin_cap = min(self.jog_max_mps, HARD_MAX_LIN_MPS)
                self.cur_vx = clamp(self.cur_vx, -lin_cap, lin_cap)
                self.cur_vx = clamp(self.cur_vx, -HARD_MAX_LIN_MPS, HARD_MAX_LIN_MPS)
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

            idle = (mode == "idle" and abs(vx) < 1e-6 and abs(wz) < 1e-6)

            # ---- wire handover to fpms-missions -----------------------------
            # While a mission is running and this operator is not actively
            # driving, teleop says NOTHING on /cmd_vel. Both the idle heartbeat
            # and the lurch auto-zero are suppressed.
            #
            # The lurch suppression is the non-obvious half and it is not a
            # weakening of the guard. That guard fires on "the rover is moving
            # and teleop did not command it" — which is the precise description
            # of a mission in progress. Left armed it would zero the wire
            # continuously and no mission could ever run.
            #
            # Nothing is left unguarded. The mission node carries its own
            # obstacle, battery and link-loss aborts, and it honours stop/estop
            # directly. An operator who grabs the joystick takes the wire back
            # immediately, because `idle` goes false and this branch stops
            # applying — deliberate override stays possible at any moment.
            if idle and self.bus.mission_active():
                if not self._yielded_wire:
                    self._yielded_wire = True
                    log("mission telemetry is live — teleop yields /cmd_vel "
                        "(idle heartbeat and lurch auto-zero suppressed)")
                return
            if self._yielded_wire:
                self._yielded_wire = False
                log("mission telemetry stale or operator input — teleop has the "
                    "wire back")

            if lurch:
                # Out of the lock and straight to zero. Not via the ramp, not
                # via the idle heartbeat's rate limit: the whole point is that
                # the rover is already moving in a way nobody commanded.
                self._safe_zero()
                return

            if idle:
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
            return (self.jog_vx * self.jog_max_mps,
                    self.jog_wz * TURN_MAX_RADPS)

        if self.mode == "nudge":
            return self._nudge_step_unlocked(now)

        if self.mode == "turn":
            return self._turn_step_unlocked(now)

        if self.mode == "test":
            return self._test_step_unlocked(now)

        return 0.0, 0.0

    def _test_step_unlocked(self, now):
        """Target for the current self-test leg. Caller holds lock.

        The worker thread only ever WRITES this dict; the decision to keep
        driving is taken here, on the ROS thread, 20 times a second. So a worker
        that hangs, is descheduled or is killed cannot leave the rover driving:
        each leg carries an absolute deadline and this returns zero past it.
        """
        s = self.selftest
        if not s or s.get("token") != self._test_token:
            self.mode = "idle"
            return 0.0, 0.0
        if now >= s.get("until", 0.0):
            # The leg's time is up. Hold zero and let the worker measure.
            return 0.0, 0.0
        return (s.get("vx", 0.0), s.get("wz", 0.0))

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
        self._abort_test_unlocked(f"lurch guard: {reason}")
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

        This replaces a straight hypot(), which was direction-blind: distance is
        not progress unless it points the right way. Computed from POSE, which
        is the field on /odom_raw that tells the truth about direction — its
        sibling twist.linear.x does not (see ODOM_TWIST_SIGN).
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
        # The speed the nudge STARTED with, not whatever set_speed has been
        # changed to since. A manoeuvre that is already under way keeps the
        # parameters it was accepted and acked with.
        return (n["sign"] * n.get("speed_mps", self.nudge_mps), 0.0)

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
            # non-zero may sit under. Every command path converges here, so a
            # floor established by a future sweep applies to all of them at once
            # without touching any mode. With MIN_CMD_* at their current default
            # of 0.0 this is a no-op beyond the clamp, which is the intended
            # state until a deadband is actually demonstrated. Zero is untouched
            # either way — see snap_up().
            snapped = (0.0 < abs(vx_real) < MIN_CMD_LIN
                       or 0.0 < abs(wz_real) < MIN_CMD_ANG)
            with self.lock:
                self._deadband_snapped = snapped
                lin_cap = min(self.jog_max_mps, HARD_MAX_LIN_MPS)
            vx_out = snap_up(vx_real, MIN_CMD_LIN, lin_cap)
            wz_out = snap_up(wz_real, MIN_CMD_ANG, TURN_MAX_RADPS)

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
            # `stop` first, on purpose. It is the only command that must work
            # when everything else in this method is having a bad day.
            if action in ("stop", "estop", "auto_off"):
                self._cmd_stop(action)
            elif action == "jog":
                self._cmd_jog(payload)
            elif action == "nudge":
                self._cmd_nudge(payload)
            elif action == "turn":
                self._cmd_turn(payload)
            elif action == "test_motors":
                self._cmd_test_motors(payload)
            elif action == "read_encoders":
                self._cmd_read_encoders(payload)
            elif action == "drive_status":
                self._cmd_drive_status(payload)
            elif action == "beep":
                self._cmd_beep(payload)
            elif action == "servo":
                self._cmd_servo(payload)
            elif action in ("drive_connect", "drive_disconnect"):
                self._cmd_drive_connect(action)
            elif action == "set_speed":
                self._cmd_set_speed(payload)
            elif action == "mission":
                self._cmd_mission(payload)
            elif action == "set_coordinate":
                self._cmd_set_coordinate(payload)
            else:
                # Never silent. Includes the verb list so an operator who
                # guessed wrong gets the right answer in the same round trip.
                self._nack(action, f"unknown command {action!r}; this bridge "
                                   f"handles: {', '.join(sorted(TELEOP_ACTIONS))}")
        except ArgError as e:
            # A bad payload is the operator's problem, not a fault: nack with
            # the specific reason and leave the rover's state alone.
            self._nack(action, str(e)[:300])
        except Exception as e:
            log(f"command {action} failed: {e}")
            # A command that blew up must not leave the rover driving.
            try:
                with self.lock:
                    self.mode = "idle"
                    self.nudge = None
                    self.turn = None
                    self._abort_test_unlocked(f"{action} raised {e}")
            except Exception:
                pass
            try:
                self._safe_zero()
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
            if self.mode in ("nudge", "turn", "test"):
                # An operator grabbing the stick outranks a running manoeuvre.
                if self.mode == "test":
                    self._abort_test_unlocked("superseded by jog")
                elif self.mode == "nudge":
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
        with self.lock:
            jmax = self.jog_max_mps
        self._event("events/ack", {"action": "jog", "vx": jnum(vx, 3), "wz": jnum(wz, 3),
                                   "target_mps": jnum(vx * jmax, 4),
                                   "target_radps": jnum(wz * TURN_MAX_RADPS, 4),
                                   "jog_max_mps": jnum(jmax, 4)})

    def _cmd_stop(self, action):
        # NEVER gated. Not on battery, not on link state, not on anything.
        #
        # The first thing it does is set the abort event — BEFORE taking the
        # lock, before publishing anything. The self-test worker polls that
        # event every TEST_POLL_S (20ms), so the worst-case time from `stop`
        # landing to the test giving up commanding is ~20ms plus a control tick,
        # well inside the 100ms this has to hit. Setting it first also means a
        # `stop` still aborts the test even if the code below throws.
        self._test_abort.set()
        with self.lock:
            was = self.mode
            self.mode = "idle"
            self.nudge = None
            self.turn = None
            self._abort_test_unlocked(f"{action} from operator")
            self.jog_vx = self.jog_wz = 0.0
            self.jog_ts = 0.0
            # Bypass the ramp. A stop is not negotiable.
            self.cur_vx = 0.0
            self.cur_wz = 0.0
        for _ in range(8):
            self._safe_zero()
            time.sleep(0.02)
        log(f"{action}: hard zero sent x8, mode=idle (was {was})")
        self._event("events/ack", {"action": action, "estop": True,
                                   "stopped": True, "mode": "idle",
                                   "mode_was": was,
                                   "aborted_test": was == "test"})

    def _abort_test_unlocked(self, reason):
        """Invalidate any running self-test. Caller holds the lock.

        Bumping the token is what actually stops it: the control tick refuses to
        drive a selftest dict whose token no longer matches, so the test is dead
        from the very next tick regardless of what the worker thread is doing or
        whether it is even still scheduled.
        """
        if self.selftest is None and not self._test_thread:
            return
        self._test_token += 1
        self.selftest = None
        self._test_abort.set()
        self._test_abort_reason = reason

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
            if self.mode == "test":
                # A self-test is a sequence of measured legs; letting a nudge
                # interleave with it would corrupt the measurement AND leave two
                # sources deciding what the rover does next.
                self._event("events/nack", {"action": "nudge",
                                            "error": "motor self-test running; "
                                                     "send stop first"})
                return
            block = self._motion_block_unlocked()
            if block:
                self.last_error = block
                self._event("events/nack", {"action": "nudge", "error": block})
                return
            req_mps = self.nudge_mps
            target_m = mm / 1000.0
            # Generous but bounded: 3x the ideal time plus ramp allowance, so a
            # slipping wheel ends in a reported timeout, not an endless drive.
            # Computed off the EFFECTIVE speed so that a nudge below the floor
            # is not given a timeout budget three times longer than it needs.
            eff_mps = snap_up(req_mps, MIN_CMD_LIN,
                              min(self.jog_max_mps, HARD_MAX_LIN_MPS))
            timeout_s = min(60.0, (target_m / max(eff_mps, 1e-6)) * 3.0 + 4.0)
            self.nudge = {"x0": self.odom_x, "y0": self.odom_y,
                          "heading0": self.odom_yaw,
                          "target_m": target_m, "sign": sign,
                          "speed_mps": req_mps,
                          "t0": time.monotonic(), "timeout_s": timeout_s,
                          "dir": d}
            self.mode = "nudge"
        # Report the speed it will ACTUALLY drive at, not the one requested.
        # With the deadband floor at its default of 0.0 these are the same
        # number; if a sweep ever establishes a floor, they diverge and the ack
        # says so rather than advertising a speed the rover is not using.
        log(f"nudge start: {d} {mm}mm at {eff_mps} m/s "
            f"(requested {req_mps}; timeout {timeout_s:.1f}s)")
        self._event("events/ack", {"action": "nudge", "state": "started",
                                   "dir": d, "mm": jnum(mm, 1),
                                   "speed_mps": jnum(eff_mps, 4),
                                   "requested_mps": jnum(req_mps, 4),
                                   "deadband_floored": eff_mps != req_mps,
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
            if self.mode == "test":
                self._event("events/nack", {"action": "turn",
                                            "error": "motor self-test running; "
                                                     "send stop first"})
                return
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
        # As with nudge: report the rate that will actually be used. If an
        # angular floor is ever configured, the closed loop absorbs the
        # difference by cutting drive sooner, so measured_deg is unaffected.
        eff_radps = snap_up(DOCK_TURN_RADPS, MIN_CMD_ANG, TURN_MAX_RADPS)
        log(f"turn start: {d} {deg}deg at {eff_radps} rad/s "
            f"(requested {DOCK_TURN_RADPS}; "
            f"cut drive at {deg*TURN_COAST_FACTOR:.1f}deg, coast {TURN_SETTLE_S}s)")
        self._event("events/ack", {"action": "turn", "state": "started",
                                   "dir": d, "deg": jnum(deg, 1),
                                   "rate_radps": jnum(eff_radps, 4),
                                   "requested_radps": DOCK_TURN_RADPS,
                                   "deadband_floored": eff_radps != DOCK_TURN_RADPS,
                                   "coast_factor": TURN_COAST_FACTOR,
                                   "timeout_s": TURN_TIMEOUT_S})

    # -------------------------------------------------- motor self-test
    # WHY THIS IS A THREAD AND NOT A STATE MACHINE IN THE CONTROL TICK:
    # the test is a SEQUENCE with waits in it, and the two things that must
    # never wait are the control tick (which holds the deadman and the lurch
    # guard) and paho's network thread (which carries `stop`). A blocking
    # sequence therefore cannot live in either. It gets its own thread, and that
    # thread is given no ability to drive: it writes a target, and the control
    # tick decides — 20 times a second, with every existing guard in the path —
    # whether to honour it. If this thread dies, hangs, or is descheduled for a
    # second, each leg's absolute deadline expires and the rover stops anyway.

    def _pose_snapshot(self):
        with self.lock:
            return {"t": time.monotonic(), "x": self.odom_x, "y": self.odom_y,
                    "yaw": self.odom_yaw, "yaw_int": self.yaw_int,
                    "odom_seq": self.odom_seq}

    def _test_wait(self, token, seconds):
        """Sleep in TEST_POLL_S slices. Returns an abort reason, or None if the
        full time elapsed. Never holds the lock across a sleep."""
        deadline = time.monotonic() + seconds
        while True:
            if self._test_abort.is_set():
                return self._test_abort_reason or "aborted"
            with self.lock:
                if self._test_token != token:
                    return self._test_abort_reason or "cancelled"
                mode = self.mode
                block = self._motion_block_unlocked()
            if mode != "test":
                # Something else took the rover (jog, lurch guard, link
                # watchdog). It is no longer this test's to command.
                return f"pre-empted: mode became {mode}"
            if block:
                return block
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            time.sleep(min(TEST_POLL_S, remaining))

    def _cmd_test_motors(self, p):
        with self.lock:
            lin_cap = min(self.jog_max_mps, HARD_MAX_LIN_MPS)
        # Raises ArgError -> nack with the specific reason, from handle_command.
        args = parse_test_motors(p, MIN_CMD_LIN, lin_cap,
                                 MIN_CMD_ANG, HARD_MAX_ANG_RADPS)

        with self.lock:
            if self.mode == "test" or (self._test_thread is not None
                                       and self._test_thread.is_alive()):
                self._event("events/nack",
                            {"action": "test_motors",
                             "error": "a motor self-test is already running; "
                                      "send stop first"})
                return
            if self.mode != "idle":
                self._event("events/nack",
                            {"action": "test_motors",
                             "error": f"rover is busy (mode={self.mode}); send "
                                      f"stop before starting a self-test"})
                return
            block = self._motion_block_unlocked()
            if block:
                self.last_error = block
                self._event("events/nack", {"action": "test_motors",
                                            "error": block})
                return
            # Arm. Clearing the abort event and bumping the token happen under
            # the same lock a `stop` takes, so a stop can be ordered before or
            # after this but never lost inside it.
            self._test_abort.clear()
            self._test_abort_reason = None
            self._test_token += 1
            token = self._test_token
            self.selftest = None
            th = threading.Thread(target=self._motor_test_worker,
                                  args=(token, args),
                                  name="fpms-motor-test", daemon=True)
            self._test_thread = th

        eff = snap_up(args["speed_mps"], MIN_CMD_LIN, lin_cap)
        eff_ang = snap_up(args["ang_radps"], MIN_CMD_ANG, HARD_MAX_ANG_RADPS)
        plan = ["forward", "reverse", "spin_left", "spin_right"]
        est = len(plan) * (args["duration_s"] + TEST_SETTLE_S)
        log(f"test_motors start: {len(plan)} legs x {args['duration_s']:.2f}s at "
            f"{eff:.4f} m/s / {eff_ang:.3f} rad/s (~{est:.1f}s total)")
        self._event("events/ack", {
            "action": "test_motors", "state": "started",
            "legs": plan,
            "speed_mps": jnum(eff, 4),
            "requested_mps": jnum(args["speed_mps"], 4),
            "ang_radps": jnum(eff_ang, 4),
            "duration_s": jnum(args["duration_s"], 2),
            "settle_s": TEST_SETTLE_S,
            "estimated_total_s": jnum(est, 1),
            "deadband_floored": eff != args["speed_mps"],
            "clamps": args["notes"],
            "abortable_by": "stop",
            "result_topic": f"fpms/{THING}/events/motor_test"})
        th.start()

    def _motor_test_worker(self, token, args):
        speed = args["speed_mps"]
        ang = args["ang_radps"]
        dur = args["duration_s"]
        legs = [("forward", +speed, 0.0),
                ("reverse", -speed, 0.0),
                ("spin_left", 0.0, +ang),
                ("spin_right", 0.0, -ang)]
        results = []
        aborted = None
        t_start = time.monotonic()
        batt_before = None
        try:
            with self.lock:
                batt_before = self._battery_v_unlocked()
            for name, vx, wz in legs:
                if self._test_abort.is_set():
                    aborted = self._test_abort_reason or "aborted"
                    break
                if (time.monotonic() - t_start) > TEST_MAX_TOTAL_S:
                    aborted = (f"overall self-test time limit "
                               f"{TEST_MAX_TOTAL_S:g}s reached")
                    break

                before = self._pose_snapshot()
                with self.lock:
                    if self._test_token != token:
                        aborted = self._test_abort_reason or "cancelled"
                        break
                    block = self._motion_block_unlocked()
                    if block:
                        aborted = block
                        break
                    now = time.monotonic()
                    # The absolute deadline is set HERE and read by the control
                    # tick. Nothing the worker does afterwards can extend it.
                    self.selftest = {"vx": vx, "wz": wz, "leg": name,
                                     "token": token, "until": now + dur}
                    self.mode = "test"

                stop_reason = self._test_wait(token, dur)

                # End of leg: command zero and let the ramp and the chassis
                # settle before measuring, so the number reported is where the
                # rover ENDED UP rather than where it was still moving through.
                with self.lock:
                    if self._test_token == token and self.selftest is not None:
                        self.selftest = {"vx": 0.0, "wz": 0.0,
                                         "leg": name + ":settle",
                                         "token": token, "until": 0.0}
                settled = self._test_wait(token, TEST_SETTLE_S)
                after = self._pose_snapshot()

                leg = summarize_leg(name, vx, wz, before, after)
                if stop_reason:
                    leg["ok"] = False
                    leg["verdict"] = f"leg cut short: {stop_reason}"
                results.append(leg)

                if stop_reason:
                    aborted = stop_reason
                    break
                if settled:
                    # ANY reason the settle window ended early ends the run —
                    # including pre-emption. Carrying on to the next leg would
                    # mean setting mode back to "test" and taking the rover
                    # away from whatever (a jog, a guard) had just claimed it.
                    aborted = settled
                    break
        except Exception as e:
            aborted = f"self-test worker error: {e}"
            log(f"test_motors worker error {e}")
        finally:
            # Whatever happened, this thread hands the rover back idle and
            # stopped. The zero goes out directly as well as via the tick.
            try:
                with self.lock:
                    if self._test_token == token:
                        self.selftest = None
                        if self.mode == "test":
                            self.mode = "idle"
                        self.cur_vx = self.cur_wz = 0.0
                    if self._test_thread is threading.current_thread():
                        self._test_thread = None
            except Exception:
                pass
            try:
                self._safe_zero()
            except Exception:
                pass

        with self.lock:
            batt_after = self._battery_v_unlocked()
        ok_legs = [r for r in results if r.get("ok")]
        summary = {
            "action": "test_motors",
            "state": "aborted" if aborted else "finished",
            "ok": bool(aborted is None and len(ok_legs) == len(legs)),
            "aborted": bool(aborted),
            "abort_reason": aborted,
            "legs_planned": len(legs),
            "legs_run": len(results),
            "legs_ok": len(ok_legs),
            "speed_mps": jnum(speed, 4),
            "ang_radps": jnum(ang, 4),
            "duration_s": jnum(dur, 2),
            "elapsed_s": jnum(time.monotonic() - t_start, 2),
            "results": results,
            "battery_v_before": jnum(batt_before, 2),
            "battery_v_after": jnum(batt_after, 2),
            # Said explicitly because a self-test that silently used a speed
            # other than the one asked for is a self-test that lies.
            "note": ("speeds are REAL m/s; the deadband floor is currently "
                     f"{MIN_CMD_LIN} m/s lin / {MIN_CMD_ANG} rad/s ang and "
                     "anything under it is raised to it before reaching the "
                     "wire. Distances and angles are measured from /odom_raw "
                     "POSE and the integrated gyro, never from twist.linear.x, "
                     "which this firmware reports sign-inverted"),
            "odom_twist_sign": ODOM_TWIST_SIGN,
        }
        with self.lock:
            self.last_test = {k: summary[k] for k in
                              ("state", "ok", "legs_ok", "legs_run",
                               "abort_reason", "elapsed_s")}
            self.last_test["at"] = time.time()
        log(f"test_motors {summary['state']}: {len(ok_legs)}/{len(legs)} legs ok"
            + (f" ({aborted})" if aborted else ""))
        # Subtype is "motor_test" and NOT anything containing "alert" or
        # "fire": the backend's email_alerts matcher keys off the subtype and
        # would mail out a fire warning for a motor self-test.
        self._event("events/motor_test", summary)

    # ------------------------------------------------------------ encoders
    def _cmd_read_encoders(self, p):
        """There are no encoder ticks to read on this board. Say so, and report
        what /odom_raw does give.

        The Yahboom MicroROS Board V2.0 firmware publishes an integrated pose
        and a twist on /odom_raw and nothing else — no /wheel_ticks, no joint
        states, no per-wheel counts. Synthesising ticks by dividing distance by
        a guessed wheel radius would produce a number that looks like a
        measurement and is actually an assumption, so this reports the pose and
        the twist and names what is missing.

        Allowed with the link down: a stale reading, labelled stale, is the
        thing you want when you are diagnosing why the link is down.
        """
        now = time.monotonic()
        with self.lock:
            link_ok = self._link_ok_unlocked(now)
            age = (now - self.odom_last) if self.odom_last else None
            x, y = self.odom_x, self.odom_y
            x_mm, y_mm = self._arena_mm_unlocked()
            yaw = self.odom_yaw
            yaw_int = self.yaw_int
            twist_vx, twist_wz = self.odom_twist_vx, self.odom_twist_wz
            twist_vx_raw = self.odom_twist_vx_raw
            derived_vx = self.odom_vx
            seq = self.odom_seq
            hz = hz_from(list(self.odom_times))
            gyro = self.gyro_z

        self._event("events/encoders", {
            "action": "read_encoders",
            "ok": True,
            # ---- the honest part ----
            "ticks_available": False,
            "ticks_left": None,
            "ticks_right": None,
            "note": ("This board (Yahboom MicroROS Board V2.0, ESP32-S3, "
                     "micro-ROS) does NOT publish raw encoder tick counts. "
                     "There is no /wheel_ticks or joint_states topic and the "
                     "firmware exposes no per-wheel counters. The values below "
                     "come from /odom_raw, which is the board's own integrated "
                     "estimate. They are NOT derived from ticks here and no "
                     "tick count is inferred from them."),
            # ---- what actually exists ----
            "source_topic": "/odom_raw",
            "pose": {"x_m": jnum(x, 4), "y_m": jnum(y, 4),
                     "heading_deg": jnum(wrap180(math.degrees(yaw)), 2),
                     "heading_rad": jnum(yaw, 4)},
            "arena_mm": {"x_mm": jnum(x_mm, 1), "y_mm": jnum(y_mm, 1)},
            "twist": {"linear_x_mps": jnum(twist_vx, 4),
                      "linear_x_mps_raw": jnum(twist_vx_raw, 4),
                      "angular_z_radps": jnum(twist_wz, 4),
                      "sign_corrected": True,
                      "odom_twist_sign": ODOM_TWIST_SIGN,
                      "odom_twist_ang_sign": ODOM_TWIST_ANG_SIGN,
                      "note": ("instantaneous, as reported by the board, with "
                               "linear_x NEGATED: this firmware publishes "
                               "twist.linear.x sign-inverted relative to its "
                               "own pose.position (measured on the bench). "
                               "linear_x_mps_raw is the uncorrected field. "
                               "angular_z is NOT corrected — rotation has not "
                               "been tested, so whether it shares the "
                               "inversion is unknown. Prefer the derived "
                               "values below, which come from pose.")},
            "derived": {
                "ground_speed_mps": jnum(derived_vx, 4),
                "gyro_z_radps": jnum(gyro, 4),
                "yaw_integrated_deg": jnum(wrap180(math.degrees(yaw_int)), 2),
                "note": ("ground_speed_mps is differentiated from consecutive "
                         "/odom_raw POSITIONS by this bridge and is the "
                         "trustworthy figure — pose was correct in every bench "
                         "trial while twist was not. yaw_integrated_deg is "
                         "integrated from /imu angular_velocity.z because the "
                         "board publishes an identity quaternion and does not "
                         "fuse orientation"),
            },
            "odom_frames_since_boot": int(seq),
            "odom_hz": jnum(hz, 2),
            "odom_age_s": jnum(age, 2),
            "stale": (not link_ok),
            "ros_ok": bool(link_ok),
        })

    # -------------------------------------------------------- drive_status
    def _cmd_drive_status(self, p):
        """Never gated: the moment you most need status is the moment the gates
        are closed.

        Named `drive_status`, not `status`. fpms-rover-agent owns the generic
        `status` verb — it has no ROS dependency and so keeps answering through
        a micro-ROS outage, which is when status matters most. This one is
        additive: the motion envelope, the guards, the floors and the calibration
        that only the process holding /cmd_vel can report. Both can be asked;
        neither shadows the other. Reply goes to events/drive_status for the
        same reason, so the two never collide on the wire either.

        Round-trip timing lives on rover-agent's `ping` as well, for the same
        reason; a client wanting latency should ping that.
        """
        recv_wall = time.time()
        snap = self._status_snapshot()
        snap["action"] = "drive_status"
        snap["ok"] = True
        # If the caller sends a timestamp we report the one-way leg to this
        # process, which is a different (and slower) path than rover-agent's
        # ping — it is the number that tells you whether the ROS side is the
        # thing that is late. Null when the caller's clock is implausible.
        t_echo, t_units, uplink_ms = ping_timing(p.get("t"), recv_wall)
        snap["echo"] = t_echo
        snap["echo_units"] = t_units
        snap["uplink_ms"] = jnum(uplink_ms, 1)
        snap["seq"] = p.get("seq")
        snap["rover_recv_ts"] = jnum(recv_wall, 3)
        snap["handling_ms"] = jnum((time.time() - recv_wall) * 1000.0, 2)
        snap["owns"] = sorted(TELEOP_ACTIONS)
        snap["not_owned_here"] = {
            "ping": "fpms-rover-agent",
            "status": "fpms-rover-agent",
            "connect": "fpms-rover-agent",
            "disconnect": "fpms-rover-agent",
            "auto_on": "nobody — no autonomy loop exists; rover-agent nacks it",
        }
        self._event("events/drive_status", snap)

    def _status_snapshot(self):
        now = time.monotonic()
        with self.lock:
            link_ok = self._link_ok_unlocked(now)
            batt_v = self._battery_v_unlocked()
            batt_age = (now - self.battery_last) if self.battery_last else None
            block = self._motion_block_unlocked()
            x_mm, y_mm = self._arena_mm_unlocked()
            mode = self.mode
            jog_age = (now - self.jog_ts) if self.jog_ts else None
            snap = {
                "svc": "teleop",
                "board": "Yahboom MicroROS Board V2.0 (ESP32-S3), micro-ROS",
                "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),

                # --- link / rates ---
                "ros_ok": bool(link_ok),
                "odom_age_s": jnum((now - self.odom_last) if self.odom_last
                                   else None, 2),
                "imu_age_s": jnum((now - self.imu_last) if self.imu_last
                                  else None, 2),
                "rates_hz": {"odom": jnum(hz_from(list(self.odom_times)), 2),
                             "imu": jnum(hz_from(list(self.imu_times)), 2),
                             "battery": jnum(hz_from(list(self.battery_times)), 2),
                             "control": CONTROL_HZ,
                             "telemetry": TELEM_HZ},

                # --- power ---
                "battery_v": jnum(batt_v, 2),
                "battery_raw_decivolts": self.battery_raw,
                "battery_low": (batt_v is not None and batt_v < BATT_LOW_V),
                "battery_age_s": jnum(batt_age, 2),
                "battery_stale": (batt_age is None or batt_age > BATT_STALE_S),

                # --- mode / motion ---
                "mode": mode,
                "moving": bool(abs(self.cur_vx) > 1e-4
                               or abs(self.cur_wz) > 1e-4),
                "cmd_vel_real": {"vx": jnum(self.cur_vx, 4),
                                 "wz": jnum(self.cur_wz, 4)},
                "odom_vx": jnum(self.odom_vx, 4),
                "motion_allowed": block is None,
                "motion_block_reason": block,
                "deadman_ok": bool(mode != "jog" or (jog_age is not None
                                                     and jog_age <= JOG_DEADMAN_S)),
                "ms_since_jog": int(jog_age * 1000) if jog_age is not None else -1,
                "deadband_snapped": bool(self._deadband_snapped),
                "lurch_trips": int(self.lurch_trips),

                # --- pose ---
                "pose": {"x_mm": jnum(x_mm, 1), "y_mm": jnum(y_mm, 1),
                         "heading_deg": jnum(wrap180(math.degrees(self.odom_yaw)), 2),
                         "yaw_int_deg": jnum(wrap180(math.degrees(self.yaw_int)), 2)},
                "origin_set": self.origin is not None,

                # --- the envelope, floors first ---
                "floors": {"min_cmd_lin_mps": jnum(MIN_CMD_LIN, 4),
                           "min_cmd_ang_radps": jnum(MIN_CMD_ANG, 4),
                           "enabled": bool(MIN_CMD_LIN > 0 or MIN_CMD_ANG > 0),
                           "source": "config.env override or built-in default",
                           "note": ("UNMEASURED — defaults are 0.0 (floor off). "
                                    "The 'deadband lurch' this mechanism was "
                                    "built for was an odom twist sign "
                                    "inversion, not a real deadband. Kept, "
                                    "disabled, pending a clean pose-based "
                                    "sweep under load. Nothing may command "
                                    "below these when non-zero. "
                                    "2026-08-01: response is NOT proportional "
                                    "and LOAD IS THE HIDDEN VARIABLE — on the "
                                    "floor 0.00295 produced no motion at all, "
                                    "while free-spinning 0.0010 ran fast. Do "
                                    "not read any speed claim here without "
                                    "knowing whether the wheels were loaded.")},
                "limits": {"jog_max_mps": jnum(self.jog_max_mps, 4),
                           "nudge_mps": jnum(self.nudge_mps, 4),
                           "turn_max_radps": jnum(TURN_MAX_RADPS, 4),
                           "hard_max_lin_mps": HARD_MAX_LIN_MPS,
                           "hard_max_ang_radps": HARD_MAX_ANG_RADPS,
                           "max_nudge_mm": MAX_NUDGE_MM,
                           "max_turn_deg": MAX_TURN_DEG,
                           "servo_deg": [SERVO_MIN_DEG, SERVO_MAX_DEG],
                           "beep_ms": [BEEP_MIN_MS, BEEP_MAX_MS],
                           "batt_low_v": BATT_LOW_V,
                           "jog_deadman_s": JOG_DEADMAN_S,
                           "ros_dead_s": ROS_DEAD_S},
                "tuned_by_operator": bool(self.tuned_by_operator),
                "cmd_scale": CMD_SCALE,
                # Visible, not hidden: a correction nobody can see is a
                # correction someone will re-derive from scratch next week.
                "odom_twist_sign": ODOM_TWIST_SIGN,
                "odom_twist_ang_sign": ODOM_TWIST_ANG_SIGN,
                "odom_twist_vx_raw": jnum(self.odom_twist_vx_raw, 4),
                "odom_twist_note": ("/odom_raw twist.linear.x is sign-inverted "
                                    "relative to its own pose.position on this "
                                    "firmware (measured). It is corrected once, "
                                    "on ingest, by odom_twist_sign. All guards "
                                    "and all reported distances use "
                                    "differentiated POSE, which was correct in "
                                    "every trial. twist.angular.z is NOT "
                                    "corrected — rotation has not been tested, "
                                    "so whether it shares the inversion is "
                                    "unknown."),

                # --- services' view (only what this node can actually see) ---
                "telemetry_enabled": bool(self.telemetry_enabled),
                "telemetry_suppressed": int(self.telemetry_suppressed),
                "mqtt_connected": bool(self.bus.connected),
                "self_test": self.last_test,
                "self_test_running": bool(mode == "test"),

                # --- bookkeeping ---
                "uptime_s": jnum(time.time() - STARTED, 1),
                "last_cmd": self.last_cmd,
                "last_cmd_at": jnum(self.last_cmd_at, 3),
                "last_error": self.last_error,
                "actions": sorted(TELEOP_ACTIONS),
                "reply_topics": dict(TELEOP_ACTIONS),
                "config_notes": list(CFG_NOTES),
                "scan_note": ("/scan is dead on this board (all ranges 0.0) and "
                              "is deliberately not subscribed"),
            }
        return snap

    # ---------------------------------------------------------------- beep
    def _cmd_beep(self, p):
        ms, notes = parse_beep(p)          # ArgError -> nack, from handle_command
        with self.lock:
            link_ok = self._link_ok_unlocked()
        if not link_ok:
            # Not a motion command, but publishing into a dead micro-ROS session
            # is a no-op that would ack as though the rover had beeped. Say the
            # true thing instead.
            self._event("events/nack",
                        {"action": "beep",
                         "error": "micro-ROS link down: no /odom_raw for "
                                  f">{ROS_DEAD_S:.0f}s, so /beep would go "
                                  f"nowhere"})
            return
        try:
            m = UInt16()
            m.data = int(ms)
            self.pub_beep.publish(m)
        except Exception as e:
            self._nack("beep", f"/beep publish failed: {e}")
            return
        log(f"beep: {ms}ms")
        self._event("events/ack", {"action": "beep", "ms": int(ms),
                                   "topic": "/beep", "clamps": notes,
                                   "range_ms": [BEEP_MIN_MS, BEEP_MAX_MS],
                                   "note": ("0 silences; this command never "
                                            "sends 1, which latches the beeper "
                                            "on with no way to clear it if the "
                                            "link then drops")})

    # --------------------------------------------------------------- servo
    def _cmd_servo(self, p):
        which, angle, notes = parse_servo(p)
        with self.lock:
            block = self._motion_block_unlocked()
        if block:
            # A servo is an actuator on the same pack as the drive motors and
            # it moves something physical, so it is gated exactly like motion.
            self._event("events/nack", {"action": "servo", "error": block})
            return
        topic = f"/servo_s{which}"
        try:
            m = Int32()
            m.data = int(angle)
            self.pub_servo[which].publish(m)
        except Exception as e:
            self._nack("servo", f"{topic} publish failed: {e}")
            return
        log(f"servo: s{which} -> {angle}deg")
        self._event("events/ack", {
            "action": "servo", "which": which, "topic": topic,
            "angle_deg": int(angle),
            "requested_deg": p.get("angle"),
            "clamps": notes,
            "range_deg": [SERVO_MIN_DEG, SERVO_MAX_DEG],
            "range_note": ("clamped to "
                           f"{SERVO_MIN_DEG}..{SERVO_MAX_DEG} deg, inset from "
                           "the firmware's 0..180 so the horn never parks "
                           "against a mechanical end stop and stalls at "
                           "locked-rotor current off the drive pack; centre is "
                           "90. Override with FPMS_SERVO_MIN_DEG / "
                           "FPMS_SERVO_MAX_DEG in config.env"),
            "open_loop": True,
            "position_feedback": False})

    # ------------------------------------------------- telemetry streaming
    def _cmd_drive_connect(self, action):
        """Enable/disable THIS bridge's telemetry/drive stream. Nothing else.

        Named drive_connect/drive_disconnect rather than connect/disconnect:
        fpms-rover-agent owns the plain verbs, and two processes answering the
        same command is a race the operator resolves by guessing.

        It does not connect or disconnect MQTT (paho reconnects forever by
        design and a command that could sever the link `stop` arrives on has no
        business existing), it does not touch micro-ros-agent, and it does not
        touch fpms-rover-agent's own telemetry. Acks and nacks keep flowing
        while disconnected — an operator who muted the stream must still be told
        that his stop worked.
        """
        want = (action == "drive_connect")
        with self.lock:
            was = self.telemetry_enabled
            self.telemetry_enabled = want
            if want:
                self.telemetry_suppressed = 0
            suppressed = self.telemetry_suppressed
        log(f"{action}: telemetry streaming {'ON' if want else 'OFF'} "
            f"(was {'ON' if was else 'OFF'})")
        self._event("events/ack", {
            "action": action, "telemetry_enabled": want, "was": was,
            "changed": was != want,
            "suppressed_ticks": suppressed,
            "topic": f"fpms/{THING}/telemetry/drive",
            "scope": ("this teleop bridge's telemetry/drive stream only; "
                      "micro-ros-agent, fpms-rover-agent (including its own "
                      "connect/disconnect) and the MQTT connection itself are "
                      "untouched, and acks/nacks/events keep flowing")})

    # ----------------------------------------------------------- set_speed
    def _cmd_set_speed(self, p):
        # The effective lower bound is the LARGER of the configured deadband
        # floor and the sanity limit, so if a sweep ever raises the floor, this
        # follows it automatically.
        floor = max(MIN_CMD_LIN, SET_SPEED_MIN_MPS)
        updates, notes = parse_set_speed(p, floor, HARD_MAX_LIN_MPS)
        with self.lock:
            if self.mode != "idle":
                # Retuning the envelope under a manoeuvre that is already
                # running would change its speed halfway through, after it had
                # been acked at a different one.
                self._event("events/nack",
                            {"action": "set_speed",
                             "error": f"rover is moving (mode={self.mode}); "
                                      f"send stop before retuning speeds"})
                return
            before = {"jog_max": jnum(self.jog_max_mps, 4),
                      "nudge": jnum(self.nudge_mps, 4)}
            if "jog_max" in updates:
                # Re-clamped on write as well as on read. Three independent
                # clamps now stand between this payload and a wheel.
                self.jog_max_mps = clamp(updates["jog_max"],
                                         floor, HARD_MAX_LIN_MPS)
            if "nudge" in updates:
                self.nudge_mps = clamp(updates["nudge"],
                                       floor, HARD_MAX_LIN_MPS)
            # A nudge above the jog cap would be clipped by the hard clamp
            # anyway; make that visible rather than letting the two disagree.
            if self.nudge_mps > self.jog_max_mps:
                notes.append(f"nudge {self.nudge_mps:g} m/s lowered to jog_max "
                             f"{self.jog_max_mps:g} m/s, which is the hard clamp")
                self.nudge_mps = self.jog_max_mps
            self.tuned_by_operator = True
            after = {"jog_max": jnum(self.jog_max_mps, 4),
                     "nudge": jnum(self.nudge_mps, 4)}
        log(f"set_speed: {before} -> {after}")
        self._event("events/ack", {
            "action": "set_speed", "changed": before != after,
            "before": before, "after": after,
            "clamps": notes,
            "bounds": {"floor_mps": jnum(floor, 4),
                       "deadband_floor_mps": jnum(MIN_CMD_LIN, 4),
                       "sanity_floor_mps": SET_SPEED_MIN_MPS,
                       "hard_max_mps": HARD_MAX_LIN_MPS},
            "persisted": False,
            "note": ("runtime only — this does NOT write config.env, so a "
                     "service restart returns to the defaults. The lower bound "
                     f"is {floor:g} m/s, the larger of the configured deadband "
                     f"floor ({MIN_CMD_LIN:g}) and a sanity limit "
                     f"({SET_SPEED_MIN_MPS:g}); values below it are refused "
                     "rather than clamped so the setting you read back is "
                     "always the speed the rover uses")})

    def _cmd_mission(self, p):
        # fpms-missions owns this verb when it is running. Staying silent then
        # is the whole point: two processes acking the same mission is how an
        # operator ends up reading "accepted, no motion" from this stub while
        # the rover is in fact driving. Same rule fpms_rover_agent follows for
        # teleop's verbs, and the same reason.
        if self.bus.mission_active():
            return

        name = str(p.get("name", ""))
        if name not in ("m1", "m2", "water", "home"):
            self._nack("mission", f"unknown mission {name!r}")
            return
        # Reached only when the executor is NOT running. Answering here — rather
        # than leaving the verb unhandled — is deliberate: a mission button that
        # produces total silence is indistinguishable from a broken dashboard,
        # and the actual fault is a service that is not running.
        log(f"mission '{name}' received but fpms-missions is not publishing "
            "telemetry — refusing rather than pretending")
        self._nack("mission",
                   "the mission executor (fpms-missions) is not running, so "
                   "nothing can drive this route. teleop has never implemented "
                   "navigation and will not invent it under a moving rover. "
                   "Start it with: sudo systemctl start fpms-missions")

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
            # Gated by drive_connect/drive_disconnect. The gate is checked here
            # and nowhere else, so muting the stream cannot affect acks, nacks,
            # events, the control tick or any guard — it stops one publish.
            with self.lock:
                if not self.telemetry_enabled:
                    self.telemetry_suppressed += 1
                    return
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
                twist_vx = self.odom_twist_vx
                twist_vx_raw = self.odom_twist_vx_raw
                lurch_trips = self.lurch_trips
                jog_max = self.jog_max_mps
                nudge_max = self.nudge_mps
                test_running = (mode == "test")

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
                # True while the last published command was raised to the floor.
                # With the floor at its default of 0.0 this is always false; it
                # only becomes meaningful if a sweep establishes a real floor.
                "deadband_snapped": bool(snapped),
                # What the POSE says the rover is doing, as opposed to what it
                # was told to do. The lurch guard compares exactly these two,
                # and it is pose-differentiated rather than read from twist —
                # see odom_twist_sign below for why that matters.
                "odom_vx": jnum(odom_vx, 4),
                "lurch_trips": int(lurch_trips),

                # --- odom sign correction, surfaced rather than hidden -------
                # /odom_raw twist.linear.x is sign-inverted relative to its own
                # pose on this firmware (measured on the bench). It is corrected
                # exactly once, on ingest. This field is here so a dashboard or
                # a future investigator can SEE that a correction is in force
                # instead of rediscovering the inversion the hard way.
                "odom_twist_sign": ODOM_TWIST_SIGN,
                "odom_twist_ang_sign": ODOM_TWIST_ANG_SIGN,
                "odom_twist_vx": jnum(twist_vx, 4),       # corrected
                "odom_twist_vx_raw": jnum(twist_vx_raw, 4),  # as published
                "odom_source": "pose-differentiated (twist not trusted)",

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
                "jog_max_mps": jnum(jog_max, 4),
                "nudge_mps": jnum(nudge_max, 4),
                "self_test_running": bool(test_running),
                "origin_set": self.origin is not None,
            }
            self.bus.publish("telemetry/drive", payload, qos=0)
        except Exception as e:
            log(f"telemetry tick error {e}")

    # ------------------------------------------------------------- shutdown
    def shutdown_stop(self):
        """Zero the motors, loudly and repeatedly, before anything is torn down."""
        # Kill any self-test FIRST and outside the lock, so a worker mid-leg
        # cannot write a fresh target between here and the zeros below.
        try:
            self._test_abort.set()
        except Exception:
            pass
        try:
            with self.lock:
                self.mode = "idle"
                self.nudge = None
                self.turn = None
                self._abort_test_unlocked("service shutting down")
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
