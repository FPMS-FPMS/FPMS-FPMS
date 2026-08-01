#!/usr/bin/env python3
"""FPMS mission executor — "go to a zone on command, and come back perfectly".

Owns exactly one MQTT verb, `mission`, and one job: drive to one or more named
arena targets in order, dock and hold at each, and return to the start pose
facing the start heading. It publishes /cmd_vel while a mission runs and nothing
at all when one does not.

TERMINOLOGY, because two things were both called a "leg"
========================================================
    LEG      one stage of a route: the drive from wherever the rover is to ONE
             target. A single-target mission is a one-leg route. `MAX_SEGMENTS`
             and the hold are per LEG.
    SEGMENT  one commanded motion — a turn, or one bounded straight run.
             `split_legs` cuts a leg's straight-line distance into these.

MULTI-LEG ROUTES
================
`ROUTES` chains single-target missions into one run: `patrol` visits the
top-RIGHT zone, then the top-LEFT zone, then the water station, and retraces the
WHOLE path home. Legs are named by their single-target mission id, so a route
and the missions it is built from can never disagree about where a target is.

The route is not three missions in a row. Three missions would each retrace home
and set off again — four crossings of the arena instead of two, and three
separate chances for the re-face to be wrong. One route accumulates every
measured motion into a single list and unwinds it once.

WHY dead reckoning IS THE DEFAULT BACKEND
=========================================
Two backends are implemented and selectable per command:

  deadreckon  turn-then-drive segments, closed loop on /odom pose and integrated
              gyro Z, with a RETRACE return. No ROS navigation stack, no map, no
              AMCL, no costmaps.   <-- THE DEFAULT
  nav2        NavigateToPose goals (the action `nav2_simple_commander` wraps),
              feedback monitored and republished as progress.

`deadreckon` is the default because it is the only one of the two that has ever
worked on this rover. The prior autonomy (`/home/ubuntu/fpms_phase6_LATEST.py`)
drove exactly this way — turn, drive, stop, measure, retrace — with dead
reckoning as its ONLY localisation, and it logged 0.6 % distance error and
+/-1-4 degrees per turn. Its tuned constants are carried over below by name.

Nav2 cannot localise here yet. It needs a LaserScan in ROS; the board's /scan is
dead (every range 0.0) and the real LiDAR belongs to fpms-rover-agent, which
publishes to MQTT only. The chain LiDAR->ROS -> TF -> map -> AMCL -> costmaps is
unbuilt, so `backend: "nav2"` is expected to refuse with a specific reason until
it is. That refusal is a feature: it names the missing link instead of driving
against a pose nobody has verified. See NAV2_BRIEF.md section 7.

WHY THE RETRACE IS THE VALUABLE PART
====================================
The return does not re-plan — and this holds for a three-leg route exactly as it
does for a one-leg one. It replays the MEASURED outbound motions in reverse
order with their signs flipped:

    outbound   turn +44.7 deg (asked 45), drive +302 mm (asked 300), drive ...
    return     ... drive -302 mm, turn -44.7 deg

Every motion is undone by the number that was actually measured, not by the
number that was requested. So a turn that came out 2 degrees short is undone 2
degrees short, and the two errors cancel BY CONSTRUCTION rather than being
carried home and added to. That is why "come back perfectly" worked before, and
why it keeps working with no SLAM, no AMCL and no Nav2 running.

The return also drives BACKWARDS along the outbound path rather than turning
around. Two 180 degree turns are two more chances to be wrong, and undoing the
outbound turns in reverse leaves the rover already facing the start heading when
it arrives — the re-face is a small correction, not the plan. The LiDAR is 360
degrees, so the obstacle guard simply watches the rear cone instead of the front
one while reversing (see `front_clearance_mm(..., reverse=True)`).

Say the consequence out loud, because it is the one thing about a multi-leg
route that surprises people: a `patrol` reverses along ALL THREE legs on the way
back — roughly 2.2 m of reversing, past both zones, without stopping at them.
That is not a bug to be optimised away with a short fresh plan from the last
target. A fresh plan carries every millimetre of accumulated drift home with it;
the retrace cancels it. The retrace is the longer route and the accurate one,
and accuracy is what the operator asked for when they said "come back".

THE SPEED CONSTRAINT — READ THIS BEFORE "FIXING" THE DOCK SPEED
===============================================================
The firmware's velocity loop regulates INTEGER encoder counts per 10 ms. One
count per period is 0.0145 m/s on the wire; below that the setpoint is under a
single count and the feedback is pure quantisation noise. Measured: wire 0.0041
stalls then lurches, wire 0.012 is noise, wire 0.100 is clean. The 200-of-400
PWM dead zone is a 50 % duty feed-forward, so commanded duty is either 0 or
50-100 %. There is no gentle duty and there is no slow setpoint.

CONSEQUENCE, AND THE SINGLE MOST COUNTER-INTUITIVE THING IN THIS FILE:

    SLOW DOCKING IS SHORT BOUNDED SEGMENTS WITH FULL STOPS BETWEEN THEM.
    IT IS NOT A LOW CONTINUOUS VELOCITY.

DOCK_MPS is therefore deliberately EQUAL to CRUISE_MPS. The dock is slow because
each burst covers DOCK_STEP_MM and then stops dead, not because the wheels turn
slower. Average approach speed lands near 0.05 m/s while every individual burst
runs at a speed the loop can actually hold.

If you are reading this because the dock looks jerky and you are about to lower
DOCK_MPS: lowering it produces stall-then-lurch, which is worse and less
controlled, not smoother. It is quantisation, not tuning; the real fix is the
firmware's encoder/wheel constants. Shorten DOCK_STEP_MM instead.

COEXISTENCE WITH fpms-teleop — COMPANION CHANGES REQUIRED BEFORE THIS SHIPS
==========================================================================
1. fpms_teleop.py currently claims `mission` in TELEOP_ACTIONS and answers it
   with a stub ack ("logged only, no navigation implemented"). That entry and
   `_cmd_mission` MUST BE DELETED when this service is installed. Both processes
   would otherwise answer the same verb, and the dashboard resolves a command on
   the FIRST reply for a (thing, action) pair — so an operator could see "no
   navigation implemented" for a mission that is at that moment driving.
   rover/test_commands.py has a cross-file check for exactly this class of bug.

2. teleop's control tick publishes a ZERO Twist heartbeat every 0.5 s while it
   is idle. That is correct when teleop owns the wire and fatal when it does
   not: interleaving 2 Hz zeros into this node's 20 Hz setpoint is precisely the
   stall-then-lurch described above. teleop must suppress that idle heartbeat
   while a mission owns the wire (cheapest form: latch on the freshness of
   `fpms/<thing>/telemetry/mission`, which teleop's existing MQTT client can
   subscribe), or be stopped for the duration.

   Until that lands, this node REFUSES TO START A MISSION rather than fight for
   the wire: preflight listens on /cmd_vel while commanding nothing, and any
   traffic it hears is a foreign writer and a specific nack. A refusal that
   names the other writer is worth more than a mission that lurches.

3. `stop` / `estop` / `auto_off` stay teleop's verbs. This node SUBSCRIBES them
   and ACTS on them — instant abort, wire commanded to zero — but PUBLISHES NO
   REPLY, exactly as fpms_rover_agent stays silent on teleop's verbs. Acting
   without answering is the only way two processes can both honour a stop
   without racing each other's acknowledgement.

FRAMES
======
Arena geometry is mirrored from `frontend/src/lib/arena.ts`, which is the single
source of truth. It is mirrored as the same FRACTIONS of ARENA_MM (0.30 zone
side, 0.04 margin) that that file uses, not as pre-computed millimetres, so a
rescale there stays a one-number change here too. Zone centres are DERIVED.

    world/arena  origin BOTTOM-LEFT, +x right, +y up, millimetres.
                 heading in degrees CCW from +x; 90 = up the arena = FORWARD.
    odom         the board's own frame, metres, arbitrary origin and rotation.

`Anchor` maps one to the other. With no external localisation the anchor is the
ASSUMED start pose (ROVER_START, heading 90) — the same assumption the dashboard
renders behind its "POSE: SIMULATED" badge — and telemetry says so with
`pose_assumed: true`. Every consumer must treat a missing pose field as missing;
an unguarded read of one blanked the whole dashboard once already.

SAFETY POSTURE
==============
  * Commands arrive on paho's network thread and only set state. All motion runs
    on ONE worker thread, which is the only publisher of a non-zero Twist.
  * `stop` sets a halt latch that `DriveIO.publish` itself honours, so the worker
    cannot emit a non-zero Twist after a stop even if it is mid-segment.
  * Abort is by POLLING, never by sleeping: the worker checks the abort set at
    every 20 Hz tick and inside every settle, so a stop lands in <= ~50 ms.
  * Bounded by ceilings the payload cannot raise: per-segment timeout, total
    mission timeout, hard clamps in real m/s immediately before the wire.
  * Aborts on: stop command, micro-ROS link loss, low battery, an obstacle
    inside the stop distance in the cone the rover is moving towards, a stalled
    segment, and either timeout.

This module is importable WITHOUT rclpy or paho installed, so the geometry —
waypoint derivation, segment planning, retrace inversion, heading wrap — can be
tested off-robot.
"""

import json
import math
import os
import signal
import threading
import time
from collections import deque
from dataclasses import dataclass

# ROS and MQTT are optional AT IMPORT TIME on purpose: everything above the
# "REQUIRES ROS" banner is pure arithmetic and is unit-tested on a laptop that
# has neither. Nothing below the banner is reachable without them, and main()
# refuses to start rather than pretending.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                           HistoryPolicy)
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import UInt16
    HAVE_ROS = True
    ROS_IMPORT_ERROR = ""
except Exception as _e:                                     # pragma: no cover
    HAVE_ROS = False
    ROS_IMPORT_ERROR = str(_e)
    Node = object

try:
    import paho.mqtt.client as mqtt
    from paho.mqtt.client import CallbackAPIVersion
    HAVE_MQTT = True
    MQTT_IMPORT_ERROR = ""
except Exception as _e:                                     # pragma: no cover
    HAVE_MQTT = False
    MQTT_IMPORT_ERROR = str(_e)

# Nav2 is optional even on the robot — it is not installed yet, and the backend
# that needs it must nack with the reason rather than crash the service.
try:
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    HAVE_NAV2 = True
    NAV2_IMPORT_ERROR = ""
except Exception as _e:                                     # pragma: no cover
    HAVE_NAV2 = False
    NAV2_IMPORT_ERROR = str(_e)


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

# Written by teleop's `set_coordinate`. Read-only here: if the operator has told
# teleop where the rover really is, that beats this file's assumed start pose.
TELEOP_ORIGIN_FILE = "/home/ubuntu/.fpms_teleop_origin.json"
STARTED = time.time()

CFG_NOTES = []


def _cfg_float(key, default, lo, hi):
    """config.env override, clamped. Out-of-range is noted, never silent."""
    raw = CFG.get(key)
    if raw is None:
        return default
    try:
        v = float(raw)
    except Exception:
        CFG_NOTES.append(f"{key}={raw!r} not numeric; using {default}")
        return default
    if not math.isfinite(v) or not (lo <= v <= hi):
        CFG_NOTES.append(f"{key}={v} outside [{lo}, {hi}]; using {default}")
        return default
    return v


# ====================================================== ARENA (mirrors arena.ts)
# frontend/src/lib/arena.ts is the SINGLE SOURCE OF TRUTH for arena geometry.
# What is mirrored here is its DERIVATION, not its output: the same ARENA_MM and
# the same two fractions it uses (`const Z = 0.3 * ARENA_MM`, `const M = 0.04 *
# ARENA_MM`), so rescaling the arena there is still a one-number change here.
# Zone centres and the start pose are computed, never typed in.
#
# rover/tests parse arena.ts and compare it against these values, so a drift
# between the two files fails a test instead of driving the rover to the wrong
# corner.
ARENA_MM = 1200.0
ZONE_FRAC = 0.30        # arena.ts: const Z
MARGIN_FRAC = 0.04      # arena.ts: const M
ZONE_MM = ZONE_FRAC * ARENA_MM
MARGIN_MM = MARGIN_FRAC * ARENA_MM

# arena.ts: FORWARD_HEADING_DEG. +y is up and psi is CCW from +x, so "up the
# arena" is 90 and nothing else.
FORWARD_HEADING_DEG = 90.0

# Rects anchored BOTTOM-LEFT, exactly as arena.ts ZONES.
ZONES = {
    "zone-a": {"label": "ZONE A", "x_mm": MARGIN_MM,
               "y_mm": ARENA_MM - MARGIN_MM - ZONE_MM,           # top-left
               "w_mm": ZONE_MM, "h_mm": ZONE_MM},
    "zone-b": {"label": "ZONE B", "x_mm": ARENA_MM - MARGIN_MM - ZONE_MM,
               "y_mm": ARENA_MM - MARGIN_MM - ZONE_MM,           # top-right
               "w_mm": ZONE_MM, "h_mm": ZONE_MM},
    "water-station": {"label": "WATER", "x_mm": MARGIN_MM,
                      "y_mm": MARGIN_MM,                         # bottom-left
                      "w_mm": ZONE_MM, "h_mm": ZONE_MM},
}

# arena.ts ROVER_START: the bottom-right start box, nose FORWARD.
ROVER_START = {
    "x_mm": ARENA_MM - MARGIN_MM - ZONE_MM / 2.0,
    "y_mm": MARGIN_MM + ZONE_MM / 2.0,
    "heading_deg": FORWARD_HEADING_DEG,
}


def zone_center(zone_id):
    """Centre of a zone rect, world mm. Derived, so a rescale follows for free."""
    z = ZONES[zone_id]
    return (z["x_mm"] + z["w_mm"] / 2.0, z["y_mm"] + z["h_mm"] / 2.0)


# The four single-target missions. `home` is the start pose, not a zone rect,
# and is the only one with a heading requirement of its own.
MISSIONS = ("m1", "m2", "water", "home")


def mission_target(name):
    """(x_mm, y_mm, final_heading_deg or None) for a single-target mission."""
    if name == "m1":
        x, y = zone_center("zone-a")
        return x, y, None
    if name == "m2":
        x, y = zone_center("zone-b")
        return x, y, None
    if name == "water":
        x, y = zone_center("water-station")
        return x, y, None
    if name == "home":
        return ROVER_START["x_mm"], ROVER_START["y_mm"], ROVER_START["heading_deg"]
    raise KeyError(name)


# WHICH PHYSICAL CORNER EACH NAME MEANS — READ BEFORE WRITING ANY LABEL
# =====================================================================
# The mission ids are historical and they do NOT line up with what an operator
# says out loud. Standing behind the rover at the start box:
#
#     the zone STRAIGHT AHEAD (top-right, `zone-b`) is mission `m2`
#     the FAR zone (top-left, `zone-a`)             is mission `m1`
#
# so the operator's spoken "Zone 1" — meaning the one in front of them — is the
# code's `m2`. Nothing is renumbered here: renaming ids to match one operator's
# vocabulary would silently change what every stored command, log line and
# dashboard button means. Instead every operator-facing string names the CORNER,
# because a corner is the one description that cannot be read two ways.
TARGET_LABEL = {
    "m1": "top-LEFT zone (zone-a, the far zone from the start box)",
    "m2": "top-RIGHT zone (zone-b, straight ahead of the start box)",
    "water": "bottom-LEFT water station",
    "home": "start box, bottom-RIGHT, facing up the arena",
}


# ====================================================================== ROUTES
# A ROUTE is an ordered list of single-target missions driven back to back
# without returning between them, finished by ONE retrace of the whole thing.
#
# Legs are named by their single-target mission id rather than by coordinates so
# there stays exactly one definition of where each target is (`mission_target`).
# A route therefore cannot drift away from the single-target mission it is built
# from — they are the same numbers by construction.
#
# WHY THIS ORDER (m2 -> m1 -> water) AND NOT ANOTHER
# --------------------------------------------------
# Two orders tie for shortest at 2232 mm of outbound travel: m2->m1->water and
# water->m1->m2. Both walk three sides of the arena perimeter with two 90 deg
# turns; every other permutation needs a diagonal and costs 300-600 mm more.
# The tie is broken by the FIRST leg: the rover starts at the bottom-right
# facing 90 deg, which points exactly at m2, so leg 1 needs NO turn at all. Each
# turn is worth +/-1-4 deg of heading error that every later leg inherits, so
# the order that spends zero turns before the first drive is the one to take.
# It also happens to match the order an operator reads out ("zone in front,
# then the far zone, then water"), which is a nice accident and not the reason.
ROUTES = {
    "patrol": {
        "label": "ALL TARGETS",
        "legs": ("m2", "m1", "water"),
        # Operator-facing, corner-named. Never "zone 1 then zone 2".
        "describe": ("top-RIGHT zone, then top-LEFT zone, then the bottom-LEFT "
                     "water station, then RETRACE the whole path back to the "
                     "start box"),
    },
}

# Everything the `mission` verb will accept. Single-target missions keep their
# own tuple because `mission_target` is defined for exactly those and several
# payloads iterate it.
COMMANDABLE = MISSIONS + tuple(ROUTES)


def is_route(name):
    return name in ROUTES


def route_legs(name):
    """Ordered [(leg_name, x_mm, y_mm, final_heading_deg or None), ...].

    A single-target mission is returned as a ONE-LEG route on purpose. The
    executor then has exactly one shape to run, so there is no second code path
    that could drift away from the one the single-target missions have already
    been proven on — the multi-leg case is the single case with the loop running
    more than once, and nothing else.
    """
    if name in ROUTES:
        return [(leg,) + tuple(mission_target(leg)) for leg in ROUTES[name]["legs"]]
    return [(name,) + tuple(mission_target(name))]


def route_description(name):
    """One operator-facing sentence naming the physical corners, never numbers."""
    if name in ROUTES:
        return ROUTES[name]["describe"]
    return TARGET_LABEL.get(name, name)


def in_arena(x_mm, y_mm, pad_mm=0.0):
    return (pad_mm <= x_mm <= ARENA_MM - pad_mm
            and pad_mm <= y_mm <= ARENA_MM - pad_mm)


# ================================================================ CALIBRATION
# Same measurement, same constant, same reason as fpms_teleop.py: commanding
# linear.x = 0.10 produced roughly 0.61 m/s of ground speed. Everything in this
# file is expressed in REAL m/s and divided by CMD_SCALE on the way to the wire.
# If the rover is re-measured, this is the only edit — in both files.
CMD_SCALE = 6.1
WIRE_FLOOR_MPS = 0.0145   # one encoder count per 10 ms PID period; see docstring


def to_cmd(desired_mps):
    return desired_mps / CMD_SCALE


def to_cmd_ang(desired_radps):
    # The 6x was measured on the linear axis only, so applying it to rotation is
    # an assumption — but one that can only turn the rover SLOWER than asked,
    # which is the correct direction to be wrong in. Turns are closed loop on the
    # gyro and self-correct for it being wrong either way.
    return desired_radps / CMD_SCALE


# ===================================================================== SPEEDS
# CRUISE_MPS sits at ~2 encoder counts per PID period on the wire (0.18 / 6.1 =
# 0.0295, vs the 0.0145 floor), i.e. one full count of margin. Anything slower
# is inside the quantisation noise described in the docstring.
#
# NOTE THAT THIS IS FASTER THAN TELEOP'S HARD_MAX_LIN_MPS (0.12 real). That is
# not an oversight and it is not a licence to speed: teleop's envelope is a
# hand-on-the-joystick envelope, and all of it sits BELOW the firmware's
# expressible floor (0.05 real = 0.0082 wire). A mission cannot be run down
# there — it would stall and lurch its way across the arena. Slowness here is
# bought with short segments and full stops, which is the only currency the
# firmware accepts.
CRUISE_MPS = _cfg_float("FPMS_MISSION_CRUISE_MPS", 0.18, WIRE_FLOOR_MPS * CMD_SCALE, 0.25)

# EQUAL TO CRUISE BY DESIGN. See the docstring: the dock is slow because of
# DOCK_STEP_MM and the stop between steps, never because of this number.
DOCK_MPS = CRUISE_MPS

TURN_RADPS = _cfg_float("FPMS_MISSION_TURN_RADPS", 0.45, 0.10, 0.90)

# Last-line clamps, applied immediately before the wire, in real units. No
# command path — present or future, MQTT or internal — can get past them.
HARD_MAX_LIN_MPS = 0.25
HARD_MAX_ANG_RADPS = 0.90

# Heading hold during a drive leg. phase6 used a P gain of 10 clamped to +/-6 in
# its own power units; this is the same shape in rad/s. Corrections below the
# firmware's angular floor are silently ignored by the board, which is exactly
# why legs are SHORT rather than why the gain is high — a long leg cannot be
# steered straight by a correction the hardware cannot express.
HEADING_KP = 1.2                    # rad/s per rad of error
HEADING_CORR_MAX_RADPS = 0.25


# ================================================== SEGMENTATION / TOLERANCES
MAX_LEG_MM = _cfg_float("FPMS_MISSION_MAX_LEG_MM", 300.0, 50.0, 600.0)
MIN_LEG_MM = 30.0        # shorter than this is stop-settle noise, so it is
                         # folded into the previous leg instead of commanded
DOCK_APPROACH_MM = _cfg_float("FPMS_MISSION_DOCK_APPROACH_MM", 150.0, 40.0, 400.0)
DOCK_STEP_MM = _cfg_float("FPMS_MISSION_DOCK_STEP_MM", 40.0, 15.0, 100.0)

ARRIVE_TOL_MM = 25.0     # closer than this and another leg is noise, not progress
BEARING_TOL_DEG = 4.0    # heading error under this is not worth a turn segment
HEADING_TOL_DEG = 3.0    # re-face tolerance at the end of a mission
MAX_REFACE_DEG = 30.0    # a bigger error means the retrace failed; report it,
                         # do not spin the rover round trying to hide it

# THE SEGMENT BUDGET IS PER LEG, NOT PER ROUTE. This is a deliberate choice and
# it is worth the two constants.
#
# What the budget exists to catch is ADAPTIVE RE-PLANNING THAT WILL NOT
# CONVERGE: `_drive_to` loops "measure bearing -> turn or drive" until the
# target is inside ARRIVE_TOL_MM, and a rover that is oscillating instead of
# closing must be stopped rather than left to churn. Convergence is a property
# of ONE target. A route that converges perfectly on every leg would trip a
# per-route counter purely for having more legs, and — worse — the counter would
# mean something different on leg 3 than on leg 1, so the same misbehaviour
# would abort early in the route and be tolerated late in it. That is a guard
# whose meaning drifts, which is no guard at all.
#
# So MAX_SEGMENTS keeps its value and its meaning, applied per leg (a
# single-target mission is a one-leg route, so it is unchanged), and a separate
# absolute ceiling keeps the whole route bounded — because "every leg terminates"
# does not by itself prove "the route terminates" if legs could ever be added
# dynamically. Nominal `patrol` is ~20 outbound + ~20 retrace + 1 re-face; with
# a correction turn after most legs a real run lands near 60-70. 200 is room to
# be wrong without being unbounded, and MISSION_TIMEOUT_S bounds it in time too.
MAX_SEGMENTS = 40        # PER LEG: adaptive re-planning must terminate, always
MAX_ROUTE_SEGMENTS = 200  # PER ROUTE: absolute backstop across all legs + retrace

CONTROL_HZ = 20.0        # phase6's loop rate, carried over
CONTROL_DT = 1.0 / CONTROL_HZ

# phase6's turn-coast trick, and the reason it measured +/-1-4 deg: cut drive at
# 93 % of the target, let momentum carry the rest, then MEASURE what actually
# happened instead of assuming the request was honoured. The measurement is what
# the retrace replays, so this constant is load-bearing twice over.
TURN_COAST_FACTOR = 0.93
TURN_SETTLE_S = 1.2
DRIVE_COAST_FACTOR = 0.90
STOP_SETTLE_S = _cfg_float("FPMS_MISSION_STOP_SETTLE_S", 0.45, 0.20, 2.0)

HOLD_S = _cfg_float("FPMS_MISSION_HOLD_S", 2.0, 0.0, 30.0)

# Segment stall detection — commanded, but not moving. On this chassis that
# means the firmware dead zone swallowed the setpoint; continuing to command it
# just winds up integrator state for a lurch later.
STALL_CHECK_S = 2.0
STALL_MIN_MM = 5.0
TURN_STALL_CHECK_S = 2.5
TURN_STALL_MIN_DEG = 2.0

MISSION_TIMEOUT_S = _cfg_float("FPMS_MISSION_TIMEOUT_S", 240.0, 30.0, 900.0)


# ======================================================== SAFETY / HEALTH
FRONT_STOP_MM = _cfg_float("FPMS_MISSION_FRONT_STOP_MM", 120.0, 60.0, 600.0)
FRONT_CONE_DEG = 30.0     # +/- about the nose
ROTATE_CLEAR_MM = 80.0    # any bearing, while turning in place
LIDAR_STALE_S = 3.0       # older than this and the obstacle guard is blind
ROS_DEAD_S = 3.0          # no odometry for this long => micro-ROS link is down
BATT_LOW_V = 11.1

TELEM_HZ = 2.0
IDLE_TELEM_S = 5.0        # slow heartbeat so the dashboard can say "no mission"

# Preflight listen window for a foreign /cmd_vel writer. teleop idles at 2 Hz,
# so a window over 0.5 s sees at least one of its zeros.
FOREIGN_LISTEN_S = 1.2
FOREIGN_ABORT_MARGIN = 3  # extra /cmd_vel messages over a 2 s window that this
                          # node did not send => somebody else took the wire

NAV2_ACTION = "navigate_to_pose"
NAV2_WAIT_S = 3.0
NAV2_FRAME = CFG.get("FPMS_MISSION_MAP_FRAME", "map")
# Where arena (0,0) sits in the map frame, metres. With no map built yet this is
# an assumption stated once, not scattered through the goal builder.
MAP_ORIGIN_X_M = _cfg_float("FPMS_MISSION_MAP_ORIGIN_X_M", 0.0, -100.0, 100.0)
MAP_ORIGIN_Y_M = _cfg_float("FPMS_MISSION_MAP_ORIGIN_Y_M", 0.0, -100.0, 100.0)

BACKENDS = ("deadreckon", "nav2")
DEFAULT_BACKEND = CFG.get("FPMS_MISSION_BACKEND", "deadreckon")
if DEFAULT_BACKEND not in BACKENDS:
    CFG_NOTES.append(f"FPMS_MISSION_BACKEND={DEFAULT_BACKEND!r} unknown; using deadreckon")
    DEFAULT_BACKEND = "deadreckon"

# Set FPMS_MISSION_REQUIRE_LIDAR=0 only for a bench run with the LiDAR unplugged.
REQUIRE_LIDAR = CFG.get("FPMS_MISSION_REQUIRE_LIDAR", "1") not in ("0", "false", "no")
# Set FPMS_MISSION_ALLOW_SHARED_CMDVEL=1 only once teleop's idle heartbeat is
# suppressed and you are deliberately accepting the risk.
ALLOW_SHARED_CMDVEL = CFG.get("FPMS_MISSION_ALLOW_SHARED_CMDVEL", "0") in ("1", "true", "yes")


# ==================================================================== HELPERS
def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def wrap180(deg):
    """Fold degrees into (-180, 180]."""
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def wrap_pi(rad):
    r = (float(rad) + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if r == -math.pi else r


def heading_error_deg(target_deg, actual_deg):
    """Signed shortest rotation from actual to target, degrees CCW."""
    return wrap180(target_deg - actual_deg)


def bearing_deg(dx_mm, dy_mm):
    """World bearing of a displacement, degrees CCW from +x."""
    return math.degrees(math.atan2(dy_mm, dx_mm))


def jnum(v, nd=None):
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


def log(*a):
    print(time.strftime("[%H:%M:%S]"), *a, flush=True)


# ================================================================== SEGMENTS
@dataclass
class Segment:
    """One commanded motion, and what it actually did.

    `target` is signed: degrees CCW for a turn, millimetres along the current
    heading for a drive (negative = reverse). `measured` is filled in AFTER the
    segment settles and is the number the retrace inverts — never `target`.
    """
    kind: str                 # "turn" | "drive"
    target: float
    measured: float = 0.0
    dock: bool = False        # bounded dock burst rather than a cruise leg
    retrace: bool = False     # part of the replayed return
    reason: str = ""
    elapsed_s: float = 0.0
    lateral_mm: float = 0.0   # drift across the leg, reported not corrected

    @property
    def executed(self):
        return bool(self.reason)


def split_legs(dist_mm):
    """Break ONE LEG's straight run into bounded segments: cruise, then dock.

    (Historical name. What comes out are SEGMENTS of a single leg — see the
    module docstring's terminology note — not legs of a route.)

    Two different bounds for two different reasons. The cruise legs are bounded
    at MAX_LEG_MM so heading is re-measured often enough that dead reckoning
    stays honest and the obstacle guard is re-evaluated from a standstill. The
    final DOCK_APPROACH_MM is broken into DOCK_STEP_MM bursts because that — NOT
    a lower speed — is the only way this firmware can dock slowly.

    Returns [(mm, is_dock), ...] summing to dist_mm.
    """
    out = []
    d = float(dist_mm)
    if not math.isfinite(d) or d <= 0.0:
        return out

    dock_total = min(DOCK_APPROACH_MM, d)
    cruise = d - dock_total

    while cruise > 1e-6:
        leg = min(MAX_LEG_MM, cruise)
        # Never leave a stub behind: a leg under MIN_LEG_MM is shorter than the
        # settle noise it would be measured against, so absorb it here.
        if 0.0 < cruise - leg < MIN_LEG_MM:
            leg = cruise
        out.append((leg, False))
        cruise -= leg

    if dock_total > 1e-6:
        n = max(1, int(math.ceil(dock_total / DOCK_STEP_MM)))
        step = dock_total / n
        out.extend([(step, True)] * n)
    return out


def plan_route(x_mm, y_mm, heading_deg, tx_mm, ty_mm):
    """Nominal turn-then-drive plan from a pose to a point.

    Nominal because execution re-plans: after every leg the bearing is
    re-measured and a correction turn inserted if it has drifted past
    BEARING_TOL_DEG. This plan is what telemetry counts segments against and
    what the ETA is computed from.
    """
    dx, dy = tx_mm - x_mm, ty_mm - y_mm
    dist = math.hypot(dx, dy)
    segs = []
    if dist > ARRIVE_TOL_MM:
        turn = heading_error_deg(bearing_deg(dx, dy), heading_deg)
        if abs(turn) >= BEARING_TOL_DEG:
            segs.append(Segment("turn", turn))
        for mm, dock in split_legs(dist):
            segs.append(Segment("drive", mm, dock=dock))
    return segs


def plan_multi_route(x_mm, y_mm, heading_deg, targets):
    """Chain `plan_route` across an ordered list of (x_mm, y_mm) waypoints.

    Returns [((tx_mm, ty_mm), [Segment, ...]), ...] — grouped by leg, because
    the segment budget, the telemetry phase and the hold at each target are all
    per-leg ideas and flattening them here would only mean regrouping later.
    `route_segments` flattens when a flat list is what is wanted.

    THERE IS NO SECOND GEOMETRY ROUTINE HERE, AND THERE MUST NEVER BE ONE.
    Each leg is `plan_route` called again, and the pose it is planned from is the
    pose the previous leg ENDS at, obtained by walking that leg's own segments
    through `apply_segment` — the same function the executor's forward
    kinematics and the preview both use. Anything else (chaining bearings
    analytically, say) would be a parallel implementation of the same geometry,
    and the two would eventually disagree. A drawn route that disagrees with the
    driven one is worse than no route at all.

    `measured=False` because nothing has executed: a plan is made of targets.
    The executor still re-measures after every leg and inserts corrections, so
    this is the nominal intent, never a promise.
    """
    pose = (float(x_mm), float(y_mm), float(heading_deg))
    legs = []
    for tx, ty in targets:
        segs = plan_route(pose[0], pose[1], pose[2], tx, ty)
        pose = apply_segments(pose, segs, measured=False)
        legs.append(((tx, ty), segs))
    return legs


def route_segments(legs):
    """Flatten `plan_multi_route` output into one segment list, in drive order."""
    out = []
    for _target, segs in legs:
        out.extend(segs)
    return out


def invert_segments(segs):
    """THE RETRACE. Reverse the executed segments and negate what was MEASURED.

    This is the whole trick, and it is three lines because it has to be
    obviously correct:

      * reversed order, so the last thing done is the first thing undone;
      * sign flipped, so each motion is cancelled rather than repeated;
      * MEASURED magnitude, so the error in each outbound motion is undone by
        the same amount it was made by, and cancels instead of accumulating.

    Drives are replayed as reverse motion rather than as a turn-around, so the
    outbound turns are undone in place and the rover arrives already facing the
    start heading. Segments that never executed are skipped — an aborted segment
    moved by whatever it measured before it aborted, which is what is recorded.
    """
    out = []
    for s in reversed(segs):
        if not s.executed:
            continue
        if abs(s.measured) < 1e-9:
            continue
        out.append(Segment(s.kind, -s.measured, dock=s.dock, retrace=True))
    return out


def apply_segment(pose, seg, measured=True):
    """Forward-kinematics one segment. pose = (x_mm, y_mm, heading_deg)."""
    x, y, h = pose
    val = seg.measured if (measured and seg.executed) else seg.target
    if seg.kind == "turn":
        return (x, y, wrap180(h + val))
    r = math.radians(h)
    return (x + val * math.cos(r), y + val * math.sin(r), h)


def apply_segments(pose, segs, measured=True):
    for s in segs:
        pose = apply_segment(pose, s, measured=measured)
    return pose


def segment_timeout_s(seg):
    """Per-segment ceiling: 3x the nominal duration, plus the settle, plus slack.

    Generous on purpose — this is the backstop for a segment that is making slow
    progress, while the stall detector is what catches one making none.
    """
    if seg.kind == "drive":
        nominal = abs(seg.target) / 1000.0 / max(CRUISE_MPS, 1e-6)
        return max(3.0, nominal * 3.0 + STOP_SETTLE_S + 2.0)
    nominal = math.radians(abs(seg.target)) / max(TURN_RADPS, 1e-6)
    return max(4.0, nominal * 3.0 + TURN_SETTLE_S + 2.0)


def eta_seconds(segs):
    """Nominal remaining time for a segment list, settles included."""
    total = 0.0
    for s in segs:
        if s.kind == "drive":
            total += abs(s.target) / 1000.0 / max(CRUISE_MPS, 1e-6) + STOP_SETTLE_S
        else:
            total += math.radians(abs(s.target)) / max(TURN_RADPS, 1e-6) + TURN_SETTLE_S
    return total


def route_eta_seconds(legs, returns_home):
    """Nominal wall time for a whole route, from `plan_multi_route` output.

    The retrace is counted as a SECOND OUTBOUND rather than estimated on its
    own: it replays the same motions with the same magnitudes, so by
    construction it costs what the outbound cost. Every target gets a HOLD_S,
    including the last one — the worker settles at every target it reaches,
    which the old single-target estimate for `home` quietly left out.
    """
    outbound = eta_seconds(route_segments(legs))
    total = outbound + len(legs) * HOLD_S
    return total + outbound if returns_home else total


def remaining_distance_mm(segs):
    return sum(abs(s.target) for s in segs if s.kind == "drive")


# ============================================================ OBSTACLE GUARD
def front_clearance_mm(ranges_m, cone_deg=FRONT_CONE_DEG, reverse=False,
                       range_max_m=6.0):
    """Nearest return inside the cone the rover is moving towards, mm or None.

    Fed from fpms-rover-agent's `telemetry/lidar` payload, because that agent
    owns the only live LiDAR on this rover — the board's own /scan is dead (all
    ranges 0.0) and must never be used here.

    The scan is 360 degrees, so reversing is guarded exactly as well as driving
    forwards: `reverse=True` looks at the cone about 180 rather than about 0.
    That is what makes a reverse retrace safe enough to prefer over turning
    around twice.

    Zeros are "no return", not "obstacle at the sensor" — the same trap Nav2
    falls into with a zero-filled LaserScan. Saturated bins are the driver's max
    range, not a surface (arena.ts RANGE_SATURATION_FRAC).
    """
    if not ranges_m:
        return None
    n = len(ranges_m)
    step = 360.0 / n
    centre = 180.0 if reverse else 0.0
    sat = range_max_m * 0.995
    best = None
    for i, r in enumerate(ranges_m):
        try:
            rv = float(r)
        except Exception:
            continue
        if not math.isfinite(rv) or rv <= 0.0 or rv >= sat:
            continue
        if abs(wrap180(i * step - centre)) > cone_deg:
            continue
        mm = rv * 1000.0
        if best is None or mm < best:
            best = mm
    return best


def min_clearance_mm(ranges_m, range_max_m=6.0):
    """Nearest return at any bearing — the guard used while turning in place."""
    return front_clearance_mm(ranges_m, cone_deg=180.0, reverse=False,
                              range_max_m=range_max_m)


# ===================================================================== FRAME
@dataclass
class Anchor:
    """Maps the board's odom frame onto the arena frame.

    Established once from a pose that is BELIEVED to be a known arena pose. With
    nothing localising this rover that belief is the assumed start pose, and
    `assumed` says so all the way out to the dashboard — which already renders a
    "POSE: SIMULATED" badge for exactly this case.
    """
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_yaw_deg: float = 0.0
    arena_x_mm: float = ROVER_START["x_mm"]
    arena_y_mm: float = ROVER_START["y_mm"]
    arena_heading_deg: float = ROVER_START["heading_deg"]
    assumed: bool = True


def arena_from_odom(anchor, odom_x, odom_y, odom_yaw_deg):
    """odom metres -> arena millimetres + heading degrees."""
    rot = math.radians(anchor.arena_heading_deg - anchor.odom_yaw_deg)
    dx = (odom_x - anchor.odom_x) * 1000.0
    dy = (odom_y - anchor.odom_y) * 1000.0
    c, s = math.cos(rot), math.sin(rot)
    return (anchor.arena_x_mm + dx * c - dy * s,
            anchor.arena_y_mm + dx * s + dy * c,
            wrap180(odom_yaw_deg + math.degrees(rot)))


def arena_to_map_m(x_mm, y_mm):
    """Arena millimetres -> Nav2 map frame metres. One place, one assumption."""
    return (MAP_ORIGIN_X_M + x_mm / 1000.0, MAP_ORIGIN_Y_M + y_mm / 1000.0)


def standoff_point(x_mm, y_mm, tx_mm, ty_mm, back_off_mm=DOCK_APPROACH_MM):
    """The point `back_off_mm` short of the target, along the approach line.

    Nav2 hands over here. Its controller cannot dock on this chassis — its
    velocity profile assumes a machine that can express slow motion — so the
    last DOCK_APPROACH_MM is always driven by the bounded-burst primitive
    regardless of backend.
    """
    dx, dy = tx_mm - x_mm, ty_mm - y_mm
    d = math.hypot(dx, dy)
    if d <= back_off_mm or d < 1e-6:
        return (x_mm, y_mm)
    k = (d - back_off_mm) / d
    return (x_mm + dx * k, y_mm + dy * k)


# =========================================================================
#                          EVERYTHING BELOW REQUIRES ROS
# =========================================================================

ABORT_STOP = "stop commanded"
ABORT_LINK = "micro-ROS link down"
ABORT_BATT = "battery low"
ABORT_OBSTACLE = "obstacle inside stop distance"
ABORT_TIMEOUT = "mission timeout"
ABORT_SEG_TIMEOUT = "segment timeout"
ABORT_STALL = "segment stalled"
ABORT_WIRE = "another /cmd_vel writer"
ABORT_SHUTDOWN = "service shutting down"


class Bus:
    """MQTT wrapper that never raises into the caller and reconnects forever.

    Same shape as teleop's: connect_async + loop_start, so a broker that is down
    at boot delays telemetry rather than preventing the node from starting. A
    mission in flight is completely unaffected by the broker going away — except
    that `stop` can no longer arrive, which is why the ROS-side watchdogs abort
    on their own rather than waiting to be told.
    """

    def __init__(self):
        self.client = mqtt.Client(client_id=f"{THING}-missions",
                                  callback_api_version=CallbackAPIVersion.VERSION2)
        if USER:
            self.client.username_pw_set(USER, PASS or None)
        self.client.reconnect_delay_set(min_delay=1, max_delay=15)
        self.client.will_set(f"fpms/{THING}/events/offline",
                             json.dumps({"thing": THING, "status": "offline",
                                         "svc": "missions",
                                         "reason": "unexpected disconnect"}),
                             qos=1, retain=False)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.connected = False
        self.on_command = None
        self.on_lidar = None

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
            # The one verb this service OWNS.
            client.subscribe(f"fpms/{THING}/commands/mission", qos=1)
            # Verbs it ACTS on and never ANSWERS — teleop owns the reply. See
            # the module docstring; this is the same discipline fpms_rover_agent
            # applies to teleop's verbs, for the same reason.
            for verb in ("stop", "estop", "auto_off", "set_coordinate"):
                client.subscribe(f"fpms/{THING}/commands/{verb}", qos=1)
            # The only live LiDAR on this rover. The board's /scan is dead.
            client.subscribe(f"fpms/{THING}/telemetry/lidar", qos=0)
            self.publish("events/online",
                         {"svc": "missions", "status": "online",
                          "actions": ["mission"],
                          "missions": list(COMMANDABLE),
                          "single_targets": list(MISSIONS),
                          "backends": list(BACKENDS),
                          "default_backend": DEFAULT_BACKEND,
                          "acts_silently_on": ["stop", "estop", "auto_off"],
                          "arena_mm": ARENA_MM,
                          "targets": {m: {"x_mm": jnum(mission_target(m)[0], 1),
                                          "y_mm": jnum(mission_target(m)[1], 1),
                                          "label": TARGET_LABEL.get(m)}
                                      for m in MISSIONS},
                          # Multi-leg routes, advertised with their leg order and
                          # a corner-naming description — a consumer that builds
                          # a button from `label` alone would otherwise have to
                          # guess which physical zone "zone 1" meant.
                          "routes": {r: {"legs": list(ROUTES[r]["legs"]),
                                         "label": ROUTES[r]["label"],
                                         "describe": ROUTES[r]["describe"],
                                         "leg_labels": [TARGET_LABEL.get(leg)
                                                        for leg in ROUTES[r]["legs"]]}
                                     for r in ROUTES},
                          "limits": {"cruise_mps": CRUISE_MPS,
                                     "dock_mps": DOCK_MPS,
                                     "turn_radps": TURN_RADPS,
                                     "max_leg_mm": MAX_LEG_MM,
                                     "dock_step_mm": DOCK_STEP_MM,
                                     "front_stop_mm": FRONT_STOP_MM,
                                     "mission_timeout_s": MISSION_TIMEOUT_S,
                                     "max_segments_per_leg": MAX_SEGMENTS,
                                     "max_segments_per_route": MAX_ROUTE_SEGMENTS,
                                     "batt_low_v": BATT_LOW_V}}, qos=1)
        except Exception as e:
            log(f"MQTT: on_connect error {e}")

    def _on_disconnect(self, _c, _u, *args):
        self.connected = False
        log("MQTT: disconnected; paho will retry")

    def _on_message(self, _c, _u, msg):
        # paho's network thread. Must return fast and must never raise: an
        # exception here kills the MQTT loop and with it the only channel a
        # `stop` can arrive on.
        try:
            topic = msg.topic
            try:
                payload = json.loads(msg.payload.decode() or "{}")
            except Exception:
                payload = {}
            if not isinstance(payload, dict):
                payload = {}
            if topic.endswith("/telemetry/lidar"):
                if self.on_lidar:
                    self.on_lidar(payload)
                return
            action = topic.rsplit("/", 1)[-1]
            if self.on_command:
                self.on_command(action, payload)
        except Exception as e:
            log(f"MQTT: message handler error {e}")

    def publish(self, suffix, payload, qos=0):
        try:
            payload.setdefault("ts", time.time())
            payload.setdefault("thing", THING)
        except Exception:
            pass
        if not self.connected:
            return False
        try:
            self.client.publish(f"fpms/{THING}/{suffix}",
                                json.dumps(payload), qos=qos)
            return True
        except Exception as e:
            log(f"MQTT publish failed {e}")
            return False


class MissionNode(Node):
    """Sensor state, the single /cmd_vel writer, and the mission worker.

    ROS callbacks only WRITE state under `self.lock`. The worker thread only
    READS it. The one exception is the halt latch, which is a threading.Event
    precisely so that any thread can set it and the publisher honours it without
    needing the lock.
    """

    def __init__(self, bus):
        super().__init__("fpms_missions")
        self.bus = bus
        self.lock = threading.RLock()

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", qos)

        # Position from /odom when fpms-odom-tf is running (it fuses pose with a
        # bias-corrected gyro heading and stamps with the Pi's clock), falling
        # back to the board's /odom_raw when it is not — that node is authored
        # but not yet installed, and a mission must not depend on it.
        self.create_subscription(Odometry, "/odom", self._on_odom_fused, qos)
        self.create_subscription(Odometry, "/odom_raw", self._on_odom_raw, qos)
        # Heading is ALWAYS integrated here, from /imu, never taken from an
        # orientation quaternion: the board does not fuse orientation and
        # publishes identity on every message. Integrating locally also means
        # turns keep their +/-1-4 deg accuracy whether or not odom-tf is up.
        self.create_subscription(Imu, "/imu", self._on_imu, qos)
        self.create_subscription(UInt16, "/battery", self._on_battery, qos)
        # Listening to our own output topic is how a second writer is detected.
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, qos)

        self.odom_x = None
        self.odom_y = None
        self.odom_last = 0.0
        self.odom_source = None       # "/odom" | "/odom_raw"
        self.odom_fused_last = 0.0

        self.yaw_int = 0.0            # radians, continuous (NOT wrapped) — the
                                      # turn loop subtracts two samples of it,
                                      # so wrapping here would break a >180 turn
        self.gyro_z = 0.0
        self.gyro_bias = 0.0
        self.gyro_bias_n = 0
        self._imu_prev_t = None

        self.battery_raw = None
        self.battery_last = 0.0

        self.lidar = None
        self.lidar_last = 0.0

        self.anchor = self._initial_anchor()

        # /cmd_vel traffic accounting for the foreign-writer detector.
        self._pub_times = deque(maxlen=200)
        self._rx_times = deque(maxlen=400)

        self.halt = threading.Event()     # set => publish() forces zero
        self.shutdown = threading.Event()
        self.state = MissionState()

        self.create_timer(1.0 / TELEM_HZ, self._telemetry_tick)
        log("missions node up: /cmd_vel publisher; /odom + /odom_raw + /imu + "
            "/battery subs")
        for note in CFG_NOTES:
            log(f"config: {note}")

    # ------------------------------------------------------------ ROS input
    def _on_odom_fused(self, msg):
        self._take_odom(msg, "/odom")

    def _on_odom_raw(self, msg):
        # Only used while /odom is absent or stale. Both carry the same pose
        # information; /odom additionally has a heading this node can cross-check
        # against its own integration.
        with self.lock:
            fresh = (time.monotonic() - self.odom_fused_last) < 1.0
        if fresh:
            return
        self._take_odom(msg, "/odom_raw")

    def _take_odom(self, msg, source):
        try:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            if not (math.isfinite(x) and math.isfinite(y)):
                return
            now = time.monotonic()
            with self.lock:
                # POSE ONLY. msg.twist.linear.x is SIGN-INVERTED relative to this
                # very pose on this firmware, so no guard, tolerance or progress
                # check in this file reads it — every one of them differentiates
                # position instead. A guard built on twist reads a correct
                # forward move as a reversal and aborts it.
                self.odom_x = x
                self.odom_y = y
                self.odom_last = now
                self.odom_source = source
                if source == "/odom":
                    self.odom_fused_last = now
        except Exception as e:
            log(f"odom callback error {e}")

    def _on_imu(self, msg):
        try:
            now = time.monotonic()
            with self.lock:
                raw = float(msg.angular_velocity.z)
                if not math.isfinite(raw):
                    return
                # Zero-rate bias dominates an integrated heading, so estimate it
                # only while genuinely parked — commanded zero AND no measured
                # wheel motion — or a slow real rotation gets eaten as drift.
                if self.state.phase in ("idle", "hold") and not self.state.driving:
                    if self.gyro_bias_n < 2000:
                        self.gyro_bias_n += 1
                    a = 1.0 / min(max(self.gyro_bias_n, 1), 200)
                    self.gyro_bias = (1 - a) * self.gyro_bias + a * raw
                self.gyro_z = raw - self.gyro_bias
                dt = 0.0 if self._imu_prev_t is None else (now - self._imu_prev_t)
                self._imu_prev_t = now
                # A stalled link resuming must not inject a huge bogus angle into
                # a running turn.
                if 0.0 < dt < 0.5:
                    self.yaw_int += self.gyro_z * dt
        except Exception as e:
            log(f"imu callback error {e}")

    def _on_battery(self, msg):
        try:
            with self.lock:
                self.battery_raw = int(msg.data)   # DECIVOLTS
                self.battery_last = time.monotonic()
        except Exception as e:
            log(f"battery callback error {e}")

    def _on_cmd_vel(self, _msg):
        # rclpy delivers this node's own publications back to it, so a raw count
        # proves nothing on its own — see `foreign_writer()` for the comparison
        # that does.
        self._rx_times.append(time.monotonic())

    def on_lidar(self, payload):
        try:
            with self.lock:
                self.lidar = payload
                self.lidar_last = time.monotonic()
        except Exception:
            pass

    # --------------------------------------------------------------- state
    def _initial_anchor(self):
        """Assume the start pose unless the operator has told teleop otherwise."""
        a = Anchor()
        try:
            with open(TELEOP_ORIGIN_FILE) as fh:
                d = json.load(fh)
            a.odom_x = float(d["ref_x"])
            a.odom_y = float(d["ref_y"])
            a.arena_x_mm = float(d["x_mm"])
            a.arena_y_mm = float(d["y_mm"])
            # That file records POSITION only — teleop's set_coordinate has no
            # heading argument — so the heading half of the anchor stays assumed
            # whatever happens, and `assumed` stays true.
            log(f"anchor: position from {TELEOP_ORIGIN_FILE} "
                f"({a.arena_x_mm:.0f}, {a.arena_y_mm:.0f}) mm; heading assumed FORWARD")
        except Exception:
            log(f"anchor: assumed start pose ({a.arena_x_mm:.0f}, "
                f"{a.arena_y_mm:.0f}) mm heading {a.arena_heading_deg:.0f}deg "
                "— nothing localises this rover")
        return a

    def pose(self):
        """(x_mm, y_mm, heading_deg) in the arena frame, or None with no odom."""
        with self.lock:
            if self.odom_x is None:
                return None
            return arena_from_odom(self.anchor, self.odom_x, self.odom_y,
                                   math.degrees(self.yaw_int))

    def odom_xy(self):
        with self.lock:
            return self.odom_x, self.odom_y

    def yaw(self):
        with self.lock:
            return self.yaw_int

    def link_ok(self):
        with self.lock:
            return (self.odom_x is not None
                    and (time.monotonic() - self.odom_last) < ROS_DEAD_S)

    def battery_v(self):
        with self.lock:
            return None if self.battery_raw is None else self.battery_raw / 10.0

    def lidar_fresh(self):
        with self.lock:
            return self.lidar is not None and (time.monotonic() - self.lidar_last) < LIDAR_STALE_S

    def clearance_mm(self, reverse=False, any_bearing=False):
        """Nearest obstacle in the direction of travel, or None if blind."""
        with self.lock:
            if self.lidar is None or (time.monotonic() - self.lidar_last) >= LIDAR_STALE_S:
                return None
            ranges = self.lidar.get("ranges_m") or []
            rmax = float(self.lidar.get("range_max_m") or 6.0)
        if any_bearing:
            return min_clearance_mm(ranges, range_max_m=rmax)
        return front_clearance_mm(ranges, reverse=reverse, range_max_m=rmax)

    def foreign_writer(self):
        """True when /cmd_vel carries more traffic than this node produced.

        Counted over a window rather than matched per-message: at 20 Hz our own
        publications are 50 ms apart, so any timestamp-matching scheme would
        happily adopt a stranger's message as one of ours. Counting cannot be
        fooled that way.
        """
        now = time.monotonic()
        win = 2.0
        rx = sum(1 for t in self._rx_times if now - t <= win)
        tx = sum(1 for t in self._pub_times if now - t <= win)
        return rx > tx + FOREIGN_ABORT_MARGIN

    # ------------------------------------------------------------- the wire
    def publish(self, vx_real, wz_real):
        """The ONLY place a non-zero Twist leaves this process.

        The halt latch is honoured HERE rather than at the call sites, so a
        `stop` that lands between a worker's abort check and its publish still
        cannot put motion on the wire. There is no ramp: ramping through speeds
        the firmware cannot express is fiction (see the docstring), so the
        setpoint is commanded directly and stopping means commanding exactly 0.
        """
        try:
            if self.halt.is_set() or self.shutdown.is_set():
                vx_real = wz_real = 0.0
            # Last-line clamps, in real units, after every decision above.
            vx = clamp(float(vx_real), -HARD_MAX_LIN_MPS, HARD_MAX_LIN_MPS)
            wz = clamp(float(wz_real), -HARD_MAX_ANG_RADPS, HARD_MAX_ANG_RADPS)
            t = Twist()
            t.linear.x = float(to_cmd(vx))
            t.linear.y = 0.0
            t.linear.z = 0.0
            t.angular.x = 0.0
            t.angular.y = 0.0
            t.angular.z = float(to_cmd_ang(wz))
            self.pub_cmd.publish(t)
            self._pub_times.append(time.monotonic())
        except Exception as e:
            log(f"cmd_vel publish failed {e}")

    def stop_wire(self, n=3):
        """Command a dead stop. A zero Twist is the one message that is always
        safe to send from any process in any state, so this never checks
        ownership of the wire and never gives up on an exception."""
        for _ in range(max(1, n)):
            try:
                t = Twist()
                self.pub_cmd.publish(t)
                self._pub_times.append(time.monotonic())
            except Exception:
                pass
            time.sleep(0.02)

    # ---------------------------------------------------------- telemetry
    def _telemetry_tick(self):
        try:
            st = self.state
            active = st.phase not in ("idle",)
            if not active and (time.monotonic() - st.last_idle_pub) < IDLE_TELEM_S:
                return
            if not active:
                st.last_idle_pub = time.monotonic()
            self.bus.publish("telemetry/mission", self.snapshot())
        except Exception as e:
            log(f"telemetry error {e}")

    def snapshot(self):
        st = self.state
        p = self.pose()
        with self.lock:
            src = self.odom_source
        out = {
            "mission": st.name,
            "backend": st.backend,
            "phase": st.phase,
            "segment_i": st.segment_i,
            "segments_n": st.segments_n,
            "segment_kind": st.segment_kind,
            # Route progress. `leg_label` names the CORNER rather than a number
            # so an operator reading the card cannot mistake which zone the
            # rover is at (see TARGET_LABEL).
            "leg_i": st.leg_i,
            "legs_n": st.legs_n,
            "leg": st.leg_name,
            "leg_label": TARGET_LABEL.get(st.leg_name) if st.leg_name else None,
            "distance_remaining_mm": jnum(st.distance_remaining_mm, 1),
            "distance_travelled_mm": jnum(st.distance_travelled_mm, 1),
            "eta_s": jnum(st.eta_s, 1),
            "elapsed_s": jnum(st.elapsed(), 2),
            "target": ({"x_mm": jnum(st.target[0], 1), "y_mm": jnum(st.target[1], 1)}
                       if st.target else None),
            "pose_assumed": bool(self.anchor.assumed),
            "odom_source": src,
            "link_ok": self.link_ok(),
            "batt_v": jnum(self.battery_v(), 1),
            "front_mm": jnum(self.clearance_mm(reverse=st.reversing), 0),
            "lidar_ok": self.lidar_fresh(),
        }
        # Pose fields are OMITTED, not zero-filled, when there is no odometry.
        # A consumer that sees 0,0 cannot tell it apart from a real corner.
        if p:
            out["x_mm"] = jnum(p[0], 1)
            out["y_mm"] = jnum(p[1], 1)
            out["heading_deg"] = jnum(p[2], 1)
        return out


@dataclass
class MissionState:
    """Everything the telemetry tick needs, written only by the worker."""
    name: str = None
    backend: str = None
    phase: str = "idle"          # idle|planning|outbound|docking|hold|return|reface|aborting
    segment_i: int = 0
    segments_n: int = 0
    segment_kind: str = None
    # Which leg of a multi-leg route is being driven. A single-target mission is
    # leg 1 of 1, so these are always meaningful and never need a None check.
    leg_i: int = 0
    legs_n: int = 0
    leg_name: str = None
    leg_segment_i: int = 0       # reset per leg; the budget is measured on it
    leg_budget: int = MAX_SEGMENTS
    distance_remaining_mm: float = 0.0
    distance_travelled_mm: float = 0.0
    eta_s: float = 0.0
    target: tuple = None
    started: float = 0.0
    driving: bool = False        # commanded non-zero right now
    reversing: bool = False      # ...and in reverse, so the guard swaps cones
    last_idle_pub: float = 0.0

    def elapsed(self):
        return 0.0 if not self.started else (time.monotonic() - self.started)


class MissionAbort(Exception):
    """Raised out of any depth of the worker; carries the operator-facing reason."""

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


# ================================================================ BACKENDS
class DeadReckonBackend:
    """Turn-then-drive segments with a retrace return. THE DEFAULT.

    Every primitive here follows the same three-beat rhythm, which is the whole
    reason the prior code measured 0.6 % and +/-1-4 deg:

        command at a speed the firmware can hold
        -> cut early and coast
        -> STOP, settle, and MEASURE what actually happened.

    The measurement is not decoration. It is what the retrace inverts, so an
    unmeasured segment is a segment whose error goes home with the rover.
    """

    name = "deadreckon"

    def __init__(self, node, runner):
        self.node = node
        self.runner = runner

    def available(self):
        return True, ""

    # -- primitives --------------------------------------------------------
    def run_segment(self, seg):
        if seg.kind == "turn":
            return self._turn(seg)
        return self._drive(seg)

    def _turn(self, seg):
        node, runner = self.node, self.runner
        t0 = time.monotonic()
        yaw0 = node.yaw()
        target_rad = math.radians(seg.target)
        sign = 1.0 if seg.target >= 0 else -1.0
        # THE COAST TRICK: stop driving at 93 % of the requested angle and let
        # momentum carry the rest. Driving all the way to the target overshoots
        # by however much the chassis carries, every time, in the same direction
        # — which is a bias, and a bias is the one error a retrace cannot cancel
        # because it appears identically on the way back.
        stop_at = abs(target_rad) * TURN_COAST_FACTOR
        timeout = segment_timeout_s(seg)
        reason = "done"

        node.state.driving = True
        node.state.reversing = False
        try:
            while True:
                runner.check_abort(turning=True)
                now = time.monotonic()
                turned = abs(node.yaw() - yaw0)
                if turned >= stop_at:
                    break
                if now - t0 > timeout:
                    reason = ABORT_SEG_TIMEOUT
                    break
                if (now - t0 > TURN_STALL_CHECK_S
                        and math.degrees(turned) < TURN_STALL_MIN_DEG):
                    # Commanded but not rotating: the firmware dead zone ate the
                    # setpoint. Continuing to command it only stores up a lurch.
                    reason = ABORT_STALL
                    break
                node.publish(0.0, sign * TURN_RADPS)
                time.sleep(CONTROL_DT)
        finally:
            node.publish(0.0, 0.0)
            node.state.driving = False

        # Coast, then measure. The settle is polled rather than slept so a stop
        # still lands inside it.
        runner.settle(TURN_SETTLE_S)
        seg.measured = math.degrees(node.yaw() - yaw0)
        seg.elapsed_s = time.monotonic() - t0
        seg.reason = reason
        if reason in (ABORT_SEG_TIMEOUT, ABORT_STALL):
            raise MissionAbort(f"{reason} (turn asked {seg.target:+.1f}deg, "
                               f"measured {seg.measured:+.1f}deg)")
        return seg

    def _drive(self, seg):
        node, runner = self.node, self.runner
        t0 = time.monotonic()
        x0, y0 = node.odom_xy()
        if x0 is None:
            raise MissionAbort(ABORT_LINK)
        yaw0 = node.yaw()
        sign = 1.0 if seg.target >= 0 else -1.0
        reverse = sign < 0
        target_mm = abs(seg.target)
        stop_at = target_mm * DRIVE_COAST_FACTOR
        timeout = segment_timeout_s(seg)
        speed = DOCK_MPS if seg.dock else CRUISE_MPS
        reason = "done"
        along = lateral = 0.0

        node.state.driving = True
        node.state.reversing = reverse
        try:
            while True:
                runner.check_abort(reverse=reverse)
                now = time.monotonic()
                along, lateral = self._displacement(x0, y0, yaw0)
                if sign * along >= stop_at:
                    break
                if now - t0 > timeout:
                    reason = ABORT_SEG_TIMEOUT
                    break
                if now - t0 > STALL_CHECK_S and abs(along) < STALL_MIN_MM:
                    reason = ABORT_STALL
                    break
                # Heading hold. Corrections smaller than the firmware's angular
                # floor simply do not reach the wheels, which is why legs are
                # bounded at MAX_LEG_MM instead of relying on this to steer.
                err = wrap_pi(yaw0 - node.yaw())
                wz = clamp(HEADING_KP * err, -HEADING_CORR_MAX_RADPS,
                           HEADING_CORR_MAX_RADPS)
                node.publish(sign * speed, wz)
                time.sleep(CONTROL_DT)
        finally:
            node.publish(0.0, 0.0)
            node.state.driving = False
            node.state.reversing = False

        # FULL STOP between segments. On a chassis that cannot express slow
        # motion this stop is the speed control: it is what makes a sequence of
        # bursts a slow approach instead of a lurch.
        runner.settle(STOP_SETTLE_S)
        along, lateral = self._displacement(x0, y0, yaw0)
        seg.measured = along
        seg.lateral_mm = lateral
        seg.elapsed_s = time.monotonic() - t0
        seg.reason = reason
        if reason in (ABORT_SEG_TIMEOUT, ABORT_STALL):
            raise MissionAbort(f"{reason} (drive asked {seg.target:+.0f}mm, "
                               f"measured {seg.measured:+.0f}mm)")
        return seg

    def _displacement(self, x0, y0, yaw0):
        """Signed travel along the leg heading, and drift across it, in mm."""
        x, y = self.node.odom_xy()
        if x is None:
            raise MissionAbort(ABORT_LINK)
        dx = (x - x0) * 1000.0
        dy = (y - y0) * 1000.0
        c, s = math.cos(yaw0), math.sin(yaw0)
        return (dx * c + dy * s, -dx * s + dy * c)


class Nav2Backend:
    """NavigateToPose goals — the action `nav2_simple_commander` wraps.

    Deliberately NOT the default. Read NAV2_BRIEF.md section 7 before expecting
    this to work: Nav2 cannot localise without a LaserScan in ROS, the board's
    /scan is dead, and the real LiDAR publishes to MQTT only. Until that chain
    is built this backend's job is to refuse with the reason, which `available()`
    does before anything moves.

    Two things stay with the dead-reckoning primitives even when Nav2 drives:

      DOCKING. Nav2's controller assumes a machine that can express slow motion.
      This one cannot, so the goal is a standoff pose DOCK_APPROACH_MM short of
      the target and the last stretch is always bounded bursts.

      THE WIRE. While a goal is active, Nav2's controller_server is the writer,
      so this node's control loop publishes nothing and its obstacle guard steps
      aside (Nav2 has its own costmaps). Normal termination is therefore two
      steps in order: cancel the goal, THEN take the wire back and stop.

      A `stop` is the exception and stays unconditional — a zero Twist is safe
      from any process at any time, and a stop that waits politely for a goal to
      cancel is not a stop.
    """

    name = "nav2"

    def __init__(self, node, runner):
        self.node = node
        self.runner = runner
        self.client = None
        self.goal_handle = None
        self.feedback = None

    def available(self):
        if not HAVE_NAV2:
            return False, (f"nav2 packages not importable ({NAV2_IMPORT_ERROR}); "
                           "nav2_msgs/nav2_simple_commander are not installed")
        try:
            if self.client is None:
                self.client = ActionClient(self.node, NavigateToPose, NAV2_ACTION)
            if not self.client.wait_for_server(timeout_sec=NAV2_WAIT_S):
                return False, (f"nav2 action server '{NAV2_ACTION}' did not appear "
                               f"in {NAV2_WAIT_S:.0f}s — the LiDAR->ROS->TF->map->"
                               "AMCL chain is not up (see NAV2_BRIEF.md s7)")
        except Exception as e:
            return False, f"nav2 action client failed: {e}"
        return True, ""

    def goto(self, x_mm, y_mm, heading_deg):
        """Send one goal and block until it terminates, publishing progress."""
        ok, why = self.available()
        if not ok:
            raise MissionAbort(why)

        mx, my = arena_to_map_m(x_mm, y_mm)
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = NAV2_FRAME
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.pose.position.x = float(mx)
        ps.pose.position.y = float(my)
        half = math.radians(heading_deg) / 2.0
        ps.pose.orientation.z = math.sin(half)
        ps.pose.orientation.w = math.cos(half)
        goal.pose = ps

        self.feedback = None
        send = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        t0 = time.monotonic()
        while not send.done():
            self.runner.check_abort(nav2_active=True)
            if time.monotonic() - t0 > NAV2_WAIT_S * 2:
                raise MissionAbort("nav2 goal was never accepted")
            time.sleep(0.05)
        self.goal_handle = send.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            raise MissionAbort("nav2 rejected the goal")

        result_future = self.goal_handle.get_result_async()
        try:
            while not result_future.done():
                # Nav2 owns the wire here — check_abort must not put a Twist on
                # it, so the nav2_active flag suppresses the local stop and lets
                # the cancel below do the stopping.
                self.runner.check_abort(nav2_active=True)
                if self.runner.state.elapsed() > MISSION_TIMEOUT_S:
                    raise MissionAbort(ABORT_TIMEOUT)
                time.sleep(0.05)
        except MissionAbort:
            self.cancel()
            raise
        self.goal_handle = None
        return True

    def cancel(self):
        gh, self.goal_handle = self.goal_handle, None
        if gh is None:
            return
        try:
            fut = gh.cancel_goal_async()
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 2.0:
                time.sleep(0.05)
        except Exception as e:
            log(f"nav2 cancel failed {e}")
        # Only now is the wire ours again.
        self.node.stop_wire()

    def _on_feedback(self, msg):
        try:
            fb = msg.feedback
            self.feedback = fb
            st = self.runner.state
            st.distance_remaining_mm = float(fb.distance_remaining) * 1000.0
            st.eta_s = float(fb.estimated_time_remaining.sec)
        except Exception:
            pass


# ================================================================== RUNNER
class MissionRunner:
    """Accepts commands, refuses them with reasons, and runs the one worker."""

    def __init__(self, node, bus):
        self.node = node
        self.bus = bus
        self.state = node.state
        self.thread = None
        self.abort_reason = None
        self.lock = threading.Lock()
        self.backends = {"deadreckon": DeadReckonBackend(node, self),
                         "nav2": Nav2Backend(node, self)}

    # ------------------------------------------------------------ commands
    def handle_command(self, action, payload):
        """Runs on paho's network thread. Sets state; never blocks; never moves."""
        try:
            if action in ("stop", "estop", "auto_off"):
                # ACT, DO NOT ANSWER. teleop owns the reply for these verbs; two
                # processes racing an ack for the same stop is how an operator
                # ends up reading the wrong one.
                self.request_abort(ABORT_STOP)
                return
            if action == "set_coordinate":
                # ACT, DO NOT ANSWER — teleop owns this verb and its ack.
                #
                # Without this the anchor is read once at boot and never again,
                # so an operator who repositions the rover and re-zeros it via
                # teleop gets a correctly-updated origin file and a mission node
                # still planning from the OLD origin. Every route would be
                # silently offset by however far the rover had been moved, and
                # the preview would draw that wrong route confidently.
                self._reload_anchor()
                return
            if action != "mission":
                return
            self._cmd_mission(payload)
        except Exception as e:
            log(f"command handler error {e}")

    def _nack(self, reason, **extra):
        p = {"action": "mission", "error": reason, "accepted": False}
        p.update(extra)
        log(f"mission NACK: {reason}")
        self.bus.publish("events/nack", p, qos=1)

    def _cmd_mission(self, payload):
        name = str(payload.get("name", "") or "")
        backend = str(payload.get("backend", "") or DEFAULT_BACKEND)

        if name not in COMMANDABLE:
            self._nack(f"unknown mission {name!r}", valid=list(COMMANDABLE))
            return
        if backend not in BACKENDS:
            self._nack(f"unknown backend {backend!r}", valid=list(BACKENDS))
            return

        # PLAN ONLY. Answers "where would you go?" without touching the wire.
        # Handled before every check below on purpose: a preview commands no
        # motion, so refusing it for a flat battery, a stale LiDAR or a busy
        # wire would withhold exactly the information an operator needs while
        # deciding whether to run the thing at all. It still refuses without a
        # pose, because a route drawn from a guessed start is a lie on a map.
        if payload.get("preview"):
            self._preview(name, backend)
            return

        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                self._nack(f"mission {self.state.name!r} already running "
                           f"(phase {self.state.phase}); send stop first",
                           running=self.state.name)
                return

            # ---- refusals, each naming exactly what is wrong ----
            if not self.node.link_ok():
                self._nack(f"{ABORT_LINK}: no odometry for >{ROS_DEAD_S:.0f}s "
                           "(not restarting the agent — it self-reconnects in 90-225s)")
                return
            v = self.node.battery_v()
            if v is not None and v < BATT_LOW_V:
                self._nack(f"{ABORT_BATT}: {v:.1f}V < {BATT_LOW_V}V threshold")
                return
            if REQUIRE_LIDAR and not self.node.lidar_fresh():
                self._nack("no LiDAR telemetry: the front obstacle guard would be "
                           "blind (fpms-rover-agent owns the only live scanner; "
                           "the board's /scan is dead). "
                           "Set FPMS_MISSION_REQUIRE_LIDAR=0 to override.")
                return
            pose = self.node.pose()
            if pose is None:
                self._nack("no pose: odometry has not been seen yet")
                return
            legs = route_legs(name)
            for leg_name, tx, ty, _lh in legs:
                if not in_arena(tx, ty):
                    self._nack(f"leg {leg_name!r} target ({tx:.0f}, {ty:.0f}) mm "
                               "is outside the arena")
                    return

            b = self.backends[backend]
            ok, why = b.available()
            if not ok:
                self._nack(why, backend=backend, fallback="deadreckon")
                return

            if not self._wire_is_ours():
                self._nack(
                    "another process is publishing /cmd_vel (fpms-teleop's idle "
                    "zero heartbeat, most likely). Interleaving its zeros with "
                    "this node's setpoint produces stall-then-lurch, so the "
                    "mission is refused rather than fought for. Stop fpms-teleop "
                    "or suppress its idle heartbeat while a mission is active; "
                    "see fpms_missions.py, COEXISTENCE.")
                return

            # The plan is built BEFORE acceptance so the ETA below is a real
            # number rather than a guess, and so a route that cannot fit inside
            # the mission timeout is refused now instead of aborting halfway
            # round with the rover parked at the far corner.
            plan_legs = plan_multi_route(pose[0], pose[1], pose[2],
                                         [(t[1], t[2]) for t in legs])
            plan = route_segments(plan_legs)
            returns_home = name != "home"
            eta = route_eta_seconds(plan_legs, returns_home)
            if eta > MISSION_TIMEOUT_S:
                self._nack(
                    f"route {name!r} needs ~{eta:.0f}s nominal but "
                    f"MISSION_TIMEOUT_S is {MISSION_TIMEOUT_S:.0f}s, so it would "
                    "abort part-way and leave the rover away from the start box. "
                    "Raise FPMS_MISSION_TIMEOUT_S in /etc/fpms/config.env, or "
                    "run the legs as separate missions.",
                    eta_s=jnum(eta, 1), timeout_s=MISSION_TIMEOUT_S)
                return

            # ---- accepted ----
            self.node.halt.clear()
            self.abort_reason = None
            st = self.state
            st.name = name
            st.backend = backend
            st.phase = "planning"
            st.segment_i = 0
            st.segments_n = 0
            st.segment_kind = None
            st.distance_travelled_mm = 0.0
            st.leg_i = 0
            st.legs_n = len(legs)
            st.leg_name = None
            st.leg_segment_i = 0
            # The FINAL target of the route, so a consumer that draws one marker
            # draws the last place the rover is going rather than the first.
            st.target = (legs[-1][1], legs[-1][2])
            st.started = time.monotonic()

            st.segments_n = len(plan)
            st.distance_remaining_mm = remaining_distance_mm(plan)
            st.eta_s = eta

            self.thread = threading.Thread(
                target=self._run, name="mission",
                args=(name, backend, legs, pose), daemon=True)
            self.thread.start()

        route_str = " -> ".join(f"({t[1]:.0f},{t[2]:.0f})" for t in legs)
        log(f"mission {name!r} accepted on backend {backend!r}: "
            f"({pose[0]:.0f},{pose[1]:.0f})mm h={pose[2]:.0f}deg -> {route_str}mm, "
            f"{len(legs)} leg(s), {len(plan)} planned segments, "
            f"{'retrace home' if returns_home else 'one way'}")
        self.bus.publish("events/ack",
                         {"action": "mission", "name": name, "accepted": True,
                          "started": True, "backend": backend,
                          "route": route_description(name),
                          "legs": [{"name": t[0], "label": TARGET_LABEL.get(t[0]),
                                    "x_mm": jnum(t[1], 1), "y_mm": jnum(t[2], 1)}
                                   for t in legs],
                          "legs_n": len(legs),
                          # Kept for consumers written against the single-target
                          # payload: the route's LAST target, which for a
                          # one-leg mission is the same field it always was.
                          "target": {"x_mm": jnum(legs[-1][1], 1),
                                     "y_mm": jnum(legs[-1][2], 1)},
                          "from": {"x_mm": jnum(pose[0], 1), "y_mm": jnum(pose[1], 1),
                                   "heading_deg": jnum(pose[2], 1)},
                          "pose_assumed": bool(self.node.anchor.assumed),
                          "segments_planned": len(plan),
                          "eta_s": jnum(st.eta_s, 1),
                          "return_strategy": "retrace" if returns_home else "none",
                          "returns_home": returns_home}, qos=1)

    def _reload_anchor(self):
        """Re-read teleop's origin file after a set_coordinate.

        Refused while a mission is running. Moving the arena frame under a route
        that is being driven would leave the executor steering toward a target
        that has silently jumped, using distances measured against the old
        frame — the rover would keep driving and every number would be wrong.
        The operator can stop, re-zero, and start again.
        """
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                log("set_coordinate ignored: a mission is running and the arena "
                    "frame must not move under it. Stop the mission first.")
                return
        a = self.node._initial_anchor()
        with self.node.lock:
            self.node.anchor = a
        log(f"anchor reloaded after set_coordinate: "
            f"({a.arena_x_mm:.0f}, {a.arena_y_mm:.0f}) mm")

    def _preview(self, name, backend):
        """Publish the route this mission WOULD drive. Commands nothing.

        The waypoints are produced by walking `plan_multi_route` — which is
        `plan_route` called once per leg — through the same `apply_segment`
        forward kinematics the executor uses, rather than by a second geometry
        routine written for the map. That is the point: a preview computed a
        different way would eventually disagree with the drive, and a route
        drawn on a dashboard that the rover does not actually follow is worse
        than drawing nothing. A multi-leg route changes NOTHING about that rule;
        it just runs the same loop more than once.

        What it CANNOT show is re-planning. Execution re-measures the bearing
        after every leg and inserts corrections, so the real path deviates from
        this one — this is the nominal intent, not a promise. `nominal: true`
        says so to anyone rendering it.

        The RETURN is not drawn. It is a retrace, so its line is this line
        walked backwards: drawing it would put a second stroke exactly on top of
        the first and tell the operator nothing they cannot already see.
        `return_strategy: "retrace"` says what happens instead.
        """
        pose = self.node.pose()
        if pose is None:
            self._nack("no pose yet, so there is no route to preview "
                       "(odometry has not been seen)", preview=True)
            return

        legs = route_legs(name)
        for leg_name, tx, ty, _lh in legs:
            if not in_arena(tx, ty):
                self._nack(f"leg {leg_name!r} target ({tx:.0f}, {ty:.0f}) mm is "
                           "outside the arena", preview=True)
                return

        plan_legs = plan_multi_route(pose[0], pose[1], pose[2],
                                     [(t[1], t[2]) for t in legs])
        plan = route_segments(plan_legs)

        # Cumulative pose after each segment. measured=False because nothing has
        # executed — these are the TARGETS, which is what a preview means.
        #
        # One flat waypoint list, because that is what the arena map draws: a
        # polyline. Each point carries the leg it belongs to so a renderer that
        # wants to colour or label the legs can, without needing a second,
        # differently-shaped payload.
        pts = [{"x_mm": jnum(pose[0], 1), "y_mm": jnum(pose[1], 1),
                "heading_deg": jnum(pose[2], 1), "kind": "start",
                "leg_i": 0, "leg": None}]
        p = pose
        leg_meta = []
        for i, ((tx, ty), segs) in enumerate(plan_legs, 1):
            leg_name = legs[i - 1][0]
            for seg in segs:
                p = apply_segment(p, seg, measured=False)
                pts.append({"x_mm": jnum(p[0], 1), "y_mm": jnum(p[1], 1),
                            "heading_deg": jnum(p[2], 1),
                            "kind": seg.kind, "dock": bool(seg.dock),
                            "leg_i": i, "leg": leg_name})
            # The last point of a leg is a target the rover holds at. Marked so
            # the map can put a pin there rather than treating it as one more
            # anonymous vertex in a long polyline.
            if segs:
                pts[-1]["waypoint"] = leg_name
            leg_meta.append({"i": i, "name": leg_name,
                             "label": TARGET_LABEL.get(leg_name),
                             "x_mm": jnum(tx, 1), "y_mm": jnum(ty, 1),
                             "final_heading_deg": jnum(legs[i - 1][3], 1),
                             "segments": len(segs),
                             "distance_mm": jnum(remaining_distance_mm(segs), 1)})

        returns_home = name != "home"
        self.bus.publish("telemetry/mission_plan", {
            "mission": name,
            "backend": backend,
            "nominal": True,
            "route": route_description(name),
            "pose_assumed": bool(self.node.anchor.assumed),
            "from": {"x_mm": jnum(pose[0], 1), "y_mm": jnum(pose[1], 1),
                     "heading_deg": jnum(pose[2], 1)},
            # Unchanged meaning for a one-leg mission; for a route it is the
            # LAST target, which is where the outbound line ends.
            "target": {"x_mm": jnum(legs[-1][1], 1), "y_mm": jnum(legs[-1][2], 1),
                       "final_heading_deg": jnum(legs[-1][3], 1)},
            "legs": leg_meta,
            "legs_n": len(legs),
            "segments": [{"kind": s.kind, "target": jnum(s.target, 1),
                          "dock": bool(s.dock)} for s in plan],
            "waypoints": pts,
            "distance_mm": jnum(remaining_distance_mm(plan), 1),
            "eta_s": jnum(route_eta_seconds(plan_legs, returns_home), 1),
            "return_strategy": "retrace" if returns_home else "none",
            "returns_home": returns_home,
        }, qos=1)

        log(f"preview {name!r}: {len(legs)} leg(s), {len(plan)} segments, "
            f"{remaining_distance_mm(plan):.0f}mm outbound, no motion commanded")
        self.bus.publish("events/ack",
                         {"action": "mission", "name": name, "preview": True,
                          "accepted": True, "started": False,
                          "legs_n": len(legs),
                          "segments_planned": len(plan)}, qos=1)

    def _wire_is_ours(self):
        """Nobody else is publishing /cmd_vel.

        Checked by LOOKING BACK, never by sleeping: this runs on paho's network
        thread, and blocking there for a listen window would stall the keepalive
        and delay the `stop` that arrives on the same thread. Since this node
        publishes nothing at all while idle, every /cmd_vel message seen during a
        quiet spell is by definition somebody else's.
        """
        if ALLOW_SHARED_CMDVEL:
            return True
        now = time.monotonic()
        tx = [t for t in self.node._pub_times if now - t <= FOREIGN_LISTEN_S]
        rx = [t for t in self.node._rx_times if now - t <= FOREIGN_LISTEN_S]
        if tx:
            # We published recently (a stop, a previous mission winding down), so
            # a quiet-spell test is not available; fall back to the count test.
            return not self.node.foreign_writer()
        return not rx

    # ------------------------------------------------------------- aborting
    def request_abort(self, reason):
        """Instant, from any state, from any thread.

        The halt latch goes first so that `publish()` starts forcing zeros
        immediately — before the worker has even noticed — and the wire is
        commanded stopped right after. The worker then unwinds and reports.
        """
        first = self.abort_reason is None
        if first:
            self.abort_reason = reason
        self.node.halt.set()
        self.node.stop_wire(5)
        if first:
            log(f"ABORT: {reason}")
        # An abort with nothing running still leaves the rover commanded-stopped
        # and the latch clears on the next accepted mission.

    def check_abort(self, reverse=False, turning=False, nav2_active=False):
        """Polled at every control tick and inside every settle."""
        if self.abort_reason:
            raise MissionAbort(self.abort_reason)
        if self.node.shutdown.is_set():
            raise MissionAbort(ABORT_SHUTDOWN)
        if self.state.elapsed() > MISSION_TIMEOUT_S:
            raise MissionAbort(ABORT_TIMEOUT)
        if not self.node.link_ok():
            raise MissionAbort(ABORT_LINK)
        v = self.node.battery_v()
        if v is not None and v < BATT_LOW_V:
            raise MissionAbort(f"{ABORT_BATT}: {v:.1f}V")
        if not nav2_active:
            clear = self.node.clearance_mm(reverse=reverse, any_bearing=turning)
            limit = ROTATE_CLEAR_MM if turning else FRONT_STOP_MM
            if clear is not None and clear < limit:
                where = "any bearing" if turning else ("rear" if reverse else "front")
                raise MissionAbort(f"{ABORT_OBSTACLE}: {clear:.0f}mm {where} "
                                   f"< {limit:.0f}mm")
            if self.node.foreign_writer():
                raise MissionAbort(ABORT_WIRE + " appeared mid-mission")

    def settle(self, seconds):
        """A stop is not a sleep. Polled so an abort lands inside it."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self.check_abort()
            self.node.publish(0.0, 0.0)
            time.sleep(0.05)

    # --------------------------------------------------------------- worker
    def _run(self, name, backend_name, legs, start_pose):
        """Drive every leg in order, hold at each, then ONE retrace of the lot.

        THE RETRACE IS OF THE WHOLE ROUTE, NOT OF EACH LEG. That is the entire
        reason a multi-leg route is worth having rather than three missions run
        back to back: `executed` accumulates across every leg, so
        `invert_segments` unwinds the complete outbound path in one pass and
        each individual motion is still cancelled by the amount it was MEASURED
        at. Retracing leg by leg would send the rover home between targets and
        drive the arena four times over; re-planning a short hop home from the
        last target instead would be the one thing the module docstring says not
        to do, because a fresh plan carries the accumulated drift home with it
        rather than cancelling it.

        The consequence to be honest about: the return reverses along all three
        legs, so a `patrol` reverses for the full outbound distance. That is
        safe here only because the LiDAR is 360 deg and the guard watches the
        REAR cone while reversing (`front_clearance_mm(..., reverse=True)`), and
        it is why the arena being obstacle-free is a stated precondition and not
        an assumption buried in the geometry.
        """
        st = self.state
        backend = self.backends[backend_name]
        executed = []
        outcome = "completed"
        reason = "done"
        start_yaw = self.node.yaw()
        t0 = time.monotonic()
        final_heading = legs[-1][3]

        try:
            for leg_i, (leg_name, tx, ty, _leg_heading) in enumerate(legs, 1):
                # Fresh convergence budget per leg — see MAX_SEGMENTS.
                self._begin_leg(leg_name, leg_i)
                st.target = (tx, ty)
                st.phase = "outbound"
                if backend_name == "nav2":
                    # Standoff from the LIVE pose, not from start_pose: on leg 2
                    # and beyond the rover is nowhere near where the mission was
                    # accepted, and a standoff measured from the start box would
                    # aim at a point on the wrong side of the target.
                    here = self.node.pose()
                    if here is None:
                        raise MissionAbort(ABORT_LINK)
                    sx, sy = standoff_point(here[0], here[1], tx, ty)
                    approach = bearing_deg(tx - here[0], ty - here[1])
                    backend.goto(sx, sy, approach)
                    # Nav2 got us close; the dock is always ours.
                    st.phase = "docking"
                    executed += self._drive_to(tx, ty, dock_only=True)
                else:
                    executed += self._drive_to(tx, ty)

                # A full stop at every target. On this firmware a stop IS the
                # speed control (see the docstring), so the hold is doing double
                # duty: it is the operator-visible dwell AND the settle that
                # makes the next leg's first measurement trustworthy.
                st.phase = "hold"
                self.settle(HOLD_S)

            if name == "home":
                # `home` IS the return. Retracing it would drive the rover back
                # to wherever it happened to be when the button was pressed,
                # which is the opposite of what the operator asked for.
                self._begin_leg("reface", len(legs) + 1)
                st.phase = "reface"
                executed += self._reface(final_heading)
            else:
                self._begin_leg("home", len(legs) + 1)
                st.phase = "return"
                st.target = (ROVER_START["x_mm"], ROVER_START["y_mm"])
                if backend_name == "nav2":
                    hx, hy, hh = mission_target("home")
                    here = self.node.pose()
                    if here is None:
                        raise MissionAbort(ABORT_LINK)
                    sx, sy = standoff_point(here[0], here[1], hx, hy)
                    backend.goto(sx, sy, hh)
                    executed += self._drive_to(hx, hy, dock_only=True)
                else:
                    # THE RETRACE. Not a fresh plan: the measured outbound
                    # motions, reversed and negated, so their errors cancel.
                    # After a multi-leg route this unwinds EVERY leg in one
                    # pass — see this method's docstring for why that is the
                    # point rather than an accident.
                    retrace = invert_segments(executed)
                    st.segments_n = len(executed) + len(retrace)
                    st.distance_remaining_mm = remaining_distance_mm(retrace)
                    st.eta_s = eta_seconds(retrace)
                    # A retrace is a FIXED list, not an adaptive search, so the
                    # convergence budget is the wrong shape for it: on a long
                    # route the replay legitimately runs more segments than
                    # MAX_SEGMENTS while behaving perfectly. Its own length is
                    # the exact right bound — running one more segment than the
                    # list holds would be a bug, not slow progress.
                    self._begin_leg("home", len(legs) + 1, budget=len(retrace))
                    for seg in retrace:
                        executed.append(self._run_one(backend, seg))
                self._begin_leg("reface", len(legs) + 2)
                st.phase = "reface"
                executed += self._reface(ROVER_START["heading_deg"], start_yaw=start_yaw)

            st.phase = "idle"
        except MissionAbort as e:
            outcome = "aborted"
            reason = e.reason
            st.phase = "aborting"
        except Exception as e:                                # pragma: no cover
            outcome = "aborted"
            reason = f"internal error: {e}"
            st.phase = "aborting"
            log(f"mission worker error {e}")
        finally:
            self.node.stop_wire(5)
            st.driving = False
            st.reversing = False

        self._report(name, backend_name, legs, start_pose, executed,
                     outcome, reason, time.monotonic() - t0, start_yaw)
        st.phase = "idle"
        st.name = None
        st.segment_kind = None
        st.distance_remaining_mm = 0.0
        st.eta_s = 0.0
        st.leg_i = 0
        st.legs_n = 0
        st.leg_name = None

    def _begin_leg(self, leg_name, leg_i, budget=MAX_SEGMENTS):
        """Start a leg: name it, and give it a FRESH segment budget.

        The single place a leg's budget is reset, so a leg cannot be started
        without one. `budget` is per-leg by default (see MAX_SEGMENTS); the
        retrace overrides it because a fixed replay is bounded by its own
        length, not by a convergence allowance.
        """
        st = self.state
        st.leg_name = leg_name
        st.leg_i = leg_i
        st.leg_segment_i = 0
        st.leg_budget = int(budget)

    def _run_one(self, backend, seg):
        st = self.state
        st.segment_i += 1
        st.leg_segment_i += 1
        st.segment_kind = seg.kind
        # PER-LEG first, because it is the guard that carries meaning: it says
        # "this leg is not converging on its target". The route ceiling below is
        # the backstop that keeps the whole thing finite regardless.
        if st.leg_segment_i > st.leg_budget:
            raise MissionAbort(
                f"segment budget exhausted ({st.leg_budget} for leg "
                f"{st.leg_i}/{st.legs_n} {st.leg_name!r}); the rover is not "
                "converging on that target")
        if st.segment_i > MAX_ROUTE_SEGMENTS:
            raise MissionAbort(f"route segment ceiling exhausted "
                               f"({MAX_ROUTE_SEGMENTS} across all legs)")
        out = backend.run_segment(seg)
        if out.kind == "drive":
            st.distance_travelled_mm += abs(out.measured)
        return out

    def _drive_to(self, tx, ty, dock_only=False):
        """Adaptive turn-and-drive: re-measure the bearing after every leg.

        The plan is recomputed from the CURRENT estimated pose each time round,
        so a leg that came out 3 % short or 2 degrees off is corrected by the
        next leg instead of being carried to the target. What is recorded in
        `executed` is the segments actually run, and that is what the retrace
        inverts.
        """
        st = self.state
        backend = self.backends["deadreckon"]   # docking is always local
        executed = []
        while True:
            self.check_abort()
            pose = self.node.pose()
            if pose is None:
                raise MissionAbort(ABORT_LINK)
            dx, dy = tx - pose[0], ty - pose[1]
            dist = math.hypot(dx, dy)
            st.distance_remaining_mm = dist
            if dist <= ARRIVE_TOL_MM:
                return executed
            if dock_only and dist > DOCK_APPROACH_MM * 1.5:
                # Nav2 was supposed to leave us within the dock approach. It did
                # not, so say so rather than silently driving the whole way on a
                # backend the operator did not pick.
                raise MissionAbort(
                    f"nav2 stopped {dist:.0f}mm short of the target, outside the "
                    f"{DOCK_APPROACH_MM:.0f}mm dock approach")

            turn = heading_error_deg(bearing_deg(dx, dy), pose[2])
            if abs(turn) >= BEARING_TOL_DEG:
                executed.append(self._run_one(backend, Segment("turn", turn)))
                continue

            legs = split_legs(dist)
            mm, dock = legs[0]
            st.phase = "docking" if dock else st.phase
            st.segments_n = max(st.segments_n, st.segment_i + len(legs))
            st.eta_s = eta_seconds([Segment("drive", m, dock=d) for m, d in legs])
            executed.append(self._run_one(backend, Segment("drive", mm, dock=dock)))

    def _reface(self, heading_deg, start_yaw=None):
        """Re-face the start heading — a correction, not the plan.

        After a retrace the rover should already be within a couple of degrees:
        every outbound turn has been undone by its own measurement. A large error
        here means the retrace did not do its job, so it is REPORTED rather than
        turned out — spinning the rover to hide a broken heading estimate would
        destroy the evidence and probably point it somewhere worse.
        """
        executed = []
        pose = self.node.pose()
        if pose is None:
            raise MissionAbort(ABORT_LINK)
        if start_yaw is not None:
            # Prefer the gyro delta over the arena heading: it is the same
            # quantity the turns were measured in, so no frame conversion sits
            # between the measurement and the correction.
            err = -wrap180(math.degrees(self.node.yaw() - start_yaw))
        else:
            err = heading_error_deg(heading_deg, pose[2])
        if abs(err) < HEADING_TOL_DEG:
            return executed
        if abs(err) > MAX_REFACE_DEG:
            log(f"re-face SKIPPED: heading error {err:+.1f}deg exceeds "
                f"{MAX_REFACE_DEG:.0f}deg — reporting instead of spinning")
            return executed
        executed.append(self._run_one(self.backends["deadreckon"],
                                      Segment("turn", err)))
        return executed

    # --------------------------------------------------------------- report
    def _report(self, name, backend_name, legs, start_pose, executed,
                outcome, reason, elapsed, start_yaw):
        """events/mission_done — MEASURED outcome, never the requested one."""
        pose = self.node.pose()
        st = self.state
        drives = [s for s in executed if s.kind == "drive" and s.executed]
        turns = [s for s in executed if s.kind == "turn" and s.executed]
        travelled = sum(abs(s.measured) for s in drives)

        # WHAT THE FINAL ERROR IS MEASURED AGAINST. For `home` the promise was
        # "be at the start box", so the reference is ROVER_START. For every other
        # mission the promise was "come back to where you set off", so it is the
        # pose the mission started from — which is the number the retrace is
        # actually being judged on, and it stays meaningful for an aborted run.
        err = None
        if pose:
            ref = ((ROVER_START["x_mm"], ROVER_START["y_mm"]) if name == "home"
                   else (start_pose[0], start_pose[1]))
            err = {"dx_mm": jnum(pose[0] - ref[0], 1),
                   "dy_mm": jnum(pose[1] - ref[1], 1),
                   "dist_mm": jnum(math.hypot(pose[0] - ref[0], pose[1] - ref[1]), 1),
                   "heading_deg": jnum(
                       wrap180(math.degrees(self.node.yaw() - start_yaw)), 1)}

        payload = {
            "action": "mission", "name": name, "backend": backend_name,
            "outcome": outcome, "reason": reason,
            "elapsed_s": jnum(elapsed, 2),
            "distance_travelled_mm": jnum(travelled, 1),
            "segments_executed": len(executed),
            "turns": len(turns), "drives": len(drives),
            "retraced": len([s for s in executed if s.retrace]),
            # The route's FINAL target. Same field, same meaning, for a one-leg
            # mission; `legs` below carries the rest.
            "target": {"x_mm": jnum(legs[-1][1], 1), "y_mm": jnum(legs[-1][2], 1)},
            "route": route_description(name),
            "legs": [{"name": t[0], "label": TARGET_LABEL.get(t[0]),
                      "x_mm": jnum(t[1], 1), "y_mm": jnum(t[2], 1)} for t in legs],
            "legs_n": len(legs),
            # The leg it was WORKING ON when it finished or aborted — not a
            # count of legs completed, because a run that aborts halfway down
            # leg 2 was on leg 2 and had completed one. On an abort this is the
            # difference between "never left the start box" and "stopped at the
            # far corner", which is the first thing an operator needs to know.
            "leg_at_end": max(0, min(st.leg_i, len(legs))),
            "leg_at_end_label": TARGET_LABEL.get(st.leg_name),
            "start_pose": {"x_mm": jnum(start_pose[0], 1),
                           "y_mm": jnum(start_pose[1], 1),
                           "heading_deg": jnum(start_pose[2], 1)},
            "final_pose_error": err,
            "pose_assumed": bool(self.node.anchor.assumed),
            # The raw measurements, so an operator can see WHERE the error came
            # from rather than only that there was one. This is also the record
            # that would let someone re-derive the 0.6 % / +/-1-4 deg figures.
            "measured": [{"kind": s.kind,
                          "asked": jnum(s.target, 1),
                          "measured": jnum(s.measured, 1),
                          "retrace": s.retrace, "dock": s.dock,
                          "reason": s.reason,
                          "elapsed_s": jnum(s.elapsed_s, 2)} for s in executed],
        }
        if pose:
            payload["final_pose"] = {"x_mm": jnum(pose[0], 1),
                                     "y_mm": jnum(pose[1], 1),
                                     "heading_deg": jnum(pose[2], 1)}
        log(f"mission {name!r} {outcome}: {reason}; {travelled:.0f}mm travelled, "
            f"{len(executed)} segments over {len(legs)} leg(s) "
            f"(last: {st.leg_name!r}), {elapsed:.1f}s")
        self.bus.publish("events/mission_done", payload, qos=1)
        self.bus.publish("telemetry/mission", self.node.snapshot())


# ======================================================================= MAIN
def main():
    if not HAVE_ROS:
        raise SystemExit(f"rclpy is required to run this service ({ROS_IMPORT_ERROR}). "
                         "Source /opt/ros/humble/setup.bash and set ROS_DOMAIN_ID=20.")
    if not HAVE_MQTT:
        raise SystemExit(f"paho-mqtt is required to run this service ({MQTT_IMPORT_ERROR}).")

    dom = os.environ.get("ROS_DOMAIN_ID")
    if dom != "20":
        # Not fatal — but on the wrong domain the node sees no topics at all and
        # every mission refuses with "link down", which is a confusing way to
        # discover an environment problem.
        log(f"WARNING: ROS_DOMAIN_ID={dom!r}, expected '20' — this rover's board "
            "publishes on domain 20 only")

    bus = Bus()
    bus.start()

    rclpy.init()
    node = MissionNode(bus)
    runner = MissionRunner(node, bus)
    bus.on_command = runner.handle_command
    bus.on_lidar = node.on_lidar

    def _sig(_s, _f):
        log("signal: stopping any mission and zeroing the wire")
        node.shutdown.set()
        runner.request_abort(ABORT_SHUTDOWN)
        try:
            rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    log(f"fpms_missions up: thing={THING} backend_default={DEFAULT_BACKEND} "
        f"cruise={CRUISE_MPS:.3f}m/s (wire {to_cmd(CRUISE_MPS):.4f}) "
        f"dock_step={DOCK_STEP_MM:.0f}mm")
    try:
        rclpy.spin(node)
    except Exception as e:
        log(f"spin ended: {e}")
    finally:
        # Ten zeros on the way out, for the same reason teleop sends them: the
        # last message the board received must be a stop.
        try:
            node.shutdown.set()
            node.stop_wire(10)
        except Exception:
            pass
        bus.stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
