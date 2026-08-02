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

Nav2 is not the default because it has never driven this rover. `backend:
"nav2"` refuses with a specific reason until its action server is actually up.
That refusal is a feature: it names the missing link instead of driving against
a pose nobody has verified. See NAV2_BRIEF.md section 7.

WHERE THE POSE COMES FROM, AND WHY IT IS NAMED IN EVERY PAYLOAD
===============================================================
"deadreckon" is the driving STRATEGY — turn, drive, stop, measure. It is not a
claim about where the rover thinks it is. Those are separate, and conflating
them is how a route ends up drawn confidently from a guessed origin.

    deadreckon      no absolute fix has ever been adopted; the anchor is the
                    ASSUMED start pose. R2_COORDINATES.md section 6a measures
                    this at +/-40-75 mm per 1 m leg and 50-150 mm absolute, with
                    the two largest terms — assumed start position and heading —
                    unobservable by odometry at any level of tuning.
    slam_anchored   the anchor was last re-established from a `map` -> `odom`
                    transform taken AT REST, and dead reckoning has carried it
                    forward for at most one bounded segment since.

There is no third state and the two are never averaged. Every telemetry payload,
every planned route and every leg of the mission report carries `pose_source`,
because "leg 3 ran on dead reckoning because localisation dropped" is the answer
to "why was this run 8 cm out", and it only exists if it was recorded as it
happened rather than reconstructed afterwards.

THE ARENA HAS NO WALLS — do not reach for the wall fit in fpms_odom_tf.py.
Measured on the live scan: 350 of 360 bins return, minimum 861 mm, median
1983 mm, maximum 6000 mm. There is no 1.2 m box to fit, and a fit with nothing
to fit either refuses or latches onto four unrelated surfaces and reports a
confident wrong pose. Localisation here is scan matching against the real room.

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

The one condition under which that argument stops holding is LOCALISATION. With
a live `map` -> `odom` fix, "where am I" is an observation rather than an
integral, so a fresh plan home carries no accumulated drift and `return:
"planned"` becomes the better choice. It is available and it is NOT the default:
it is refused at accept time when no fix has been adopted, and re-checked at the
moment of use, because localisation can drop during the outbound run. Falling
back to the retrace loses nothing but the shortcut — and it is never silent, the
report says which strategy actually drove.

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
firmware's encoder/wheel constants. Shorten DOCK_STEP_MM — but not below
MIN_MOVE_MM, and read the next section before you try.

...AND THERE IS A FLOOR ON HOW SHORT A BURST CAN BE
===================================================
"Slow is short bursts" has a limit that the first draft of this file walked
straight past. Measured on this rover (WHEELS OFF, free-spinning — see the
caveat below):

    pulse 0.25 s  ->  no motion at all
    pulse 0.35 s  ->  MOVES, about 0.18 per pulse

A velocity setpoint is not raw duty. The firmware's PID has to converge on it,
and below roughly a third of a second it never gets there — the burst ends while
the wheels are still deciding whether to turn. So MIN_PULSE_S is a real physical
floor, and every commanded motion has to be longer than it or it is not a slow
motion, it is no motion:

    MIN_MOVE_MM = CRUISE_MPS * MIN_PULSE_S * 1000   ~= 63 mm at 0.18 m/s
    MIN_TURN_DEG = degrees(TURN_RADPS * MIN_PULSE_S) ~= 9 deg at 0.45 rad/s

DOCK_STEP_MM, MIN_LEG_MM, BEARING_TOL_DEG and HEADING_TOL_DEG are all raised to
those floors at import, loudly, in CFG_NOTES. A 40 mm dock burst and a 4 degree
correction turn were both below the floor and would simply not have moved the
rover — and the stall detector would then have aborted the mission blaming the
firmware dead zone. A correction the chassis cannot express is REPORTED, never
commanded.

**EVERY PULSE NUMBER ABOVE WAS MEASURED WITH THE WHEELS OFF THE GROUND.** Under
load the floor can only be longer, never shorter, so treating these as the floor
is the safe direction to be wrong in — but MIN_PULSE_S must be re-measured on
the floor before any distance figure from this file is trusted.

THE MOTOR LAYOUT IS MIRRORED — EVERY TURN IS BACKWARDS FROM WHAT THIS CODE
ORIGINALLY ASSUMED
==========================================================================
Rewired 2026-08-01 while chasing a dead cable:

    M1 = FRONT-RIGHT (was front-left)    M3 = FRONT-LEFT (was front-right)
    M2 = REAR-RIGHT  (was rear-left)     M4 = REAR-LEFT  (was rear-right)

The firmware still treats M1/M2 as one side and M3/M4 as the other, so FORWARD
is unaffected and every distance constant survives. But the two sides have
swapped places physically, so a commanded +angular.z now rotates the chassis the
other way. That is `TURN_WIRE_SIGN`, and it is applied at exactly ONE place —
`MissionNode.publish`, the single point angular.z reaches the wire — so the turn
primitive and the heading hold inside a drive are corrected together. The prior
working code carried the identical constant for the identical reason
(`TURN_SIGN = -1  # PHASE5: flipped — new chassis turns opposite`, phase6_latest.py).

Two consequences worth stating out loud:

  * The HEADING HOLD is the dangerous half, not the turn. A turn is closed loop
    on the gyro and would at least notice; a heading-hold correction with the
    wrong sign is POSITIVE FEEDBACK, and the rover spirals while every number in
    the log looks reasonable.
  * `TURN_WIRE_SIGN = -1` is DERIVED FROM THE REWIRING, NOT MEASURED. So `_turn`
    carries a wrong-way detector: signed gyro progress that goes backwards past
    TURN_WRONG_WAY_DEG aborts the mission naming this constant. A wrong guess
    therefore costs one 10-degree twitch, not a driven-into-the-wall mission.
    Settle it with one teleop `turn` and set FPMS_MISSION_TURN_WIRE_SIGN.

The board's own odom YAW is mirrored by the same rewiring — which is one more
reason heading here is integrated from /imu (a physical sensor, unaffected) and
never read from an orientation quaternion.

WHICH YAW IS USED FOR WHAT, AND WHY THEY ARE NOT INTERCHANGEABLE
================================================================
There are two yaws and this file used to conflate them, which quietly corrupted
the one measurement the retrace depends on.

    BOARD YAW   pose.pose.orientation on /odom_raw. Same frame as that message's
                pose.position, arbitrary origin, mirrored by the rewiring.
    GYRO YAW    integrated /imu angular_velocity.z. True rotation, arbitrary
                origin, unaffected by wiring.

Projecting an odom-frame displacement (dx, dy) onto the GYRO yaw is a frame
error: the two agree only if the gyro integral happened to be zero at the same
instant and orientation the board's odom frame was zeroed, which nothing
arranges. NAV2_BRIEF.md section 3a prescribes the fix and `fpms_teleop.py`
`summarize_leg` and `fpms_odom_tf.py` `travel_sign()` both already do it:

    along = dx*cos(BOARD_yaw) + dy*sin(BOARD_yaw)      # same message, same frame

So: distance and its SIGN come from the board yaw; heading control, turn
measurement and the arena heading come from the gyro. If the board publishes an
identity orientation this degenerates to `dx`, which is exactly the signed
quantity NAV2_BRIEF's measured table was read from — the identity case is the
measured case, not a fallback.

THE ORIGIN MUST BE RE-ZEROED AFTER EVERY BOOT
=============================================
The origin file teleop writes survives reboots. The board's odometry does not —
it restarts at 0. A reference captured at odom (9.2, 11.9) therefore places a
freshly-booted rover at (-9182, -11926) mm inside a 1200 mm arena, and this node
will plan a confident route from there. That happened.

`origin_is_stale()` refuses the file when it predates the current boot, and the
arena-bounds preflight refuses any mission whose start pose is outside the arena
at all. Both are cheap, both are checked before anything moves, and either one
alone would have caught it.

ARM, AND WHY IT IS A PAYLOAD KEY AND NOT A VERB
===============================================
Motion is refused unless the operator has armed the rover, and any abort, stop,
completion or ARM_TIMEOUT_S of silence disarms it again. Arming lives in the
`mission` payload (`{"arm": true}`) rather than in a new MQTT verb because this
service owns exactly one verb: a second one would have to be mirrored into
fpms_rover_agent's SILENT_VERBS or it would race a bogus "unknown command" nack
onto the dashboard, and that is a change in a file this work does not own.
Previews and probes stay allowed while disarmed — they command nothing.

WHAT THE BOARD ACTUALLY PROVIDES — `{"probe": "encoders"}`
==========================================================
There is no per-wheel encoder topic on this rover. The board publishes an
INTEGRATED pose and a chassis twist on `/odom_raw` and nothing else: no tick
counts, no per-wheel velocities, no duty, no current, no stall flags. The prior
working code read four tick counters straight off a Rosmaster board over serial;
that protocol cannot address this ESP32 board at all, which is why Rosmaster_Lib
returns version -1 and four zeros here. Those zeros mean WRONG PROTOCOL, not
stopped wheels.

So the probe reports what exists and NAMES WHAT DOES NOT, and it refuses to
synthesise the rest. Dividing an integrated pose back into four wheel counts
would produce numbers that look like measurements, agree with each other by
construction, and could never show the one thing per-wheel counts are for — that
one wheel is doing something different from the others. The dead rear-left cable
was found by a yaw-drift symmetry test, not by reading counts, and a fabricated
count would have hidden it.

It also publishes `twist.linear.x` and the pose-derived displacement for the
SAME interval side by side. That pair is the single measurement that settles the
sign inversion, and it costs nothing to carry.

WHAT IN THIS FILE IS MEASURED, AND WHAT IS NOT
==============================================
Nothing below has been measured on the loaded rover, and none of it should be
quoted as if it had. They are advertised as `unverified` in events/online for
exactly that reason.

    TURN_WIRE_SIGN = -1     DERIVED from the rewiring, never observed. Backed by
                            a wrong-way abort so a wrong value costs one twitch.
    MIN_PULSE_S = 0.35      Measured WHEELS OFF. Under load it can only be
                            longer, so it is the safe direction to be wrong in —
                            but MIN_MOVE_MM, MIN_TURN_DEG, DOCK_STEP_MM,
                            BEARING_TOL_DEG and HEADING_TOL_DEG all derive from
                            it, so re-measuring it moves the whole envelope.
    CMD_SCALE = 6.1         Measured once, free-spinning, on the linear axis
                            only. Applying it to rotation is an assumption that
                            can only turn the rover slower than asked.
    LiDAR mount calibration UNVERIFIED (fpms_lidar_ros.py:164-168). A wrong
                            rotation sign mirrors every scan, so the map is built
                            mirrored, matches itself perfectly, and localises the
                            rover confidently into a reflected room.

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

# tf2 is optional for the same reason Nav2 is: the LOCALISATION FIX this node
# consumes (`map`->`odom`, the REP-105 edge an absolute correction is allowed to
# move) is published by whatever is localising — slam_toolbox or AMCL — and until
# that bringup lands, nothing publishes it. Missing tf2, or a missing transform,
# must degrade this node to dead reckoning with a stated reason, never prevent it
# starting.
#
# NOTE the landmine in NAV2_BRIEF.md section 6b: `tf_transformations` is
# installed and BROKEN (a user-local NumPy 2.x shadows the system one). It is
# deliberately not imported; quaternion->yaw is two lines of `yaw_from_quat`.
try:
    from tf2_ros import Buffer, TransformListener
    from rclpy.time import Time as RclTime
    HAVE_TF2 = True
    TF2_IMPORT_ERROR = ""
except Exception as _e:                                     # pragma: no cover
    HAVE_TF2 = False
    TF2_IMPORT_ERROR = str(_e)


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


def _cfg_sign(key, default):
    """config.env override for a +1/-1 chassis polarity constant.

    Its own reader rather than `_cfg_float` with a [-1, 1] range, because 0 and
    0.5 are both inside that range and neither is a direction. A polarity that
    silently became 0 would stop every turn dead and look like a dead motor.
    """
    raw = CFG.get(key)
    if raw is None:
        return default
    s = str(raw).strip().lower()
    if s in ("1", "+1", "ccw", "normal"):
        return 1
    if s in ("-1", "cw", "mirrored", "flipped"):
        return -1
    CFG_NOTES.append(f"{key}={raw!r} is not +1 or -1; using {default:+d}")
    return default


def _boot_time():
    """Wall-clock seconds at which this machine booted, or None off-Linux.

    Used only to decide whether a file written before the last boot can still
    be believed. `/proc/uptime` rather than psutil: no dependency, and it is the
    same number `uptime` prints.
    """
    try:
        with open("/proc/uptime") as fh:
            up = float(fh.read().split()[0])
        return time.time() - up
    except Exception:
        return None


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


# ============================================================ CHASSIS POLARITY
# THE MOTOR LAYOUT IS MIRRORED. See the docstring section of the same name.
# M1/M2 are now the RIGHT side and M3/M4 the LEFT, so the firmware's idea of
# "side A" is the opposite physical side to the one this code was written
# against, and a commanded +angular.z rotates the chassis clockwise where it used
# to rotate it counter-clockwise.
#
# Applied at ONE place — MissionNode.publish — so the turn primitive and the
# heading hold inside a drive can never disagree about which way is positive.
# The prior working code carried the same constant for the same reason:
# phase6_latest.py `TURN_SIGN = -1  # PHASE5: flipped — new chassis turns opposite`.
#
# -1 is DERIVED FROM THE REWIRING AND HAS NOT BEEN MEASURED. `_turn` therefore
# aborts on a wrong-way rotation instead of trusting it, so a wrong value costs
# one small twitch rather than a mission.
TURN_WIRE_SIGN = _cfg_sign("FPMS_MISSION_TURN_WIRE_SIGN", -1)
TURN_WIRE_SIGN_MEASURED = False     # flip this ONLY after a real turn confirms it

# How far a turn may rotate the WRONG way before it is called wrong rather than
# noisy. Comfortably above gyro noise over a couple of seconds and well below any
# turn the planner emits, so it can only fire on a genuine polarity error.
TURN_WRONG_WAY_DEG = _cfg_float("FPMS_MISSION_TURN_WRONG_WAY_DEG", 8.0, 3.0, 45.0)


# ================================================= THE MINIMUM COMMANDED MOTION
# Measured WHEELS OFF: a 0.25 s pulse produced no motion at all and a 0.35 s
# pulse moved. The firmware is converging a PID onto a velocity setpoint, so a
# burst shorter than this ends before the wheels have been persuaded to turn.
# Under load the floor can only be LONGER, so using the free-spinning number as
# the floor is the safe direction to be wrong in — but it is not a measurement of
# the loaded rover and must not be quoted as one.
MIN_PULSE_S = _cfg_float("FPMS_MISSION_MIN_PULSE_S", 0.35, 0.10, 1.50)
MIN_PULSE_MEASURED_UNDER_LOAD = False

# The shortest motion worth commanding, derived rather than typed so that
# re-measuring MIN_PULSE_S or changing the cruise speed moves both floors.
MIN_MOVE_MM = CRUISE_MPS * MIN_PULSE_S * 1000.0
MIN_TURN_DEG = math.degrees(TURN_RADPS * MIN_PULSE_S)


def _floor_note(name, value, floor, unit, why):
    """Raise a tolerance to a physical floor, and SAY SO. Never silently."""
    if value >= floor:
        return value
    CFG_NOTES.append(f"{name} {value:g}{unit} is below the {why} floor "
                     f"{floor:.1f}{unit}; raised to it — the chassis cannot "
                     f"express a smaller motion")
    return floor


# ======================================================== ARM / MOTION CONSENT
# Nothing moves until an operator arms it, and it disarms itself again on any
# abort, stop, completion, or after ARM_TIMEOUT_S of not being used. Set
# FPMS_MISSION_REQUIRE_ARM=0 only on a bench where the wheels are off the floor.
REQUIRE_ARM = CFG.get("FPMS_MISSION_REQUIRE_ARM", "1") not in ("0", "false", "no")
ARM_TIMEOUT_S = _cfg_float("FPMS_MISSION_ARM_TIMEOUT_S", 120.0, 10.0, 3600.0)


# ================================================== SEGMENTATION / TOLERANCES
MAX_LEG_MM = _cfg_float("FPMS_MISSION_MAX_LEG_MM", 300.0, 50.0, 600.0)
# Shorter than this is stop-settle noise, so it is folded into the previous
# segment instead of commanded — and it can never be shorter than the shortest
# burst the firmware will act on at all.
MIN_LEG_MM = _floor_note("MIN_LEG_MM", 30.0, MIN_MOVE_MM, "mm", "minimum-pulse")
DOCK_APPROACH_MM = _cfg_float("FPMS_MISSION_DOCK_APPROACH_MM", 150.0, 40.0, 400.0)
# 40 mm at 0.18 m/s is a 0.22 s burst — below the 0.35 s that moves anything, so
# the whole dock would have stood still and then aborted as a stall. The dock is
# slow because of the STOP between bursts, and a burst too short to move is not a
# slower dock, it is no dock.
DOCK_STEP_MM = _floor_note(
    "DOCK_STEP_MM", _cfg_float("FPMS_MISSION_DOCK_STEP_MM", 70.0, 15.0, 150.0),
    MIN_MOVE_MM, "mm", "minimum-pulse")

ARRIVE_TOL_MM = 25.0     # closer than this and another leg is noise, not progress
# A correction smaller than the chassis can express must be REPORTED, not
# commanded: commanding it produces a burst too short to move, which the stall
# detector then correctly aborts the mission over.
#
# SAY THE CONSEQUENCE OUT LOUD, because it bounds what this rover can do on dead
# reckoning alone: a 9 degree bearing tolerance over a 300 mm segment is up to
# 47 mm of lateral error that the follower will not try to correct, because it
# CANNOT. That is not a tuning choice — it is MIN_PULSE_S times TURN_RADPS. The
# two ways to shrink it are a shorter minimum pulse (measure it under load; it
# may be worse, not better) and localisation, which corrects position directly
# instead of trying to steer the error away.
BEARING_TOL_DEG = _floor_note("BEARING_TOL_DEG", 4.0, MIN_TURN_DEG, "deg",
                              "minimum-pulse")
HEADING_TOL_DEG = _floor_note("HEADING_TOL_DEG", 3.0, MIN_TURN_DEG, "deg",
                              "minimum-pulse")
MAX_REFACE_DEG = 30.0    # a bigger error means the retrace failed; report it,
                         # do not spin the rover round trying to hide it

# GOLDEN TECHNIQUE (phase6_latest.py `_p5_navdrive`, the HOME correction block):
# after the retrace has put the heading back, close any residual positional gap
# with straight nudges and NO turn. A turn here would spend the heading the
# retrace just recovered in order to fix a few millimetres of position, which is
# the wrong trade — so the trim drives forwards or backwards along the heading it
# already has, at most HOME_TRIM_TRIES times, and gives up quietly.
HOME_TRIM_TRIES = 3
HOME_TRIM_MAX_MM = 200.0
HOME_TRIM_TOL_MM = ARRIVE_TOL_MM

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

# ============================================ LOCALISATION (`map` -> `odom`)
# THE ROVER HAS NO ARENA WALLS. Measured on the live scan: 350 of 360 bins
# return, minimum 861 mm, median 1983 mm, maximum 6000 mm. There is no 1.2 m box
# out there. The wall-fit localiser in fpms_odom_tf.py (`arena_fix_from_scan`)
# is mathematically fine and physically inapplicable here, and MUST NOT be
# enabled — a fit that has no walls to fit will either refuse or, worse, latch
# onto four unrelated surfaces and report a confident wrong pose.
#
# The room itself is feature-rich, and slam_toolbox, nav2_amcl, nav2_map_server
# and robot_localization are all already installed. So localisation comes from
# SCAN MATCHING AGAINST THE REAL ROOM, built in rover/nav2 and rover/slam by
# another agent, and it reaches this node the only way an absolute correction is
# allowed to: as the `map` -> `odom` transform.
#
# THIS NODE CONSUMES THAT EDGE AND COMPUTES NOTHING. Under REP-105 `odom` ->
# `base_footprint` must stay continuous and keep drifting, and every absolute
# jump lands on `map` -> `odom` instead. Whether slam_toolbox, AMCL or anything
# else is publishing it is not this file's business — which is exactly why the
# consumer is four lines of arithmetic (`pose_from_map_to_odom`) and why it keeps
# working if the producer changes.
#
# WHAT THE POSE ACTUALLY IS, AT ANY MOMENT — AND IT IS NEVER A SILENT BLEND
# ------------------------------------------------------------------------
#     "deadreckon"      no fix has ever been adopted. Anchor is the ASSUMED start
#                       pose. R2_COORDINATES.md section 6a: +/-40-75 mm per 1 m
#                       leg, 50-150 mm absolute, and unbounded over a route.
#     "slam_anchored"   the anchor was last re-established from a `map` -> `odom`
#                       fix taken AT REST, and dead reckoning has carried it
#                       forward for at most one bounded segment since.
#
# Those are the only two, they are named in every telemetry payload and in every
# leg of the mission report, and they are never averaged together. When the
# operator asks why a run was 8 cm out, "leg 3 ran on dead reckoning because
# localisation dropped" is the answer that matters, and it is only available if
# the follower recorded it at the time.
#
# WHY THE FIX IS TAKEN AT REST, AND WHY THE ANCHOR MOVES RATHER THAN THE POSE
# ---------------------------------------------------------------------------
# Scans reach this Pi over MQTT at ~9.5 Hz, so at least 16 mm of travel separates
# one from the next and a match computed while moving is stale by more than the
# error it is meant to remove. The turn-then-drive rhythm already stops between
# segments, so stop-and-fix costs nothing — it is the same settle.
#
# The fix moves the ANCHOR, not the live pose. Blending a correction into the
# pose would leave this node with two sources of truth that disagree mid-leg, and
# a `_drive` measuring its own displacement would read the correction as motion it
# had performed. Re-anchoring keeps exactly one pipeline and puts the jump at a
# standstill where nothing is measuring against it.
ARENA_FIX_ENABLED = CFG.get("FPMS_MISSION_ARENA_FIX", "1") not in ("0", "false", "no")
# Whether the LiDAR mount calibration behind every fix has been measured.
# LIDAR_ZERO_OFFSET_DEG and LIDAR_ROTATION_SIGN (fpms_lidar_ros.py:164-168) are
# UNVERIFIED, and this matters MORE with SLAM than it did with a wall fit: a
# wrong rotation sign mirrors every scan, so the map is built mirrored, matches
# itself perfectly, and localises the rover confidently into a reflected room.
# Until it is measured, a fix buys precision without proving accuracy, and every
# payload that carries one carries this flag beside it.
ARENA_FIX_CALIBRATION_VERIFIED = False
ARENA_FIX_MAP_FRAME = CFG.get("FPMS_MISSION_MAP_FRAME", "map")
ARENA_FIX_ODOM_FRAME = CFG.get("FPMS_MISSION_ODOM_FRAME", "odom")
FIX_MAX_AGE_S = 3.0            # older than this and it is not "where I am now"
FIX_STILL_S = 0.5              # wheels must have been commanded zero this long
# Guards on ADOPTING a fix. A scan match that fails does not report failure — it
# reports a pose. At a standstill, a correction this large is far more likely to
# be a bad match or a loop closure landing mid-route than a real discovery, and
# adopting it would rotate or translate every remaining leg.
FIX_MAX_HEADING_DISAGREE_DEG = _cfg_float(
    "FPMS_MISSION_FIX_MAX_HEADING_DEG", 45.0, 5.0, 180.0)
FIX_MAX_JUMP_MM = _cfg_float("FPMS_MISSION_FIX_MAX_JUMP_MM", 400.0, 50.0, 2000.0)

POSE_DEADRECKON = "deadreckon"
POSE_SLAM = "slam_anchored"

# HOW THE ROVER COMES HOME.
#   retrace  replay the MEASURED outbound motions in reverse. Proven on this
#            rover, needs no map, and cancels drift by construction. THE DEFAULT,
#            and the only strategy that works with no localisation at all.
#   planned  plan a fresh route home from the localised pose. Shorter, and better
#            WHEN AND ONLY WHEN localisation is live — a fresh plan carries every
#            millimetre of accumulated dead-reckoning drift home with it, so
#            without a fix it is strictly worse than the retrace.
# Never silently chosen: `planned` is refused at accept time if localisation is
# not actually live, rather than falling back and quietly driving the other one.
RETURN_STRATEGIES = ("retrace", "planned")
DEFAULT_RETURN = CFG.get("FPMS_MISSION_RETURN", "retrace")
if DEFAULT_RETURN not in RETURN_STRATEGIES:
    CFG_NOTES.append(f"FPMS_MISSION_RETURN={DEFAULT_RETURN!r} unknown; using retrace")
    DEFAULT_RETURN = "retrace"

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


def _rate_hz(times, now, window=5.0):
    """Publish rate of a topic from its arrival times, or None.

    Over a window rather than from the last two samples: a single late message
    on a WiFi link that swings -47 to -78 dBm would otherwise report a topic as
    dead when it is merely jittering.
    """
    try:
        recent = [t for t in times if now - t <= window]
        if len(recent) < 2:
            return None
        span = recent[-1] - recent[0]
        if span <= 0:
            return None
        return (len(recent) - 1) / span
    except Exception:
        return None


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

    Returns [(mm, is_dock), ...] summing to dist_mm — or an EMPTY LIST when the
    whole remaining distance is under MIN_MOVE_MM. Empty means "this chassis
    cannot express a motion that short", which is a real answer and the caller
    treats it as arrival with a reported residual. Emitting the segment anyway
    would command a burst too short to move, and the stall detector would then
    abort the mission over a firmware limit nobody violated.
    """
    out = []
    d = float(dist_mm)
    if not math.isfinite(d) or d < MIN_MOVE_MM:
        return out

    dock_total = min(DOCK_APPROACH_MM, d)
    cruise = d - dock_total
    # A cruise remainder too short to command belongs to the dock, not to a
    # segment that would stand still.
    if 0.0 < cruise < MIN_MOVE_MM:
        dock_total += cruise
        cruise = 0.0

    while cruise > 1e-6:
        leg = min(MAX_LEG_MM, cruise)
        # Never leave a stub behind: a leg under MIN_LEG_MM is shorter than the
        # settle noise it would be measured against, so absorb it here.
        if 0.0 < cruise - leg < MIN_LEG_MM:
            leg = cruise
        out.append((leg, False))
        cruise -= leg

    if dock_total > 1e-6:
        # Two bounds, and the SMALLER wins: at most DOCK_STEP_MM per burst so the
        # approach is slow, but never so many bursts that each one drops under
        # MIN_MOVE_MM and stops moving. 150 mm of approach in 70 mm steps is 3
        # bursts of 50 mm by the first rule alone — every one of them too short
        # to move.
        by_step = int(math.ceil(dock_total / DOCK_STEP_MM))
        by_floor = int(math.floor(dock_total / MIN_MOVE_MM))
        n = max(1, min(by_step, max(1, by_floor)))
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

    ONE THING THE THREE LINES CANNOT DO ON THEIR OWN: a motion that came out
    below MIN_MOVE_MM / MIN_TURN_DEG cannot be replayed as its own burst, because
    a burst that short does not move this chassis at all (see the docstring). It
    is not simply dropped — dropping it silently would leave exactly that much
    error uncancelled, which is the one thing a retrace exists to prevent.

    So it is carried into the NEXT ADJACENT motion of the same kind, and ONLY
    while nothing of the other kind intervenes. That restriction is the whole
    correctness argument and it is easy to get wrong: two drives with no turn
    between them share a heading and genuinely sum, but two drives SEPARATED BY A
    TURN point in different directions, and adding them would send the rover off
    along the wrong one. The moment a segment of the other kind appears, any
    pending carry becomes uncancellable and is handed to the residual instead.

    `retrace_residual` reports whatever could not be given back, rather than
    leaving a few millimetres of unexplained error looking like drift.
    """
    return _retrace_walk(segs)[0]


def retrace_residual(segs):
    """What `invert_segments` could not give back, per kind.

    Non-zero here means the rover will stop that far short of where it set off,
    for a reason that is physical rather than a fault: the chassis has no motion
    that small, or the small motion sat between two turns and could not be merged
    into either. Reported in events/mission_done so a millimetre of unexplained
    error never has to be guessed at.
    """
    return _retrace_walk(segs)[1]


def _retrace_walk(segs):
    """(retrace segments, residual per kind). One walk, so the replay and the
    report of what it could not replay can never disagree."""
    out = []
    residual = {"turn_deg": 0.0, "drive_mm": 0.0}
    key = {"turn": "turn_deg", "drive": "drive_mm"}
    carry_kind = None
    carry = 0.0

    def flush():
        nonlocal carry_kind, carry
        if carry_kind is not None and abs(carry) > 1e-9:
            residual[key[carry_kind]] += carry
        carry_kind, carry = None, 0.0

    for s in reversed(segs):
        if not s.executed or abs(s.measured) < 1e-9:
            continue
        if carry_kind is not None and carry_kind != s.kind:
            # A segment of the other kind has intervened, so the pending motion
            # can no longer be merged into anything: after a turn, the next drive
            # points somewhere else entirely.
            flush()
        carry_kind = s.kind
        carry -= s.measured
        if min_pulse_ok(s.kind, carry):
            out.append(Segment(s.kind, carry, dock=s.dock, retrace=True))
            carry_kind, carry = None, 0.0
    flush()
    return out, residual


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


def turn_settle_s(deg):
    """How long to let a turn coast before MEASURING it.

    GOLDEN TECHNIQUE (phase6_latest.py `_p5_navdrive._spin`): the coast after a
    big spin is longer than the coast after a small one, and that code waited an
    extra 0.6 s beyond its normal settle whenever a turn exceeded 45 degrees.
    Measuring during the coast is measuring a moving rover, and the number it
    produces is the number the retrace replays — so a settle that is too short
    does not make the mission quicker, it makes the way home wrong.
    """
    return TURN_SETTLE_S + (0.6 if abs(float(deg)) > 45.0 else 0.0)


def min_pulse_ok(kind, target):
    """Is this motion long enough for the firmware to act on it at all?

    See the docstring: a burst under MIN_PULSE_S ends before the velocity loop
    has converged, so it moves nothing. Planning is supposed to guarantee this,
    and this is the assertion that says so out loud at the one place a Segment
    becomes a command.
    """
    if kind == "turn":
        return abs(float(target)) >= MIN_TURN_DEG - 1e-9
    return abs(float(target)) >= MIN_MOVE_MM - 1e-9


def origin_is_stale(origin_mtime, boot_time, now=None):
    """Was teleop's origin file written BEFORE the machine last booted?

    The file survives a reboot; the board's odometry does not — it restarts at 0.
    So a reference captured at odom (9.2, 11.9) m puts a freshly-booted rover at
    (-9182, -11926) mm inside a 1200 mm arena, and this node will plan a
    confident route from there. That is not hypothetical; it happened.

    Pure arithmetic on two timestamps so it can be tested off-robot. `None` for
    either input means "cannot tell", and the honest answer to that is False —
    the arena-bounds preflight is the backstop that does not need a clock.
    """
    if origin_mtime is None or boot_time is None:
        return False
    return float(origin_mtime) < float(boot_time)


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
    # BOARD yaw at the moment the anchor was taken — the same frame as odom_x/y,
    # so it is the right angle to rotate an odom-frame displacement by. NOT the
    # gyro; see the docstring section on the two yaws.
    odom_yaw_deg: float = 0.0
    # GYRO integral at the moment the anchor was taken. Heading is measured as a
    # DELTA from this, so it never inherits the gyro's arbitrary zero.
    gyro_yaw_deg: float = 0.0
    arena_x_mm: float = ROVER_START["x_mm"]
    arena_y_mm: float = ROVER_START["y_mm"]
    arena_heading_deg: float = ROVER_START["heading_deg"]
    assumed: bool = True
    # False until odometry has actually been seen. An anchor taken before the
    # first /odom_raw message has both yaw references at 0 by default rather than
    # by measurement, and would silently offset every route by whatever the rover
    # had already turned through.
    established: bool = False
    source: str = "assumed start pose"


def arena_from_odom(anchor, odom_x, odom_y, odom_yaw_deg, gyro_yaw_deg=None):
    """odom metres -> arena millimetres + heading degrees.

    POSITION is rotated by the BOARD yaw the anchor was taken at, because
    odom_x/odom_y live in the board's frame and only the board's own yaw
    describes that frame's orientation.

    HEADING comes from the GYRO delta when one is supplied — the gyro is a
    physical sensor and is unaffected by the mirrored motor layout, while the
    board's dead-reckoned yaw is mirrored by it. `gyro_yaw_deg=None` keeps the
    old board-yaw-only behaviour so the geometry stays testable on its own.
    """
    rot = math.radians(anchor.arena_heading_deg - anchor.odom_yaw_deg)
    dx = (odom_x - anchor.odom_x) * 1000.0
    dy = (odom_y - anchor.odom_y) * 1000.0
    c, s = math.cos(rot), math.sin(rot)
    if gyro_yaw_deg is None:
        heading = wrap180(odom_yaw_deg + math.degrees(rot))
    else:
        heading = wrap180(anchor.arena_heading_deg
                          + (gyro_yaw_deg - anchor.gyro_yaw_deg))
    return (anchor.arena_x_mm + dx * c - dy * s,
            anchor.arena_y_mm + dx * s + dy * c,
            heading)


def arena_to_map_m(x_mm, y_mm):
    """Arena millimetres -> Nav2 map frame metres. One place, one assumption."""
    return (MAP_ORIGIN_X_M + x_mm / 1000.0, MAP_ORIGIN_Y_M + y_mm / 1000.0)


def map_m_to_arena_mm(x_m, y_m):
    """Nav2 map frame metres -> arena millimetres. The exact inverse, here so
    that the fix consumer and the Nav2 goal builder can never disagree about
    where the map frame's origin is."""
    return ((x_m - MAP_ORIGIN_X_M) * 1000.0, (y_m - MAP_ORIGIN_Y_M) * 1000.0)


def pose_from_map_to_odom(mx_m, my_m, myaw_rad, odom_x_m, odom_y_m, odom_yaw_rad):
    """Compose `map`->`odom` with the live odom pose into an arena pose.

    The exact inverse of `fpms_odom_tf.map_to_odom_from_fix`, which builds the
    transform as T_map_odom = T_map_base * inverse(T_odom_base). Composing it
    back the other way returns the fixed pose:

        T_map_base = T_map_odom * T_odom_base

    Pure arithmetic, deliberately: it is the one piece of the fix path that can
    be checked without a robot, a LiDAR or a running tf tree.
    Returns (x_mm, y_mm, heading_deg) in the ARENA frame.
    """
    c, s = math.cos(myaw_rad), math.sin(myaw_rad)
    bx = mx_m + c * odom_x_m - s * odom_y_m
    by = my_m + s * odom_x_m + c * odom_y_m
    x_mm, y_mm = map_m_to_arena_mm(bx, by)
    return (x_mm, y_mm, wrap180(math.degrees(odom_yaw_rad + myaw_rad)))


def fix_rejects(fix_pose, dr_pose, max_heading_deg=FIX_MAX_HEADING_DISAGREE_DEG,
                max_jump_mm=FIX_MAX_JUMP_MM):
    """Why an arena fix must NOT be adopted. A list of strings, empty to accept.

    A list rather than a bool, because a correction that quietly declines to fire
    leaves whoever is debugging it with nothing but silence. All three refusals
    are real failure modes of scan matching, not hypotheticals:

      * A FAILED SCAN MATCH DOES NOT REPORT FAILURE — it reports a pose. In a
        room with repeated structure the match can land on the wrong feature, and
        at a standstill a large heading disagreement is far more likely to be
        that than a genuine discovery. Adopting it would rotate every remaining
        leg of the route.
      * A fix outside the arena is not a better answer about where the rover is;
        it is evidence that something upstream is wrong. The likeliest cause is a
        mirrored scan from an unverified LIDAR_ROTATION_SIGN, which builds a
        mirrored map that then matches itself perfectly.
      * A large translation jump at rest is a loop closure landing mid-route.
        Legitimate for the map; not something to re-anchor a running mission on
        without saying so.

    All three limits are config-overridable, because "too large to believe"
    depends on the room and none of these numbers has been measured in it.
    """
    out = []
    fx, fy, fh = fix_pose
    dx_, dy_, dh = dr_pose
    if not (math.isfinite(fx) and math.isfinite(fy) and math.isfinite(fh)):
        out.append("fix contains a non-finite value")
        return out
    if not in_arena(fx, fy):
        out.append(f"fix ({fx:.0f}, {fy:.0f}) mm is outside the "
                   f"{ARENA_MM:.0f} mm arena")
    dh_err = abs(wrap180(fh - dh))
    if dh_err > max_heading_deg:
        out.append(f"fix heading {fh:+.1f}deg is {dh_err:.1f}deg from dead "
                   f"reckoning (max {max_heading_deg:.0f}) — at a standstill "
                   "that is a failed scan match reporting a pose, not a "
                   "discovery")
    jump = math.hypot(fx - dx_, fy - dy_)
    if jump > max_jump_mm:
        out.append(f"fix is {jump:.0f}mm from dead reckoning "
                   f"(max {max_jump_mm:.0f})")
    return out


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
ABORT_TURN_SIGN = "turn went the WRONG WAY"
ABORT_DISARMED = "rover is not armed"


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
                          "return_strategies": list(RETURN_STRATEGIES),
                          "default_return": DEFAULT_RETURN,
                          # PAYLOAD MODES, not extra verbs. This service owns
                          # exactly one verb; a second would have to be mirrored
                          # into fpms_rover_agent's SILENT_VERBS or it would race
                          # an "unknown command" nack over the real reply. A
                          # dashboard builds its buttons from these.
                          "payload_modes": {
                              "run": {"name": list(COMMANDABLE),
                                      "backend": list(BACKENDS),
                                      "return": list(RETURN_STRATEGIES)},
                              "preview": {"preview": True,
                                          "moves": False},
                              "arm": {"arm": True, "moves": False,
                                      "required_before_motion": REQUIRE_ARM,
                                      "timeout_s": ARM_TIMEOUT_S,
                                      "auto_disarms_on": ["stop", "estop",
                                                          "auto_off", "abort",
                                                          "mission end",
                                                          "timeout"]},
                              "probe": {"probe": ["encoders", "odom", "all"],
                                        "moves": False,
                                        "replies_on": "events/encoder_probe"},
                          },
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
                                     "batt_low_v": BATT_LOW_V,
                                     # The physical floors. A consumer that
                                     # offers a "nudge 20 mm" control needs to
                                     # know the chassis has no such motion.
                                     "min_pulse_s": MIN_PULSE_S,
                                     "min_move_mm": jnum(MIN_MOVE_MM, 1),
                                     "min_turn_deg": jnum(MIN_TURN_DEG, 1),
                                     "turn_wire_sign": TURN_WIRE_SIGN},
                          # UNMEASURED CONSTANTS, ADVERTISED AS SUCH. Every one
                          # of these is derived or measured free-spinning, and a
                          # consumer that quotes an accuracy figure without them
                          # is quoting a guess.
                          "unverified": {
                              "turn_wire_sign_measured": TURN_WIRE_SIGN_MEASURED,
                              "min_pulse_measured_under_load":
                                  MIN_PULSE_MEASURED_UNDER_LOAD,
                              "lidar_mount_calibration_verified":
                                  ARENA_FIX_CALIBRATION_VERIFIED,
                          },
                          "config_notes": list(CFG_NOTES)}, qos=1)
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
        # The BOARD's own yaw, from the same message as odom_x/odom_y. Distance
        # is projected onto THIS, never onto the gyro — they are different frames
        # with different zeros, and the file used to conflate them. See the
        # docstring section "WHICH YAW IS USED FOR WHAT".
        self.odom_yaw = 0.0
        self.odom_yaw_identity = None  # None until the first message answers it
        self.odom_last = 0.0
        self.odom_source = None       # "/odom" | "/odom_raw"
        self.odom_fused_last = 0.0
        # Arrival times and the last raw twist, kept ONLY so the encoder probe
        # can report what the board really publishes without commanding anything.
        # No guard, tolerance or progress check anywhere in this file reads
        # twist — it is sign-inverted relative to its own pose.
        self._odom_times = deque(maxlen=64)
        self._imu_times = deque(maxlen=128)
        self._probe_twist_x = None
        self._probe_pose_prev = None   # (t, x, y, board_yaw)
        self._probe_along_mm = None

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

        # The arena fix, consumed as `map`->`odom` and nothing else. Absent
        # today (fpms_odom_tf.py's LIDAR_FIX_ENABLED is False), which is a
        # degradation to dead reckoning with a stated reason, not a failure.
        self.tf_buffer = None
        self.tf_listener = None
        self.fix_last = None          # the last ADOPTED fix, for telemetry
        self.fix_rejects_last = []
        self.fix_adopted_n = 0
        self.fix_at = None            # monotonic time of the last adopted fix
        self.still_since = time.monotonic()
        if ARENA_FIX_ENABLED and HAVE_TF2:
            try:
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
                log(f"localisation: listening for {ARENA_FIX_MAP_FRAME} -> "
                    f"{ARENA_FIX_ODOM_FRAME} (slam_toolbox or AMCL); fixes are "
                    "taken AT REST only, and are UNVERIFIED until the LiDAR "
                    "mount yaw and rotation sign are measured "
                    "(fpms_lidar_ros.py:164-168) — a mirrored scan builds a "
                    "mirrored map that matches itself perfectly")
            except Exception as e:
                log(f"localisation: tf2 setup failed ({e}); dead reckoning only")
                self.tf_buffer = None
        elif ARENA_FIX_ENABLED:
            log(f"localisation: tf2_ros not importable ({TF2_IMPORT_ERROR}); "
                "dead reckoning only")

        # /cmd_vel traffic accounting for the foreign-writer detector.
        self._pub_times = deque(maxlen=200)
        self._rx_times = deque(maxlen=400)

        self.halt = threading.Event()     # set => publish() forces zero
        self.shutdown = threading.Event()
        self.state = MissionState()
        # Set by main() once the runner exists. Read only for telemetry, so a
        # None here degrades a status field rather than anything that moves.
        self.runner = None

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
            q = msg.pose.pose.orientation
            byaw = yaw_from_quat(q)
            if not math.isfinite(byaw):
                byaw = 0.0
            # NAV2_BRIEF section 5 records an open contradiction between two files
            # about whether this orientation is a real encoder yaw or the identity
            # quaternion. Rather than pick a side, observe it — and note that the
            # identity case is not a degradation: it makes `along` collapse to dx,
            # which is exactly the signed quantity NAV2_BRIEF's measured table was
            # read from.
            identity = (abs(float(q.z)) < 1e-9 and abs(float(q.x)) < 1e-9
                        and abs(float(q.y)) < 1e-9)
            now = time.monotonic()
            with self.lock:
                self.odom_yaw = byaw
                if self.odom_yaw_identity is None:
                    self.odom_yaw_identity = identity
                    what = ("IDENTITY, so `along` collapses to dx — which is the "
                            "measured case, not a fallback"
                            if identity else "a real yaw")
                    log(f"odom orientation on {source} is {what}")
                elif self.odom_yaw_identity and not identity:
                    self.odom_yaw_identity = False
                    log(f"odom orientation on {source} started publishing a real "
                        "yaw; distance projection now uses it")
                self._odom_times.append(now)
                prev = self._probe_pose_prev
                if prev is not None and now - prev[0] > 0.0:
                    dx = (x - prev[1]) * 1000.0
                    dy = (y - prev[2]) * 1000.0
                    self._probe_along_mm = (dx * math.cos(prev[3])
                                            + dy * math.sin(prev[3]))
                self._probe_pose_prev = (now, x, y, byaw)
                try:
                    self._probe_twist_x = float(msg.twist.twist.linear.x)
                except Exception:
                    self._probe_twist_x = None
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
                # The anchor's two yaw references are only meaningful once there
                # is odometry to take them from. Establishing it at construction
                # would leave both at 0.0 by DEFAULT rather than by measurement,
                # and every route would then be silently rotated by whatever the
                # rover had already turned through since the board booted.
                if not self.anchor.established:
                    self._establish_anchor(x, y, byaw)
        except Exception as e:
            log(f"odom callback error {e}")

    def _establish_anchor(self, odom_x, odom_y, board_yaw_rad):
        """Fill in the anchor's yaw references from the first odometry sample.

        Position references are left ALONE when they came from teleop's origin
        file: that file records a (odom, arena) pair captured at a moment the
        operator vouched for, and replacing it with wherever the rover happens to
        be now would throw away the only external information this node has.
        """
        a = self.anchor
        if a.source == "assumed start pose":
            a.odom_x, a.odom_y = odom_x, odom_y
        a.odom_yaw_deg = math.degrees(board_yaw_rad)
        a.gyro_yaw_deg = math.degrees(self.yaw_int)
        a.established = True
        log(f"anchor established from {a.source}: arena "
            f"({a.arena_x_mm:.0f}, {a.arena_y_mm:.0f}) mm heading "
            f"{a.arena_heading_deg:.0f}deg <- odom ({a.odom_x:+.3f}, "
            f"{a.odom_y:+.3f}) m board_yaw {a.odom_yaw_deg:+.1f}deg "
            f"gyro {a.gyro_yaw_deg:+.1f}deg")

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
        """Assume the start pose unless the operator has told teleop otherwise.

        THE ORIGIN FILE OUTLIVES THE ODOMETRY IT REFERS TO. It records a pair —
        "odom was here, the arena was there" — and survives a reboot, while the
        board's odometry restarts at 0. A reference captured at odom (9.2, 11.9)
        therefore places a freshly-booted rover nine metres outside a 1.2 m
        arena, and this node will plan a confident route from there. It has
        happened. So the file is refused when it predates the current boot, and
        the operator is told to re-zero rather than left to discover it at speed.
        """
        a = Anchor()
        try:
            mtime = os.path.getmtime(TELEOP_ORIGIN_FILE)
            if origin_is_stale(mtime, _boot_time()):
                log(f"anchor: IGNORING {TELEOP_ORIGIN_FILE} — it was written "
                    f"{(time.time() - mtime) / 60.0:.0f} min ago, BEFORE this "
                    "boot. The board's odometry restarted at 0, so its reference "
                    "no longer means anything. Re-zero with set_coordinate. "
                    "Falling back to the assumed start pose.")
                raise ValueError("origin file predates boot")
            with open(TELEOP_ORIGIN_FILE) as fh:
                d = json.load(fh)
            a.odom_x = float(d["ref_x"])
            a.odom_y = float(d["ref_y"])
            a.arena_x_mm = float(d["x_mm"])
            a.arena_y_mm = float(d["y_mm"])
            a.source = TELEOP_ORIGIN_FILE
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
        """(x_mm, y_mm, heading_deg) in the arena frame, or None with no odom.

        Position is rotated by the BOARD yaw (same frame as the position it
        rotates); heading is the GYRO delta since the anchor (a physical sensor,
        unaffected by the mirrored motor layout, and free of the skid-steer
        effective-track factor an encoder-derived yaw would carry). The two are
        different frames with different zeros and this file used to use one for
        both — see the docstring, "WHICH YAW IS USED FOR WHAT".
        """
        with self.lock:
            if self.odom_x is None:
                return None
            return arena_from_odom(self.anchor, self.odom_x, self.odom_y,
                                   math.degrees(self.odom_yaw),
                                   gyro_yaw_deg=math.degrees(self.yaw_int))

    def odom_xy(self):
        with self.lock:
            return self.odom_x, self.odom_y

    def board_yaw(self):
        """The BOARD's own yaw, radians. The frame odom_xy lives in — the ONLY
        angle an odom-frame displacement may be projected onto."""
        with self.lock:
            return self.odom_yaw

    def pose_source(self):
        """Which of the two localisation states the current pose came from.

        Never a blend and never a guess. `slam_anchored` means the anchor was
        last re-established from a `map`->`odom` fix and dead reckoning has
        carried it forward since; `deadreckon` means no fix has ever been
        adopted and the anchor is still the ASSUMED start pose. Recorded per leg
        so that "leg 3 ran on dead reckoning because localisation dropped" is an
        answer somebody can give afterwards instead of a theory.
        """
        with self.lock:
            return POSE_SLAM if self.fix_at is not None else POSE_DEADRECKON

    def fix_age_s(self):
        with self.lock:
            if self.fix_at is None:
                return None
            return time.monotonic() - self.fix_at

    # ------------------------------------------------------------- arena fix
    def at_rest(self):
        """Commanded stopped, and stopped for long enough to have settled.

        A fix taken in motion is worthless here: the scan crosses MQTT at
        9.83 Hz, so at least 16 mm of travel separates one scan from the next.
        `still_since` is reset by `publish()` on every non-zero setpoint, so this
        cannot be fooled by a worker that merely believes it has stopped.
        """
        return (time.monotonic() - self.still_since) >= FIX_STILL_S

    def read_arena_fix(self):
        """The arena pose implied by the live `map`->`odom`, or None with a why.

        Returns (pose_or_None, note). This node NEVER computes a fix — scan
        matching belongs to whatever owns `map`->`odom`. All that happens here is
        composing that transform with the live odom pose, which
        `pose_from_map_to_odom` does in four lines of arithmetic. Keeping the
        consumer that small is what lets the producer change from slam_toolbox to
        AMCL to anything else without touching this file.
        """
        if not ARENA_FIX_ENABLED:
            return None, "localisation disabled by config"
        if self.tf_buffer is None:
            return None, "no tf2 listener"
        x, y = self.odom_xy()
        if x is None:
            return None, "no odometry"
        try:
            tr = self.tf_buffer.lookup_transform(
                ARENA_FIX_MAP_FRAME, ARENA_FIX_ODOM_FRAME, RclTime())
        except Exception as e:
            # The normal case until the SLAM bringup lands: nothing is
            # broadcasting this edge, so the rover is dead reckoning and says so.
            return None, (f"no {ARENA_FIX_MAP_FRAME}->{ARENA_FIX_ODOM_FRAME} "
                          f"transform ({type(e).__name__}) — no localisation is "
                          "running, so this run is dead reckoning")
        try:
            t = tr.transform.translation
            myaw = yaw_from_quat(tr.transform.rotation)
            now = self.get_clock().now()
            stamp = tr.header.stamp
            age = (now.nanoseconds * 1e-9
                   - (float(stamp.sec) + float(stamp.nanosec) * 1e-9))
        except Exception as e:
            return None, f"malformed transform ({e})"
        if math.isfinite(age) and age > FIX_MAX_AGE_S:
            return None, (f"fix is {age:.1f}s old (max {FIX_MAX_AGE_S:.0f}) — "
                          "it does not describe where the rover is now")
        return (pose_from_map_to_odom(float(t.x), float(t.y), myaw,
                                      float(x), float(y), self.board_yaw()),
                "ok")

    def adopt_fix(self, fix_pose, when):
        """RE-ANCHOR onto an absolute fix. Returns (adopted, note).

        The fix moves the ANCHOR, not the pose: dead reckoning then carries on
        from a corrected origin using the same one pipeline it always used. The
        alternative — blending a fix into the live pose — would give this node
        two sources of truth that disagree mid-leg, and a `_drive` measuring its
        own displacement would see the correction as motion it had performed.

        Not called during a retrace. The retrace replays MEASURED motions and
        does not consult the arena frame, so a re-anchor there would change
        nothing except the numbers in the log.
        """
        dr = self.pose()
        if dr is None:
            return False, "no dead-reckoned pose to compare against"
        rejects = fix_rejects(fix_pose, dr)
        if rejects:
            with self.lock:
                self.fix_rejects_last = rejects
            log(f"arena fix REJECTED at {when}: " + "; ".join(rejects))
            return False, "; ".join(rejects)
        x, y = self.odom_xy()
        with self.lock:
            a = self.anchor
            a.odom_x, a.odom_y = x, y
            a.odom_yaw_deg = math.degrees(self.odom_yaw)
            a.gyro_yaw_deg = math.degrees(self.yaw_int)
            a.arena_x_mm, a.arena_y_mm, a.arena_heading_deg = fix_pose
            a.established = True
            a.source = f"arena LiDAR fix ({when})"
            # `assumed` stays TRUE while the LiDAR mount yaw and rotation sign are
            # unverified. A mirrored scan of a square arena fits perfectly and
            # returns a confidently reflected pose, so a fix taken through an
            # uncalibrated mount buys precision without proving accuracy — and
            # the dashboard's "POSE: SIMULATED" badge must not come off for that.
            a.assumed = not ARENA_FIX_CALIBRATION_VERIFIED
            self.fix_rejects_last = []
            self.fix_adopted_n += 1
            self.fix_at = time.monotonic()
            self.fix_last = {"when": when, "x_mm": jnum(fix_pose[0], 1),
                             "y_mm": jnum(fix_pose[1], 1),
                             "heading_deg": jnum(fix_pose[2], 1),
                             "moved_mm": jnum(math.hypot(fix_pose[0] - dr[0],
                                                         fix_pose[1] - dr[1]), 1),
                             "turned_deg": jnum(wrap180(fix_pose[2] - dr[2]), 1),
                             "calibration_verified": ARENA_FIX_CALIBRATION_VERIFIED,
                             "ts": time.time()}
        caveat = ("" if ARENA_FIX_CALIBRATION_VERIFIED
                  else " — mount yaw and rotation sign are UNVERIFIED, so this "
                       "is precision, not proven accuracy")
        log(f"arena fix ADOPTED at {when}: dead reckoning said "
            f"({dr[0]:.0f}, {dr[1]:.0f})mm h={dr[2]:.0f}deg, fix says "
            f"({fix_pose[0]:.0f}, {fix_pose[1]:.0f})mm h={fix_pose[2]:.0f}deg"
            + caveat)
        return True, "adopted"

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

        THE MIRRORED MOTOR LAYOUT IS CORRECTED HERE AND NOWHERE ELSE. Every
        caller — the turn primitive, the heading hold inside a drive, any future
        one — passes a yaw rate in the ARENA's sense (positive = CCW), and
        TURN_WIRE_SIGN converts it to whatever the rewired chassis currently
        means by positive. One place, so the two can never disagree, and so
        re-measuring the polarity is one constant rather than an audit.
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
            t.angular.z = float(to_cmd_ang(wz) * TURN_WIRE_SIGN)
            self.pub_cmd.publish(t)
            self._pub_times.append(time.monotonic())
            # Anything non-zero means the rover is no longer at rest, and an
            # arena fix taken from here on describes where it WAS. Reset the
            # stillness clock at the point the command leaves, not at the point
            # a worker believes it has stopped.
            if vx != 0.0 or wz != 0.0:
                self.still_since = time.monotonic()
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
            # WHICH LOCALISATION THIS POSE CAME FROM. Two named states, never a
            # blend: `deadreckon` (anchor is the assumed start pose) or
            # `slam_anchored` (anchor was re-established from a map->odom fix at
            # rest, dead reckoning since). `pose_fix_age_s` says how long since.
            "pose_source": self.pose_source(),
            "pose_fix_age_s": jnum(self.fix_age_s(), 1),
            "pose_anchor": self.anchor.source,
            "fix_note": st.fix_note,
            # The arm latch lives on the runner, but an operator watching the
            # dashboard needs to see it beside the mission state rather than
            # having to remember whether the last ack armed or disarmed.
            "armed": (self.runner.armed() if self.runner else None),
            "arm_required": REQUIRE_ARM,
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
    fix_note: str = None         # what the last stop-and-fix attempt did, or why
                                 # it did nothing — surfaced so a correction that
                                 # silently stops firing still says so
    pose_source: str = POSE_DEADRECKON   # never a blend of the two; see
                                         # MissionNode.pose_source()
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

        # A turn shorter than the chassis can express is a planning bug, not a
        # small turn: it would command a burst too short to move and the stall
        # detector would then abort the mission blaming the firmware.
        if not min_pulse_ok("turn", seg.target):
            raise MissionAbort(
                f"turn of {seg.target:+.1f}deg is below MIN_TURN_DEG "
                f"({MIN_TURN_DEG:.1f}) — shorter than the {MIN_PULSE_S:.2f}s "
                "pulse this firmware needs to move at all")

        node.state.driving = True
        node.state.reversing = False
        try:
            while True:
                runner.check_abort(turning=True)
                now = time.monotonic()
                # SIGNED, not abs(). With `abs` a turn driven the WRONG WAY
                # reaches the target magnitude just as happily as a correct one,
                # stops, records the opposite angle and carries on — which is
                # exactly the failure the mirrored motor layout produces, and it
                # would look like a successful turn in every log.
                turned = sign * (node.yaw() - yaw0)
                if turned >= stop_at:
                    break
                if turned <= -math.radians(TURN_WRONG_WAY_DEG):
                    # See TURN_WIRE_SIGN. Its value is derived from the rewiring,
                    # not measured, so this is the check that makes a wrong guess
                    # cost one twitch instead of a mission.
                    reason = ABORT_TURN_SIGN
                    break
                if now - t0 > timeout:
                    reason = ABORT_SEG_TIMEOUT
                    break
                if (now - t0 > TURN_STALL_CHECK_S
                        and abs(math.degrees(turned)) < TURN_STALL_MIN_DEG):
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
        # still lands inside it, and it is LONGER after a big turn because a big
        # turn coasts further (phase6_latest.py waited an extra 0.6 s past 45 deg).
        runner.settle(turn_settle_s(seg.target))
        seg.measured = math.degrees(node.yaw() - yaw0)
        seg.elapsed_s = time.monotonic() - t0
        seg.reason = reason
        if reason == ABORT_TURN_SIGN:
            raise MissionAbort(
                f"{ABORT_TURN_SIGN}: asked {seg.target:+.1f}deg, the rover "
                f"rotated {seg.measured:+.1f}deg — the OPPOSITE way. The motor "
                "layout was mirrored on 2026-08-01 (M1/M2 are now the right "
                f"side), so TURN_WIRE_SIGN={TURN_WIRE_SIGN:+d} is wrong for this "
                "chassis. Set FPMS_MISSION_TURN_WIRE_SIGN="
                f"{-TURN_WIRE_SIGN:+d} in /etc/fpms/config.env and re-run.")
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
        # THE BOARD's yaw, not the gyro's. dx/dy below are in the board's odom
        # frame and only the board's own yaw describes that frame's orientation;
        # projecting them onto the gyro integral mixes two frames with different
        # zeros and silently scales every distance the retrace depends on. This
        # is NAV2_BRIEF section 3a's prescription, and what fpms_teleop.py
        # `summarize_leg` and fpms_odom_tf.py `travel_sign()` already do.
        byaw0 = node.board_yaw()
        gyaw0 = node.yaw()                  # for heading hold only
        sign = 1.0 if seg.target >= 0 else -1.0
        reverse = sign < 0
        target_mm = abs(seg.target)
        stop_at = target_mm * DRIVE_COAST_FACTOR
        timeout = segment_timeout_s(seg)
        speed = DOCK_MPS if seg.dock else CRUISE_MPS
        reason = "done"
        along = lateral = 0.0

        if not min_pulse_ok("drive", seg.target):
            raise MissionAbort(
                f"drive of {seg.target:+.0f}mm is below MIN_MOVE_MM "
                f"({MIN_MOVE_MM:.0f}) — shorter than the {MIN_PULSE_S:.2f}s "
                "pulse this firmware needs to move at all")

        node.state.driving = True
        node.state.reversing = reverse
        try:
            while True:
                runner.check_abort(reverse=reverse)
                now = time.monotonic()
                along, lateral = self._displacement(x0, y0, byaw0)
                # THE MINIMUM PULSE IS A FLOOR ON TIME, NOT ONLY ON DISTANCE. The
                # firmware is converging a PID onto a velocity setpoint; cutting
                # the burst early because the coast target was met on a noisy
                # early sample leaves it having never converged, and the rover
                # does not move. Planning guarantees every segment is at least
                # this long, so holding for it can never overshoot a segment that
                # was legitimately shorter.
                if now - t0 >= MIN_PULSE_S and sign * along >= stop_at:
                    break
                if now - t0 > timeout:
                    reason = ABORT_SEG_TIMEOUT
                    break
                if now - t0 > STALL_CHECK_S and abs(along) < STALL_MIN_MM:
                    reason = ABORT_STALL
                    break
                # Heading hold, in the ARENA's sense of positive — publish()
                # applies TURN_WIRE_SIGN. Getting that sign wrong here is worse
                # than getting a turn wrong: a heading hold with inverted
                # polarity is POSITIVE FEEDBACK and the rover spirals while every
                # number in the log stays plausible.
                err = wrap_pi(gyaw0 - node.yaw())
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
        # bursts a slow approach instead of a lurch. It is also the window in
        # which an arena fix can be taken, if one is ever published — a fix in
        # motion is worth nothing at 9.83 Hz over MQTT.
        runner.settle(STOP_SETTLE_S)
        along, lateral = self._displacement(x0, y0, byaw0)
        seg.measured = along
        seg.lateral_mm = lateral
        seg.elapsed_s = time.monotonic() - t0
        seg.reason = reason
        if reason in (ABORT_SEG_TIMEOUT, ABORT_STALL):
            raise MissionAbort(f"{reason} (drive asked {seg.target:+.0f}mm, "
                               f"measured {seg.measured:+.0f}mm)")
        return seg

    def _displacement(self, x0, y0, board_yaw0):
        """Signed travel along the leg heading, and drift across it, in mm.

        `board_yaw0` is the BOARD's yaw at the start of the segment, from the
        same message stream as x0/y0. If the board publishes an identity
        orientation this degenerates to `dx`, which is exactly the signed
        quantity NAV2_BRIEF's measured twist-vs-pose table was read from — the
        identity case is the measured case, not a fallback.
        """
        x, y = self.node.odom_xy()
        if x is None:
            raise MissionAbort(ABORT_LINK)
        dx = (x - x0) * 1000.0
        dy = (y - y0) * 1000.0
        c, s = math.cos(board_yaw0), math.sin(board_yaw0)
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
        # ARM. Monotonic deadline rather than a bool, so "armed" cannot outlive
        # the operator's attention: an arm that is never used expires on its own.
        self._armed_until = 0.0
        self._armed_by = None

    # ----------------------------------------------------------------- arming
    def armed(self):
        return (not REQUIRE_ARM) or (time.monotonic() < self._armed_until)

    def arm_remaining_s(self):
        return max(0.0, self._armed_until - time.monotonic())

    def set_armed(self, on, why, by=None):
        """The ONE place the arm latch moves. Disarming is always allowed and
        never conditional — a disarm that could be refused is not a safety
        feature."""
        was = self.armed()
        if on:
            self._armed_until = time.monotonic() + ARM_TIMEOUT_S
            self._armed_by = by
        else:
            self._armed_until = 0.0
            self._armed_by = None
        if was != self.armed():
            log(f"{'ARMED' if on else 'DISARMED'}: {why}"
                + (f" (expires in {ARM_TIMEOUT_S:.0f}s)" if on else ""))
        return self.armed()

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
        # ---- payload MODES that command nothing, handled before anything else.
        # They live in this verb's payload rather than in verbs of their own
        # because this service owns exactly one verb: a second one would have to
        # be mirrored into fpms_rover_agent's SILENT_VERBS or it would race an
        # "unknown command" nack onto the dashboard over the real reply, and
        # fpms_rover_agent.py is not a file this work owns.
        if "arm" in payload:
            self._cmd_arm(bool(payload.get("arm")))
            return
        if payload.get("probe"):
            self._cmd_probe(str(payload.get("probe")))
            return

        name = str(payload.get("name", "") or "")
        backend = str(payload.get("backend", "") or DEFAULT_BACKEND)
        ret = str(payload.get("return", "") or DEFAULT_RETURN)

        if name not in COMMANDABLE:
            self._nack(f"unknown mission {name!r}", valid=list(COMMANDABLE))
            return
        if backend not in BACKENDS:
            self._nack(f"unknown backend {backend!r}", valid=list(BACKENDS))
            return
        if ret not in RETURN_STRATEGIES:
            self._nack(f"unknown return strategy {ret!r}",
                       valid=list(RETURN_STRATEGIES))
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
            # ARM FIRST. It is the operator's own consent to motion, so refusing
            # for it before anything else means an unarmed rover never reports a
            # flat battery or a stale LiDAR as the reason it did not move.
            if not self.armed():
                self._nack(
                    f"{ABORT_DISARMED}. Nothing moves until the rover is armed: "
                    'send {"arm": true} on this same `mission` command, then the '
                    "mission. It disarms itself on any stop or abort, when a "
                    f"mission finishes, and after {ARM_TIMEOUT_S:.0f}s unused. "
                    "Set FPMS_MISSION_REQUIRE_ARM=0 only with the wheels off the "
                    "floor.",
                    armed=False, arm_timeout_s=ARM_TIMEOUT_S)
                return
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
            # THE HIGHEST-VALUE FIX IS THE ONE TAKEN BEFORE THE FIRST SEGMENT.
            # The two largest dead-reckoning error terms are the ASSUMED start
            # position and heading, and neither is observable by odometry — 3 deg
            # of start-heading error is 52 mm of lateral error after one metre,
            # and it is inherited by every leg that follows. The rover is
            # stationary right now, which is the only condition under which a fix
            # is worth taking at all.
            self.try_arena_fix("mission start")

            pose = self.node.pose()
            if pose is None:
                self._nack("no pose: odometry has not been seen yet")
                return
            # THE GUARD THAT WOULD HAVE CAUGHT THE 10 m START. A stale origin
            # file plus a rebooted board once put the rover at (-9182, -11926) mm
            # inside a 1200 mm arena, and it planned from there with complete
            # confidence. A start pose outside the arena is never a route worth
            # driving, whatever produced it.
            if not in_arena(pose[0], pose[1]):
                self._nack(
                    f"start pose ({pose[0]:.0f}, {pose[1]:.0f}) mm is OUTSIDE the "
                    f"{ARENA_MM:.0f} mm arena, so every target is somewhere the "
                    "rover cannot be. Almost always a stale origin: the board's "
                    "odometry restarts at 0 on every boot while the origin file "
                    "survives. Re-zero with set_coordinate and try again.",
                    x_mm=jnum(pose[0], 1), y_mm=jnum(pose[1], 1),
                    anchor_source=self.node.anchor.source)
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

            # A PLANNED RETURN IS ONLY BETTER THAN A RETRACE IF LOCALISATION IS
            # REALLY LIVE. Without it, a fresh plan home is computed from a pose
            # that has accumulated every millimetre of the outbound drift and
            # carries all of it back with it, while the retrace cancels that
            # drift by construction. So this is refused rather than silently
            # downgraded: an operator who asked for `planned` and got `retrace`
            # anyway would have no way to know which one drove.
            if ret == "planned" and self.node.pose_source() != POSE_SLAM:
                self._nack(
                    "return strategy 'planned' needs live localisation, and no "
                    f"{ARENA_FIX_MAP_FRAME}->{ARENA_FIX_ODOM_FRAME} fix has been "
                    "adopted. A fresh plan home from a dead-reckoned pose carries "
                    "the whole outbound drift with it; the retrace cancels it. "
                    "Start the SLAM bringup, or send return='retrace'.",
                    requested_return=ret, pose_source=self.node.pose_source(),
                    fix=self._fix_status())
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
            st.fix_note = None

            # PLAN, THEN FOLLOW — in that order, and visibly. The route goes out
            # on telemetry/mission_plan BEFORE the worker exists, produced by the
            # same `_publish_plan` the preview uses from the same `plan_legs`, so
            # the line the operator sees is the line the rover is about to drive
            # rather than a second drawing of the same intent. A drawn route the
            # rover does not follow is worse than no route at all.
            self._publish_plan(name, backend, pose, legs, plan_legs, plan,
                               committed=True, ret=ret)

            self.thread = threading.Thread(
                target=self._run, name="mission",
                args=(name, backend, legs, pose, ret), daemon=True)
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

    # ------------------------------------------------------------- arena fix
    def try_arena_fix(self, when):
        """Take an arena fix if one is available AND the rover is at rest.

        Returns a short note, always. Never raises, never moves anything, and
        never blocks for longer than a tf lookup — it is called from the command
        thread at mission start and from the worker between legs.

        The rover being stationary is not a nicety. Scans reach this Pi through
        MQTT at 9.83 Hz, so a fix taken while moving describes a pose at least
        16 mm stale, which is larger than the entire error budget the fix exists
        to buy. Stopping between segments is already the rhythm this file drives
        in, so stop-and-fix costs nothing extra.
        """
        try:
            if not ARENA_FIX_ENABLED:
                return "arena fix disabled"
            if not self.node.at_rest():
                return "not at rest; a fix in motion is worth nothing"
            fix, note = self.node.read_arena_fix()
            if fix is None:
                return note
            ok, why = self.node.adopt_fix(fix, when)
            return "adopted" if ok else f"rejected: {why}"
        except Exception as e:                                # pragma: no cover
            log(f"arena fix attempt failed at {when}: {e}")
            return f"error: {e}"

    # ----------------------------------------------------------- arm / probe
    def _cmd_arm(self, on):
        """Arm or disarm. Commands no motion either way."""
        if on and not REQUIRE_ARM:
            log("arm requested but FPMS_MISSION_REQUIRE_ARM=0 — motion is "
                "already ungated; arming is a no-op")
        if on:
            self.set_armed(True, "operator armed the rover", by="mqtt")
        else:
            # Disarming also drops any mission in flight. An operator who
            # withdraws consent to motion has not asked for the current motion to
            # finish first.
            if self.thread is not None and self.thread.is_alive():
                self.request_abort("disarmed by operator")
            self.set_armed(False, "operator disarmed the rover")
        self.bus.publish("events/ack",
                         {"action": "mission", "mode": "arm", "accepted": True,
                          "started": False, "armed": self.armed(),
                          "arm_required": REQUIRE_ARM,
                          "expires_in_s": jnum(self.arm_remaining_s(), 1),
                          "arm_timeout_s": ARM_TIMEOUT_S}, qos=1)

    def _cmd_probe(self, what):
        """Report what the drive board ACTUALLY provides. Commands no motion.

        WHY THIS IS NOT AN ENCODER READOUT, AND WILL NOT PRETEND TO BE ONE
        ------------------------------------------------------------------
        There is no per-wheel encoder topic on this rover. The board publishes
        an INTEGRATED pose and a twist on `/odom_raw` and nothing else — no tick
        counts, no per-wheel velocities, no duty or current. The prior working
        code read four tick counters directly off a Rosmaster board over serial
        (`sum(abs(e[i]-e0[i]) for i in range(4))/4.0 * MM_PER_TICK`); that
        protocol cannot address this ESP32 board at all, which is why
        `Rosmaster_Lib` returns version -1 and four zeros here. Those zeros mean
        WRONG PROTOCOL, not stopped wheels.

        So this reports what exists and NAMES WHAT DOES NOT. Synthesising
        per-wheel counts by dividing an integrated pose back down would produce
        four numbers that look like measurements, agree with each other by
        construction, and could never reveal the one thing per-wheel counts are
        for — that one wheel is doing something different from the others.

        It also puts the twist and the pose-derived displacement for the SAME
        interval side by side, because that pair is the single measurement that
        settles the sign inversion, and it costs nothing to publish it.
        """
        if what not in ("encoders", "odom", "all"):
            self._nack(f"unknown probe {what!r}",
                       valid=["encoders", "odom", "all"])
            return
        node = self.node
        with node.lock:
            now = time.monotonic()
            odom_hz = _rate_hz(node._odom_times, now)
            imu_hz = _rate_hz(node._imu_times, now)
            twist_x = node._probe_twist_x
            along = node._probe_along_mm
            payload = {
                "action": "mission", "mode": "probe", "probe": what,
                "commanded_motion": False,
                "source_topics": {
                    "/odom_raw": {"present": node.odom_x is not None,
                                  "hz": jnum(odom_hz, 2),
                                  "age_s": jnum(now - node.odom_last, 2)
                                  if node.odom_last else None,
                                  "provides": ["pose.position.x/y (integrated, m)",
                                               "pose.orientation (see identity note)",
                                               "twist.linear.x (SIGN-INVERTED)",
                                               "twist.angular.z"]},
                    "/imu": {"hz": jnum(imu_hz, 2),
                             "provides": ["angular_velocity.z (rad/s)"],
                             "orientation_fused": False},
                    "/battery": {"provides": ["decivolts"]},
                },
                "integrated_pose_m": {"x": jnum(node.odom_x, 4),
                                      "y": jnum(node.odom_y, 4)},
                "board_yaw_deg": jnum(math.degrees(node.odom_yaw), 2),
                "board_orientation_is_identity": node.odom_yaw_identity,
                "gyro": {"rate_radps": jnum(node.gyro_z, 4),
                         "bias_radps": jnum(node.gyro_bias, 5),
                         "bias_samples": node.gyro_bias_n,
                         "integrated_deg": jnum(math.degrees(node.yaw_int), 2)},
                # The discriminating pair. NAV2_BRIEF section 3a was written from
                # exactly this comparison, and nothing in this file reads the
                # twist for any other purpose.
                "sign_check": {
                    "twist_linear_x": jnum(twist_x, 4),
                    "pose_along_mm_last_sample": jnum(along, 3),
                    "note": ("twist.linear.x is SIGN-INVERTED relative to its own "
                             "pose on this firmware. If these two disagree in "
                             "sign, the pose is right. No guard in this file "
                             "reads twist."),
                },
                # The point of the verb: an explicit inventory of absences.
                "not_available": [
                    "per-wheel encoder TICK COUNTS — no topic publishes them",
                    "per-wheel velocities — the board publishes one chassis twist",
                    "motor duty / current / temperature — not published",
                    "wheel slip or stall flags — not published",
                    "a fused orientation — /imu carries an identity quaternion; "
                    "heading here is integrated from angular_velocity.z",
                ],
                "why_not_synthesised": (
                    "dividing the integrated pose back into four wheel counts "
                    "would produce numbers that agree by construction and could "
                    "never show one wheel behaving differently — which is the "
                    "only thing per-wheel counts are for. The dead rear-left "
                    "cable was found by a yaw-drift symmetry test, not by "
                    "reading counts."),
                "constants_in_use": {
                    "TURN_WIRE_SIGN": TURN_WIRE_SIGN,
                    "TURN_WIRE_SIGN_measured": TURN_WIRE_SIGN_MEASURED,
                    "CMD_SCALE": CMD_SCALE,
                    "MIN_PULSE_S": MIN_PULSE_S,
                    "MIN_PULSE_measured_under_load": MIN_PULSE_MEASURED_UNDER_LOAD,
                    "MIN_MOVE_MM": jnum(MIN_MOVE_MM, 1),
                    "MIN_TURN_DEG": jnum(MIN_TURN_DEG, 1),
                    "WIRE_FLOOR_MPS": WIRE_FLOOR_MPS,
                },
                "arena_fix": self._fix_status(),
                "config_notes": list(CFG_NOTES),
            }
        log(f"probe {what!r}: odom {payload['source_topics']['/odom_raw']['hz']}Hz, "
            f"imu {payload['source_topics']['/imu']['hz']}Hz, "
            "no per-wheel encoder topic exists — reported as missing")
        self.bus.publish("events/encoder_probe", payload, qos=1)

    def _fix_status(self):
        """One shape for "what does the arena fix know", used by probe, telemetry
        and the mission report so the three can never drift apart."""
        node = self.node
        _fix, note = (None, "not attempted")
        try:
            _fix, note = node.read_arena_fix()
        except Exception as e:                                # pragma: no cover
            note = f"error: {e}"
        return {
            "enabled": ARENA_FIX_ENABLED,
            "available": _fix is not None,
            "note": note,
            "at_rest": node.at_rest(),
            "pose_source": node.pose_source(),
            "age_s": jnum(node.fix_age_s(), 1),
            "adopted_n": node.fix_adopted_n,
            "last": node.fix_last,
            "last_rejects": list(node.fix_rejects_last),
            "map_frame": ARENA_FIX_MAP_FRAME,
            "odom_frame": ARENA_FIX_ODOM_FRAME,
            # SAY IT EVERY TIME. A wrong LiDAR rotation sign mirrors every scan,
            # so SLAM builds a mirrored map, matches it perfectly, and localises
            # the rover confidently into a reflected room. An unverified mount
            # calibration is not a footnote — it is the difference between a fix
            # that is precise and one that is right.
            "calibration_verified": ARENA_FIX_CALIBRATION_VERIFIED,
            "calibration_caveat": (
                "LIDAR_ZERO_OFFSET_DEG and LIDAR_ROTATION_SIGN "
                "(fpms_lidar_ros.py:164-168) are UNVERIFIED; 1 deg of mount yaw "
                "is 10.5 mm, and a wrong rotation sign mirrors every scan — the "
                "resulting map matches itself perfectly while being reflected"),
        }

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
        self._publish_plan(name, backend, pose, legs, plan_legs, plan,
                           committed=False)

        log(f"preview {name!r}: {len(legs)} leg(s), {len(plan)} segments, "
            f"{remaining_distance_mm(plan):.0f}mm outbound, no motion commanded")
        self.bus.publish("events/ack",
                         {"action": "mission", "name": name, "preview": True,
                          "accepted": True, "started": False,
                          "legs_n": len(legs),
                          "segments_planned": len(plan)}, qos=1)

    def _publish_plan(self, name, backend, pose, legs, plan_legs, plan,
                      committed, ret=DEFAULT_RETURN):
        """Publish the route on telemetry/mission_plan. PLAN, THEN FOLLOW.

        The same payload, from the same code, whether it was asked for as a
        preview or is about to be driven — `committed` is the only difference,
        and it says which. That is the point: a route drawn on the dashboard that
        the rover does not then follow is worse than drawing nothing, and the
        only way to be sure they agree is for there to be one producer.

        Acceptance publishes this BEFORE the worker starts, so the drawn line
        always exists before the first wheel turns rather than being reconstructed
        afterwards from telemetry.
        """
        # Cumulative pose after each segment. measured=False because nothing has
        # executed — these are the TARGETS, which is what a plan means.
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
            # False = "this is what I WOULD drive"; True = "this is what I AM
            # about to drive, published before the first wheel turns".
            "committed": bool(committed),
            "route": route_description(name),
            "pose_assumed": bool(self.node.anchor.assumed),
            "pose_source": self.node.anchor.source,
            "arena_fix": self._fix_status(),
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
            "return_strategy": (ret or DEFAULT_RETURN) if returns_home else "none",
            "returns_home": returns_home,
        }, qos=1)

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
        # AUTO-DISARM. Whatever went wrong, the operator's consent to motion was
        # given for the run that just ended, not for whatever anyone sends next.
        # Re-arming is one message and is the cheap half of this transaction.
        self.set_armed(False, f"aborted ({reason})")
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
    def _run(self, name, backend_name, legs, start_pose, ret=DEFAULT_RETURN):
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
        # WHICH LOCALISATION EACH LEG ACTUALLY RAN ON. Recorded as the leg
        # finishes, not inferred afterwards: "leg 3 ran on dead reckoning because
        # localisation dropped" is the answer to "why was this run 8 cm out", and
        # it only exists if somebody wrote it down at the time.
        leg_sources = []
        used_return = ret

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
                # speed control (see the docstring), so the hold is doing triple
                # duty: it is the operator-visible dwell, the settle that makes
                # the next leg's first measurement trustworthy, AND the window in
                # which an arena fix can be taken. Stop-and-fix at waypoints is
                # the whole reason a fix is worth +/-5-9 mm instead of nothing —
                # a scan crossing MQTT at 9.83 Hz is at least 16 mm stale if the
                # rover is still moving when it arrives.
                st.phase = "hold"
                # The pose source the leg was DRIVEN on, captured before the fix
                # at the end of it — otherwise every leg would claim the
                # localisation that only arrived once it was already parked.
                driven_on = self.node.pose_source()
                self.settle(HOLD_S)
                st.fix_note = self.try_arena_fix(f"leg {leg_i} ({leg_name})")
                st.pose_source = self.node.pose_source()
                leg_sources.append({"i": leg_i, "leg": leg_name,
                                    "label": TARGET_LABEL.get(leg_name),
                                    "driven_on": driven_on,
                                    "fix_at_target": st.fix_note})

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
                elif ret == "planned" and self.node.pose_source() == POSE_SLAM:
                    # A PLANNED RETURN, AND ONLY WHEN LOCALISATION IS STILL LIVE.
                    # This is not the default and never becomes it by accident:
                    # accept-time refused `planned` without a fix, and this
                    # re-checks at the moment of use, because localisation can
                    # drop during the outbound run. If it has, the retrace below
                    # is still available and still correct — falling back to it
                    # loses nothing except the shortcut.
                    #
                    # It is better than a retrace ONLY here: with a fix, "where
                    # am I" is an observation rather than an integral, so a fresh
                    # plan does not carry the outbound drift home with it. Without
                    # one it carries all of it, which is exactly what the retrace
                    # exists to prevent.
                    hx, hy, _hh = mission_target("home")
                    log(f"return: PLANNED from a localised pose "
                        f"(fix {self.node.fix_age_s():.1f}s old) rather than a "
                        "retrace")
                    executed += self._drive_to(hx, hy)
                    st.fix_note = self.try_arena_fix("home, after planned return")
                    self._begin_leg("home-trim", len(legs) + 1,
                                    budget=HOME_TRIM_TRIES + 1)
                    executed += self._home_trim(start_pose)
                else:
                    if ret == "planned":
                        used_return = "retrace"
                        log("return: 'planned' was requested but localisation is "
                            "no longer live, so the RETRACE is driving instead — "
                            "it needs no map and cancels drift by construction")
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
                    # The rover is stopped at the start box and the retrace has
                    # already put the heading back, so this is the second most
                    # valuable moment in the whole run to take a fix — and the
                    # trim below is what turns it into millimetres.
                    st.fix_note = self.try_arena_fix("home, after retrace")
                    # A FRESH BUDGET. The retrace's budget was its own length —
                    # exactly right for a fixed replay and exactly zero left over
                    # — so the trim needs one of its own or its first nudge would
                    # abort the run for "not converging".
                    self._begin_leg("home-trim", len(legs) + 1,
                                    budget=HOME_TRIM_TRIES + 1)
                    executed += self._home_trim(start_pose)
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
            # AUTO-DISARM ON COMPLETION TOO, not only on abort. The operator
            # armed the rover for this run; the next one is a new decision.
            self.set_armed(False, f"mission {name!r} finished ({outcome})")

        self._report(name, backend_name, legs, start_pose, executed,
                     outcome, reason, time.monotonic() - t0, start_yaw,
                     leg_sources=leg_sources, used_return=used_return,
                     requested_return=ret)
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
            if not legs:
                # `split_legs` returning nothing means the remaining distance is
                # under MIN_MOVE_MM: real, but shorter than any motion this
                # chassis can perform. That is arrival — as close as the hardware
                # goes — and the residual is REPORTED rather than chased with a
                # burst that would stand still and then abort as a stall.
                log(f"arrived within {dist:.0f}mm of the target; closer than "
                    f"MIN_MOVE_MM ({MIN_MOVE_MM:.0f}), which is the shortest "
                    "motion this firmware will act on")
                st.distance_remaining_mm = dist
                return executed
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
        if abs(err) < HEADING_TOL_DEG or not min_pulse_ok("turn", err):
            # Below MIN_TURN_DEG the correction is not small, it is IMPOSSIBLE:
            # the burst would be shorter than the firmware's minimum pulse and
            # the rover would not move. Reporting a 5 degree residual is honest;
            # commanding a turn that stands still and then aborting the mission
            # as a stall is not.
            if abs(err) >= HEADING_TOL_DEG:
                log(f"re-face SKIPPED: {err:+.1f}deg is under MIN_TURN_DEG "
                    f"({MIN_TURN_DEG:.1f}) — the chassis has no turn that small")
            return executed
        if abs(err) > MAX_REFACE_DEG:
            log(f"re-face SKIPPED: heading error {err:+.1f}deg exceeds "
                f"{MAX_REFACE_DEG:.0f}deg — reporting instead of spinning")
            return executed
        executed.append(self._run_one(self.backends["deadreckon"],
                                      Segment("turn", err)))
        return executed

    def _home_trim(self, start_pose):
        """Close the residual gap to the start pose WITHOUT turning.

        GOLDEN TECHNIQUE, from phase6_latest.py `_p5_navdrive`'s final HOME
        correction: after the retrace, measure the gap to home and nudge
        straight — forwards if home is ahead, backwards if it is behind — at most
        a few times, and give up quietly.

        THE NO-TURN RULE IS THE WHOLE POINT. The retrace has just undone every
        outbound turn by the amount it was measured at, which is the only reason
        the heading is right. Spending a turn here to fix a few millimetres of
        position would trade the good quantity for the bad one; each turn is
        worth +/-1-4 degrees, and at the start box a degree is worth far more
        than a millimetre. So the trim drives along the heading it already has
        and accepts whatever lateral error remains — which the report names.

        Every nudge is recorded in `executed`, so it appears in the measured log
        exactly like any other segment.
        """
        executed = []
        for _ in range(HOME_TRIM_TRIES):
            self.check_abort()
            pose = self.node.pose()
            if pose is None:
                raise MissionAbort(ABORT_LINK)
            dx = start_pose[0] - pose[0]
            dy = start_pose[1] - pose[1]
            gap = math.hypot(dx, dy)
            if gap <= HOME_TRIM_TOL_MM:
                return executed
            # Signed component along the CURRENT heading. The cross component is
            # deliberately discarded rather than corrected — see the docstring.
            r = math.radians(pose[2])
            along = dx * math.cos(r) + dy * math.sin(r)
            step = clamp(along, -HOME_TRIM_MAX_MM, HOME_TRIM_MAX_MM)
            if not min_pulse_ok("drive", step):
                log(f"home trim done: {gap:.0f}mm off, of which {along:+.0f}mm "
                    f"is along the heading — under MIN_MOVE_MM "
                    f"({MIN_MOVE_MM:.0f}), so no motion can close it")
                return executed
            log(f"home trim: {gap:.0f}mm from the start box, nudging "
                f"{step:+.0f}mm along the current heading, no turn")
            executed.append(self._run_one(self.backends["deadreckon"],
                                          Segment("drive", step, dock=True)))
        return executed

    # --------------------------------------------------------------- report
    def _report(self, name, backend_name, legs, start_pose, executed,
                outcome, reason, elapsed, start_yaw, leg_sources=None,
                used_return=None, requested_return=None):
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
            # WHICH LOCALISATION DROVE WHAT. The first question after an 8 cm
            # miss is which legs were localised and which were dead reckoning,
            # and it is unanswerable unless it was recorded as it happened.
            "pose_source": self.node.pose_source(),
            "leg_pose_sources": list(leg_sources or []),
            "return_strategy": used_return or requested_return,
            "return_strategy_requested": requested_return,
            "arena_fix": self._fix_status(),
            # What the retrace could NOT give back, because the chassis has no
            # motion that small. Non-zero here explains a residual error that
            # would otherwise look like drift. Meaningless for a planned return
            # — nothing was inverted — so it is omitted rather than reported as
            # a zero somebody could mistake for a perfect cancellation.
            "retrace_residual": ({
                k: jnum(v, 2) for k, v in retrace_residual(
                    [s for s in executed if not s.retrace]).items()}
                if (used_return or requested_return) == "retrace" else None),
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
    node.runner = runner
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
