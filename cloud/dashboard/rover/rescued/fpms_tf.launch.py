#!/usr/bin/env python3
"""
fpms_tf.launch.py - static TF publishers that complete the FPMS Nav2 tree.

WHY THIS FILE EXISTS
======================
Nav2 and slam_toolbox both need a connected TF tree from `odom` down to the
laser frame. Only one edge in that tree changes at runtime:

    odom -> base_footprint      DYNAMIC, published by fpms_odom_tf.py.

Everything below `base_footprint` is a fixed mechanical offset on this
chassis - the LiDAR and IMU do not move relative to the body - so those edges
are STATIC transforms, which is exactly what this launch file publishes:

    base_footprint -> base_link
    base_link      -> laser_frame
    base_link      -> imu_frame

`map -> odom` is published by slam_toolbox (see ../slam/), NOT here and NOT by
fpms_odom_tf.py. REP-105 gives that edge to the localiser and to nobody else.

This file does NOT publish odom -> base_footprint. Do not add it here: that
edge already has an owner (fpms_odom_tf.py) and a second broadcaster for the
same edge is a textbook way to get two disagreeing transforms fighting each
other in tf2, which manifests as a TF tree that looks connected right up until
you look closely at timestamps and one of the two publishers "wins" at random.

REQUIRED COMPANION CHANGE - READ BEFORE LAUNCHING THIS FILE
================================================================
fpms_odom_tf.py ALSO ships a static broadcaster for base_footprint ->
laser_frame directly (its `PUBLISH_LASER_STATIC_TF` flag). That predates
base_link existing in this tree at all. Running both that broadcaster and this
launch file at once gives `laser_frame` two different parents (base_footprint
from one node, base_link from this one) - not a warning, an actively broken,
non-tree TF graph. `PUBLISH_LASER_STATIC_TF` is already `False` in
fpms_odom_tf.py. If you ever see `laser_frame` with two parents in
`ros2 run tf2_tools view_frames`, this is the first thing to check.

WHY base_link EXISTS AT ALL (base_footprint alone is not enough)
=====================================================================
`base_footprint` is Nav2's convention for "the robot's origin projected onto
the ground plane" (z pinned to 0, no roll/pitch) - it is what the planner and
costmaps key off. Sensors, however, are not mounted at ground level, so REP-105
puts a second frame, `base_link`, at the robot's actual body reference point
(which CAN carry a z offset and, on an uneven robot, roll/pitch) and hangs
every sensor off of `base_link`, not `base_footprint` directly. Two frames
because "where Nav2 plans" and "where the sensors are relative to the chassis"
are different concerns that happen to coincide in x/y but not in z.

base_footprint -> base_link, this chassis
-------------------------------------------
Pure vertical lift, no rotation: base_link sits directly above base_footprint
at the height of the drive axle centreline. Wheel diameter is MEASURED
hardware (NAV2_BRIEF.md section 2, hardware truth table): 70mm, so the axle
centreline - and therefore base_link - sits 35mm above the ground. This is not
a placeholder; it follows directly from the measured wheel diameter and needs
no further measurement.

################################################################################
#  THE MOUNT OFFSETS BELOW ARE PLACEHOLDERS. NOBODY HAS PUT A RULER ON THIS    #
#  ROBOT. THEY ARE NOW LAUNCH ARGUMENTS SO THEY CAN BE SUPPLIED WITHOUT        #
#  EDITING CODE - THAT DOES NOT MAKE THE DEFAULTS TRUE.                        #
################################################################################

WHY THIS MATTERS MORE THAN IT LOOKS - THE ARITHMETIC
=======================================================
R2_COORDINATES.md section 4.5: a constant LiDAR mount-yaw error rotates the
ENTIRE inferred pose about the sensor. Over a 600 mm lever arm that is

    1 degree of mount yaw  ~=  10.5 mm of position error

so a mount yaw that is "about right, probably within a couple of degrees" is
already a ~2 cm bias on every fix, applied consistently, in a direction that
depends on heading - i.e. it does not average out, it warps the map. For a
centimetre-accuracy target the mount yaw has to be known to better than
~0.5 degrees. Nothing in software can find this number for you; it is a
property of how the sensor is bolted on.

A wrong laser TRANSLATION is milder but not free: it puts every obstacle in
the wrong place in the costmap, and during rotation it makes the world appear
to swing about the wrong centre, which the scan matcher sees as inconsistent
geometry and partially absorbs into pose error.

A wrong laser PITCH is the sneaky one. The scan plane must be level. If the
LiDAR is mounted with a downward pitch `p` and sits at height `h` above the
floor, the beams strike the floor at range `h / tan(p)`:

    h = 0.10 m, p = 1 deg  ->  floor strike at  5.7 m
    h = 0.10 m, p = 2 deg  ->  floor strike at  2.9 m

The room this rover is in returns a median of ~1.98 m and a maximum of 6.0 m,
so a mount pitch of only 1-2 degrees turns the far half of every scan into a
ring of phantom obstacles on the floor. That is not a subtle degradation - it
is a fake wall that moves with the robot, and it will wreck both the map and
every costmap built from it.

WHAT THE OPERATOR MUST PHYSICALLY MEASURE
============================================
All in the ROS body convention: +x out the nose, +y to the robot's LEFT,
+z up. Origin is `base_link` = 35 mm above the floor, on the chassis
centreline, over the drive axle.

  1. laser_x  - horizontal distance from base_link to the LiDAR's OPTICAL
                CENTRE (the rotating mirror axis, not the case edge), positive
                forward. Callipers or a steel rule; +/- 2 mm is good enough.
  2. laser_y  - same, positive to the left. If the LiDAR looks centred, still
                measure it: 5 mm of lateral offset is 5 mm of map bias.
  3. laser_z  - height of the optical centre above base_link, i.e.
                (height above floor) - 0.035. Least critical of the three for
                planar SLAM, but needed for the pitch check below.
  4. laser_yaw - THE IMPORTANT ONE, and it is an ANGLE, not a distance.
                Method: park the rover with its nose squarely against a flat
                wall, run `ros2 topic echo /scan_lidar --once`, and find the
                index of the minimum range. Index 180 is dead ahead. Each
                index is 1 degree, so `laser_yaw = (180 - argmin) * pi/180`.
                Better: take the ranges either side of the minimum and fit,
                which gets you well below the 1 degree bin size. Repeat with
                the rover rotated 90 and 180 degrees; the answer must be the
                same every time. If it is not, the mount is loose.
  5. laser_roll / laser_pitch - level the scan plane. Put a small spirit level
                or a phone inclinometer on the LiDAR's mounting face and read
                both axes. Target < 0.5 degrees. If you cannot get it level
                mechanically, measure the residual and pass it here; the TF
                will at least place the returns correctly, though a tilted
                plane still slices the room at an angle.
  6. IMU offsets - only matter if a robot_localization EKF is ever added.
                fpms_odom_tf.py reads /imu's gyro-z directly and a pure yaw
                rate about a vertical axis is insensitive to translation, so
                these stay at zero, honestly labelled, until an EKF needs them.

NOT MEASURABLE HERE, BUT A HARD PREREQUISITE
===============================================
`LIDAR_ROTATION_SIGN` in fpms_lidar_ros.py (currently -1, marked UNVERIFIED)
decides whether the scan is MIRRORED. A mirror is not a rigid transform: no
value of laser_yaw can undo it. In a feature-rich room a mirrored scan will
not converge, or will converge to something confidently wrong. Run the
left-wall test documented at the top of fpms_lidar_ros.py BEFORE trusting any
map. That file is owned by another agent - report the result, do not edit it.

Also note the double-correction trap: `LIDAR_ZERO_OFFSET_DEG` in
fpms_lidar_ros.py rotates the scan in the same sense `laser_yaw` does. It is
currently 0.0. **Correct mount yaw in exactly one of the two places - this
one.** Applying half in each is how you end up chasing a bias that changes
whenever either file is touched.

USAGE
========
    # defaults (PLACEHOLDERS - the launch says so, loudly)
    ros2 launch nav2/fpms_tf.launch.py

    # with real measurements
    ros2 launch nav2/fpms_tf.launch.py \
        laser_x:=0.052 laser_y:=0.000 laser_z:=0.071 \
        laser_yaw:=-0.0122 measured:=true

    # refuse to start unless the offsets have actually been supplied
    ros2 launch nav2/fpms_tf.launch.py require_measured:=true ...

`require_measured:=true` is the one to put in the mapping run's command line.
It turns "we forgot to pass the offsets" from a map you will trust for weeks
into a launch that fails in two seconds.

ROS_DOMAIN_ID
===============
This rover is hard-wired to ROS_DOMAIN_ID=20 (NAV2_BRIEF.md section 2) and
nowhere else - a node on the wrong domain starts cleanly, publishes/subscribes
successfully, and talks to nobody, which is a much more confusing failure than
a crash. `_enforce_domain_id()` below runs at launch-description build time
(i.e. as soon as `ros2 launch` parses this file, before any node starts) and
refuses to proceed if the environment disagrees, matching the guard already in
fpms_odom_tf.py and deadband_sweep.py.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ============================================================================
# FRAMES
# ============================================================================
BASE_FOOTPRINT = "base_footprint"
BASE_LINK = "base_link"
LASER_FRAME = "laser_frame"
IMU_FRAME = "imu_frame"

# ============================================================================
# base_footprint -> base_link : MEASURED (derived from measured wheel dia.)
# Wheel diameter 70mm is hardware truth (NAV2_BRIEF.md section 2) -> axle
# centreline, and therefore base_link, sits at half that above the ground.
# No rotation: base_link is upright and axis-aligned with base_footprint.
# ============================================================================
BASE_LINK_X_M = 0.0
BASE_LINK_Y_M = 0.0
BASE_LINK_Z_M = 0.035   # = WHEEL_DIAMETER_M / 2, from NAV2_BRIEF hardware truth
BASE_LINK_YAW_RAD = 0.0

# ============================================================================
# !! base_link -> laser_frame - PLACEHOLDER DEFAULTS. UNMEASURED.           !!
# These are the values used when the corresponding launch argument is not
# given. They exist so the tree is CONNECTED and nothing NaNs - they are not
# measurements and the launch banner says so every time it starts.
# See "WHAT THE OPERATOR MUST PHYSICALLY MEASURE" in the module docstring.
# ============================================================================
LASER_X_DEFAULT = 0.0        # MEASURE ME
LASER_Y_DEFAULT = 0.0        # MEASURE ME
LASER_Z_DEFAULT = 0.065      # MEASURE ME (mast guess: 100mm above floor - 35mm base_link rise)
LASER_ROLL_DEFAULT = 0.0     # MEASURE ME (level the scan plane)
LASER_PITCH_DEFAULT = 0.0    # MEASURE ME (level the scan plane - see floor-strike arithmetic)
LASER_YAW_DEFAULT = 0.0      # MEASURE ME (1 deg = ~10.5 mm of position error)

# ============================================================================
# !! base_link -> imu_frame - PLACEHOLDER DEFAULTS. UNMEASURED.             !!
# Zero because the IMU is assumed roughly at body centre. Very likely wrong in
# Z (the ESP32-S3 board the IMU lives on is not at axle height). Harmless
# today: fpms_odom_tf.py integrates /imu gyro-z directly and yaw rate about a
# vertical axis does not care where on the rigid body it is measured. It stops
# being harmless the moment a robot_localization EKF fuses IMU acceleration.
# ============================================================================
IMU_X_DEFAULT = 0.0          # MEASURE ME (only if an EKF is added)
IMU_Y_DEFAULT = 0.0          # MEASURE ME (only if an EKF is added)
IMU_Z_DEFAULT = 0.0          # MEASURE ME (only if an EKF is added)
IMU_ROLL_DEFAULT = 0.0       # MEASURE ME (only if an EKF is added)
IMU_PITCH_DEFAULT = 0.0      # MEASURE ME (only if an EKF is added)
IMU_YAW_DEFAULT = 0.0        # MEASURE ME (only if an EKF is added)

# The exact placeholder tuple, used by require_measured to tell "the operator
# supplied a number that happens to be zero" apart from "the operator supplied
# nothing at all". Only the laser is checked: it is the only frame SLAM and the
# costmaps actually consume.
_LASER_PLACEHOLDER = (
    LASER_X_DEFAULT, LASER_Y_DEFAULT, LASER_Z_DEFAULT,
    LASER_ROLL_DEFAULT, LASER_PITCH_DEFAULT, LASER_YAW_DEFAULT,
)

# 1 degree of mount yaw over this lever arm, in millimetres. R2_COORDINATES.md
# section 4.5. Printed in the banner so the cost of skipping the measurement is
# on screen rather than in a document nobody opens.
_YAW_LEVER_ARM_M = 0.6


def _enforce_domain_id():
    """Refuse to build a launch description on the wrong ROS domain.

    Copied from the same guard in fpms_odom_tf.py / deadband_sweep.py so all
    three fail the same way. This runs when `ros2 launch` parses this file -
    before any static_transform_publisher process starts - so a domain
    mistake is caught immediately instead of producing three nodes that start
    cleanly and are simply never seen by anything else on the rover.
    """
    cur = os.environ.get("ROS_DOMAIN_ID")
    if cur is None:
        os.environ["ROS_DOMAIN_ID"] = "20"
        print("fpms_tf.launch.py: ROS_DOMAIN_ID was unset; defaulting to 20 "
              "(mandatory for this rover)")
    elif cur != "20":
        raise SystemExit(
            "refuse: ROS_DOMAIN_ID=%r in this shell but this rover requires "
            "20. Fix the environment and re-run." % (cur,))


def _static_tf_node(name, parent, child, x, y, z, roll, pitch, yaw):
    """One static_transform_publisher process for one edge of the tree.

    tf2_ros's static_transform_publisher only ever owns a single edge per
    process (there is no fan-out arg for multiple children), so completing
    the 3-edge tree below takes 3 separate Node actions, not 1.
    """
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        # Named args (--x/--y/... /--frame-id/--child-frame-id) are the
        # Humble-recommended form; the old positional
        # "x y z yaw pitch roll frame child" form still works but is
        # deprecated and easy to transpose (roll/pitch/yaw order differs
        # from the named form below) - named args are used deliberately.
        arguments=[
            "--x", str(x), "--y", str(y), "--z", str(z),
            "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
            "--frame-id", parent,
            "--child-frame-id", child,
        ],
        output="screen",
    )


def _f(context, name):
    """Resolve one launch argument to a float, with a legible error."""
    raw = LaunchConfiguration(name).perform(context)
    try:
        return float(raw)
    except (TypeError, ValueError):
        raise SystemExit(
            "refuse: launch argument %s=%r is not a number. Offsets are in "
            "METRES and angles in RADIANS - if you measured 12 degrees, pass "
            "0.2094, not 12." % (name, raw))


def _b(context, name):
    """Resolve one launch argument to a bool the way ros2 launch users type it."""
    raw = str(LaunchConfiguration(name).perform(context)).strip().lower()
    if raw in ("1", "true", "yes", "on"):
        return True
    if raw in ("0", "false", "no", "off"):
        return False
    raise SystemExit(
        "refuse: launch argument %s=%r is not a boolean (true/false)."
        % (name, raw))


def _launch_setup(context, *_args, **_kwargs):
    """Resolve arguments, shout about unmeasured ones, then build the nodes.

    This is an OpaqueFunction rather than plain Node actions because the
    banner has to contain the ACTUAL numbers in use. A warning that says
    "offsets may be placeholders" is ignorable; one that prints
    `laser_yaw = 0.0 rad (PLACEHOLDER, = 0.0 mm of bias you have not
    measured)` is not.
    """
    lx = _f(context, "laser_x")
    ly = _f(context, "laser_y")
    lz = _f(context, "laser_z")
    lroll = _f(context, "laser_roll")
    lpitch = _f(context, "laser_pitch")
    lyaw = _f(context, "laser_yaw")

    ix = _f(context, "imu_x")
    iy = _f(context, "imu_y")
    iz = _f(context, "imu_z")
    iroll = _f(context, "imu_roll")
    ipitch = _f(context, "imu_pitch")
    iyaw = _f(context, "imu_yaw")

    measured = _b(context, "measured")
    require_measured = _b(context, "require_measured")

    laser = (lx, ly, lz, lroll, lpitch, lyaw)
    still_placeholder = laser == _LASER_PLACEHOLDER

    if require_measured and (still_placeholder or not measured):
        raise SystemExit(
            "refuse: require_measured:=true but the base_link -> laser_frame "
            "offsets are still the unmeasured placeholders "
            "(x=%g y=%g z=%g roll=%g pitch=%g yaw=%g, measured:=%s).\n"
            "        Measure them - see the module docstring of "
            "nav2/fpms_tf.launch.py - then pass them as launch arguments and "
            "add measured:=true.\n"
            "        This guard exists because a map built on guessed offsets "
            "looks fine and is wrong, and you will trust it for weeks."
            % (lx, ly, lz, lroll, lpitch, lyaw, measured))

    if still_placeholder or not measured:
        print(
            "\n"
            "################################################################\n"
            "#  fpms_tf.launch.py: LiDAR MOUNT OFFSETS ARE UNMEASURED       #\n"
            "################################################################\n"
            "  base_link -> laser_frame  x=%.4f y=%.4f z=%.4f m\n"
            "                            roll=%.5f pitch=%.5f yaw=%.5f rad\n"
            "  Nobody has put a ruler on this robot. Every map, costmap and\n"
            "  pose produced with these numbers inherits the error.\n"
            "  1 deg of mount yaw = %.1f mm of position error over a %.2f m\n"
            "  lever arm (R2_COORDINATES.md 4.5).\n"
            "  Measure, then pass laser_x/y/z/roll/pitch/yaw and measured:=true.\n"
            "  Use require_measured:=true to make this an error, not a notice.\n"
            "################################################################\n"
            % (lx, ly, lz, lroll, lpitch, lyaw,
               1000.0 * _YAW_LEVER_ARM_M * (3.14159265358979 / 180.0),
               _YAW_LEVER_ARM_M))
    else:
        print("fpms_tf.launch.py: base_link -> laser_frame offsets declared "
              "MEASURED: x=%.4f y=%.4f z=%.4f m, roll=%.5f pitch=%.5f "
              "yaw=%.5f rad" % (lx, ly, lz, lroll, lpitch, lyaw))

    if (ix, iy, iz, iroll, ipitch, iyaw) != (
            IMU_X_DEFAULT, IMU_Y_DEFAULT, IMU_Z_DEFAULT,
            IMU_ROLL_DEFAULT, IMU_PITCH_DEFAULT, IMU_YAW_DEFAULT):
        print("fpms_tf.launch.py: non-default IMU offsets supplied "
              "(x=%.4f y=%.4f z=%.4f roll=%.5f pitch=%.5f yaw=%.5f)"
              % (ix, iy, iz, iroll, ipitch, iyaw))

    print("fpms_tf.launch.py: publishing base_footprint->base_link->"
          "laser_frame and base_link->imu_frame as STATIC transforms. "
          "odom->base_footprint is NOT published here (fpms_odom_tf.py owns "
          "it) and map->odom is NOT published here (slam_toolbox owns it).")

    return [
        _static_tf_node(
            "fpms_tf_base_footprint_to_base_link",
            BASE_FOOTPRINT, BASE_LINK,
            BASE_LINK_X_M, BASE_LINK_Y_M, BASE_LINK_Z_M,
            0.0, 0.0, BASE_LINK_YAW_RAD,
        ),
        _static_tf_node(
            "fpms_tf_base_link_to_laser_frame",
            BASE_LINK, LASER_FRAME,
            lx, ly, lz, lroll, lpitch, lyaw,
        ),
        _static_tf_node(
            "fpms_tf_base_link_to_imu_frame",
            BASE_LINK, IMU_FRAME,
            ix, iy, iz, iroll, ipitch, iyaw,
        ),
    ]


def generate_launch_description():
    _enforce_domain_id()

    declare = [
        DeclareLaunchArgument(
            "laser_x", default_value=str(LASER_X_DEFAULT),
            description="base_link -> laser_frame X in METRES, +forward. "
                        "UNMEASURED PLACEHOLDER by default."),
        DeclareLaunchArgument(
            "laser_y", default_value=str(LASER_Y_DEFAULT),
            description="base_link -> laser_frame Y in METRES, +left. "
                        "UNMEASURED PLACEHOLDER by default."),
        DeclareLaunchArgument(
            "laser_z", default_value=str(LASER_Z_DEFAULT),
            description="base_link -> laser_frame Z in METRES, +up, measured "
                        "from 35 mm above the floor. UNMEASURED PLACEHOLDER."),
        DeclareLaunchArgument(
            "laser_roll", default_value=str(LASER_ROLL_DEFAULT),
            description="LiDAR mount roll in RADIANS. Non-zero tilts the scan "
                        "plane; see the floor-strike arithmetic in the "
                        "module docstring."),
        DeclareLaunchArgument(
            "laser_pitch", default_value=str(LASER_PITCH_DEFAULT),
            description="LiDAR mount pitch in RADIANS. 1-2 degrees of "
                        "downward pitch puts the far half of every scan into "
                        "the floor. UNMEASURED PLACEHOLDER."),
        DeclareLaunchArgument(
            "laser_yaw", default_value=str(LASER_YAW_DEFAULT),
            description="LiDAR mount yaw in RADIANS. THE CRITICAL ONE: 1 "
                        "degree = ~10.5 mm of position error. Correct mount "
                        "yaw HERE, not in fpms_lidar_ros.py's "
                        "LIDAR_ZERO_OFFSET_DEG - never both."),
        DeclareLaunchArgument(
            "imu_x", default_value=str(IMU_X_DEFAULT),
            description="base_link -> imu_frame X in METRES. Only matters if "
                        "a robot_localization EKF is added."),
        DeclareLaunchArgument(
            "imu_y", default_value=str(IMU_Y_DEFAULT),
            description="base_link -> imu_frame Y in METRES."),
        DeclareLaunchArgument(
            "imu_z", default_value=str(IMU_Z_DEFAULT),
            description="base_link -> imu_frame Z in METRES."),
        DeclareLaunchArgument(
            "imu_roll", default_value=str(IMU_ROLL_DEFAULT),
            description="IMU mount roll in RADIANS."),
        DeclareLaunchArgument(
            "imu_pitch", default_value=str(IMU_PITCH_DEFAULT),
            description="IMU mount pitch in RADIANS."),
        DeclareLaunchArgument(
            "imu_yaw", default_value=str(IMU_YAW_DEFAULT),
            description="IMU mount yaw in RADIANS."),
        DeclareLaunchArgument(
            "measured", default_value="false",
            description="Set true ONLY after physically measuring the laser "
                        "offsets. It suppresses the unmeasured banner - it "
                        "does not change any transform."),
        DeclareLaunchArgument(
            "require_measured", default_value="false",
            description="Set true to REFUSE to launch while the laser "
                        "offsets are still the placeholders. Use this for the "
                        "mapping run."),
    ]

    ld = LaunchDescription()
    for a in declare:
        ld.add_action(a)
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
