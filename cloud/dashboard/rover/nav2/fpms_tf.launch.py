#!/usr/bin/env python3
"""
fpms_tf.launch.py - static TF publishers that complete the FPMS Nav2 tree.

WHY THIS FILE EXISTS
======================
Nav2 needs a connected TF tree from `odom` down to every sensor frame. Only
one edge in that tree changes at runtime:

    odom -> base_footprint      DYNAMIC, published by fpms_odom_tf.py.

Everything below `base_footprint` is a fixed mechanical offset on this
chassis - the LiDAR and IMU do not move relative to the body - so those edges
are STATIC transforms, which is exactly what this launch file publishes:

    base_footprint -> base_link
    base_link      -> laser_frame
    base_link      -> imu_frame

This file does NOT publish odom -> base_footprint. Do not add it here: that
edge already has an owner (fpms_odom_tf.py) and a second broadcaster for the
same edge is a textbook way to get two disagreeing transforms fighting each
other in tf2, which manifests as a TF tree that looks connected right up until
you look closely at timestamps and one of the two publishers "wins" at random.

REQUIRED COMPANION CHANGE - READ BEFORE LAUNCHING THIS FILE
================================================================
fpms_odom_tf.py ALSO ships a static broadcaster for base_footprint ->
laser_frame directly (its `PUBLISH_LASER_STATIC_TF` flag, on by default). That
predates base_link existing in this tree at all. Running both that broadcaster
and this launch file at once gives `laser_frame` two different parents
(base_footprint from one node, base_link from this one) - not a warning, an
actively broken, non-tree TF graph. fpms_odom_tf.py's own comment anticipated
exactly this ("Set PUBLISH_LASER_STATIC_TF=False if a URDF / robot_state_
publisher is ever introduced, so this node does not fight it for ownership of
the transform") - this launch file IS that introduction, so
`PUBLISH_LASER_STATIC_TF` has been set to False in fpms_odom_tf.py alongside
adding this file. If you ever see `laser_frame` with two parents in
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

LIDAR AND IMU MOUNT OFFSETS - UNMEASURED, DO NOT TRUST, DO NOT SHIP ON
============================================================================
Both `laser_frame` and `imu_frame` are placeholders. NOBODY HAS PUT A RULER ON
THIS ROBOT YET. The numbers below are plausible-looking guesses (LiDAR roughly
centred and up on a mast, IMU roughly at body centre) chosen so the tree is
CONNECTED and Nav2 does not crash - they are not a substitute for measurement.

    A wrong laser offset makes obstacles appear in the wrong place in the
    costmap. This is the single most common reason Nav2 "refuses to plan" or
    plans through what should be a wall: not a planner bug, a TF bug that
    looks like a planner bug.

Before trusting ANY map, costmap or AMCL pose produced with this launch file:
  1. Physically measure, with a ruler/calipers, from the base_link point
     defined above (35mm above ground, centred over the drive axle) to the
     LiDAR's optical centre and to the IMU chip, in the same x-forward /
     y-left / z-up body convention used below.
  2. Measure the LiDAR's mounting YAW - if its zero-degree ray does not point
     exactly out the nose, every scan is rotated by that error.
  3. Replace the constants below. Nothing else in this file needs to change.

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
# !! base_link -> laser_frame - PLACEHOLDER. UNMEASURED. DO NOT TRUST.      !!
# !! MUST BE MEASURED ON THE PHYSICAL ROBOT WITH A RULER BEFORE ANY MAP OR !!
# !! COSTMAP BUILT WITH THIS TRANSFORM IS TRUSTED.                         !!
#
# x forward(+)/back(-), y left(+)/right(-), z up(+) from base_link (which is
# already 35mm off the ground - see BASE_LINK_Z_M above). yaw is the mount
# rotation: if the LiDAR's zero-degree ray does not point straight out the
# nose, every obstacle Nav2 sees is rotated by exactly this error.
#
# These numbers are NOT the same numbers that used to live in fpms_odom_tf.py
# (LASER_X/Y/Z_OFFSET_M there) - those were measured relative to
# base_footprint (ground level); these are relative to base_link (35mm up).
# Do not copy one set into the other without adjusting Z by BASE_LINK_Z_M.
# ============================================================================
LASER_X_OFFSET_M = 0.0        # MEASURE ME
LASER_Y_OFFSET_M = 0.0        # MEASURE ME
LASER_Z_OFFSET_M = 0.065      # MEASURE ME (placeholder: mast guess, 100mm above ground - 35mm base_link rise)
LASER_YAW_RAD = 0.0           # MEASURE ME

# ============================================================================
# !! base_link -> imu_frame - PLACEHOLDER. UNMEASURED. DO NOT TRUST.        !!
# !! MUST BE MEASURED ON THE PHYSICAL ROBOT BEFORE ANY HEADING DERIVED     !!
# !! THROUGH THIS FRAME IS TRUSTED FOR ANYTHING BEYOND THE GYRO-Z          !!
# !! INTEGRATION fpms_odom_tf.py ALREADY DOES DIRECTLY OFF /imu.          !!
#
# Best guess only: IMU is assumed roughly at the chassis/body centre, i.e.
# co-located with base_link (zero offset, zero rotation). This is very likely
# wrong in Z at minimum (the ESP32-S3 drive board the IMU lives on is not at
# axle height) - it is set to zero purely so the tree is connected and
# nothing NaNs, not because zero is a real measurement.
# ============================================================================
IMU_X_OFFSET_M = 0.0          # MEASURE ME
IMU_Y_OFFSET_M = 0.0          # MEASURE ME
IMU_Z_OFFSET_M = 0.0          # MEASURE ME
IMU_ROLL_RAD = 0.0            # MEASURE ME
IMU_PITCH_RAD = 0.0           # MEASURE ME
IMU_YAW_RAD = 0.0             # MEASURE ME


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


def generate_launch_description():
    _enforce_domain_id()

    footprint_to_base = _static_tf_node(
        "fpms_tf_base_footprint_to_base_link",
        BASE_FOOTPRINT, BASE_LINK,
        BASE_LINK_X_M, BASE_LINK_Y_M, BASE_LINK_Z_M,
        0.0, 0.0, BASE_LINK_YAW_RAD,
    )

    base_to_laser = _static_tf_node(
        "fpms_tf_base_link_to_laser_frame",
        BASE_LINK, LASER_FRAME,
        LASER_X_OFFSET_M, LASER_Y_OFFSET_M, LASER_Z_OFFSET_M,
        0.0, 0.0, LASER_YAW_RAD,
    )

    base_to_imu = _static_tf_node(
        "fpms_tf_base_link_to_imu_frame",
        BASE_LINK, IMU_FRAME,
        IMU_X_OFFSET_M, IMU_Y_OFFSET_M, IMU_Z_OFFSET_M,
        IMU_ROLL_RAD, IMU_PITCH_RAD, IMU_YAW_RAD,
    )

    print("fpms_tf.launch.py: publishing base_footprint->base_link->"
          "laser_frame and base_link->imu_frame as STATIC transforms. "
          "laser/imu offsets are UNMEASURED PLACEHOLDERS - see this file's "
          "module docstring before trusting any map. odom->base_footprint "
          "is NOT published here (fpms_odom_tf.py owns it).")

    return LaunchDescription([footprint_to_base, base_to_laser, base_to_imu])
