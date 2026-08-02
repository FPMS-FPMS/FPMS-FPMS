#!/usr/bin/env python3
"""fpms_slam_localization.launch.py - localise against an already-built map.

READ cloud/dashboard/rover/NAV2_BRIEF.md, nav2/TF_TREE.md and
slam/fpms_slam_mapping.launch.py FIRST. This file assumes all three.

-------------------------------------------------------------------------------
WHAT THIS IS FOR
-------------------------------------------------------------------------------
This is what runs for every MISSION. It loads the pose graph serialised by the
mapping run, matches live /scan_lidar against it, and publishes map -> odom.
It does not extend or modify the map.

Drive once to map (fpms_slam_mapping.launch.py). Run this for everything after.

-------------------------------------------------------------------------------
WHAT MUST ALREADY BE RUNNING
-------------------------------------------------------------------------------
Identical to the mapping run, and for the same reasons:

  1. micro-ros-agent            (already a service; NEVER restart it)
  2. fpms_lidar_ros.py          publishing /scan_lidar
  3. fpms_odom_tf.py            publishing /odom and odom -> base_footprint
  4. nav2/fpms_tf.launch.py     base_footprint -> base_link -> laser_frame,
                                WITH THE SAME laser offsets used for mapping

Point 4 is not a formality. The map was built with a particular
base_link -> laser_frame transform baked into every scan placement. Localising
with a different one applies that difference as a constant pose bias against a
map that cannot argue back. If the offsets are re-measured, the correct
response is to RE-MAP, not to localise against the old map with new numbers.

-------------------------------------------------------------------------------
THE START POSE IS A REAL INPUT
-------------------------------------------------------------------------------
slam_toolbox's localization mode does LOCAL correlative matching. It refines a
pose; it does not solve the global kidnapped-robot problem.
correlation_search_space_dimension is 0.3 m, so a starting guess more than
about 0.15 m from the truth is outside the matcher's search window - it will
lock onto a wrong local minimum or fail to converge, and in a room with
repeated structure a wrong lock can look confident.

So one of these must be true before any motion is commanded:
  (a) the rover is physically on the same floor mark the mapping run started
      from, and start_pose is left at 0/0/0; or
  (b) start_x / start_y / start_yaw are passed with the rover's real pose in
      MAP coordinates; or
  (c) an operator publishes a 2D Pose Estimate in rviz2 on /initialpose after
      startup and confirms the scan overlays the map before anything moves.

There is no fourth option where the rover works it out on its own.

-------------------------------------------------------------------------------
THE ODOMETRY TWIST SIGN
-------------------------------------------------------------------------------
/odom_raw's twist.linear.x is SIGN-INVERTED relative to its own pose
(NAV2_BRIEF.md 3a). slam_toolbox reads the TF odom -> base_footprint and the
scan and subscribes to no odometry message, so it is immune. The Nav2 stack is
not - see the "TWIST SIGN" block in nav2/nav2_params_slam.yaml.

-------------------------------------------------------------------------------
USAGE
-------------------------------------------------------------------------------
    # map at slam/maps/fpms_room.{posegraph,data}, rover on the start mark
    ros2 launch cloud/dashboard/rover/slam/fpms_slam_localization.launch.py

    # a different map, and a known non-origin start pose
    ros2 launch .../fpms_slam_localization.launch.py \\
        map_name:=/home/ubuntu/maps/lab_room \\
        start_x:=1.20 start_y:=-0.35 start_yaw:=1.5708

`map_name` carries NO file extension - slam_toolbox appends .posegraph and
.data itself. This launch file checks both exist before starting anything,
because slam_toolbox's own failure for a missing map is a log line followed by
a node that sits there publishing nothing.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


THIS_DIR = os.path.dirname(os.path.realpath(__file__))

DEFAULT_PARAMS = os.path.join(THIS_DIR, "mapper_params_localization.yaml")

# No extension. slam_toolbox appends .posegraph and .data.
DEFAULT_MAP = os.path.join(THIS_DIR, "maps", "fpms_room")

# The YAML root key in mapper_params_localization.yaml is `slam_toolbox:`, so
# the node must carry that name or every parameter is silently ignored and the
# node falls back to compiled-in defaults - including mode:mapping, which would
# quietly start building a NEW map instead of localising in the old one.
NODE_NAME = "slam_toolbox"


def _enforce_domain_id():
    """Refuse to build a launch description on the wrong ROS domain."""
    cur = os.environ.get("ROS_DOMAIN_ID")
    if cur is None:
        os.environ["ROS_DOMAIN_ID"] = "20"
        print("fpms_slam_localization.launch.py: ROS_DOMAIN_ID was unset; "
              "defaulting to 20 (mandatory for this rover)")
    elif cur != "20":
        raise SystemExit(
            "refuse: ROS_DOMAIN_ID=%r in this shell but this rover requires "
            "20. Fix the environment and re-run." % (cur,))


def _f(context, name):
    """Resolve one launch argument to a float, with a legible error."""
    raw = LaunchConfiguration(name).perform(context)
    try:
        return float(raw)
    except (TypeError, ValueError):
        raise SystemExit(
            "refuse: launch argument %s=%r is not a number. start_x/start_y "
            "are METRES and start_yaw is RADIANS - if you meant 90 degrees, "
            "pass 1.5708." % (name, raw))


def _launch_setup(context, *_args, **_kwargs):
    """Check the map exists, build the start pose, then start the node.

    An OpaqueFunction rather than plain substitutions because map_start_pose is
    a double ARRAY parameter. There is no way to build one from a
    LaunchConfiguration without resolving the values first, and resolving them
    also lets the map be checked for existence before a node is spawned to fail
    on it.
    """
    params_file = LaunchConfiguration("params_file").perform(context)
    map_name = LaunchConfiguration("map_name").perform(context)
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")

    sx, sy, syaw = (_f(context, "start_x"), _f(context, "start_y"),
                    _f(context, "start_yaw"))

    # slam_toolbox appends these two suffixes. Checking here turns "the node
    # started and localises nothing" into a one-line error naming the file.
    missing = [map_name + ext for ext in (".posegraph", ".data")
               if not os.path.isfile(map_name + ext)]
    if missing:
        raise SystemExit(
            "refuse: serialised map not found - missing %s\n"
            "        map_name must have NO file extension; slam_toolbox adds "
            ".posegraph and .data.\n"
            "        If the room has not been mapped yet, run\n"
            "          ros2 launch %s\n"
            "        drive the room, then\n"
            "          ros2 service call /slam_toolbox/serialize_map "
            "slam_toolbox/srv/SerializePoseGraph \"{filename: '%s'}\""
            % (", ".join(missing),
               os.path.join(THIS_DIR, "fpms_slam_mapping.launch.py"),
               map_name))

    if not os.path.isfile(params_file):
        raise SystemExit("refuse: params_file %r does not exist." % params_file)

    if (sx, sy, syaw) == (0.0, 0.0, 0.0):
        print(
            "\n"
            "----------------------------------------------------------------\n"
            " fpms_slam_localization: start pose is 0.0, 0.0, 0.0\n"
            " That asserts the rover is EXACTLY where the mapping run began.\n"
            " The scan matcher's capture range is about +/-0.15 m; a worse\n"
            " guess than that will lock onto the wrong place, confidently.\n"
            " Either put the rover on the mapping start mark, pass\n"
            " start_x/start_y/start_yaw, or set the pose in rviz2 on\n"
            " /initialpose BEFORE commanding any motion.\n"
            "----------------------------------------------------------------\n")
    else:
        print("fpms_slam_localization: start pose x=%.3f y=%.3f yaw=%.4f rad"
              % (sx, sy, syaw))

    print("fpms_slam_localization: loading %s.{posegraph,data}" % map_name)

    slam = Node(
        package="slam_toolbox",
        # The dedicated localization executable, not async_slam_toolbox_node
        # with mode:localization. They are different nodes: this one runs the
        # elastic pose-graph localiser with a bounded rolling scan buffer, so
        # the graph does not grow for the length of a mission.
        executable="localization_slam_toolbox_node",
        name=NODE_NAME,
        output="screen",
        respawn=False,
        parameters=[
            params_file,
            {
                # Overrides the value in the params file so a public repo never
                # carries an absolute path to one machine's home directory.
                "map_file_name": map_name,
                "map_start_pose": [sx, sy, syaw],
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            },
        ],
        remappings=[("/tf", "/tf"), ("/tf_static", "/tf_static")],
        arguments=["--ros-args", "--log-level", log_level],
    )
    return [slam]


def generate_launch_description():
    _enforce_domain_id()

    declare_args = [
        DeclareLaunchArgument(
            "params_file", default_value=DEFAULT_PARAMS,
            description="slam_toolbox localization parameters. Defaults to the "
                        "file beside this launch file.",
        ),
        DeclareLaunchArgument(
            "map_name", default_value=DEFAULT_MAP,
            description="Serialised pose graph, WITHOUT extension. "
                        "slam_toolbox appends .posegraph and .data. Both must "
                        "exist or this launch refuses to start.",
        ),
        DeclareLaunchArgument(
            "start_x", default_value="0.0",
            description="Rover X in MAP metres at startup. See the start-pose "
                        "block in this file's docstring - the matcher's "
                        "capture range is about +/-0.15 m.",
        ),
        DeclareLaunchArgument(
            "start_y", default_value="0.0",
            description="Rover Y in MAP metres at startup.",
        ),
        DeclareLaunchArgument(
            "start_yaw", default_value="0.0",
            description="Rover yaw in RADIANS at startup. 90 degrees is "
                        "1.5708, not 90.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="false on real hardware.",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="ROS log level for slam_toolbox.",
        ),
    ]

    set_domain = SetEnvironmentVariable("ROS_DOMAIN_ID", "20")
    set_stdout = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "0")

    ld = LaunchDescription()
    ld.add_action(set_domain)
    ld.add_action(set_stdout)
    for a in declare_args:
        ld.add_action(a)
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
