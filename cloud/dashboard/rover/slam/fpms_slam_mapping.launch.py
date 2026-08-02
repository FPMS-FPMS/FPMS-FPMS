#!/usr/bin/env python3
"""fpms_slam_mapping.launch.py - build a map of the real room, once.

READ cloud/dashboard/rover/NAV2_BRIEF.md AND nav2/TF_TREE.md FIRST.
This file assumes both.

-------------------------------------------------------------------------------
WHAT THIS IS FOR
-------------------------------------------------------------------------------
This is the MAPPING run. The operator drives the rover around the room by
teleop, slam_toolbox builds a pose graph and an occupancy grid from the real
LiDAR, and at the end the graph is SERIALISED to disk. It is run once per room.
Missions afterwards run slam/fpms_slam_localization.launch.py against that
serialised graph.

It replaces the arena-wall-fit approach entirely. That approach assumed a
known 1200 x 1200 mm walled box; the rover is not in one. A live scan sample
returns 350 of 360 bins with min 0.861 m, median 1.983 m and max 6.000 m -
a real, feature-rich room several metres across with at least one sightline
past 6 m. There is more than enough geometry for a correlative scan matcher to
lock onto, and no box to fit.

-------------------------------------------------------------------------------
WHAT MUST ALREADY BE RUNNING - ALL OF IT, IN THIS ORDER
-------------------------------------------------------------------------------
  1. micro-ros-agent            (already a service; NEVER restart it)
  2. fpms_lidar_ros.py          publishing /scan_lidar
  3. fpms_odom_tf.py            publishing /odom and odom -> base_footprint
  4. nav2/fpms_tf.launch.py     publishing base_footprint -> base_link ->
                                laser_frame, WITH MEASURED laser offsets

This launch file deliberately starts NONE of them, and in particular does not
include fpms_tf.launch.py. nav2/TF_TREE.md's rule is "one publisher per edge,
no exceptions"; a launch file that conveniently re-publishes the static
transforms is exactly how an edge acquires a second broadcaster, and the
resulting TF graph looks connected until you notice two publishers winning at
random. Bring the chain up one link at a time and verify each with
`ros2 topic echo` / `tf2_echo` before starting this.

If the TF chain is incomplete, slam_toolbox does not crash. It logs about
failing to compute odom pose and silently produces nothing, which reads as
"SLAM is not working" rather than "TF is broken". Check TF first, always.

-------------------------------------------------------------------------------
THE MOUNT OFFSETS ARE NOT OPTIONAL FOR THIS RUN
-------------------------------------------------------------------------------
Everything downstream inherits the map this run produces. If the LiDAR mount
yaw is wrong by 1 degree, every scan is rotated by 1 degree, which over a
600 mm lever arm is ~10.5 mm of position error (R2_COORDINATES.md 4.5) applied
consistently and in a heading-dependent direction - it does not average out,
it warps the map, and the warp is then baked into every mission that localises
against it. Run step 4 above with `require_measured:=true`.

Two things must be settled before this run, and only one of them is a number
this repo owns:
  * base_link -> laser_frame x/y/z/roll/pitch/yaw  - owned by
    nav2/fpms_tf.launch.py, measure and pass as launch arguments.
  * LIDAR_ROTATION_SIGN in fpms_lidar_ros.py (currently -1, UNVERIFIED) -
    owned by another agent. This decides whether the scan is MIRRORED, and a
    mirror is not a rigid transform: no TF value can undo it. Run the
    left-wall test at the top of that file first. A mirrored scan in a
    feature-rich room will either fail to converge or converge to something
    confidently wrong.

-------------------------------------------------------------------------------
HOW TO DRIVE THE MAPPING RUN
-------------------------------------------------------------------------------
The mapper only gains a graph node after minimum_travel_distance (0.10 m) or
minimum_travel_heading (0.15 rad) of motion, and this chassis moves in ~0.35 s
bursts with full stops (NAV2_BRIEF.md 3b). That is not a problem - it is
close to ideal, because a stationary fix has no motion-latency error at all
(R2_COORDINATES.md 4.5 puts in-motion scan latency at >=16 mm and at-rest at
0 mm). Drive it the way the chassis wants to move:

  * Short bursts, full stop, pause ~1 s. Let 5-10 scans land while stopped.
  * Cover the whole room, and drive along walls as well as across the middle -
    the matcher needs to see the same structure from different angles.
  * RETURN TO THE START and drive a second lap. Loop closure is what removes
    accumulated drift, and it cannot fire on geometry it has only seen once.
  * Watch rviz2 (Map + TF displays). If walls double up, stop: that is drift
    outrunning the matcher, and continuing just bakes it in.

-------------------------------------------------------------------------------
SAVING THE RESULT - DO BOTH
-------------------------------------------------------------------------------
1) Serialise the POSE GRAPH. This is the artefact localisation mode reloads,
   and it preserves the graph, not just the raster:

     ros2 service call /slam_toolbox/serialize_map \\
       slam_toolbox/srv/SerializePoseGraph \\
       "{filename: '<repo>/cloud/dashboard/rover/slam/maps/fpms_room'}"

   Writes fpms_room.posegraph and fpms_room.data. NO extension in `filename`.

2) Save a PGM/YAML raster as well. It is human-viewable, it is what
   nav2_map_server and AMCL consume if the SLAM path ever has to be bypassed,
   and it is the only form in which a reviewer can look at the map and say
   "that is not the shape of the room":

     ros2 run nav2_map_server map_saver_cli \\
       -f <repo>/cloud/dashboard/rover/slam/maps/fpms_room

   Do this while THIS launch is still running - map_saver_cli subscribes to
   /map.

-------------------------------------------------------------------------------
THE ODOMETRY TWIST SIGN - WHY THIS NODE IS SAFE AND WHAT IS NOT
-------------------------------------------------------------------------------
/odom_raw's twist.linear.x is SIGN-INVERTED relative to its own pose
(NAV2_BRIEF.md 3a). slam_toolbox is immune to this: it consumes the TF
odom -> base_footprint and the scan, and subscribes to no odometry message at
all. Nothing in this launch reads a twist.

That immunity does NOT extend to the rest of the stack. See the
"TWIST SIGN" block in nav2/nav2_params_slam.yaml for what must consume /odom
(sign-corrected by fpms_odom_tf.py) and what must never touch /odom_raw.

-------------------------------------------------------------------------------
USAGE
-------------------------------------------------------------------------------
    ros2 launch cloud/dashboard/rover/slam/fpms_slam_mapping.launch.py

    # with an explicit scan topic or a tuned params file
    ros2 launch .../fpms_slam_mapping.launch.py \\
        params_file:=/path/to/mapper_params_mapping.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# The directory this file lives in. Used for the default params path so the
# stack can be launched straight out of the repo checkout with
# `ros2 launch <path>` and no colcon package around it - which is how it will
# actually be run on the Pi during bring-up.
THIS_DIR = os.path.dirname(os.path.realpath(__file__))

DEFAULT_PARAMS = os.path.join(THIS_DIR, "mapper_params_mapping.yaml")

# slam_toolbox's own config files use `slam_toolbox:` as the YAML root key, so
# the node MUST be named slam_toolbox or every parameter in the file is
# silently ignored and the node runs on its compiled-in defaults - which
# include scan_topic:/scan, the DEAD topic. Named explicitly rather than relying
# on the executable's default node name.
NODE_NAME = "slam_toolbox"


def _enforce_domain_id():
    """Refuse to build a launch description on the wrong ROS domain.

    Same guard as fpms_odom_tf.py, deadband_sweep.py and
    nav2/fpms_tf.launch.py, so all of them fail the same way. A node on the
    wrong domain starts cleanly, subscribes successfully, and never receives a
    scan - which looks exactly like a broken LiDAR bridge.
    """
    cur = os.environ.get("ROS_DOMAIN_ID")
    if cur is None:
        os.environ["ROS_DOMAIN_ID"] = "20"
        print("fpms_slam_mapping.launch.py: ROS_DOMAIN_ID was unset; "
              "defaulting to 20 (mandatory for this rover)")
    elif cur != "20":
        raise SystemExit(
            "refuse: ROS_DOMAIN_ID=%r in this shell but this rover requires "
            "20. Fix the environment and re-run." % (cur,))


def generate_launch_description():
    _enforce_domain_id()

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    declare_args = [
        DeclareLaunchArgument(
            "params_file", default_value=DEFAULT_PARAMS,
            description="slam_toolbox mapping parameters. Defaults to the file "
                        "beside this launch file - NOT slam_toolbox's own "
                        "mapper_params_online_async.yaml, whose defaults are "
                        "sized for a robot crossing a warehouse and whose "
                        "scan_topic is the dead /scan.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="false on real hardware. fpms_odom_tf.py stamps with "
                        "the Pi's ROS clock; true here would make every "
                        "transform look decades stale and slam_toolbox would "
                        "drop every scan.",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="ROS log level for slam_toolbox. 'debug' prints every "
                        "scan match, which is genuinely useful when the "
                        "matcher is not converging and unreadable otherwise.",
        ),
    ]

    # NAV2_BRIEF.md 2: ROS_DOMAIN_ID 20 is MANDATORY and the drive board
    # firmware stores it itself. Setting it here as well as in the guard above
    # means a forgotten `export` cannot produce a silently deaf mapper.
    set_domain = SetEnvironmentVariable("ROS_DOMAIN_ID", "20")

    # Unbuffered stdout so a crash traceback reaches the journal instead of
    # dying in a pipe buffer. Matches how the existing fpms-* services log.
    set_stdout = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "0")

    slam = Node(
        package="slam_toolbox",
        # ASYNC, not sync. The synchronous node processes every scan and blocks
        # the queue if it falls behind; ours arrive over MQTT with p90 179 ms
        # of jitter, so a queue that must be drained in order is a queue that
        # will stall. The async node drops what it cannot keep up with and
        # keeps the pose current, which is the right failure mode when the
        # transport is the unreliable part.
        executable="async_slam_toolbox_node",
        name=NODE_NAME,
        output="screen",
        # No respawn. If the mapper dies mid-run the map is compromised and the
        # operator needs to know and restart deliberately; a silent respawn
        # would produce a second, disjoint map in the same session.
        respawn=False,
        parameters=[
            params_file,
            # Overrides the file. Declared with an explicit value_type because
            # a LaunchConfiguration resolves to the STRING "false", and
            # use_sim_time is a bool - passing the string makes the node reject
            # the parameter at startup.
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        # Listed explicitly. These are the transforms that break first if a
        # namespace is ever introduced, and a SLAM node quietly talking to the
        # wrong /tf is the single most confusing failure in this stack.
        remappings=[("/tf", "/tf"), ("/tf_static", "/tf_static")],
        arguments=["--ros-args", "--log-level", log_level],
    )

    print(
        "fpms_slam_mapping.launch.py: starting slam_toolbox "
        "async_slam_toolbox_node in MAPPING mode.\n"
        "  PREREQUISITES (this file starts none of them):\n"
        "    /scan_lidar          fpms_lidar_ros.py\n"
        "    /odom + odom->base_footprint   fpms_odom_tf.py\n"
        "    base_footprint->base_link->laser_frame   "
        "nav2/fpms_tf.launch.py require_measured:=true\n"
        "  slam_toolbox will publish map->odom. Nothing else may publish it.\n"
        "  When the room is covered, SERIALISE the graph - see this file's "
        "docstring. An unserialised mapping run is a map you will have to "
        "drive again.")

    ld = LaunchDescription()
    ld.add_action(set_domain)
    ld.add_action(set_stdout)
    for a in declare_args:
        ld.add_action(a)
    ld.add_action(slam)
    return ld
