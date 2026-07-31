#!/usr/bin/env python3
"""fpms_nav2.launch.py - Nav2 bringup for the FPMS rover (ROS 2 Humble).

READ cloud/dashboard/rover/NAV2_BRIEF.md FIRST. This file assumes it.

This deliberately does NOT include nav2_bringup's own launch files. Those pull
in nav2_params.yaml from the nav2_bringup share directory, a velocity smoother,
and a composed container - and this rover needs a different answer to all three
(see the "VELOCITY SMOOTHER - DELIBERATELY ABSENT" block in nav2_params.yaml).
Composing the nodes into one container would also collapse eight separate
journal streams into one, and NAV2_BRIEF.md section 7 is explicit that this
stack has to be brought up and verified one link at a time. Separate processes
are worth the extra RAM here.

-------------------------------------------------------------------------------
WHAT THIS LAUNCHES
-------------------------------------------------------------------------------
  map_server          serves arena_map.pgm on /map
  amcl                map -> odom
  controller_server   RegulatedPurePursuit -> cmd_vel
  planner_server      NavFn
  behavior_server     spin / backup / drive_on_heading / wait
  bt_navigator        NavigateToPose, NavigateThroughPoses
  waypoint_follower   FollowWaypoints (the three-zone mission)
  two lifecycle managers (localisation, navigation)

-------------------------------------------------------------------------------
WHAT THIS DOES *NOT* LAUNCH, AND MUST ALREADY BE RUNNING
-------------------------------------------------------------------------------
  the LiDAR -> ROS bridge          publishing sensor_msgs/LaserScan
  fpms_odom_tf.py                  /odom and odom -> base_footprint TF
  the cmd_vel unit shim            /cmd_vel_nav -> /cmd_vel  (see below)

Nav2 will start happily without any of them and then fail in ways that point at
the wrong component - a missing scan reads as "AMCL is not converging", a
missing TF reads as a tf2 extrapolation error inside the controller. Run the
prerequisites checklist in README.md before this file. It exists because these
failures are genuinely hard to diagnose from the Nav2 side.

-------------------------------------------------------------------------------
WHY cmd_vel IS REMAPPED AWAY FROM /cmd_vel BY DEFAULT
-------------------------------------------------------------------------------
NAV2_BRIEF.md section 3b: the board takes linear.x "with no gain" but commanding
0.10 produces roughly 0.61 m/s actual - about 6x. Nav2's velocities are true SI
m/s because that is the frame its costmaps and goal checks live in, so something
has to divide by ~6 before the board sees it.

That something is a shim node, and it is not this launch file's job. What IS
this launch file's job is making sure that when the shim is missing, the rover
STAYS STILL rather than executing a 6x command. Hence the default
cmd_vel_topic of /cmd_vel_nav: nothing is subscribed, nothing moves, and the
mistake is visible in `ros2 topic info` instead of at 1.1 m/s across a 1.2 m
arena. Point this at /cmd_vel only once the shim is verified.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


# The directory this file lives in. Used for the default params/map paths so the
# stack can be launched straight out of the repo checkout with `ros2 launch
# <path>` and no colcon package around it - which is how it will actually be run
# on the Pi during bring-up.
THIS_DIR = os.path.dirname(os.path.realpath(__file__))

# Order matters: localisation must reach ACTIVE before navigation, because
# controller_server's costmap blocks waiting for map -> odom.
LOCALIZATION_NODES = ["map_server", "amcl"]
NAVIGATION_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
]


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    use_localization = LaunchConfiguration("use_localization")

    declare_args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(THIS_DIR, "nav2_params.yaml"),
            description="Nav2 parameter file. Defaults to the one beside this "
                        "launch file - NOT nav2_bringup's, whose defaults are "
                        "sized for rooms and make this 1.2 m arena impassable.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(THIS_DIR, "arena_map.yaml"),
            description="Occupancy grid for the arena. See README.md for how "
                        "to generate arena_map.pgm.",
        ),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="Have the lifecycle managers configure+activate the "
                        "stack automatically.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="false on real hardware. fpms_odom_tf.py stamps with "
                        "the Pi's ROS clock; true here would make every "
                        "transform look decades stale.",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info",
            description="Per-node ROS log level. Use 'debug' on the node you "
                        "are diagnosing, never on all of them - a 20 Hz "
                        "controller at debug floods the journal.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value="/cmd_vel_nav",
            description="Where Nav2 publishes velocity. DEFAULTS TO A TOPIC "
                        "THE BOARD DOES NOT LISTEN TO, on purpose - the unit "
                        "shim must sit between Nav2 and /cmd_vel. See the "
                        "module docstring and README.md.",
        ),
        DeclareLaunchArgument(
            "use_localization", default_value="true",
            description="Set false to run planner+controller on odom alone "
                        "(no map, no AMCL). Useful while the LiDAR bridge is "
                        "still being built - the stack will then dead-reckon, "
                        "with all the drift that implies.",
        ),
    ]

    # NAV2_BRIEF.md section 2: ROS_DOMAIN_ID 20 is MANDATORY on every ROS call,
    # and the drive board firmware stores it itself - a node on the default
    # domain 0 simply never sees /odom_raw or reaches /cmd_vel, with no error
    # anywhere. Setting it here rather than trusting the operator's shell means
    # a forgotten `export` cannot produce a silently deaf navigation stack.
    set_domain = SetEnvironmentVariable("ROS_DOMAIN_ID", "20")

    # Unbuffered stdout, so a crash traceback reaches the journal instead of
    # dying in a pipe buffer. Matches how the existing fpms-* services log.
    set_stdout = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "0")

    # yaml_filename is injected here rather than hard-coded in nav2_params.yaml
    # so that the params file cannot carry a stale absolute path to somebody
    # else's arena. convert_types=True lets "true"/"false" arguments land as
    # real booleans rather than strings, which Nav2 rejects.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
        },
        convert_types=True,
    )

    # Applied to every Nav2 node. /tf and /tf_static are listed explicitly
    # because they are the transforms that break first if a namespace is ever
    # introduced, and because a Nav2 node quietly talking to the wrong /tf is
    # the single most confusing failure in this stack.
    common_remaps = [("/tf", "/tf"), ("/tf_static", "/tf_static")]

    # Only controller_server and behavior_server emit velocity. Both must be
    # redirected or a recovery spin would bypass the unit shim and hit the board
    # at ~6x - the exact hazard the shim exists to prevent.
    drive_remaps = common_remaps + [("cmd_vel", cmd_vel_topic)]

    arguments = ["--ros-args", "--log-level", log_level]

    localization = GroupAction(
        condition=IfCondition(use_localization),
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=False,
                parameters=[configured_params],
                remappings=common_remaps,
                arguments=arguments,
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                respawn=False,
                parameters=[configured_params],
                remappings=common_remaps,
                arguments=arguments,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": LOCALIZATION_NODES,
                    # Longer than the Nav2 default. AMCL's first scan callback
                    # has to wait for the LiDAR bridge, and a bridge that
                    # reconnects to the serial device can take several seconds.
                    "bond_timeout": 10.0,
                }],
            ),
        ],
    )

    navigation = GroupAction(actions=[
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            respawn=False,
            parameters=[configured_params],
            remappings=drive_remaps,
            arguments=arguments,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            respawn=False,
            parameters=[configured_params],
            remappings=common_remaps,
            arguments=arguments,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            respawn=False,
            parameters=[configured_params],
            remappings=drive_remaps,
            arguments=arguments,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            respawn=False,
            parameters=[configured_params],
            remappings=common_remaps,
            arguments=arguments,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            respawn=False,
            parameters=[configured_params],
            remappings=common_remaps,
            arguments=arguments,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": NAVIGATION_NODES,
                # This manager is declared AFTER the localisation one so it
                # activates second. That order is load-bearing: without
                # map -> odom the global costmap never finishes activating, the
                # manager times out on controller_server, and the error blames
                # the controller for what is really a missing transform.
                "bond_timeout": 10.0,
            }],
        ),
    ])

    ld = LaunchDescription()
    ld.add_action(set_domain)
    ld.add_action(set_stdout)
    for a in declare_args:
        ld.add_action(a)
    ld.add_action(localization)
    ld.add_action(navigation)
    return ld
