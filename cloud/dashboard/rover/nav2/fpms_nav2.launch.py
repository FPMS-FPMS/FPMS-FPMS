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
LOCALISATION MODES - the argument that matters most
-------------------------------------------------------------------------------
`localization_mode` selects who publishes map -> odom. Exactly one thing may:
TF_TREE.md's "one publisher per edge, no exceptions" applies to this edge as
much as to any other.

  slam  (DEFAULT)  slam_toolbox owns map -> odom AND publishes /map. This
                   launch file starts NEITHER map_server NOR amcl - they are
                   started by slam/fpms_slam_localization.launch.py's node
                   instead, which must ALREADY BE RUNNING. Also loads
                   nav2_params_slam.yaml on top of nav2_params.yaml.
                   This is the real-localisation path: the rover localises off
                   the room's own geometry, against a map it built.

  amcl             The pre-SLAM path: map_server serves a static map file and
                   nav2_amcl owns map -> odom. Kept because AMCL is already
                   configured and is a genuine robustness fallback, but
                   R2_COORDINATES.md 4.4 puts published AMCL accuracy at
                   ~8.5 cm RMSE in an empty environment and ~33.7 cm in a
                   cluttered one, so it is not the precision path.

  none             No map, no map -> odom producer, costmaps on odom alone.
                   Dead reckoning with all the unbounded drift that implies
                   (REP-105: pose in odom "can drift over time, without any
                   bounds"). Useful only for bringing up the controller in
                   isolation. If you use this, set the costmaps' global_frame
                   to odom as well or every lifecycle node will block forever
                   waiting for a transform nobody is publishing.

-------------------------------------------------------------------------------
WHAT THIS DOES *NOT* LAUNCH, AND MUST ALREADY BE RUNNING
-------------------------------------------------------------------------------
  the LiDAR -> ROS bridge          fpms_lidar_ros.py, publishing /scan_lidar
  fpms_odom_tf.py                  /odom and odom -> base_footprint TF
  nav2/fpms_tf.launch.py           base_footprint -> base_link -> laser_frame
  slam/fpms_slam_localization...   map -> odom   (localization_mode:=slam only)
  the cmd_vel unit shim            /cmd_vel_nav -> /cmd_vel  (see below)

Nav2 will start happily without any of them and then fail in ways that point at
the wrong component - a missing scan reads as "the localiser is not
converging", a missing TF reads as a tf2 extrapolation error inside the
controller. Verify each link with `ros2 topic echo` / `tf2_echo` before this
file. It exists because these failures are genuinely hard to diagnose from the
Nav2 side.

-------------------------------------------------------------------------------
ODOMETRY: /odom, NEVER /odom_raw
-------------------------------------------------------------------------------
NAV2_BRIEF.md 3a: /odom_raw's twist.linear.x is sign-inverted relative to its
own pose. fpms_odom_tf.py is the single point of correction and republishes on
/odom. Humble's controller_server and behavior_server hard-code their
OdomSmoother onto the RELATIVE topic `odom`, which happens to resolve to /odom
in the root namespace - correct by accident. `odom_remap` below makes it
correct by declaration, so that adding a namespace one day cannot silently
point them at nothing. The full argument is in the TWIST SIGN block at the top
of nav2_params_slam.yaml.

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
mistake is visible in `ros2 topic info` instead of at 1.1 m/s across a room.
Point this at /cmd_vel only once the shim is verified.
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

VALID_MODES = ("slam", "amcl", "none")


def _mode(context):
    """Resolve and validate localization_mode, failing loudly on a typo.

    A misspelled mode must not silently fall through to "no localisation" -
    that is a rover navigating on dead reckoning while the operator believes it
    is localised, which is the single worst failure this stack can have.
    """
    raw = LaunchConfiguration("localization_mode").perform(context).strip().lower()
    if raw not in VALID_MODES:
        raise SystemExit(
            "refuse: localization_mode=%r is not one of %s. See this file's "
            "docstring." % (raw, ", ".join(VALID_MODES)))
    return raw


def _launch_setup(context, *_args, **_kwargs):
    params_file = LaunchConfiguration("params_file")
    overlay_file = LaunchConfiguration("overlay_params_file").perform(context)
    map_yaml = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    mode = _mode(context)

    # yaml_filename is injected here rather than hard-coded in nav2_params.yaml
    # so that the params file cannot carry a stale absolute path to somebody
    # else's map. convert_types=True lets "true"/"false" arguments land as real
    # booleans rather than strings, which Nav2 rejects.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
        },
        convert_types=True,
    )

    # ROS 2 merges parameter files in list order and LATER FILES WIN, so the
    # overlay is appended, never prepended. It carries only the values that
    # change when the world is a real room rather than a 1.2 m arena; see its
    # own header. Empty string means "no overlay", which is what the amcl and
    # none modes want.
    param_files = [configured_params]
    if mode == "slam":
        if not overlay_file:
            overlay_file = os.path.join(THIS_DIR, "nav2_params_slam.yaml")
        if not os.path.isfile(overlay_file):
            raise SystemExit(
                "refuse: overlay_params_file %r does not exist." % overlay_file)
        param_files.append(overlay_file)
    elif overlay_file:
        param_files.append(overlay_file)

    # Applied to every Nav2 node. /tf and /tf_static are listed explicitly
    # because they are the transforms that break first if a namespace is ever
    # introduced, and because a Nav2 node quietly talking to the wrong /tf is
    # the single most confusing failure in this stack.
    common_remaps = [("/tf", "/tf"), ("/tf_static", "/tf_static")]

    # See "ODOMETRY" in the module docstring. This is a no-op in the root
    # namespace and exists to make that no-op explicit.
    odom_remap = [("odom", "/odom")]

    # Only controller_server and behavior_server emit velocity. Both must be
    # redirected or a recovery spin would bypass the unit shim and hit the board
    # at ~6x - the exact hazard the shim exists to prevent. Both also consume
    # odometry, hence odom_remap.
    drive_remaps = common_remaps + odom_remap + [("cmd_vel", cmd_vel_topic)]

    arguments = ["--ros-args", "--log-level", log_level]

    actions = []

    if mode == "amcl":
        actions += [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=False,
                parameters=param_files,
                remappings=common_remaps,
                arguments=arguments,
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                respawn=False,
                parameters=param_files,
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
        ]

    actions += [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            respawn=False,
            parameters=param_files,
            remappings=drive_remaps,
            arguments=arguments,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            respawn=False,
            parameters=param_files,
            remappings=common_remaps,
            arguments=arguments,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            respawn=False,
            parameters=param_files,
            remappings=drive_remaps,
            arguments=arguments,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            respawn=False,
            parameters=param_files,
            remappings=common_remaps + odom_remap,
            arguments=arguments,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            respawn=False,
            parameters=param_files,
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
                #
                # In slam mode there IS no localisation manager here, because
                # slam_toolbox is a plain node, not a lifecycle node
                # (use_lifecycle_node defaults false). It must therefore be
                # RUNNING AND PUBLISHING map -> odom before this launch, or the
                # global costmap blocks exactly as it would with a dead AMCL.
                "bond_timeout": 10.0,
            }],
        ),
    ]

    if mode == "slam":
        print("fpms_nav2.launch.py: localization_mode=slam. map -> odom and "
              "/map come from slam_toolbox (slam/fpms_slam_localization."
              "launch.py), which MUST already be running. map_server and amcl "
              "are NOT started. Overlay: %s" % overlay_file)
    elif mode == "amcl":
        print("fpms_nav2.launch.py: localization_mode=amcl. map_server + "
              "nav2_amcl own map -> odom. Published AMCL accuracy is ~8.5-33.7 "
              "cm (R2_COORDINATES.md 4.4) - this is the fallback path, not the "
              "precision one.")
    else:
        print("fpms_nav2.launch.py: localization_mode=none. NOTHING publishes "
              "map -> odom. The costmaps' global_frame must be odom or every "
              "lifecycle node will block. Pose drift is unbounded.")

    return actions


def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(THIS_DIR, "nav2_params.yaml"),
            description="Base Nav2 parameter file. Defaults to the one beside "
                        "this launch file - NOT nav2_bringup's, whose defaults "
                        "inflate a small space into a single obstacle.",
        ),
        DeclareLaunchArgument(
            "overlay_params_file", default_value="",
            description="Extra parameter file merged AFTER params_file (later "
                        "wins). Empty means: nav2_params_slam.yaml when "
                        "localization_mode:=slam, nothing otherwise.",
        ),
        DeclareLaunchArgument(
            "localization_mode", default_value="slam",
            description="Who publishes map -> odom: 'slam' (slam_toolbox, the "
                        "real-localisation path), 'amcl' (static map + "
                        "particle filter, the fallback), or 'none' (dead "
                        "reckoning). See this file's docstring.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(THIS_DIR, "arena_map.yaml"),
            description="Occupancy grid for map_server. Used ONLY when "
                        "localization_mode:=amcl - in slam mode the map comes "
                        "from slam_toolbox and this is ignored.",
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
                        "module docstring.",
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

    ld = LaunchDescription()
    ld.add_action(set_domain)
    ld.add_action(set_stdout)
    for a in declare_args:
        ld.add_action(a)
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
