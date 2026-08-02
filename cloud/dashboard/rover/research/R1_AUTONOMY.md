# R1_AUTONOMY — Route planning & accurate route following for a 1.2 m arena skid-steer rover

Research report. No code changes. All URLs below were fetched or returned by search during this
research; where a page could not be content-verified it is explicitly marked **[URL seen in search
index, content not verified]**.

## 0. The hardware contract this report is written against

| Fact | Consequence for every technique below |
|---|---|
| Firmware slams ~50% duty on ANY non-zero `/cmd_vel`. 0.0010 span fast, 0.00295 did not move. Load is the hidden variable. | **There is no velocity control.** `cmd_vel.linear.x` is a *direction+enable* bit, not a speed. Any algorithm that outputs a continuously-varying speed is being ignored by the plant. |
| Slow = short bursts + full stops. Min pulse ~0.35 s (free-spinning). | Motion is **quantised**. The smallest possible displacement is `v_full x 0.35 s`. Nothing can be commanded finer than that quantum. |
| Pose is dead-reckoned from an ASSUMED start. No AMCL, no SLAM. | Nav2 has **no `map -> odom` producer**. Every Nav2 node will block on TF unless a static identity transform is published. Drift is never corrected. |
| `/odom_raw` twist.linear.x is SIGN-INVERTED vs its own pose.position. | Anything that consumes odom *twist* — Nav2 behavior server collision projection, velocity smoother, progress checker, DWB — will believe the rover is going the wrong way. **This must be fixed in a republisher before any Nav2 node subscribes.** |
| 2D LiDAR `/scan_lidar` ~9.5 Hz. | ~3 scans per 0.35 s burst — useless for closed-loop control during motion, **excellent** for an absolute fix while stopped. |
| Arena 1.2 x 1.2 m, known rectangle, known targets. | The map is trivially known a priori. Global path *planning* is nearly free; the hard problem is 100% **execution accuracy**, not planning. |

**One-line thesis:** in a 1.2 m box the planner is a solved problem you can write in 20 lines; the
error budget is dominated by open-loop execution and unbounded dead-reckoning drift. Every technique
below is graded on whether it reduces *execution* error.

---

## 1. Real open-source code: plan a route, then follow it

### 1.1 `nav2_simple_commander` — the canonical Python route API (ROS 2 Humble)

**What it is:** Nav2's official Python wrapper. `BasicNavigator` exposes the full task API.
Verified against the Humble branch source.

- URL (source, fetched): https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_simple_commander/nav2_simple_commander/robot_navigator.py
- URL (package): https://github.com/ros-navigation/navigation2/tree/main/nav2_simple_commander
- URL (waypoint example): https://github.com/ros-navigation/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_waypoint_follower.py
- URL (API docs, Humble): https://docs.ros.org/en/humble/p/nav2_simple_commander/

**Confirmed public methods in the Humble branch** (this list is exact, from the source file):
`setInitialPose`, `goToPose(pose, behavior_tree='')`, `goThroughPoses(poses, behavior_tree='')`,
`followWaypoints(poses)`, `followPath(path, controller_id='', goal_checker_id='')`,
`spin(spin_dist=1.57, time_allowance=10)`, `backup(backup_dist=0.15, backup_speed=0.025,
time_allowance=10)`, `assistedTeleop(time_allowance=30)`, `cancelTask`, `isTaskComplete`,
`getFeedback`, `getResult`, `waitUntilNav2Active(navigator='bt_navigator', localizer='amcl')`,
`getPath(start, goal, planner_id='', use_start=False)`, `getPathThroughPoses`, `smoothPath`,
`changeMap`, `clearAllCostmaps` / `clearLocalCostmap` / `clearGlobalCostmap`,
`getGlobalCostmap` / `getLocalCostmap`, `lifecycleStartup`, `lifecycleShutdown`.

**Note there is NO `driveOnHeading()` in the Humble `BasicNavigator`.** It exists as an action
server and BT node (see 1.3) but the Humble Python wrapper only ships `spin` and `backup`. If you
want `DriveOnHeading` from Python on Humble you must build the `nav2_msgs/action/DriveOnHeading`
action client yourself (~15 lines) — do not assume the helper exists.

**Survives our hardware?**
- `goToPose` / `followWaypoints` / `goThroughPoses` — **PARTIAL.** The API call itself is fine; what
  it delegates to (a continuous controller) is not. See 1.2.
- `spin()` and `backup()` — **YES, these are the useful ones.** They are measured, odometry-terminated
  moves, i.e. exactly the segment-and-settle primitive this chassis needs. Caveat: `backup_speed`
  default 0.025 m/s is meaningless here (the chassis will do ~full speed regardless); the *distance*
  argument is what matters.
- `waitUntilNav2Active(localizer='amcl')` — **NOT APPLICABLE as written.** There is no AMCL. Call
  it as `waitUntilNav2Active(localizer=None)` or it hangs forever waiting for a lifecycle node that
  does not exist.

### 1.2 Regulated Pure Pursuit (RPP) — the reference carrot-follower

**What it is:** Nav2's recommended controller for differential/skid-steer. Full parameter table and
example YAML fetched verbatim from the Humble branch README.

- URL (fetched, verbatim source of the values below):
  https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_regulated_pure_pursuit_controller/README.md
- URL (docs): https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html
- URL (Humble API): https://docs.ros.org/en/humble/p/nav2_regulated_pure_pursuit_controller/

**Verbatim upstream defaults:**

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 1.0
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 3.2
      max_robot_pose_search_dist: 10.0
      use_interpolation: false
      cost_scaling_dist: 0.3
      cost_scaling_gain: 1.0
      inflation_cost_scaling_factor: 3.0
```

**Survives our hardware? MOSTLY NOT — this is the single biggest "NOT APPLICABLE" in the report.**

RPP's entire value proposition is *modulating linear velocity*: `use_regulated_linear_velocity_scaling`
slows on curvature, `use_cost_regulated_linear_velocity_scaling` slows near obstacles,
`approach_velocity_scaling_dist` ramps down into the goal, `min_approach_linear_velocity` is the floor.
**All five of these knobs are no-ops on a chassis that outputs 50% duty for any non-zero setpoint.**
RPP will compute a beautiful 0.05 m/s approach and the rover will launch at full speed into the goal.

Additional geometric disqualifiers in a 1.2 m arena:
- `lookahead_dist: 0.6` is *half the arena*. The carrot is frequently outside the arena or behind a wall.
- `regulated_linear_scaling_min_radius: 0.9` — every turn in a 1.2 m box is sharper than 0.9 m radius,
  so regulation is permanently saturated at `regulated_linear_scaling_min_speed: 0.25`.
- `max_robot_pose_search_dist: 10.0` — 8x the arena.
- `approach_velocity_scaling_dist: 1.0` — defaults to "costmap forward extent minus one cell"; in a
  1.2 m arena the robot is *always* inside the approach-scaling zone.

**The only ways RPP survives, both requiring work:**

(a) **A duty-cycle chopper node between the controller and the firmware.** Insert a node that takes
Nav2's `/cmd_vel` (continuous) and emits bang-bang bursts whose *duty ratio* equals
`v_desired / v_full`, with ON time >= 0.35 s. This is software PWM at the trajectory level. It is the
only honest way to make a bang-bang plant track a continuous controller. Consequences you must accept:
effective control bandwidth drops to ~1/(2 x 0.35 s) ~= 1.4 Hz, so `controller_frequency: 20.0` is
theatre — the loop is really running at ~1.4 Hz, and `lookahead_dist` must be >> the per-pulse
displacement or the carrot moves less than one quantum and the rover chatters. Nav2's own sanctioned
insertion point for this class of transform is the velocity smoother, which already has a
`deadband_velocity: [Vx, Vy, Vw]` parameter documented as "minimum thresholds, below which we set its
value to 0 ... useful when your robot's breaking torque from stand still is non-trivial so sending
very small values will pull high amounts of current."
  - URL: https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html
  - URL (source): https://github.com/ros-navigation/navigation2/tree/main/nav2_velocity_smoother
  - URL (Humble API): https://docs.ros.org/en/ros2_packages/humble/api/nav2_velocity_smoother/index.html
  - **Survives? PARTIALLY.** `deadband_velocity` is the *right idea for the opposite problem* (robots
    that stall at low command). It cannot create a slow speed. It is still worth setting so that
    Nav2's regulated trickle-speeds get zeroed rather than becoming full-speed lurches — i.e. use it
    as a **"don't twitch" filter**, not as a speed controller.

(b) **Give up on FollowPath and use the behavior primitives instead** — see 1.3. This is the
recommendation.

### 1.3 `DriveOnHeading` + `Spin` — Nav2's own measured-move primitives (THE key finding)

**What it is:** Nav2 ships a behavior server with open-loop-commanded / odometry-terminated moves.
Crucially, Nav2's own repo contains a behavior tree whose *entire purpose* is driving a square from
these primitives for odometry calibration — i.e. upstream Nav2 already endorses segment-and-settle
when you cannot trust a controller.

- URL (fetched verbatim): https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_bt_navigator/behavior_trees/odometry_calibration.xml
- URL (docs page): https://docs.nav2.org/behavior_trees/trees/odometry_calibration.html
- URL (BT node docs): https://docs.nav2.org/configuration/packages/bt-plugins/actions/DriveOnHeading.html
- URL (Humble action API): https://api.nav2.org/actions/humble/driveonheading.html
- URL (behavior server config): https://docs.nav2.org/configuration/packages/configuring-behavior-server.html

**Verbatim upstream file:**

```xml
<!--
  his Behavior Tree drives in a square for odometry calibration experiments
-->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Repeat num_cycles="3">
      <Sequence name="Drive in a square">
        <DriveOnHeading dist_to_travel="2.0" speed="0.2" time_allowance="12"/>
        <Spin spin_dist="1.570796" is_recovery="false"/>
        <DriveOnHeading dist_to_travel="2.0" speed="0.2" time_allowance="12"/>
        <Spin spin_dist="1.570796" is_recovery="false"/>
        <DriveOnHeading dist_to_travel="2.0" speed="0.2" time_allowance="12"/>
        <Spin spin_dist="1.570796" is_recovery="false"/>
        <DriveOnHeading dist_to_travel="2.0" speed="0.2" time_allowance="12"/>
        <Spin spin_dist="1.570796" is_recovery="false"/>
      </Sequence>
    </Repeat>
  </BehaviorTree>
</root>
```

**Survives our hardware? YES — this is the most applicable single artefact found.**
- `dist_to_travel` is a **displacement**, terminated on measured pose delta, not on a velocity profile.
  `speed` is ignored-in-effect by our firmware, which is fine — the *termination condition* is what
  gives accuracy.
- `Spin spin_dist` is a **yaw delta**, again terminated on measurement.
- Scale the numbers: `dist_to_travel` 2.0 -> 0.2..0.5; `time_allowance` 12 -> 4..6.

**Two hardware-specific hazards you must handle:**
1. **Twist sign inversion.** The behavior server projects the current *twist* forward to do collision
   checking (`simulate_ahead_time`). With `/odom_raw` twist inverted, that projection points
   backwards and the behavior can abort with a phantom collision. Fix the sign upstream.
2. **Humble cannot disable collision checks on `Spin`.** Upstream issue confirming the flag exists in
   newer Nav2 but is missing in Humble:
   https://github.com/ros-navigation/navigation2/issues/5633
   In a 1.2 m arena with walls 0.6 m away and default inflation, `Spin`'s forward-projection collision
   check will frequently veto legitimate rotations. Either shrink inflation drastically (section 2),
   or run your own spin primitive instead of Nav2's.

### 1.4 Rotation Shim Controller — "turn first, then drive"

**What it is:** a shim plugin between the controller server and a primary controller; it rotates the
robot in place to the path's initial heading before handing over. Written explicitly for "robots that
can rotate in place, such as differential and omnidirectional robots".

- URL (fetched): https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_rotation_shim_controller/README.md
- Verified defaults: `angular_dist_threshold: 0.785`, `forward_sampling_distance: 0.5`,
  `rotate_to_heading_angular_vel: 1.8`, `max_angular_accel: 3.2`, `simulate_ahead_time: 1.0`,
  `rotate_to_goal_heading: false`, `primary_controller: <plugin>`.

**Survives? CONCEPTUALLY YES, MECHANICALLY NO.** The *decomposition* it encodes — rotate to heading,
then translate — is exactly right for a skid-steer that cannot arc smoothly. But it hands off to a
primary controller after the rotation, and that primary controller is a continuous-velocity
controller (see 1.2). `forward_sampling_distance: 0.5` is also ~40% of the arena. **Take the idea,
not the plugin.** Implement rotate-then-translate yourself as `Spin` + `DriveOnHeading`.

### 1.5 Waypoint follower + pause-at-waypoint

- URL: https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html
- URL: https://docs.nav2.org/configuration/packages/nav2_waypoint_follower-plugins/wait_at_waypoint.html
- Upstream `nav2_params.yaml` defaults (fetched): `waypoint_follower: loop_rate: 20`,
  `stop_on_failure: false`, `waypoint_task_executor_plugin: "wait_at_waypoint"`,
  `waypoint_pause_duration: 200` (ms).

**Survives? YES, and `waypoint_pause_duration` should be raised aggressively (1500-3000 ms).** The
pause is free accuracy: it lets the chassis fully stop, lets the ~9.5 Hz LiDAR deliver 15-30 clean
stationary scans, and lets IMU yaw settle before the next leg's heading is computed. Also set
`stop_on_failure: true` — in a 1.2 m arena, silently skipping a failed waypoint means the rover is
somewhere unknown.

### 1.6 Concrete third-party rover repos (existence verified)

| Repo | URL | What it is | Verified? | Useful to us? |
|---|---|---|---|---|
| linorobot2 | https://github.com/linorobot/linorobot2 (humble branch: https://github.com/linorobot/linorobot2/tree/humble) | 2WD/4WD/Mecanum ROS 2 robots, ships working Nav2 + slam_toolbox + robot_localization EKF configs | **YES** — fetched `linorobot2_navigation/config/navigation.yaml` from the humble branch | Best-quality *complete* small-robot Nav2 param set found. Uses RotationShimController wrapping DWB. Values in section 2. |
| ROBOTIS TurtleBot3 | https://github.com/ROBOTIS-GIT/turtlebot3 (param fetched: `turtlebot3_navigation2/param/burger.yaml`, humble branch) | Reference small (0.1 m radius) diff-drive Nav2 config | **YES** — fetched | Closest official param set to our scale. Values in section 2. |
| RoverRobotics ROS2 | https://github.com/RoverRobotics/roverrobotics_ros2 | Commercial 4-wheel **skid-steer** rover driver + ROS 2 packages | Repo existence confirmed via search index; param files not individually fetched | Relevant as a real 4WD skid-steer reference platform. **[params not content-verified]** |
| AntonioConsiglio/rover_ros2 | https://github.com/AntonioConsiglio/rover_ros2 | Differential rover, Raspberry Pi 5 + **ESP32 via micro-ROS over WiFi** + OAK-D Lite, ROS 2 Humble, Nav2 | **YES** — repo page fetched. Confirmed: Nav2, micro-ROS/ESP32, Humble, Docker. **However, no Nav2 param files or waypoint-following code were visible in the repo root listing** — do not assume a usable param set is there. | Architecturally the closest match to our stack (Pi-class SBC + ESP32 micro-ROS + Nav2). Worth reading for the micro-ROS transport, not for navigation tuning. |
| Rosmo_ROS2_Diffdrive | https://github.com/rosmo-robot/Rosmo_ROS2_Diffdrive | ROS 2 + micro-ROS skid-steer control with ESP32 | Existence via search index only | ESP32 `/cmd_vel` -> motor mapping reference. **[content not verified]** |
| albertrichard080/ros2_esp32-microros | https://github.com/albertrichard080/ros2_esp32-microros | ROS 2 Humble + micro-ROS RC tank, skid steering, ESP32 | Existence via search index only | Same. **[content not verified]** |
| forg_bot | https://github.com/joefscholtz/forg_bot | Curiosity-inspired holonomic rover, ROS 2 Humble navigation | Existence via search index only | Holonomic, so kinematically **NOT APPLICABLE**. **[content not verified]** |

**I did NOT find** a public repo doing "plan-then-follow in a sub-2 m arena with a bang-bang chassis
and no localisation". That combination appears to be genuinely unusual — which is itself a finding:
you are outside the envelope every one of these repos was tuned for, and copying their params
wholesale will fail.

### 1.7 How do real projects handle a chassis that cannot creep?

Honest answer from the search: **they don't have this problem, because they fix it in the firmware.**
The universally-recommended solution in the ROS community is a **minimum-PWM map / feedforward
deadband compensation**: find the PWM at which the wheels just begin to turn under load, then map the
commanded velocity range onto `[PWM_min, PWM_max]` rather than `[0, PWM_max]`, and close a PI loop on
encoder feedback.

- URL: https://answers.ros.org/question/298154/help-with-editing-my-arduino-motor-code-be-controlled-by-cmd_vel/
- URL: https://answers.ros.org/question/64591/cmd_vel-and-motor-command/
- URL: https://www.fusybots.com/post/solving-differential-drive-kinematics-and-implementing-it-on-a-custom-mobile-robot
- URL: https://automaticaddison.com/how-to-control-a-robots-velocity-remotely-using-ros/

**Survives? This is a firmware fix, not a ROS fix — and it is the highest-leverage change available.**
The measured symptom (0.0010 -> fast, 0.00295 -> did not move, load was the hidden variable) is the
textbook signature of *no PWM mapping and no closed-loop speed control on the ESP32*. If the ESP32 is
in scope for change, implementing a min-PWM map + encoder PI loop deletes the entire "cannot creep"
constraint and unlocks RPP, DWB, MPPI and everything else. **If the ESP32 is out of scope, then every
continuous-velocity controller in Nav2 is permanently NOT APPLICABLE and the burst-primitive
architecture is not a workaround — it is the correct architecture.**

---

## 2. Nav2 configuration for a 1.2 m x 1.2 m arena

### 2.1 Why the defaults destroy this arena — with the arithmetic

Upstream `nav2_bringup/params/nav2_params.yaml` (fetched from the humble branch:
https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_bringup/params/nav2_params.yaml):

```
local_costmap:  update_frequency 5.0, publish_frequency 2.0, resolution 0.05,
                width 3, height 3, robot_radius 0.22,
                inflation_radius 0.55, cost_scaling_factor 3.0
global_costmap: update_frequency 1.0, publish_frequency 1.0, resolution 0.05,
                robot_radius 0.22, inflation_radius 0.55, cost_scaling_factor 3.0
planner_server: nav2_navfn_planner/NavfnPlanner, tolerance 0.5, allow_unknown true
controller:     controller_frequency 20.0, xy_goal_tolerance 0.25, yaw_goal_tolerance 0.25
```

Now apply them to a 1.2 m box with walls on all four sides:

- **Costmap grid:** 1.2 m / 0.05 m = **24 x 24 cells**. That is the entire world. The `local_costmap`
  rolling window is `3 x 3 m` — **6.25x larger than the arena**.
- **Inscribed/lethal band:** the inflation layer "places a lethal cost around obstacles within the
  robot's fully inscribed radius" (https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html).
  With `robot_radius: 0.22`, a 0.22 m band around all four walls is cost 253/254. Free square =
  `1.2 - 2(0.22) = 0.76 m`. At 0.05 m resolution that is 15 x 15 cells of nominally-free space, and
  it is only "free" of *lethal* cost.
- **Inflation:** `inflation_radius: 0.55` vs an arena half-width of 0.6 m. Every cell within 0.55 m
  of any wall carries inflated cost — that is **every cell except a ~0.1 x 0.1 m patch at the exact
  centre**. The planner sees a uniformly-expensive blob with no gradient to descend. This is precisely
  the "inflate a 1.2 m box into one solid obstacle" failure.
- **Goal tolerance:** `xy_goal_tolerance: 0.25` is **21% of the arena width**. A "reached goal"
  declaration at 0.25 m is meaningless when your targets are corners 1.2 m apart.
- **Planner tolerance:** `tolerance: 0.5` is **42% of the arena width** — NavFn will happily return a
  path ending half a metre from the water station.
- **Progress checker:** `required_movement_radius: 0.5` within `movement_time_allowance: 10.0` — the
  rover must move 0.5 m (42% of the arena) every 10 s or the controller aborts. On a burst-driven
  chassis doing 0.35 s pulses with settle pauses, this **will** false-trigger.

### 2.2 Real reference param sets at small scale (both fetched and verified)

**TurtleBot3 Burger** (`turtlebot3_navigation2/param/burger.yaml`, humble branch — fetched):
- `controller_frequency: 10.0` (half of upstream default)
- local costmap: 3 x 3 m, `resolution: 0.05`, rolling window, **`robot_radius: 0.1`**,
  **`inflation_radius: 0.5`**, **`cost_scaling_factor: 5.0`**
- global costmap: `resolution: 0.05`, `robot_radius: 0.1`, `inflation_radius: 0.5`,
  `cost_scaling_factor: 5.0`, voxel layer 16 voxels @ 0.05 m z-res
- `xy_goal_tolerance: 0.25`, `yaw_goal_tolerance: 0.25`
- DWB `max_vel_x: 0.3`, `max_vel_theta: 1.0`, `acc_lim_x: 3.0`, `acc_lim_theta: 3.2`
- planner: NavFn, `tolerance: 0.5`, `use_astar: false`, `allow_unknown: true`

**linorobot2** (`linorobot2_navigation/config/navigation.yaml`, humble branch — fetched):
- controller: **RotationShimController** (`angular_dist_threshold: 0.785`,
  `forward_sampling_distance: 0.5`, `rotate_to_heading_angular_vel: 1.8`, `max_angular_accel: 3.2`)
  wrapping **DWB** (`max_vel_x: 0.4`, `max_vel_theta: 0.75`, `acc_lim_x: 2.5`, `acc_lim_theta: 3.2`,
  `sim_time: 1.7`)
- costmaps: `resolution: 0.05`, local `3 x 3`, `robot_radius: 0.22`, `inflation_radius: 0.55`,
  **`cost_scaling_factor: 1.0`** (note: lower than upstream — a *flatter* decay)
- `xy_goal_tolerance: 0.25`, `yaw_goal_tolerance: 0.25`
- planner: NavFn, `tolerance: 0.5`, `use_astar: false`
- velocity_smoother: `max_velocity: [0.5, 0.0, 2.5]`, `max_accel: [2.5, 0.0, 3.2]`,
  `smoothing_frequency: 20.0`

**Critical observation:** *neither* reference — including the one built for a 0.1 m-radius robot —
goes below `resolution: 0.05` or `inflation_radius: 0.5`. **There is no published Nav2 param set for
a sub-2 m arena.** Search returned only generic tuning advice
(https://docs.nav2.org/tuning/index.html, https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/)
and the maintainers' *opposite* advice — "increase your inflation layer cost scale and radius to
adequately produce a smooth potential across the entire map" — which is room-scale reasoning that
inverts at 1.2 m. **You are deriving these numbers, not copying them.** Treat the block below as a
derived starting point requiring bench validation, not a citation.

### 2.3 Derived param set for 1.2 m x 1.2 m (starting point, NOT copied from any repo)

```yaml
# ---- geometry ----
# Use an explicit footprint polygon, not robot_radius, if the rover is non-circular.
# robot_radius must be the REAL circumscribed radius; do not pad it "for safety" —
# in a 1.2 m arena every centimetre of padding costs 2 cm of reachable space.
global_costmap:
  ros__parameters:
    global_frame: odom            # NOT "map" — see section 3.1
    resolution: 0.01              # 1.2 m -> 120 x 120 cells; trivial CPU at this size
    width: 2                      # >= arena + margin
    height: 2
    origin_x: -0.4
    origin_y: -0.4
    robot_radius: 0.11            # MEASURE IT
    footprint_padding: 0.0        # upstream default 0.01 is 1 cm you cannot spare
    update_frequency: 2.0
    publish_frequency: 1.0
    inflation_layer:
      inflation_radius: 0.05      # ~ half the robot radius. NOT 0.55.
      cost_scaling_factor: 15.0   # steep decay: cost must reach ~0 well before mid-arena
    always_send_full_costmap: True

local_costmap:
  ros__parameters:
    global_frame: odom
    rolling_window: true
    width: 1.5                    # NOT 3 — that is bigger than the world
    height: 1.5
    resolution: 0.01
    robot_radius: 0.11
    update_frequency: 5.0
    inflation_layer:
      inflation_radius: 0.05
      cost_scaling_factor: 15.0

planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.03             # NOT 0.5 (42% of the arena)
      use_astar: true             # 120x120 grid: A* is free, and gives cleaner paths
      allow_unknown: false

controller_server:
  ros__parameters:
    controller_frequency: 5.0     # a 0.35 s motion quantum caps useful bandwidth at ~1.4 Hz;
                                  # 20 Hz just burns CPU producing commands the plant ignores
    progress_checker:
      required_movement_radius: 0.05   # NOT 0.5 — that is 42% of the arena
      movement_time_allowance: 20.0    # burst + settle cycles are slow; be generous
    goal_checker:
      plugin: "nav2_controller::StoppedGoalChecker"   # see 3.2
      xy_goal_tolerance: 0.04     # >= 1.5 x the motion quantum. See warning below.
      yaw_goal_tolerance: 0.10
      trans_stopped_velocity: 0.02
      rot_stopped_velocity: 0.05
      stateful: True
```

**Hard warning on `xy_goal_tolerance`:** the minimum achievable displacement is
`v_full x 0.35 s`. If `v_full` is ~0.15 m/s that is a **5.25 cm quantum**, so
`xy_goal_tolerance: 0.04` is *unreachable* and the rover will oscillate around the goal forever.
**Measure `v_full` first, compute the quantum, and set `xy_goal_tolerance >= 1.5 x quantum.** Any
tolerance tighter than the motion quantum is a guaranteed infinite-loop bug. This is the single most
important number in this document and it cannot be sourced from any repo — only from your bench.

---

## 3. Making Nav2 run at all without localisation, and accuracy techniques

### 3.1 The `map -> odom` problem

Nav2 requires a REP-105 TF tree of at minimum `map -> odom -> base_link -> [sensors]`; the
`map -> odom` transform is "usually provided by a localization package such as AMCL".

- URL: https://docs.nav2.org/setup_guides/transformation/setup_transforms.html
- URL: https://docs.nav2.org/concepts/index.html

**Two viable options, both verified as documented patterns:**
1. **Set `global_frame: odom` on both costmaps** and drop the `map` frame entirely. Nav2 docs state
   that "if your system does not have a map_frame, you can remove it and make sure world_frame is set
   to the value of odom_frame" (this phrasing is from the robot_localization/Nav2 odom setup guide:
   https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html). Also drop the
   `static_layer` from both costmaps — with no localisation, a static map layer will progressively
   disagree with reality as drift accumulates, producing ghost walls the rover refuses to drive through.
2. **Publish a static identity `map -> odom`** via `tf2_ros static_transform_publisher`. Simpler, but
   it *lies*: it asserts drift is zero. Functionally equivalent to (1) with extra steps and a
   misleading RViz display. Prefer (1).

**Survives? YES (option 1), and it is mandatory** — without it every Nav2 lifecycle node stalls.
But understand what you have bought: **the global costmap is now a dead-reckoned frame.** Ghost
obstacles from drift are real and raytrace-clearing will not fully remove them, because the drift is
in the *pose*, not the *scan*.

### 3.2 `StoppedGoalChecker` — "settle" enforced by Nav2 itself

**What it is:** a goal checker that additionally requires the robot to be *stopped* inside tolerance
before declaring success. Verified from the Humble source.

- URL (fetched): https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_controller/plugins/stopped_goal_checker.cpp
- Verified declared defaults: **`rot_stopped_velocity` default `0.25`**, **`trans_stopped_velocity`
  default `0.25`** (both declared via `declare_parameter_if_not_declared`).

**Survives? YES — use it instead of `SimpleGoalChecker`.** It converts "arrived" from a position test
into a position-AND-stopped test, which is the "settle" half of segment-and-settle, for free.
**Hazard:** it reads odom *twist*. With the sign inversion, magnitude is presumably still correct so
the stopped test likely still works — **but verify**, because if the driver reports a stale non-zero
twist after a burst ends, the goal checker will never fire. The defaults of 0.25 m/s and 0.25 rad/s
are far too loose for a bang-bang chassis that is either at ~full speed or exactly zero; tighten to
~0.02 / 0.05 so "stopped" means stopped.

### 3.3 Segment-and-settle / measured-move retrace

**Technique (well-supported, multiple sources):** decompose every route into alternating
*turn-in-place* and *drive-straight* primitives, terminate each on a measured pose delta, and come to
a **full stop with a settle pause between segments**. Research on differential-drive waypoint tracking
confirms that "trajectories made by concatenating straight motion and in-place turning primitives are
ones that can be easily followed by a differential drive robot."

- URL: https://www.researchgate.net/publication/335575486_Development_of_Waypoint_Tracking_Controller_for_Differential_Drive_Mobile_Robot
- URL: https://www.researchgate.net/publication/264122061_Waypoints_guidance_of_differential-drive_mobile_robots_with_kinematic_and_precision_constraints
- Nav2's own instantiation of exactly this: the `odometry_calibration.xml` BT in 1.3.
- Nav2's sanctioned pause mechanism: `WaitAtWaypoint` / `waypoint_pause_duration` (1.5).

**Survives? YES — this is the recommended core architecture.** It is the *only* motion model that a
bang-bang chassis can execute faithfully, and it makes each leg's error independent and measurable
rather than smeared across a continuous trajectory.

**Measured-move retrace for the return trip:** record, per leg, the *measured* pose delta actually
achieved (from `/odom_raw` pose — never from twist, and never from the commanded value), then drive
the return by replaying the recorded list reversed and negated. This cancels *repeatable* systematic
error (wheel-diameter mismatch, track-width error) which, per the dead-reckoning literature, is the
dominant and most consistent component: dead reckoning "has relatively small standard deviation,
showing that it is consistently off."

### 3.4 Encoder-count / pose-delta termination instead of time

**Technique:** never terminate a move on elapsed time; terminate on accumulated encoder ticks or
integrated pose. "Accumulate tick counts from both encoders, apply the forward kinematics equations,
and you get the robot's estimated pose."
- URL: https://zbotic.in/differential-drive-robot-kinematics-speed-control-guide/
- URL: https://www.ijcaonline.org/archives/volume95/number13/16654-6632/

**Survives? YES, with a substitution.** The ESP32 exposes no encoder topic, but `/odom_raw`
**pose.position** is the exact equivalent and is stated to be self-consistent. Terminate every burst
sequence on `|pose_now - pose_at_leg_start| >= target`. **NEVER integrate twist** (sign-inverted).
The 0.35 s quantum means you will overshoot by up to one quantum; account for it by targeting
`n = round(target / quantum)` pulses and accepting the residual, or by ending each leg with the
measured residual recorded for the retrace.

### 3.5 UMBmark — calibrate out systematic error (the highest-value calibration)

**What it is:** the University of Michigan Benchmark. Drive a square path **clockwise and
counter-clockwise**, measure the final position error in both directions, and solve for the two
dominant systematic error sources: **unequal wheel diameters** and **incorrect wheelbase**. Reported
results: "a consistent improvement of at least one order of magnitude in odometric accuracy (with
respect to systematic errors)", and the method can isolate wheel diameters differing "by as little as
0.1%".

- URL: http://www-personal.umich.edu/~johannb/umbmark.htm
- URL (paper PDF): https://johnloomis.org/ece445/topics/odometry/borenstein/paper60.pdf
- URL (companion paper PDF): https://johnloomis.org/ece445/topics/odometry/borenstein/paper58.pdf
- URL (Semantic Scholar): https://www.semanticscholar.org/paper/UMBmark:-a-benchmark-test-for-measuring-odometry-in-Borenstein-Feng/3911b15c805f4276fb368a5f5992b7cd962b60c2

**Survives? YES — and Nav2's `odometry_calibration.xml` (1.3) is literally the tool to run it**, once
`dist_to_travel` is scaled from 2.0 m to something that fits a 1.2 m arena (0.4-0.6 m sides, run
CW and CCW). An order of magnitude on systematic error is the cheapest accuracy available and
requires no new hardware.

### 3.6 Skid-steer specific: your heading is the weak link

4-wheel skid-steer violates the differential-drive no-slip assumption by construction. Evidence:
- "Skid-steering configurations are prone to slip during turning maneuvers, resulting in inaccurate
  trajectory prediction using the conventional differential drive kinematic model." The **separated
  ICR** approach substitutes an experimentally-measured effective track width for the physical
  wheel-to-wheel width and "gives better dead reckoning results than the differential drive kinematic
  model." URL: https://ph01.tci-thaijo.org/index.php/rmutt-journal/article/view/257672
- Pentzer et al., online ICR estimation, Journal of Field Robotics:
  https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21509
- Comparative measurement: in 90-degree turn tests, SLAM was **466% more accurate at measuring angles**
  than dead reckoning, and dead-reckoning maps "showed evidence of robot slipping".

**Survives? YES, and it changes the design:** **use the `/imu` yaw for every turn, not wheel odometry.**
A `Spin` terminated on wheel-derived yaw on a 4-wheel skid-steer is the single largest error source in
the system. Also: the effective track width for a skid-steer is *not* the physical one — measure it
(UMBmark gives you exactly this number) and use it.

### 3.7 The technique nobody else needs but you do: stop-and-fix against known walls

Your arena is a **known 1.2 x 1.2 m rectangle** and you have a 2D LiDAR. While stopped at a
segment boundary you can fit lines to the walls and recover absolute `(x, y, yaw)` — no AMCL, no
SLAM, no particle filter, no map server. Two perpendicular walls fully determine planar pose in a
known rectangle.

- RANSAC line fitting from a ROS LiDAR (reference implementation):
  https://github.com/Rotvie/ransac-lidar-ros
- 2D LiDAR MCL package if you later want a probabilistic version:
  https://github.com/NaokiAkai/mcl_ros
- Rectangular-landmark-augmented AMCL (evidence the "fit rectangles, correct pose" idea is a
  published technique): https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1652251/full

**Survives? YES — the best fit to this hardware of anything in this report.** It exploits every
constraint rather than fighting them: 9.5 Hz is plenty when you are *stopped* (a 2 s settle pause =
~19 scans to average); it needs no continuous motion; and it converts unbounded drift into bounded
per-leg error. This is the one technique that turns "nothing localises the rover" from a fatal
premise into a solved problem.

---

## 4. The case AGAINST Nav2 here — even-handed

### 4.1 Arguments against

1. **Nav2 is architecturally a drift-correction consumer, not a drift-tolerant system.** Its own
   documentation describes the `map -> odom` transform as something that "updates live in use" from a
   localisation package, and characterises odometry as useful "for dead-reckoning ... *between global
   position updates*" (https://docs.nav2.org/concepts/index.html). There are no global position
   updates here. Every costmap, every plan, and every goal check inherits uncorrected drift. The
   planner's output is precise about a pose that is wrong.
2. **The published evidence on dead reckoning vs. corrected localisation is unambiguous** — ~1.05 m
   vs ~0.58 m position error on a circle test, and 466% worse angular measurement for dead reckoning.
   Nav2 does not fix this; it *assumes it away*.
3. **Every continuous-velocity controller (RPP, DWB, TEB, MPPI) is a no-op on this plant.** Section
   1.2 enumerates five RPP velocity-regulation features that the firmware discards. You would be
   running, debugging and tuning ~15 parameters that cannot affect the robot.
4. **Arena-scale mismatch is total, not marginal.** Default lookahead (0.6 m) = half the arena; local
   costmap (3x3 m) = 6.25x the arena; planner tolerance (0.5 m) = 42% of arena width; progress-checker
   radius (0.5 m) = 42% of arena width; inflation (0.55 m) ~= the arena half-width. There is no
   published sub-2 m Nav2 param set to copy (section 2.2) — you would be inventing all of it.
5. **Failure modes get harder to debug.** With ~10 lifecycle nodes, a BT, two costmaps and a
   controller plugin between the goal and the wheels, "the rover went 8 cm too far" becomes a
   multi-hour investigation. A 200-line primitive sequencer makes the same bug a one-minute read.
6. **Humble-specific friction:** `Spin` cannot disable collision checks
   (https://github.com/ros-navigation/navigation2/issues/5633), and `BasicNavigator` has no
   `driveOnHeading` (section 1.1) — so the two features you most want are the two with rough edges.

### 4.2 Arguments FOR Nav2 (the operator's side, fairly stated)

1. `DriveOnHeading` + `Spin` + the BT engine are **genuinely good, tested implementations of the exact
   primitives you need** — and they are already written, already handle timeouts/preemption/feedback,
   and already have an ActionServer interface. Reimplementing them is real work with real bugs.
2. The BT navigator gives you **recovery, retry, cancellation and timeout semantics for free**, which
   a hand-rolled sequencer always ends up needing and always implements worse.
3. `nav2_simple_commander` gives a clean Python task API for the mission layer (go to corner A, corner
   B, water station, return) with feedback and result codes.
4. **Migration path.** If the ESP32 ever gets a min-PWM map + encoder PI loop (section 1.7), the
   "cannot creep" constraint vanishes and the full Nav2 controller stack becomes usable *without
   rewriting the mission layer*. A bespoke controller would have to be thrown away.
5. Obstacle avoidance via costmaps, if the arena ever gains dynamic obstacles.

### 4.3 Recommended synthesis

**Use Nav2, but only the top and bottom of it — skip the middle.**

- **Keep:** `nav2_simple_commander` / BT navigator for mission sequencing; `nav2_behaviors`
  (`Spin`, `DriveOnHeading`) for primitives; `nav2_waypoint_follower` with a long
  `waypoint_pause_duration`; `StoppedGoalChecker`; costmaps *only* if you want obstacle checking.
- **Skip:** `controller_server` FollowPath with RPP/DWB (velocity regulation is a no-op —
  section 1.2), AMCL, map_server/static_layer, and any velocity smoothing that assumes a continuous
  actuator.
- **Add (not from Nav2):** the LiDAR wall-fix corrector (3.7), the twist-sign republisher, and the
  burst sequencer.

This keeps the operator's Nav2 requirement satisfied and the migration path open, while putting
accuracy where it actually comes from: measured primitives plus an absolute fix between legs.

---

## TOP 5 THINGS TO IMPLEMENT

**1. Stop-and-fix pose correction against the known arena walls, between every leg.**
Cite: https://github.com/Rotvie/ransac-lidar-ros (RANSAC line fitting from ROS LiDAR); supporting:
https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1652251/full
*Reason:* it is the only item on this list that **bounds** drift instead of merely slowing it, and a
9.5 Hz LiDAR is more than sufficient when the rover is stopped in a known 1.2 m rectangle.

**2. Rebuild motion as segment-and-settle primitives (`Spin` + `DriveOnHeading`), terminated on
`/odom_raw` pose delta — not FollowPath, not a velocity controller.**
Cite: https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_bt_navigator/behavior_trees/odometry_calibration.xml
(Nav2's own square-driving BT, `DriveOnHeading dist_to_travel` + `Spin spin_dist`)
*Reason:* displacement-terminated primitives are the only motion model a bang-bang 50%-duty chassis
can execute faithfully, and Nav2 already ships them.

**3. Fix `/odom_raw` twist sign in a republisher, and set both costmaps' `global_frame: odom` (drop
`map`, drop `static_layer`), before any Nav2 node starts.**
Cite: https://docs.nav2.org/setup_guides/transformation/setup_transforms.html and
https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html ("if your system does not have
a map_frame ... make sure world_frame is set to the value of odom_frame")
*Reason:* without the frame fix Nav2 never activates; without the sign fix the behavior server's
collision projection, the progress checker and `StoppedGoalChecker` all reason about a rover moving
the wrong way.

**4. Measure `v_full`, compute the motion quantum `v_full x 0.35 s`, and set every tolerance from it —
then rescale the whole costmap/planner param set for 1.2 m.**
Cite (the defaults being overridden, fetched verbatim):
https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_bringup/params/nav2_params.yaml
(`inflation_radius: 0.55`, `robot_radius: 0.22`, `resolution: 0.05`, `tolerance: 0.5`,
`xy_goal_tolerance: 0.25`, `required_movement_radius: 0.5`)
*Reason:* `inflation_radius: 0.55` against a 0.6 m arena half-width leaves essentially no free space,
and any `xy_goal_tolerance` tighter than the motion quantum is a guaranteed infinite oscillation —
this number cannot be copied from any repo, only measured.

**5. Run UMBmark (bidirectional square, CW and CCW) to calibrate systematic odometry error, and use
`/imu` yaw — not wheel odometry — to terminate every turn.**
Cite: http://www-personal.umich.edu/~johannb/umbmark.htm and
https://johnloomis.org/ece445/topics/odometry/borenstein/paper60.pdf; skid-steer slip evidence:
https://ph01.tci-thaijo.org/index.php/rmutt-journal/article/view/257672
*Reason:* UMBmark reports "at least one order of magnitude" improvement in systematic odometry error
for zero hardware cost, and on a 4-wheel skid-steer the wheel-derived heading is the single largest
error source (466% worse than corrected localisation on 90-degree turns).

**Honourable mention (highest leverage of all, but out of ROS scope):** implement a minimum-PWM map
plus an encoder PI loop on the ESP32 (https://answers.ros.org/question/298154/help-with-editing-my-arduino-motor-code-be-controlled-by-cmd_vel/).
The measured symptom — 0.0010 fast, 0.00295 dead, load-dependent — is the textbook signature of a
missing PWM mapping. Fixing it deletes the "cannot creep" constraint entirely and unlocks the whole
Nav2 controller stack. If the firmware is ever in scope, do this before anything else on the list.

---

## Appendix: sources

Fetched and content-verified:
- https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_simple_commander/nav2_simple_commander/robot_navigator.py
- https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_regulated_pure_pursuit_controller/README.md
- https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_rotation_shim_controller/README.md
- https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_bringup/params/nav2_params.yaml
- https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_bt_navigator/behavior_trees/odometry_calibration.xml
- https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_controller/plugins/stopped_goal_checker.cpp
- https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble/turtlebot3_navigation2/param/burger.yaml
- https://raw.githubusercontent.com/linorobot/linorobot2/humble/linorobot2_navigation/config/navigation.yaml
- https://github.com/AntonioConsiglio/rover_ros2

Documentation / index-verified (page exists, cited for its documented values):
- https://docs.nav2.org/configuration/packages/configuring-costmaps.html
- https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html
- https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html
- https://docs.nav2.org/configuration/packages/configuring-behavior-server.html
- https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html
- https://docs.nav2.org/configuration/packages/nav2_waypoint_follower-plugins/wait_at_waypoint.html
- https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html
- https://docs.nav2.org/configuration/packages/bt-plugins/actions/DriveOnHeading.html
- https://docs.nav2.org/behavior_trees/trees/odometry_calibration.html
- https://docs.nav2.org/setup_guides/transformation/setup_transforms.html
- https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html
- https://docs.nav2.org/concepts/index.html
- https://docs.nav2.org/tuning/index.html
- https://api.nav2.org/actions/humble/driveonheading.html
- https://docs.ros.org/en/humble/p/nav2_simple_commander/
- https://github.com/ros-navigation/navigation2/issues/5633

Literature:
- http://www-personal.umich.edu/~johannb/umbmark.htm
- https://johnloomis.org/ece445/topics/odometry/borenstein/paper60.pdf
- https://johnloomis.org/ece445/topics/odometry/borenstein/paper58.pdf
- https://ph01.tci-thaijo.org/index.php/rmutt-journal/article/view/257672
- https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21509
- https://www.researchgate.net/publication/335575486_Development_of_Waypoint_Tracking_Controller_for_Differential_Drive_Mobile_Robot

Existence-only (returned by search index, contents NOT verified — do not rely on specifics):
- https://github.com/RoverRobotics/roverrobotics_ros2
- https://github.com/rosmo-robot/Rosmo_ROS2_Diffdrive
- https://github.com/albertrichard080/ros2_esp32-microros
- https://github.com/joefscholtz/forg_bot
- https://github.com/NaokiAkai/mcl_ros
- https://github.com/Rotvie/ransac-lidar-ros
