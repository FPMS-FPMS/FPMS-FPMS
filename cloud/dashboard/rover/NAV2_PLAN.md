# Nav2 — planning only. Nav2 plans, B8B drives.

Investigated 2026-08-04 against the existing `nav2/` config.

## THE RULE

**Never launch `fpms_nav2.launch.py` unmodified.** It always starts
`controller_server` + `bt_navigator`, and those would drive the wheels. Nothing
but our segment executor may reach the motors. (Its `cmd_vel` defaults to
`/cmd_vel_nav`, a deliberately dead topic — if you ever do launch it, leave that
default alone.)

## Minimum node set for planning only

1. `nav2_map_server/map_server` — serves `/map` (transient-local)
2. `nav2_planner/planner_server` — instantiates `global_costmap` internally
3. `nav2_lifecycle_manager` with `node_names: ["map_server","planner_server"]`

NOT needed, and must NOT run: controller_server, bt_navigator, behavior_server,
waypoint_follower, local_costmap, amcl, slam_toolbox.

Get a path from the **action** `/compute_path_to_pose`
(`nav2_msgs/action/ComputePathToPose`). There is no `get_plan` service in Humble.

## Three things to fix before it will work

1. **`arena_map.pgm` DOES NOT EXIST.** It is derived, not committed. Generate it,
   and do NOT overwrite the documented yaml:

        python3 make_arena_map.py --out /tmp/arena_map
        diff /tmp/arena_map.yaml arena_map.yaml   # numbers must match
        cp /tmp/arena_map.pgm ./arena_map.pgm

2. **`robot_radius: 0.15` is too small.** The file flags this itself
   (`nav2_params.yaml:549-557`): a 230 mm-wide chassis needs **0.17**
   (`sqrt(0.12^2 + 0.115^2) = 0.166`). At 0.17 the free centre is [0.17, 1.03]^2
   and zone-a (0.228, 0.972) still fits, with only ~58 mm of margin.
   **Measure the chassis before setting this.** Change both costmap blocks
   (lines 558 and 668).

3. **Disable `obstacle_layer` in `global_costmap.plugins`** — leave
   `["static_layer","inflation_layer"]`. Otherwise planner_server sits in
   `configuring` forever waiting for an observation source, because the three
   `topic: "/scan_lidar"` entries are marked PLACEHOLDER and the board's own
   `/scan` is dead.

## Existing config, as-is

- Planner: `GridBased` = NavFn, `use_astar: false` (Dijkstra), tolerance 0.05
- Global costmap: frame `map`, resolution 0.01, origin (-0.10,-0.10),
  `inflation_radius 0.20`, `cost_scaling_factor 10.0`
- Map: 140x140 px at 10 mm/px
- `yaml_filename` is deliberately empty and injected from the `map` launch arg

## Converting `Nav2Backend` to planning-only

`fpms_missions.py:2724-2841`. Small, surgical diff:

| line | change |
|---|---|
| 401 | import `ComputePathToPose` instead of `NavigateToPose` |
| 907 | `NAV2_ACTION` -> `"compute_path_to_pose"` |
| 2764 | `ActionClient(..., ComputePathToPose, ...)` |
| 2780-2789 | `goal.goal = ps`, `goal.planner_id = "GridBased"`, `use_start = False` |
| 2792 | drop `feedback_callback` — this action has no feedback |
| 2803-2817 | return `result.path.poses` mapped through `map_m_to_arena_mm` |
| 2806-2812 | `check_abort(nav2_active=True)` -> `False`; Nav2 never owns the wire |
| **3592-3606** | the real integration point: replace `backend.goto(...)` with `for wp in backend.plan(tx, ty): self._drive_to(wp.x, wp.y)`, keeping the golden `_drive_to`/`_run_one` as the executor |

`arena_to_map_m` round-trips exactly: `MAP_ORIGIN_*_M = 0.0`, so map metres ==
arena mm / 1000.

## What a static map->odom breaks

**Planning is unaffected** — NavFn works purely on the global costmap in `map`,
and `ComputePathToPose` with `use_start: false` resolves `map->base_footprint`
once.

**But the pose is dead reckoning wearing a `map` label.** Error grows without
bound, so the plan is right in map coordinates while its *start* drifts.
**Re-plan at every waypoint from a fresh pose** rather than trusting one long
plan.

- `amcl` and `slam_toolbox` must NOT run — two publishers on map->odom is the one
  thing TF_TREE.md forbids
- `set_initial_pose`, `/initialpose`, particle recovery are all inert
- `bt_navigator`/`controller_server` assume localisation converges; their
  ClearCostmap+Spin recovery is meaningless against a frozen transform, and
  `SimpleProgressChecker` would abort as odom drift makes progress look wrong.
  This is exactly why we do not run them.

## Bring-up sequence

Every shell: `source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=20`

1. Generate `arena_map.pgm` (above). Verify 140x140, diff shows only comments.
2. `ros2 run tf2_ros tf2_echo map base_footprint` — at rest expect
   ~(0.972, 0.228), yaw ~1.571, and exactly ONE publisher per edge.
3. Measure the chassis, set `robot_radius`.
4. map_server alone -> configure -> activate. Verify `/map` is 140x140,
   resolution 0.01, origin -0.1/-0.1.
5. planner_server alone (obstacle_layer disabled) -> configure -> activate.
   Verify it reaches `active` without blocking.
6. Ask for a path start -> zone-b:

        ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
          "{goal: {header: {frame_id: map}, pose: {position: {x: 0.972, y: 0.972}, \
            orientation: {w: 1.0}}}, planner_id: 'GridBased', use_start: false}"

   Verify: poses run roughly straight up x~0.97 from y 0.228 -> 0.972, all inside
   [0.17, 1.03]. Then zone-a and water — those must curve around the centre and
   never cut diagonally through a wall.
7. Only then wire it into missions.
