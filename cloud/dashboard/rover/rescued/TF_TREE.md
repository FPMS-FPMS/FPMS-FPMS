# FPMS rover TF tree

What publishes what, and how to verify each edge with `tf2_echo`. Read
`NAV2_BRIEF.md` section 7 first — this file only covers the tree itself, not
the rest of the Nav2 blocking chain (LiDAR→ROS, map, AMCL, costmaps).

## The expected tree

```
map                        [DYNAMIC — /tf   — slam_toolbox]
 └── odom
      └── base_footprint    [DYNAMIC — /tf   — fpms_odom_tf.py]
           └── base_link    [STATIC  — /tf_static — fpms_tf.launch.py]
                ├── laser_frame  [STATIC  — /tf_static — fpms_tf.launch.py]
                └── imu_frame    [STATIC  — /tf_static — fpms_tf.launch.py]
```

`map -> odom` is now owned by **`slam_toolbox`** (`../slam/`), not AMCL. Run
either `slam/fpms_slam_mapping.launch.py` (building a map) or
`slam/fpms_slam_localization.launch.py` (localising in one); both publish that
edge, and only one of them may run at a time. `nav2_amcl` remains configured in
`nav2_params.yaml` and is selectable via `fpms_nav2.launch.py
localization_mode:=amcl`, but AMCL and slam_toolbox must **never** run together
— that is two publishers on one edge, the same failure this file warns about
everywhere else.

Until a SLAM node is running, `odom` is the root of the tree as far as
`ros2 run tf2_tools view_frames` is concerned, and that is expected, not a bug.

### Who owns which edge

| Edge | Publisher | Topic | Kind |
|---|---|---|---|
| `map` → `odom` | `slam_toolbox` (via `../slam/*.launch.py`) | `/tf` | dynamic, 50 Hz (`transform_publish_period 0.02`), recomputed only when the scan matcher adds a node |
| `odom` → `base_footprint` | `fpms_odom_tf.py` | `/tf` | dynamic, ~10 Hz (paced by `/odom_raw`) |
| `base_footprint` → `base_link` | `fpms_tf.launch.py` (`static_transform_publisher`) | `/tf_static` | static, latched, published once |
| `base_link` → `laser_frame` | `fpms_tf.launch.py` (`static_transform_publisher`) | `/tf_static` | static, latched, published once |
| `base_link` → `imu_frame` | `fpms_tf.launch.py` (`static_transform_publisher`) | `/tf_static` | static, latched, published once |

**One publisher per edge, no exceptions.** `fpms_odom_tf.py` used to also
publish `base_footprint → laser_frame` directly via its
`PUBLISH_LASER_STATIC_TF` flag. That flag is now `False` — `fpms_tf.launch.py`
owns the path to `laser_frame` via `base_link` instead. If you ever re-enable
that flag while also running this launch file, `laser_frame` gets two parents
and the tree stops being a tree. `ros2 run tf2_tools view_frames` will show it
as two disconnected-looking chains, or `tf2_echo` between the two candidate
parents will report wildly inconsistent numbers that change depending on
which broadcaster most recently won the race.

## Before you run any of this

Every command below silently returns nothing useful — not an error, just no
output — if `ROS_DOMAIN_ID` is wrong. This rover is **hard-wired to 20**
(NAV2_BRIEF.md §2). In every shell you run these commands from:

```bash
export ROS_DOMAIN_ID=20
source /opt/ros/humble/setup.bash
```

## Starting the tree

```bash
# Edge 1: odom -> base_footprint (already authored; NOT yet installed as a
# service per NAV2_BRIEF.md §5 — run it manually until it is)
python3 fpms_odom_tf.py

# Edges 2-4: base_footprint -> base_link -> {laser_frame, imu_frame}
# Pass the MEASURED mount offsets; require_measured refuses to start without them.
ros2 launch nav2/fpms_tf.launch.py \
    laser_x:=<m> laser_y:=<m> laser_z:=<m> laser_yaw:=<rad> \
    measured:=true require_measured:=true

# Edge 0: map -> odom  (pick exactly one)
ros2 launch ../slam/fpms_slam_mapping.launch.py        # building a map
ros2 launch ../slam/fpms_slam_localization.launch.py   # localising in one
```

## Verifying each edge with `tf2_echo`

Run each of these in its own terminal (same `ROS_DOMAIN_ID=20` shell). Let
each run a few seconds before judging it — static edges print once
immediately, `odom -> base_footprint` needs `/odom_raw` flowing first.

### 1. `odom` → `base_footprint` (dynamic)

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

**Healthy** — new samples every ~0.1 s (paced by `/odom_raw` at ~10 Hz), pose
changing plausibly (a few cm/loop while driving, ~0 while still), never NaN:

```
At time 1234.567
- Translation: [0.142, -0.031, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.087, 0.996]
```

**Broken**:
- Nothing prints at all → `fpms_odom_tf.py` is not running, or wrong
  `ROS_DOMAIN_ID`, or `micro-ros-agent` is down (see NAV2_BRIEF.md §1/§5 —
  **never restart `micro-ros-agent`**, it costs a 90–225 s reconnect).
- Prints stop dead mid-session → the node's own watchdog suspended the
  broadcast because `/odom_raw` went stale (`ODOM_STALE_SEC=2.0`); check its
  log for `"ERROR /odom_raw stale ..."`. This is deliberate — a frozen
  transform would be worse than no transform (fpms_odom_tf.py's own comment:
  "A frozen TF is strictly worse than no TF").
- `Failure at 1234.567 / Lookup would require extrapolation into the future` →
  usually a clock/stamp problem, not a real gap; `fpms_odom_tf.py` stamps with
  the Pi's own ROS clock specifically to avoid this (see its `_stamp()`
  docstring) — if you see this, something upstream of that guard changed.

### 2. `base_footprint` → `base_link` (static)

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
```

**Healthy** — prints once immediately (static transforms are latched, `tf2_echo`
does not need new samples), translation `[0.0, 0.0, 0.035]`, identity rotation:

```
At time 0.0
- Translation: [0.000, 0.000, 0.035]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
```

**Broken** — nothing prints within a couple seconds → `fpms_tf.launch.py` is
not running, or wrong `ROS_DOMAIN_ID`.

### 3. `base_link` → `laser_frame` (static, PLACEHOLDER OFFSETS)

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
```

**Healthy (structurally)** — prints once, translation is whatever the
`laser_x` / `laser_y` / `laser_z` **launch arguments** were set to. Their
defaults are **unmeasured placeholders** (`[0.0, 0.0, 0.065]`, all angles 0) —
the transform being present and non-NaN only proves the tree is connected, it
does **not** prove the numbers are right. Do not treat a clean `tf2_echo` here
as license to trust a map built with it.

The offsets are launch arguments now, not source constants:

```bash
ros2 launch nav2/fpms_tf.launch.py \
    laser_x:=0.052 laser_y:=0.000 laser_z:=0.071 \
    laser_roll:=0.0 laser_pitch:=0.0 laser_yaw:=-0.0122 \
    measured:=true
```

`require_measured:=true` makes the launch **refuse to start** while the
offsets are still the placeholders. Use it for the mapping run — a map built
on guessed offsets looks fine and is wrong.

**Why the yaw is the one that matters:** 1° of mount yaw is ~10.5 mm of
position error over a 600 mm lever arm (`../research/R2_COORDINATES.md` §4.5).
It is a constant rotation of every scan, so it does not average out; it warps
the map, and the warp is then baked into every mission localised against it.

**Broken** — nothing prints → launch file not running / wrong domain. Values
present but obstacles in the costmap appear offset or rotated from where they
physically are → the placeholders have not been replaced with measured ones
yet. The measurement procedure (including how to get the yaw to better than
the LiDAR's 1° bin size, and the floor-strike arithmetic that makes mount
*pitch* matter) is in the module docstring of `fpms_tf.launch.py`.

**Not fixable here:** `LIDAR_ROTATION_SIGN` in `fpms_lidar_ros.py` (currently
`-1`, marked UNVERIFIED) decides whether the scan is **mirrored**. A mirror is
not a rigid transform — no value of `laser_yaw` can undo it. Run the left-wall
test at the top of that file before trusting any map.

### 4. `base_link` → `imu_frame` (static, PLACEHOLDER OFFSETS)

```bash
ros2 run tf2_ros tf2_echo base_link imu_frame
```

**Healthy (structurally)** — prints once, `[0.0, 0.0, 0.0]`, identity
rotation (current placeholder: IMU co-located with `base_link`, unmeasured).
**Broken** — same failure modes as #3.

### 5. Composed sanity check — `odom` → `laser_frame`

```bash
ros2 run tf2_ros tf2_echo odom laser_frame
```

This exercises the full 3-hop chain at once
(`odom→base_footprint→base_link→laser_frame`). If #1–#3 each work
individually but this fails, the tree has a break tf2 isn't reporting clearly
at the single-edge level — check for a second, conflicting broadcaster on one
of the intermediate edges first (see "One publisher per edge" above).

### 6. Whole-tree view

```bash
ros2 run tf2_tools view_frames
```

Produces `frames.pdf` in the working directory. **Healthy**: one tree, root
`odom`, four boxes total (`odom`, `base_footprint`, `base_link`,
`laser_frame`, `imu_frame` — five, counting correctly), no red/disconnected
nodes, `base_footprint`'s "Broadcaster" listed as `fpms_odom_tf` and the
other two as the `static_transform_publisher` processes. **Broken**: any frame
missing, any frame with more than one incoming edge, or a broadcaster you did
not expect (a leftover process from a previous run, or `PUBLISH_LASER_STATIC_TF`
having been flipped back on in `fpms_odom_tf.py`).
