# Measuring the LiDAR mount

This document exists so nobody has to re-derive this crouched next to a rover.

It covers `base_link -> laser_frame`: where the scanner is, which way it faces,
and — first, before anything else — whether its scan is mirrored.

## The symptom

The image boots. Every unit is active. And then:

```
$ systemctl status fpms-nav2
   Active: failed
$ journalctl -u fpms-nav2
   REFUSING: FPMS_TF_OFFSETS_MEASURED is not 1 in /etc/fpms/config.env.
   The LiDAR mount transform has never been measured.
```

**That refusal is the good outcome.** It exists because of the bad one, which
looked like this: a rover that started perfectly, published a complete TF tree,
built a map, navigated confidently, and was wrong by a fixed rotation the whole
time.

## Why it was invisible

`nav2/fpms_tf.launch.py` carries six offsets for `base_link -> laser_frame`:

```python
LASER_X_DEFAULT = 0.0        # MEASURE ME
LASER_Y_DEFAULT = 0.0        # MEASURE ME
LASER_Z_DEFAULT = 0.065      # MEASURE ME (mast guess)
LASER_ROLL_DEFAULT = 0.0     # MEASURE ME (level the scan plane)
LASER_PITCH_DEFAULT = 0.0    # MEASURE ME (level the scan plane)
LASER_YAW_DEFAULT = 0.0      # MEASURE ME (1 deg = ~10.5 mm of position error)
```

They were made launch **arguments** so they could be supplied without editing
code. Nothing supplied them. `fpms-tf.service` passes
`laser_x:=${FPMS_TF_LASER_X:-0.0}` and friends, and `/etc/fpms/config.env` sets
every one of those variables to `0.0`.

So the TF tree published a complete, well-formed, entirely fictional transform:
the scanner exactly on the axle centreline, exactly level, exactly aligned with
the nose. **Zero is a legal value.** No node errored, no log complained, and
`ros2 run tf2_tools view_frames` drew a beautiful tree.

That is the whole failure mode, and it is the same one as the DDS bug in
`DDS.md`: not a crash, a plausible number. Only base_footprint -> base_link
(z = 0.035, which follows from the measured 70 mm wheel diameter) was ever real.

## The arithmetic that makes it matter

From the launch file's own header, over this chassis's ~600 mm lever arm:

```
1 degree of mount yaw  ~=  10.5 mm of position error
```

A mount that is "about right, probably within a couple of degrees" is already a
~2 cm bias on every fix. What turns that from an annoyance into a wrecked map is
the *direction*: **the obstacle cone guard and the occupancy grid inherit the
same rotation in the same sense.** Errors that enter every consumer in the same
direction do not average out over a mission — they accumulate. For a
centimetre-accuracy target, mount yaw has to be known to better than ~0.5°.

Pitch fails differently and more spectacularly. A scanner at height `h` with
downward pitch `p` puts its beams into the floor at range `h / tan(p)`:

| height | pitch | floor strike |
|---|---|---|
| 0.10 m | 1° | 5.7 m |
| 0.10 m | 2° | 2.9 m |

This room returns a median of ~1.98 m and a maximum of 6.0 m. So 1–2° of mount
pitch turns the far half of every scan into a ring of phantom obstacles — a fake
wall that moves with the robot.

Hence the guards. `fpms-nav2.service`, `fpms-slam-localization.service` and
`fpms-slam-mapping.service` each carry an `ExecStartPre` that refuses to start
while `FPMS_TF_OFFSETS_MEASURED` is not `1`. Navigating on an unmeasured mount
produces confidently wrong positions, which is worse than not navigating.

## THE MIRROR COMES FIRST

Before any of the above, one thing has to be settled, and it is not a TF value.

`LIDAR_ROTATION_SIGN` in `fpms_lidar_ros.py` is `-1` and marked **UNVERIFIED**.
It decides whether the scan is handed the same way as ROS or **mirrored**.

**A mirror is not a rigid transform.** No value of `laser_yaw` can undo one — a
reflection is not in the rotation group, so there is nothing to cancel it with.
The consequence is specific and nasty: if you measure yaw against a mirrored
world, the estimator converges happily and gives you a confident, plausible,
completely wrong number. You cannot detect this by looking at the result.

So the mirror check is step one, and everything else is conditional on it.

### How the check works

`mirror_verdict()` in `fpms_scanmatch.py` compares two independent witnesses
over the same hand-turn:

- the **scan's** apparent rotation, from correlating the two range profiles;
- the **gyro's** rotation, from integrating `/imu`'s `angular_velocity.z`.

The chassis turns by `+dθ`. Every static feature in the world therefore appears,
*from the scanner*, to turn by `−dθ`. So on a correctly-handed scan the two have
**opposite signs**.

| scan vs gyro | verdict |
|---|---|
| opposite signs | `ok` — handedness is correct |
| same sign | `mirrored` — flip `LIDAR_ROTATION_SIGN` |
| turn < ~15°, or ratio outside 0.55–1.8 | `inconclusive` — turn further, try again |

The `inconclusive` band is not padding. Near zero the signs are noise, and a
ratio far from 1 means either `gyro_scale` is wrong or the rover translated as
well as turned — in both cases the handedness question has not actually been
answered, and saying so is the correct output.

`fpms_lidar_ros.py` is owned by another agent. **Report the verdict; do not edit
that file as part of this procedure.**

## The double-correction trap

`LIDAR_ZERO_OFFSET_DEG` (in `fpms_lidar_ros.py`, currently `0.0`) and
`laser_yaw` (in the TF launch) **rotate the scan in the same sense**.

Correct mount yaw in **exactly one** of them. Applying half in each produces a
result that looks almost right, and a bias that moves whenever either file is
touched — which is how you spend a weekend chasing a number that is not there.

**Use `laser_yaw`**, i.e. `FPMS_TF_LASER_YAW` in `/etc/fpms/config.env`. It is a
configuration value, it needs no code edit, it is the one the launch file
documents as the place to put it, and it is not in a file another agent owns.

## What the tool is

Two files, useless apart:

| file | what it is |
|---|---|
| `/usr/local/lib/fpms/fpms_scanmatch.py` | pure numpy. No ROS, no hardware, no I/O — functions from arrays to numbers, testable on any machine |
| `/usr/local/bin/fpms-calibrate-lidar` | the ROS 2 node. Subscribes to `/scan_lidar` and `/imu`, and does nothing else |

### It commands nothing. You push the rover by hand.

**`fpms-calibrate-lidar` publishes to no actuation topic at all** — not
`/cmd_vel`, not `/cmd_duty`, nothing. It is the same contract as
`fpms_charact.py`: the operator moves the rover, with a hand on it, and the
program watches.

That is not a limitation being worked around. It buys three things:

1. **No motion calibration is needed.** Every estimate uses the *direction* of
   apparent motion and never its magnitude. See below — this matters more than
   it sounds.
2. **No safety surface.** Nothing in this tool can move the rover, so there is
   no failure mode where a calibration run drives into a wall.
3. **The gyro becomes an independent witness.** Heading from integrating
   `angular_velocity.z` does not depend on wheel calibration either, so the
   mirror check rests on two measurements that share no common error.

It is also the only thing that *works*. `cmd_vel` on this rover cannot crawl —
the deadzone means any non-zero velocity is about 0.7 m/s — so a
software-driven calibration would consist of a series of lunges.

## What is observable, and what is not

State this before quoting any number at anybody.

| quantity | observable? | how |
|---|---|---|
| **mirror** | **yes** | rotate in place; a mirrored scan turns the wrong way |
| **yaw** | **yes** | push straight; static features stream past at an angle that *is* the mount yaw |
| **x, y** | **weakly** | rotate in place; an off-centre scanner swings on a lever arm. Reported with a caveat, and it assumes the rover pivoted about `base_link`'s origin — a hand-turned rover pivots wherever it happens to pivot |
| **z** | **no** | a 2D scan plane carries no height information. **Use a ruler.** One unambiguous measurement |
| **roll, pitch** | **no** | only ever detectable as a floor strike. **Diagnosed, never estimated.** Level the mount by eye or with a phone inclinometer |

`plane_level_diagnostic()` does not estimate a tilt angle and does not pretend
to. It reports whether a band of suspiciously short returns is *clustered in
bearing* — which is what a tilted plane grazing the floor looks like — as
opposed to scattered, which is just furniture. It tells you whether to go and
look at the mount. That is all it can honestly do.

## Why this needs no distance calibration

The rover's counts/mm is disputed. It has been recorded as 14.8 and measured at
5.5 — a 2.7× error that is behind every distance overshoot this project has
seen, and `COUNTS_PER_REV` is still 3255 where it should be ~1120.

**None of that can corrupt these estimates**, because none of them uses a
distance. `yaw_from_straight_push()` computes

```
psi = pi - atan2(dy, dx)      (wrapped)
```

`atan2` of a vector is unchanged if you scale both components — so the *length*
of the observed motion, which is where any odometry error would live, never
enters the answer. The magnitude is used for exactly one thing: deciding whether
the push was long enough to trust the direction (confidence ramps in over
~50–200 mm of travel), and that judgement comes from the scan itself, not from
the wheels.

The same is true of the mirror check: it compares *signs*, and a gyro that is
mis-scaled by 20% still has the right sign.

## The procedure

**The order is load-bearing.** Do not skip ahead to yaw.

The exact flag spellings are whatever `fpms-calibrate-lidar --help` says — that
tool is the authority on its own interface. What follows is the order and the
reasoning, which do not change.

### 0. Before you start

Get the environment right, and understand why:

```sh
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fpms/fastdds_udp_only.xml
export ROS_LOCALHOST_ONLY=1
```

**The profile line is not optional and it is not boilerplate.** This tool is a
subscriber created by hand, long after the publishers started — which is
precisely the case that fails in `DDS.md`. Without it, Fast DDS prefers shared
memory, discovery succeeds, `ros2 topic info /scan_lidar` reports a publisher,
and **not one scan is ever delivered**. There is no error anywhere. You would
sit there watching a tool that says it is waiting for scans while the LiDAR runs
perfectly two metres away.

Then confirm data is actually moving before you start pushing furniture around:

```sh
ros2 topic hz /scan_lidar     # must report a rate, not hang
ros2 topic hz /imu            # same
```

Two more things:

- **Nothing needs to be stopped for the tool's sake** — it commands nothing. But
  you are about to push the rover by hand, so make sure nothing *else* is
  commanding it. A mission or a teleop session that starts mid-push will move
  the chassis under your hands and corrupt the estimate without saying so.
- **A parked gyro reading exactly `0.0` is healthy.** There is a ±0.01 rad/s
  deadband. Do not go looking for a broken IMU because a stationary rover
  reports zero angular velocity — that has already cost this project time once.

### 1. Mirror check — rotate in place, by hand

Put the rover somewhere with structure on several sides (a room corner is
ideal; a bare corridor is the worst case, because rotation genuinely is not
observable in one). Start the tool's mirror check, then **turn the chassis by
hand through at least 30°** — comfortably more than the 15° floor where the
signs stop being noise. Turn smoothly, and try not to translate.

Repeat it two or three times, in both directions.

- `ok` in both directions → handedness is correct. Go to step 2.
- `mirrored` → **stop.** `LIDAR_ROTATION_SIGN` must be flipped in
  `fpms_lidar_ros.py` before any yaw number means anything. That file is owned
  by another agent: report the verdict and the evidence, wait for the change,
  restart `fpms-lidar-ros`, and re-run this step.
- `inconclusive` → read the detail line. It tells you which of the three cases
  you are in: not enough turn, a stale or featureless scan, or a scan/gyro ratio
  that means you translated as well as turned.

### 2. Yaw — several straight pushes

Only now.

Push the rover **straight along its own +x** (nose direction) by 300–500 mm, in
one smooth motion, along the chassis axis rather than along the room. Do not
turn while pushing — a rotation mixed into the push biases the direction, which
is the one thing this estimate depends on.

**Do this several times.** Four or five pushes, in different parts of the room,
some forward and some backward. The point is not to average away noise so much
as to *catch a loose mount*: if the answer is not the same every time, the
scanner is moving relative to the chassis and no number will save you. Tighten
it and start again from step 1.

A short nudge is worse than useless — its direction is dominated by scan noise.
Below ~50 mm the tool reports zero confidence, and it only reaches full
confidence around 200 mm.

### 3. Apply — in exactly one place

Convert to **radians** and write it to `/etc/fpms/config.env`:

```sh
# 1 degree = 0.01745 rad. If you measured 0.70 degrees, the value is 0.0122.
FPMS_TF_LASER_YAW=-0.0122
FPMS_TF_LASER_X=0.052
FPMS_TF_LASER_Y=0.000
FPMS_TF_LASER_Z=0.071
```

`fpms-tf.service` carries the warning for a reason: **if you measured 12
degrees, the value is 0.2094, not 12.** A degree value pasted into a radian
field is a 57× error that still launches cleanly.

Leave `LIDAR_ZERO_OFFSET_DEG` at `0.0`. See the double-correction trap above.

Set `laser_z` from a **ruler** — height of the optical centre (the rotating
mirror axis, not the case edge) above the floor, minus 0.035 for base_link's
rise. It is not observable from a 2D scan and never will be.

Then, and only then:

```sh
FPMS_TF_OFFSETS_MEASURED=1
```

That variable does two things at once: it satisfies the `ExecStartPre` guards on
`fpms-nav2` and both SLAM units, and it is passed to the launch file as
`measured:=1`, which suppresses the unmeasured banner. Setting it while any
offset is still a guess is signing your name to a number.

Restart the TF publisher and everything downstream of it:

```sh
sudo systemctl restart fpms-tf
sudo systemctl status fpms-tf     # must not print the PLACEHOLDER banner
```

### 4. VERIFY — the step people skip

Re-run the yaw estimate **with the new transform live**. The residual mount yaw
must now come back near zero. If it comes back at roughly *double* what you
originally measured, you have applied the correction in the wrong sense; if it
comes back roughly *unchanged*, the value did not reach the launch (check for a
`\r` in `config.env` — `FPMS_TF_OFFSETS_MEASURED=1\r` is not `1`).

Also do the physical check the launch file describes, because it is independent
of everything above: park the rover with its nose squarely against a flat wall,
`ros2 topic echo /scan_lidar --once`, and confirm the minimum range lands at
index 180 (dead ahead). Then put a wall on the rover's **left** only; short
returns must appear around index 270. If they appear near index 90, you are back
at step 1 and the mirror is wrong.

### 5. Optional — the lever arm

If you want a cross-check on the ruler for `laser_x` / `laser_y`, rotate in
place again and let `lever_arm_from_rotation()` solve for the offset.

**Treat this as a sanity check on a ruler, not a replacement for one.** It is
singular at zero rotation, ill-conditioned near it, and it assumes the rotation
was about `base_link`'s origin — which is exactly the assumption a hand-turned
rover is least likely to satisfy. If it agrees with your ruler to within a
centimetre, believe the ruler and move on. If it disagrees wildly, believe the
ruler and move on.

### 6. Level diagnostic

Run the floor-strike check from a few positions in the room. It is a smell test
with one job: telling you whether to go and look at the mount.

If it reports a floor strike, put a small spirit level or a phone inclinometer
on the LiDAR's mounting face and fix it **mechanically**. Target < 0.5°. Roll
and pitch are not observable from a 2D scan, so there is no software answer
here — and a tilted plane still slices the room at an angle even if you enter
the residual into the TF.

## A worked example

Illustrative. This is the *shape* of the output, not a recording — see the
limits section: none of this has run on a rover.

```
$ fpms-calibrate-lidar          # mirror check, hand-rotate on the prompt

  waiting for /scan_lidar and /imu ... ok (9.6 Hz, 25.1 Hz)
  turn the rover by hand, at least 30 degrees, then stop.

  scan   -31.4 deg
  gyro   +32.1 deg   (ratio 0.98)
  VERDICT: ok
    scan turned -31.4 deg against a gyro 32.1 deg - opposite signs,
    which is correct.

$ fpms-calibrate-lidar          # yaw, four straight pushes

  push 1   d=0.412 m   yaw = -0.71 deg   confidence 1.00
  push 2   d=0.385 m   yaw = -0.66 deg   confidence 1.00
  push 3   d=0.301 m   yaw = -0.73 deg   confidence 1.00
  push 4   d=0.044 m   REJECTED - too short to trust the direction

  mount yaw   -0.70 deg  =  -0.0122 rad   (3 pushes, spread 0.07 deg)
  that is ~7.3 mm of position error at the 600 mm lever arm.

  put this in /etc/fpms/config.env, and NOWHERE ELSE:
      FPMS_TF_LASER_YAW=-0.0122
  do NOT also change LIDAR_ZERO_OFFSET_DEG - they rotate the same way.
```

The spread across pushes — 0.07° here — is the number that tells you whether to
believe the answer. It is a direct measurement of mount rigidity plus operator
technique. A spread of a degree or more means something is loose.

## Troubleshooting

| what you see | what it means | what to do |
|---|---|---|
| tool waits forever for `/scan_lidar`, but `ros2 topic info` shows a publisher | the DDS shared-memory bug. Discovery works, data does not cross process eras | set `FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fpms/fastdds_udp_only.xml`. See `DDS.md` |
| `ModuleNotFoundError: fpms_scanmatch` | `/usr/local/lib/fpms` is not on `sys.path` | run `PYTHONPATH=/usr/local/lib/fpms fpms-calibrate-lidar`, and report it — stage 30 installs a `.pth` that should make this impossible |
| `/usr/bin/env: bad interpreter` | a `\r` on the shebang | the file was checked out CRLF. `verify_image.sh` checks for exactly this; rebuild after `git add --renormalize .` |
| every range is `0.0`, scan looks like a clear 360° | the agent publishes on a timer, not on data, so a dead scanner still emits full-length messages | the scanner or its link is dead. `0.0` converts to `inf` = "no return", which is why this reads as clear rather than blind |
| `mirrored` | handedness is wrong | flip `LIDAR_ROTATION_SIGN` (owned by another agent). Nothing downstream is trustworthy until this is fixed |
| `inconclusive`, "turned only N deg" | turn was under ~15°, so the signs are noise | turn further |
| `inconclusive`, "ratio too far apart" | you translated as well as turned, or `gyro_scale` is wrong | turn on the spot; if it persists, suspect the gyro scale |
| `inconclusive`, "scan barely moved" | stale scans, or nothing with structure in view | check the scan rate; move somewhere with walls |
| yaw differs by degrees between pushes | the mount is loose, or you rotated during the push | tighten the mount, re-run from step 1 |
| verify step returns ~double the original yaw | correction applied in the wrong sense | negate `FPMS_TF_LASER_YAW` |
| verify step returns the original yaw unchanged | the value never reached the launch | check `config.env` for CRLF; confirm `fpms-tf` restarted |
| floor strike suspected | the scan plane is tilted | level the mount mechanically. Not solvable in software |
| `fpms-nav2` still refuses after measuring | `FPMS_TF_OFFSETS_MEASURED` is not exactly `1` | check for a trailing `\r`; restart the unit |

## How to verify the image even has the tool

On the build host, against the artefact:

```sh
sudo selftest/verify_image.sh fpms-os-<version>-<date>.img.xz
```

`check_calibration` asserts that both files landed, that the node is executable
with an **LF** shebang, that the library parses and still defines all eight
functions the node calls, that `/usr/local/lib/fpms` is on `sys.path` via a
`.pth`, and that the directory is not world-writable — it is on every
interpreter's path, so anything that can write there can inject a module into
every ROS node.

Stage 30 makes the equivalent checks against the staged overlay during the
build, so a broken library fails the build rather than the arena.

Note what neither can establish: **they parse the library, they never import
it.** `numpy` is not resolved, and a `NameError` inside a function body survives
both untouched.

## What is still unknown

Say this out loud before quoting any of it at anybody.

- **None of this has ever run on a rover.** Not the tool, not the procedure. The
  estimators have been exercised only against synthetic scans. Every confidence
  threshold in `fpms_scanmatch.py` — the 15° mirror floor, the 0.55–1.8 ratio
  band, the 50–200 mm confidence ramp, the 4% floor-strike fraction, the 0.6
  bearing-concentration cut — is a considered guess, not a calibrated value.
  Expect to move at least one of them after the first real session.
- **`LIDAR_ROTATION_SIGN` is still `-1` and still unverified.** The evidence for
  the zero offset is good (phase6 aimed at markers and stopped at walls using
  it); the evidence for the *sign* is weaker, because phase6's front check was
  symmetric ±25° and carries no left/right information on its own.
- **`z`, roll and pitch are not measurable here and never will be** from a 2D
  scan. A ruler and a spirit level are not a workaround; they are the method.
- **The lever arm assumes rotation about `base_link`'s origin.** A hand-turned
  rover does not do that.
- **`gyro_scale` is not independently verified.** The mirror check only needs
  the sign, so it survives a mis-scaled gyro — but a ratio outside 0.55–1.8 will
  read as `inconclusive` when the real fault is scale, and the tool cannot tell
  those apart.
- **This document does not define the tool's command line.**
  `fpms-calibrate-lidar --help` is the authority on its own flags. What is
  written here is the order of operations and the reasoning behind it, and those
  are the parts that would still be true if the interface changed tomorrow.

If any of this turns out to be wrong on hardware, **correct this file rather
than working around it.** Removing a guard for a problem you have not confirmed
fixed is a mistake this project has made before.
