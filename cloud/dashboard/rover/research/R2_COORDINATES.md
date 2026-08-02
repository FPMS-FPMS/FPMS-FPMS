# R2 — Coordinates, Pose and Centimetre Accuracy

**Research only. No production code changed. Nothing was ssh'd, committed or pushed.**

> **On "Tesla's coordinate code":** Tesla's localisation stack is proprietary and unpublished.
> There is no public source to read and this report does not pretend otherwise. What follows is
> the *public, published* body of technique that production AV stacks and the ROS ecosystem
> actually use — REP-105 frames, odometry calibration, EKF fusion, map-referenced scan
> correction — translated to our 1200x1200 mm indoor arena and our actual sensor set.
> Where a number is our own derivation rather than a citation, it is marked **[derived]**.

---

## 0. Our hardware, as it actually is

Established by reading the repo (not assumed):

| Fact | Where |
|---|---|
| ESP32 micro-ROS exposes only `/cmd_vel`, `/odom_raw`, `/imu`, `/battery` | task brief; no `/joint_states` publisher found |
| No per-wheel encoder topic, no `/joint_states` | — |
| `/odom_raw` `twist.linear.x` is **sign-inverted** vs its own `pose.position` | `ODOM_TWIST_SIGN = -1`, `rover/fpms_odom_tf.py:387` (mirrored `fpms_teleop.py:150`, `deadband_sweep.py:154`) |
| Pose is dead-reckoned by **differentiating `/odom_raw` pose**, never integrating twist | `rover/fpms_odom_tf.py:1734-1837` |
| Heading comes from **integrating `/imu` `angular_velocity.z`**; IMU orientation quaternion is discarded | `rover/fpms_odom_tf.py:1518, 1689` |
| TF published today: `odom -> base_footprint` only (+ statics) | `rover/fpms_odom_tf.py:1935-1944`; `rover/nav2/fpms_tf.launch.py:208-228` |
| **No `map` frame exists. Nothing corrects pose.** | `rover/nav2/TF_TREE.md:17`; `LIDAR_FIX_ENABLED = False` at `rover/fpms_odom_tf.py:583` |
| `/scan_lidar`, `frame_id=laser_frame`, ~9.83 Hz, 360 bins @ 1°, range 0.12–6.0 m | `rover/fpms_lidar_ros.py:179-183` |
| Scan arrives over **MQTT**, not a direct serial driver | `rover/fpms_lidar_ros.py` |
| LiDAR mount `LIDAR_ZERO_OFFSET_DEG=0.0`, `LIDAR_ROTATION_SIGN=-1` — both marked **UNVERIFIED** | `rover/fpms_lidar_ros.py:164-168` |
| Geometry: wheel dia 70 mm, track 170 mm, 1320 ticks/rev, 0.16657 mm/tick | `rover/fpms_odom_tf.py:368-373` |
| Arena 1200 mm is already a constant in three places | `rover/fpms_odom_tf.py:589`, `fpms_missions.py:277`, `nav2/make_arena_map.py:103` |
| An arena wall-fit localiser is **already written and unit-tested, but disabled** | `manhattan_offset`, `wall_fit`, `arena_fix_from_scan`, `map_to_odom_from_fix` — `rover/fpms_odom_tf.py:1194` ff. |
| `nav2_amcl`, `slam_toolbox`, `robot_localization` installed on the Pi; AMCL configured, **not running** | `rover/NAV2_BRIEF.md:244-248`; `rover/nav2/nav2_params.yaml:82` |

Two of these change the whole answer and are flagged again later:
the wall-fit localiser **already exists and is switched off**, and AMCL is **already configured**
against `nav2/arena_map.yaml` and never launched.

---

## 1. Coordinate frames done properly (REP-105)

**What it is.** REP-105 is the ROS standard that fixes the *names* and *semantics* of
`base_link`, `odom`, `map`, `earth`, and the tree they form.

**Citation.** <https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst>
(also ROS index: <https://www.ros.org/reps/rep-0105.html>)

**The normative sentences that matter here** (verbatim from the REP):

- `base_link` — "The coordinate frame called `base_link` is rigidly attached to the mobile robot base."
- `odom` — "The coordinate frame called `odom` is a world-fixed frame. The pose of a mobile
  platform in the `odom` frame can drift over time, without any bounds. This drift makes the
  `odom` frame useless as a long-term global reference."
- **why `odom` must be continuous** — "the pose of a mobile platform in the `odom` frame always
  evolves in a smooth way, without discrete jumps."
- **why `map` may jump** — "The `map` frame is not continuous, meaning the pose of a mobile
  platform in the `map` frame can change in discrete jumps at any time."
- **tree shape** — "each coordinate frame has one parent coordinate frame, and any number of
  child coordinate frames."
- **which transform localisation is allowed to touch** — "The localization component does not
  broadcast the transform from `map` to `base_link`. Instead, it ... broadcast[s] the transform
  from `map` to `odom`."

**The rule, stated bluntly.** A localisation fix computes `map -> base_link`. It must then
publish

```
map->odom  =  (map->base_link)_fix  *  inverse( (odom->base_link)_odometry )
```

and publish **that**. It must **never** rewrite `odom -> base_link`, because a single frame may
have only one parent and because `odom` is contractually smooth — controllers, local costmaps
and velocity estimators differentiate it and will produce a spike if you teleport it.

**Why this matters for us specifically.** Continuous `odom` is what the local planner and any
obstacle-avoidance loop consume; jumping it mid-leg injects a fake velocity impulse. Discrete
`map` jumps are *expected* and *fine* — that is exactly where a LiDAR fix belongs.

**Do we have what it needs?** Yes, structurally. `rover/fpms_odom_tf.py` already publishes
`odom -> base_footprint` correctly, and the already-written (disabled) fix path is named
`map_to_odom_from_fix` — i.e. the existing design is REP-105-correct. The `map` edge is simply
absent because `LIDAR_FIX_ENABLED = False`.

**Expected accuracy contribution.** None directly — this is a correctness constraint, not an
accuracy technique. Getting it *wrong*, however, silently corrupts everything downstream.

---

## 2. Dead-reckoning accuracy limits, and how to calibrate them

### 2.1 The two error classes

Borenstein & Feng's central distinction: **systematic** errors (kinematic imperfections —
unequal wheel diameters, wrong effective track) versus **non-systematic** errors (slip, floor
irregularity). Systematic errors are *repeatable* and therefore *removable by calibration*;
non-systematic ones are not, and set the noise floor.

**Citation.** UMBmark landing page, J. Borenstein, Univ. of Michigan:
<https://websites.umich.edu/~johannb/umbmark.htm>
Paper: Borenstein & Feng, *"UMBmark: A Benchmark Test for Measuring Odometry Errors in Mobile
Robots"*, SPIE 1995 — <https://johnloomis.org/ece445/topics/odometry/borenstein/paper60.pdf>
Companion: *"Measurement and Correction of Systematic Odometry Errors in Mobile Robots"*,
IEEE T-RA 1996 — <https://johnloomis.org/ece445/topics/odometry/borenstein/paper58.pdf>
Semantic Scholar record:
<https://www.semanticscholar.org/paper/3911b15c805f4276fb368a5f5992b7cd962b60c2>

### 2.2 Why the effective track differs from the geometric one (skid-steer)

On a differential *wheeled* robot, yaw comes from `(S_r - S_l)/b` with `b` the geometric
wheel separation. On a **skid-steer** chassis the wheels cannot roll in the turn direction —
they must scrub laterally — so the vehicle rotates about an instantaneous centre of rotation
(ICR) displaced from the geometric axle. The consequence is that the *effective* track that
makes encoder-derived yaw correct is **larger than the geometric track**, commonly by a factor
of roughly 1.5–2.5x, and it is **not constant**: it varies with surface friction, load, tyre
wear and turn rate.

**Citations.** ICR / separated-ICR skid-steer kinematics and slip modelling:
- Slip-compensated tracked/skid odometry, ROBOMECH J. (open access):
  <https://link.springer.com/article/10.1186/s40648-017-0095-1>
- Online odometry calibration for differential drive under slippage, *Robotics* 13(1):7 (MDPI, open access):
  <https://www.mdpi.com/2218-6801/13/1/7> — canonical URL <https://www.mdpi.com/2218-6581/13/1/7>
- Slip-aware wheel odometry for 4-wheeled skid-steer robots:
  <https://www.researchsquare.com/article/rs-9959755/v1>

**Do we have what it needs? — and a genuinely good piece of news.**
`TRACK_M = 0.170` in `rover/fpms_odom_tf.py:371` is the *geometric* track and would be badly
wrong as an effective track. **But we never use it for heading.** Heading is integrated from the
IMU gyro (`rover/fpms_odom_tf.py:1689`). Using the gyro for yaw **sidesteps the entire
effective-track problem** — the single largest systematic error on a skid-steer chassis — for
free. This is the right architectural choice and it should be preserved, not "fixed" by
switching to encoder-derived yaw.

That also means the classic UMBmark *`Eb` wheelbase* correction is **largely irrelevant to us**:
it calibrates exactly the term we do not use. The *`Ed`/scale* half still matters.

### 2.3 The UMBmark procedure (for completeness and for the scale term)

The canonical procedure, per the sources above:

1. Drive a **square path of side `L`** (Borenstein used **4 m x 4 m**; scale to the arena —
   for us `L` must be well under 1200 mm, so a ~0.8 m square, at the cost of proportionally
   worse resolution on the estimated angles).
2. Run it **5 times clockwise and 5 times counter-clockwise**. Bidirectionality is the whole
   point: it separates two systematic errors that otherwise **mutually mask** each other in a
   one-direction test.
3. Measure the return-position error `(x, y)` of each of the 10 runs against the true start pose.
4. Compute the **centre of gravity** (arithmetic mean) of each five-run cluster:
   `x_cg,cw`, `y_cg,cw`, `x_cg,ccw`, `y_cg,ccw`.
5. Solve for the two error angles (with `Y` forward, `X` right):

   ```
   alpha = (x_cg,ccw - x_cg,cw) / (-4L)          [Type A: same-sign in both directions]
   beta  = -(x_cg,cw + x_cg,ccw) / (-4L)         [Type B: opposite-sign]
   ```
   (equivalently from the `y` values:
   `alpha = (y_cg,cw + y_cg,ccw)/(-4L)`, `beta = (y_cg,cw - y_cg,ccw)/(-4L)`)

6. Derive the corrections:

   ```
   R  = (L/2) / sin(beta/2)                      # radius of the curved-instead-of-straight path
   Ed = (R + b/2) / (R - b/2)                    # wheel-diameter ratio  D_right / D_left
   Eb = 90 deg / (90 deg - alpha)                # effective-wheelbase scale factor
   ```

7. Apply: rescale the two wheel circumferences by `Ed` (preserving mean travel so the *distance*
   scale is untouched), and redefine the wheelbase in software as `b_corrected = Eb * b_nominal`.

   Formula set as reproduced at <https://solderspot.net/2014/05/08/navbot-error-correction-using-umbmark/>
   and <https://thetechnicgear.com/2014/06/howto-calibrate-differential-drive-robot/>;
   original derivation in paper58/paper60 above.

8. Report the residual as `E_max,syst` — the larger of the two cluster CoG radii.

**Quoted benefit.** Borenstein & Feng report "a consistent improvement of at least one order of
magnitude in odometric accuracy (with respect to systematic errors)" for a calibrated robot.

**Do we have what it needs?** **Partly, and the useful part is cheap.**
- The full UMBmark needs per-wheel control of the two wheel diameters to apply `Ed`. We have
  **no per-wheel encoder topic and no `/joint_states`** — we cannot observe or apply a
  left/right split. **`Ed` is not applicable to us.**
- `Eb` calibrates encoder-derived yaw, which **we do not use** (gyro instead). **Not applicable.**
- What **is** applicable and **is** worth doing is the **straight-line scale calibration**: the
  single scalar that maps `/odom_raw` pose-delta to true ground distance. Drive a measured
  1000 mm, read the integrated distance, and correct `MM_PER_TICK` /
  `WHEEL_DIAMETER_M` (`rover/fpms_odom_tf.py:368-373`) by the ratio. Repeat forward and
  backward, 5x each, average. **This is the single highest-value calibration available to us**
  and it is a 20-minute bench job — see the budget in §6, where uncalibrated scale is the
  dominant dead-reckoning term.

**Expected accuracy.** Straight-line scale calibration reduces a typical 1–3 % unmodelled
distance-scale error to roughly **0.2–0.5 %** (limited by tape-measure resolution and tyre
compression variation) **[derived]** — i.e. from 10–30 mm/m down to **2–5 mm/m**.

---

## 3. Sensor fusion: `robot_localization` EKF (wheel odom + IMU)

**What it is.** `robot_localization` is the standard ROS EKF/UKF state estimator. In the
`odom`-frame role, `ekf_filter_node` consumes `/odom_raw` and `/imu`, and **itself becomes the
publisher of `odom -> base_link`**.

**Citations.**
- Package docs (state estimation nodes, parameters):
  <http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html>
- "Preparing Your Data for Use with robot_localization" (the sensor-trust guidance):
  <http://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html>
  (source of truth, plain text, mirror-safe:
  <https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ros2/doc/preparing_sensor_data.rst>)
- Moore & Stouch, *"A Generalized Extended Kalman Filter Implementation for the Robot Operating
  System"*, IAS-13:
  <https://docs.ros.org/en/lunar/api/robot_localization/html/_downloads/robot_localization_ias13_revised.pdf>
- Worked wheel-odom + IMU config walkthrough:
  <https://blog.abdurrosyid.com/2021/07/21/fusing-wheel-odometry-and-imu-data-using-robot_localization-in-ros/>

**What each sensor is trusted for.** The package documentation's own rules:

- "**If the odometry provides both position and linear velocity, fuse the linear velocity.**"
  (Position from odometry is an integral of the same measurement — fusing both double-counts and
  makes the filter over-confident.)
- "**If the odometry provides both orientation and angular velocity, fuse the orientation.**"
- With two yaw sources: "If both produce orientations with accurate covariance matrices, it's
  safe to fuse the orientations." Otherwise fuse only the better sensor, or put the weaker one
  in **differential mode**.
- On covariances: inflating them to suppress a variable is "**both unnecessary and even
  detrimental to the performance of robot_localization**" — disable the variable in the
  `odomN_config` / `imuN_config` boolean matrix instead. No fused variable may have zero variance.
- `two_d_mode: true` for a planar robot — it fuses hard zeros for `z`, `roll`, `pitch` and their
  rates, which is exactly right for us and stops the filter drifting in unobservable DOFs.

**Typical process-noise settings.** `process_noise_covariance` is a 15x15 diagonal-dominant
matrix. The shipped defaults (x,y ≈ 0.05; yaw ≈ 0.06; vx,vy ≈ 0.025; vyaw ≈ 0.02) are a sane
starting point. The tuning rule that matters: **process noise expresses how much you distrust
the motion model between updates**. Raise `vyaw`/`yaw` process noise on a skid-steer chassis,
because scrub makes the constant-velocity yaw model genuinely poor; keep `x`/`vx` process noise
low, because longitudinal wheel odometry on a hard flat floor is good. Tune process noise
*after* the measurement covariances are honest, never before.

**What fusing gyro yaw with encoder yaw actually buys on skid-steer.**
Honestly: on *our* machine, **very little** — because we already use gyro-only yaw and have no
encoder yaw to fuse. The general result is that encoder-derived yaw on skid-steer is corrupted
by the unknown, surface-dependent effective track (§2.2), while gyro yaw is unbiased in the
short term but **random-walks without bound**. Fusing them lets the (locally accurate, slowly
drifting) gyro dominate the short term while the encoder yaw weakly bounds long-term gyro drift.
On skid-steer, though, encoder yaw is so biased that the honest configuration is **gyro yaw
only** — which is what `rover/fpms_odom_tf.py:1689` already does. Neither source is
map-referenced, so **no combination of the two bounds absolute heading error.** Only an
exteroceptive fix (§4) does that.

**Do we have what it needs?**
- `robot_localization` **is installed** (`rover/NAV2_BRIEF.md:244-248`) but **no EKF config
  exists anywhere in the repo**. So: available, unconfigured.
- **CRITICAL BLOCKER, ours specifically:** the documented best practice is *fuse the twist, not
  the pose*. Our `/odom_raw` `twist.linear.x` is **sign-inverted relative to its own
  `pose.position`** (`ODOM_TWIST_SIGN = -1`, `rover/fpms_odom_tf.py:387`). Feeding `/odom_raw`
  straight into `odom0` with `vx: true` would make the EKF integrate motion **backwards**. Any
  EKF adoption **must** be fed by a sign-corrected republished topic, never by raw `/odom_raw`.
  The brief's rule — direction comes from differentiating pose, never from twist — must survive
  into any EKF wiring.
- We have no `/joint_states`, so nothing else is available to fuse.

**Expected accuracy.** An EKF here is a **smoothing and plumbing** win, not an accuracy win:
better-conditioned velocity estimates, proper covariance propagation, and a standards-compliant
`odom -> base_link` publisher. **It cannot reduce unbounded drift**, because every input is
proprioceptive. Expect **0 mm** of improvement in absolute arena position. Do not adopt it
expecting centimetres.

---

## 4. LiDAR-based correction in a KNOWN arena, without SLAM

The arena is a known 1200 x 1200 mm rectangle of walls. That is an unusually favourable case:
the map is exact, closed, small, and Manhattan. No SLAM is needed — SLAM solves for a map we
already have.

### 4.1 Line / wall fitting (split-and-merge, RANSAC, Hough)

**What it is.** Segment the 360-point scan into straight segments, fit lines, and match those
lines to the four known walls. Classic extractors: **split-and-merge** (fastest, best
performance in the standard comparison), **RANSAC** (robust to outliers), **Hough transform**.

**Citations.**
- Nguyen, Martinelli, Tomatis & Siegwart, *"A Comparison of Line Extraction Algorithms using 2D
  Laser Rangefinder for Indoor Mobile Robotics"* (IROS 2005) — the standard reference; finds
  split-and-merge fastest and among the most accurate:
  <https://www.researchgate.net/publication/224623236>
- Split-and-merge segmentation for 2-D range images:
  <https://www.researchgate.net/publication/3887792>
- Feature-based laser scan matching for indoor mapping (open access, PMC):
  <https://pmc.ncbi.nlm.nih.gov/articles/PMC5017430/>

**Needs:** a scan, known wall geometry, and a calibrated sensor mount yaw.
**Have it?** **Yes — and it is already implemented.** `wall_fit` and `manhattan_offset` exist in
`rover/fpms_odom_tf.py` (~line 1194 ff.), self-tested, gated off by `LIDAR_FIX_ENABLED = False`
(`:583`).

### 4.2 Manhattan-frame estimation

**What it is.** Exploit that all walls are mutually orthogonal: estimate a single global
rotation from the *aggregate* distribution of scan-point normals / segment directions, rather
than from any one wall. Extremely robust, and gives yaw independent of position.

**Citations.**
- Straub et al., *"Real-time Manhattan World Rotation Estimation in 3D"* (IROS 2015), MIT open access:
  <https://dspace.mit.edu/bitstream/handle/1721.1/107428/Leonard_Real-time%20manhattan.pdf>
- Manhattan-world structural regularity + line RANSAC for 2D LiDAR indoor layout:
  <https://arxiv.org/pdf/2001.05422> (Indoor Layout Estimation by 2D LiDAR and Camera Fusion)
- Linear four-point LiDAR SLAM for Manhattan worlds (RA-L 2023):
  <https://mpil-gist.github.io/assets/paper/2023_ral_linear.pdf>

**The gotcha nobody mentions until it bites:** a **square** arena is 90°-rotationally symmetric.
A Manhattan estimate is therefore only ever **yaw modulo 90°**. The quadrant must be
disambiguated by the gyro/odometry prior. With a 1200x1200 square there is no geometric way to
break the tie from a single scan — the prior is mandatory. (A 1200x1200 arena is square, so
even wall-length matching cannot break it. If the arena were rectangular, it could.)

**Have it?** Yes — `manhattan_offset` exists in `rover/fpms_odom_tf.py`, and we have a
continuously-integrated gyro heading to supply the quadrant prior.

### 4.3 ICP against the known map

**What it is.** Iterative Closest Point: align the live scan to the known wall polygon by
minimising point-to-line distance, seeded from the current pose estimate.

**Citations.**
- Real-Time 2-D Lidar Odometry Based on ICP, *Sensors* 21(21):7162 (open access):
  <https://www.mdpi.com/1424-8220/21/21/7162> / <https://pmc.ncbi.nlm.nih.gov/articles/PMC8587105/>
- 2D grid map building using ICP and line extraction:
  <https://www.researchgate.net/publication/290573872>

**Needs:** a good initial guess (ICP is local and will happily converge to a wrong 90° rotation
in a square room), and outlier rejection for anything in the arena that is not a wall.
**Have it?** The initial guess, yes (dead-reckoned pose). Point-to-**line** ICP against 4 known
segments is trivial to implement and strictly better-conditioned than generic point-to-point.

### 4.4 Scan matching / AMCL against an occupancy map

**What it is.** Particle-filter localisation (`nav2_amcl`) against a pre-built occupancy grid.
The standard, batteries-included answer.

**Citations.**
- Nav2 AMCL configuration: <https://docs.nav2.org/configuration/packages/configuring-amcl.html>
- Nav2 transforms / REP-105 in practice:
  <https://docs.nav2.org/setup_guides/transformation/setup_transforms.html>

**Realistic accuracy (published).** AMCL translational RMSE is reported at **~8.5 cm in an
empty environment**, degrading to **~33.7 cm in a cluttered real office**; a pose-graph
alternative reached 7.2 cm (<https://arxiv.org/pdf/2308.05443>). BIM-referenced AMCL variants
"maintain errors within 10 cm" (<https://www.sciencedirect.com/science/article/abs/pii/S0263224126006524>).

**Verdict for us: AMCL is the WRONG tool for a ±1 cm target.** Its published accuracy is
~10 cm — an order of magnitude short. AMCL is designed for large, cluttered, ambiguous
environments where the hard problem is *global* ambiguity. Our problem is the opposite: the map
is exact, tiny, and fully observable. A direct **wall-fit / point-to-line ICP is both simpler
and roughly 10x more accurate here** than a particle filter, because it does not quantise the
world into a grid and does not represent the posterior with a finite particle set.
`rover/nav2/nav2_params.yaml:82` already configures AMCL against `nav2/arena_map.yaml` — useful
as a fallback and for Nav2 compatibility, but **it is not the route to ±1 cm.**

### 4.5 Why the wall fit is so accurate here — the geometry **[derived]**

This is the key quantitative argument, so it is shown rather than asserted.

- LD06/RPLIDAR-A1-class 2D LiDAR: single-shot range accuracy **~±15 mm**
  (LD06 datasheet: 15 mm accuracy, 1° angular resolution —
  <https://www.yahboom.net/xiazai/LiDar-LD06/LDROBOT_LD06_Development_manual.pdf>;
  RPLIDAR A1 spec <https://www.slamtec.com/en/lidar/a1spec>).
- Arena is 1200 mm; LiDAR `range_max` is 6.0 m. **All four walls are visible from everywhere in
  the arena, always.** All 360 beams land on a wall. This makes `(x, y, yaw)` **fully
  observable from a single scan** — unlike a corridor, where translation along the corridor is
  unobservable.
- A wall subtends a large arc: from the arena centre a 1200 mm wall at ~600 mm spans ~90°,
  i.e. **~90 points at 1° resolution**.
- Fitting a line to `N` independent points reduces the *random* component of the perpendicular
  offset by `sqrt(N)`:
  `sigma_perp ~ 15 mm / sqrt(90) ~ 1.6 mm`.
- Wall *angle* error for a span `S` with `N` points: `sigma_theta ~ sigma * sqrt(12/N) / S`
  `= 15 * sqrt(12/90) / 1200 rad ~ 0.0046 rad ~ 0.26 deg` per wall;
  averaging four orthogonal walls (Manhattan) roughly halves it to **~0.13 deg**.
- Systematic range bias does **not** average down — but in a *closed* rectangle it largely
  **cancels in position**: a uniform +b mm bias pushes all four walls outward symmetrically,
  which perturbs the inferred *arena size*, not the *centre*. This is a real structural
  advantage of a closed known box.

**Realistic single-scan wall-fit accuracy for this arena: ~±3–5 mm in x/y and ~±0.15–0.3° in
yaw** — comfortably inside a ±1 cm requirement, with margin.

**The two things that will actually ruin it (both ours, both fixable):**

1. **Uncalibrated LiDAR mount yaw.** `LIDAR_ZERO_OFFSET_DEG = 0.0` and
   `LIDAR_ROTATION_SIGN = -1` are marked **UNVERIFIED** (`rover/fpms_lidar_ros.py:164-168`).
   A constant mount-yaw error `d` rotates the *entire* inferred pose about the sensor; across a
   600 mm arena half-width that is a position error of `600 * d`. **1° of uncalibrated mount
   yaw = ~10.5 mm of position error** — on its own, that blows the entire ±1 cm budget.
   Mount yaw must be calibrated to **< 0.5°**. Also, `LIDAR_ROTATION_SIGN` being wrong mirrors
   the scan, which in a *square* arena produces a plausible-looking but wrong fit — a silent
   failure mode. Verify the sign before trusting any fix.
2. **Latency.** The scan arrives at ~9.83 Hz **over MQTT** (`rover/fpms_lidar_ros.py:179`), so
   the true age of a scan is one scan period plus broker transport — call it 105 ms + transport.
   At 0.15 m/s that is **≥16 mm of travel**, i.e. larger than the entire error budget.
   **Mitigation, and it is the decisive design choice: take fixes at rest.** Stop between legs,
   take 3–5 scans, fit, apply the `map -> odom` correction while stationary. Latency error goes
   to **zero**, averaging over scans cuts the random term by another `sqrt(5)`, and the
   discrete `map` jump happens exactly when REP-105 says it is safe. If fixes must be taken
   in motion, the scan must be timestamped and the correction applied against the *time-aligned*
   `odom` pose, never the latest one.

---

## 5. Achieving centimetre accuracy — the honest answer

**Dead reckoning alone cannot deliver ±1 cm absolute arena position. Not over 1 m, not with any
amount of tuning, and not in principle.** Three independent reasons:

1. **The start pose is assumed, not measured.** Every dead-reckoned pose is
   *(assumed origin) + (integrated motion)*. If the rover is hand-placed to ±15 mm and ±3°, it
   is already outside a ±10 mm budget **before it moves**, and the 3° heading error alone
   contributes `1000 * sin(3°) = 52 mm` of lateral error over a 1 m leg. No proprioceptive
   sensor can observe this term.
2. **Error is unbounded by construction.** REP-105 says it outright: pose in `odom` "can drift
   over time, without any bounds." Gyro heading random-walks; wheel scale error accumulates
   linearly with distance. Both grow monotonically. A ±1 cm *bound* requires something that
   *bounds*, and only an exteroceptive, map-referenced measurement does that.
3. **Skid-steer slip is non-systematic.** Per Borenstein & Feng, slip is precisely the class of
   error that calibration **cannot** remove. Every start, stop and turn scrubs an unrepeatable
   amount.

**What dead reckoning *can* deliver on a single, calibrated, straight 1 m leg:** roughly
**±10–20 mm** after straight-line scale calibration, *relative to where the leg started*, in
good conditions. That is genuinely useful for a single leg and completely inadequate as an
absolute arena position after several legs.

**What must be added:** a map-referenced exteroceptive fix. We already have the sensor
(`/scan_lidar`), the map (`ARENA_MM = 1200.0`, in three files), and — remarkably — **the fitting
code, already written and unit-tested and switched off** (`LIDAR_FIX_ENABLED = False`,
`rover/fpms_odom_tf.py:583`).

---

## 6. A REALISTIC ACCURACY BUDGET — per 1 m leg

Errors in millimetres, for one ~1 m straight leg at ~0.15 m/s (~6.7 s).
Lateral error from heading error uses `1000 mm * sin(theta)`; **1° = 17.5 mm**. **[derived
except where a source is cited]**

### 6a. Dead reckoning only (today's system)

| # | Error source | Mechanism | Now | After cheap calibration |
|---|---|---|---|---|
| 1 | **Assumed start position** | hand placement; never observed | **±10–20 mm** | ±10–20 mm (unchanged) |
| 2 | **Assumed start heading** | hand placement; 3° -> 52 mm lateral | **±30–50 mm** | ±30–50 mm (unchanged) |
| 3 | **Wheel/scale factor** | nominal 70 mm dia & 0.16657 mm/tick unverified; 1–3 % | **±10–30 mm** | **±2–5 mm** (straight-line cal, §2.3) |
| 4 | **Gyro bias residual** | 0.02–0.05 °/s over 6.7 s -> 0.13–0.33° -> lateral | ±2–6 mm | ±2–6 mm |
| 5 | **Gyro random walk** | MEMS ARW over one leg | ±1–2 mm | ±1–2 mm |
| 6 | **Accumulated heading from prior legs** | gyro drift is cumulative across the mission | **±10–40 mm** and growing | growing |
| 7 | **Skid-steer slip (start/stop scrub)** | non-systematic; 2 events/leg | ±3–10 mm | ±3–10 mm (irreducible) |
| 8 | **Encoder quantisation** | 0.16657 mm/tick | ±0.1 mm | ±0.1 mm |
| 9 | **Pose-differencing / timing jitter** | discrete `/odom_raw` sampling | ±1–3 mm | ±1–3 mm |
| | **RSS total, single leg** | | **~±40–75 mm** | **~±35–65 mm** |
| | **Absolute error after several legs** | terms 1,2,6 accumulate | **50–150 mm+** | **50–150 mm+** |

**Read this table honestly:** calibration (row 3) is real and worth doing, but it attacks the
*third*-largest term. Rows 1, 2 and 6 — all of them **unobservable without an external
reference** — dominate, and no amount of odometry work touches them.
**Dead reckoning alone: ~±5 cm at best on one leg, 10–15 cm+ absolute. We are 5–15x short of ±1 cm.**

### 6b. With the LiDAR arena fix enabled (stop-and-fix at each waypoint)

| # | Error source | Mechanism | Contribution |
|---|---|---|---|
| 1 | LiDAR range noise after line fit | 15 mm / sqrt(~90 pts), x4 walls, x5 scans | **±1–2 mm** |
| 2 | LiDAR systematic range bias | largely cancels in a closed rectangle | ±1–3 mm |
| 3 | **LiDAR mount yaw calibration residual** | 600 mm lever arm; 0.5° -> 5.2 mm | **±3–5 mm** (dominant; calibrate hard) |
| 4 | Wall-fit yaw residual | ~0.13–0.26° over 4 walls | ±2–4 mm equivalent |
| 5 | Arena model error | are the walls truly 1200.0 mm, truly square? | **±2–5 mm** (go measure) |
| 6 | Non-wall returns (obstacles, cables, other robots) | outlier rejection residual | ±1–3 mm |
| 7 | Latency | **0 mm if fixing at rest**; 16 mm+ if fixing in motion | **0 mm (at rest)** |
| 8 | Dead reckoning *between* fixes | rows 3,4,5,7 of table 6a over ≤1 m | ±5–12 mm |
| | **RSS total, at a fix point** | | **~±5–9 mm** |
| | **RSS worst case, mid-leg between fixes** | | ~±10–15 mm |

**Verdict: ±1 cm is achievable at fix points, and only at fix points.**
Mid-leg, expect ~1–1.5 cm. If ±1 cm is required *continuously* rather than at waypoints, fixes
must be taken in motion with proper timestamping, and the LiDAR's 9.83 Hz MQTT-transported rate
becomes the binding constraint.

---

## 7. What we must add to reach ±1 cm

Ordered by value-per-unit-effort. Nothing here requires new hardware.

1. **Enable and validate the arena wall fit that already exists.**
   Flip `LIDAR_FIX_ENABLED` (`rover/fpms_odom_tf.py:583`) and wire `arena_fix_from_scan` ->
   `map_to_odom_from_fix` -> a `map -> odom` broadcast. **Highest value by a wide margin — the
   code is already written and tested.** This is the only item that converts unbounded drift
   into a bounded error.
2. **Calibrate the LiDAR mount yaw to < 0.5°, and verify `LIDAR_ROTATION_SIGN`.**
   Both are `UNVERIFIED` at `rover/fpms_lidar_ros.py:164-168`. This is the **dominant** term in
   budget 6b — 1° of error alone exceeds the entire ±1 cm budget. Method: place the rover
   squarely against a known wall at a known heading, fit, and solve for the residual offset.
   A wrong `LIDAR_ROTATION_SIGN` mirrors the scan and, in a square arena, fails *silently*.
3. **Adopt stop-and-fix at every waypoint.** Kills the ≥16 mm latency term outright and buys a
   further `sqrt(N)` from multi-scan averaging. Cheapest large win available; costs only time.
4. **Physically measure the arena.** `ARENA_MM = 1200.0` is a design value in three files. If
   the real box is 1204 x 1197 mm, that error goes straight into every fix (row 5 of 6b).
5. **Straight-line scale calibration** of `MM_PER_TICK` / `WHEEL_DIAMETER_M`
   (`rover/fpms_odom_tf.py:368-373`). Cuts row 3 of 6a from 10–30 mm to 2–5 mm, which is what
   makes the *between-fix* interpolation good enough.
6. **Seed the initial pose from a LiDAR fix instead of assuming it.** Directly deletes rows 1
   and 2 of budget 6a — the two largest dead-reckoning terms — for essentially no cost once
   item 1 works.
7. **Quadrant disambiguation guard.** The square arena gives yaw only mod 90°. Gate every fix
   against the gyro prior and **reject** any fix implying a >45° correction; log it loudly
   rather than applying it.
8. **Optional, later: `robot_localization` EKF.** Nice for standards-compliance and covariance
   hygiene; **worth ~0 mm of absolute accuracy**. If adopted, it **must** be fed a
   sign-corrected `/odom_raw` (`ODOM_TWIST_SIGN = -1`, `rover/fpms_odom_tf.py:387`) — feeding
   raw twist makes the EKF integrate motion backwards.
9. **Not recommended for the ±1 cm target: AMCL.** Already configured
   (`rover/nav2/nav2_params.yaml:82`) and useful as a Nav2-compatible fallback, but its
   published accuracy is ~8.5–33.7 cm — 10–30x short. Keep it for robustness, do not rely on it
   for precision.

### Bottom line

**With the current sensors, ±1 cm is achievable — but only with the LiDAR fix enabled and its
mount calibrated, and realistically only at stop-and-fix waypoints.**
Dead reckoning alone, however well calibrated, lands at roughly **±5 cm per leg and 10–15 cm
absolute**, and that gap is structural, not a tuning failure. The good news is that the
expensive part — the wall-fitting mathematics — is **already written, already unit-tested, and
sitting behind a `False` flag.**

---

## 8. Sources

- REP-105, Coordinate Frames for Mobile Platforms — <https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst> · <https://www.ros.org/reps/rep-0105.html>
- Borenstein & Feng, UMBmark landing page — <https://websites.umich.edu/~johannb/umbmark.htm>
- Borenstein & Feng, *UMBmark: A Benchmark Test for Measuring Odometry Errors in Mobile Robots* (SPIE 1995) — <https://johnloomis.org/ece445/topics/odometry/borenstein/paper60.pdf>
- Borenstein & Feng, *Measurement and Correction of Systematic Odometry Errors in Mobile Robots* (IEEE T-RA 1996) — <https://johnloomis.org/ece445/topics/odometry/borenstein/paper58.pdf>
- UMBmark equation walkthroughs — <https://solderspot.net/2014/05/08/navbot-error-correction-using-umbmark/> · <https://thetechnicgear.com/2014/06/howto-calibrate-differential-drive-robot/>
- Slip-compensated odometry for tracked/skid vehicles, ROBOMECH J. — <https://link.springer.com/article/10.1186/s40648-017-0095-1>
- Online odometry calibration under slippage, *Robotics* 13(1):7 — <https://www.mdpi.com/2218-6581/13/1/7>
- Slip-aware wheel odometry for 4-wheeled skid-steer robots — <https://www.researchsquare.com/article/rs-9959755/v1>
- robot_localization, preparing sensor data — <http://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html> · <https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ros2/doc/preparing_sensor_data.rst>
- robot_localization, state estimation nodes — <http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html>
- Moore & Stouch, *A Generalized EKF Implementation for ROS* (IAS-13) — <https://docs.ros.org/en/lunar/api/robot_localization/html/_downloads/robot_localization_ias13_revised.pdf>
- Wheel odometry + IMU fusion walkthrough — <https://blog.abdurrosyid.com/2021/07/21/fusing-wheel-odometry-and-imu-data-using-robot_localization-in-ros/>
- Nguyen et al., *A Comparison of Line Extraction Algorithms using 2D Laser Rangefinder* (IROS 2005) — <https://www.researchgate.net/publication/224623236>
- Split-and-merge segmentation for 2-D range images — <https://www.researchgate.net/publication/3887792>
- Feature-based laser scan matching for indoor mapping — <https://pmc.ncbi.nlm.nih.gov/articles/PMC5017430/>
- Straub et al., *Real-time Manhattan World Rotation Estimation in 3D* (IROS 2015) — <https://dspace.mit.edu/bitstream/handle/1721.1/107428/Leonard_Real-time%20manhattan.pdf>
- Indoor Layout Estimation by 2D LiDAR and Camera Fusion (Manhattan + line RANSAC) — <https://arxiv.org/pdf/2001.05422>
- Linear Four-Point LiDAR SLAM for Manhattan World (RA-L 2023) — <https://mpil-gist.github.io/assets/paper/2023_ral_linear.pdf>
- Real-Time 2-D Lidar Odometry Based on ICP, *Sensors* 21(21):7162 — <https://www.mdpi.com/1424-8220/21/21/7162>
- Occupancy Grid to Pose Graph: robust BIM-based 2D-LiDAR localization (AMCL RMSE figures) — <https://arxiv.org/pdf/2308.05443>
- BIM-based AMCL for indoor mobile robots — <https://www.sciencedirect.com/science/article/abs/pii/S0263224126006524>
- Nav2 AMCL configuration — <https://docs.nav2.org/configuration/packages/configuring-amcl.html>
- Nav2 setting up transformations — <https://docs.nav2.org/setup_guides/transformation/setup_transforms.html>
- LDROBOT LD06 development manual (15 mm range accuracy, 1° resolution) — <https://www.yahboom.net/xiazai/LiDar-LD06/LDROBOT_LD06_Development_manual.pdf>
- SLAMTEC RPLIDAR A1 specifications — <https://www.slamtec.com/en/lidar/a1spec>
