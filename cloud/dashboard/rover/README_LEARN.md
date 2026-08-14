# FPMS run recorder and calibration fitter

## This is not machine learning

There is no AI model here, nothing is trained, and nothing generalises beyond
this one chassis. This is **parameter estimation**: ordinary least squares on a
few dozen numbers, fitting six physical constants that this rover's code
currently disputes or has never measured.

That distinction is not pedantry, it is the honest answer to the request that
produced this work. The ask was for "an AI learning model on the Orange Pi that
the data streams to". **That cannot be built here.** The Pi has the RKNN
*runtime* installed — enough to *execute* a model somebody else compiled — and
nothing else. Authoring or converting a model requires `rknn-toolkit2`, which
runs only on an x86 host, and this project has no x86 host. Anyone who tells you
otherwise is describing a plan, not a thing that exists.

What *does* exist is a genuine learning problem going unserved, and it is worth
more than a model would be: **every run this rover makes already measures the
ground truth needed to fix its own drive constants, and then throws it away.**
`fpms_missions.py` computes commanded-vs-measured for every segment, reports it
once in `events/mission_done`, and nothing keeps it. These two files keep it and
fit it.

## The files

| File | What it does | Where it runs |
|---|---|---|
| `fpms_learn_recorder.py` | Subscribe-only ROS node. One NDJSON record per motion episode. | On the Pi, as a service |
| `units/fpms-learn-recorder.service` | systemd unit for the above | `/etc/systemd/system/` |
| `fpms_learn_fit.py` | Reads the NDJSON offline, fits the constants, refuses when it can't | Anywhere with Python 3.8+ |
| `fpms_missions_residual.patch` | Additive diff making `fpms_missions.py` publish `telemetry/residual` | **Not applied** |

The fitter is pure standard library. No numpy, no scipy, nothing to install.

## The recorder cannot drive the rover

Three structural guards, not a comment promising good behaviour:

1. The node class overrides `create_publisher` to raise. There is no path in
   rclpy that puts a message on a topic without one.
2. A once-per-second audit kills the process if `self.publishers` is ever
   non-empty — which would catch a publisher created through some future rclpy
   internal that bypassed (1).
3. The MQTT client class overrides `publish()` and `will_set()` to raise. The
   only MQTT traffic it generates is SUBSCRIBE.

The unit declares no ordering against `fpms-missions`, `fpms-teleop` or
`micro-ros-agent`, deliberately. A recorder that can delay or restart the thing
it records is worse than no recorder.

## What it can determine

**`counts_per_mm` — yes, with a caveat about the reference.**
Currently disputed between 5.5 (hand push, 1000 mm tape, motors off), 6.0
(derived from `TICKS_PER_REV = 1320`) and 14.8 (taped 2026-08-04, unconfirmed).
This is fittable, *but only against a length reference that does not come from
the encoders.* Encoders cannot calibrate encoders.

- **Tape measure** (`truth` records appended by hand) — traceable to a real
  metre. The gold standard.
- **LiDAR front-wall closure** across a straight run at a flat wall — genuinely
  independent of the drivetrain, but noisy. Because the reference then carries
  error of its own, errors-in-variables biases an OLS slope *toward zero*, so a
  LiDAR-only answer is an **under-estimate** by an unknown amount. The fitter
  says so on every LiDAR-only result.
  **Caveat found while testing (2026-08-07): `/scan_lidar` does not deliver.**
  The topic is advertised, `fpms-lidar-ros` logs 9.5 Hz with 0 drops, and there
  are two subscribers — and *nothing receives a message*. `ros2 topic echo
  --once` sat silent for 20 s on a topic with a live publisher. Discovery works;
  data does not flow. This is a separate fault and was not chased (read-only
  session), but it means the ROS LiDAR reference is **currently unavailable**,
  which leaves the tape measure as the only working encoder-independent
  reference today. The recorder captures the `front_mm` from the MQTT
  `telemetry/mission` path as well — that one goes through `fpms-rover-agent`
  and *does* work — and the fitter falls back to it automatically.

- **`/odom` pose delta — refused.** The board integrates that from these same
  encoders using its own `COUNTS_PER_REV`. Regressing ticks on odom recovers the
  *firmware's assumption*, produces an r² of 1.000, and is confidently wrong.
  The fitter computes it anyway as a **consistency check**: if it does not come
  out at the flashed constant, the odom pipeline has a second fault on top of
  the scale dispute.

### `counts_per_mm` is meaningless without the wheel-combining rule

This fell out of testing the fitter against synthetic data with a known answer,
and it is not obvious. There are three ways this codebase turns four wheel
deltas into one distance, **they give different constants, and the constant is
only valid for the one it was fitted on**:

| Rule | Used by | Behaviour with one persistently high wheel |
|---|---|---|
| `median4` — mean of the 2nd and 3rd sorted deltas | `fpms_drive.dist_mm` | **Biased high, and the bias grows with wheel noise** |
| `front2` — mean of the two front wheels | `fpms_drive.tick_yaw` | Stable |
| `mean4` — plain mean | — | Carries the full slip of the worst wheel |

`median4` was chosen in `fpms_drive.py` because it is "immune to one bad wheel
in either direction". That is true of the *value* but not of the *bias*. With
the rear-right reading +24% under power, the median of four degenerates to the
mean of the top **two** of the remaining three, so it rides upward as the wheels
get noisier — measured over 200k trials at that slip profile:

```
inter-wheel scatter   median4    front2
      1%              +0.6%      +0.0%
      3%              +1.3%      +0.0%
      6%              +2.6%      +0.0%
```

Against a 5.5-vs-6.0 dispute that is 9% wide, a 1–3% estimator bias is a real
fraction of the answer. So the fitter reports `counts_per_mm` under **all three
rules**, and flags the spread. A constant fitted on `front2` and pasted into code
that measures with `median4` is wrong by that spread, in a direction that gets
worse as the drivetrain wears.

**Take the number matching the rule your consuming code uses.** For
`fpms_drive.py` today, that is `median4`.

**Per-wheel slip factors — yes, and this needs no external reference at all.**
Each wheel's ratio to the signed median of four is dimensionless, so it is the
one thing here measurable today with no tape, no gyro and no wall. Forward and
reverse are fitted separately on purpose: a wheel reading 1.24 forward and 0.81
reverse is not slipping, it is wired or geared asymmetrically, and pooling the
two reports a clean 1.02 and hides a real fault. This matters here — the
rear-right wheel already read +24% against its neighbours on a watched floor
run, and on this rover a per-wheel anomaly has once turned out to be a wheel
coming off.

**The duty→velocity curve and the stall threshold — yes, with the right runs.**
Fitted in two stages so that coast does not corrupt the answer. Every burst is
command → cut → **coast** → stop, and the recorder measures after the coast, so
`distance = v · t_commanded + coast`. Stage 1 regresses distance on duration
*within* each duty level: the slope is the steady-state velocity and the
intercept is the coast, separated out rather than smeared into the velocity.
Stage 2 regresses those velocities on duty; the x-intercept is the stall. The
result is then checked against direct observation — the highest duty that
produced no motion must lie below the fitted stall, and the lowest duty that
did produce motion above it. A fitted stall that contradicts a burst somebody
watched is reported as an inconsistency, not averaged in.

Coast itself falls out of stage 1 for free, and this rover has only ever guessed
at it (`COAST_MM = 40.0  # conservative`, against 86 mm measured once at duty 70).

**`MIN_PULSE_S` under load — as a bracket, not a point.**
The configured 0.35 s was measured **wheels off**, and the code already says so
(`MIN_PULSE_MEASURED_UNDER_LOAD = False`). Under load it can only be longer.
The fitter reports the longest commanded pulse that moved nothing and the
shortest that moved. A bracket is the honest form of this answer because the
transition is a probability, not an edge — the same 0.30 s burst moves on a warm
battery and does not on a cold one.

**The stall duty and the minimum pulse are confounded, and the fitter separates
them.** A burst that moved nothing did so for one of two entirely different
reasons: the duty was below breakaway, or the pulse ended before the velocity
loop got going. Feeding a too-short burst at a healthy duty into the stall
estimator makes it conclude that a duty which demonstrably drives this rover is
below its own breakaway — which is exactly what the synthetic dataset caught it
doing before the split was added. Only bursts of ≥0.60 s count as stall
evidence; shorter ones are routed to the `MIN_PULSE_S` estimator, which in turn
only considers duties already *seen to move this rover*. If the resulting
bracket comes out inverted — a longer burst failing while a shorter one
succeeded at a working duty — the fitter refuses and says so, because that means
something other than duration decided those outcomes (battery sag, a latched
estop, or a duty sitting near breakaway where the result is a coin flip).

## What it cannot determine

**Every turn-related parameter, until the gyro deadband is fixed.**

`effective_track_mm` is the constant that makes `dyaw = (d_right − d_left) /
track` true. Fitting it requires a measured `dyaw` from something that is not
the wheels. On this build there is nothing:

- `/imu` `angular_velocity.z` is **identically 0.0** — ~230 samples measured on
  2026-08-06, all zero, *including through a confirmed hand rotation of the
  whole chassis*. The chip, the accelerometer, the register configuration and
  the register base were each verified good; the fault is the gyro register read
  itself. **Every turn in this dataset measures 0 degrees.**
- `/odom` yaw is integrated on the board from these same wheels using the
  board's own track. Circular twice over.

So the effective track has **four conflicting values live in this codebase**
(105 mm in `fpms_rtos_follower.py`, 170 mm geometric in `fpms_odom_tf.py`,
255 mm effective in `fpms_drive.py`, 271 mm in older notes) and this dataset
cannot arbitrate between any of them.

Everything downstream is stuck behind the same wall:

- **`TURN_WIRE_SIGN`** (`_MEASURED = False`; `/etc/fpms/config.env` sets `+1`
  while the code derives `−1`) cannot be confirmed from data in which every turn
  reads zero.
- **`TURN_RADPS`**, and therefore **`MIN_TURN_DEG` = `TURN_RADPS × MIN_PULSE_S`**,
  is unmeasurable for the same reason.

The estimator for the track **is written**, is exercised against the data
format, and **refuses** — naming the gyro. It starts returning numbers on the
first dataset recorded after the deadband is fixed, with no change to any of
this code and with the drives already banked still usable. That is the whole
reason the recorder logs `/imu` even though every sample is currently 0.0.

**Manual escape hatch:** a `truth` record carrying `true_deg` — an operator
marking the floor and rotating the rover by hand — is accepted by the track
estimator. Twelve of those would unblock it without a firmware fix. Note that
the project rule is *never turn-test this rover for liveness*, because it
destroys heading; these must be deliberate, measured, operator-supervised turns,
not a probe.

## The refusal rule

Every estimator can be run on three samples and will print a number with a
flattering standard error, because with three samples the residual variance is
estimated from one degree of freedom and means nothing. So each estimator carries
two gates and must pass **both**:

1. **A hard floor** on sample count *and on the design*. A perfect regression
   through five points that all sit at the same commanded distance determines
   the slope not at all, however small its residual looks. The fitter checks the
   lever ratio `max|d| / min|d|` and rejects a design without one.

2. **A resolution gate.** The 95% CI must be *narrower than the dispute it was
   run to settle*. Estimating `counts_per_mm` as 5.7 ± 0.9 does not choose
   between 5.5 and 6.0 — it contains both, and reporting "5.7" would retire an
   open question by rounding. Where a parameter has competing published values,
   the required precision is derived **from that gap**, and the fitter reports
   how many more segments the *observed* scatter says are needed to reach it.

A refusal names what is missing and how much more of it is needed. **A refusal
is a result**, and no `REFUSED` value may be copied into `/etc/fpms/config.env`
or `fpms_missions.py`.

## Minimum runs before each parameter becomes identifiable

A "segment" is one motion episode: one burst, its coast, and the measurement
taken at rest afterwards. A single mission produces roughly 6–12 of them, so
these numbers are smaller in wall-clock terms than they look.

| Parameter | Segments | Design requirement | Blocked on |
|---|---:|---|---|
| `counts_per_mm` | **16** | ≥3 distinct distances, lever ratio ≥2.5, ≥3 encoder-independent references. The floor rises automatically if the observed scatter is worse than ~8%. | A tape measure or a flat wall |
| per-wheel slip | **12** | ≥4 forward **and** ≥4 reverse | Nothing — measurable today |
| duty→velocity + stall | **17** | 5 duty levels × 3 repeats at ≥2 distinct pulse durations, **plus** ≥2 bursts at a duty below breakaway that move nothing | Nothing — measurable today |
| `MIN_PULSE_S` under load | **~10** | 6 distinct durations straddling the threshold, ≥3 of each outcome, wheels loaded on the floor | Nothing — measurable today |
| `effective_track_mm` | **12 turns** | ≥20° each, ≥4 per direction | **The gyro deadband** |
| `TURN_WIRE_SIGN` | — | one unambiguous measured turn | **The gyro deadband** |

The 16 for `counts_per_mm` is not a ritual number. With per-segment scatter
around 8% — which is what a chassis whose rear-right wheel slips 24% produces —
a 95% CI half-width of 4.5%, i.e. just enough to separate 5.5 from 6.0, needs
n ≈ (t·σ/target)² ≈ 14. Sixteen, with two spare for the segments the straightness
gate rejects. The fitter recomputes the real requirement from the *observed*
scatter and reports it, so this is a starting point, not a promise.

**Cheapest path to the most value:** one mission of six straight legs at three
different lengths, driven at a flat wall, with an operator taping two of them.
That is a single run and it moves `counts_per_mm`, all four slip factors and the
coast constant from "disputed" to "measured".

## Adding ground truth

The recorder never invents a truth record. Append one by hand to the day's file:

```json
{"rec":"truth","ts":1754500000.0,"seq":41,"true_mm":802.0,"note":"tape, start mark to stop mark, 2 people"}
```

`seq` is the episode number the recorder prints and stores. For a turn, use
`"true_deg"` instead of `"true_mm"`.

## Install and run

Nothing below has been done — the rover was left untouched.

```sh
# On the Pi
sudo install -m0644 fpms-learn-recorder.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now fpms-learn-recorder

# Smoke test without installing anything (60 s, echoes to stdout)
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=20 python3 /tmp/fpms_learn/fpms_learn_recorder.py --once --print

# Fit, once there is data
python3 fpms_learn_fit.py /home/ubuntu/fpms_learn_data --verbose --json fit.json
```

`ROS_DOMAIN_ID=20` lives in the **unit**, not `/etc/fpms/config.env`, which does
not set it and never has — same as every other `fpms-*.service`. On any other
domain this node sees no topics and silently writes an empty dataset, which is
the worst possible failure for a recorder because it looks exactly like a rover
that never moved. The recorder warns loudly at startup if the domain is wrong,
and writes a heartbeat record every 60 s carrying per-topic message rates, so a
dataset recorded with a dead `/wheel_ticks` is self-evidently bad rather than
quietly bad.

## One thing found along the way

`fpms_missions.py` exists in **two diverged copies**:

- `cloud/dashboard/rover/fpms_missions.py` — 6315 lines, **has** `_publish_residual`
- `fpms-pi.local:/home/ubuntu/fpms_missions.py` — 6287 lines, **does not**

`test_stack.py` reads the first. Its assertion *"missions publishes the
per-segment residual stream"* **passes**. The rover runs the second, which has
never published a residual — which is why the six `/fpms/residual/*` topics
exist, are advertised, are mirrored by the foxglove bridge, and have never
carried a message.

So this was never a missing feature. It is a **deployment gap wearing a missing
feature's clothes, and the green gate is what has been hiding it.** The two
copies differ in 33 regions in *both* directions — the mirror has a
calibration-profile block (`ODOM_SCALE`) the Pi lacks; the Pi has gyro liveness
checks and a `BatteryState` subscription the mirror lacks — so neither is simply
"newer" and neither can be copied wholesale over the other. Reconciling them is
a separate and larger job than this one, and it should happen before the next
competition.

`fpms_missions_residual.patch` backports the mirror's own already-written
`_publish_residual` onto the deployed copy (dropping the one field that
references `ODOM_SCALE`, which does not exist there). 87 lines added, none
removed, none modified. It is **not applied**.
