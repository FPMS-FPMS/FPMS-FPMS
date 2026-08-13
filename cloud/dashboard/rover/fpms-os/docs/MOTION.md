# Continuous motion control

`/usr/local/lib/fpms/fpms_motion.py` — the control math for driving this rover
smoothly instead of in bursts.

This document is the argument, not the API. It explains why the rover moves the
way it does today, what firmware v3 makes possible, what the new controller
actually computes, where every gain came from, when it refuses to run, and —
at length, because it matters more than the rest — what has never been tested.

> **Nothing in this document has been verified on a rover.** No wheel has turned
> under this code. Every number below is either measured on *other* code paths,
> derived from a model, or an explicit policy choice. Section 9 is the honest
> list and it is long.

---

## 1. Why the rover moves in bursts today

The mission executor drives like this:

```
burst → STOP → settle → measure at rest → residual → correct → repeat
```

That looks timid. It is not. It is the only thing that works on the firmware
the rover has been running, and the reason is one constant in Yahboom's factory
build:

> `PWM_MOTOR_DEAD_ZONE` = 200 of a 400-tick scale, added as **feed-forward** to
> every velocity command.

So *any* non-zero `cmd_vel` becomes about 50 % duty. Amplitude is discarded
before it reaches the wheels. The consequences follow arithmetically:

| Quantity | Factory firmware |
|---|---|
| Smallest expressible non-zero speed | ≈ **0.65 m/s** |
| Smallest expressible move | ≈ **227 mm** |
| Effect of commanding 0.06 m/s for 1.5 s | travelled **over 1 m** (measured) |

**Slow motion there is not difficult. It is unrepresentable.** Given a
drivetrain with exactly one usable speed, the only remaining control variable
is *how long the wheels are on and how often they are off* — which is precisely
what the burst loop manipulates. Every constant around it is a consequence:

- `MIN_PULSE_S = 0.35` — a burst shorter than this ends before the wheels turn.
- `MIN_MOVE_MM`, `MIN_TURN_DEG` — that pulse times the cruise speed, and that
  pulse times the turn rate. Both *derived*, not typed.
- `DOCK_STEP_MM`, `STOP_SETTLE_S` — slowness bought from the stops, not from
  the speed.

The executor also earned some hard rules that survive into the new path:

- **Measure at rest.** A measurement taken while moving, over MQTT at ~10 Hz,
  is worth very little.
- **Signed comparisons, never `abs()`.** A turn driven the wrong way reaches
  the target magnitude just as happily as a correct one, and looks like a
  success in every log.
- **Bias is worse than noise.** An error that appears in the same direction
  every time cannot be cancelled by retracing, because it reappears identically
  on the way back. This is why B8B cut turns at 93 % and coasted.

`CMD_SCALE = 6.1` belongs to this era too, and it is worth naming as a warning:
it is a **defect artefact of a saturated dead-zone path, not a calibration**.
It does not appear anywhere in the new controller and must never be carried in.

---

## 2. What firmware v3 changes

`firmware_v3/fpms_config.h` is explicit — the dead zone is gone and must never
be reintroduced. The board becomes a dumb duty amplifier:

| | Factory | Firmware v3 |
|---|---|---|
| Primary actuation | `/cmd_vel` through a PID and a dead zone | **`/cmd_duty`**, `data[4]`, −100…100 **percent**, order `[FL, FR, RL, RR]` |
| Velocity path | always live | compiled in but **disarmed at boot** |
| Encoder feedback | integrated odometry only | **raw per-wheel ticks at 25 Hz** |
| `/odom_raw` position sign | inverted | correct |
| Gyro | dead (register non-contiguity bug) | real ICM42670P, 20 Hz |
| Deadman | — | 300 ms, enforced in the one function allowed to touch a motor |

Two consequences make this controller possible at all:

1. **A duty of 8 % really is 8 %.** A speed can be commanded, held, and trimmed.
2. **`/wheel_ticks` is raw.** `counts_per_mm` can be re-derived from a tape
   measure without a reflash — which matters, because that constant is the
   longest-running open number in the project.

**This controller therefore targets firmware v3 and `/cmd_duty` only, and
refuses to produce a command on anything else.** See section 5.

---

## 3. What the module is, and what it is not

`fpms_motion.py` is pure. No ROS, no serial, no clock, no globals, no file I/O.
Time is a parameter everywhere and every controller is

```python
new_state, output = controller.step(state, t, measurement)
```

with frozen dataclasses for state, so an entire trajectory can be replayed
offline against a synthetic plant. That is how the bugs in section 8 were
found, and it is the same contract `fpms_scanmatch.py` already keeps.

It is **not** a ROS node, it does **not** own safety (the obstacle cone, the arm
gate, the stop path and the board's deadman all live elsewhere), and it does
**not** replace burst-stop-settle everywhere. Below a floor set by
`min_moving_duty` a move genuinely cannot be profiled — `TrapezoidalProfile`
reports `is_burst` and the honest answer there is still a bounded pulse and a
measurement.

---

## 4. The control structure

Four loops. Feedforward carries the motion; feedback only trims it.

```
          ticks ─────────────► travel_mm_from_ticks ──► median mm ─┐
                                    (+ spread, a diagnostic)       │
                                                                   ▼
  distance ──► TrapezoidalProfile ──► s_ref, v_ref ──► DistanceController ──► v
                    │                                        ▲
                    └──► braking envelope (caps v) ───────────┘

  gyro wz ──► integrate_gyro ──► yaw ──► HeadingHold ────────────────────► ω
                                     └─► TurnController (turns) ─────────► ω

                          (v, ω) ──► mix_duty ──► [FL, FR, RL, RR] ints
```

### 4.1 The trapezoidal profile

Rest to rest over a distance `D`, cruise `V`, acceleration `a`, duty floor
`v_min`. Three cases:

```
d_ramp = V² / 2a

TRAPEZOID   D ≥ 2·d_ramp        v_peak = V
TRIANGLE    D < 2·d_ramp        v_peak = √(a·D)
BURST       v_peak < v_min and D < v_min²/a
```

The burst case is named honestly because it is the boundary at which
*"continuous motion where the hardware permits it"* stops permitting: the
chassis cannot accelerate to a speed it can express and stop again inside the
distance available. The command degenerates to a square pulse at `v_min` and
the rover arrives carrying energy. Hand those to the measure-and-correct
executor.

Two outputs, and using both is the point:

- **`at_time(t)`** → the ideal reference `(s_ref, v_ref)`. Feedforward, and the
  position the trim closes on.
- **`v_envelope(s)`** → the fastest speed from which the move can *still* stop
  at the target, as a function of **measured** position:

  ```
  rem = D − s
  v_env = −a·τ + √( (a·τ)² + 2·a·rem )        (τ = actuator lag)
  ```

  With `τ = 0` this is the textbook `√(2·a·rem)`. **The lag term is not
  optional** — see section 8.3. The envelope is what makes arrival
  non-overshooting even when the time reference is wrong: if the rover lags its
  profile (low battery, carpet, an `a_max` guessed too high) the reference keeps
  advancing and the trim keeps asking for more speed, and this cap removes
  exactly as much of it as the remaining distance cannot absorb.

Finally `command_v` applies three rules in order: inside the stop band command
exactly zero; never exceed the envelope; never command `0 < v < v_min`.

### 4.2 Distance control

```
s_ref, v_ref = profile.at_time(t − t₀)
e            = s_ref − s_measured                         [mm]
I           ← bounded, conditional  (see 4.5)
trim         = clamp( Kp·e + I , ±trim_max )              [m/s]
v_cmd        = command_v( v_ref + trim , s_measured , stop_band )
```

`s_measured` is the **median** of four per-wheel tick deltas, never a twist,
never a velocity (section 6.1).

A pure PID on remaining distance would fight the profile: early in the move the
error is large *by construction* — the profile has not arrived either — and a
PID reads that as something to fix, commanding exactly the step the ramp exists
to avoid. So the trim is bounded to a fraction of cruise and can never become
the primary signal.

### 4.3 Heading hold on straight runs

```
e = wrap_pi(yaw₀ − yaw)
ω = clamp( Kp·e − Kd·ω_measured , ±ω_max )
```

**No integral, deliberately.** An integrator on heading would integrate gyro
bias into a permanent steer over a long leg, and `gyro_scale` is unmeasured so
the size of that steer is unknown too. A straight-line hold has no persistent
disturbance needing one — a crooked chassis shows up as `lr_asymmetry` in the
mixer, which is a measurement, not a wind-up. P-only also cannot wind up, which
matters because this term keeps running while the linear axis is saturated.

**The damping term is rate feedback, not a derivative.** `−Kd·ω_measured` uses
the gyro's own rate output, so nothing differentiates a noisy integral and
there is no filter to tune.

**This loop runs without a measured `gyro_scale`,** and that is a real result
rather than a concession: it regulates an error to *zero*, so multiplying the
sensor by any positive constant leaves the equilibrium unchanged and only moves
the effective loop gain, which the clamp bounds. What the scale cannot excuse is
a **sign**: an inverted heading hold is positive feedback and the rover spirals
while every logged number stays plausible. That is why the mixer refuses without
`side_map_confirmed`.

### 4.4 Turn in place — "perfect turning" is two problems

**No overshoot → damping, deliberately overdamped.** Model the plant near the
target as an integrator behind a first-order actuator lag `τ`:

```
τ·θ̈ + θ̇ = ω_cmd ,      ω_cmd = Kp·e − Kd·θ̇

τ·s² + (1 + Kd)·s + Kp = 0
ωₙ   = √(Kp/τ)
ζ    = (1 + Kd) / (2·√(Kp·τ))
Kd   = 2·ζ·√(Kp·τ) − 1
```

With `Kp = 2.0 s⁻¹`, `τ = 0.15 s`, `ζ = 1.2`:

```
Kd = 2·1.2·√(0.30) − 1 = 2.4·0.548 − 1 = 0.315 s
```

Why overdamped rather than critical: an overshoot is a **bias**, and a bias is
the one error a retrace cannot cancel. That is the same reasoning behind B8B's
`_COAST = 0.93`, and it is worth more than the fraction of a second `ζ = 1.2`
costs over `ζ = 1.0`.

Why an *unmeasured* `τ` is tolerable in the damping: `ζ ∝ 1/√τ`, so if the true
actuator is **faster** than the assumed 0.15 s the loop becomes **more** damped,
not less. Assuming a slow actuator is the safe direction and it is the direction
assumed. **The envelope is the opposite** — see section 9.

**No hunting → a structural terminal band plus a latch.** The band is derived
from three physical floors, never typed:

```
err_floor   = ω_min · dt        smallest angle one tick at the duty floor makes
coast_floor = ω_min · τ         angle carried after a cut at the floor rate
tol         = max(caller's tolerance, err_floor, coast_floor)
```

Nothing smaller than `err_floor` is expressible, so nothing smaller is chased.
And the band **latches**: the first time the turn enters it, the controller
commits — commands zero, waits for stillness, and *reports* whatever residual
remains. It never re-opens a correction. That makes "no hunting" a property of
the structure rather than of the gains: after the latch there is no loop left
that could oscillate.

**Settle** requires both `|e| ≤ tol` **and** `|ω| ≤ 0.02 rad/s`, held for 0.2 s.
The stillness threshold sits above the board's ±0.01 rad/s deadband because **a
healthy parked gyro reads exactly 0.0** — zero means still, not broken. And the
converse rule: *never command a turn to find out whether the rover is alive.* It
destroys the heading the mission depends on and proves nothing that watching the
pose passively does not.

**There is no coast cut.** B8B cut turns at 93 % because at `TRN = 50` duty with
nothing slower commandable there was no other way to avoid a repeatable
overshoot. This profile decelerates under control, so cutting early would only
trade a repeatable overshoot for a repeatable undershoot — the same bias,
mirrored. See section 7.3.

### 4.5 Anti-windup and rate limiting

Every integrator in the module goes through one function; every duty step
through another. There are no others.

`integrate_bounded` has three protections:

1. **Hard clamp** on the accumulated term, so a stuck rover cannot store an
   unbounded correction and release it as a lurch when it frees.
2. **Freeze in the direction of saturation.** At the duty ceiling more integral
   buys no output and only has to be unwound later — the classic overshoot
   after a slow start. Integration *away* from saturation still runs, so
   recovery is immediate. The mixer reports `saturated_sign` for exactly this,
   in the chassis's own sense of forward, so the controller never needs to know
   about wiring polarity.
3. **Freeze during acceleration.** There the tracking error is dominated by
   unmodelled motor lag rather than by any persistent disturbance, and
   integrating it delivers a step of surplus speed exactly at cruise onset.

`rate_limit` bounds every wheel's duty change per tick. The limit is **derived
from `a_max`**, not typed:

```
max_duty_step = min( policy ceiling , a_max · dt / mps_per_duty )
```

so there is one acceleration limit and not two. Without that tie the rate
limiter quietly becomes the real acceleration limit whenever it is the tighter
of the pair, and the profile a test verified is not the profile the rover ran.

**One deliberate exception:** `stop_command()` is not rate limited. Cutting to
zero is always the safe direction and is the only command valid in every state,
on any firmware, with no calibration at all.

### 4.6 Duty mixing

```
common       = v / mps_per_duty
differential = ω / radps_per_duty
left  = common − differential          right = common + differential
```

REP-103 signs: `+v` forward, `+ω` counter-clockwise from above.

**The angular half does not go through track width, on purpose.**
`LR_WHEELS_DISTANCE` is UNMEASURED — the operator measured 105 mm front-to-back,
which is the *wheelbase* — and a skid steer's effective track is not its
geometric one anyway, because four non-steered wheels must scrub to rotate.
Measuring rad/s per unit differential duty gives the number geometry was only
ever a proxy for.

Then, in order:

- **`lr_asymmetry` as a ratio correction**: `left /= √k`, `right *= √k`. This
  corrects the ratio by `k` while leaving the geometric mean duty — and so the
  commanded speed — unchanged. Putting the whole correction on one side would
  silently change the speed as well as the balance, and the distance loop would
  spend its authority undoing a trim.
- Per-wheel trims (default unity — see section 7.1).
- `duty_forward_sign`, applied last to all four together, because it describes
  the flashed binary's polarity, not the geometry.
- **Headroom policy: when it will not all fit, heading wins.** The common term
  is scaled down first and the differential preserved. Losing speed costs time;
  losing the differential costs the heading, and a heading error integrates into
  a position error that grows for the rest of the leg.
- **Group scaling, not per-wheel clipping.** If trims push one wheel over the
  ceiling, all four scale together. Clipping the offender alone would destroy
  the left/right ratio just corrected — and a duty imbalance is a *steer*: the
  rover would curve away at full speed while every commanded number still looked
  right.
- Wheels commanded non-zero but below `min_moving_duty` are **reported, not
  silently raised**. On a differential command a genuinely slower side may sit
  below the floor, and forcing it up corrupts the very differential that steers.
  "The inside wheel is not turning" is a real explanation for a curve that came
  out wrong, and it should be visible.

---

## 5. Refusal conditions

A refusal is a successful outcome of the design. The alternative — emitting
motion the controller cannot control, or closing a loop on a constant nobody
measured — is how this project produced confident wrong numbers before.

| Refusal | Trigger | Why |
|---|---|---|
| `firmware is not v3` | `/fpms_health[0] < 30000` | The 50 % dead zone makes a profiled move unrepresentable. A trapezoid cruising at 0.08 m/s with a 65 mm ramp and a millimetre-scale terminal band has **no representation** there. Running it anyway produces 0.65 m/s lurches with a feedback loop confidently reporting that it is correcting them. |
| `no /fpms_health evidence` | topic silent, or fewer than 12 fields | **No evidence is not evidence of v3.** On this stack Fast DDS shared memory has let discovery succeed while no data flowed at all, so "I have not heard otherwise" is exactly what a transport failure produces. A short array is a different message, not a partial one. |
| `/fpms_health is stale` | newest sample > 2 s old (published at 2 Hz) | A health field that lies is worse than no health field. |
| `estop is latched` | flags bit0 | Clearing it is an operator action, not a controller action. |
| `the board is on the velocity path` | `active_path == 2` | This controller owns `/cmd_duty`. Two writers to one chassis is how a runaway hides. |
| `the IMU did not initialise` | flags bit3 clear | Heading comes entirely from the gyro; `/odom_raw` carries an identity quaternion and there is no wheel-derived heading to fall back on. |
| `required measured constants are missing` | any needed key absent or refused | Lists **every** missing key at once, so a bring-up produces one shopping list rather than five successive refusals. |
| `heading has no measured gyro_scale` | turning on an unscaled integral | Heading *hold* tolerates an unknown scale; arriving at a *specific* angle does not. |

**Not refused, deliberately** — and each of these would have been a mistake:

- `deadman_firing` (flags bit2). That is the *normal* state before the first
  command. Refusing on it would refuse every cold start.
- `enc_moved_mask == 0`. A parked rover has moved no wheels, by definition.
- A gyro reading exactly `0.0`. It is healthy; there is a ±0.01 rad/s deadband.

### What is required, and what happens when it is absent

Every physical constant is `Optional` and starts as `None`. There is no
`except: pass` in the module and no plausible substitute for a measurement
anywhere in it.

| Key | Needed by | Status today |
|---|---|---|
| `counts_per_mm` | anything closing on distance | **MEASURED ≈ 5.5**, caveated |
| `gyro_scale` | `TurnController` only | **UNMEASURED → all turns refuse** |
| `mps_per_duty` | everything | **NEW KEY — nothing writes it yet** |
| `radps_per_duty` | turns, heading bias | **NEW KEY** |
| `min_moving_duty` | the profile floor | **NEW-ish** — the example ships `0.0`, which is refused |
| `duty_forward_sign` | the mixer | **NEW KEY**, and binary |
| `side_map_confirmed` | the mixer | **NEW FLAG**, operator's eyes |
| `lr_asymmetry`, `wheel_trims` | trim only | optional; absent = unity |
| `coast_mm`, `coast_deg` | **nothing** | parsed for diagnostics, not applied |

Three notes on that table:

- **`min_moving_duty: 0.0` is refused.** A chassis with no stiction floor does
  not exist, so `0.0` is not a measurement of zero — it is the absence of a
  measurement. Adopting it would let the profile command duties that cannot turn
  a wheel while the integrator wound up against a stationary robot: the dead
  zone's failure, arrived at from the other direction.
- **A profile not marked `"firmware": "v3"` gets `mps_per_duty`,
  `radps_per_duty` and `min_moving_duty` refused** while `counts_per_mm`
  survives. A hand push measures the same number through any firmware because
  the wheels are passive; a duty-to-speed number measured through a dead zone is
  an artefact of saturation. That is what `CMD_SCALE = 6.1` was.
- **`counts_per_mm` is required, never defaulted.** The values on record
  disagree by up to 2.7×: **5.5** measured by hand push, **6.00** implied by
  `fpms_duty_driver.py:114`'s `TKMM = π·70/1320` — a 9 % overshoot on its own —
  and **14.8** from an earlier bare-metal build. One factor explains the 2.3 m
  overshoot, the 744 mm plan that drove ~2 m, and the 300 mm move that went
  ~1 m. Pick it with a tape measure, not from a header.

---

## 6. Facts the controller is built around

### 6.1 Trust pose and ticks, never twist

`/odom_raw`'s `twist.linear.x` has an **inverted sign relative to its own
`pose.position`**. Measured wheels-off: commanded +0.012 gave twist −0.842 and
pose +1.395. A guard comparing a commanded sign against reported twist aborts
every *correct* move — which killed the first deadband sweep and produced a
retracted "290 mm backward lurch" that was in fact a 290 mm forward move.

**No function in this module accepts a twist. There is nowhere to pass one.**
Linear feedback comes from `/wheel_ticks`; heading from the gyro.

### 6.2 Four wheels disagreeing is mechanical before it is numerical

`travel_mm_from_ticks` takes the **median** of the four wheel deltas and reports
the **spread**. A mean moves by a quarter of one wheel's error; a median moves by
nothing. Worked example from the test bench — three wheels at 10 mm, one
spinning free at 100 mm:

```
median  10.0 mm        spread  90.0 mm        mean would be 32.5 mm
```

On this chassis three per-wheel anomalies were each explained away separately,
and then a wheel came off. The spread is a diagnostic, not a correction: if it
grows over a run, that is a mechanical fault announcing itself.

### 6.3 A zero gyro is healthy

±0.01 rad/s deadband. The stillness threshold is set at 0.02 rad/s so that zero
reads as *still*. It is never used to decide whether the sensor is alive.

---

## 7. Three measurements deliberately **not** carried across

A measurement is only valid in the regime it was taken in. This section exists
because three sets of numbers in this repo are widely cited as golden B8B
constants and **are not** — a grep of all ten files in `golden_backup/` returns
zero hits for any of them. They come from `firmware_linorobot/moves_main.cpp:28–42`,
which drives at `BASE_DUTY 70` with a kick to 140. B8B crawls at duty 26 with no
kick and no trims.

### 7.1 The per-wheel trims `{0.820, 0.839, 0.914, 1.000}`

Measured at base duty 70. This controller cruises near the duty floor, perhaps a
third of that, and friction, stiction and the motor curve are all nonlinear
across a 3× span of duty. **These four numbers are not hardcoded anywhere in
`fpms_motion.py` and must not be.** Trims arrive from a calibration profile or
default to unity. If you want them, re-measure at the duty this controller
actually uses.

### 7.2 The kick-start (duty 140 for 220 ms)

Supported, **off by default**, and explicitly a hypothesis.

*For it:* static friction exceeds kinetic. A profile starting at exactly the
duty floor may fail to break away, sit still, and then lurch when something
gives.

*Against it, and this is the stronger argument:* the floor this controller
starts at **is** `min_moving_duty`. If that number was measured properly — the
duty at which a **stationary loaded** rover first breaks away — then it already
contains the breakaway margin and a kick is redundant. A kick is only justified
when the measured floor is a *sustained* floor: the lowest duty that keeps an
already-rolling rover rolling, which is a different and lower number.

**So the real fix is a measurement, not a constant: record which of the two
floors you measured.** With only one number, treat it as the breakaway floor and
leave the kick disabled. Enable it only after watching the rover fail to start
from a floor-duty command.

If you do enable it, both parameters are **UNMEASURED** and both derive from
*this* controller's floor, not from the base-70 regime:

```
amplitude = kick_ratio · min_moving_duty      (default 2×, → ~12 % duty, not 140)
duration  = 0.12 s                            (3 ticks at 25 Hz, not 220 ms)
```

140 would be more than twice `HARD_MAX_DUTY`. The kick is the one place in the
module where a duty step is not rate limited — a breakaway impulse that ramps
over eight ticks is not an impulse — and what bounds the lurch is the small
amplitude, not the ramp.

### 7.3 The coast compensation

`coast_mm` / `coast_deg` are parsed and **not applied**. Two independent
reasons:

1. **Wrong regime.** `coast_mm` is measured by cutting from *cruise*, because
   that is the burst executor's only option. This controller decelerates under
   control and cuts at the **duty floor**, where the carried momentum is a much
   smaller and different number. Subtracting a cruise-cut coast from a
   floor-cut move removes distance that was never carried.
2. **Diagnostic hazard.** `DRIVE_COAST_FACTOR = 0.90` in `fpms_missions.py` has
   no golden ancestor — golden never coasted drives at all — and a constant 0.90
   *ratio* is indistinguishable on a residual plot from a 10 % `counts_per_mm`
   error. That is exactly the distinction `_publish_residual`'s own docstring
   teaches operators to read: **constant offset = coast, constant ratio =
   scale**. Applying either shape here would destroy it.

What replaces it is *computed*: the terminal band includes `v_min · τ`, the
distance carried after a cut at the rate we actually cut at. If a coast term is
ever genuinely needed on this path it is a new measurement with a new name —
"distance carried after a cut at the duty floor" — and it will be millimetres.

---

## 8. Every gain, and what happened when it was simulated

The module was exercised against a first-order plant (`τ` lag from commanded
duty to chassis speed) with a placeholder calibration. **This is a simulation
against a model, not a rover.** Its only real value is that it found three bugs
that reading the code did not.

### 8.1 The gains

| Gain | Value | Where it comes from |
|---|---|---|
| Control rate | 25 Hz | **Matches the measurement**, not faster. v3 publishes `/wheel_ticks` at `FPMS_TICKS_HZ = 25`. Running faster than the feedback arrives differentiates stale data. The board's own control task runs at 50 Hz, so every command is applied at least twice before refresh. |
| `a_max` | 0.25 m/s² | **Policy, not measurement.** At 0.18 m/s that is 0.72 s and 65 mm to cruise. Too low costs only time; too high shows up as the rover lagging its profile, which bounded feedback absorbs and the residual reports. Neither direction produces a confidently wrong distance. |
| `alpha_max` | 1.2 rad/s² | Same argument. 0.375 s and 4.8° to reach `TURN_RADPS = 0.45`. |
| `Kp` distance | 1.0 s⁻¹ | Expressed as **1/T** so it reads as a decision: a standing error closes with T = 1.0 s, chosen **slower** than the profile's own 0.72 s acceleration so the trim cannot fight the ramp it rides on. |
| `Ki` distance | 0.2 s⁻², clamped to 20 mm/s | Exists for exactly one disturbance: `mps_per_duty` measured on a fuller battery. At the clamp it contributes ~11 % of cruise. Frozen during acceleration and into saturation. |
| trim ceiling | 30 % of cruise | Feedback trims; it never becomes the primary signal. |
| `Kp` heading | 1.2 rad/s per rad | Carried unchanged from `fpms_missions.HEADING_KP`, itself the shape of B8B's `_KPH = 10`. 1/Kp = 0.83 s; on the ideal velocity-commanded model that is first-order and cannot overshoot. |
| heading clamp | 0.25 rad/s | `HEADING_CORR_MAX_RADPS`, the analogue of B8B's ±6 duty counts ≈ ±23 % differential. The clamp is what stops a bad gyro becoming a spin. |
| `Kd` heading | 0.15 s | Damps the actuator lag the ideal model omits; ≤ 0.14 rad/s at the maximum plausible yaw rate. |
| `Kp` turn | 2.0 s⁻¹ | A 0.5 s residual time constant — the trim settles well inside the profile's own decel ramp. |
| `Kd` turn | 0.315 s | **Derived**, not tuned: `2ζ√(Kp·τ) − 1` with ζ = 1.2, τ = 0.15 s. |
| ζ | 1.2 | Overdamped on purpose. Overshoot is a bias; a bias survives the retrace. |
| duty rate limit | ≤ 8 %/tick, and `a_max·dt/mps_per_duty` | Two ceilings, the tighter wins, and tying it to `a_max` keeps one acceleration limit instead of two. |
| stillness | 0.02 rad/s | Above the ±0.01 rad/s parked deadband. |

### 8.2 What the simulation showed after the fixes

```
TURNS                                   STRAIGHTS
  +90°  → +90.35°   0 reversals           1000 mm →  997.7 mm   6.3 s
 −180°  → −180.32°  0 reversals            300 mm →  298.0 mm   2.4 s
  +360° → +360.38°  0 reversals             70 mm →   67.5 mm   1.1 s
  +15°  →  +15.28°  0 reversals    20 % weak plant →  996.0 mm  10.6 s
```

Zero direction reversals is the anti-hunting claim; the residual is the stop
band doing its job and is reported, not hidden. A 1 m leg takes 6.3 s where the
burst executor's own arithmetic gives ~0.05 m/s average.

### 8.2.1 A SECOND, INDEPENDENT PLANT DISAGREED — AND FOUND A REAL BIAS

The table above came from the plant this file's author wrote. A separate plant,
written against the public API only (`selftest/` scratch, first-order lag on the
commanded rate, `omega += (k·duty − omega)·dt/τ`), reported something different
and much worse:

```
BEFORE the measured-rate cut band          AFTER
   15° → +3.04°      90° → +4.47°            15° → −0.52°     90° → −0.34°
   45° → +4.66°     180° → +4.08°            45° → −0.16°    180° → −0.38°
   worst |err| 4.66°, 0 reversals            worst |err| 0.62°, 0 reversals
```

Every angle overshot by roughly `ω_peak · τ` = 0.45 × 0.15 = **3.9°**, the same
direction every time. Two plants disagreeing by 13× is not noise, and the
second one was right about the mechanism: **`_command_v` decided when to cut
from POSITION alone, while the distance carried after the cut is set by the
SPEED at that moment.** The chassis trails its own command by τ for the whole
deceleration ramp, so the move cut while still travelling near cruise. §6's
envelope lag term does not catch this — that term makes the *command*
stoppable, and the command was never the problem.

The fix is in `_command_v`: the stop band is widened to `v_measured · lag_s`,
the distance a first-order decay actually carries once the command is zero.
Note it is `v·τ` and **not** `v²/(2a)` — the latter is the distance under active
braking, which does not apply after the command has gone to zero.

Both plants now agree to within 0.7° and both show zero reversals, which is the
only reason either number is quoted. **Neither is a rover.**

Two things this did NOT fix, recorded rather than smoothed over:

- **Straight runs still undershoot 5–9 mm** (744 mm → 735.4). The cut band is
  `max(arrive_tol, unclosable, carry)`, and with `arrive_tol` at 20 mm against a
  ~12 mm carry the fixed term dominates and the move stops short. That is an
  *arrival-reporting* tolerance being used as a *cut* threshold — two different
  quantities sharing one constant. The residual is reported, so a caller can
  close it; fixing it properly means separating the two and is not a gain change.
- **Sensitivity to an unmeasured τ is still large.** At 90° with the controller
  assuming 0.15 s, a real τ of 0.30 s now overshoots 6.4° (was 12.2°) and 0.60 s
  overshoots 21.7° (was 25.8°). Better, not solved. **τ is the single most
  valuable number to measure before this drives anything**, and when in doubt
  set it HIGHER — the error from over-estimating τ is a small undershoot, and
  from under-estimating it is an overshoot bias.

### 8.3 The three bugs, because they are the interesting part

**1. A unit-dispatch error that produced correct distances at one sixth of the
speed.** `TrapezoidalProfile` overrides the base profile's methods to convert
mm/s → m/s at the edge. The base class's `command_v` called `self.v_envelope()`,
which Python dispatched to the *subclass* override — so a routine working in
mm/s received a cap in m/s, a factor of 1000, and every commanded speed
collapsed onto the duty floor. The module imported cleanly, the loop converged,
and 1000 mm moves finished 4 mm from target. **The numbers were right and the
rover was crawling for 33 seconds.** Only simulating the trajectory and looking
at *peak speed* found it. Internal code now calls `_native` methods that
overrides never touch.

**2. `wrap_pi` on the turn made a 180° turn drive 539°.** The gyro integral is a
*cumulative* angle, not a compass heading — `integrate_gyro` never wraps it,
precisely so a turn larger than π is representable. Wrapping the error made a
180° target integrate as −180°. It would have looked like a spin, with a
plausible log.

**3. An overshoot could never be corrected, and a +1.3° bias hid behind it.**
The turn only ever commanded in the target's direction, so once past the target
it commanded zero and waited for a timeout. Fixed with a bounded opposite-sense
trim that the terminal latch closes for good. Behind it was a real modelling
error: the textbook braking envelope `√(2a·rem)` assumes the chassis decelerates
the instant the command drops, and every turn from 15° to 360° arrived a
consistent **+1.3° past target** — the same direction every time, i.e. exactly
the bias a retrace cannot cancel. No gain fixes that; the envelope was solving
the wrong equation. Adding the lag term brought arrival to +0.35°.

---

## 9. What is unverified — all of it

**No rover has run this code.** Not one wheel has turned. In addition:

1. **The whole controller is untested on hardware.** The simulation in §8 is a
   first-order plant with a placeholder calibration. It proves the arithmetic is
   self-consistent and nothing about the rover.
2. **`gyro_scale` has never been measured.** Every turn refuses today. That is
   the intended behaviour and it is also a hard blocker — `--spin-check 90` is
   the first thing to run.
3. **`mps_per_duty`, `radps_per_duty`, `duty_forward_sign` and
   `side_map_confirmed` do not exist in any calibration file, and
   `fpms_charact.py` does not write them.** Until it does, this controller
   refuses everything. That is four measurements, not four defaults.
4. **`min_moving_duty` has never been measured on v3**, and the shipped example
   value of `0.0` is refused. Everything downstream of it — the profile floor,
   the terminal bands, the anti-hunt guarantee, the kick amplitude — is derived
   from a number nobody has.
5. **`counts_per_mm ≈ 5.5` was measured on a chassis whose front-left hub was
   working loose** — that wheel later came off. The 2.7× correction is robust;
   the third digit is not.
6. **`τ = 0.15 s` (actuator lag) is an estimate and the envelope's sensitivity
   to it is one-sided.** For the *damping* it is safe: ζ ∝ 1/√τ, so a faster
   real actuator is more damped. For the *braking envelope* it is not — assuming
   too small a τ under-brakes and overshoots. Simulated: a real τ of 0.30 s
   against an assumed 0.15 s overshot 90° by 2.9°, and 0.60 s overshot by 6.7°.
   **Measure it, and when in doubt set it higher.** The failure mode of assuming
   too large is a slow, undershooting arrival, which is correctable.
7. **`a_max` and `alpha_max` are policy numbers with no measurement behind
   them.** The chassis may not achieve them; the feedback absorbs that and the
   residual reports it, but the profile's timing will be optimistic.
8. **The `[FL, FR, RL, RR]` order is the firmware's declaration, not an
   observation.** `fpms_duty_driver.py` carries `SIDE_MAP_MEASURED = False` over
   exactly this question, and `fpms_missions.py` records the opposite mapping
   after the 2026-08-01 rewiring. Straight motion survives a swapped map; every
   turn and every heading correction inverts.
9. **`duty_forward_sign` is a property of the flashed binary, not of the repo.**
   On 2026-08-06, +60 % on all four drove every wheel *backward*, confirmed by
   eye. The repo's `MOTORn_INV` values have since been flipped, so a board
   flashed from current source *should* want +1 — but "should" is not a
   measurement, and getting it wrong drives the rover backwards at the full
   commanded speed.
10. **`lr_asymmetry`'s direction convention is assumed** to be
    `left_travel / right_travel`. The calibration example does not define it.
    Getting it backwards doubles the imbalance instead of cancelling it.
11. **The breakaway kick is a hypothesis with two unmeasured parameters.**
12. **Nothing here has been integrated with the executor, the obstacle guard,
    the arm gate or the stop path.** This is the math layer only.
13. **Judge every one of these by the operator's eyes, not by odometry.** This
    rover has reported clean travel while spinning in place and has lied about
    direction. Nothing in this module changes that. The residual and the wheel
    spread exist to make error *visible*, not to make odometry trustworthy.

---

## 10. Bring-up order

Nothing below drives on the floor until the step before it passed.

1. **Prove the firmware.** `ros2 topic echo /fpms_health` → field 0 must read
   `30001`. If the topic is silent, suspect the DDS transport before the board
   (`FASTRTPS_DEFAULT_PROFILES_FILE`, see `docs/DDS.md`) — discovery can succeed
   while no data flows.
2. **Push check, no battery, 30 seconds.** `fpms_charact.py --push-check`. Push
   forward along a tape measure: `x` must increase, and `/wheel_ticks` must go
   **positive on all four**. This also re-derives `counts_per_mm` on a sound
   chassis, which item 5 above asks for.
3. **On blocks, wheels off the floor.** One 1 s `/cmd_duty` pulse at +20 % on
   all four. Watch which way the wheels turn → `duty_forward_sign`. Then one
   side alone → `side_map_confirmed`. Both are operator's-eyes measurements and
   neither can be inferred.
4. **The duty floor, on blocks then on the floor.** Step duty up until the
   wheels *start from rest* under load. That is `min_moving_duty`, and record
   that it is the **breakaway** floor (§7.2).
5. **`mps_per_duty` and `radps_per_duty`.** Constant duty at ~2× the floor over
   a tape-measured run; and a spin against `--spin-check 90`, which also gives
   `gyro_scale`.
6. **Only now, motion.** Start with `StraightRun` over 300 mm with heading hold
   off, on blocks, comparing commanded duty against `/wheel_duty` — if you
   command 26 and it reads 0, the deadman is firing. Then heading hold. Then
   turns. Then the floor.

Log `Calibration.notes` at startup, every time. Those `ADOPTED` / `REFUSED` /
`ABSENT` lines are the only evidence that a number you set is a number being
used.
