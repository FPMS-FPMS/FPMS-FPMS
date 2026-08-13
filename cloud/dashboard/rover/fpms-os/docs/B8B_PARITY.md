# B8B parity — what the mission executor kept, what it changed, what it never had

Audit of `fpms_missions.py` (6315 lines, the live executor) against the golden
B8B/phase6 code in `golden_backup/`, driven by the claims in
`../GOLDEN_B8B_STUDY.md`.

**This document proposes no rewrite.** `fpms_missions.py` is gate-verified at
m2 = 744.0 mm (`../test_stack.py:96`) and `../STACK.md:89` records that it was
deliberately **not** rewritten. Everything below is either a finding, or a
surgical addition that can be reverted with one line.

**Method.** Read, not run. Nothing here was executed against hardware; the
statuses below are the statuses the source files themselves carry.

| status | meaning |
|---|---|
| MEASURED | a file records an operator measurement and the date/run it came from |
| DERIVED | arithmetic from a measured quantity |
| ASSUMED | typed by someone, with a stated rationale, never checked |
| UNMEASURED | typed, no rationale, no check |
| DISPUTED | two files in this repo give different values for the same physical quantity |

---

## 0. The headline, before the table

**Three of the constants I was asked to account for were never in B8B.** The
per-wheel trims, the kick-start and the coast compensation do not appear
anywhere in `golden_backup/`. I grepped all ten files: zero hits for `0.820`,
`0.839`, `0.914`, `KICK_DUTY`, `1270`, `1448`.

They come from a different codebase entirely — the ESP32 open-loop sketches:

- `../firmware_linorobot/moves_main.cpp:28` `BASE_DUTY = 70`
- `../firmware_linorobot/moves_main.cpp:29-30` `KICK_DUTY = 140`, `KICK_MS = 220`
- `../firmware_linorobot/moves_main.cpp:31` `TRIM[4] = {0.820, 0.839, 0.914, 1.000}`
- `../firmware_linorobot/moves_main.cpp:37` `TRIM_REV[4] = {0.820, 0.737, 0.894, 0.960}`
- `../firmware_linorobot/moves_main.cpp:41-42` `COAST_FWD = 1270`, `COAST_REV = 1448`
- duplicated in `../firmware_linorobot/fwd800_main.cpp:30-32` and `../rescued/`

`../GOLDEN_B8B_STUDY.md` does not actually claim otherwise. Its **LIVE
CONSTANTS** block (lines 68-78) is the B8B list and contains none of them; the
trims/kick/coast appear 23 lines further down under **"MEASURED HARDWARE FACTS
TO CARRY OVER (verified this session)"** (lines 101-111), which is a different
claim about a different session. `../SESSION_2026-08-03.md:85` and
`../PI_FILE_INVENTORY.md:230-233` both attribute them to the firmware sketches.

This matters because the two regimes are incompatible: B8B crawls at **duty 26
with no kick and no trims** (`../golden_backup/phase6_latest.py:1309`,
`:1352`, `:1366`); the sketches crawl at **base duty 70 with a kick to 140 and
four per-wheel scale factors**. You cannot carry both. Nothing was lost by
carrying neither into a file that publishes `Twist`.

**The genuine loss list is short**, and none of it is on the duty path. See §3.

---

## 1. The parity table

`M` = `fpms_missions.py`. `G` = `golden_backup/phase6_latest.py`.
`D` = `fpms_duty_driver.py`.

### 1a. The constants named in the brief

| Golden constant | Golden site | Today | Site | Verdict |
|---|---|---|---|---|
| `TURN_SIGN = -1` | `G:178` | `TURN_WIRE_SIGN = -1` | `M:771` | **KEPT**, same value, same purpose, applied at one place (`M:4209`). Both UNMEASURED — `M:772` says so and `M:4455-4460` aborts on a wrong-way turn rather than trusting it. `../SPEC.md:243` notes `config.env.example` ships `1`, so the two already disagree. DISPUTED |
| `_TKMM = pi*70/1320` | `G:1309` | absent from `M`; `TKMM` in `D:114` | — | **NOT APPLICABLE** to `M` (§2). Carried only into the duty module, and **stale there**: 1320 ticks/rev over a 70 mm wheel = **6.00 counts/mm**, while `../SPEC.md:242` records **5.5** MEASURED (2026-08-06) with a 65 mm wheel. DISPUTED — see §3.5 |
| `_DRV = 26` | `G:1309` | `CRUISE_MPS = 0.08` | `M:732` | **NOT APPLICABLE** (§2). No duty exists on the `Twist` path |
| `_TRN = 50` | `G:1309` | `TURN_RADPS = 0.45` | `M:738` | **NOT APPLICABLE** (§2) |
| `_COAST = 0.93` | `G:1309` | `TURN_COAST_FACTOR = 0.93` | `M:1034` | **KEPT**, exact value, and the reasoning is reproduced at `M:1030-1033` and `M:4400-4404` |
| `_HZ = 20` | `G:1309` | `CONTROL_HZ = 20.0` | `M:1027-1028` | **KEPT**, exact, commented "phase6's loop rate, carried over". Used at `M:4477` and `M:4611` |
| `_FSTOP = 120` | `G:1310` | `FRONT_STOP_MM = 120.0` | `M:1053` | **KEPT**, exact value, now config-overridable within [60, 600] |
| `_KPH = 10`, clamp ±6 | `G:1310`, `G:1365` | `HEADING_KP = 1.2`, clamp ±0.25 rad/s | `M:750-751` | **CHANGED, and the change is not verifiable** — see §4.1 |
| `HARD_MAX_DUTY = 60` | **not golden** — `D:140` | `HARD_MAX_LIN_MPS 0.25` / `HARD_MAX_ANG_RADPS 0.90` | `M:742-743`, applied `M:4201-4202` | **PRESENT IN KIND.** `D:140`'s own docstring derives it from `TRN=50`, so it is the duty module's invention, not a golden carry |
| `DEADMAN_S = 0.30` | **not golden** — `D:147` | absent host-side | — | **ABSENT, mitigated board-side.** `D:147` says it "matches the 300 ms deadman ARCHITECTURE_V2.md specifies for the board". That board deadman exists: `../firmware_v3/fpms_config.h:56` `FPMS_CMD_TIMEOUT_MS 300`; on today's factory firmware it is `../firmware/components/rover_config/include/rover_config.h:135` `CFG_DEF_CMD_TIMEOUT_MS 500`. B8B itself had **no** deadman. See §3.4 |
| `MAX_CONTINUOUS_MOTION_S = 15.0` | **not golden** — `D:156` | `burst_cap_s` + `MISSION_TIMEOUT_S 240` | `M:1522-1532`, `M:1049` | **PRESENT IN KIND, and better.** `D:156` itself says "B8B's per-move timeout was 12 s". `burst_cap_s` is per-segment and sensor-independent (`M:1495-1519`), which the 15 s blanket cap is not |
| base duty 70 + trims `{0.820, 0.839, 0.914, 1.000}` | **not golden** — `moves_main.cpp:28,31` | absent everywhere in Python | — | **NEVER IN B8B** (§0). Legitimately absent from `M` (§2). **Also absent from `D`** — see §3.6 |
| kick-start duty 140 for 220 ms | **not golden** — `moves_main.cpp:29-30` | absent everywhere in Python | — | **NEVER IN B8B** (§0). §2, §3.6 |
| coast comp 1270 fwd / 1448 rev counts | **not golden** — `moves_main.cpp:41-42` | absent everywhere in Python | — | **NEVER IN B8B** (§0). §2, §3.6 |

### 1b. The behaviours

| Golden behaviour | Golden site | Today | Site | Verdict |
|---|---|---|---|---|
| **burst → STOP → settle → measure-at-rest → residual → correct** | `G:1349-1369` + `G:1479-1494` | the whole `_drive`/`_turn` shape | `M:4394-4641` | **KEPT, and extended.** Burst loop `M:4549-4611`; unconditional stop in `finally` `M:4612-4615`; polled settle `M:4624`, `M:5531-5552`; measure at rest `M:4627-4630`; residual published per segment `M:5784-5848`; correction by re-measuring the bearing each iteration `M:5939-6039`. The residual publication is **new** — golden measured but never reported |
| Plan fully before moving | `G:1455-1472` | `_cmd_mission` → `_preview`/`_publish_plan` → worker | `M:4860-5090`, `M:5298-5456` | **KEPT** |
| **First hardware command is a STOP** | study line 41 | absent | `M:5025` clears the halt latch and starts the worker; no `stop_wire` | **LOST.** Cheap to restore — §3.1 |
| Drop any leg < 30 mm | `G:1465` (`if sd<30: continue`) | `MIN_LEG_MM = 30.0` | `M:965`, used `M:1315` | **KEPT, exact value** |
| Log the entire plan before moving | `G:1470-1472` | `telemetry/mission_plan` | `M:5349-5456` | **KEPT, and machine-readable** |
| Turn: cut at coast, then **integrate 250 ms further and return the ACTUAL degrees** | `G:1338-1348` | settle then `seg.measured = degrees(yaw - yaw0)` | `M:4490-4493` | **KEPT.** Settle length changed — §4.2 |
| Extra 0.6 s dwell past a 45° spin | `G:1480-1481` | `turn_settle_s()` adds 0.6 above 45° | `M:1552-1562` | **KEPT, exact**, and `M:1555-1557` cites the golden line |
| 0.3 s between segments | `G:1494` | `STOP_SETTLE_S = 0.45` | `M:1037` | **CHANGED** (§4.2) |
| 0.4 s dwell in retrace | `G:1416` | no distinct value; retrace uses the same `STOP_SETTLE_S`/`turn_settle_s` | `M:5699-5700` | **CHANGED** — folded into the common path (§4.2) |
| Guard order: stop → arrival → front LiDAR → distance → timeout | `G:1355-1362` | stop/shutdown → mission timeout → link → battery → obstacle → foreign writer, then distance/cap/frame/timeout/stall in the segment | `M:5500-5529`, `M:4560-4601` | **KEPT IN SHAPE, superset in content.** Link, battery, foreign-writer, burst-cap, odom-frame and stall guards are all new |
| Arrival guard on a **locked marker's** distance, every tick (400 mm / 5 mm home) | `G:1358-1360`, `G:1910-1911` | no marker code at all — `grep -n marker fpms_missions.py` returns one unrelated comment (`M:5039`) | `ARRIVE_TOL_MM = 25.0` `M:975`, `_drive_to` `M:5992` | **REPLACED, not ported.** See §3.2 |
| `_face_and_drive`: face the marker, chunk at `min(500, dist-stop_at+80)`, 15 attempts | `G:1370-1389` | `split_legs` (`MAX_LEG_MM 300` cruise, `DOCK_STEP_MM` dock) + `MAX_SEGMENTS 40` | `M:1272-1331`, `M:961`, `M:971`, `M:1024` | **KEPT IN SHAPE** — bounded chunks, bounded attempts, dock steps smaller than cruise steps. Values all changed and all rationalised in place |
| Front guard fails **open** on missing data (`_fr()` returns 9999) | `G:1319-1321` | `if clear is not None and clear < limit` | `M:5514-5516` | **KEPT — including the flaw.** `../STACK.md:416-419` §9.7 states plainly that fail-open-on-absent was left alone because it is a behavioural change to a proven file that could not be tested |
| `_spin` has **no** front guard and **no** timeout | study line 55 | `_turn` has both | `M:5515` (`ROTATE_CLEAR_MM 80`, any bearing), `M:4467-4475` | **DELIBERATELY NOT CARRIED.** A golden bug, fixed |
| `TIMEOUT` reports the **requested** mm as driven | `G:1369` (`return mm,"TIMEOUT"`) | aborts and reports the **measured** value | `M:4632-4636` | **DELIBERATELY NOT CARRIED.** The study calls it "odometry optimism — a known bug" (study line 88) |
| `NO_ENC` → return 0 mm and **continue to the next segment** | `G:1350-1351` | `ABORT_LINK`, mission stops | `M:4518-4519`, `M:5508-5509` | **DELIBERATELY NOT CARRIED.** Driving on with no encoder is how a segment goes unmeasured and the retrace cannot give it back |
| `BLOCKED` → skip the segment and continue, no reroute | `G:1491-1492` | `ObstacleDetour` reroute, or abort | `M:5520-5527`, `M:5939-6058` | **DELIBERATELY NOT CARRIED, and upgraded** |
| **Retrace home**: reversed outbound list, reverse-drive then undo turn | `G:1394-1421` | `invert_segments` over everything `executed` | `M:1399-1476`, `M:5688-5700` | **KEPT, and strictly stronger.** Golden reversed `(actual_turn, mm_done)` tuples; today it reverses and negates the **measured** value of every `Segment`, carries sub-minimum motions into the next adjacent motion of the same kind, and **reports what it could not give back** (`retrace_residual`, `M:1435-1444`). Golden silently dropped those |
| Retrace budget | golden: none, list-driven | `budget=len(retrace)` | `M:5698` | **KEPT IN KIND** |
| Final HOME nudge: measure the gap, drive straight, no turn, ≤3 tries, 25 mm tolerance, `min(gap, 200)` | `G:1427-1451` | `_home_trim` | `M:6098-6142`, `M:1000-1002` | **KEPT — every number matches.** `HOME_TRIM_TRIES 3`, `HOME_TRIM_TOL_MM = ARRIVE_TOL_MM = 25`, `HOME_TRIM_MAX_MM 200`. `M:6101-6103` cites the golden block. The no-turn rule is stated and enforced (`M:6106-6112`, `M:6130-6132`) |
| Re-face at the end | `G:1380-1382` (face the marker) | `_reface`, gyro-delta preferred, **refuses** above `MAX_REFACE_DEG 30` | `M:6060-6096`, `M:991` | **CHANGED, deliberately.** Golden always turned; today a large error is reported rather than spun out |

### 1c. Present today with no golden ancestor

| Today | Site | Note |
|---|---|---|
| `DRIVE_COAST_FACTOR = 0.90` | `M:1036`, used `M:4532` | **NEW.** Golden's `_fwd` cut at `dm >= mm` with no coast fraction (`G:1362`) — only turns coasted. The drive analogue is reasoned, but the 0.90 is UNMEASURED (`../SPEC.md:245` lists `coast_mm` as unset). See §4.3 |
| `burst_cap_s` / `FULL_DUTY_MPS 0.65` | `M:1495-1532` | **NEW**, sensor-independent, derived from a real incident |
| `ABORT_ODOM_FRAME` | `M:4593-4595`, `M:3406` | **NEW.** Catches the "moved 1533.6 mm" report from a rover that went ~2 m |
| `TURN_WRONG_WAY_DEG 8.0` abort | `M:923`, `M:4455-4460` | **NEW.** Makes an unmeasured `TURN_WIRE_SIGN` cost one twitch |
| `telemetry/residual` | `M:5784-5848` | **NEW.** Golden measured everything and published none of it |
| `ODOM_SCALE`, `ODOM_POSE_SIGN` | `M:855`, `M:911` | **NEW** boundary corrections; both default to "no change" |
| Arena fix at every hold | `M:5628`, `M:4039-4136` | **NEW**; `ARENA_FIX_CALIBRATION_VERIFIED = False` (`M:1139`) |

---

## 2. Which absences are legitimate — the raw-duty question

**The executor publishes `Twist` on `/cmd_vel`.** `M:4181-4219` is the only
place a non-zero command leaves the process, and it emits `Twist` and nothing
else. `../STACK.md:438-440` states the duty path is not wired up and that
wiring it is real work, not a config change.

Everything below is meaningless on that wire, and its absence from
`fpms_missions.py` is correct, not a loss:

1. **Per-wheel trims `{0.820, 0.839, 0.914, 1.000}` and `TRIM_REV`.** A `Twist`
   has no per-wheel term. Four scale factors require four duty channels, which
   is `set_motor(m1,m2,m3,m4)` (`G:1330-1331`, `G:1366`) or
   `moves_main.cpp:152`. **Confirmed legitimately absent** — and they are
   firmware constants, so their correct home is `firmware_v3`, not the Pi.
2. **Kick-start duty 140 for 220 ms.** A kick is an amplitude discontinuity on a
   duty channel. On this firmware every non-zero velocity is already ~50 % duty
   (`M:692-696`, `../NEXT_SESSION_B8B_DUTY.md:7-9`) — the setpoint *is* a
   permanent kick, which is the problem, not the fix. **Confirmed legitimately
   absent.**
3. **Coast compensation 1270 / 1448 counts.** Counts, and direction-dependent,
   subtracted from a count target on the board (`moves_main.cpp:41-42`). The
   executor has no count target; it differences pose in metres
   (`M:4643-4658`). Its coast handling is fractional and lives in
   `TURN_COAST_FACTOR`/`DRIVE_COAST_FACTOR`. **Confirmed legitimately absent**,
   though the fractional replacement is UNMEASURED (§4.3).
4. **`_DRV = 26` / `_TRN = 50`.** Duty percentages. `CRUISE_MPS`/`TURN_RADPS`
   occupy the same slot in the structure. **Legitimately absent — with a
   caveat**: `../NEXT_SESSION_B8B_DUTY.md:16-18` says `CRUISE_MPS` is "an
   amplitude knob on a path that discards amplitude". So the slot exists but the
   knob is inert. That is a firmware problem, already documented, and not
   something `fpms_missions.py` can fix.
5. **`_TKMM`.** A counts→mm factor. The executor reads metres off `/odom_raw`;
   the conversion happens on the board. **Legitimately absent** — see §3.5 for
   what that costs.
6. **`HARD_MAX_DUTY`, `DEADMAN_S`, `MAX_CONTINUOUS_MOTION_S`.** Not golden at
   all; they are `fpms_duty_driver.py`'s own safety envelope for a wire that is
   not connected. Two have equivalents in `M` (§1a); one does not (§3.4).

**So: the per-wheel trims and the kick-start do fall in this category, exactly
as suspected, and so does the coast compensation.**

---

## 3. Genuine losses — carried by golden, applicable today, not present

Five. Only two of them are worth acting on.

### 3.1 The first hardware command is no longer a STOP — LOST, trivial to fix

The study's property #2 (line 41) is explicit: `open_bot()` → `stop_bot()` →
re-plan → move, and "the first hardware command is a STOP".

Today `_cmd_mission` clears the halt latch at `M:5025` and starts the worker. No
`stop_wire` runs before the first burst. `stop_wire` appears at `M:4768`
(Nav2 cancel), `M:5490` (abort), `M:5728` (worker `finally`), `M:6300`
(shutdown) — never at mission start.

**Why it still matters on the `Twist` path**: this node publishes nothing while
idle (`M:5461-5465`), but it is not the only writer. `fpms_teleop.py` publishes,
and the foreign-writer check (`M:5528-5529`, `M:4166-4180`) *detects* another
writer without *countermanding* whatever it last put on the wire.

**Recommendation: PORT IT.** One line after `M:5025`:

```python
self.node.halt.clear()
self.node.stop_wire()          # B8B: the first hardware command is a STOP
```

Cost: three zero-`Twist` messages and 60 ms, before any motion. It cannot change
a plan, cannot change a measurement, and cannot fail — `stop_wire` swallows its
own exceptions (`M:4230-4231`). Planner-neutral by construction, so the m2 gate
is untouched. **Testable**: assert `stop_wire` was called before the first
`publish` with a non-zero argument.

### 3.2 The exteroceptive arrival guard — REPLACED, and the replacement is unverified

Golden checked, at 20 Hz inside every burst, whether a **locked marker** was
within `_stop_at` (`G:1358-1360`; 400 mm for a target, 5 mm for home,
`G:1910-1911`), and `_face_and_drive` (`G:1370-1389`) closed the final approach
on the marker's bearing.

`fpms_missions.py` contains **no marker code whatsoever**. Arrival is
`dist <= ARRIVE_TOL_MM` against the *estimated* pose (`M:5992`, `M:975`).

This is not a straight loss — golden's markers came from LiDAR clustering
(`../golden_backup/B6_perfect.py:67-74`) and that subsystem is not in this
stack. Its structural replacement is the arena fix at every hold
(`M:5628`, `M:4039-4136`). But be blunt about what changed:

- Golden's loop closed on the **world**, 20× a second, during motion.
- Today's closes on **odometry**, with an optional scan-match correction taken
  only at stops, and `ARENA_FIX_CALIBRATION_VERIFIED = False` (`M:1139`).

Given `../STACK.md:425-428` §9.10 ("this rover has reported clean travel while
spinning in place"), that is the single biggest behavioural difference in this
audit.

**Recommendation: NOTHING, here.** Restoring it means restoring marker
detection — a subsystem, not an addition, and exactly the rewrite this document
refuses. The right response is to finish verifying the arena fix. Record the
gap; do not patch it.

### 3.3 Golden's per-move 12 s ceiling — REPLACED by two better ones

`G:1354` bounded every `_fwd` at 12 s of wall clock regardless of sensors.
Today's `segment_timeout_s` (`M:1535-1549`) is derived from `CRUISE_MPS`, "a
SETPOINT the firmware ignores" — the file says so at `M:1541-1543` — so it is
not a wall-clock ceiling at all. The real ceiling is `burst_cap_s`
(`M:1522-1532`), which reads no odometry and is tighter than golden's 12 s for
every segment the planner emits (`MAX_LEG_MM = 300` → 0.74 s).

**Recommendation: NOTHING.** Strictly better than what it replaced. Noted only
so nobody re-adds a 12 s constant believing it was lost.

### 3.4 No host-side deadman — ABSENT, and B8B never had one either

Neither `G` nor `M` has one. `D:147` invented `DEADMAN_S = 0.30` for the duty
wire. On the `Twist` path today the only deadman is board-side:
`CFG_DEF_CMD_TIMEOUT_MS 500` on factory firmware
(`../firmware/components/rover_config/include/rover_config.h:135`), becoming
`FPMS_CMD_TIMEOUT_MS 300` on v3 (`../firmware_v3/fpms_config.h:56`).

**Recommendation: NOTHING in `fpms_missions.py`.** A host-side deadman would
have to re-send the last command to be useful, and `D:29-31` explains why that
is the one thing a watchdog must never do. The board's timeout is the right
place and it exists. Worth stating in `FAILURE_MODES.md` that the executor
relies on it.

### 3.5 `_TKMM` is gone from the Pi, and the number it encoded is still DISPUTED

Legitimately absent from `M` (§2.5) — but the consequence is that the executor
has **no independent scale**. Every distance it measures is whatever
`COUNTS_PER_REV` the board integrates with, corrected only by `ODOM_SCALE`
(`M:855`), which defaults to `1.0` and is UNMEASURED (`../SPEC.md:246`).

And where the constant *was* carried, it is stale:

| source | implied counts/mm | status |
|---|---|---|
| `G:1309` / `D:114` `pi*70/1320` | **6.00** | ASSUMED — `D:115-119` already flags the factory header's 1040 as a contradiction |
| `../SPEC.md:242` | **5.5** (`COUNTS_PER_REV` 1120, 65 mm wheel) | MEASURED 2026-08-06, "on a chassis whose front-left hub was working loose" |
| `../GOLDEN_B8B_STUDY.md:107` | 14.8 | MEASURED, SUPERSEDED — 2.7× too high |

`D:114`'s comment tells the reader to "expect to re-measure this first if
distances come out ~27 % long" — it predates the 5.5 measurement and does not
know about it. 6.00 vs 5.5 is a **9 % overshoot** on the duty path.

**Recommendation:** a comment-only correction in `fpms_duty_driver.py:114-119`
pointing at `SPEC.md:242`. I do not own that file — flagging it here. **Do not
change the value**: `D` is unreachable hardware today
(`../NEXT_SESSION_B8B_DUTY.md:37-44`) and the 5.5 came off a loose hub.

### 3.6 The trims/kick/coast have no home anywhere — ABSENT FROM THE DUTY PATH TOO

Worth stating, because it is the one place the §0 finding has a future cost:
`fpms_duty_driver.py` — the module written specifically to be the raw-duty wire
— contains **no** kick-start, **no** per-wheel trims and **no** coast
compensation. Grep for `kick`, `trim`, `1270`, `1448` in it: nothing but an
unrelated docstring line at `D:22`.

That is arguably correct by its own charter (`D:19-26`: "It is NOT the executor
… no control loop, no distance integration"), and per-wheel trims belong in
firmware regardless. But when `/cmd_duty` is eventually wired up, somebody will
need those four numbers and they will not be on that path. `firmware_v3` is
where they go.

**Recommendation: NOTHING now.** Record it so the next session finds it.

---

## 4. Present but changed — and whether the change is explained

### 4.1 `_KPH = 10` clamped ±6 → `HEADING_KP = 1.2` clamped ±0.25 rad/s

`M:745-749` explains the intent: "phase6 used a P gain of 10 clamped to +/-6 in
its own power units; this is the same shape in rad/s."

**The shape is the same. The claim that the values correspond is not
checkable.** Golden's gain produced *duty counts* against a base of 26
(`G:1365-1366`), i.e. a ±23 % differential — the study says so at line 30.
Today's produces *rad/s* against a linear setpoint of 0.08 m/s, and converting
one to the other requires a duty↔velocity map that this firmware does not have:
`CMD_SCALE = 6.1` is "a defect artefact, not a calibration"
(`../STACK.md:404-408` §9.4, `../SPEC.md:244`), and the dead zone makes every
non-zero setpoint ~50 % duty anyway.

Both are ASSUMED. `1.2` and `0.25` carry no derivation anywhere in the repo.

**Recommendation: NOTHING.** Changing an untestable gain on a gate-verified file
buys nothing, and `M:746-749` is right that a heading hold on this firmware is
mostly fiction — which is *why* legs are short. The honest fix is the residual
stream, which already exists (`M:5784-5848`). If you want one improvement,
improve the comment: say "the same shape, not a converted value, and the
conversion is not available on this firmware".

### 4.2 The dwells

| dwell | golden | today | explained? |
|---|---|---|---|
| after a spin, before measuring | 0.25 s (`G:1341`) | `TURN_SETTLE_S = 1.2` (`M:1035`) | **No.** 4.8× longer, no rationale at the constant. Safe direction (a longer settle can only make the measurement better) but undocumented |
| extra past 45° | 0.6 s (`G:1481`) | +0.6 s (`M:1562`) | **Yes**, cited at `M:1555-1557` |
| between segments | 0.3 s (`G:1494`) | `STOP_SETTLE_S = 0.45` (`M:1037`) | **Partly.** Rationale for *having* the stop is at `M:4617-4621`; the 0.45 itself is UNMEASURED. `PLANNING.md:355` budgets on it |
| in retrace | 0.4 s (`G:1416`) | none distinct | **Not stated.** Folded into the common path. Since 0.45 > 0.4, the retrace settles slightly longer than golden did — harmless, but nobody wrote it down |

**Recommendation: NOTHING but a comment.** These are all in the conservative
direction and all feed `eta_seconds`; changing them perturbs the ETA and the
`PLANNING.md` timing budget for no measured gain.

### 4.3 `DRIVE_COAST_FACTOR = 0.90` — an addition with a diagnostic trap

Golden did not cut drives early (`G:1362`). Today every drive is cut at 90 % of
target (`M:1036`, `M:4532`) and the remaining 10 % is expected from coast.

The reasoning is sound and identical to the turn coast trick. The trap is in the
residual documentation: `M:5796-5799` teaches the operator that "a CONSTANT
RATIO between measured and target … is a SCALE error — counts/mm". But if the
chassis coasts *less* than 10 % of a burst, `DRIVE_COAST_FACTOR` alone produces
a constant ratio below 1.0 on every drive, which looks exactly like a scale
error and is not.

`coast_mm` is unset in the calibration profile (`../SPEC.md:245`), so nobody
knows which it is.

**Recommendation: comment only**, at `M:1036` or in the `_publish_residual`
docstring — "a drive ratio near 0.90 with zero cumulative growth is
`DRIVE_COAST_FACTOR`, not a scale error." No behavioural change; measure
`coast_mm` with `stack/fpms_charact.py --drive` before touching the value.

### 4.4 Arrival distances: 400 mm / 5 mm → 25 mm

`B6_TARGET_STOP_RAW = 400` and `B6_HOME_STOP_RAW = 5` (`G:1910-1911`) were
**marker-referenced** raw LiDAR distances, not arena coordinates. `ARRIVE_TOL_MM
= 25` (`M:975`) is an arena-coordinate radius, and `DOCK_APPROACH_MM = 150`
(`M:966`) is the standoff. These are not comparable quantities and the change is
a consequence of §3.2, not an independent decision. Not flagged as a loss.

---

## 5. The USB port conflict — settled: **drive = 1.3, LiDAR = 1.2**

`fpms_duty_driver.py:172-190` raises this as unresolved. It is resolvable from
the repo, and its own conclusion is right.

**Golden's claim** (`../golden_backup/phase6_latest.py:117-119`):
`D500_PORT` (LiDAR) = `1.3`, `YAHBOOM_PORT` (drive) = `1.2`. Repeated in
`../golden_backup/phase6_perfect_mission.py:95-97` (with a different controller
address, `fc800000`, not `fc880000`) and in the `../rescued/` copies of both.

**Everything else in the stack says the opposite**, and one of them is the unit
file that actually drives the board:

| evidence | says | weight |
|---|---|---|
| `../micro-ros-agent.service:16` `--dev …usb-0:1.3:1.0-port0` | drive = **1.3** | **decisive** — this is the running service |
| `../micro-ros-agent.service:2,13-15` | "ESP32-S3 board (1.3) from the LiDAR (1.2)"; both CP2102s share an ID_SERIAL so only topology distinguishes them | corroborating |
| `../fpms-rover-agent.py:636-637` | `LIDAR_BY_PATH` = 1.2, `UROS_BY_PATH` = 1.3 | corroborating |
| `../fpms-os/overlay/etc/fpms/config.env:92,96` | `FPMS_LIDAR_PORT` = 1.2, `FPMS_DRIVE_PORT` = 1.3 | corroborating — the shipped image |
| `../fpms-os/SPEC.md:199-200` | board 1.3, LiDAR 1.2 | corroborating |
| `../NAV2_BRIEF.md:52-53` | board 1.3 @ 921600, LiDAR 1.2 @ 230400 | corroborating |
| `../firmware_v3/flash_fpms_v3.sh:25,33-34` | "board = usb-0:1.3, lidar = usb-0:1.2 — do not mix these up" | corroborating |
| `../firmware/README.md:285,565` | flashes the board on 1.3 | corroborating — you cannot flash a LiDAR |
| `../fpms_lidar_ros.py:23` | "Only one process can hold …usb-0:1.2" (the LiDAR) | corroborating |

Nine independent files, all later than the golden code, all agreeing, including
two that are executed daily and one that successfully flashes firmware. **The
golden assignment is stale** — the cables were swapped between then and now, as
`D:186` guesses. `fpms_duty_driver.py:191-192` already binds the correct values;
only its header calls the matter open.

**Status: MEASURED-EQUIVALENT** — inferred from a running unit file and a
working flash procedure, not from a `udevadm` capture taken today. `D:188-190`
is right that the confirmation before any first open is
`udevadm info -q path -n <dev>` plus the agent's journal. **Do not resolve it by
trying both.**

**Recommendation:** downgrade `fpms_duty_driver.py:172-190` from "CONFLICT,
UNRESOLVED" to "RESOLVED: 1.3 is the board — golden is stale", keeping the
`udevadm` check. I do not own that file; flagging it.

---

## 6. What I could not determine from the code

Blunt list. None of these is answerable by reading.

1. **Whether `HEADING_KP = 1.2` behaves like `_KPH = 10` did.** Not derivable
   without a duty↔velocity map this firmware does not provide (§4.1).
2. **Whether `DRIVE_COAST_FACTOR = 0.90` matches real coast.** `coast_mm` is
   unset (§4.3).
3. **Whether `TURN_SETTLE_S = 1.2` is right, or merely safe.** No measurement
   anywhere; the only thing known is that golden used 0.25 and got ±1-4°.
4. **Which counts/mm is true.** 6.00 / 5.5 / 14.8 all have provenance and the
   5.5 was taken on a chassis with a loose hub (§3.5).
5. **Whether `TURN_WIRE_SIGN = -1` is correct.** UNMEASURED by the file's own
   admission (`M:772`), and `config.env.example` ships the opposite
   (`../SPEC.md:243`).
6. **What is actually on the Pi.** `../STACK.md:398-403` §9.3 says the repo copy
   was ~1100 lines behind the Pi and that the Pi was off. This audit is of the
   repo file. If the Pi's copy differs, every line number here is a repo line
   number.
7. **Whether the golden code would even run today.** Its transport is the
   framed serial protocol, and the standing memory on this project is that B8B
   ran on different hardware and that transport is unreachable on the current
   board. Parity with B8B is therefore a design target, not a fallback.

---

## 7. Recommendation summary

| # | item | action |
|---|---|---|
| 3.1 | first command is a STOP | **PORT.** One line after `M:5025`. Planner-neutral, revertible, testable |
| 3.2 | marker arrival guard | **NOTHING.** Record the gap; finish verifying the arena fix instead |
| 3.3 | 12 s per-move ceiling | **NOTHING.** Already superseded by `burst_cap_s` |
| 3.4 | host-side deadman | **NOTHING.** Board-side timeout is the right place and exists |
| 3.5 | `TKMM` = 6.00 counts/mm vs measured 5.5 | **Comment fix in `fpms_duty_driver.py:114-119`** (not my file). Do not change the value |
| 3.6 | trims/kick/coast have no home | **NOTHING.** They belong in `firmware_v3` when `/cmd_duty` is wired |
| 4.1 | `HEADING_KP` correspondence | **Comment only**: say it is a shape, not a conversion |
| 4.2 | dwells | **Comment only.** Changing them perturbs the ETA and `PLANNING.md`'s budget |
| 4.3 | `DRIVE_COAST_FACTOR` residual trap | **Comment only**, at `M:1036` or in `_publish_residual` |
| 5 | USB port conflict | **Resolved: drive = 1.3.** Downgrade the header in `fpms_duty_driver.py:172-190` (not my file) |

**One code change is recommended in total**, and it is three zero-`Twist`
messages before the first burst.

---

## See also

| file | what it holds |
|---|---|
| `../GOLDEN_B8B_STUDY.md` | the study this audits — note the LIVE CONSTANTS block (68-78) and the hardware-facts block (101-111) are different claims |
| `../NEXT_SESSION_B8B_DUTY.md` | why `/cmd_vel` cannot crawl, and what wiring `/cmd_duty` costs |
| `../STACK.md` §7, §8, §9 | the firmware change table, Phase 1/2, and the blunt risk list |
| `../fpms_duty_driver.py` | the raw-duty wire; unexecuted, and the home of the port conflict |
| `PLANNING.md` | the planner, the residual stream, and the timing budget that depends on the dwells |
| `../firmware_linorobot/moves_main.cpp` | where the trims, kick and coast compensation actually come from |
