# FPMS Orange Pi 5B — complete file inventory and measured-constants harvest

Survey date **2026-08-05**. Read-only pass over both filesystems on the rover
(`fpms-pi.local`). The eMMC was mounted `-o ro` and unmounted again; `/tmp`
scratch was removed. Nothing on the Pi was modified. `fpms_missions.py` was
being edited live by another agent throughout and was only ever read.

---

## 0. Headline findings

1. **The `.tar.gz` savepoints contain nothing unique.** Every file inside every
   archive on both filesystems already exists byte-identical on the SD card.
   Verified by extracting all 15 archives and comparing 917 member hashes
   against 1,464 SD hashes. Detail in §2.
2. **The eMMC contains nothing unique either.** Its only non-vendor content is
   older or identical copies of SD files. The one directory the previous pass
   had not opened — `FPMS_BACKUPS/phase6_20260524/ros2_src/` — is a snapshot of
   `~/ros2_ws/src` plus the Yahboom `ROSMASTERX3` documentation repo. Detail in §3.
3. **The 6.00 vs 14.8 counts/mm contradiction is settled, in the files, with raw
   evidence.** 14.8 counts/mm is a direct operator tape-measure (800 mm of real
   travel ↔ 11,836 encoder counts). 6.00 is a superseded theoretical value that
   phase5/phase6/teach-in still carry. Detail in §4 — this is the most important
   section in this document.
4. **The "~227 mm minimum executable burst" was recorded**, in two places, before
   today. `fpms_firmware/README.md:47` states "minimum burst ~0.35 s, minimum
   move ~230 mm"; and it is `FULL_DUTY_MPS × MIN_PULSE_S = 0.65 × 0.35 = 227.5 mm`,
   both constants being live in `fpms_missions.py`. Detail in §5.
5. **The board is currently running Yahboom stock firmware, not the custom
   firmware.** `ros2 param list /YB_Car_Node` returns empty — the custom ESP-IDF
   image's parameter server is absent, and `/scan` (which the custom firmware
   deliberately does not publish) is present. So the 50 %-duty dead zone that
   causes the minimum-burst problem is **active right now**. Detail in §6.
6. **`~/fpms_firmware/README.md` (25,931 B) is the single most valuable file on
   the rover.** It is a complete root-cause analysis of the low-speed defect with
   measured numbers throughout. It has been rescued.

---

## 1. Method and scope

| Item | Value |
|---|---|
| Running root | `/dev/mmcblk1p2` (SD card), 29 G, 81 % full |
| eMMC | `/dev/mmcblk0p2`, 29 G, normally unmounted; a stale `dd` clone from April 2026 |
| Archives examined | 15 `.tar.gz` (7 unique by hash), all extracted and diffed |
| Hashes compared | 917 archive members vs 1,464 SD files vs 280 eMMC files |

Vendor ROS workspaces (`~/ros2_ws`, `~/uros_ws`, `~/yahboomcar_ros2_ws`,
`~/yahboomcar_src`, `~/robot_localization`, `~/esp/esp-idf`, `~/upstream_fw`)
were noted but not inventoried, per instruction. `~/ros2_ws/src/fpms_base/` is
the one FPMS-authored package inside them (see §7).

---

## 2. The archives — opened, diffed, and empty of new work

All archives, deduplicated by sha256. Sizes in bytes.

| sha256 (short) | Size | Canonical path | Contents |
|---|---|---|---|
| `7ededd0c` | 2,558,662 | `~/fpms_RELEASE/FPMS_CURRENT_WORKING_SAVE_20260506_222744.tar.gz` | 62 `fpms_B*/b*` mission scripts, May 5–6 |
| `98417a47` | 2,529,762 | `~/fpms_RELEASE/FPMS_FULL_SAVE_20260506_222640.tar.gz` | same minus `fpms_LATEST_WORKING_MISSION.py` |
| `ac23b18e` | 42,005 | `~/fpms_RELEASE/FPMS_CODE_FOR_REBUILD_20260506_214533.tar.gz` | B6 + B7-with-spray + ESP32 spray test |
| `8459cb74` | 39,363 | `~/fpms_RELEASE/FPMS_GOLDEN_B6_STRAIGHT_ONLY_WORKING_20260506_110712.tar.gz` | `fpms_B6_STRAIGHT_ONLY.py` 78,091 B |
| `60a30cfb` | 57,704 | `~/fpms_RELEASE/FPMS_GOLDEN_B84_LIDAR_GAP_WORKING_20260506_221908.tar.gz` | `fpms_B8_SAFE_REBUILD.py` 140,160 B |
| `07fd1891` | 60,074 | `~/fpms_RELEASE/FPMS_GOLDEN_AUTONOMOUS_DRIVING_B86_FINAL_20260507_120619.tar.gz` | `fpms_B8_SAFE_REBUILD.py` 145,011 B — **newest B-lineage build**; 4 identical copies on SD, none on eMMC |
| `d22e11d0` | 5,626 | `~/fpms_teachin/FPMS_TEACHIN_WORKING_20260521_233828.tar.gz` | teach-in server + `routes_FINAL.json` |
| 7 × ~24 MB | — | `~/fpms_BACKUPS/fpms_*.tar.gz` | `~/fpms` + `~/fpms_RELEASE` **including a 3,700-file `.venv`**; the venv is >95 % of the bytes |

**Result of the diff: zero archive-unique files.** Two candidate hits
(`Rosmaster_Lib.py`, `fpms_base/base_driver.py`) turned out to be artifacts of
depth/exclusion limits in the first sweep; both were re-checked directly and are
byte-identical to live SD copies. The archives are pure redundancy.

Every eMMC archive is byte-identical to an SD archive of the same name. The
eMMC adds no archive that the SD lacks; the SD has two the eMMC lacks
(`B86_FINAL`, `TEACHIN_WORKING`).

**Recommendation: the 7 × 24 MB `~/fpms_BACKUPS` tarballs are ~170 MB of
duplicated virtualenv and can be deleted whenever space is needed.** Nothing
references them. (Not done — this pass was read-only.)

---

## 3. eMMC contents

`/mnt/emmc` is a stale April 2026 `dd` clone of an older root, plus:

| Path | Size | Verdict |
|---|---|---|
| `FPMS_BACKUPS/{20260529_2121,20260529_2159}/` | 2 files each | `fpms_phase6_birdeye.py` 119,493 B — identical to SD `fpms_phase6_LATEST.py` |
| `FPMS_BACKUPS/LED_WORKING_20260529/` | 2 files | identical to SD `fpms_phase6_LED_WORKING.py` |
| `FPMS_BACKUPS/PERFECT_MISSION_20260526/` | 5 files | + `librknnrt.so` 7.3 MB, `yolov8n.rknn` 4.3 MB (NPU runtime + model) |
| `FPMS_BACKUPS/PRE_DASHBOARD_20260527_2332/` | 3 files | identical to SD |
| `FPMS_BACKUPS/phase6_20260524/` | `fpms_phase5_GOLDEN.py`, `fpms_phase6_WORKING.py` (×2) | both identical to SD copies (`~/fpms_phase5_GOLDEN.py`, `~/fpms_RELEASE/phase6/fpms_phase6_WORKING.py`) |
| `FPMS_BACKUPS/phase6_20260524/ros2_src/` | 26 dirs | snapshot of `~/ros2_ws/src`; `fpms_base/` identical to live |
| `…/ros2_src/ROSMASTERX3/` | 530 MB git pack + ~60 PDFs | **Yahboom vendor documentation — the only copy on the rover.** Includes "13. Timer captures the encoder data.pdf", "12. Control motor.pdf", "14. Robot kinematics analysis theory" |
| `…/ros2_src/ldlidar_ros2/` | LiDAR SDK + launch files | `d500.launch.py` etc.; vendor, not on SD |
| `home/ubuntu/fpms_RELEASE_BACKUP_20260506_223000/` | 6 archives | all byte-identical to SD copies |

**The `ROSMASTERX3` documentation set is the one thing on the eMMC worth
keeping**, because it is the authoritative source for the encoder derivation
(`13 × 20 × 4 = 1040`) and the GPIO table, and it exists nowhere else on the
rover. It is 530 MB and was not downloaded in this pass. If the counts/rev
question ever needs an independent check, `13. Timer captures the encoder
data.pdf` is where to look.

---

## 4. MEASURED CONSTANTS

Every row cites file and line. **Bold = directly measured against physical
reality.** Paths are on the Pi unless marked.

### 4.1 The counts-per-mm question — RESOLVED

The operator's live contradiction (phase6 derives 6.00, the notes say 14.8) is
settled by a file that measured it directly.

> `~/fpms_saved/fwd800_main.cpp:34–37`
> ```
> /* CORRECTED from the operator's tape measure: the rover really travelled 800mm
>  * while the encoders reported 11836 counts -> 14.8 counts/mm. The earlier 27.7
>  * came from a rough eyeball estimate and was 1.9x too high, so every distance
>  * computed with it was short by nearly half. */
> const float COUNTS_PER_MM = 14.8f;
> ```

11,836 / 800 = **14.795 counts/mm**. This is a tape measure against real travel —
the only empirical counts/mm number anywhere on the rover.

It was then propagated into the firmware config, with the arithmetic shown:

> `~/lino/config/custom/fpms_config.h:96–104`
> ```
> /* CORRECTED 2026-08-04 from an operator tape-measure. 14.8 counts/mm was
>    measured over 800mm of real travel; 14.8 * pi * 70mm = 3255 counts/rev.
>    The old 1320 came from the golden driver, which counted in a different mode,
>    and is 2.5x too small -- the velocity PID would drive 2.5x faster than the
>    commanded m/s and every odometry distance would read short by the same. */
> #define COUNTS_PER_REV1 3255
> ```

Check: 14.795 × π × 70 mm = 14.795 × 219.911 = **3,253.6 ≈ 3255**. Self-consistent.

The full lineage of the number, in order:

| Value | Where | Basis | Status |
|---|---|---|---|
| 27.7 counts/mm | `fwd200_main.cpp:35`, `fwd400_main.cpp` (superseded) | "a 3 s crawl produced ~5532 counts and travelled ~200 mm" — **eyeball estimate of the distance** | **WRONG, 1.9× too high**, retracted in `fwd800_main.cpp:35` |
| 24.0 counts/mm | `fwd200_main.cpp:36` | theory: 1320 CPR through `attachFullQuad` on a 70 mm wheel | theoretical only; never measured |
| **14.8 counts/mm** | **`fwd800_main.cpp:38`, `moves_main.cpp:39`, `KEEP/moves_main.cpp:39`** | **800 mm tape measure ↔ 11,836 counts** | **MEASURED — use this** |
| 6.00 counts/mm | `fpms_phase6_LATEST.py:1309` and 11 sibling files (`_TKMM=_m.pi*70.0/1320.0`) | derived from 1320 CPR; `_TKMM` is 0.1666 **mm per tick**, i.e. 6.00 counts/mm | **superseded**, still live in phase5/phase6/teach-in |
| 5.32 counts/mm | `fpms_firmware/README.md:392` | operator's Arduino driver: 1170 CPR / 219.9 mm | self-consistent but not measured on this rover |
| 6.90 counts/mm | `fpms_firmware/README.md:393` | Yahboom stock: 1040 CPR (13 lines × 20 × 4) / 150.8 mm | 150.8 mm is not a 70 mm wheel |
| 5.66 counts/mm | `fpms_firmware/README.md:395` | third-party repo, 2244 CPR / 396.4 mm | **a different robot** — README says do not copy |

**14.8 / 6.00 = 2.465.** A stack still using `_TKMM` over-reports distance by
2.47×. The operator today watched ~50 mm of travel reported as 182 mm — a 3.64×
over-report. The 2.47× constant error accounts for most of that; the remainder is
consistent with wheel slip and coast, which `moves_main.cpp` documents separately
(see 4.3). **The direction and rough magnitude match, so `_TKMM` is at minimum a
large part of today's failure.**

### The physical cause: the gearbox is 74:1, not 30:1

`fpms_missions.py:811–826` already contains the reconciliation, and it is exact:

> ```
> # ODOM SCALE IS UNRESOLVED AND IS DELIBERATELY NOT CORRECTED HERE.
> # Two independently derived numbers disagree by 2.47x:
> #   6.00 counts/mm  = pi*70/1320, from 11 magnetic lines x 30:1 gear x 4
> #                     quadrature (fpms_phase6_LATEST.py:1309 `_TKMM`, and the
> #                     teach-in server). DERIVED from an assumed gearbox.
> #  14.8 counts/mm  = operator tape measure over 800mm of real travel,
> #                     2026-08-04 (fpms_config.h:96). MEASURED.
> # 14.8/6.00 = 2.467, which is exactly 74/30: a 74:1 gearbox where 30:1 was
> # assumed. 14.8 * pi * 70mm = 3255 counts/rev, and 44 counts/motor-rev x 74 =
> # 3256. The MEASURED number is the one consistent with the observed
> # over-reporting...
> ```

**2.467 = 74/30 exactly**, and 44 counts/motor-rev × 74 = 3256 ≈ 3255. So the
discrepancy is not a mystery scale factor: the `GEAR_RATIO_BY_RPM` table in the
teach-in server (`teachin_server_8089_FINAL.py:11`,
`{"550": 19, "333": 30, "205": 56}`) was indexed at `MD520_RPM = "333"` → 30:1,
but the fitted motor is a 74:1 unit. Every derivation that starts from
"11 lines × 30 × 4 = 1320" inherits that error.

This closes the loop with the measurement: 14.8 counts/mm is both tape-measured
*and* explained by an integer gearbox ratio. Treat it as settled.

One caveat remains, which no file resolves: the 14.8 was measured with the
**bare-metal Arduino/ESP32Encoder firmware reading the encoder pins directly**,
while `_TKMM` is applied to counts arriving from the **Yahboom micro-ROS firmware
over ROS**. The gearbox argument applies to both (it is mechanical), so the two
should agree — but that has not been demonstrated end to end. Confirm with one
tape measure read through `/odom_raw` before relying on it.

Note also `~/test_odom_projection.py:137` already hard-codes
`scale_ratio = 14.8 / 6.00`, and `fpms_missions.py:6203` surfaces the conflict in
an operator-facing message. The project has known about this; it was never applied.

### 4.2 Minimum move / burst / dead zone

| Constant | Value | Source | Basis |
|---|---|---|---|
| PWM full scale | 400 ticks (10 MHz / 25 kHz) | `fpms_firmware/README.md:41` | vendor source |
| `PWM_MOTOR_DEAD_ZONE` | **200 ticks = exactly 50.0 % duty** | `README.md:27,42` | vendor source — the root cause |
| Smallest non-zero duty | **50.25 %** | `README.md:43` | arithmetic; bottom half of the range does not exist |
| **Minimum burst** | **~0.35 s** | **`README.md:47`; `fpms_missions.py:787` `MIN_PULSE_S = 0.35`** | **measured, WHEELS OFF** (`fpms_missions.py:280`); `MIN_PULSE_MEASURED_UNDER_LOAD = False` (`:788`) |
| **Minimum move** | **~230 mm** | **`README.md:48`** | **measured** |
| Minimum move (derived) | **227.5 mm** | `FULL_DUTY_MPS = 0.65` (`fpms_missions.py:1372`) × `MIN_PULSE_S = 0.35` (`:787`) | the ~227 mm number, reconstructible from live constants |
| `MIN_MOVE_MM` as coded | ~63 mm | `fpms_missions.py:156,792` = `CRUISE_MPS × MIN_PULSE_S × 1000` at 0.18 m/s | **CONTRADICTION — see §5** |
| `MIN_TURN_DEG` | ~9° | `fpms_missions.py:157,793` = `TURN_RADPS 0.45 × 0.35 s` | derived |
| Observed failure | 300 mm commanded → **~1 m travelled**, collision | `README.md:48,319` | measured |
| Recommended MIN_PWM floor | 7.5–15 % | `README.md:96` (cites `R4_FIRMWARE.md`) | analysis, not measured |
| Deadband sweep result | **UNKNOWN — run aborted** | `~/sweep.log`, `~/deadband_sweep_state.json` | see §4.7 |

### 4.3 Open-loop drive constants (bare-metal calibration firmware)

From `~/KEEP/moves_main.cpp` — the richest single source of measured motion
constants on the rover. All measured 2026-08-04.

| Constant | Value | Line | Basis |
|---|---|---|---|
| `BASE_DUTY` | 70 (8-bit) | `:29` | crawl duty |
| `KICK_DUTY` / `KICK_MS` | 140 / 220 ms | `:30–31` | **measured**: at duty 43–52 from standstill the motors only buzzed; a kick breaks stiction |
| `TRIM[4]` forward | 0.820, 0.839, 0.914, 1.000 | `:32` | **measured**: at duty 62 for 3 s, counts were M1 20288, M2 19988, M3 18730, M4 17677 → right side ran ~10 % faster (`crawl_enc_main.cpp:31–37`) |
| `TRIM_REV[4]` reverse | 0.820, 0.737, 0.894, 0.960 | `:38` | **measured**: with forward trims, a reverse leg gave M1 753.6, M2 857.3, M3 770.8, M4 785.3 mm — M2 overruns ~14 % |
| `COAST_FWD` | 1270 counts (≈86 mm) | `:42` | **measured overshoot** |
| `COAST_REV` | 1448 counts (≈98 mm) | `:43` | **re-measured**: cutting at 9880 gave 11328 counts |
| `TURN_DUTY` | 95 | `:45` | turning scrubs all four tyres |
| `TURN_COAST_L_DEG` | 38.0° | `:49` | **measured**: 36 → 95.8°, 42 → 78.0°; interpolates to 38 for 90° |
| `TURN_COAST_R_DEG` | 27.0° | `:50` | **measured**: 28 → 87.1° |
| `SLIP_ABORT_FRAC` | 0.55 | `:54` | **measured**: 0.35 tripped on a healthy reverse (35 % spread); the loose-wheel failure was 292 % |
| `SLIP_CHECK_AFTER` | 7000 counts (≈470 mm) | `:55` | consistent with 14.8 counts/mm (7000/470 = 14.89) |

Left/right turn coast genuinely differ (38° vs 27°) — the file is explicit that
this is not symmetric and must not be collapsed to one number.

### 4.4 Wheel / chassis geometry

| Constant | Value | Source | Status |
|---|---|---|---|
| Wheel diameter | 0.070 m | `lino/config/custom/fpms_config.h:106`; `README.md:376` | consistent everywhere |
| Wheel circumference | 219.9 mm | `README.md:376` | = π × 70 |
| `COUNTS_PER_REV` (current) | **3255** | `fpms_config.h:101–104` | **derived from the 14.8 measurement** |
| `COUNTS_PER_REV` (deployed) | 1320 | `~/fpms_config_deployed.h:69–72` | **superseded** |
| `LR_WHEELS_DISTANCE` (track) | 0.170 m | `fpms_config.h:110`; `README.md:377`; `fpms_odom_tf.py:368`; `rover_config.h:123` | **UNMEASURED** — "Track is still unmeasured — measure it before trusting turn geometry" (`fpms_config.h:107–109`) |
| `WHEELBASE_M` (follower) | **0.105 m** | `fpms_rtos_follower.py:190`, labelled "the MEASURED 105 mm" | **third live value — see §5.9** |
| Wheelbase (front↔rear) | **105 mm** | `fpms_config.h:107–108` | **measured** — explicitly *not* the track |
| Wheelbase (alt figure) | 100 mm | `README.md:413` | **CONTRADICTION with 105 mm — see §5** |
| Effective track rule | 1.2–1.8× geometric | `README.md:412` | skid-steer scrub; a second figure of 1.5–2.5× appears at `README.md:610` — **CONTRADICTION** |
| `MOTOR_MAX_RPM` | 180 | `fpms_config.h:86` | estimate; ~0.66 m/s on a 70 mm wheel |
| `FULL_DUTY_MPS` | **0.65 m/s** | `fpms_missions.py:1372`; `README.md:378,486` | **measured** at the old 50 % floor |
| `MOTOR_POWER_MEASURED_VOLTAGE` | 11.6 V | `fpms_config.h:90` | **measured 2026-08-03**, pack was low; corroborated by `~/battcheck.log` (`battery_raw 116 volts=11.6`) |
| `bat_divider` | 5.0 | `README.md:379` | **GUESS** — "the divider ratio is undocumented" |

### 4.5 Polarity — measured, and corrected once

`~/lino/config/custom/fpms_config.h:122–168` records the full measurement and a
retraction. Raw encoder deltas, one motor at a time (`enc_main.cpp`):

```
  motor   own encoder on FWD        own encoder on REV
  M1      -11802 / -12158           +13015 / +11508
  M2      -16356 / -17199           +13021 / +12808
  M3      -12835 / -11153           +9419  / +8788
  M4      -10850                    +12824
```

First reading: "all four count negative on forward → invert all four." That was
**wrong**, and the file says why (`:140–153`): an open-loop test the operator
watched showed that driving all four on `IN_A` ran the LEFT side forward and the
RIGHT side backward — a pure wiring fact, no encoders involved. So `IN_A` is not
"forward" for the right pair, and their negative counts were already correct.
**Inverting all four made it worse: a commanded 564 mm produced −2080 mm of
phantom travel.**

Final, measured values:

| Setting | Value | Line |
|---|---|---|
| `MOTOR1_ENCODER_INV` (front-left, board M3/H3) | true | `:155` |
| `MOTOR2_ENCODER_INV` (front-right, board M1/H1) | false | `:156` |
| `MOTOR3_ENCODER_INV` (rear-left, board M4/H4) | true | `:157` |
| `MOTOR4_ENCODER_INV` (rear-right, board M2/H2) | false | `:158` |
| `MOTOR1_INV` / `MOTOR3_INV` (left pair) | false | `:165,167` |
| `MOTOR2_INV` / `MOTOR4_INV` (right pair — **wired opposite**) | true | `:166,168` |

Corroborated independently in every bare-metal test file:
`const bool ON_IN_B[4] = {true, true, false, false};` with the comment "Right
pair (M1, M2) is wired opposite: forward = duty on IN_B"
(`fwd200_main.cpp:28`, `fwd400`, `fwd800`, `crawl_enc_main.cpp:23`).

Pin map, consistent across all bare-metal files and `fpms_config.h:175–182`:

```
IN_A[4]  = {4, 15, 9, 13}    // board M1, M2, M3, M4
IN_B[4]  = {5, 16, 10, 14}
ENC_A[4] = {6, 47, 11, 1}    // board H1, H2, H3, H4
ENC_B[4] = {7, 48, 12, 2}
```

Side assignment (from the 2026-08-01 rewire): M1 = front-right, M2 = rear-right,
M3 = front-left, M4 = rear-left (`fpms_config.h:170–173`, `README.md:156`). This
**contradicts the golden driver's older `set_motor(L,L,R,R)` comment**; a
mirrored side assignment leaves FORWARD correct and inverts every TURN.

### 4.6 Host-side compensation factors (all flagged as defects, not calibrations)

| Constant | Value | Source | README verdict |
|---|---|---|---|
| `CMD_SCALE` | **6.1** | `fpms_missions.py:704`; `fpms_teleop.py:106` | "**a symptom, not a calibration** … That factor is the defect … should collapse toward 1.0 … **Do not carry 6.1 across**" (`README.md:164–175`). A comment at `fpms_missions.py:287` records a different value, `CMD_SCALE = 1.0`, "measured once, free-spinning" — **CONTRADICTION** |
| `TURN_WIRE_SIGN` | **−1** | `fpms_missions.py:771` | "**DERIVED FROM THE REWIRING, NOT MEASURED**" (`:194,278`). README: "It must become +1" after reflash (`:160`) |
| `ODOM_TWIST_SIGN` | −1 in 3 files | `fpms_teleop.py:150`, `fpms_odom_tf.py:387`, `deadband_sweep.py:154` | must become +1 with the custom firmware (`README.md:145–149`) |
| `ODOM_TWIST_ANG_SIGN` | +1 | `fpms_teleop.py:156` | stays +1 |

Measured twist-sign table, wheels off the ground (`README.md:132–136`):

| commanded `linear.x` | reported `twist.linear.x` | pose displacement |
|---|---|---|
| +0.012 | mean **−0.842** | **+1.395** (forward) |
| +0.100 | mean **−1.225** | **+3.505** (forward) |
| −0.012 | mean **+0.574** | **−1.506** (backward) |

Hence the host-stack rule *"TRUST POSE. NEVER TRUST TWIST."*

### 4.7 Recorded runs — what the logs actually contain

| File | Size | What is in it |
|---|---|---|
| `~/sweep.log` | 2,168 B | Deadband sweep, 2026-07-31 16:01. **ABORTED after 6 steps.** `MIN_CMD_LIN = UNKNOWN`, `MIN_CMD_ANG = UNKNOWN`. Aborted because "observed lin=−0.2973 opposite to commanded +0.0120". Caveat in the file: no-load measurement, treat as a **lower bound** |
| `~/deadband_sweep_state.json` | 3,153 B | The raw steps. **Every step from 0.002 to 0.012 m/s reports `mean_early = 0.0`, `mean_late = 0.0`, `motion: false`.** Odometry reported literally zero velocity throughout. Config: `step_lin 0.002`, `cap_lin 0.1`, `step_ang 0.004`, `cap_ang 0.2`, `hold_s 1.5`, `control_hz 20` |
| `~/deadband_sweep_state.json.dryrun` | 25,267 B | Simulated run, not hardware. No measurement value |
| `~/turnloop.log` | 458 B | 6 × (left +1.2 rad/s 3 s, right −1.2 rad/s 3 s). **No measurements recorded, only the commands.** This is the destructive turn test — it destroys heading and yields nothing |
| `~/map_odom.log` | 324 B | `map`→`odom` static transform: translation (0.972, 0.228, 0), rotation quaternion (0, 0, 0.707107, 0.707107) = **+90° yaw**. Matches `arena_zones.json` start pose exactly |
| `~/mqtt.log` | 908 B | **LiDAR: `zero_offset = 0.0 deg`, `sign = −1`, marked "UNVERIFIED, run the wall test".** Rate 9.38 Hz, 335/360 bearings returned, 0 dropped |
| `~/battcheck.log` | 1,768 B | `/odom_raw` 11.15 Hz, `/battery` 1.00 Hz, `/imu` 25.05 Hz, `/scan_lidar` **0.00 Hz**. `battery_raw 116 → 11.6 V`. `ORIENTATION_IS_IDENTITY = True` (IMU quaternion not fused). Pose stayed at exactly 0 for the whole capture |
| `~/fpms_mission.log` | 241,284 B | Mission executor log, last written 2026-05-31 |
| `~/agent_v6.log` | 16.4 MB | Rover agent log, 2026-08-03 |
| `~/fpms_teachin/routes.json` | 992 B | A real teach-in recording — see below |

`routes.json` holds one taught route: 5 × "Forward 10cm" (`vx 0.1`, 1.0 s) plus
a strafe, then 2 × "Turn left 45 degrees" (`vz 0.55`, 1.427996660722633 s).
**Both are derived, not measured**: 0.1 m/s × 1.0 s = 100 mm exactly, and
0.55 rad/s × 1.428 s = 0.7854 rad = 45.000°. Their only evidential value is that
the operator accepted those commands as producing those distances. The two
`routes_FINAL*.json` files are empty (`{"1": [], "2": []}`).

### 4.7b LiDAR mount — every value is a placeholder, and the placeholders are live

`~/nav2/fpms_tf.launch.py` publishes `base_link → laser_frame`. Its own header
says the block is `PLACEHOLDER DEFAULTS. UNMEASURED.` (`:203`):

| Constant | Value | Line |
|---|---|---|
| `LASER_X_DEFAULT` | 0.0 — `# MEASURE ME` | `:209` |
| `LASER_Y_DEFAULT` | 0.0 — `# MEASURE ME` | `:210` |
| `LASER_Z_DEFAULT` | 0.065 — `# MEASURE ME (mast guess)` | `:211` |
| `LASER_ROLL/PITCH_DEFAULT` | 0.0 — `# MEASURE ME (level the scan plane)` | `:212–213` |
| `LASER_YAW_DEFAULT` | 0.0 — `# MEASURE ME (1 deg = ~10.5 mm of position error)` | `:214` |
| `BASE_LINK_Z_M` | 0.035 (= wheel radius) | `:199` |

The file documents an example invocation with real-looking numbers at `:154`
(`laser_x:=0.052 laser_y:=0.000 laser_z:=0.071 laser_yaw:=-0.0122 measured:=true`)
— **those are an example, not a configuration.**
`/etc/systemd/system/fpms-tf.service:11` launches the file **with no arguments**,
so the 0.0 placeholders are what is actually published.

Error sensitivity, from the file's own analysis (`:71`, `:89–90`):
1° of mount yaw ≈ **10.5 mm** of position error; at h = 0.10 m, 1° of pitch puts
the floor strike at 5.7 m, 2° at 2.9 m.

Scanner side: `fpms_lidar_ros.py:164` `LIDAR_ZERO_OFFSET_DEG = 0.0`,
`:168` `LIDAR_ROTATION_SIGN = -1` — the latter marked UNVERIFIED, matching
`~/mqtt.log`'s runtime line *"mount: zero_offset=0.0 deg sign=-1 ← UNVERIFIED,
run the wall test"*. `fpms_tf.launch.py:141–143` warns that
`LIDAR_ZERO_OFFSET_DEG` and `laser_yaw` **double-correct** if both are set.

Also: `~/fpms_odom_tf.py:606` `LIDAR_MAX_SPAN_ERROR_M = 0.10` (checks
`|d_near + d_far − 1.200|`, i.e. it assumes the 1200 mm arena), `:607`
`LIDAR_MAX_JUMP_M = 0.30`.

### 4.7c Runtime overrides — there are none

Every `/etc/systemd/system/fpms-*.service` unit (11 of them) was checked. They
set only `ROS_DOMAIN_ID=20`, `RMW_IMPLEMENTATION`, `PYTHONUNBUFFERED`, `HOME`,
plus `FPMS_FOLLOW_DRY_RUN=1` and `FPMS_TUNNEL_ALLOW_CMDVEL=0`.

**No calibration constant is overridden at runtime.** Every value in this
document is the in-source default. `fpms_missions.py:807` references an
`/etc/fpms/config.env` that **does not exist**, so all the `_cfg_float` /
`_cfg_sign` env hooks fall through to their defaults. Changing a constant means
editing the source, not setting an environment variable.

Also note the teleop deadband floors are **off**: `fpms_teleop.py:278–279`
`MIN_CMD_LIN = 0.0`, `MIN_CMD_ANG = 0.0`, with the comment "floor OFF — no linear
deadband has been demonstrated" (`:230–231`).

### 4.8 Arena geometry

From `~/nav2/arena_zones.json` — self-described as derived from
`frontend/src/lib/arena.ts`, "the source of truth for these numbers".

| Item | Value |
|---|---|
| Arena | **1200 × 1200 mm**, origin bottom-left, +x right, +y up |
| `zone-a` | 360 × 360 mm at (48, 792), centre (228, 972) |
| `zone-b` | 360 × 360 mm at (792, 792), centre (972, 972) |
| `water-station` | 360 × 360 mm at (48, 48), centre (228, 228) |
| **Start pose** | **(972, 228) mm = (0.972, 0.228) m, heading 90° (1.5707963 rad)** |
| Map resolution | 0.01 m/px, origin `[-0.1, -0.1, 0]` (`arena.yaml`, `arena_map.yaml`) |

`~/fpms_markers.json` uses a **different, incompatible frame**: M1 (−189, 1058),
M2 (−1227, 111), HOME (0, −500) — negative coordinates, so not the 0–1200 arena
frame. **CONTRADICTION — see §5.**

`~/.fpms_teleop_origin.json` anchors the teleop frame:
`ref_x −3.053122, ref_y −4.556103 → x_mm 972.0, y_mm 228.0` (2026-08-04).

### 4.9 Timing and interface

| Item | Value | Source |
|---|---|---|
| ROS domain | **20** (hard-wired) | `README.md:188`, `TF_TREE.md` |
| Serial | UART0 @ **921600** | `README.md:188` |
| `/odom_raw` | 10 Hz | `README.md:197`; measured 11.15 Hz (`battcheck.log`) |
| `/imu` | 25 Hz, orientation **not** fused | `README.md:198`; measured 25.05 Hz |
| `/battery` | 1 Hz, **decivolts** (÷10) | `README.md:199` |
| micro-ROS agent reconnect | **90–225 s, measured** | `README.md:293–294` |
| First micro-ROS build | 10–25 min | `README.md:243` |
| `ODOM_STALE_SEC` | 2.0 | `TF_TREE.md` |
| LiDAR rate | 9.38 Hz, 335/360 bearings | `mqtt.log` |

---

## 5. Contradictions — flagged explicitly

1. **counts/mm: 6.00 vs 14.8 (2.47×).** RESOLVED in favour of **14.8**, which is
   the only tape-measured value (§4.1). 6.00 (`_TKMM`) is still live in 12
   phase5/phase6 files and in the teach-in server. Residual unknown: whether the
   ROS-side count stream scales the same as the bare-metal one.
2. **`MIN_MOVE_MM`: 63 mm vs 227–230 mm.** `fpms_missions.py:792` computes
   `MIN_MOVE_MM = CRUISE_MPS × MIN_PULSE_S × 1000` ≈ 63 mm at 0.18 m/s, but the
   rover does not cruise at 0.18 m/s during a minimum burst — the dead zone puts
   it at `FULL_DUTY_MPS = 0.65`, giving 227.5 mm, and the README measured ~230 mm.
   **The coded floor is 3.6× too small.** This is very likely the specific number
   that cost the session.
3. **`CMD_SCALE`: 6.1 vs 1.0.** `fpms_missions.py:704` sets 6.1; the doc comment
   at `:287` says "CMD_SCALE = 1.0, measured once, free-spinning". README says
   6.1 is a defect artifact and must be re-measured after reflash.
4. **Wheelbase: 105 mm vs 100 mm.** `fpms_config.h:107` says the operator
   measured **105 mm** front-to-back; `README.md:413` says "the 100 mm figure
   that appears elsewhere". Minor, but both are presented as the same quantity.
5. **Effective-track multiplier: 1.2–1.8× vs 1.5–2.5×.** `README.md:412` vs
   `README.md:610`, same document. Track itself is **unmeasured** either way.
6. **Marker frame vs arena frame.** `fpms_markers.json` has negative mm
   coordinates; `arena_zones.json` is a 0–1200 mm first-quadrant frame. They
   cannot both be the world frame.
7. **Side assignment.** The 2026-08-01 rewire (M1 front-right … M3 front-left)
   contradicts the golden driver's `set_motor(L,L,R,R)`. Forward stays correct;
   **every turn inverts.** This is what `TURN_WIRE_SIGN = -1` papers over, and
   that sign was never measured.
8. **`enc_counts_per_rev` 1170 vs 1320 vs 3255.** README and
   `fpms_firmware/components/rover_config/include/rover_config.h:112` default to
   1170 and call the project briefs' number wrong; `fpms_config_deployed.h` uses
   1320; the current `fpms_config.h` uses 3255. **Only 3255 traces to a
   measurement**, and only 3255 is explained by the 74:1 gearbox (§4.1).
9. **Track width has three live values, none measured.** 0.170 m
   (`fpms_config.h:110`, `fpms_odom_tf.py:368`, `rover_config.h:123`); 0.105 m
   (`fpms_rtos_follower.py:190`, labelled "the MEASURED 105 mm"); and
   `fpms_config.h:107–109` states plainly that the operator's 105 mm was
   **front-to-back — the wheelbase, not the track**. So
   `fpms_rtos_follower.py` is very likely dividing yaw rate by a wheelbase.
   Compounding it, `fpms_odom_tf.py:368` comments "wheelbase and track are equal
   on this chassis", which the 105 mm measurement contradicts.
10. **LiDAR mount double-correction risk.** `LIDAR_ZERO_OFFSET_DEG`
    (`fpms_lidar_ros.py:164`) and `laser_yaw` (`fpms_tf.launch.py`) both rotate
    the scan; `fpms_tf.launch.py:141–143` warns they must not both be set. Both
    are currently 0.0, so nothing is corrected at all — and `mqtt.log` says the
    sign is UNVERIFIED.
11. **Two arena frames plus two stale ones.** Live: 1200 × 1200 mm
    (`fpms_missions.py:520` `ARENA_MM = 1200.0`). Stale, in the old `~/fpms/`
    UI generation: 1600 × 900 (`fpms/fpms_d500_live.py:10–11`) and 2000 × 2000
    (`fpms/fpms_a3.py:11–12`). Plus the incompatible `fpms_markers.json` frame
    (item 6). Only the 1200 mm frame is current.
12. **`TURN_WIRE_SIGN` is explicitly marked unverified in code.**
    `fpms_missions.py:772` `TURN_WIRE_SIGN_MEASURED = False`, with the comment
    "flip this ONLY after a real turn confirms it". It never was.

---

## 6. Firmware state — three lineages, and what is actually running

There are **three** distinct firmware efforts on the rover:

| Lineage | Location | State |
|---|---|---|
| Yahboom stock micro-ROS V2.0.0 | `~/microROS_Robot_V2.0.0.bin` (1,517,120 B, Aug 4 15:58); backups `~/firmware_backup/yahboom_microros_v2_stock.bin`, `~/stock_backup_preflash.bin` | flashed by `~/flash_factory.sh` (erases NVS, writes at 0x0) |
| Custom ESP-IDF (`fpms_firmware`) | `~/fpms_firmware/` — README documents a **verified build 2026-08-02** and a successful flash the same day | **built and flashed Aug 2, but no longer running** |
| linorobot2 (`~/lino`) | dirty git repo, uncommitted; `config/custom/fpms_config.h` carries the 3255 CPR and the measured polarity | never deployed as far as the files show |

**What is running right now (passively checked, 2026-08-05):**

```
nodes:  /YB_Car_Node  /fpms_missions  /fpms_odom_tf  /fpms_teleop
topics: /battery /beep /cmd_vel /imu /odom /odom_raw /scan /scan_lidar ...
ros2 param list /YB_Car_Node  ->  (empty)
```

The custom firmware's whole selling point is a runtime parameter server
(`enc_counts_per_rev`, `min_pwm_percent`, …) and it deliberately does **not**
publish `/scan` (`README.md:177–182`). The parameter list is empty and `/scan` is
present, so **the board is on Yahboom stock firmware.** It was reflashed on
2026-08-04 via `flash_factory.sh`.

**Consequence: the 50 %-duty dead zone is active.** The ~227 mm minimum burst,
the inability to make short moves, and the lurching are all expected behaviour of
the image currently on the board. None of the `fpms_config.h` corrections
(3255 CPR, per-wheel polarity) are in effect either — that file belongs to the
undeployed linorobot2 build.

`~/lino` remains dirty with uncommitted FPMS work: modified `config/config.h`
(adds `#ifdef USE_FPMS_CONFIG` include hook), `firmware/lib/imu/imu.h`,
`firmware/platformio.ini`, `firmware/src/firmware.cpp`; untracked
`config/custom/fpms_config.h` and `firmware/lib/imu/icm42670_imu.h`. HEAD is
`30c872b battery: fix unused warning of skip_dip`. **This work exists only in the
working tree — it is not committed anywhere.**

---

## 7. File inventory by purpose

Sorted by mtime within each group. Sizes in bytes. `sha` = first 16 hex chars.
"≡" marks byte-identical duplicates.

### 7.1 Dashboards / UI (mission GUIs)

| mtime | Size | Path | sha | Note |
|---|---|---|---|---|
| 2026-05-22 00:02 | 97,215 | `~/fpms_phase5_WORKING_OBS_AVOID.py` | `e4ba0eb325015a6e` | |
| 2026-05-22 00:17 | 97,380 | `~/fpms_phase5_WORKING_FULL_MISSION.py` | `f11f382a61f6542b` | |
| 2026-05-22 01:00 | 103,214 | `~/fpms_phase5_LATEST.py` | `816189a6730c2367` | |
| 2026-05-22 11:24 | 104,596 | `~/fpms_phase5_WORKING.py` | `1a29a233b3b7ccc9` | |
| 2026-05-23 01:42 | 103,551 | `~/fpms_phase5_GOLDEN.py` | `6d27dc7d9953f9d4` | ≡ eMMC `phase6_20260524/fpms_phase5_GOLDEN.py` |
| 2026-05-23 17:05 | 103,433 | `~/fpms_phase5.py` | `e82be82187612d39` | |
| 2026-05-24 01:18 | 110,646 | `~/fpms_RELEASE/phase6/fpms_phase6_WORKING.py` | `2fb7d98f54ace35e` | ≡ 2 eMMC copies |
| 2026-05-26 11:25 | 117,662 | `~/fpms_phase6_PERFECT_MISSION.py` | `8dbb64e0602f5ae9` | ≡ eMMC PERFECT_MISSION + PRE_DASHBOARD |
| 2026-05-27 23:56 | 117,242 | `~/fpms_phase6_M1_M2_WORKING.py` | `7b62d85e849202b5` | |
| 2026-05-29 20:49 | 117,816 | `~/fpms_phase6_LED_WORKING.py` | `73c9a5c344cae843` | ≡ eMMC LED_WORKING |
| 2026-05-29 21:18 | 119,493 | `~/fpms_phase6_birdeye.py` | `9bc5b9248c39bc67` | ≡ ↓ |
| 2026-05-29 21:58 | **119,493** | **`~/fpms_phase6_LATEST.py`** | `9bc5b9248c39bc67` | **best phase6 build; Flask :8085** |
| 2026-08-04 15:33 | 119,493 | `~/KEEP/fpms_phase6_LATEST.py` | `9bc5b9248c39bc67` | ≡ ↑ |

**There are only 11 distinct phase5/phase6 builds, not 20.** `fpms_phase6_LATEST.py`,
`fpms_phase6_birdeye.py` and `KEEP/fpms_phase6_LATEST.py` are **one file in three
places**. The eMMC adds no new dashboard version — confirmed again this pass.

Additional phase5/phase6 copies live under `~/fpms_RELEASE/`:
`GOLDEN_P5_OBSTACLE_AVOIDANCE_20260521_220836.py` (`_TRN=32, _COAST=0.87`),
`GOLDEN_P5_PERFECT_RETURN_20260522_223105.py` (`_TRN=36, _COAST=0.82`),
`phase6/fpms_phase6_birdeye_20260525_1146.py` (`_TRN=40, _COAST=0.93`). These are
distinct tunings, not byte duplicates — the turn/coast pair differs in each, so
they record real tuning history even though the code around them is near-identical.
Counting these, the phase5/phase6 family is **12 phase5 + 6 phase6 files, of
which 11 are genuinely distinct builds**.

Older lineage (all inside the May archives *and* loose on SD): `fpms_B6_*`,
`fpms_B7_*`, `fpms_B8_*`, `fpms_b3/b4/b6/b8_*` — ~62 files, superseded by phase5.
Newest of that family is `fpms_B8_SAFE_REBUILD.py` 145,011 B (2026-05-07), only
inside `FPMS_GOLDEN_AUTONOMOUS_DRIVING_B86_FINAL_*.tar.gz`.

### 7.2 Motion / mission drivers (live)

| mtime | Size | Path | Note |
|---|---|---|---|
| 2026-07-28 20:39 | 30,603 | `~/fpms-rover-agent.py` | camera + lidar → dashboard |
| 2026-07-31 17:01 | 22,736 | `~/fpms_lidar_ros.py` | MQTT lidar → `/scan_lidar` |
| 2026-08-02 22:35 | 113,608 | `~/fpms_odom_tf.py` | `/odom_raw`+`/imu` → `/odom`, `odom`→`base_footprint`. **Carries the 1320 CPR assumption** |
| 2026-08-04 16:16 | 142,093 | `~/fpms_teleop.py` | MQTT → `/cmd_vel`; `CMD_SCALE` at `:106` |
| 2026-08-04 19:03 | 38,992 | `~/fpms_ros_tunnel.py` | |
| 2026-08-04 19:07 | 49,299 | `~/fpms_rtos_follower.py` | |
| **2026-08-05 (live)** | ~315 K | **`~/fpms_missions.py`** | **being edited by another agent during this survey — not rescued** |
| — | — | `~/fpms_missions_staged.py` | near-twin of the above; identical constant values, line numbers shift ~+50 after line 800 |
| — | — | `~/test_odom_projection.py` | tests the arena projection; `:137` hard-codes `scale_ratio = 14.8 / 6.00` |

Backups `~/fpms_missions.py.bak.*` — 9 files, 200 K→302 K, Aug 3–4. Normal
save-point churn; the newest live file supersedes all of them.

`~/fpms_saved/` is a mirror of ~40 top-level `~` files taken 2026-08-04 14:23;
`~/KEEP/` is a 3-file curated subset. Neither contains anything unique.
`~/fpms_RELEASE/FPMS_SESSION_HANDOFF_20260506_021945.txt` (55 lines) contains
**no calibration numbers**, and `~/fpms_RELEASE/fpms_b6_source_used.txt` is
**0 bytes**.

### 7.3 Firmware sources

| mtime | Size | Path | Note |
|---|---|---|---|
| 2026-08-02 18:41 | 4,194,304 | `~/firmware_backup/yahboom_microros_v2_stock.bin` | **the irreplaceable factory image** |
| 2026-08-02 22:16 | 25,931 | **`~/fpms_firmware/README.md`** | **the key document** |
| 2026-08-03 12:30 | 4,194,304 | `~/stock_backup_preflash.bin` | second factory backup |
| 2026-08-03 13:14 | 6,120 | `~/fpms_config_deployed.h` | linorobot config, **1320 CPR** (superseded) |
| 2026-08-03 22:39 | 3,543 | `~/enctest/src/main.cpp` ≡ `~/fpms_saved/enc_main.cpp` | encoder polarity + mapping test |
| 2026-08-04 14:04 | 11,472 | `~/crawl/src/main.cpp` ≡ `~/KEEP/moves_main.cpp` | **the motion-constants file** |
| 2026-08-04 15:06 | 11,294 | **`~/lino/config/custom/fpms_config.h`** | **3255 CPR + measured polarity; uncommitted** |
| 2026-08-04 15:58 | 1,517,120 | `~/microROS_Robot_V2.0.0.bin` | Yahboom stock, currently flashed |
| 2026-08-05 12:10–12:19 | — | `~/nvs_backup/` | `nvs_ORIGINAL.bin`, `ybdata_ORIGINAL.bin`, `ybdata_WIFIUDP.bin`, `app.bin`, `parttable.bin` — today's NVS work |

### 7.4 Calibration / measurement scripts (**the constants live here**)

| mtime | Size | Path | What it measured |
|---|---|---|---|
| 2026-05-10 16:22 | 1,446 | `~/encoder_test.py` | early encoder probe |
| 2026-05-10 16:34 | 2,510 | `~/m1_encoder_test.py` | |
| 2026-07-31 14:01 | 7,039 | `~/sweep.py` | deadband sweep driver |
| 2026-07-31 16:20 | 39,865 | `~/deadband_sweep.py` | the sweep tool; `ODOM_TWIST_SIGN` at `:154` |
| 2026-08-03 22:39 | 3,543 | `~/fpms_saved/enc_main.cpp` | **per-wheel encoder sign + mapping** |
| 2026-08-04 08:42 | 4,721 | `~/fpms_saved/crawl_enc_main.cpp` | **per-wheel trims (20288/19988/18730/17677)** |
| 2026-08-04 08:48 | 4,858 | `~/fpms_saved/fwd200_main.cpp` | 27.7 counts/mm (**retracted**) |
| 2026-08-04 08:53 | 4,752 | `~/fpms_saved/fwd400_main.cpp` | 14.8 counts/mm |
| 2026-08-04 09:15 | 6,577 | **`~/fpms_saved/fwd800_main.cpp`** | **the 800 mm / 11,836 count tape measure** |
| 2026-08-04 14:04 | 11,472 | **`~/KEEP/moves_main.cpp`** | **coast, turn coast, reverse trims, slip** |
| 2026-08-04 16:05 | 3,790 | `~/factory_odom.py` | |
| 2026-08-05 12:15 | 4,666 | `~/nvs_backup/nvstool.py` | NVS reader (today) |

### 7.5 Configs

`~/nav2/`: `arena.yaml`, `arena.pgm` (19,615 B), `arena_zones.json`,
`arena_map.yaml`, `nav2_params.yaml` (42,395 B), `nav2_params_slam.yaml`,
`fpms_nav2.launch.py`, `fpms_tf.launch.py`, `make_arena_map.py`.
`~/slam/`: `fpms_slam_mapping.launch.py`, `fpms_slam_localization.launch.py`,
`mapper_params_*.yaml`.
`~/`: `arena_map.pgm`/`.yaml`, `fpms_markers.json`, `.fpms_teleop_origin.json`.
Systemd units: `fpms-lidar-ros`, `fpms-teleop`, `fpms-ros-tunnel`,
`fpms-rtos-follower`, `micro-ros-agent`, `fpms-uros-agent-run`.

### 7.6 Notes / docs

Only **two** prose documents exist on the rover:

| Size | Path | Value |
|---|---|---|
| 25,931 | **`~/fpms_firmware/README.md`** | **highest-value file on the rover** |
| 9,302 | `~/nav2/TF_TREE.md` | TF ownership, one-publisher-per-edge, `tf2_echo` verification |

Both reference a **`NAV2_BRIEF.md`** and a **`research/R4_FIRMWARE.md`** and a
`SESSION_HANDOFF.md` that **do not exist on the Pi** — they live in the cloud
repo. `R4_FIRMWARE.md` is cited as holding the full root-cause analysis and the
7.5–15 % MIN_PWM band; worth locating there.

### 7.7 Logs with data

Covered in §4.7. Also `~/build*.log` (5 files, PlatformIO builds Aug 3),
`~/flash.log`, `~/setup1.log`, `~/tf_launch.log`, `~/agent_v6.log` (16.4 MB).

---

## 8. Rescued to `rover/rescued/`

All verified by sha256 against the remote. Nothing pre-existing was overwritten.

| File | Size | sha256 |
|---|---|---|
| `README.md` (= `fpms_firmware/README.md`) | 25,931 | `fa1e98e62b37a8d94a59a071670a24d15a6b2749f7db62be1328717aa52c5835` |
| `TF_TREE.md` | 9,302 | `5125ecafbcbfd9a47bc45068c95e9ff78d9304e2a753f5d1cc4054cd4622e82f` |
| `moves_main.cpp` | 11,472 | `39eff479f3fc8a26cfbdaac0ebb65081147bd56ef39a0afaae11dbc765c8381b` |
| `fwd800_main.cpp` | 6,577 | `e8286adf79bb8f8fa317b5b1b0a6fe6473dc58970d326675ef5b41de26061a38` |
| `fwd400_main.cpp` | 4,752 | `75367ed06d84a0debbbc7347dd50d14501be542862ce490d74550ef4a1f2430c` |
| `fwd200_main.cpp` | 4,858 | `584c47a683aa67c42eb5daee1df1ede9e8b1be39c178b8e1d116b4a303d6a6b7` |
| `crawl_enc_main.cpp` | 4,721 | `24ce175c6d378bbb187d8153371f1a3b354189ae6c2b6075f9c156d037aa6afc` |
| `enc_main.cpp` | 3,543 | `9d2175fefc792651332afbacab5d5f2010133e4cae33280d3ad9ae3318124b85` |
| `fpms_config.h` (lino, 3255 CPR) | 11,294 | `d0b31e0cf38f354fb428599aa0f858f6a55398e0abb6a6de2ab0feae365c5667` |
| `fpms_config_deployed.h` (1320 CPR) | 6,120 | `fed451b44377de82ecc0459f7708c0475709bb82bdb45b00a88353dc02ad8f8c` |
| `fpms_odom_tf.py` | 113,608 | `fba7afbea60fccc36c3e8945bdb2c5824563a72e86cb7242b0e20e2173db0de6` |
| `teachin_server_8089_FINAL.py` | 16,460 | `259d21ed52c89b3a82b6414274329c919d69c9ad3bd9fc25513dac4d564641d9` |
| `deadband_sweep.py` | 39,865 | `084c2affbc4f6555448366d58c770b86bc887bcc2257205b6745a56c6439176a` |
| `sweep.py` | 7,039 | `f901593011858c33fc7a1daffa31d637aa07c6534118f1a70a11b2b3fed87115` |
| `deadband_sweep_state.json` | 3,153 | `fbd6fd0997251e665db23843de3f8c85d4121d6bdeb23c704188226fb1f34cde` |
| `sweep.log` | 2,168 | `7b69dccb76b13f6a57187f32af3199cc75ef445712c8d9fdc965d13be5f55fd4` |
| `routes.json` | 992 | `6518936b1007da99941104ae4e89f70155d2fd5267769f583243d4b0fe4b47ef` |
| `arena_zones.json` | 889 | `6a5b69129b809fbd3055c1cecb0f5a8bfcecf256cb48b877e40b6b3c51c8d8cf` |
| `arena.yaml` | 125 | `401d33e3a86bd3de1c7a3f81e5cb523f04a1a9de29b3cf13fe75436c5c091c65` |
| `fpms_markers.json` | 145 | `59075140e67e27c0b4f9fdfcefb7e961228495370e47375214649c732973b9d3` |
| `turnloop.log` | 458 | `cc19a9d319269c491e4cc6f3d24842b1ad43bec4d9dc4a0c5d53bf1da3944d6b` |
| `map_odom.log` | 324 | `0817ce6efbe9422c20b01d1b693faac48f004118f64a4499d2d34d6ee4394b12` |
| `rover_config.h` (4-way CPR table, all firmware defaults) | 9,789 | `223a6f904ac6551755ed41cc95e477e0695bb30e4eb105016473d097813a16f5` |
| `fpms_tf.launch.py` (LiDAR mount placeholders) | 23,904 | `357959c84175d7be64426a8488468d81793d2aa88c1ed40bedbfae9bf3fd5dba` |
| `fpms_lidar_ros.py` | 22,736 | `2f413fe378ee96582eb2bbfed4de42aca6989e065e4f60a2390277acba4e3153` |

Previously rescued and left untouched: `fpms_phase6_LATEST.py`,
`fpms_phase6_PERFECT_MISSION.py`, `fpms_dashboard.py`,
`fpms_dashboard_BEFORE_B1_AUTONOMY.py`, `fpms_hdmi_display.py`,
`fpms_yolo_npu.py`, `teachin_server_8089_WORKING_DIRECT_TURNS.py`.

---

## 9. What is worth using, ranked

1. **`rescued/README.md` — read it end to end before touching motion code.**
   It is a complete root-cause analysis of the exact failure the operator hit
   today, with the arithmetic, the measured numbers, and an explicit list of the
   three host-side constants that must change together. Nothing else on the rover
   comes close.

2. **Fix `MIN_MOVE_MM` in `fpms_missions.py` — this is the session-killer.**
   `:792` computes it from `CRUISE_MPS` (0.18 m/s), giving ~63 mm. The dead zone
   means a minimum burst actually travels `FULL_DUTY_MPS × MIN_PULSE_S` =
   0.65 × 0.35 = **227.5 mm**, and the README measured ~230 mm. The planner is
   therefore emitting segments it physically cannot execute, and treating a
   227 mm lurch as a 63 mm move. Use 230 mm until re-measured under load
   (`MIN_PULSE_MEASURED_UNDER_LOAD` is `False` at `:788`).

3. **Replace `_TKMM` with the measured constant, or stop using encoder distance.**
   `_TKMM = pi*70/1320` (0.1666 mm/tick) is in `fpms_phase6_LATEST.py:1309` and
   11 siblings, and `fpms_odom_tf.py:371` carries the same `TICKS_PER_REV = 1320`.
   The measured value is 14.8 counts/mm → **0.0676 mm/tick**, i.e. 3255 CPR. Any
   code path still using `_TKMM` over-reports distance by 2.47×, and that ratio
   is **exactly 74/30 — a 74:1 gearbox where 30:1 was assumed** (§4.1). This is a
   mechanical fact, not a fudge factor, so it applies to every count source.
   `fpms_missions.py:811–826` already documents it and deliberately does not
   apply it; applying it is the single highest-value code change available.

4. **Decide the firmware question deliberately.** The board is on Yahboom stock,
   so the 50 % dead zone — the cause of the 227 mm floor — is live. Two fixes
   already exist and neither is deployed: `~/fpms_firmware` (built and flashed
   once on Aug 2, ESP-IDF, runtime-tunable) and `~/lino` (linorobot2, carries
   3255 CPR and the measured polarity, uncommitted). Reflashing either changes
   `ODOM_TWIST_SIGN`, `TURN_WIRE_SIGN` and `CMD_SCALE` semantics **together**;
   `README.md:614–618` warns that changing them piecemeal makes the rover drive
   wrong in a new way.

5. **`rescued/moves_main.cpp` is a working open-loop motion recipe.** Kick-start
   (140 duty, 220 ms) then crawl (70), per-direction wheel trims, separate
   forward/reverse coast (1270/1448 counts), separate left/right turn coast
   (38°/27°), and a slip detector that catches a wheel falling off. If closed-loop
   control keeps running away, this is a known-good fallback that does not depend
   on the velocity PID at all.

6. **`~/lino/config/custom/fpms_config.h` is uncommitted and unbacked-up.**
   It holds the 3255 CPR derivation and the per-wheel polarity measurements with
   their raw encoder deltas. It exists only in a dirty working tree on an SD card
   that is 81 % full. It is now in `rescued/`, but it should be committed.

7. **Back up the eMMC's `ROSMASTERX3` documentation.** 530 MB of Yahboom PDFs,
   the only copy on the rover, and the authoritative source for the encoder
   derivation and GPIO map. It is on a stale clone that nothing else needs.

8. **Re-run the deadband sweep.** `sweep.log` shows it aborted after 6 steps with
   `MIN_CMD_LIN = UNKNOWN`; `deadband_sweep_state.json` shows odometry reporting
   exactly zero throughout. There is no measured deadband number in the project —
   only the theoretical 50.25 % floor. The sweep's own caveat stands: it is a
   no-load lower bound.

9. **Measure the track width, and reconcile the three values.**
   `LR_WHEELS_DISTANCE = 0.170 m` is unmeasured and `fpms_config.h:107–109` says
   so plainly; `fpms_rtos_follower.py:190` independently uses 0.105 m and calls
   it "the MEASURED 105 mm", but 105 mm is the *wheelbase*. One of these two
   nodes is computing yaw rate from the wrong dimension.

10. **Measure the LiDAR mount — it is all zeros right now.** Every offset in
    `nav2/fpms_tf.launch.py` is a `MEASURE ME` placeholder, and `fpms-tf.service`
    launches it with no arguments, so 0.0 is what is published. By the file's own
    arithmetic, 1° of unmodelled mount yaw is 10.5 mm of position error. Anything
    that fuses LiDAR with odometry is working from an unmeasured transform. Do
    the wall test that `mqtt.log` has been asking for, and set the offset in
    exactly one place (§5.10).

11. **Remember there is no runtime config.** `/etc/fpms/config.env` does not
    exist, and no systemd unit overrides a calibration constant, so every
    `FPMS_*` environment hook falls through to its in-source default (§4.7c).
    Changing a constant means editing the file. This also means the constants in
    this document are exactly what the rover is running.

### Dead ends — do not spend time here

- **The `.tar.gz` savepoints.** Fully opened and diffed; zero unique content.
- **The eMMC as a source of lost work.** Nothing unique except vendor docs.
- **`~/fpms_BACKUPS/*.tar.gz`** — 170 MB of duplicated virtualenv.
- **`turnloop.log`** — records commands only, no measurements, and the test
  itself destroys heading.
- **`routes_FINAL.json` / `routes_FINAL_WORKING.json`** — both empty.
- **`deadband_sweep_state.json.dryrun`** — simulated, not hardware.
- **The `fpms_B*/b*` lineage** (~62 files) — superseded by phase5/phase6.
- **AWS / gateway-era code** — confirmed absent again; nothing to find.
