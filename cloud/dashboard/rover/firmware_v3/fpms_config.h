#ifndef FPMS_CONFIG_H
#define FPMS_CONFIG_H

/* ===========================================================================
 * FPMS rover -- Yahboom MicroROS Board V2.0 (ESP32-S3), 4WD skid steer.
 * FIRMWARE V3 ("duty-first"). WRO Canada Nationals build.
 *
 * READ THIS FIRST IF YOU ARE NEW:
 * The board is a DUMB DUTY AMPLIFIER. The Pi decides how fast to go; the board
 * just applies the percentage it is told and reports what the encoders did.
 * The full topic list is at the top of fpms_main.cpp.
 *
 * WHY THIS EXISTS -- three defects this firmware is built to make impossible:
 *
 *  1. VENDOR DEAD ZONE. Yahboom's factory firmware ADDED PWM_MOTOR_DEAD_ZONE
 *     (200 of a 400-tick scale) as feed-forward on the velocity path, so any
 *     non-zero cmd_vel became ~50% duty ~= 0.65 m/s. Amplitude was discarded
 *     and the smallest possible move was ~227 mm. There is NO dead zone here
 *     and duty passes through untouched. Never add one.
 *
 *  2. VELOCITY-PID RUNAWAY (2026-08-04). With encoder polarity inverted
 *     relative to motor polarity, the PID became POSITIVE feedback: it ran to
 *     full duty on a command AND on a hand push, and only a hardware reset
 *     stopped it. It hit a wall. V3's answer is structural, not a better tune:
 *     the DEFAULT path has no feedback loop at all, so a polarity mistake can
 *     only flip a reported number -- it cannot drive the wheels.
 *
 *  3. BACKWARDS ODOMETRY. Factory firmware integrated position backwards
 *     (confirmed 5/5 against operator observation) and the Pi compensates with
 *     FPMS_MISSION_ODOM_POSE_SIGN=-1. This firmware integrates with the
 *     CORRECT sign, so that compensation must be set back to +1.
 * =========================================================================== */

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * BUILD IDENTITY. Published on /fpms_health so the operator can prove, from
 * the Pi, which firmware is actually running. Bump FPMS_FW_VERSION on every
 * flash. This exists because "is the new firmware actually on there?" has been
 * a real question in this project more than once.
 * ------------------------------------------------------------------------- */
#define FPMS_FW_VERSION   30001          /* 3.00.01 */
#define NODE_NAME         "fpms_drive"

/* ===========================================================================
 * SECTION 1 -- SAFETY CONSTANTS. Change these only with the operator present.
 * =========================================================================== */

/* DEADMAN. If no actuation command arrives within this window, ALL MOTORS GO
 * TO ZERO. Enforced inside applyMotors() -- the single function that is
 * allowed to touch the motor pins -- and applyMotors() is called from loop()
 * on EVERY iteration, not only from the micro-ROS timer. That matters: if the
 * agent dies the timer stops firing, and a deadman that lived only in the
 * timer callback would never fire either, leaving the last duty latched on the
 * pins forever. 300 ms is ~6 missed commands at B8B's 20 Hz control rate. */
#define FPMS_CMD_TIMEOUT_MS 300

/* Hard ceiling on commanded duty, in percent. The interface accepts -100..100
 * and clamps; this lets the operator cap the whole rover for a first run on
 * the floor without editing any Pi code. Leave at 100 for competition. */
#define FPMS_DUTY_LIMIT_PCT 100

/* The velocity/PID path is DISARMED at boot and re-DISARMED on every
 * reconnect. Set to 1 only if you want cmd_vel to be armed at power-on --
 * DO NOT do this for competition. See "/cmd_enable" in fpms_main.cpp. */
#define FPMS_VEL_ARMED_AT_BOOT 0

/* Compile the velocity/PID path at all. The operator asked for it to exist, so
 * it is IN, but disarmed. If you ever want the absolute guarantee that the
 * runaway path cannot be reached by any sequence of messages, set this to 0
 * and the code is not compiled -- a topic that does not exist cannot be
 * published to. */
#define FPMS_ENABLE_CMD_VEL 1

/* ===========================================================================
 * SECTION 2 -- ROS / LINK
 * =========================================================================== */

/* The whole FPMS stack runs on domain 20. The board must join it EXPLICITLY,
 * because in micro-ROS the CLIENT declares its domain in the CREATE_PARTICIPANT
 * request -- the agent's own ROS_DOMAIN_ID does not place it. Without this the
 * board publishes perfectly into domain 0, the agent logs a flawless session,
 * and every topic still shows "Publisher count: 0". Consumed by
 * rcl_init_options_set_domain_id() in createEntities(). */
#define FPMS_ROS_DOMAIN_ID 20

/* 921600 proved unstable on this board: the micro-ROS session died within 1-3
 * minutes of every reset, never established on a cold boot, and motion made it
 * worse. A CP2102 with four motor drivers switching nearby is marginal there,
 * and framing errors kill the XRCE session. MUST match the -b flag on the
 * micro-ros-agent unit. */
#define BAUDRATE 230400

/* PUBLISH RATES. These are a SERIAL BUDGET, not a preference.
 * 230400 baud ~= 23 kB/s. A nav_msgs/Odometry is ~700 B once serialised
 * (two 36-element covariance matrices = 576 B of it), sensor_msgs/Imu ~350 B.
 * Naively publishing both at the 50 Hz control rate is ~52 kB/s -- more than
 * twice the link -- and an overrun does not degrade gracefully: it corrupts
 * framing and kills the XRCE session. Measured factory rates were 11 Hz odom /
 * 25 Hz imu, which is the same ballpark as these.
 *   odom 10 Hz (7.0 kB/s) + imu 20 Hz (7.0 kB/s) + ticks 25 Hz (~0.8 kB/s)
 *   + duty 25 Hz (~0.8 kB/s) + health 2 Hz + battery 1 Hz  ~= 16 kB/s (~70%).
 * If you add a publisher, subtract the bandwidth from something else. */
#define FPMS_CONTROL_HZ    50    /* motor update + deadman evaluation         */
#define FPMS_TICKS_HZ      25    /* raw encoder counts -- B8B's measurement   */
#define FPMS_DUTY_ECHO_HZ  25    /* duty actually applied (deadman visible)   */
#define FPMS_ODOM_HZ       10
#define FPMS_IMU_HZ        20
#define FPMS_HEALTH_HZ      2
#define FPMS_BATTERY_HZ     1

/* ===========================================================================
 * SECTION 3 -- CHASSIS / DRIVETRAIN
 * =========================================================================== */

#define LINO_BASE SKID_STEER                /* 4WD */

/* The Yahboom drives each motor from TWO complementary PWM pins (M1A/M1B =
 * 4/5 and so on) rather than the PWM-pin-plus-direction-pins topology most
 * stock drivers assume. That is exactly what upstream's BTS7960 driver does
 * (upstream documents it as also covering A4950/DRV8833 modules), so no custom
 * motor class is needed. It requires MOTORn_PWM == -1. */
#define USE_BTS7960_MOTOR_DRIVER

/* USE_SHORT_BRAKE deliberately NOT defined. spin(0) therefore lands in
 * brake(), which on this topology writes BOTH pins to zero = COAST. The golden
 * phase6 driver stopped by coasting and every constant it tuned -- above all
 * the 0.93 turn-coast factor -- is calibrated against coast. Active braking
 * would invalidate all of them. */

/* MOTOR_MAX_RPM is an ESTIMATE and only matters on the (disarmed) velocity
 * path -- the duty path never consults it. 180 RPM on a 70 mm wheel is
 * ~0.66 m/s, consistent with the ~0.65 m/s the vendor firmware produced at
 * full duty. */
#define MOTOR_MAX_RPM 180
#define MAX_RPM_RATIO 0.85
#define MOTOR_OPERATING_VOLTAGE 12
#define MOTOR_POWER_MAX_VOLTAGE 12
#define MOTOR_POWER_MEASURED_VOLTAGE 11.6   /* measured 2026-08-03; pack low  */

#define K_P 0.6
#define K_I 0.8
#define K_D 0.1

/* 20 kHz stays above audible; 10 bits gives 1024 duty steps, so 1% of duty is
 * ~10 steps and a crawl setpoint has real resolution to land on. */
#define PWM_BITS 10
#define PWM_FREQUENCY 20000

/* ==========================================================================
 * FPMS_COUNTS_PER_REV -- THE ONE NUMBER TO CHANGE.
 * Encoder counts per WHEEL revolution at the gearbox OUTPUT shaft.
 *
 * >>> SUPERSEDED 2026-08-06. THE REST OF THIS COMMENT BLOCK IS HISTORY. <<<
 * It argues for 3255 counts/rev and a 70 mm wheel. BOTH WERE MEASURED WRONG.
 * The live values and the measurement that produced them are immediately
 * above the #define at the end of this block. Read that, not this.
 * (Kept because the reasoning explains HOW the wrong number was arrived at,
 * and because deleting the trail is how it gets re-derived next time.)
 *
 * *** THIS CONSTANT WAS CONTESTED. IT HAS NOW BEEN MEASURED. ***
 *
 * Current value comes from MEASUREMENT, not derivation: fwd800_main.cpp
 * recorded a tape-measured run in which the rover really travelled 800 mm
 * while the encoders reported 11836 counts = 14.8 counts/mm.
 *     14.8 counts/mm * pi * 70 mm = 3255 counts/rev
 * Competing numbers on record: 1320 (spec product 11 lines x 30:1 x4 -- a
 * DERIVATION), and a hand-push test that read 0.743x actual. They disagree by
 * more than a factor of two, so do not trust any of them without your own tape
 * measurement through THIS firmware.
 *
 * DECODING MODE -- DECIDED ONCE, HERE:
 *   upstream firmware/lib/encoder/encoder.h calls ESP32Encoder::attachHalfQuad()
 *   on ESP32. That is **x2 decoding** (both edges of ONE channel), NOT the x4
 *   that every spec-sheet calculation assumes. THIS FIRMWARE KEEPS HALF-QUAD --
 *   it is what the 14.8 counts/mm measurement was taken through, so keeping it
 *   is the only way that measurement stays valid.
 *   If anyone ever switches to attachFullQuad(), this constant must DOUBLE to
 *   6510 IN THE SAME COMMIT. Leaving it would make the rover run 2x too fast
 *   and report half the distance. There is no need to switch: the ESP32-S3 has
 *   4 PCNT units (soc_caps.h SOC_PCNT_UNITS_PER_GROUP 4 -- the ESP32Encoder
 *   README claiming 2 is wrong), so all four wheels already count in hardware.
 *
 * COUPLED TO WHEEL_DIAMETER -- EDIT THEM TOGETHER:
 *   odometry is  distance = ticks * pi * D / CPR.
 *   What was measured is 14.8 counts per MILLIMETRE; the per-rev figure is that
 *   times pi times D. Yahboom's ROS chassis line standardises on 65 mm wheels,
 *   not the 70 mm assumed here. If the wheel measures 65 mm, BOTH must change:
 *   CPR becomes 14.8 * pi * 65 = 3022. Changing WHEEL_DIAMETER alone silently
 *   rescales every distance by D_new/D_old.
 *
 * YOU DO NOT NEED TO REFLASH TO FIX THIS. /wheel_ticks publishes RAW counts,
 * so the Pi can re-derive counts/mm from a tape-measured push at any time.
 * That is the entire reason raw ticks are on the wire.
 *
 * ZERO-BATTERY CHECK (costs no charge, do it first): power the board, leave
 * motors idle, PUSH the rover along a tape measure by hand, and watch
 * /wheel_ticks and /odom_raw x. The distance should match the tape, and the
 * SIGN must be POSITIVE when pushed forward.
 * ========================================================================== */
/* *** MEASURED 2026-08-06 -- 3255 WAS 2.7x TOO HIGH. ***
 *
 * Tape-measured through THIS firmware's /wheel_ticks, by HAND PUSH with the
 * motors off, which is the measurement this block asks for above. Passive
 * wheels are ground-coupled and cannot slip, so there is no slip term:
 *
 *     hand push FORWARD  1000 mm -> 5.68 counts/mm  (wheels 2-4)
 *     hand push BACKWARD 1000 mm -> 5.47 counts/mm  (all four, after a hub refit)
 *     powered run        1346 mm -> 6.56 counts/mm  (inflated by tyre slip)
 *
 * The two hand pushes agree within 4 %. counts/mm = 5.5. Wheel diameter was
 * measured across the tyre at 65 mm (see WHEEL_DIAMETER below), so:
 *
 *     5.5 counts/mm * pi * 65 mm = 1123  ->  1120
 *
 * The 14.8 counts/mm figure was real, but ONLY for the bare-metal
 * ESP32Encoder build it was taken on. PI_FILE_INVENTORY 4.1 flagged exactly
 * this risk and asked for a tape measure through the ROS count source before
 * relying on it. That measurement has now been done and 14.8 does not carry
 * across. The 74:1 gearbox theory invented to justify 3255 is also wrong.
 *
 * This is the 2.3 m overshoot: at 3255, a commanded 1000 mm waits for 14800
 * counts, which at 5.5 counts/mm is 2690 mm of real travel. It equally
 * explains the 744 mm plan that drove ~2 m and the 300 mm move that went ~1 m.
 * One constant, not three bugs.
 *
 * CAVEAT: every push was taken on a chassis whose front-left hub was working
 * loose (that wheel later came off). The 2.7x is robust; the third digit is
 * not. Re-measure on a sound chassis and refine. */
#define FPMS_COUNTS_PER_REV 1120

#define COUNTS_PER_REV1 FPMS_COUNTS_PER_REV
#define COUNTS_PER_REV2 FPMS_COUNTS_PER_REV
#define COUNTS_PER_REV3 FPMS_COUNTS_PER_REV
#define COUNTS_PER_REV4 FPMS_COUNTS_PER_REV

/* MEASURED 2026-08-06: the operator measured 65 mm across the tyre, not the
 * 70 mm previously assumed. The block above warns that this is COUPLED to
 * FPMS_COUNTS_PER_REV -- both were changed in the same edit, from the same
 * measurement. Do not change one alone. */
#define WHEEL_DIAMETER 0.065
/* Track = LEFT-to-RIGHT wheel separation. The operator measured 105 mm
 * FRONT-to-BACK, which is the WHEELBASE, not this. Track is still UNMEASURED.
 * It only affects the velocity path and reported angular_z -- the duty path
 * and /wheel_ticks are unaffected -- but measure it before trusting turn
 * geometry from odometry. */
#define LR_WHEELS_DISTANCE 0.170

/*
ROBOT ORIENTATION            firmware index -> board silkscreen
         FRONT               MOTOR1 = board M3 (front-left)
    MOTOR1  MOTOR2           MOTOR2 = board M1 (front-right)
    MOTOR3  MOTOR4           MOTOR3 = board M4 (rear-left)
         BACK                MOTOR4 = board M2 (rear-right)
*/

/* MOTOR DIRECTION -- MEASURED, operator-observed 2026-08-04, open loop.
 * Driving all four on IN_A at equal duty: LEFT ran forward, RIGHT ran BACKWARD.
 * No encoders and no PID were involved -- this is a pure wiring fact. The
 * right-side motors are wired with opposite polarity, so they must be inverted
 * or a "forward" command spins the rover on the spot.
 * Applied inside MotorInterface::spin(), so the duty path gets it for free. */
/* *** ALL FOUR FLIPPED 2026-08-06 -- MEASURED ON THIS FIRMWARE. ***
 *
 * Commanding +60 % on all four via /cmd_duty drove EVERY wheel BACKWARD:
 *     ticks [-47952, -58868, -28919, -50845] over 15 s
 * and the operator confirmed by eye: "All backwards!". Verified in the other
 * direction too -- -60 % on all four drove the rover forward, 53 in of travel.
 *
 * This was ONE global sign error, not four wiring faults. The old right-pair
 * inversions were doing their job: the drivetrain was coherent, all four
 * turning together. It was simply that "together" was backward.
 *
 * The 2026-08-04 note below ("LEFT ran forward, RIGHT ran BACKWARD") was taken
 * on the FACTORY firmware through different driver code. Like the 14.8
 * counts/mm figure, it did not carry across to this build.
 *
 * FLIPPED HERE AND NOT ON THE ENCODERS, DELIBERATELY. Either flip satisfies the
 * velocity-PID sign rule, but flipping MOTORn_ENCODER_INV instead would leave
 * positive duty driving backward while odometry integrated it as forward
 * travel -- the self-consistent lie that has already cost this project two
 * "successful" Mission-2 runs. MOTORn_ENCODER_INV is measured correct (a hand
 * spin forward gives POSITIVE ticks on all four) and is left alone.
 *
 * Until this is flashed, FORWARD IS NEGATIVE DUTY on the Pi side. */
#define MOTOR1_INV true    /* front-left  (board M3) -- was false, measured */
#define MOTOR2_INV false   /* front-right (board M1) -- was true            */
#define MOTOR3_INV true    /* rear-left   (board M4) -- was false           */
#define MOTOR4_INV false   /* rear-right  (board M2) -- was true            */

/* ENCODER DIRECTION -- MEASURED 2026-08-03 (tools/enc_main.cpp drives one
 * motor at a time and reports all four deltas), then RE-INTERPRETED 2026-08-04
 * once the wiring fact above was known.
 *
 *   motor   own encoder driven on IN_A
 *   M1      -11802 / -12158      M2  -16356 / -17199
 *   M3      -12835 / -11153      M4  -10850
 * All four count NEGATIVE when driven on IN_A. Pin mapping is CORRECT (motor N
 * moves encoder N; small cross-counts are chassis shake).
 *
 * The re-interpretation, which is the whole point:
 *   LEFT  (M1/M3): IN_A really IS forward, and the encoder counted negative
 *                  -> genuinely inverted            -> INV = true
 *   RIGHT (M2/M4): IN_A is physically BACKWARD, so counting negative was
 *                  already CORRECT                  -> INV = false
 *
 * *** NOTE FOR THE REVIEWER ***
 * This deliberately does NOT "match MOTORn_INV". A note in circulation claims
 * the corrected polarity should be false/true/false/true, mirroring
 * MOTORn_INV. Worked through, that is the RUNAWAY polarity, not the fix:
 *   M2 has MOTOR2_INV=true, so spin(+) -> reverse() -> IN_B -> physically
 *   FORWARD -> raw encoder counts POSITIVE. Applying ENCODER_INV=true there
 *   would report NEGATIVE rpm for a positive command == positive feedback.
 * The values below are the ones consistent with the measurements above.
 *
 * AND IT DOES NOT MATTER MUCH ANY MORE, WHICH IS THE POINT OF V3:
 * on the default duty path these four booleans are MEASUREMENT-ONLY. Getting
 * them wrong flips the sign of a published number. It cannot move a wheel.
 * Verify them with the on-blocks test in the report before arming cmd_vel. */
#define MOTOR1_ENCODER_INV true    /* front-left  (board M3/H3) */
#define MOTOR2_ENCODER_INV false   /* front-right (board M1/H1) */
#define MOTOR3_ENCODER_INV true    /* rear-left   (board M4/H4) */
#define MOTOR4_ENCODER_INV false   /* rear-right  (board M2/H2) */

/* ===========================================================================
 * SECTION 4 -- PIN MAP (Yahboom MicroROS Board V2.0, corroborated 3x against
 * Yahboom's published pinout).
 *
 * *** DO NOT DELETE THIS CONFIG AND FALL BACK ON linorobot's stock
 * *** esp32s3_config.h. That file assigns MOTOR3 to GPIO 39/40, which on THIS
 * *** board are the IMU's I2C lines. The #error guards at the bottom of this
 * *** file exist to catch exactly that mistake at compile time.
 * =========================================================================== */

#define MOTOR1_ENCODER_A 11   /* front-left  = board E3/H3 */
#define MOTOR1_ENCODER_B 12
#define MOTOR2_ENCODER_A 6    /* front-right = board E1/H1 */
#define MOTOR2_ENCODER_B 7
#define MOTOR3_ENCODER_A 1    /* rear-left   = board E4/H4 -- GPIO1 is an
                               * ENCODER, not the battery sense. Getting this
                               * wrong is a documented past mistake. */
#define MOTOR3_ENCODER_B 2
#define MOTOR4_ENCODER_A 47   /* rear-right  = board E2/H2 */
#define MOTOR4_ENCODER_B 48

#ifdef USE_BTS7960_MOTOR_DRIVER
  #define MOTOR1_PWM -1       /* no separate enable pin on this topology */
  #define MOTOR1_IN_A 9       /* front-left  = board M3 */
  #define MOTOR1_IN_B 10

  #define MOTOR2_PWM -1
  #define MOTOR2_IN_A 4       /* front-right = board M1 */
  #define MOTOR2_IN_B 5

  #define MOTOR3_PWM -1
  #define MOTOR3_IN_A 13      /* rear-left   = board M4 */
  #define MOTOR3_IN_B 14

  #define MOTOR4_PWM -1
  #define MOTOR4_IN_A 15      /* rear-right  = board M2 */
  #define MOTOR4_IN_B 16

  /* PARENTHESISED, unlike the stock template. Upstream writes this as
   * `pow(2, PWM_BITS) - 1` with no brackets, so `-PWM_MAX` expands to
   * `-pow(2,PWM_BITS) - 1` = -1025 rather than -1023, quietly making the PID's
   * negative clamp asymmetric. Small, but it is free to fix and it is exactly
   * the kind of thing nobody finds while debugging something else. */
  #define PWM_MAX (pow(2, PWM_BITS) - 1)
  #define PWM_MIN (-PWM_MAX)
#endif

#define LED_PIN 45            /* MCU indicator LED */

/* ICM42670P on I2C0. These two values are MEASURED-CONFIRMED and they are also
 * the reason the stock esp32s3 pin map is dangerous here. */
#define SDA_PIN 40
#define SCL_PIN 39

/* Servo headers S1/S2 per Yahboom's published pinout. Driven as 50 Hz / 16-bit
 * LEDC through the same setupPwm()/setPwm() helpers the motors use, so channel
 * allocation stays with one owner and cannot collide. */
#define FPMS_SERVO1_PIN 8
#define FPMS_SERVO2_PIN 21

/* BUZZER -- INTENTIONALLY NOT DEFINED.
 * The operator asked for /beep, and the code for it is written and ready in
 * fpms_main.cpp. It is gated off because Yahboom's pinout material we have
 * does NOT give a buzzer GPIO, and guessing one on this board means possibly
 * driving an encoder or I2C line as an output. That is a real way to break
 * odometry or the IMU. Confirm the pin from the board silkscreen or Yahboom's
 * schematic, then uncomment and set it -- one line, no other change needed.
 *   #define FPMS_BUZZER_PIN <confirmed gpio>
 */

/* Battery sense: GPIO 3, NOT the template's GPIO 1 (which is rear-left encoder
 * A on this board). The divider ratio is UNCONFIRMED, so this formula is
 * PROVISIONAL: the pack read 11.6 V on vendor firmware, which is the reference
 * point to calibrate against. Fix it by comparing /battery to a multimeter. */
#define BATTERY_PIN 3
#define BATTERY_ADJUST(v) ((v) * (3.3 / 4096 * (33 + 10) / 10))

#define ACCEL_COV { 0.01, 0.01, 0.01 }
#define GYRO_COV  { 0.001, 0.001, 0.001 }
#define ORI_COV   { 0.01, 0.01, 0.01 }
#define MAG_COV   { 1e-12, 1e-12, 1e-12 }
#define POSE_COV  { 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 }
#define TWIST_COV { 0.001, 0.001, 0.001, 0.003, 0.003, 0.003 }

/* ===========================================================================
 * SECTION 5 -- IMU (ICM42670P)
 * =========================================================================== */

/* The gyro is LOAD-BEARING. /odom_raw carries an identity quaternion (there is
 * no wheel-derived heading to fall back on), and the golden driver's +/-1-4 deg
 * turn accuracy came entirely from integrating this sensor's Z rate.
 *
 * The driver lives in firmware/lib/imu/icm42670_imu.h. Its one non-obvious
 * correctness point, and the bug that made a real 180 deg turn integrate to
 * 0.0 deg: on this part GYRO_DATA_X1 is 0x11 and is NOT contiguous with the
 * accel block at 0x0B. Reading gyro as bytes 6-11 of a 12-byte accel burst
 * silently returns ZEROS. The driver reads gyro from its own address.
 *
 * It also probes WHO_AM_I (0x75, expect 0x67) on BOTH strap addresses and
 * orders config-before-enable per the datasheet.
 *
 * WHY THIS FIRMWARE DOES NOT USE THE TDK ARDUINO LIBRARY: it was considered.
 * The known defect (register non-contiguity) is already fixed here, this
 * driver is written against this exact board, and it needs no network fetch at
 * build time. Adding an unpinned external dependency that CANNOT be
 * compile-tested before Nationals trades a fixed bug for an unknown one. If
 * the diagnostic below shows the gyro still dead after flashing, swapping to
 * tdk-invn-oss/motion.arduino.ICM42670P is the documented fallback.
 *
 * VERIFY BY CONTROL CHANNEL, NOT BY ASSUMPTION. FPMS_IMU_DIAG exports the raw
 * registers through imu_msg.orientation (which this firmware never fuses and
 * otherwise leaves at identity), because the IMU is on the ESP32's I2C bus so
 * Linux cannot probe it, and Serial is the micro-ROS transport so nothing may
 * print to it. After flashing, `ros2 topic echo /imu --field orientation`:
 *     w != 0    WHO_AM_I 0x67 | addr<<8 | readok<<16  -> 26727 or 26983
 *     z != 0    PWR_MGMT0 | GYRO_CONFIG0<<8 | ACCEL_CONFIG0<<16
 *     x         raw gyro Z -- MUST swing while you turn the chassis by hand
 * If w stays 0 the probe failed; if x stays 0 the gyro data path is still dead.
 *
 * IMU_TWEAK must be defined HERE and not in the driver header: it is consumed
 * inside imu_interface.h's getData(), and imu.h includes default_imu.h (which
 * pulls in imu_interface.h) BEFORE icm42670_imu.h. Defining it in the driver
 * means it does not exist yet when getData() is compiled, the #ifdef silently
 * expands to nothing, and the diagnostic reports message defaults instead of
 * register contents -- which happened once and nearly produced a wrong
 * conclusion. imu_interface.h includes config.h at its top, so this is early
 * enough. */
#define USE_ICM42670_IMU
#define FPMS_IMU_DIAG 1

extern volatile int32_t fpms_diag_gyro_z;   /* raw signed gyro Z register      */
extern volatile int32_t fpms_diag_accel_z;  /* raw accel Z: proves reads work  */
extern volatile int32_t fpms_diag_cfg;      /* pwr | gyro_cfg<<8 | accel_cfg<<16 */
extern volatile int32_t fpms_diag_who;      /* WHO_AM_I | addr<<8 | readok<<16 */

#define IMU_TWEAK { \
    imu_msg_.orientation.x = (double)fpms_diag_gyro_z; \
    imu_msg_.orientation.y = (double)fpms_diag_accel_z; \
    imu_msg_.orientation.z = (double)fpms_diag_cfg; \
    imu_msg_.orientation.w = (double)fpms_diag_who; \
}

/* Brought up before the IMU driver touches the bus. */
#define BOARD_INIT { \
    Wire.begin(SDA_PIN, SCL_PIN); \
    Wire.setClock(400000); \
}

/* createEntities() returns false instead of hanging, so the autoconnect state
 * machine can retry cleanly rather than wedging in an error loop. */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
    flashLED(3); \
    return false; }}

/* ===========================================================================
 * SECTION 6 -- COMPILE-TIME PIN CONFLICT GUARDS.
 *
 * These are cheap and they catch the single most damaging class of mistake on
 * this board: a stock/example config assigning a motor or servo to the I2C
 * lines (GPIO 39/40) or to an encoder channel. linorobot's stock
 * esp32s3_config.h really does put MOTOR3 on 39/40. If someone edits the pin
 * map above by hand, the build should fail loudly rather than the IMU going
 * quiet at Nationals.
 * =========================================================================== */

#if (MOTOR1_IN_A == SDA_PIN) || (MOTOR1_IN_B == SDA_PIN) || \
    (MOTOR2_IN_A == SDA_PIN) || (MOTOR2_IN_B == SDA_PIN) || \
    (MOTOR3_IN_A == SDA_PIN) || (MOTOR3_IN_B == SDA_PIN) || \
    (MOTOR4_IN_A == SDA_PIN) || (MOTOR4_IN_B == SDA_PIN)
  #error "FPMS: a motor pin collides with SDA_PIN (40). The IMU will die. Check SECTION 4."
#endif

#if (MOTOR1_IN_A == SCL_PIN) || (MOTOR1_IN_B == SCL_PIN) || \
    (MOTOR2_IN_A == SCL_PIN) || (MOTOR2_IN_B == SCL_PIN) || \
    (MOTOR3_IN_A == SCL_PIN) || (MOTOR3_IN_B == SCL_PIN) || \
    (MOTOR4_IN_A == SCL_PIN) || (MOTOR4_IN_B == SCL_PIN)
  #error "FPMS: a motor pin collides with SCL_PIN (39). The IMU will die. Check SECTION 4."
#endif

#if (BATTERY_PIN == MOTOR3_ENCODER_A)
  #error "FPMS: BATTERY_PIN is on rear-left encoder A. It is GPIO 3, not GPIO 1."
#endif

#if (FPMS_SERVO1_PIN == SDA_PIN) || (FPMS_SERVO1_PIN == SCL_PIN) || \
    (FPMS_SERVO2_PIN == SDA_PIN) || (FPMS_SERVO2_PIN == SCL_PIN)
  #error "FPMS: a servo pin collides with the I2C bus. Check SECTION 4."
#endif

#endif /* FPMS_CONFIG_H */
