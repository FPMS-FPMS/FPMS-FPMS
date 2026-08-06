#ifndef FPMS_CONFIG_H
#define FPMS_CONFIG_H

#include <stdint.h>

/* ---- IMU diagnostic ------------------------------------------------------
 * These MUST live here rather than in icm42670_imu.h. IMU_TWEAK is consumed
 * inside imu_interface.h's getData(), and imu.h includes default_imu.h (which
 * pulls in imu_interface.h) BEFORE it includes icm42670_imu.h. Defining the
 * macro in the driver header means it does not exist yet when getData() is
 * compiled, the #ifdef silently expands to nothing, and the diagnostic reports
 * message defaults instead of register contents -- which is exactly what
 * happened on the first attempt and nearly produced a wrong conclusion.
 * imu_interface.h includes config.h at its top, so this is early enough.
 *
 * Carried out through imu_msg.orientation, which this firmware never fuses and
 * otherwise leaves at identity. The IMU is on the ESP32's I2C bus so it cannot
 * be probed from Linux, and Serial is the micro-ROS transport so nothing may
 * print to it. */
#define FPMS_IMU_DIAG 1
extern volatile int32_t fpms_diag_gyro_z;   /* raw signed gyro Z register     */
extern volatile int32_t fpms_diag_accel_z;  /* raw accel Z: proves reads work */
extern volatile int32_t fpms_diag_cfg;      /* pwr | gyro_cfg<<8 | accel_cfg<<16 */
extern volatile int32_t fpms_diag_who;      /* WHO_AM_I | addr<<8 | readok<<16 */

#define IMU_TWEAK { \
    imu_msg_.orientation.x = (double)fpms_diag_gyro_z; \
    imu_msg_.orientation.y = (double)fpms_diag_accel_z; \
    imu_msg_.orientation.z = (double)fpms_diag_cfg; \
    imu_msg_.orientation.w = (double)fpms_diag_who; \
}

/* FPMS rover -- Yahboom MicroROS Board V2.0 (ESP32-S3), 4WD skid steer.
 *
 * Replaces the vendor firmware, which had two defects that made centimetre
 * accuracy physically unreachable:
 *   1. PWM_MOTOR_DEAD_ZONE (200 of a 400-tick scale) was ADDED as feed-forward
 *      to every non-zero speed, so 50.25%% duty was the smallest output that
 *      existed and ~230 mm the smallest possible move.
 *   2. Its velocity loop regulated INTEGER encoder counts per 10 ms, so any
 *      setpoint below ~1 count/period was unrepresentable and the PID dithered.
 * Neither exists here: duty passes through untouched and the PID regulates
 * float RPM.
 */

#define LED_PIN 45                          // board_pins.h:89 MCU indicator LED

#define LINO_BASE SKID_STEER                // 4WD

/* The Yahboom drives each motor from TWO complementary PWM pins (M1A/M1B =
 * 4/5 and so on). That is exactly what the BTS7960 driver does -- upstream
 * documents it as also covering A4950/DRV8833 modules -- so no custom motor
 * class is needed. It requires MOTORn_PWM == -1. */
#define USE_BTS7960_MOTOR_DRIVER

/* ICM42670P gyro -- ENABLED 2026-08-05.
 *
 * The driver (firmware/lib/imu/icm42670_imu.h) is finished and correct: it
 * probes WHO_AM_I on both strap addresses, orders config-before-enable per the
 * datasheet, and reads GYRO_DATA_X1 (0x11) from its OWN address instead of
 * assuming it is contiguous with the accel burst -- that assumption was what
 * made a real 180 deg turn integrate to 0.0 deg.
 *
 * It was nonetheless NOT being compiled in. imu.h selects the class with
 *     #ifdef USE_ICM42670_IMU
 *         #define IMU ICM42670IMU
 *     #endif
 * and nothing in the tree ever defined USE_ICM42670_IMU, so imu.h fell through
 * to its `#ifndef IMU -> FakeIMU` default and the build silently shipped a stub
 * that publishes zeros. Defining it here is safe: firmware.cpp includes
 * config.h (line 31) BEFORE imu.h (line 38), so the macro exists by the time
 * that #ifdef is evaluated.
 *
 * This gyro is load-bearing -- /odom_raw carries an identity quaternion, so
 * there is no wheel-derived heading to fall back on, and the golden driver's
 * +/-1-4 deg turn accuracy came entirely from integrating this sensor's Z rate.
 *
 * VERIFY BY CONTROL CHANNEL, NOT BY ASSUMPTION: FPMS_IMU_DIAG exports the raw
 * registers through imu_msg.orientation (which this firmware never fuses).
 * After flashing, `ros2 topic echo /imu --field orientation` must show
 *     w = 103   (WHO_AM_I 0x67, plus addr<<8 and readok<<16 -> 26727 or 26983)
 *     z != 0    (PWR_MGMT0 | GYRO_CONFIG0<<8 | ACCEL_CONFIG0<<16)
 *     x         raw gyro Z, must swing while the chassis is turned by hand
 * If w stays 0 the probe failed; if x stays 0 the gyro data path is still dead.
 */
#define USE_ICM42670_IMU

#define K_P 0.6
#define K_I 0.8
#define K_D 0.1

#define ACCEL_COV { 0.01, 0.01, 0.01 }
#define GYRO_COV { 0.001, 0.001, 0.001 }
#define ORI_COV { 0.01, 0.01, 0.01 }
#define MAG_COV { 1e-12, 1e-12, 1e-12 }
#define POSE_COV { 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 }
#define TWIST_COV { 0.001, 0.001, 0.001, 0.003, 0.003, 0.003 }

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2
    MOTOR3  MOTOR4
         BACK
*/

/* MOTOR_MAX_RPM is a first estimate and MUST be calibrated: linorobot derives
 * max linear speed from it, so a wrong value mis-scales every cmd_vel. 180 RPM
 * on a 70 mm wheel is ~0.66 m/s, consistent with the ~0.65 m/s the vendor
 * firmware produced at full duty. */
#define MOTOR_MAX_RPM 180
#define MAX_RPM_RATIO 0.85
#define MOTOR_OPERATING_VOLTAGE 12
#define MOTOR_POWER_MAX_VOLTAGE 12
#define MOTOR_POWER_MEASURED_VOLTAGE 11.6   // measured 2026-08-03; pack was low

/* ==========================================================================
 * FPMS_COUNTS_PER_REV -- THE ONE NUMBER TO CHANGE. Encoder counts per WHEEL
 * revolution (4x-decoded quadrature, at the gearbox OUTPUT shaft).
 *
 * Load-bearing TWICE over, which is why it gets its own name:
 *   - odometry divides raw ticks by it, so it sets REPORTED DISTANCE; and
 *   - the velocity PID divides by it to compute measured RPM, so it sets ACTUAL
 *     SPEED. Set it N times too large and measured RPM reads N times too low,
 *     the PID saturates chasing a setpoint it believes it never reaches, and the
 *     rover runs N times too FAST. A wrong value here does not merely mislabel
 *     the crawl -- it destroys it.
 *
 * SETTLED 2026-08-05 at 3255, on MEASUREMENT rather than on derivation:
 *   fpms_saved/fwd800_main.cpp:34 records a tape-measured run -- the rover
 *   really travelled 800mm while the encoders reported 11836 counts, i.e.
 *   14.8 counts/mm. 14.8 * pi * 70mm = 3255.
 *
 * That run was taken THROUGH THIS FIRMWARE'S OWN ENCODER PATH, which is what
 * makes it authoritative and what makes every theoretical derivation suspect.
 * The decisive detail, from linorobot's firmware/lib/encoder/encoder.h: on
 * ESP32 it calls ESP32Encoder::attachHalfQuad(), which is x2 decoding -- both
 * edges of ONE channel -- NOT the x4 that every spec-sheet calculation assumes.
 * A whole factor of 2 lives in the decoding mode, before gearbox or line count
 * is even discussed. Yahboom's own doc gives the MD520 as an 11-line magnetic
 * encoder in 1:19 / 1:30 / 1:56 variants, and phase6's 1320 is exactly the
 * 11 x 30 x 4 spec product -- computed for a decoding mode this firmware does
 * not use, on a gearbox variant nobody confirmed. It was never measured here.
 *
 * So: trust the tape, not the datasheet. Do not revert this to 1320 on the
 * strength of phase6 having used that number.
 *
 * TWO COUPLINGS THAT WILL SILENTLY BREAK THIS CONSTANT -- read before editing:
 *
 *  (a) WHEEL_DIAMETER. What was measured is 14.8 counts per MILLIMETRE; the
 *      per-rev figure is that times pi times the wheel diameter. Odometry is
 *      self-consistent for any diameter ONLY while the pair agrees, because
 *      distance = ticks * pi * D / CPR. Yahboom's ROS chassis line standardises
 *      on 65mm wheels, not the 70mm assumed here. If the wheel measures 65mm,
 *      BOTH must change together: CPR becomes 14.8 * pi * 65 = 3022. Changing
 *      WHEEL_DIAMETER alone silently rescales every distance by D_new/D_old.
 *
 *  (b) DECODING MODE. If anyone ever switches encoder.h to attachFullQuad()
 *      (x4), this constant must DOUBLE to 6510 in the same commit. Leaving it
 *      would make the rover run 2x too fast and report half the distance.
 *
 * ZERO-BATTERY CONFIRMATION (do this first, it costs no charge):
 *   Power the board, leave the motors idle, and PUSH the rover by hand along a
 *   tape measure. Read the /odom_raw x delta. It should match the tape to a few
 *   percent -- and the SIGN should be positive when pushed forward. This checks
 *   the constant and the direction sign together without a driven run.
 *
 * IF IT EVER NEEDS RE-DERIVING: drive a tape-measured distance at linear.x=0.05
 * (slow enough that the wheels cannot slip -- exactly the crawl this firmware
 * exists to make possible), then
 *     TRUE_CPR = FPMS_COUNTS_PER_REV * (reported_distance / tape_distance)
 * ========================================================================== */
#define FPMS_COUNTS_PER_REV 3255

#define COUNTS_PER_REV1 FPMS_COUNTS_PER_REV
#define COUNTS_PER_REV2 FPMS_COUNTS_PER_REV
#define COUNTS_PER_REV3 FPMS_COUNTS_PER_REV
#define COUNTS_PER_REV4 FPMS_COUNTS_PER_REV

#define WHEEL_DIAMETER 0.070
/* Track (LEFT-to-RIGHT wheel separation). The operator measured 105mm
   FRONT-to-BACK, which is the wheelbase, not this. Track is still unmeasured --
   measure it before trusting turn geometry. */
#define LR_WHEELS_DISTANCE 0.170

/* 20 kHz stays above audible; 10 bits gives 1024 duty steps so a crawl
 * setpoint has real resolution to land on. */
#define PWM_BITS 10
#define PWM_FREQUENCY 20000

/* USE_SHORT_BRAKE deliberately NOT defined. The golden driver stopped by
 * coasting (set_motor(0,0,0,0)) and every one of its tuned constants -- the
 * 0.93 turn coast factor above all -- is calibrated against coast. Active
 * braking on every PID zero-crossing would also add jerk during a crawl. */

/* INVERT ENCODER COUNTS -- MEASURED 2026-08-03, two clean passes, per wheel
 * (tools/enc_main.cpp drives one motor at a time and reports all four deltas):
 *
 *   motor   own encoder on FWD        own encoder on REV
 *   M1      -11802 / -12158           +13015 / +11508
 *   M2      -16356 / -17199           +13021 / +12808
 *   M3      -12835 / -11153           +9419  / +8788
 *   M4      -10850                    +12824
 *
 * All four count NEGATIVE when driven forward. Pin mapping is CORRECT -- motor N
 * moves encoder N; the small cross-counts are chassis shake.
 *
 * Left false, the velocity PID reads a negative RPM when it commanded positive,
 * so the error GROWS as it corrects and the wheel saturates at full duty. The
 * rover spins on a pure-forward command, and the odometry -- integrated from the
 * same inverted encoders -- reports clean forward travel. That lie fooled every
 * sensor-based test; only the operator watching caught it.
 */
/* CORRECTED 2026-08-04 after an OPEN-LOOP test the operator watched.
 *
 * Driving all four on IN_A at equal duty: LEFT ran forward, RIGHT ran backward.
 * No encoders, no PID -- a pure wiring fact. The right-side motors are wired
 * with opposite polarity (see MOTORn_INV below).
 *
 * That invalidates the first reading of the encoder test, which called IN_A
 * "forward" for every motor:
 *   LEFT  (firmware MOTOR1/3 = board M3/M4): IN_A really is forward, and the
 *         encoders counted NEGATIVE -> genuinely inverted.
 *   RIGHT (firmware MOTOR2/4 = board M1/M2): IN_A is physically BACKWARD, so
 *         counting negative was CORRECT -> not inverted.
 * Inverting all four (as first attempted) made the runaway worse: a commanded
 * 564mm produced -2080mm of phantom travel.
 */
#define MOTOR1_ENCODER_INV true    // front-left  (board M3/H3)
#define MOTOR2_ENCODER_INV false   // front-right (board M1/H1)
#define MOTOR3_ENCODER_INV true    // rear-left   (board M4/H4)
#define MOTOR4_ENCODER_INV false   // rear-right  (board M2/H2)

/* INVERT MOTOR DIRECTIONS -- MEASURED, operator-observed 2026-08-04.
 * The RIGHT-side motors are wired with opposite polarity to the left, so they
 * must be inverted or a forward command spins the rover on the spot. Verified
 * open-loop (no encoders, no PID): with the right pair flipped, all four run the
 * same physical direction and the rover crawls straight forward. */
#define MOTOR1_INV false   // front-left  (board M3)
#define MOTOR2_INV true    // front-right (board M1) -- wired opposite
#define MOTOR3_INV false   // rear-left   (board M4)
#define MOTOR4_INV true    // rear-right  (board M2) -- wired opposite

/* Physical side assignment follows the 2026-08-01 rewire in SESSION_HANDOFF.md
 * (M1=front-right, M2=rear-right, M3=front-left, M4=rear-left), which
 * contradicts the golden driver's older set_motor(L,L,R,R) comment. A mirrored
 * side assignment leaves FORWARD correct and inverts every TURN, so this is
 * verified per-motor before any mission rather than trusted. */
#define MOTOR1_ENCODER_A 11   // front-left  = board E3
#define MOTOR1_ENCODER_B 12
#define MOTOR2_ENCODER_A 6    // front-right = board E1
#define MOTOR2_ENCODER_B 7
#define MOTOR3_ENCODER_A 1    // rear-left   = board E4
#define MOTOR3_ENCODER_B 2
#define MOTOR4_ENCODER_A 47   // rear-right  = board E2
#define MOTOR4_ENCODER_B 48

#ifdef USE_BTS7960_MOTOR_DRIVER
  #define MOTOR1_PWM -1       // no separate enable pin on this topology
  #define MOTOR1_IN_A 9       // front-left  = board M3
  #define MOTOR1_IN_B 10

  #define MOTOR2_PWM -1
  #define MOTOR2_IN_A 4       // front-right = board M1
  #define MOTOR2_IN_B 5

  #define MOTOR3_PWM -1
  #define MOTOR3_IN_A 13      // rear-left   = board M4
  #define MOTOR3_IN_B 14

  #define MOTOR4_PWM -1
  #define MOTOR4_IN_A 15      // rear-right  = board M2
  #define MOTOR4_IN_B 16

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

/* The whole FPMS stack runs on domain 20. The board must join it explicitly
 * because in micro-ROS the CLIENT declares its domain in the CREATE_PARTICIPANT
 * request -- the agent's own ROS_DOMAIN_ID does not place it. Without this the
 * board publishes perfectly into domain 0, the agent logs a flawless session,
 * and every topic still shows Publisher count: 0.
 * Consumed by rcl_init_options_set_domain_id() in firmware.cpp. */
#define FPMS_ROS_DOMAIN_ID 20

/* 921600 proved unstable on this board: the micro-ROS session died within 1-3
 * minutes of every reset, the board never established a session on a cold boot,
 * and motion made it worse. A CP2102 at 921600 with four motor drivers
 * switching nearby is marginal, and framing errors kill the XRCE session.
 * 230400 trades headroom we do not need (odom+imu at 50Hz is a few KB/s) for
 * margin we badly do. MUST match the -b flag on the micro-ros-agent unit. */
#define BAUDRATE 230400
#define NODE_NAME "fpms_drive"

/* ICM42670P sits on I2C0 (board_pins.h:86-87). Wire is brought up by
 * BOARD_INIT below so the ported IMU driver can use it directly. */
#define SDA_PIN 40
#define SCL_PIN 39

/* board_pins.h:85 -- GPIO 3, NOT the template's GPIO 1, which on this board is
 * the rear-left encoder A channel. The divider ratio is unknown, so this
 * formula is provisional: the pack read 11.6 V on vendor firmware, which is
 * the reference point to calibrate BATTERY_ADJUST against. */
#define BATTERY_PIN 3
#define BATTERY_ADJUST(v) ((v) * (3.3 / 4096 * (33 + 10) / 10))
#define BATTERY_DIP 0.98

#define BOARD_INIT { \
    Wire.begin(SDA_PIN, SCL_PIN); \
    Wire.setClock(400000); \
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
    flashLED(3); \
    return false; }}

#endif
