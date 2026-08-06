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

/* No IMU macro on purpose: the ICM42670P on this board is not in upstream's
 * supported list, so imu.h auto-selects FakeIMU and the firmware builds and
 * runs. Heading will be wrong until the ICM42670P driver is ported -- and it
 * MUST be, because /odom_raw carries an identity quaternion (measured
 * 2026-08-03, 134 msgs), so there is no wheel-derived heading to fall back on
 * and the golden driver's +/-1-4 deg turns came entirely from this gyro. */

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

/* 1320 CPR comes from the golden phase6 driver's own constant
 * (_TKMM = pi*70/1320 = 0.1666 mm/tick, fpms_phase6_LATEST.py:1309), the code
 * that achieved 0.6%% distance error. The 1170 quoted in the project briefs is
 * NOT what the working code used. */
/* CORRECTED 2026-08-04 from an operator tape-measure. 14.8 counts/mm was
   measured over 800mm of real travel; 14.8 * pi * 70mm = 3255 counts/rev.
   The old 1320 came from the golden driver, which counted in a different mode,
   and is 2.5x too small -- the velocity PID would drive 2.5x faster than the
   commanded m/s and every odometry distance would read short by the same. */
#define COUNTS_PER_REV1 3255
#define COUNTS_PER_REV2 3255
#define COUNTS_PER_REV3 3255
#define COUNTS_PER_REV4 3255

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
