#ifndef FPMS_CONFIG_H
#define FPMS_CONFIG_H

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

/* 1320 CPR comes from the golden phase6 driver's own constant
 * (_TKMM = pi*70/1320 = 0.1666 mm/tick, fpms_phase6_LATEST.py:1309), the code
 * that achieved 0.6%% distance error. The 1170 quoted in the project briefs is
 * NOT what the working code used. */
#define COUNTS_PER_REV1 1320
#define COUNTS_PER_REV2 1320
#define COUNTS_PER_REV3 1320
#define COUNTS_PER_REV4 1320

#define WHEEL_DIAMETER 0.070
#define LR_WHEELS_DISTANCE 0.170

/* 20 kHz stays above audible; 10 bits gives 1024 duty steps so a crawl
 * setpoint has real resolution to land on. */
#define PWM_BITS 10
#define PWM_FREQUENCY 20000

/* USE_SHORT_BRAKE deliberately NOT defined. The golden driver stopped by
 * coasting (set_motor(0,0,0,0)) and every one of its tuned constants -- the
 * 0.93 turn coast factor above all -- is calibrated against coast. Active
 * braking on every PID zero-crossing would also add jerk during a crawl. */

// INVERT ENCODER COUNTS -- all false until measured per wheel, not assumed
#define MOTOR1_ENCODER_INV false
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV false
#define MOTOR4_ENCODER_INV false

// INVERT MOTOR DIRECTIONS -- likewise
#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false

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

/* The whole FPMS stack runs on domain 20; the board must join it
 * explicitly because the micro-ROS client, not the agent, picks the
 * domain. */
#define FPMS_ROS_DOMAIN_ID 20

#define BAUDRATE 921600
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
