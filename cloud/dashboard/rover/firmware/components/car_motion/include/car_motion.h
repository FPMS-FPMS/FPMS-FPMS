/*
 * car_motion.h -- body-frame command in, four wheel speeds out.
 * Differential / skid-steer mixing, command slew limiting, and the
 * /cmd_vel watchdog.
 *
 * ==================================================================
 *  !!!!!  PHYSICAL LAYOUT: THE SIDES ARE MIRRORED  !!!!!
 * ==================================================================
 *
 * This chassis was rewired on 2026-08-01 while chasing a dead cable, and
 * the two sides SWAPPED PLACES. The motor connectors did not move; the
 * motors did. The current, physical truth is:
 *
 *      M1 (index 0) = FRONT-RIGHT        M3 (index 2) = FRONT-LEFT
 *      M2 (index 1) = REAR-RIGHT         M4 (index 3) = REAR-LEFT
 *
 * The vendor firmware, the third-party reference tree, the operator's own
 * Arduino sketch (built 2026-05-02, which explicitly labels M1/M2 as the
 * left side), and every document in this repo written before 2026-08-01
 * all assume the OPPOSITE. None of them is wrong -- they all predate the
 * rewire. Forward motion is unaffected either way, because both sides get
 * the same speed, which is exactly why this went unnoticed for so long.
 * ROTATION is inverted.
 *
 * The host stack compensates today: fpms_missions.py carries
 * TURN_WIRE_SIGN = -1 and applies it at exactly one site. THIS FIRMWARE
 * MAKES THAT COMPENSATION WRONG. Once this firmware is flashed, a
 * commanded +angular.z rotates the chassis counter-clockwise, per
 * REP-103, and TURN_WIRE_SIGN must become +1. See README, "Breaking
 * changes".
 *
 * If the chassis is ever rewired again, change MOTOR_SIDE_OF_INDEX below
 * and NOTHING ELSE. It is the single point of truth for the layout.
 */
#ifndef CAR_MOTION_H
#define CAR_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Build-time side assignment. -1 = left, +1 = right.
 * Index is the motor/encoder index (0 = M1 .. 3 = M4). */
#define MOTOR_SIDE_LEFT    (-1)
#define MOTOR_SIDE_RIGHT   (+1)

#define MOTOR_SIDE_OF_M1   MOTOR_SIDE_RIGHT   /* front-right */
#define MOTOR_SIDE_OF_M2   MOTOR_SIDE_RIGHT   /* rear-right  */
#define MOTOR_SIDE_OF_M3   MOTOR_SIDE_LEFT    /* front-left  */
#define MOTOR_SIDE_OF_M4   MOTOR_SIDE_LEFT    /* rear-left   */

/* Build-time drive polarity per motor, in the "positive = robot moves
 * forward" frame. On a mirrored chassis one side's motors are physically
 * rotated 180 degrees, so their H-bridge sense is reversed. WHICH side is
 * a wiring fact nobody has measured, so both are +1 here and the runtime
 * parameters invert_left / invert_right exist to fix it in the field
 * without a reflash. VERIFY WITH THE ONE-MOTOR-AT-A-TIME PROCEDURE IN
 * THE README BEFORE THE ROVER TOUCHES THE FLOOR. */
#define MOTOR_DRIVE_SIGN_M1   (+1)
#define MOTOR_DRIVE_SIGN_M2   (+1)
#define MOTOR_DRIVE_SIGN_M3   (+1)
#define MOTOR_DRIVE_SIGN_M4   (+1)

/* Same again for the encoders: +1 means counts increase when the wheel
 * drives the robot forward. */
#define MOTOR_ENC_SIGN_M1     (+1)
#define MOTOR_ENC_SIGN_M2     (+1)
#define MOTOR_ENC_SIGN_M3     (+1)
#define MOTOR_ENC_SIGN_M4     (+1)

void car_motion_init(void);

/* New /cmd_vel. vx in m/s (forward positive), wz in rad/s (CCW positive,
 * REP-103). Also feeds the watchdog. Safe to call from the micro-ROS task. */
void car_motion_set_cmd(float vx, float wz);

/* One 100 Hz control step: watchdog check, slew limit, mix, and run the
 * wheel loops. Call from the control task and nowhere else. */
void car_motion_step(void);

/* Measured body velocity reconstructed from the wheel encoders.
 * This is the SAME quantity, from the SAME wheel speeds, that odometry
 * integrates -- see odometry.c. They cannot disagree in sign. */
void car_motion_get_measured(float *vx, float *wz);

/* Immediate stop: zero the command, the ramp and the loops. */
void car_motion_stop(void);

/* True if the watchdog is currently holding the motors down because no
 * /cmd_vel has arrived recently. */
bool car_motion_watchdog_tripped(void);

/* Re-apply polarity from rover_config. Call after a parameter change. */
void car_motion_apply_polarity(void);

#ifdef __cplusplus
}
#endif
#endif /* CAR_MOTION_H */
