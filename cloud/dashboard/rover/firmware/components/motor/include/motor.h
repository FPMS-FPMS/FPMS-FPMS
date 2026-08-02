/*
 * motor.h -- per-wheel closed-loop velocity control.
 *
 * Four independent loops, one per wheel, each producing a signed request in
 * [-PWM_FULL_SCALE, +PWM_FULL_SCALE] that is handed to pwm_motor_set(), which
 * applies the min-PWM map. This module never touches duty directly.
 *
 * TWO DEPARTURES FROM THE VENDOR / REFERENCE DESIGN
 * -------------------------------------------------
 *
 * 1. FULL CONTROLLER AUTHORITY.
 *    The reference clamps its PID output to PWM_MOTOR_MAX_VALUE, defined as
 *    (400 - dead_zone) = 200, because the dead zone is then added on top.
 *    Since we map instead of adding, the controller keeps the whole
 *    +/-400 range and therefore twice the resolution.
 *
 * 2. THE INTEGRAL IS A FRACTIONAL POSITION ERROR, NOT A SUM OF INTEGER
 *    VELOCITY ERRORS.
 *    This is what makes slow motion controllable at all, and it is worth
 *    being precise about. The reference converts a commanded speed to an
 *    integer count target per 10 ms period:
 *
 *        speed_count[i] = speed_m[i] / (WHEEL_CIRCLE/ENCODER_CIRCLE/PID_PERIOD)
 *
 *    At 0.003 m/s that target is 0.2 counts per period, while the feedback
 *    is an integer that is 0 most periods and 1 occasionally. The loop is
 *    not controlling anything: it is dithering, and with the additive dead
 *    zone every dither became a 50%-duty lurch.
 *
 *    Here the reference position advances by a FLOAT number of counts every
 *    period and the error is (reference position - measured position). A
 *    0.2 count/period setpoint accumulates one full count of error every
 *    five periods and the loop tracks it properly. Mathematically the
 *    integral of velocity error IS position error, so this is the same PID
 *    -- but computed without quantising the thing being integrated.
 *
 * FEED-FORWARD
 * ------------
 * A duty feed-forward term, kff * target, is derived automatically from
 * rover_config's max_wheel_mps and wheel geometry, so the loop starts near
 * the right duty instead of integrating up to it from zero. An error in
 * max_wheel_mps costs responsiveness only; the integrator absorbs it.
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "pwm_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_COUNT 4

/* Control period. 100 Hz. The reference used 10 ms (100 Hz) as well; the
 * value matters because kp/ki/kd are expressed per control period. */
#define MOTOR_CONTROL_PERIOD_MS  10
#define MOTOR_CONTROL_DT_S       (MOTOR_CONTROL_PERIOD_MS / 1000.0f)

void motor_init(void);

/* Per-motor polarity, in the "positive = drives the robot forward" frame.
 * drive_sign flips which half of the H-bridge is driven; enc_sign flips the
 * sense of the encoder. Normally they move together (a motor mounted
 * backwards inverts both), but a swapped encoder pair inverts only enc_sign,
 * so they are independent. Set by car_motion.c from rover_config. */
void motor_set_polarity(int idx, int drive_sign, int enc_sign);

/* Target wheel speed, metres per second at the tyre contact patch,
 * positive = forward. */
void motor_set_target_mps(const float target_mps[MOTOR_COUNT]);

/* One control step: sample encoders, run the four loops, write PWM.
 * Call exactly once per MOTOR_CONTROL_PERIOD_MS from the control task. */
void motor_update(void);

/* Measured wheel speeds (m/s, lightly filtered) and the raw per-period
 * count deltas from the most recent motor_update(). */
void motor_get_speed_mps(float out[MOTOR_COUNT]);
void motor_get_last_delta(int32_t out[MOTOR_COUNT]);

/* Zero the targets, clear every integrator, and stop the bridges.
 * Clearing the integrators matters: a watchdog stop that left the position
 * error intact would slam the motors the instant commands resumed. */
void motor_stop(bool brake);

#ifdef __cplusplus
}
#endif
#endif /* MOTOR_H */
