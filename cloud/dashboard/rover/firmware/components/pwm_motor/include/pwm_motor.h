/*
 * pwm_motor.h -- H-bridge PWM output stage for the four drive motors.
 *
 * ==================================================================
 *  THIS FILE CONTAINS THE FIX. Everything else in this firmware is
 *  scaffolding around it.
 * ==================================================================
 *
 * THE DEFECT
 * ----------
 * The vendor / reference driver for this board linearises motor stiction
 * by ADDING a constant to the controller output:
 *
 *     #define PWM_MOTOR_DUTY_TICK_MAX (10000000 / 25000)   // 400
 *     #define PWM_MOTOR_DEAD_ZONE     (200)                // 50% of full scale
 *     #define PWM_MOTOR_MAX_VALUE     (400 - 200)          // PID output clamp
 *
 *     static int PwmMotor_Ignore_Dead_Zone(int speed) {
 *         if (speed > 0) return speed + PWM_MOTOR_DEAD_ZONE;
 *         if (speed < 0) return speed - PWM_MOTOR_DEAD_ZONE;
 *         return 0;
 *     }
 *
 * Consequences, all of them arithmetic rather than opinion:
 *
 *   - The smallest non-zero output is (1 + 200)/400 = 50.25% duty. The
 *     bottom half of the actuator range does not exist.
 *   - The PID is clamped to +/-200, so the controller has 200 distinct
 *     levels instead of 400: half the resolution as well as half the range.
 *   - Every non-zero /cmd_vel, however small, produces the same lurch.
 *
 * Measured on this rover, exactly as predicted: minimum burst ~0.35 s,
 * minimum move ~230 mm, and a 300 mm commanded segment that travelled
 * about a metre and hit an obstacle.
 *
 * THE FIX: MAP, DO NOT ADD
 * ------------------------
 * A non-zero request is mapped PROPORTIONALLY onto [MIN_PWM, FULL_SCALE]:
 *
 *     |req| <= EPS            ->  0                     (a real, silent stop)
 *     |req| >  EPS            ->  MIN_PWM + (|req| - EPS)
 *                                          * (FULL - MIN_PWM)
 *                                          / (FULL - EPS)
 *
 * Properties, in contrast with the additive version:
 *
 *   - SURJECTIVE.  The full commanded range [EPS, FULL] covers the full
 *     usable duty range [MIN_PWM, FULL]. Nothing is unreachable.
 *   - MONOTONIC and CONTINUOUS above EPS: doubling the request roughly
 *     doubles the torque, which is what every caller already assumes.
 *   - FULL AUTHORITY. The controller is clamped to +/-FULL_SCALE, not
 *     +/-(FULL_SCALE - MIN_PWM), so it keeps all 400 levels.
 *   - Still breaks stiction when asked to: the smallest non-zero output
 *     is MIN_PWM, which is meant to be the smallest duty that turns a
 *     loaded wheel.
 *
 * MIN_PWM DEFAULTS TO ZERO
 * ------------------------
 * At MIN_PWM = 0 the map above is a pure proportional pass-through: one
 * tick of request gives one tick of duty. That is precisely what the
 * operator's own known-good Arduino driver for this board did -- same
 * 25 kHz carrier, duty written straight through across its full 0..255
 * range, no dead-zone term -- and it drove this hardware. So the default
 * is the one setting on this axis with measured evidence behind it, not
 * a guess, and the smallest non-zero output drops from 50.25% duty to
 * 0.25% while the number of distinct levels rises from 200 to 400.
 *
 * Raise MIN_PWM only if a LOADED wheel is measured to stall at low duty,
 * and raise it as little as that measurement allows.
 *
 * BELOW MIN_PWM (when it is non-zero)
 * -----------------------------------
 * Speeds slower than a non-zero MIN_PWM produces are still reachable, but
 * by duty modulation rather than by a lower duty: the velocity loop drives
 * the request across EPS at up to 100 Hz, so the wheel gets short MIN_PWM
 * pulses. Even then the quantum is a 10 ms pulse rather than today's
 * ~0.35 s stiction lurch at 50% duty -- and the smaller MIN_PWM is, the
 * smaller that quantum. One more reason to tune it DOWN, not up.
 *
 * MIN_PWM and EPS are runtime parameters (see rover_config.h). Calibrating
 * them must not need a reflash.
 */
#ifndef PWM_MOTOR_H
#define PWM_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 25 kHz carrier (inaudible) from a nominal 10 MHz timebase => 400 ticks per
 * period. The 400-tick scale is deliberately identical to the vendor's so that
 * every number in research/R4_FIRMWARE.md, and every value the operator has in
 * their head, transfers across without conversion.
 *
 * THIS IS THE COMMAND SCALE, AND IT IS PUBLIC. motor.c clamps its PID output
 * to +/-PWM_FULL_SCALE and derives its feed-forward gain from it, and
 * pwm_motor_last_duty() reports in it. Do not change it to suit a peripheral.
 *
 * The output stage is LEDC (see pwm_motor.c for why -- the previous MCPWM
 * stage was inferred rather than evidenced, and drove nothing on hardware).
 * LEDC resolution is a bit depth, so pwm_motor.c runs its timer at 10 bits
 * and converts 400 command ticks -> 1024 duty ticks at the last step.
 * 1024 > 400, so every command level survives; min_pwm_percent and
 * pwm_eps_percent are applied BEFORE that conversion, on this 400-tick scale,
 * and their meaning is exactly unchanged. */
#define PWM_TIMER_RESOLUTION_HZ   10000000
#define PWM_CARRIER_FREQ_HZ       25000
#define PWM_FULL_SCALE            (PWM_TIMER_RESOLUTION_HZ / PWM_CARRIER_FREQ_HZ)  /* 400 */

typedef enum {
    PWM_MOTOR_M1 = 0,
    PWM_MOTOR_M2 = 1,
    PWM_MOTOR_M3 = 2,
    PWM_MOTOR_M4 = 3,
    PWM_MOTOR_COUNT = 4,
} pwm_motor_id_t;

/* Configure LEDC and park all four bridges in coast (all eight half-bridge
 * inputs driven low) before the timer is ever started. */
void pwm_motor_init(void);

/* Apply a signed request in [-PWM_FULL_SCALE, +PWM_FULL_SCALE].
 * The min-PWM map documented above is applied here and only here.
 * Positive = the direction that drives the A pin; per-motor polarity is
 * resolved one level up, in car_motion.c. */
void pwm_motor_set(pwm_motor_id_t id, int32_t request);

/* Stop one or all motors. brake=true shorts the winding (both bridge
 * inputs high); brake=false coasts (both low). The watchdog coasts --
 * braking a runaway is more violent than letting it roll. */
void pwm_motor_stop(pwm_motor_id_t id, bool brake);
void pwm_motor_stop_all(bool brake);

/* Duty ticks last written, signed, for telemetry and calibration.
 * This is the value AFTER the min-PWM map. */
int32_t pwm_motor_last_duty(pwm_motor_id_t id);

/* The mapping function, exposed so it can be reasoned about and unit
 * tested off-target without bringing up LEDC. Pure function of its
 * argument plus the live rover_config. */
int32_t pwm_motor_map_min_pwm(int32_t request);

#ifdef __cplusplus
}
#endif
#endif /* PWM_MOTOR_H */
