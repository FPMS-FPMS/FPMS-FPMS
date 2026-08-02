/*
 * rover_config.h -- every per-robot constant that MUST be calibrated on the
 * physical rover, in one struct, settable at runtime over ROS parameters and
 * persisted to NVS so calibration survives a power cycle.
 *
 * WHY THIS EXISTS
 * ---------------
 * The defect this firmware fixes was a compile-time `#define` that could only
 * be changed by a reflash (PWM_MOTOR_DEAD_ZONE). Calibrating it needs perhaps
 * twenty iterations of "try a value, watch the wheel". Twenty reflashes is a
 * day; twenty `ros2 param set` calls is ten minutes.
 *
 * RULE: if a number describes THIS robot rather than THIS board, it belongs
 * here, not in a #define.
 */
#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== *
 *  DEFAULTS
 *
 *  Read the justification for every one of these in README.md
 *  ("Constants that MUST be measured on hardware"). Several are known
 *  to be wrong and are flagged MEASURE-ME.
 * ================================================================== */

/* --- THE FIX -------------------------------------------------------
 * Minimum duty, as a percentage of full scale, that a NON-ZERO speed
 * request is mapped onto. See pwm_motor.c :: pwm_motor_map_min_pwm().
 *
 * The stock firmware ADDED 200 of a 400-tick scale (50.0%) to the PID
 * output as a feed-forward stiction kick. That made 50.25% duty the
 * smallest expressible non-zero output and deleted the bottom half of
 * the actuator range. We instead MAP proportionally into
 * [MIN_PWM, 100%], so the whole commanded range stays expressible.
 *
 * THE DEFAULT IS ZERO, AND THAT IS DELIBERATE.
 *
 * At 0 the map degenerates to a pure proportional pass-through: a
 * request of 1 tick produces 1 tick of duty. That is EXACTLY what the
 * operator's own known-good Arduino driver for this board did (25 kHz
 * carrier, 8-bit duty written straight through, no dead-zone term at
 * all) and it demonstrably drove this hardware. So zero is not a guess
 * -- it is the one value on this axis with measured evidence behind it.
 *
 * A non-zero MIN_PWM is a stiction feed-forward, and it is available the
 * moment anyone wants it:
 *
 *     ros2 param set /YB_Car_Node min_pwm_percent 8.0
 *
 * RAISE IT ONLY IF A LOADED WHEEL MEASURABLY STALLS AT LOW DUTY, AND
 * RAISE IT AS LITTLE AS POSSIBLE. The asymmetry matters: too low is
 * benign, because the velocity loop's integrator winds the duty up until
 * the wheel breaks free and you lose only a little low-end
 * responsiveness. Too high RECREATES THE EXACT DEFECT WE ARE FIXING.
 * research/R4_FIRMWARE.md suggests 7.5-15% if a floor turns out to be
 * needed at all; do not start there, end there. */
#define CFG_DEF_MIN_PWM_PERCENT      0.0f
#define CFG_MAX_MIN_PWM_PERCENT      40.0f   /* refuse anything sillier */

/* Controller-output magnitude, in percent of full scale, below which we
 * emit a hard zero instead of MIN_PWM. Without this the motor would buzz
 * at MIN_PWM forever on PID noise around a zero setpoint. */
#define CFG_DEF_PWM_EPS_PERCENT      0.5f

/* --- Velocity loop -------------------------------------------------
 * Acting on encoder counts per control period. See motor.c for the
 * exact form; note the integral term is computed from a FRACTIONAL
 * reference position, not from accumulated integer errors, which is
 * what makes sub-count-per-tick setpoints controllable at all. */
#define CFG_DEF_PID_KP               1.5f
#define CFG_DEF_PID_KI               0.35f
#define CFG_DEF_PID_KD               0.0f

/* --- Wheel geometry ------------------------------------------------
 * !!! STILL MEASURE-ME, BUT NO LONGER A COIN TOSS. !!!
 *
 * Four pairs are in circulation for this rover:
 *
 *   counts/rev   mm/rev   counts/mm   source
 *   ----------   ------   ---------   ---------------------------------------
 *      1170       219.9      5.32     <-- DEFAULT. Cross-checked against the
 *                                     operator's own working Arduino driver
 *                                     for this board, which states 70 mm
 *                                     wheels / 1170 CPR / 5.32 counts per mm.
 *                                     The three numbers are self-consistent
 *                                     (1170 / (pi*70) = 5.32), which no other
 *                                     pair on this list manages.
 *      1040       150.8      6.90     quoted for the stock firmware. 1040 is
 *                                     corroborated by Yahboom's own encoder
 *                                     note (13 lines x 20 reduction x 4 edges)
 *                                     but 150.8 mm is corroborated by nothing
 *                                     and is not a 70 mm wheel.
 *      1320       219.9      6.00     host stack (fpms_odom_tf.py). The 70 mm
 *                                     wheel agrees; the 1320 CPR does not.
 *      2244       396.4      5.66     the third-party reference repo -- a
 *                                     DIFFERENT robot. Listed only so nobody
 *                                     copies it in by accident.
 *
 * The 1170/219.9 pair is the only one where the wheel diameter, the count
 * and the counts-per-mm all agree with each other, so it is the default.
 * It is still not a measurement taken on THIS rover today. Confirm it
 * with the two-minute push-the-rover-2-metres procedure in the README --
 * every distance the rover reports is proportional to this number. */
#define CFG_DEF_ENC_COUNTS_PER_REV   1170.0f
#define CFG_DEF_WHEEL_CIRCUM_MM      219.9f

/* Effective skid-steer track width in metres. NOT the geometric track:
 * a skid-steer chassis scrubs, so the yaw rate you actually get from a
 * given wheel-speed difference corresponds to a track WIDER than the
 * ruler distance between the wheels. The geometric track was recorded
 * as 0.170 m; the effective value is typically 1.2-1.8x that.
 * (The 100 mm figure that appears elsewhere for this chassis is the
 * WHEELBASE, front axle to rear axle, and is not this number.)
 * MEASURE-ME (spin-in-place procedure in README). */
#define CFG_DEF_TRACK_WIDTH_M        0.170f

/* Wheel speed, m/s, at 100% duty and no load. Used ONLY to derive the
 * feed-forward gain (duty ticks per count-per-period), so an error here
 * costs responsiveness, not accuracy -- the integrator absorbs it.
 * Derived from a measured 0.65 m/s at the stock firmware's ~50% duty
 * floor (fpms_missions.py FULL_DUTY_MPS). MEASURE-ME. */
#define CFG_DEF_MAX_WHEEL_MPS        1.30f

/* --- Command shaping ----------------------------------------------- */
#define CFG_DEF_MAX_ACCEL_MPS2       1.00f  /* slew limit on body Vx      */
#define CFG_DEF_MAX_ALPHA_RADPS2     6.00f  /* slew limit on body Wz      */
#define CFG_DEF_CMD_TIMEOUT_MS       500    /* /cmd_vel watchdog (req. 7) */

/* --- Battery -------------------------------------------------------
 * v_pack = adc_volts * BAT_DIVIDER * bat_scale
 * The divider ratio on this board is NOT documented. 5.0 is the ratio
 * required for a 12.6 V pack to stay inside the ESP32-S3 ADC's usable
 * range at 11 dB attenuation, so it is a plausible design value -- but
 * it is a GUESS. MEASURE-ME: put a multimeter on the pack, compare with
 * /battery, and set bat_scale to the ratio. */
#define CFG_DEF_BAT_DIVIDER          5.0f
#define CFG_DEF_BAT_SCALE            1.0f

/* --- Polarity ------------------------------------------------------
 * Field-fixable sign flips, for when a motor or encoder turns out to be
 * wired backwards. Build-time side assignment lives in car_motion.h;
 * these are the runtime escape hatch so a wiring surprise does not need
 * a reflash. */
#define CFG_DEF_INVERT_LEFT          false
#define CFG_DEF_INVERT_RIGHT         false
#define CFG_DEF_INVERT_ENC_LEFT      false
#define CFG_DEF_INVERT_ENC_RIGHT     false

/* ================================================================== */

typedef struct {
    /* the fix */
    float    min_pwm_percent;
    float    pwm_eps_percent;
    /* velocity loop */
    float    pid_kp;
    float    pid_ki;
    float    pid_kd;
    /* geometry */
    float    enc_counts_per_rev;
    float    wheel_circum_mm;
    float    track_width_m;
    float    max_wheel_mps;
    /* shaping / safety */
    float    max_accel_mps2;
    float    max_alpha_radps2;
    int32_t  cmd_timeout_ms;
    /* battery */
    float    bat_divider;
    float    bat_scale;
    /* polarity */
    bool     invert_left;
    bool     invert_right;
    bool     invert_enc_left;
    bool     invert_enc_right;
} rover_config_t;

/* Load defaults, then overlay whatever was persisted in NVS.
 * Never fails: a missing/corrupt NVS blob just leaves the defaults. */
void rover_config_init(void);

/* Read-only pointer to the live config. Fields are plain scalars written
 * only from the micro-ROS task and read from the control task; on Xtensa
 * these are naturally atomic word writes, and every consumer re-reads
 * them once per control period, so no lock is needed. */
const rover_config_t *rover_config_get(void);

/* Mutable access for the parameter server. Call rover_config_commit()
 * after a batch of writes so dependent caches (counts-per-metre, the
 * feed-forward gain, the min-PWM tick value) are recomputed. */
rover_config_t *rover_config_mut(void);
void rover_config_commit(void);

/* Persist the current config to NVS. Returns true on success. */
bool rover_config_save(void);

/* Derived, recomputed by rover_config_commit(). */
float rover_config_counts_per_metre(void);   /* encoder counts per metre    */
float rover_config_metres_per_count(void);

#ifdef __cplusplus
}
#endif
#endif /* ROVER_CONFIG_H */
