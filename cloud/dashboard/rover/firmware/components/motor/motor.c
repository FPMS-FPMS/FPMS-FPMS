#include "motor.h"

#include <math.h>
#include <string.h>

#include "encoder.h"
#include "rover_config.h"

/* Integrator clamp, in encoder counts of lag. A stalled wheel must not be
 * able to wind the position error up without bound: the base term keeps a
 * little authority available at a standstill, and the speed-proportional
 * term lets a fast wheel carry proportionally more lag before the clamp
 * bites. Roughly 60 counts at rest is ~13 mm of wheel travel with the
 * default geometry -- enough to pull through a carpet edge, not enough to
 * store a lurch. */
#define POS_ERR_BASE_COUNTS   60.0f
#define POS_ERR_SPEED_GAIN    4.0f

/* Reporting filter on measured wheel speed. The raw per-period count is a
 * small integer (2-60 counts), so it is coarse; this smooths /odom_raw's
 * twist without touching the control path, which still sees raw counts. */
#define SPEED_FILTER_ALPHA    0.30f

typedef struct {
    float  ref_counts;     /* fractional commanded position */
    float  meas_counts;    /* measured position             */
    float  target_cps;     /* counts per control period     */
    float  prev_vel_err;
    float  speed_mps;      /* filtered, for reporting       */
    int32_t last_delta;
    int8_t drive_sign;
    int8_t enc_sign;
} wheel_t;

static wheel_t s_w[MOTOR_COUNT];

void motor_init(void)
{
    memset(s_w, 0, sizeof(s_w));
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        s_w[i].drive_sign = 1;
        s_w[i].enc_sign   = 1;
    }
    pwm_motor_init();
    encoder_init();
}

void motor_set_polarity(int idx, int drive_sign, int enc_sign)
{
    if (idx < 0 || idx >= MOTOR_COUNT) {
        return;
    }
    s_w[idx].drive_sign = (drive_sign < 0) ? -1 : 1;
    s_w[idx].enc_sign   = (enc_sign   < 0) ? -1 : 1;
}

void motor_set_target_mps(const float target_mps[MOTOR_COUNT])
{
    const float cpm = rover_config_counts_per_metre();
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        float v = target_mps[i];
        if (!isfinite(v)) {
            v = 0.0f;
        }
        s_w[i].target_cps = v * cpm * MOTOR_CONTROL_DT_S;
    }
}

void motor_update(void)
{
    const rover_config_t *cfg = rover_config_get();

    int32_t raw[MOTOR_COUNT];
    encoder_sample(raw);

    const float mpc = rover_config_metres_per_count();

    /* Feed-forward gain: duty ticks per (count per control period).
     * = full duty / counts-per-period at full speed. */
    const float max_cps = cfg->max_wheel_mps
                        * rover_config_counts_per_metre()
                        * MOTOR_CONTROL_DT_S;
    const float kff = (max_cps > 0.01f) ? ((float)PWM_FULL_SCALE / max_cps) : 0.0f;

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        wheel_t *w = &s_w[i];

        const float delta = (float)(w->enc_sign * raw[i]);
        w->last_delta   = w->enc_sign * raw[i];
        w->meas_counts += delta;

        const float meas_mps = (delta * mpc) / MOTOR_CONTROL_DT_S;
        w->speed_mps += SPEED_FILTER_ALPHA * (meas_mps - w->speed_mps);

        if (w->target_cps == 0.0f) {
            /* A commanded stop is a real stop: coast, and drop the loop
             * state so nothing is stored up for the next command. Holding
             * zero speed under closed loop would be a servo brake, which is
             * the wrong behaviour for a watchdog trip on a rover that may
             * be being pushed by hand. */
            w->ref_counts   = w->meas_counts;
            w->prev_vel_err = 0.0f;
            pwm_motor_set((pwm_motor_id_t)i, 0);
            continue;
        }

        w->ref_counts += w->target_cps;

        float pos_err = w->ref_counts - w->meas_counts;

        const float lim = POS_ERR_BASE_COUNTS
                        + POS_ERR_SPEED_GAIN * fabsf(w->target_cps);
        if (pos_err > lim) {
            pos_err       = lim;
            w->ref_counts = w->meas_counts + lim;   /* back-calculation */
        } else if (pos_err < -lim) {
            pos_err       = -lim;
            w->ref_counts = w->meas_counts - lim;
        }

        const float vel_err = w->target_cps - delta;

        float u = kff * w->target_cps
                + cfg->pid_kp * vel_err
                + cfg->pid_ki * pos_err
                + cfg->pid_kd * (vel_err - w->prev_vel_err);
        w->prev_vel_err = vel_err;

        if (u >  (float)PWM_FULL_SCALE) u =  (float)PWM_FULL_SCALE;
        if (u < -(float)PWM_FULL_SCALE) u = -(float)PWM_FULL_SCALE;

        /* pwm_motor_set() applies the min-PWM map. Note the request handed
         * over is the controller's own units on a 0..400 scale -- the map
         * is the only thing that knows about MIN_PWM, and it lives in one
         * place so it cannot drift out of sync between motors. */
        pwm_motor_set((pwm_motor_id_t)i, (int32_t)(w->drive_sign * (int32_t)lrintf(u)));
    }
}

void motor_get_speed_mps(float out[MOTOR_COUNT])
{
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        out[i] = s_w[i].speed_mps;
    }
}

void motor_get_last_delta(int32_t out[MOTOR_COUNT])
{
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        out[i] = s_w[i].last_delta;
    }
}

void motor_stop(bool brake)
{
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        s_w[i].target_cps   = 0.0f;
        s_w[i].ref_counts   = s_w[i].meas_counts;
        s_w[i].prev_vel_err = 0.0f;
        s_w[i].speed_mps    = 0.0f;
    }
    pwm_motor_stop_all(brake);
}
