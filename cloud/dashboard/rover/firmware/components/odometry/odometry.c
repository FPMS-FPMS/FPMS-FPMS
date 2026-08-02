#include "odometry.h"

#include <math.h>
#include <string.h>

#include "car_motion.h"
#include "freertos/FreeRTOS.h"
#include "motor.h"
#include "rover_config.h"

/* Light low-pass on the reported velocities only. The pose integration
 * uses the raw per-step displacement, so filtering here cannot introduce
 * a pose/twist inconsistency -- it only smooths the number, never its
 * sign, because a first-order filter with a positive coefficient cannot
 * cross zero against a same-signed input. */
#define VEL_FILTER_ALPHA  0.25f

static odom_state_t   s_st;
static portMUX_TYPE   s_mux = portMUX_INITIALIZER_UNLOCKED;

/* Mirrors car_motion.h. Kept local rather than exported so that changing
 * the layout still means editing exactly one place: MOTOR_SIDE_OF_M*. */
static const int8_t k_side[MOTOR_COUNT] = {
    MOTOR_SIDE_OF_M1, MOTOR_SIDE_OF_M2, MOTOR_SIDE_OF_M3, MOTOR_SIDE_OF_M4,
};

void odometry_init(void)
{
    memset(&s_st, 0, sizeof(s_st));
}

void odometry_reset(void)
{
    portENTER_CRITICAL(&s_mux);
    s_st.x = 0.0f;
    s_st.y = 0.0f;
    s_st.theta = 0.0f;
    portEXIT_CRITICAL(&s_mux);
}

void odometry_step(void)
{
    const rover_config_t *cfg = rover_config_get();
    const float mpc = rover_config_metres_per_count();

    int32_t d[MOTOR_COUNT];
    motor_get_last_delta(d);

    /* Metres travelled by each side during this control period. */
    float sum_l = 0.0f, sum_r = 0.0f;
    int   n_l = 0, n_r = 0;
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        const float dist = (float)d[i] * mpc;
        if (k_side[i] == MOTOR_SIDE_LEFT) { sum_l += dist; n_l++; }
        else                              { sum_r += dist; n_r++; }
    }
    const float dl = (n_l > 0) ? (sum_l / (float)n_l) : 0.0f;
    const float dr = (n_r > 0) ? (sum_r / (float)n_r) : 0.0f;

    /* ---- the single source of both pose and twist ---- */
    const float d_centre = 0.5f * (dl + dr);
    const float d_theta  = (cfg->track_width_m > 1e-4f)
                         ? ((dr - dl) / cfg->track_width_m)
                         : 0.0f;

    /* Second-order (midpoint) arc integration. Over a 10 ms step the
     * difference from the naive form is negligible, but it costs nothing
     * and removes a systematic inward bias on sustained turns. */
    const float mid = s_st.theta + 0.5f * d_theta;
    const float dx  = d_centre * cosf(mid);
    const float dy  = d_centre * sinf(mid);

    const float inst_vx = d_centre / MOTOR_CONTROL_DT_S;
    const float inst_wz = d_theta  / MOTOR_CONTROL_DT_S;

    portENTER_CRITICAL(&s_mux);
    s_st.x     += dx;
    s_st.y     += dy;
    s_st.theta += d_theta;
    /* wrap to (-pi, pi] */
    if (s_st.theta > (float)M_PI)  s_st.theta -= 2.0f * (float)M_PI;
    if (s_st.theta < -(float)M_PI) s_st.theta += 2.0f * (float)M_PI;

    s_st.vx += VEL_FILTER_ALPHA * (inst_vx - s_st.vx);
    s_st.wz += VEL_FILTER_ALPHA * (inst_wz - s_st.wz);
    portEXIT_CRITICAL(&s_mux);
}

void odometry_get(odom_state_t *out)
{
    if (!out) {
        return;
    }
    portENTER_CRITICAL(&s_mux);
    *out = s_st;
    portEXIT_CRITICAL(&s_mux);
}
