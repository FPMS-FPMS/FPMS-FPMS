#include "car_motion.h"

#include <math.h>

#include "esp_timer.h"
#include "motor.h"
#include "rover_config.h"

static const int8_t k_side[MOTOR_COUNT] = {
    MOTOR_SIDE_OF_M1, MOTOR_SIDE_OF_M2, MOTOR_SIDE_OF_M3, MOTOR_SIDE_OF_M4,
};
static const int8_t k_drive_sign[MOTOR_COUNT] = {
    MOTOR_DRIVE_SIGN_M1, MOTOR_DRIVE_SIGN_M2, MOTOR_DRIVE_SIGN_M3, MOTOR_DRIVE_SIGN_M4,
};
static const int8_t k_enc_sign[MOTOR_COUNT] = {
    MOTOR_ENC_SIGN_M1, MOTOR_ENC_SIGN_M2, MOTOR_ENC_SIGN_M3, MOTOR_ENC_SIGN_M4,
};

/* Commanded (raw, from /cmd_vel) and ramped (what the wheels actually chase).
 * Written from the micro-ROS task on core 0, read from the control task on
 * core 1. All three are single 32-bit aligned scalars, so a torn read is not
 * possible.
 *
 * s_cmd_ms is deliberately a 32-bit MILLISECOND stamp rather than the raw
 * 64-bit esp_timer microsecond value: a 64-bit load on a 32-bit core is two
 * instructions and CAN tear across a concurrent write, which in this exact
 * place would mean a spurious or a missed watchdog trip. Unsigned subtraction
 * is correct across the ~49 day wrap. */
static volatile float    s_cmd_vx;
static volatile float    s_cmd_wz;
static volatile uint32_t s_cmd_ms;
static volatile bool     s_cmd_seen;

static float s_ramp_vx;
static float s_ramp_wz;
static bool  s_wd_tripped = true;      /* start tripped: no command yet */

static float s_meas_vx;
static float s_meas_wz;

static float slew(float current, float target, float max_delta)
{
    const float d = target - current;
    if (d >  max_delta) return current + max_delta;
    if (d < -max_delta) return current - max_delta;
    return target;
}

void car_motion_apply_polarity(void)
{
    const rover_config_t *cfg = rover_config_get();
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        const bool left = (k_side[i] == MOTOR_SIDE_LEFT);
        const int inv_drive = left ? (cfg->invert_left  ? -1 : 1)
                                   : (cfg->invert_right ? -1 : 1);
        const int inv_enc   = left ? (cfg->invert_enc_left  ? -1 : 1)
                                   : (cfg->invert_enc_right ? -1 : 1);
        motor_set_polarity(i,
                           k_drive_sign[i] * inv_drive,
                           k_enc_sign[i]   * inv_enc);
    }
}

void car_motion_init(void)
{
    motor_init();
    car_motion_apply_polarity();
    s_cmd_vx = 0.0f;
    s_cmd_wz = 0.0f;
    s_cmd_ms = 0;
    s_cmd_seen = false;
    s_ramp_vx = 0.0f;
    s_ramp_wz = 0.0f;
    s_wd_tripped = true;
}

void car_motion_set_cmd(float vx, float wz)
{
    if (!isfinite(vx)) vx = 0.0f;
    if (!isfinite(wz)) wz = 0.0f;
    s_cmd_vx = vx;
    s_cmd_wz = wz;
    /* Stamp LAST: the control task must never see a fresh timestamp
     * attached to a stale command. */
    s_cmd_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_cmd_seen = true;
}

void car_motion_stop(void)
{
    s_cmd_vx  = 0.0f;
    s_cmd_wz  = 0.0f;
    s_ramp_vx = 0.0f;
    s_ramp_wz = 0.0f;
    motor_stop(false);
}

bool car_motion_watchdog_tripped(void) { return s_wd_tripped; }

void car_motion_step(void)
{
    const rover_config_t *cfg = rover_config_get();

    /* -------- watchdog (requirement 7) --------------------------------
     * Independent of micro-ROS: this runs in the control task, so it
     * still fires if the executor stalls, the agent dies, or the USB
     * cable is pulled. That is the whole point -- a watchdog that lives
     * inside the thing it is watching is not a watchdog. */
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    const uint32_t age_ms = now_ms - s_cmd_ms;   /* wrap-safe */
    const bool stale = (!s_cmd_seen) || (age_ms > (uint32_t)cfg->cmd_timeout_ms);

    float want_vx, want_wz;
    if (stale) {
        if (!s_wd_tripped) {
            /* Edge: drop the ramp immediately rather than decelerating
             * politely. If we have lost contact with the commander we do
             * not know how far it is safe to keep travelling. */
            s_ramp_vx = 0.0f;
            s_ramp_wz = 0.0f;
        }
        s_wd_tripped = true;
        want_vx = 0.0f;
        want_wz = 0.0f;
    } else {
        s_wd_tripped = false;
        want_vx = s_cmd_vx;
        want_wz = s_cmd_wz;
    }

    /* -------- slew limit ---------------------------------------------
     * A step change in /cmd_vel used to be delivered to the wheels as a
     * step change in duty, which on a stictioned chassis is a lurch. The
     * limits are runtime parameters so they can be relaxed once the
     * low-speed behaviour is trusted. */
    s_ramp_vx = slew(s_ramp_vx, want_vx, cfg->max_accel_mps2   * MOTOR_CONTROL_DT_S);
    s_ramp_wz = slew(s_ramp_wz, want_wz, cfg->max_alpha_radps2 * MOTOR_CONTROL_DT_S);

    /* -------- differential mixing ------------------------------------
     * REP-103 body frame: +x forward, +z up, +wz counter-clockwise.
     * The right wheels therefore speed up for a positive (CCW) yaw rate. */
    const float half_track = 0.5f * cfg->track_width_m;
    float v_left  = s_ramp_vx - s_ramp_wz * half_track;
    float v_right = s_ramp_vx + s_ramp_wz * half_track;

    /* Clamp per wheel, preserving the ratio between the two sides so a
     * saturating command curves the way it was asked to instead of
     * straightening out. */
    const float vmax = cfg->max_wheel_mps;
    const float peak = fmaxf(fabsf(v_left), fabsf(v_right));
    if (peak > vmax && peak > 0.0f) {
        const float k = vmax / peak;
        v_left  *= k;
        v_right *= k;
    }

    float target[MOTOR_COUNT];
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        target[i] = (k_side[i] == MOTOR_SIDE_LEFT) ? v_left : v_right;
    }
    motor_set_target_mps(target);

    motor_update();

    /* -------- reconstruct measured body velocity ----------------------
     * Averaged per side, then the standard differential inverse. */
    float spd[MOTOR_COUNT];
    motor_get_speed_mps(spd);

    float sum_l = 0.0f, sum_r = 0.0f;
    int   n_l = 0, n_r = 0;
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        if (k_side[i] == MOTOR_SIDE_LEFT) { sum_l += spd[i]; n_l++; }
        else                              { sum_r += spd[i]; n_r++; }
    }
    const float m_left  = (n_l > 0) ? (sum_l / (float)n_l) : 0.0f;
    const float m_right = (n_r > 0) ? (sum_r / (float)n_r) : 0.0f;

    s_meas_vx = 0.5f * (m_left + m_right);
    s_meas_wz = (cfg->track_width_m > 1e-4f)
              ? ((m_right - m_left) / cfg->track_width_m)
              : 0.0f;
}

void car_motion_get_measured(float *vx, float *wz)
{
    if (vx) *vx = s_meas_vx;
    if (wz) *wz = s_meas_wz;
}
