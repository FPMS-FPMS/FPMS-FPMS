#include "uros_params.h"

#include <string.h>

#include "car_motion.h"
#include "encoder.h"
#include "rover_config.h"

/* Parameter names. Kept identical to the rover_config_t field names
 * wherever possible so `ros2 param list` reads like the header. */
#define P_MIN_PWM       "min_pwm_percent"
#define P_PWM_EPS       "pwm_eps_percent"
#define P_KP            "pid_kp"
#define P_KI            "pid_ki"
#define P_KD            "pid_kd"
#define P_ENC_CPR       "enc_counts_per_rev"
#define P_WHEEL_MM      "wheel_circum_mm"
#define P_TRACK_M       "track_width_m"
#define P_MAX_MPS       "max_wheel_mps"
#define P_MAX_ACC       "max_accel_mps2"
#define P_MAX_ALPHA     "max_alpha_radps2"
#define P_CMD_TIMEOUT   "cmd_timeout_ms"
#define P_BAT_DIV       "bat_divider"
#define P_BAT_SCALE     "bat_scale"
#define P_INV_L         "invert_left"
#define P_INV_R         "invert_right"
#define P_INV_ENC_L     "invert_enc_left"
#define P_INV_ENC_R     "invert_enc_right"
#define P_SAVE          "save_to_nvs"
#define P_ENC1          "enc1"
#define P_ENC2          "enc2"
#define P_ENC3          "enc3"
#define P_ENC4          "enc4"

static volatile bool s_save_requested = false;

/* ------------------------------------------------------------------ *
 * Change callback. Returning false rejects the write.
 *
 * Values are copied straight into rover_config and then clamped by
 * rover_config_commit(); the clamp is the validation, and it lives in one
 * place so a parameter can never bypass it.
 * ------------------------------------------------------------------ */
static bool on_param_change(const Parameter *old_param,
                            const Parameter *new_param,
                            void *context)
{
    (void)old_param;
    (void)context;

    if (new_param == NULL || new_param->name.data == NULL) {
        /* Parameter deletion. Nothing here is deletable in normal use. */
        return false;
    }

    const char     *n   = new_param->name.data;
    rover_config_t *cfg = rover_config_mut();

    const double  d = new_param->value.double_value;
    const int64_t i = new_param->value.integer_value;
    const bool    b = new_param->value.bool_value;

    if      (strcmp(n, P_MIN_PWM)     == 0) cfg->min_pwm_percent    = (float)d;
    else if (strcmp(n, P_PWM_EPS)     == 0) cfg->pwm_eps_percent    = (float)d;
    else if (strcmp(n, P_KP)          == 0) cfg->pid_kp             = (float)d;
    else if (strcmp(n, P_KI)          == 0) cfg->pid_ki             = (float)d;
    else if (strcmp(n, P_KD)          == 0) cfg->pid_kd             = (float)d;
    else if (strcmp(n, P_ENC_CPR)     == 0) cfg->enc_counts_per_rev = (float)d;
    else if (strcmp(n, P_WHEEL_MM)    == 0) cfg->wheel_circum_mm    = (float)d;
    else if (strcmp(n, P_TRACK_M)     == 0) cfg->track_width_m      = (float)d;
    else if (strcmp(n, P_MAX_MPS)     == 0) cfg->max_wheel_mps      = (float)d;
    else if (strcmp(n, P_MAX_ACC)     == 0) cfg->max_accel_mps2     = (float)d;
    else if (strcmp(n, P_MAX_ALPHA)   == 0) cfg->max_alpha_radps2   = (float)d;
    else if (strcmp(n, P_BAT_DIV)     == 0) cfg->bat_divider        = (float)d;
    else if (strcmp(n, P_BAT_SCALE)   == 0) cfg->bat_scale          = (float)d;
    else if (strcmp(n, P_CMD_TIMEOUT) == 0) cfg->cmd_timeout_ms     = (int32_t)i;
    else if (strcmp(n, P_INV_L)       == 0) cfg->invert_left        = b;
    else if (strcmp(n, P_INV_R)       == 0) cfg->invert_right       = b;
    else if (strcmp(n, P_INV_ENC_L)   == 0) cfg->invert_enc_left    = b;
    else if (strcmp(n, P_INV_ENC_R)   == 0) cfg->invert_enc_right   = b;
    else if (strcmp(n, P_SAVE)        == 0) {
        /* Deferred: writing NVS takes tens of milliseconds and this
         * callback runs inside the micro-ROS executor. The 1 Hz timer
         * performs the save and clears the flag. */
        if (b) s_save_requested = true;
        return true;
    }
    else if (strncmp(n, "enc", 3) == 0) {
        /* enc1..enc4 are outputs. Accept the write so the server does not
         * error, but ignore it: the next export overwrites it anyway. */
        return true;
    }
    else {
        return false;   /* unknown parameter */
    }

    rover_config_commit();
    car_motion_apply_polarity();
    return true;
}

/* ------------------------------------------------------------------ */

#define ADD_DOUBLE(srv, name, val)                                  \
    do {                                                            \
        rcl_ret_t _r = rclc_add_parameter((srv), (name), RCLC_PARAMETER_DOUBLE); \
        if (_r != RCL_RET_OK) return _r;                             \
        _r = rclc_parameter_set_double((srv), (name), (double)(val));\
        if (_r != RCL_RET_OK) return _r;                             \
    } while (0)

#define ADD_INT(srv, name, val)                                     \
    do {                                                            \
        rcl_ret_t _r = rclc_add_parameter((srv), (name), RCLC_PARAMETER_INT); \
        if (_r != RCL_RET_OK) return _r;                             \
        _r = rclc_parameter_set_int((srv), (name), (int64_t)(val));  \
        if (_r != RCL_RET_OK) return _r;                             \
    } while (0)

#define ADD_BOOL(srv, name, val)                                    \
    do {                                                            \
        rcl_ret_t _r = rclc_add_parameter((srv), (name), RCLC_PARAMETER_BOOL); \
        if (_r != RCL_RET_OK) return _r;                             \
        _r = rclc_parameter_set_bool((srv), (name), (bool)(val));    \
        if (_r != RCL_RET_OK) return _r;                             \
    } while (0)

rcl_ret_t uros_params_init(rclc_parameter_server_t *server,
                           rcl_node_t *node,
                           rclc_executor_t *executor)
{
    const rclc_parameter_options_t opts = {
        /* Nothing on this rover consumes /parameter_events, and the
         * encoder export below writes four parameters every second --
         * publishing that would be pure noise on a 921600 baud link. */
        .notify_changed_over_dds   = false,
        .max_params                = 24,
        .allow_undeclared_parameters = false,
        /* low_mem_mode would drop descriptions and constrain list
         * requests; there is room for the full server here, and the
         * descriptions are worth having when the person calibrating is
         * not the person who wrote this. */
        .low_mem_mode              = false,
    };

    rcl_ret_t rc = rclc_parameter_server_init_with_option(server, node, &opts);
    if (rc != RCL_RET_OK) {
        return rc;
    }

    const rover_config_t *cfg = rover_config_get();

    ADD_DOUBLE(server, P_MIN_PWM,     cfg->min_pwm_percent);
    ADD_DOUBLE(server, P_PWM_EPS,     cfg->pwm_eps_percent);
    ADD_DOUBLE(server, P_KP,          cfg->pid_kp);
    ADD_DOUBLE(server, P_KI,          cfg->pid_ki);
    ADD_DOUBLE(server, P_KD,          cfg->pid_kd);
    ADD_DOUBLE(server, P_ENC_CPR,     cfg->enc_counts_per_rev);
    ADD_DOUBLE(server, P_WHEEL_MM,    cfg->wheel_circum_mm);
    ADD_DOUBLE(server, P_TRACK_M,     cfg->track_width_m);
    ADD_DOUBLE(server, P_MAX_MPS,     cfg->max_wheel_mps);
    ADD_DOUBLE(server, P_MAX_ACC,     cfg->max_accel_mps2);
    ADD_DOUBLE(server, P_MAX_ALPHA,   cfg->max_alpha_radps2);
    ADD_DOUBLE(server, P_BAT_DIV,     cfg->bat_divider);
    ADD_DOUBLE(server, P_BAT_SCALE,   cfg->bat_scale);
    ADD_INT   (server, P_CMD_TIMEOUT, cfg->cmd_timeout_ms);
    ADD_BOOL  (server, P_INV_L,       cfg->invert_left);
    ADD_BOOL  (server, P_INV_R,       cfg->invert_right);
    ADD_BOOL  (server, P_INV_ENC_L,   cfg->invert_enc_left);
    ADD_BOOL  (server, P_INV_ENC_R,   cfg->invert_enc_right);
    ADD_BOOL  (server, P_SAVE,        false);
    ADD_INT   (server, P_ENC1,        0);
    ADD_INT   (server, P_ENC2,        0);
    ADD_INT   (server, P_ENC3,        0);
    ADD_INT   (server, P_ENC4,        0);

    /* Descriptions are cheap and this is the file someone will read at
     * 2am with a rover on blocks. Failures are ignored on purpose: a
     * missing description must never stop the node coming up. */
    (void)rclc_add_parameter_description(server, P_MIN_PWM,
        "THE FIX. Minimum duty (percent of full scale) a non-zero request maps to. "
        "Replaces the vendor's additive 50 percent dead zone. Tune DOWN, not up.",
        "0..40");
    (void)rclc_add_parameter_description(server, P_ENC_CPR,
        "Encoder counts per wheel revolution (x4 decoding). MEASURE ME.", "");
    (void)rclc_add_parameter_description(server, P_WHEEL_MM,
        "Wheel circumference in mm. MEASURE ME.", "");
    (void)rclc_add_parameter_description(server, P_TRACK_M,
        "EFFECTIVE skid-steer track in metres, not the ruler distance. MEASURE ME.", "");
    (void)rclc_add_parameter_description(server, P_SAVE,
        "Set true to persist the current parameters to NVS. Self-clearing.", "");
    (void)rclc_add_parameter_description(server, P_ENC1,
        "Cumulative encoder counts, motor 1. Output only; updated at 1 Hz.", "");

    (void)rclc_add_parameter_constraint_double(server, P_MIN_PWM, 0.0, 40.0, 0.0);
    (void)rclc_add_parameter_constraint_integer(server, P_CMD_TIMEOUT, 50, 2000, 0);

    return rclc_executor_add_parameter_server(executor, server, on_param_change);
}

void uros_params_fini(rclc_parameter_server_t *server, rcl_node_t *node)
{
    (void)rclc_parameter_server_fini(server, node);
}

void uros_params_export_encoders(rclc_parameter_server_t *server)
{
    if (s_save_requested) {
        s_save_requested = false;
        (void)rover_config_save();
        (void)rclc_parameter_set_bool(server, P_SAVE, false);
    }

    (void)rclc_parameter_set_int(server, P_ENC1, encoder_total(0));
    (void)rclc_parameter_set_int(server, P_ENC2, encoder_total(1));
    (void)rclc_parameter_set_int(server, P_ENC3, encoder_total(2));
    (void)rclc_parameter_set_int(server, P_ENC4, encoder_total(3));
}
