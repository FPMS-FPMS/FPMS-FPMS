#include "rover_config.h"

#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"

#define NVS_NAMESPACE   "rovercfg"
#define NVS_BLOB_KEY    "cfg"

/* Bump whenever the layout of rover_config_t changes, so a stale blob from
 * an older firmware is ignored rather than reinterpreted as garbage. */
#define CFG_BLOB_MAGIC   0x52435601u   /* 'RCV' + version 1 */

typedef struct {
    uint32_t       magic;
    rover_config_t cfg;
} cfg_blob_t;

static rover_config_t s_cfg;
static float          s_counts_per_metre = 1.0f;
static float          s_metres_per_count = 1.0f;

static void load_defaults(rover_config_t *c)
{
    c->min_pwm_percent    = CFG_DEF_MIN_PWM_PERCENT;
    c->pwm_eps_percent    = CFG_DEF_PWM_EPS_PERCENT;
    c->pid_kp             = CFG_DEF_PID_KP;
    c->pid_ki             = CFG_DEF_PID_KI;
    c->pid_kd             = CFG_DEF_PID_KD;
    c->enc_counts_per_rev = CFG_DEF_ENC_COUNTS_PER_REV;
    c->wheel_circum_mm    = CFG_DEF_WHEEL_CIRCUM_MM;
    c->track_width_m      = CFG_DEF_TRACK_WIDTH_M;
    c->max_wheel_mps      = CFG_DEF_MAX_WHEEL_MPS;
    c->max_accel_mps2     = CFG_DEF_MAX_ACCEL_MPS2;
    c->max_alpha_radps2   = CFG_DEF_MAX_ALPHA_RADPS2;
    c->cmd_timeout_ms     = CFG_DEF_CMD_TIMEOUT_MS;
    c->bat_divider        = CFG_DEF_BAT_DIVIDER;
    c->bat_scale          = CFG_DEF_BAT_SCALE;
    c->invert_left        = CFG_DEF_INVERT_LEFT;
    c->invert_right       = CFG_DEF_INVERT_RIGHT;
    c->invert_enc_left    = CFG_DEF_INVERT_ENC_LEFT;
    c->invert_enc_right   = CFG_DEF_INVERT_ENC_RIGHT;
}

/* Clamp every field into a range in which the firmware cannot hurt itself.
 * A ROS parameter is a remote, unauthenticated write into a motor
 * controller; treat it as hostile input. */
static void sanitise(rover_config_t *c)
{
    if (c->min_pwm_percent < 0.0f)                    c->min_pwm_percent = 0.0f;
    if (c->min_pwm_percent > CFG_MAX_MIN_PWM_PERCENT) c->min_pwm_percent = CFG_MAX_MIN_PWM_PERCENT;

    if (c->pwm_eps_percent < 0.0f)  c->pwm_eps_percent = 0.0f;
    if (c->pwm_eps_percent > 10.0f) c->pwm_eps_percent = 10.0f;

    if (c->pid_kp < 0.0f)   c->pid_kp = 0.0f;
    if (c->pid_kp > 100.0f) c->pid_kp = 100.0f;
    if (c->pid_ki < 0.0f)   c->pid_ki = 0.0f;
    if (c->pid_ki > 100.0f) c->pid_ki = 100.0f;
    if (c->pid_kd < 0.0f)   c->pid_kd = 0.0f;
    if (c->pid_kd > 100.0f) c->pid_kd = 100.0f;

    if (c->enc_counts_per_rev < 1.0f)     c->enc_counts_per_rev = 1.0f;
    if (c->enc_counts_per_rev > 100000.0f) c->enc_counts_per_rev = 100000.0f;

    if (c->wheel_circum_mm < 1.0f)    c->wheel_circum_mm = 1.0f;
    if (c->wheel_circum_mm > 5000.0f) c->wheel_circum_mm = 5000.0f;

    if (c->track_width_m < 0.01f) c->track_width_m = 0.01f;
    if (c->track_width_m > 5.0f)  c->track_width_m = 5.0f;

    if (c->max_wheel_mps < 0.05f) c->max_wheel_mps = 0.05f;
    if (c->max_wheel_mps > 10.0f) c->max_wheel_mps = 10.0f;

    if (c->max_accel_mps2 < 0.01f)  c->max_accel_mps2 = 0.01f;
    if (c->max_accel_mps2 > 50.0f)  c->max_accel_mps2 = 50.0f;
    if (c->max_alpha_radps2 < 0.01f) c->max_alpha_radps2 = 0.01f;
    if (c->max_alpha_radps2 > 200.0f) c->max_alpha_radps2 = 200.0f;

    /* A watchdog you can disable is not a watchdog. Floor at 50 ms so a
     * fat-fingered `ros2 param set cmd_timeout_ms 0` cannot leave the
     * motors latched on, and cap at 2 s so it stays a safety device. */
    if (c->cmd_timeout_ms < 50)   c->cmd_timeout_ms = 50;
    if (c->cmd_timeout_ms > 2000) c->cmd_timeout_ms = 2000;

    if (c->bat_divider < 1.0f)  c->bat_divider = 1.0f;
    if (c->bat_divider > 50.0f) c->bat_divider = 50.0f;
    if (c->bat_scale < 0.1f)    c->bat_scale = 0.1f;
    if (c->bat_scale > 10.0f)   c->bat_scale = 10.0f;
}

void rover_config_commit(void)
{
    sanitise(&s_cfg);
    /* counts per metre = counts/rev / (circumference in metres) */
    const float circ_m = s_cfg.wheel_circum_mm / 1000.0f;
    s_counts_per_metre = s_cfg.enc_counts_per_rev / circ_m;
    s_metres_per_count = 1.0f / s_counts_per_metre;
}

void rover_config_init(void)
{
    load_defaults(&s_cfg);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        (void)nvs_flash_erase();
        err = nvs_flash_init();
    }

    if (err == ESP_OK) {
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
            cfg_blob_t blob;
            size_t len = sizeof(blob);
            if (nvs_get_blob(h, NVS_BLOB_KEY, &blob, &len) == ESP_OK &&
                len == sizeof(blob) && blob.magic == CFG_BLOB_MAGIC) {
                s_cfg = blob.cfg;
            }
            nvs_close(h);
        }
    }

    rover_config_commit();
}

const rover_config_t *rover_config_get(void) { return &s_cfg; }
rover_config_t       *rover_config_mut(void) { return &s_cfg; }

bool rover_config_save(void)
{
    rover_config_commit();

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }

    cfg_blob_t blob;
    memset(&blob, 0, sizeof(blob));
    blob.magic = CFG_BLOB_MAGIC;
    blob.cfg   = s_cfg;

    bool ok = (nvs_set_blob(h, NVS_BLOB_KEY, &blob, sizeof(blob)) == ESP_OK);
    if (ok) {
        ok = (nvs_commit(h) == ESP_OK);
    }
    nvs_close(h);
    return ok;
}

float rover_config_counts_per_metre(void) { return s_counts_per_metre; }
float rover_config_metres_per_count(void) { return s_metres_per_count; }
