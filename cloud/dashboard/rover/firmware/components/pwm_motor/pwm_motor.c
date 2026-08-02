#include "pwm_motor.h"

#include "board_pins.h"
#include "rover_config.h"

#include "driver/mcpwm_prelude.h"
#include "esp_err.h"

/* ------------------------------------------------------------------ *
 * MCPWM topology
 *
 * ESP32-S3 has 2 MCPWM groups; each group has 3 timers and 3 operators,
 * and each operator drives 2 generators from 2 comparators. One motor
 * needs 2 outputs (A and B of its H-bridge), so one operator per motor
 * is an exact fit, and 4 motors need 4 timers => 3 in group 0, 1 in
 * group 1.
 *
 * (That 3+1 split is also what the third-party reference tree uses --
 * PWM_MOTOR_TIMER_GROUP_ID_M1..M4 = 0,0,0,1 -- which is how we know the
 * reference is MCPWM-based rather than LEDC-based. LEDC would not work
 * here anyway: the ESP32-S3 has only 8 LEDC channels and the two servo
 * outputs need two of them.)
 * ------------------------------------------------------------------ */

typedef struct {
    int group;
    int gpio_a;
    int gpio_b;
} motor_hw_desc_t;

static const motor_hw_desc_t k_hw[PWM_MOTOR_COUNT] = {
    { 0, PIN_M1A, PIN_M1B },
    { 0, PIN_M2A, PIN_M2B },
    { 0, PIN_M3A, PIN_M3B },
    { 1, PIN_M4A, PIN_M4B },
};

typedef struct {
    mcpwm_timer_handle_t  timer;
    mcpwm_oper_handle_t   oper;
    mcpwm_cmpr_handle_t   cmp_a;
    mcpwm_cmpr_handle_t   cmp_b;
    mcpwm_gen_handle_t    gen_a;
    mcpwm_gen_handle_t    gen_b;
    int32_t               last_duty;   /* signed, post-map */
} motor_hw_t;

static motor_hw_t s_hw[PWM_MOTOR_COUNT];
static bool       s_ready = false;

/* Generator force levels. -1 releases the force and lets the comparator
 * drive the pin; 0 pins it low; 1 pins it high.
 *
 * Forcing rather than writing a comparator value of 0 or PWM_FULL_SCALE
 * is deliberate. At compare==0 the "timer empty" and "compare" events
 * coincide within one tick and the resulting level is ambiguous, and
 * compare==period is at the edge of what the driver accepts. Forcing
 * makes 0% and 100% exact and glitch-free instead of nearly-exact. */
#define FORCE_RELEASE  (-1)
#define FORCE_LOW      (0)
#define FORCE_HIGH     (1)

static void gen_setup(mcpwm_gen_handle_t gen, mcpwm_cmpr_handle_t cmp)
{
    /* Rising edge at the start of every period, falling edge when the
     * counter reaches the comparator: duty = cmp / PWM_FULL_SCALE. */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       cmp,
                                       MCPWM_GEN_ACTION_LOW)));
}

void pwm_motor_init(void)
{
    for (int i = 0; i < PWM_MOTOR_COUNT; ++i) {
        motor_hw_t *m = &s_hw[i];

        mcpwm_timer_config_t tcfg = {
            .group_id      = k_hw[i].group,
            .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = PWM_TIMER_RESOLUTION_HZ,
            .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks  = PWM_FULL_SCALE,
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &m->timer));

        mcpwm_operator_config_t ocfg = { .group_id = k_hw[i].group };
        ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &m->oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(m->oper, m->timer));

        mcpwm_comparator_config_t ccfg = { .flags = { .update_cmp_on_tez = true } };
        ESP_ERROR_CHECK(mcpwm_new_comparator(m->oper, &ccfg, &m->cmp_a));
        ESP_ERROR_CHECK(mcpwm_new_comparator(m->oper, &ccfg, &m->cmp_b));

        mcpwm_generator_config_t ga = { .gen_gpio_num = k_hw[i].gpio_a };
        mcpwm_generator_config_t gb = { .gen_gpio_num = k_hw[i].gpio_b };
        ESP_ERROR_CHECK(mcpwm_new_generator(m->oper, &ga, &m->gen_a));
        ESP_ERROR_CHECK(mcpwm_new_generator(m->oper, &gb, &m->gen_b));

        gen_setup(m->gen_a, m->cmp_a);
        gen_setup(m->gen_b, m->cmp_b);

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->cmp_a, 0));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->cmp_b, 0));

        /* Park low before the timer ever runs, so nothing twitches at boot. */
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->gen_a, FORCE_LOW, false));
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->gen_b, FORCE_LOW, false));

        ESP_ERROR_CHECK(mcpwm_timer_enable(m->timer));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(m->timer, MCPWM_TIMER_START_NO_STOP));

        m->last_duty = 0;
    }
    s_ready = true;
}

/* ================================================================== *
 *  THE MAP
 * ================================================================== */
int32_t pwm_motor_map_min_pwm(int32_t request)
{
    const rover_config_t *cfg = rover_config_get();

    /* MIN_PWM and EPS are stored as percentages precisely so that they mean
     * something on their own. "200" meant nothing without also knowing that
     * full scale happened to be 400; "10.0%" is self-describing, and stays
     * correct if the carrier frequency or timebase ever changes. */
    int32_t min_pwm = (int32_t)((cfg->min_pwm_percent * (float)PWM_FULL_SCALE) / 100.0f + 0.5f);
    int32_t eps     = (int32_t)((cfg->pwm_eps_percent * (float)PWM_FULL_SCALE) / 100.0f + 0.5f);

    if (min_pwm < 0)               min_pwm = 0;
    if (min_pwm > PWM_FULL_SCALE)  min_pwm = PWM_FULL_SCALE;
    if (eps     < 0)               eps     = 0;
    /* EPS must leave a usable span above it, or the map degenerates. */
    if (eps >= PWM_FULL_SCALE)     eps     = PWM_FULL_SCALE - 1;

    const int32_t sign = (request < 0) ? -1 : 1;
    int32_t mag = (request < 0) ? -request : request;

    /* A real stop, not a very slow crawl. */
    if (mag <= eps) {
        return 0;
    }
    if (mag > PWM_FULL_SCALE) {
        mag = PWM_FULL_SCALE;
    }

    /* Proportional map of (eps, FULL] onto [min_pwm, FULL].
     * 64-bit intermediate: 400 * 400 fits easily in 32 bits, but the
     * expression stays correct if PWM_FULL_SCALE is ever raised. */
    const int32_t span_in  = PWM_FULL_SCALE - eps;       /* > 0 by the clamp above */
    const int32_t span_out = PWM_FULL_SCALE - min_pwm;   /* >= 0                   */
    const int64_t scaled   = ((int64_t)(mag - eps) * (int64_t)span_out) / (int64_t)span_in;

    int32_t duty = min_pwm + (int32_t)scaled;
    if (duty > PWM_FULL_SCALE) duty = PWM_FULL_SCALE;
    if (duty < 0)              duty = 0;

    return sign * duty;
}

static void drive(motor_hw_t *m, int32_t duty)
{
    /* duty is signed, already mapped, magnitude in [0, PWM_FULL_SCALE]. */
    const bool forward = (duty >= 0);
    int32_t mag = forward ? duty : -duty;

    mcpwm_gen_handle_t drv  = forward ? m->gen_a : m->gen_b;
    mcpwm_gen_handle_t idle = forward ? m->gen_b : m->gen_a;
    mcpwm_cmpr_handle_t cmp = forward ? m->cmp_a : m->cmp_b;

    /* The non-driving half of the bridge is always held low: sign-magnitude
     * drive, so the bridge is in drive/coast rather than drive/brake and the
     * duty-to-torque curve stays monotonic through zero. */
    (void)mcpwm_generator_set_force_level(idle, FORCE_LOW, false);

    if (mag == 0) {
        (void)mcpwm_generator_set_force_level(drv, FORCE_LOW, false);
    } else if (mag >= PWM_FULL_SCALE) {
        (void)mcpwm_generator_set_force_level(drv, FORCE_HIGH, false);
    } else {
        (void)mcpwm_comparator_set_compare_value(cmp, (uint32_t)mag);
        (void)mcpwm_generator_set_force_level(drv, FORCE_RELEASE, false);
    }
}

void pwm_motor_set(pwm_motor_id_t id, int32_t request)
{
    if (!s_ready || id < 0 || id >= PWM_MOTOR_COUNT) {
        return;
    }
    if (request > PWM_FULL_SCALE)  request = PWM_FULL_SCALE;
    if (request < -PWM_FULL_SCALE) request = -PWM_FULL_SCALE;

    const int32_t duty = pwm_motor_map_min_pwm(request);
    s_hw[id].last_duty = duty;
    drive(&s_hw[id], duty);
}

void pwm_motor_stop(pwm_motor_id_t id, bool brake)
{
    if (!s_ready || id < 0 || id >= PWM_MOTOR_COUNT) {
        return;
    }
    motor_hw_t *m = &s_hw[id];
    const int lvl = brake ? FORCE_HIGH : FORCE_LOW;
    (void)mcpwm_generator_set_force_level(m->gen_a, lvl, false);
    (void)mcpwm_generator_set_force_level(m->gen_b, lvl, false);
    m->last_duty = 0;
}

void pwm_motor_stop_all(bool brake)
{
    for (int i = 0; i < PWM_MOTOR_COUNT; ++i) {
        pwm_motor_stop((pwm_motor_id_t)i, brake);
    }
}

int32_t pwm_motor_last_duty(pwm_motor_id_t id)
{
    if (id < 0 || id >= PWM_MOTOR_COUNT) {
        return 0;
    }
    return s_hw[id].last_duty;
}
