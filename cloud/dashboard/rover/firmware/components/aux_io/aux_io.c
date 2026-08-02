#include "aux_io.h"

#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"

/* The four drive motors take all of MCPWM's convenient timers, so the two
 * servos run on LEDC instead. They need 50 Hz and the motors need 25 kHz,
 * which could not share a timer in any case. */
#define SERVO_LEDC_MODE       LEDC_LOW_SPEED_MODE   /* the only mode on ESP32-S3 */
#define SERVO_LEDC_TIMER      LEDC_TIMER_0
#define SERVO_LEDC_RES        LEDC_TIMER_14_BIT     /* 16384 steps                */
#define SERVO_LEDC_FREQ_HZ    50                    /* 20 ms frame                */
#define SERVO_FULL_SCALE      16384

/* Standard hobby-servo pulse band. 500..2500 us is the wide interpretation;
 * the host already insets its own commands to 10..170 degrees, so the
 * extremes here are never reached in normal operation. */
#define SERVO_PULSE_MIN_US    500
#define SERVO_PULSE_MAX_US    2500
#define SERVO_FRAME_US        20000

static const ledc_channel_t k_servo_ch[2] = { LEDC_CHANNEL_0, LEDC_CHANNEL_1 };
static const int            k_servo_pin[2] = { PIN_SERVO_S1, PIN_SERVO_S2 };

static esp_timer_handle_t s_beep_timer;
static bool               s_ready;

static void beep_off_cb(void *arg)
{
    (void)arg;
    gpio_set_level(PIN_BEEP, 0);
}

void aux_io_init(void)
{
    /* ---- buzzer: active, high = sound ---- */
    const gpio_config_t bcfg = {
        .pin_bit_mask = 1ULL << PIN_BEEP,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    (void)gpio_config(&bcfg);
    gpio_set_level(PIN_BEEP, 0);

    const esp_timer_create_args_t targs = {
        .callback = beep_off_cb,
        .name     = "beep_off",
    };
    (void)esp_timer_create(&targs, &s_beep_timer);

    /* ---- servos ---- */
    const ledc_timer_config_t tcfg = {
        .speed_mode      = SERVO_LEDC_MODE,
        .timer_num       = SERVO_LEDC_TIMER,
        .duty_resolution = SERVO_LEDC_RES,
        .freq_hz         = SERVO_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&tcfg) != ESP_OK) {
        return;
    }

    for (int i = 0; i < 2; ++i) {
        const ledc_channel_config_t ccfg = {
            .gpio_num   = k_servo_pin[i],
            .speed_mode = SERVO_LEDC_MODE,
            .channel    = k_servo_ch[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = SERVO_LEDC_TIMER,
            /* Start with zero duty, i.e. no pulse train at all, so an
             * unpowered or unattached servo is not commanded anywhere on
             * boot. The first /servo_sN message starts driving it. */
            .duty       = 0,
            .hpoint     = 0,
        };
        (void)ledc_channel_config(&ccfg);
    }

    s_ready = true;
}

void aux_beep_command(uint16_t value)
{
    if (!s_ready) {
        return;
    }

    if (s_beep_timer) {
        (void)esp_timer_stop(s_beep_timer);
    }

    if (value == 0) {
        gpio_set_level(PIN_BEEP, 0);
        return;
    }

    gpio_set_level(PIN_BEEP, 1);

    if (value == 1) {
        return;                     /* latch on */
    }

    uint32_t ms = (value < 10) ? 10u : (uint32_t)value;
    if (s_beep_timer) {
        (void)esp_timer_start_once(s_beep_timer, (uint64_t)ms * 1000ULL);
    }
}

void aux_servo_set_deg(int channel, int32_t degrees)
{
    if (!s_ready || channel < 1 || channel > 2) {
        return;
    }
    if (degrees < SERVO_MIN_DEG) degrees = SERVO_MIN_DEG;
    if (degrees > SERVO_MAX_DEG) degrees = SERVO_MAX_DEG;

    const int32_t span_us = SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US;
    const int32_t pulse_us = SERVO_PULSE_MIN_US
                           + (degrees * span_us) / (SERVO_MAX_DEG - SERVO_MIN_DEG);

    const uint32_t duty = (uint32_t)(((int64_t)pulse_us * SERVO_FULL_SCALE) / SERVO_FRAME_US);

    const ledc_channel_t ch = k_servo_ch[channel - 1];
    (void)ledc_set_duty(SERVO_LEDC_MODE, ch, duty);
    (void)ledc_update_duty(SERVO_LEDC_MODE, ch);
}
