#include "encoder.h"

#include "board_pins.h"

#include "driver/pulse_cnt.h"
#include "esp_err.h"

/* The counter is read and cleared every control period (10 ms). At the
 * chassis's measured top speed (~1.3 m/s, ~6 wheel rev/s, ~6300 counts/s)
 * that is ~63 counts per period, so a +/-30000 range is three orders of
 * magnitude of headroom and the limits can never be reached in normal
 * operation. They exist only so a wiring fault cannot wrap the counter. */
#define PCNT_HIGH_LIMIT   30000
#define PCNT_LOW_LIMIT   (-30000)

/* 1 us of glitch rejection. The encoders on this gearbox top out around
 * 6.3 kHz per channel (~160 us per edge pair), so 1 us removes contact
 * bounce and switching noise with a 100x margin on real edges. */
#define PCNT_GLITCH_NS    1000

typedef struct {
    int gpio_a;
    int gpio_b;
} enc_pins_t;

static const enc_pins_t k_pins[ENCODER_COUNT] = {
    { PIN_H1A, PIN_H1B },
    { PIN_H2A, PIN_H2B },
    { PIN_H3A, PIN_H3B },
    { PIN_H4A, PIN_H4B },
};

static pcnt_unit_handle_t s_unit[ENCODER_COUNT];
static int64_t            s_total[ENCODER_COUNT];
static bool               s_ready;

void encoder_init(void)
{
    for (int i = 0; i < ENCODER_COUNT; ++i) {
        pcnt_unit_config_t ucfg = {
            .high_limit = PCNT_HIGH_LIMIT,
            .low_limit  = PCNT_LOW_LIMIT,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &s_unit[i]));

        pcnt_glitch_filter_config_t fcfg = { .max_glitch_ns = PCNT_GLITCH_NS };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_unit[i], &fcfg));

        /* Channel A counts edges on the A wire, using B as the level input;
         * channel B does the mirror image. Counting both edges of both wires
         * is what makes this x4 rather than x1 or x2. */
        pcnt_chan_config_t ca = {
            .edge_gpio_num  = k_pins[i].gpio_a,
            .level_gpio_num = k_pins[i].gpio_b,
        };
        pcnt_chan_config_t cb = {
            .edge_gpio_num  = k_pins[i].gpio_b,
            .level_gpio_num = k_pins[i].gpio_a,
        };
        pcnt_channel_handle_t ch_a = NULL;
        pcnt_channel_handle_t ch_b = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(s_unit[i], &ca, &ch_a));
        ESP_ERROR_CHECK(pcnt_new_channel(s_unit[i], &cb, &ch_b));

        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
            ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(
            ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
            ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(
            ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

        ESP_ERROR_CHECK(pcnt_unit_enable(s_unit[i]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit[i]));
        ESP_ERROR_CHECK(pcnt_unit_start(s_unit[i]));

        s_total[i] = 0;
    }
    s_ready = true;
}

void encoder_sample(int32_t *delta)
{
    for (int i = 0; i < ENCODER_COUNT; ++i) {
        int raw = 0;
        if (s_ready) {
            /* Read-then-clear. The race window between the two calls is a
             * few microseconds against a signal whose fastest edge spacing
             * is ~160 us, so at most one count can be missed and only at
             * full speed. Watch points with an accumulator would close it
             * completely, at the cost of an ISR per 30000 counts; the trade
             * is not worth it here. */
            (void)pcnt_unit_get_count(s_unit[i], &raw);
            (void)pcnt_unit_clear_count(s_unit[i]);
        }
        delta[i]   = (int32_t)raw;
        s_total[i] += (int64_t)raw;
    }
}

int64_t encoder_total(int idx)
{
    if (idx < 0 || idx >= ENCODER_COUNT) {
        return 0;
    }
    return s_total[idx];
}
