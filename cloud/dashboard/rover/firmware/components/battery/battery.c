#include "battery.h"

#include "board_pins.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "rover_config.h"

/* GPIO3 is ADC1 channel 2 on the ESP32-S3. */
#define BAT_ADC_UNIT      ADC_UNIT_1
#define BAT_ADC_CHANNEL   ADC_CHANNEL_2

/* Widest input range, ~0..3.1 V. This enumerator was renamed from
 * ADC_ATTEN_DB_11 in ESP-IDF v5.2; the project requires >= 5.2 (see
 * main/idf_component.yml), so the new name is the correct one and the
 * old one would only produce a deprecation warning. */
#define BAT_ADC_ATTEN     ADC_ATTEN_DB_12

#define BAT_OVERSAMPLE    16
#define BAT_FILTER_ALPHA  0.20f

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;
static bool                      s_ready;
static bool                      s_calibrated;
static float                     s_volts;

void battery_init(void)
{
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = BAT_ADC_UNIT };
    if (adc_oneshot_new_unit(&ucfg, &s_adc) != ESP_OK) {
        return;
    }

    adc_oneshot_chan_cfg_t ccfg = {
        .atten    = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(s_adc, BAT_ADC_CHANNEL, &ccfg) != ESP_OK) {
        return;
    }

    /* Curve fitting is the only scheme the ESP32-S3 supports. Without it
     * the raw code would have to be scaled by a nominal Vref, which is
     * good to maybe +/-10% -- worth having, not worth failing over. */
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cal = {
        .unit_id  = BAT_ADC_UNIT,
        .chan     = BAT_ADC_CHANNEL,
        .atten    = BAT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    s_calibrated = (adc_cali_create_scheme_curve_fitting(&cal, &s_cali) == ESP_OK);
#endif

    s_ready = true;

    /* Prime the filter so the first published value is real rather than a
     * ramp up from zero, which downstream would read as a flat battery. */
    for (int i = 0; i < 8; ++i) {
        battery_sample();
    }
}

void battery_sample(void)
{
    if (!s_ready) {
        return;
    }

    int32_t acc = 0;
    int     n   = 0;
    for (int i = 0; i < BAT_OVERSAMPLE; ++i) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, BAT_ADC_CHANNEL, &raw) == ESP_OK) {
            acc += raw;
            n++;
        }
    }
    if (n == 0) {
        return;
    }
    const int mean_raw = (int)(acc / n);

    int mv = 0;
    if (s_calibrated) {
        if (adc_cali_raw_to_voltage(s_cali, mean_raw, &mv) != ESP_OK) {
            return;
        }
    } else {
        /* 12-bit code against a nominal 3100 mV full scale at 11 dB. */
        mv = (mean_raw * 3100) / 4095;
    }

    const rover_config_t *cfg = rover_config_get();
    const float v = ((float)mv / 1000.0f) * cfg->bat_divider * cfg->bat_scale;

    if (s_volts == 0.0f) {
        s_volts = v;
    } else {
        s_volts += BAT_FILTER_ALPHA * (v - s_volts);
    }
}

float battery_volts(void) { return s_volts; }
