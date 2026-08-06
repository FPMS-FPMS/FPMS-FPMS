#include "pwm_motor.h"

#include "board_pins.h"
#include "rover_config.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

/* ================================================================== *
 *  WHY LEDC AND NOT MCPWM
 *
 *  This output stage used to be MCPWM. That topology was INFERRED from a
 *  third-party ESP-IDF tree, never from a schematic. On hardware it gave
 *  a clean boot, healthy /odom_raw, /imu and /battery topics, live ROS
 *  parameters -- and four motors that made no sound and moved 0.0 mm at
 *  every duty from 8% to 36%.
 *
 *  The evidence for LEDC is direct, not inferred. The operator's own
 *  Arduino firmware for THIS board (recovered from this machine's build
 *  cache -- see research/R6_LOCAL_ASSETS.md) provably drove these motors,
 *  and it is LEDC:
 *
 *      #define PWM_FREQ 25000
 *      #define PWM_BITS 8            // 0..255
 *      void setMotor(int pinA,int pinB,int val,bool leftSide){
 *        val=constrain(val,-PWM_MAX,PWM_MAX);
 *        if     (val>0){ledcWrite(fwd,val); ledcWrite(rev,0);}
 *        else if(val<0){ledcWrite(fwd,0);   ledcWrite(rev,-val);}
 *        else          {ledcWrite(fwd,0);   ledcWrite(rev,0);}
 *      }
 *
 *  Its link map contains ledcAttach / ledcWrite / ledc_channel_config and
 *  ZERO mcpwm symbols. Same GPIOs as this firmware already uses, so the
 *  pin map was never in question -- only the peripheral was.
 *
 * ================================================================== *
 *  THE CHANNEL BUDGET -- READ THIS BEFORE "FIXING" THE ASSIGNMENT
 *
 *  ESP32-S3 has EIGHT LEDC channels, all in the low-speed group (there is
 *  no high-speed group on the S3), and four timers.
 *  soc/esp32s3/include/soc/soc_caps.h:  SOC_LEDC_CHANNEL_NUM (8).
 *
 *  Two of those eight are ALREADY SPOKEN FOR. aux_io.c owns
 *  LEDC_CHANNEL_0 (GPIO8, servo header S1 -- which on this rover is
 *  wired to the SPRAYER) and LEDC_CHANNEL_1 (GPIO21, servo header S2),
 *  on LEDC_TIMER_0 at 50 Hz. aux_io_init() runs BEFORE this file's
 *  init (main.c: aux_io_init() then car_motion_init() -> motor_init()
 *  -> pwm_motor_init()).
 *
 *  So "one LEDC channel per half-bridge pin" would need 8 + 2 = 10
 *  channels on a part that has 8. It is not tight, it is impossible.
 *
 *  And taking channels 0 and 1 anyway would be WORSE THAN A BUILD ERROR.
 *  Re-configuring channel 0 for a motor GPIO re-points that channel's
 *  output signal, but it does NOT un-route GPIO8: the GPIO matrix still
 *  carries LEDC_LS_SIG_OUT0 to GPIO8 from aux_io's earlier
 *  ledc_channel_config(). The sprayer would then receive motor M1A's
 *  25 kHz drive waveform and fire whenever motor 1 turned.
 *
 *  WHAT THIS FILE DOES INSTEAD -- one channel per MOTOR, not per PIN.
 *
 *  A sign-magnitude H-bridge only ever needs PWM on ONE of its two
 *  inputs; the other is held low. That is exactly what the proven
 *  Arduino driver above does: ledcWrite(rev, 0) is a static low, no
 *  different from a GPIO held at 0. So each motor owns ONE LEDC channel,
 *  which is re-routed between its A and B pin when the sign of the
 *  request changes, and the non-driving pin is a plain GPIO output at 0.
 *
 *  The waveform on all eight pins is bit-for-bit what the Arduino driver
 *  produced. Four channels, not eight, and channels 0/1 plus timer 0 are
 *  left untouched for the servos. Channels 6 and 7 and timers 2 and 3
 *  stay free.
 *
 *  If the two servo outputs are ever deleted from aux_io.c, this file can
 *  be simplified to eight statically-bound channels; until then it cannot.
 *
 * ================================================================== *
 *  TIMER ASSIGNMENT
 *
 *    LEDC_TIMER_0  50 Hz, 14-bit  -- aux_io.c, servos S1/S2. NOT OURS.
 *    LEDC_TIMER_1  25 kHz, 10-bit -- ALL FOUR motors.
 *    LEDC_TIMER_2  free
 *    LEDC_TIMER_3  free
 *
 *  All four motors share ONE timer deliberately. They run at identical
 *  frequency and identical resolution, so a timer each would buy nothing
 *  and burn three of the four; and a shared timer means the four carriers
 *  are derived from one counter and cannot slowly beat against each other
 *  on the shared battery rail. hpoint is 0 on every channel, so the four
 *  bridges switch together -- the same phase relationship the MCPWM
 *  version had, so nothing about the supply loading changes.
 *
 * ================================================================== *
 *  RESOLUTION: 400 COMMAND TICKS -> 1024 LEDC TICKS
 *
 *  PWM_FULL_SCALE stays 400. It is public API: motor.c clamps the PID
 *  output to +/-PWM_FULL_SCALE and derives its feed-forward gain from it,
 *  pwm_motor_last_duty() reports in it, and every number in
 *  research/R4_FIRMWARE.md is expressed in it. Changing it would silently
 *  re-scale the velocity loop. It does not change.
 *
 *  LEDC resolution is a bit depth, so 400 is not expressible. We use
 *  10 bits = 1024 duty ticks and convert at the very last step:
 *
 *      ledc_ticks = round(mag * 1024 / 400)          (mag in [0, 400])
 *
 *  1024 > 400, so all 400 distinct command levels survive as 400 distinct
 *  duties -- the conversion adds resolution, it never removes any.
 *
 *  min_pwm_percent and pwm_eps_percent therefore mean EXACTLY what they
 *  meant before: percentages of full scale, applied by
 *  pwm_motor_map_min_pwm() on the unchanged 400-tick scale, before this
 *  conversion. min_pwm_percent = 36.0 gives 144/400 of command scale ->
 *  369/1024 of duty = 36.03%. The map itself is byte-for-byte the code
 *  that was there before.
 *
 *  Why 10 bits and not 11 (the maximum at 25 kHz)? The LEDC divider is
 *  8.8 fixed point and must be >= 1.0:
 *      10-bit: APB 80 MHz -> 3.125  OK  |  XTAL 40 MHz -> 1.5625  OK
 *      11-bit: APB 80 MHz -> 1.5625 OK  |  XTAL 40 MHz -> 0.78    NO
 *      12-bit: APB 80 MHz -> 0.78   NO
 *  10 bits is the widest depth that 25.000 kHz is exactly achievable at
 *  from EITHER clock source, so LEDC_AUTO_CLK is free to pick, and the
 *  carrier does not depend on APB staying at 80 MHz. The extra bit that
 *  11 would buy is worthless anyway: the command scale is only 400 wide.
 *
 *  One further reason 10 bits is safe: ledc_channel_config() in ESP-IDF
 *  documents a silicon bug on the ESP32/S2/S3/C3/C2/C6/P4 whereby a duty
 *  of exactly 2**duty_res -- i.e. 100% -- corrupts the hardware duty
 *  calculation IF the timer is at the part's MAXIMUM duty resolution.
 *  On ESP32-S3 that maximum is 14 bits (SOC_LEDC_TIMER_BIT_WIDTH). We are
 *  at 10, so duty == 1024 is a legitimate, reachable, glitch-free 100%
 *  and full throttle really is full throttle.
 * ================================================================== */

#define MOTOR_LEDC_MODE        LEDC_LOW_SPEED_MODE   /* the only mode on S3 */
#define MOTOR_LEDC_TIMER       LEDC_TIMER_1          /* timer 0 is aux_io's */
#define MOTOR_LEDC_DUTY_RES    LEDC_TIMER_10_BIT
#define MOTOR_LEDC_FULL_DUTY   (1 << 10)             /* 1024 */
#define MOTOR_LEDC_FIRST_CH    LEDC_CHANNEL_2        /* 0 and 1 are aux_io's */

/* Which pin of a motor currently carries the LEDC signal. */
#define BIND_NONE  (-1)
#define BIND_A     (0)
#define BIND_B     (1)

/* ESP32-S3 pins that must never be driven as a motor output:
 *   0, 3, 45, 46      strapping
 *   19, 20            native USB D-/D+
 *   26..32            SPI flash bus
 *   33..37            octal PSRAM bus on modules that carry it
 * The eight motor pins are 4,5,9,10,13,14,15,16 -- all clear. This is a
 * compile-time check so that a future edit to board_pins.h that moves a
 * motor onto a reserved pin FAILS THE BUILD rather than failing silently
 * on the bench. */
#define PWM_PIN_IS_USABLE(p)                                             \
    ((p) >= 1 && (p) <= 48 &&                                            \
     (p) != 3 && (p) != 45 && (p) != 46 &&                               \
     (p) != 19 && (p) != 20 &&                                           \
     !((p) >= 26 && (p) <= 37))

_Static_assert(PWM_PIN_IS_USABLE(PIN_M1A), "PIN_M1A cannot be an LEDC output on ESP32-S3");
_Static_assert(PWM_PIN_IS_USABLE(PIN_M1B), "PIN_M1B cannot be an LEDC output on ESP32-S3");
_Static_assert(PWM_PIN_IS_USABLE(PIN_M2A), "PIN_M2A cannot be an LEDC output on ESP32-S3");
_Static_assert(PWM_PIN_IS_USABLE(PIN_M2B), "PIN_M2B cannot be an LEDC output on ESP32-S3");
_Static_assert(PWM_PIN_IS_USABLE(PIN_M3A), "PIN_M3A cannot be an LEDC output on ESP32-S3");
_Static_assert(PWM_PIN_IS_USABLE(PIN_M3B), "PIN_M3B cannot be an LEDC output on ESP32-S3");
_Static_assert(PWM_PIN_IS_USABLE(PIN_M4A), "PIN_M4A cannot be an LEDC output on ESP32-S3");
_Static_assert(PWM_PIN_IS_USABLE(PIN_M4B), "PIN_M4B cannot be an LEDC output on ESP32-S3");

/* The channel budget, enforced. See the long comment above. */
_Static_assert((int)MOTOR_LEDC_FIRST_CH + PWM_MOTOR_COUNT <= (int)LEDC_CHANNEL_MAX,
               "not enough LEDC channels: aux_io.c holds channels 0 and 1 for the servos");
_Static_assert((int)MOTOR_LEDC_TIMER < (int)LEDC_TIMER_MAX, "bad LEDC timer");
_Static_assert(MOTOR_LEDC_FULL_DUTY == (1 << MOTOR_LEDC_DUTY_RES), "duty scale/resolution disagree");
_Static_assert(MOTOR_LEDC_FULL_DUTY >= PWM_FULL_SCALE,
               "LEDC duty resolution would throw away command levels");

typedef struct {
    int            gpio_a;
    int            gpio_b;
    ledc_channel_t ch;
} motor_hw_desc_t;

static const motor_hw_desc_t k_hw[PWM_MOTOR_COUNT] = {
    { PIN_M1A, PIN_M1B, (ledc_channel_t)(MOTOR_LEDC_FIRST_CH + 0) },
    { PIN_M2A, PIN_M2B, (ledc_channel_t)(MOTOR_LEDC_FIRST_CH + 1) },
    { PIN_M3A, PIN_M3B, (ledc_channel_t)(MOTOR_LEDC_FIRST_CH + 2) },
    { PIN_M4A, PIN_M4B, (ledc_channel_t)(MOTOR_LEDC_FIRST_CH + 3) },
};

typedef struct {
    int8_t  bound;       /* BIND_NONE / BIND_A / BIND_B                     */
    int32_t last_duty;   /* signed, post-map, in 400-tick command units     */
} motor_hw_t;

static motor_hw_t s_hw[PWM_MOTOR_COUNT];
static bool       s_ready = false;

/* INVARIANT, maintained by every function below:
 * for each motor, the pin named by s_hw[i].bound (if any) carries that
 * motor's LEDC channel, and its OTHER pin is a plain GPIO output whose
 * level is set directly. No pin is ever left floating or left routed to
 * a peripheral that is no longer driving it. */

/* ------------------------------------------------------------------ *
 * Take a pin back from LEDC (or claim it for the first time) and drive
 * it from the GPIO output register.
 *
 * gpio_set_direction(GPIO_MODE_OUTPUT) is what actually releases the
 * pin: internally it calls esp_rom_gpio_connect_out_signal(gpio,
 * SIG_GPIO_OUT_IDX, ...), re-pointing the GPIO matrix away from the
 * LEDC signal. Seeding the output register first means the pin cannot
 * flick to a stale level in between.
 * ------------------------------------------------------------------ */
static void pin_park(int gpio, int level)
{
    gpio_set_level((gpio_num_t)gpio, level);
    (void)gpio_set_direction((gpio_num_t)gpio, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)gpio, level);
}

/* Silence the channel and hand its pin back to the GPIO matrix, low. */
static void motor_unbind(const motor_hw_desc_t *d, motor_hw_t *m)
{
    (void)ledc_set_duty(MOTOR_LEDC_MODE, d->ch, 0);
    (void)ledc_update_duty(MOTOR_LEDC_MODE, d->ch);

    if (m->bound == BIND_A) {
        pin_park(d->gpio_a, 0);
    } else if (m->bound == BIND_B) {
        pin_park(d->gpio_b, 0);
    }
    m->bound = BIND_NONE;
}

/* Route this motor's channel to the requested pin. No-op if it is already
 * there, which is the steady-state case -- a re-route only happens when the
 * sign of the request changes, and the duty is at or near zero there. */
static void motor_bind(const motor_hw_desc_t *d, motor_hw_t *m, int which)
{
    if (m->bound == which) {
        return;
    }
    motor_unbind(d, m);

    const int gpio = (which == BIND_A) ? d->gpio_a : d->gpio_b;
    gpio_set_level((gpio_num_t)gpio, 0);
    ESP_ERROR_CHECK(ledc_set_pin(gpio, MOTOR_LEDC_MODE, d->ch));
    m->bound = (int8_t)which;
}

/* 400 command ticks -> 1024 LEDC ticks, round to nearest.
 * mag == PWM_FULL_SCALE gives exactly MOTOR_LEDC_FULL_DUTY, which LEDC
 * treats as 100% (max duty is 1 << resolution, inclusive). */
static inline uint32_t duty_to_ledc(int32_t mag)
{
    return (uint32_t)(((int64_t)mag * (int64_t)MOTOR_LEDC_FULL_DUTY
                       + (int64_t)(PWM_FULL_SCALE / 2)) / (int64_t)PWM_FULL_SCALE);
}

void pwm_motor_init(void)
{
    /* Park all eight bridge inputs low BEFORE any peripheral is started, so
     * nothing twitches while the timer is being configured. Until this line
     * the pins are inputs, exactly as they were under MCPWM. */
    for (int i = 0; i < PWM_MOTOR_COUNT; ++i) {
        pin_park(k_hw[i].gpio_a, 0);
        pin_park(k_hw[i].gpio_b, 0);
        s_hw[i].bound     = BIND_NONE;
        s_hw[i].last_duty = 0;
    }

    const ledc_timer_config_t tcfg = {
        .speed_mode      = MOTOR_LEDC_MODE,
        .timer_num       = MOTOR_LEDC_TIMER,
        .duty_resolution = MOTOR_LEDC_DUTY_RES,
        .freq_hz         = PWM_CARRIER_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    /* ESP_ERROR_CHECK deliberately: a failure here aborts before app_main's
     * two confirmation beeps, so "no beeps on power-up" is the loud signal.
     * The console is disabled (micro-ROS owns UART0), so silence-with-motion
     * and silence-without-beeps are the only two states we can distinguish,
     * and they must not be confused. */
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    for (int i = 0; i < PWM_MOTOR_COUNT; ++i) {
        const motor_hw_desc_t *d = &k_hw[i];

        /* Every motor pin must be output-capable on this part. The static
         * asserts above catch the pin map at compile time; this catches a
         * chip/target mismatch at run time rather than driving nothing. */
        if (!GPIO_IS_VALID_OUTPUT_GPIO(d->gpio_a) ||
            !GPIO_IS_VALID_OUTPUT_GPIO(d->gpio_b)) {
            ESP_ERROR_CHECK(ESP_ERR_NOT_SUPPORTED);
        }

        const ledc_channel_config_t ccfg = {
            .gpio_num   = d->gpio_a,     /* A first; B is a plain low GPIO */
            .speed_mode = MOTOR_LEDC_MODE,
            .channel    = d->ch,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = MOTOR_LEDC_TIMER,
            .duty       = 0,             /* duty 0 == a static low         */
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
        s_hw[i].bound = BIND_A;
    }

    s_ready = true;
}

/* ================================================================== *
 *  THE MAP
 *
 *  UNCHANGED from the MCPWM version, deliberately and to the character.
 *  This function is the entire point of this firmware; only the output
 *  stage below it moved from MCPWM to LEDC. It still works in 400-tick
 *  command units, and min_pwm_percent / pwm_eps_percent still mean
 *  percent of that full scale.
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

static void drive(const motor_hw_desc_t *d, motor_hw_t *m, int32_t duty)
{
    /* duty is signed, already mapped, magnitude in [0, PWM_FULL_SCALE]. */
    const bool forward = (duty >= 0);
    int32_t mag = forward ? duty : -duty;
    if (mag > PWM_FULL_SCALE) {
        mag = PWM_FULL_SCALE;
    }

    const int drv_which = forward ? BIND_A : BIND_B;
    const int idle_gpio = forward ? d->gpio_b : d->gpio_a;

    /* Sign convention, identical to the proven Arduino driver:
     *   positive -> PWM on A, B low
     *   negative -> PWM on B, A low
     *   zero     -> both low
     * The non-driving half of the bridge is always held low: sign-magnitude
     * drive, so the bridge is in drive/coast rather than drive/brake and the
     * duty-to-torque curve stays monotonic through zero. */
    motor_bind(d, m, drv_which);
    gpio_set_level((gpio_num_t)idle_gpio, 0);

    (void)ledc_set_duty(MOTOR_LEDC_MODE, d->ch, duty_to_ledc(mag));
    (void)ledc_update_duty(MOTOR_LEDC_MODE, d->ch);
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
    drive(&k_hw[id], &s_hw[id], duty);
}

void pwm_motor_stop(pwm_motor_id_t id, bool brake)
{
    if (!s_ready || id < 0 || id >= PWM_MOTOR_COUNT) {
        return;
    }
    const motor_hw_desc_t *d = &k_hw[id];
    motor_hw_t *m = &s_hw[id];

    /* Hand both pins back to plain GPIO before changing their level, so a
     * brake really is both inputs statically high and not a 25 kHz carrier
     * against a static high. */
    motor_unbind(d, m);

    const int lvl = brake ? 1 : 0;
    gpio_set_level((gpio_num_t)d->gpio_a, lvl);
    gpio_set_level((gpio_num_t)d->gpio_b, lvl);

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
