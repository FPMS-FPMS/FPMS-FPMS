/* FPMS drive board — FreeRTOS duty amplifier.
 *
 * DESIGN RULE: this board has NO intelligence. Duty in, sensors out.
 * No velocity setpoint, no PID, no odometry integration, no planning.
 *
 * WHY. The previous firmware (linorobot2_hardware) closed a float-RPM velocity
 * PID on the board. On this rover the left encoders are wired inverted, so the
 * PID read a negative RPM when it commanded positive, the error GREW as it
 * "corrected", the wheel saturated, and the rover spun. The odometry — computed
 * from those same encoders — reported clean forward travel while the chassis
 * span on the spot. It drove into a wall twice. Open-loop duty on identical
 * hardware gave 0.3% distance error, repeatably.
 *
 * The golden phase6 driver ("B8B") that actually worked commanded RAW DUTY at
 * 26/100, from a 20 Hz loop on the Pi, and never used a velocity setpoint. This
 * firmware exists to give that loop exactly what it expects.
 *
 * SAFETY, in order of importance:
 *   1. DEADMAN. No duty command within CMD_TIMEOUT_MS -> all motors zero. The
 *      golden code's _spin() had no timeout at all; a dead gyro spun forever.
 *   2. The motor task is the HIGHEST priority task and owns the outputs. No
 *      other task may write a pin.
 *   3. Motor commands live in a task, never in loop(). A previous test sketch
 *      drove motors from loop() and kept moving the rover after the host was
 *      shut down.
 *
 * MEASURED HARDWARE FACTS (operator-verified 2026-08-03/04 — do not "fix" these
 * without re-measuring):
 *   - RIGHT-side motors are wired with OPPOSITE polarity to the left.
 *   - Only the LEFT encoders count inverted.
 *   - ~14.8 counts/mm on 70 mm wheels (tape-measured over 800 mm).
 *   - Below ~duty 60 from rest the motors only buzz; a kick is needed.
 *   - Serial must be 230400. At 921600 the micro-ROS session died in 1-3 min.
 */

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>

// ----------------------------------------------------------------- pin map
// board_pins.h, corroborated by Yahboom docs, the PrwTsrt tree and the
// operator's own Arduino sketch.
static const int IN_A[4] = {4, 15, 9, 13};    // board M1, M2, M3, M4
static const int IN_B[4] = {5, 16, 10, 14};
static const int ENC_A[4] = {6, 47, 11, 1};   // board H1..H4
static const int ENC_B[4] = {7, 48, 12, 2};

/* Forward = duty on IN_B for the RIGHT pair, IN_A for the LEFT.
 * Operator-observed: with all four on IN_A the left ran forward and the right
 * ran backward, and the rover spun. Pure wiring, no sensors involved. */
static const bool RIGHT_SIDE[4] = {true, true, false, false};
static const bool FWD_ON_IN_B[4] = {true, true, false, false};

/* Only the LEFT encoders are inverted. Driving all four forward gave
 * M1 +20288, M2 +19988, M3 -18730, M4 -17677: right counts up, left counts
 * down. Inverting all four (an earlier wrong reading) made a commanded 564 mm
 * produce -2080 mm of phantom travel. */
static const int ENC_SIGN[4] = {+1, +1, -1, -1};

// -------------------------------------------------------------- parameters
static const int PWM_FREQ_HZ = 20000;   // above audible; at ~1 kHz the motors sing
static const int PWM_BITS = 8;          // 0..255, matching the measured duty values
static const int DUTY_MAX = 255;

/* The Pi must refresh the command faster than this or the board stops. The
 * golden loop runs at 20 Hz (50 ms), so 300 ms is six missed ticks — long
 * enough to ride out jitter, short enough that a crashed host stops the rover
 * in under a third of a second. */
static const uint32_t CMD_TIMEOUT_MS = 300;

static const uint32_t MOTOR_TASK_HZ = 100;
static const uint32_t SENSOR_TASK_HZ = 50;

// ------------------------------------------------------------------- state
ESP32Encoder enc[4];

struct DutyCmd {
    int16_t duty[4];      // -255..255, per motor, already sign-corrected by the Pi
    uint32_t stamp_ms;
};
static volatile DutyCmd g_cmd = {{0, 0, 0, 0}, 0};
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool g_deadman_tripped = false;

// --------------------------------------------------------------- motor I/O
static void motorApply(int i, int16_t duty) {
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    if (duty < -DUTY_MAX) duty = -DUTY_MAX;

    bool forward = duty >= 0;
    int mag = forward ? duty : -duty;
    // Which physical pin carries the duty depends on how that side is wired.
    bool useB = (FWD_ON_IN_B[i] == forward);
    if (useB) {
        analogWrite(IN_A[i], 0);
        analogWrite(IN_B[i], mag);
    } else {
        analogWrite(IN_B[i], 0);
        analogWrite(IN_A[i], mag);
    }
}

static void motorAllStop() {
    for (int i = 0; i < 4; i++) {
        analogWrite(IN_A[i], 0);
        analogWrite(IN_B[i], 0);
    }
}

/* Highest-priority task. It is the ONLY writer of the motor pins, so a stall or
 * crash anywhere else in the firmware still results in the deadman stopping the
 * rover rather than a stale duty being held. */
static void motorTask(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(1000 / MOTOR_TASK_HZ);
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        DutyCmd cmd;
        portENTER_CRITICAL(&g_mux);
        cmd = *(DutyCmd *)&g_cmd;
        portEXIT_CRITICAL(&g_mux);

        uint32_t age = millis() - cmd.stamp_ms;
        if (cmd.stamp_ms == 0 || age > CMD_TIMEOUT_MS) {
            if (!g_deadman_tripped) g_deadman_tripped = true;
            motorAllStop();
        } else {
            g_deadman_tripped = false;
            for (int i = 0; i < 4; i++) motorApply(i, cmd.duty[i]);
        }
        vTaskDelayUntil(&next, period);
    }
}

// ------------------------------------------------------------------ sensors
static int64_t encCount(int i) {
    return (int64_t)enc[i].getCount() * ENC_SIGN[i];
}

/* Call from the Pi-facing transport to publish counts + gyro. Kept separate
 * from the motor task so a slow I2C read can never delay a motor update. */
static void sensorTask(void *arg) {
    const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_TASK_HZ);
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        // TODO(next step): publish encCount(0..3), gyro Z and battery over
        // micro-ROS. Deliberately left unwired until the transport is chosen so
        // that no half-built publisher can hold the serial port.
        vTaskDelayUntil(&next, period);
    }
}

// --------------------------------------------------------------------- API
/* Called by the transport when a duty command arrives from the Pi. The Pi sends
 * per-motor duty already sign-corrected, so the board applies no policy at all
 * beyond clamping — the whole point of this firmware. */
void onDutyCommand(const int16_t d[4]) {
    portENTER_CRITICAL(&g_mux);
    for (int i = 0; i < 4; i++) g_cmd.duty[i] = d[i];
    g_cmd.stamp_ms = millis();
    portEXIT_CRITICAL(&g_mux);
}

void setup() {
    Serial.begin(230400);      // 921600 was unstable and killed the session

    for (int i = 0; i < 4; i++) {
        pinMode(IN_A[i], OUTPUT);
        pinMode(IN_B[i], OUTPUT);
    }
    motorAllStop();

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    for (int i = 0; i < 4; i++) {
        enc[i].attachFullQuad(ENC_A[i], ENC_B[i]);
        enc[i].clearCount();
    }

    Wire.begin(40, 39);        // ICM42670P, SDA 40 / SCL 39
    Wire.setClock(400000);

    /* Motor task above sensor task: a late sensor sample is harmless, a late
     * motor update means the rover keeps driving on a stale command. Pinned to
     * core 1 so the micro-ROS transport on core 0 cannot starve it. */
    xTaskCreatePinnedToCore(motorTask, "motor", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(sensorTask, "sensor", 8192, NULL, 3, NULL, 1);
}

void loop() {
    /* Intentionally empty. Motor commands NEVER live here: a previous test
     * sketch drove motors from loop() and kept moving the rover after the host
     * had been shut down. */
    vTaskDelay(pdMS_TO_TICKS(1000));
}
