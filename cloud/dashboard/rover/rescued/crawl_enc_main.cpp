/* SLOW FORWARD CRAWL + ENCODER READOUT. Nothing else.
 *
 * Exactly the test that worked: all four motors forward, open-loop, duty 62.
 * Runs for 3 seconds, stops, then prints the encoder delta for each wheel.
 *
 * NO PID. NO closed loop. NO ROS. Open-loop duty cannot run away -- the runaway
 * comes from the velocity loop reading an inverted encoder and winding up.
 *
 * Everything is in setup(); loop() is EMPTY, so it moves once and never repeats.
 *
 * Right side (board M1, M2) is driven on IN_B because it is wired with opposite
 * polarity to the left -- operator-verified: with all four on IN_A the left ran
 * forward and the right ran backward.
 */
#include <Arduino.h>
#include <ESP32Encoder.h>

const int IN_A[4] = {4, 15, 9, 13};    // board M1, M2, M3, M4
const int IN_B[4] = {5, 16, 10, 14};
const int ENC_A[4] = {6, 47, 11, 1};   // board H1, H2, H3, H4
const int ENC_B[4] = {7, 48, 12, 2};

// Right pair is wired opposite, so it takes duty on IN_B to go forward.
const bool ON_IN_B[4] = {true, true, false, false};
const char *WHEEL[4] = {"M1 front-right", "M2 rear-right",
                        "M3 front-left ", "M4 rear-left  "};

/* SLOWER, and trimmed straight.
 *
 * At duty 62 for 3 s the wheels did not run at equal rates, which is what makes
 * it curve. Measured counts:
 *     M1 front-right 20288    M3 front-left  18730
 *     M2 rear-right  19988    M4 rear-left   17677
 * The right side ran ~10%% faster than the left.
 *
 * Each wheel's duty is scaled to match the SLOWEST (M4), so all four turn at the
 * same rate. Trim = 17677 / that wheel's count. This is open-loop compensation;
 * the velocity PID does this job itself once the closed loop is trusted again.
 */
const int BASE_DUTY = 52;   // slower than the 62 that worked
/* Second iteration. Pass 1 (trims 0.871/0.884/0.944/1.000) left a 6.2% spread:
 *     M1 6892   M2 6839   M3 6700   M4 6491
 * Each trim is re-scaled by 6491/count, chasing the slowest wheel again. Duty
 * vs speed is not linear near stiction, so this converges by iteration rather
 * than by calculation. */
const float TRIM[4] = {0.820f,   // M1 front-right  0.871 * 6491/6892
                       0.839f,   // M2 rear-right   0.884 * 6491/6839
                       0.914f,   // M3 front-left   0.944 * 6491/6700
                       1.000f};  // M4 rear-left    (the slowest, reference)
const int RUN_MS = 3000;

ESP32Encoder enc[4];

void allStop() {
    for (int i = 0; i < 4; i++) {
        analogWrite(IN_A[i], 0);
        analogWrite(IN_B[i], 0);
    }
}

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < 4; i++) {
        pinMode(IN_A[i], OUTPUT);
        pinMode(IN_B[i], OUTPUT);
    }
    allStop();

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    for (int i = 0; i < 4; i++) {
        enc[i].attachFullQuad(ENC_A[i], ENC_B[i]);
        enc[i].clearCount();
    }

    delay(5000);                       // time to get set and watch

    int64_t before[4], after[4];
    for (int i = 0; i < 4; i++) before[i] = enc[i].getCount();

    for (int i = 0; i < 4; i++) {
        int d = (int)(BASE_DUTY * TRIM[i] + 0.5f);
        if (ON_IN_B[i]) { analogWrite(IN_A[i], 0); analogWrite(IN_B[i], d); }
        else            { analogWrite(IN_B[i], 0); analogWrite(IN_A[i], d); }
    }
    delay(RUN_MS);
    allStop();
    delay(800);                        // let it coast to rest before reading

    for (int i = 0; i < 4; i++) after[i] = enc[i].getCount();

    Serial.println();
    Serial.println("=== SLOW FORWARD CRAWL - ENCODER RESULT ===");
    Serial.printf("base duty %d (per-wheel trimmed), %d ms, all four FORWARD\n",
                  BASE_DUTY, RUN_MS);
    Serial.println();
    long long mag[4];
    long long lo = 0, hi = 0;
    for (int i = 0; i < 4; i++) {
        long long d = (long long)(after[i] - before[i]);
        mag[i] = d < 0 ? -d : d;
        if (i == 0) { lo = hi = mag[i]; }
        if (mag[i] < lo) lo = mag[i];
        if (mag[i] > hi) hi = mag[i];
        Serial.printf("  %s  duty %3d  delta %+8lld  |%lld|\n",
                      WHEEL[i], (int)(BASE_DUTY * TRIM[i] + 0.5f), d, mag[i]);
    }
    Serial.println();
    if (lo > 0) {
        Serial.printf("  spread: slowest %lld, fastest %lld  -> %.1f%% apart\n",
                      lo, hi, 100.0 * (double)(hi - lo) / (double)lo);
        Serial.println("  under ~3%% is straight; more than that still curves.");
    } else {
        Serial.println("  A WHEEL REGISTERED ZERO - too slow to break stiction,");
        Serial.println("  or that wheel is not turning. Raise BASE_DUTY.");
    }
    Serial.println("=== END - motors are off and stay off ===");
}

void loop() {
    allStop();
    delay(1000);
}
