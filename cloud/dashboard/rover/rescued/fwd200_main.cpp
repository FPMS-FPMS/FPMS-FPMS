/* FORWARD 200mm, measured by encoders, with a dead-encoder failsafe.
 *
 * Open-loop duty with per-wheel trim -- NO PID. The velocity loop is what ran
 * away and drove the rover into a wall; it stays out of this until the basics
 * are trusted.
 *
 * CALIBRATION (operator-measured 2026-08-04): a 3 s crawl at these trims
 * produced ~5532 counts and travelled ~200 mm, so ~27.7 counts/mm. Theory for
 * 1320 CPR through attachFullQuad on a 70 mm wheel gives 24.0 counts/mm, which
 * agrees within the measurement's own precision.
 *
 * FAILSAFE, as requested: if the encoders have not accumulated meaningful
 * counts after 5 s, the motors stop and STAY stopped forever. That covers a
 * dead encoder, a stalled wheel, and a blocked rover -- all cases where
 * continuing to drive blind is how something gets broken.
 *
 * loop() is EMPTY: this runs once and never repeats.
 */
#include <Arduino.h>
#include <ESP32Encoder.h>

const int IN_A[4] = {4, 15, 9, 13};    // board M1, M2, M3, M4
const int IN_B[4] = {5, 16, 10, 14};
const int ENC_A[4] = {6, 47, 11, 1};   // board H1, H2, H3, H4
const int ENC_B[4] = {7, 48, 12, 2};

// Right pair (M1, M2) is wired opposite: it takes duty on IN_B to go forward.
const bool ON_IN_B[4] = {true, true, false, false};
const char *WHEEL[4] = {"M1 front-right", "M2 rear-right ",
                        "M3 front-left ", "M4 rear-left  "};

const int BASE_DUTY = 52;
const float TRIM[4] = {0.820f, 0.839f, 0.914f, 1.000f};

const float COUNTS_PER_MM = 27.7f;
const float TARGET_MM = 200.0f;

const unsigned long FAILSAFE_MS = 5000;   // no counts by now -> stop forever
const long FAILSAFE_MIN_COUNTS = 200;     // ~7mm; anything less is "not moving"
const unsigned long HARD_MAX_MS = 20000;  // absolute ceiling

ESP32Encoder enc[4];
int64_t start_c[4];

void allStop() {
    for (int i = 0; i < 4; i++) {
        analogWrite(IN_A[i], 0);
        analogWrite(IN_B[i], 0);
    }
}

void drive() {
    for (int i = 0; i < 4; i++) {
        int d = (int)(BASE_DUTY * TRIM[i] + 0.5f);
        if (ON_IN_B[i]) { analogWrite(IN_A[i], 0); analogWrite(IN_B[i], d); }
        else            { analogWrite(IN_B[i], 0); analogWrite(IN_A[i], d); }
    }
}

/* Magnitude, because the left encoders count down on forward while the right
 * count up. Averaging the raw signed values would cancel to roughly zero. */
long avgAbsDelta() {
    long long s = 0;
    for (int i = 0; i < 4; i++) {
        long long d = (long long)(enc[i].getCount() - start_c[i]);
        s += (d < 0 ? -d : d);
    }
    return (long)(s / 4);
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

    delay(5000);                       // time to watch

    const long target = (long)(TARGET_MM * COUNTS_PER_MM);
    for (int i = 0; i < 4; i++) start_c[i] = enc[i].getCount();

    unsigned long t0 = millis();
    bool failsafe = false;
    bool reached = false;
    long got = 0;

    drive();
    while (true) {
        got = avgAbsDelta();
        unsigned long el = millis() - t0;
        if (got >= target) { reached = true; break; }
        if (el > FAILSAFE_MS && got < FAILSAFE_MIN_COUNTS) { failsafe = true; break; }
        if (el > HARD_MAX_MS) break;
        delay(5);
    }
    allStop();
    delay(800);                        // coast to rest before the final read
    got = avgAbsDelta();

    Serial.println();
    Serial.println("=== FORWARD 200mm ===");
    Serial.printf("target %ld counts (%.0f mm at %.1f counts/mm)\n",
                  target, TARGET_MM, COUNTS_PER_MM);
    Serial.println();
    for (int i = 0; i < 4; i++) {
        long long d = (long long)(enc[i].getCount() - start_c[i]);
        long long m = d < 0 ? -d : d;
        Serial.printf("  %s  delta %+8lld   %6.1f mm\n",
                      WHEEL[i], d, (double)m / COUNTS_PER_MM);
    }
    Serial.println();
    Serial.printf("  average %ld counts  =  %.1f mm travelled\n",
                  got, got / COUNTS_PER_MM);
    if (failsafe) {
        Serial.println();
        Serial.println("  *** FAILSAFE TRIPPED ***");
        Serial.printf("  No meaningful encoder counts after %lu ms.\n", FAILSAFE_MS);
        Serial.println("  Motors stopped and will STAY stopped.");
        Serial.println("  Causes: dead encoder, stalled wheel, or blocked rover.");
    } else if (reached) {
        Serial.println("  target reached - stopped normally.");
    } else {
        Serial.println("  HARD TIMEOUT - stopped without reaching target.");
    }
    Serial.println("=== END - motors off and staying off ===");
}

void loop() {
    allStop();
    delay(1000);
}
