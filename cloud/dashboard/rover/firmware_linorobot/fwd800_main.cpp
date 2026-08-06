/* FORWARD 800mm, pause, BACK 800mm. Encoder-measured, with a failsafe.
 *
 * Open-loop duty with per-wheel trim. NO PID -- the velocity loop is what ran
 * away into a wall and stays out until the basics are trusted.
 *
 * KICK-START: at crawl duty (~43-52) from a standing start the motors only
 * buzzed and the rover did not move -- audible PWM whine with not enough torque
 * to break stiction, which is what the operator heard as a beep. A short
 * high-duty kick breaks it free, then duty drops to the crawl. Standard
 * practice for low-speed DC drive, and it costs a few mm of extra travel.
 *
 * CALIBRATION (operator-measured): ~27.7 counts/mm.
 * FAILSAFE: no meaningful counts within 5 s of a leg starting -> stop forever.
 * loop() is EMPTY: runs once, never repeats.
 */
#include <Arduino.h>
#include <ESP32Encoder.h>

const int IN_A[4] = {4, 15, 9, 13};    // board M1, M2, M3, M4
const int IN_B[4] = {5, 16, 10, 14};
const int ENC_A[4] = {6, 47, 11, 1};   // board H1, H2, H3, H4
const int ENC_B[4] = {7, 48, 12, 2};

// Right pair (M1, M2) is wired opposite: forward = duty on IN_B.
const bool ON_IN_B[4] = {true, true, false, false};
const char *WHEEL[4] = {"M1 front-right", "M2 rear-right ",
                        "M3 front-left ", "M4 rear-left  "};

const int BASE_DUTY = 70;          // a couple of points up from 58, as asked
const int KICK_DUTY = 140;         // breaks stiction; without it the motors buzz
const int KICK_MS = 220;
const float TRIM[4] = {0.820f, 0.839f, 0.914f, 1.000f};

/* CORRECTED from the operator's tape measure: the rover really travelled 800mm
 * while the encoders reported 11836 counts -> 14.8 counts/mm. The earlier 27.7
 * came from a rough eyeball estimate and was 1.9x too high, so every distance
 * computed with it was short by nearly half. */
const float COUNTS_PER_MM = 14.8f;
const float LEG_MM = 800.0f;

/* SLIP DETECTOR. A wheel came off during a reverse leg and the counts went
 * 740mm / 228mm / 203mm / 565mm across the four wheels while the chassis barely
 * moved. The "no counts" failsafe cannot catch that -- slipping wheels still
 * count. If the fastest and slowest wheel disagree by more than this once
 * there is real travel, something is slipping or broken: stop and say so
 * instead of banking the average as distance. */
const float SLIP_ABORT_FRAC = 0.55f;   // 0.35 tripped on a healthy reverse (35%); the loose-wheel failure was 292%
const long SLIP_CHECK_AFTER = 7000;   // ~470mm. Reverse breaks away unevenly for the first few hundred mm and then settles; checking at 1500 counts tripped on that transient three runs running.

const unsigned long FAILSAFE_MS = 5000;
const long FAILSAFE_MIN_COUNTS = 200;
const unsigned long HARD_MAX_MS = 30000;

ESP32Encoder enc[4];
int64_t leg_start[4];

void allStop() {
    for (int i = 0; i < 4; i++) { analogWrite(IN_A[i], 0); analogWrite(IN_B[i], 0); }
}

// fwd=true drives the rover forward; fwd=false drives it backward.
void drive(int duty, bool fwd) {
    for (int i = 0; i < 4; i++) {
        int d = (int)(duty * TRIM[i] + 0.5f);
        bool useB = ON_IN_B[i] == fwd;   // flip which pin carries duty in reverse
        if (useB) { analogWrite(IN_A[i], 0); analogWrite(IN_B[i], d); }
        else      { analogWrite(IN_B[i], 0); analogWrite(IN_A[i], d); }
    }
}

// Magnitude: the left encoders count down on forward, so signed averaging
// would cancel to about zero.
long avgAbs() {
    long long s = 0;
    for (int i = 0; i < 4; i++) {
        long long d = (long long)(enc[i].getCount() - leg_start[i]);
        s += (d < 0 ? -d : d);
    }
    return (long)(s / 4);
}

// Fractional disagreement between the fastest and slowest wheel.
float slipFrac() {
    long long lo = -1, hi = 0;
    for (int i = 0; i < 4; i++) {
        long long d = (long long)(enc[i].getCount() - leg_start[i]);
        if (d < 0) d = -d;
        if (lo < 0 || d < lo) lo = d;
        if (d > hi) hi = d;
    }
    if (lo <= 0) return 1.0f;
    return (float)(hi - lo) / (float)lo;
}

// returns: 0 reached, 1 failsafe, 2 hard timeout, 3 slip
int runLeg(const char *name, bool fwd) {
    const long target = (long)(LEG_MM * COUNTS_PER_MM);
    for (int i = 0; i < 4; i++) leg_start[i] = enc[i].getCount();

    unsigned long t0 = millis();
    int result = 2;

    drive(KICK_DUTY, fwd);
    delay(KICK_MS);
    drive(BASE_DUTY, fwd);

    while (true) {
        long got = avgAbs();
        unsigned long el = millis() - t0;
        if (got >= target) { result = 0; break; }
        if (el > FAILSAFE_MS && got < FAILSAFE_MIN_COUNTS) { result = 1; break; }
        if (got > SLIP_CHECK_AFTER && slipFrac() > SLIP_ABORT_FRAC) { result = 3; break; }
        if (el > HARD_MAX_MS) { result = 2; break; }
        delay(5);
    }
    allStop();
    delay(900);

    Serial.println();
    Serial.printf("--- %s (target %ld counts = %.0f mm) ---\n", name, target, LEG_MM);
    for (int i = 0; i < 4; i++) {
        long long d = (long long)(enc[i].getCount() - leg_start[i]);
        long long m = d < 0 ? -d : d;
        Serial.printf("  %s  delta %+8lld   %6.1f mm\n",
                      WHEEL[i], d, (double)m / COUNTS_PER_MM);
    }
    long got = avgAbs();
    Serial.printf("  average %ld counts = %.1f mm  ", got, got / COUNTS_PER_MM);
    if (result == 0)      Serial.println("[reached target]");
    else if (result == 1) Serial.println("[*** FAILSAFE - not moving, stopping for good ***]");
    else if (result == 3) Serial.printf("[*** SLIP - wheels disagree by %.0f%% ***]\n",
                                        slipFrac() * 100.0f);
    else                  Serial.println("[hard timeout]");
    Serial.printf("  wheel spread %.0f%% (under 15%% is a clean straight run)\n",
                  slipFrac() * 100.0f);
    return result;
}

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < 4; i++) { pinMode(IN_A[i], OUTPUT); pinMode(IN_B[i], OUTPUT); }
    allStop();
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    for (int i = 0; i < 4; i++) {
        enc[i].attachFullQuad(ENC_A[i], ENC_B[i]);
        enc[i].clearCount();
    }
    delay(5000);

    Serial.println();
    Serial.println("=== FORWARD 800mm, THEN BACK 800mm ===");

    if (runLeg("FORWARD 800mm", true) == 1) {
        Serial.println("Failsafe on the outbound leg - NOT attempting the return.");
        allStop();
        return;
    }
    delay(2000);
    runLeg("BACK 800mm", false);

    allStop();
    Serial.println();
    Serial.println("=== END - motors off and staying off ===");
}

void loop() {
    allStop();
    delay(1000);
}
