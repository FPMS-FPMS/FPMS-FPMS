/* FORWARD 400mm, pause, BACK 400mm. Encoder-measured, with a failsafe.
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

const int BASE_DUTY = 58;          // a little above 52; 52 stalled from rest
const int KICK_DUTY = 140;         // breaks stiction
const int KICK_MS = 220;
const float TRIM[4] = {0.820f, 0.839f, 0.914f, 1.000f};

const float COUNTS_PER_MM = 27.7f;
const float LEG_MM = 400.0f;

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

// returns: 0 reached, 1 failsafe, 2 hard timeout
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
    else                  Serial.println("[hard timeout]");
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
    Serial.println("=== FORWARD 400mm, THEN BACK 400mm ===");

    if (runLeg("FORWARD 400mm", true) == 1) {
        Serial.println("Failsafe on the outbound leg - NOT attempting the return.");
        allStop();
        return;
    }
    delay(2000);
    runLeg("BACK 400mm", false);

    allStop();
    Serial.println();
    Serial.println("=== END - motors off and staying off ===");
}

void loop() {
    allStop();
    delay(1000);
}
