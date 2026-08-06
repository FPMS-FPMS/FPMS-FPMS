/* Encoder polarity + mapping test for the Yahboom MicroROS Board V2.0.
 *
 * WHY: with the closed-loop firmware, a pure-forward command spins the rover in
 * place while the odometry reports clean forward travel. That is the signature
 * of inverted encoder signs -- the PID reads negative RPM when it commanded
 * positive, error grows instead of shrinking, and the wheel saturates. The
 * odometry is integrated from the same inverted encoders, so it lies in
 * agreement. Every sensor-based test was fooled; only the operator watching
 * caught it.
 *
 * WHAT THIS DOES: drives ONE motor at a time, forward then reverse, and prints
 * the delta of ALL FOUR encoders each time. That answers, per wheel:
 *   - does its own encoder count UP when the motor is driven forward?  (sign)
 *   - does driving motor N move only encoder N?                        (mapping)
 *
 * Plain Serial at 115200. micro-ROS must be stopped while this runs.
 */
#include <Arduino.h>
#include <ESP32Encoder.h>

// board_pins.h, confirmed against Yahboom docs, the PrwTsrt tree and the
// operator's own Arduino sketch.
const int IN_A[4] = {4, 15, 9, 13};    // board M1, M2, M3, M4
const int IN_B[4] = {5, 16, 10, 14};
const int ENC_A[4] = {6, 47, 11, 1};   // board H1, H2, H3, H4
const int ENC_B[4] = {7, 48, 12, 2};

ESP32Encoder enc[4];

const int DUTY = 190;      // 8-bit; well clear of stiction, gentle enough
const int RUN_MS = 900;
const int SETTLE_MS = 700;

void allStop() {
    for (int i = 0; i < 4; i++) {
        analogWrite(IN_A[i], 0);
        analogWrite(IN_B[i], 0);
    }
}

void readAll(int64_t *out) {
    for (int i = 0; i < 4; i++) out[i] = enc[i].getCount();
}

void drive(int m, bool fwd) {
    // Sign-magnitude: duty rides on IN_A or IN_B, the idle pin stays at 0.
    analogWrite(IN_A[m], fwd ? DUTY : 0);
    analogWrite(IN_B[m], fwd ? 0 : DUTY);
}

void testMotor(int m, bool fwd) {
    int64_t before[4], after[4];
    allStop();
    delay(SETTLE_MS);
    readAll(before);
    drive(m, fwd);
    delay(RUN_MS);
    allStop();
    delay(SETTLE_MS);
    readAll(after);

    Serial.printf("MOTOR %d (board M%d) %s  ->  ", m + 1, m + 1, fwd ? "FWD" : "REV");
    for (int i = 0; i < 4; i++) {
        Serial.printf("E%d %+7lld   ", i + 1, (long long)(after[i] - before[i]));
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(2500);
    Serial.println();
    Serial.println("=== FPMS ENCODER POLARITY / MAPPING TEST ===");
    Serial.println("Each row: drive ONE motor, show the delta of all four encoders.");
    Serial.println("Healthy: driving motor N moves ONLY encoder N, and FWD is positive.");
    Serial.println();

    for (int i = 0; i < 4; i++) {
        pinMode(IN_A[i], OUTPUT);
        pinMode(IN_B[i], OUTPUT);
        analogWrite(IN_A[i], 0);
        analogWrite(IN_B[i], 0);
    }
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    for (int i = 0; i < 4; i++) {
        enc[i].attachFullQuad(ENC_A[i], ENC_B[i]);
        enc[i].clearCount();
    }
    delay(400);
}

void loop() {
    Serial.println("---- PASS ----");
    for (int m = 0; m < 4; m++) {
        testMotor(m, true);
        testMotor(m, false);
    }
    Serial.println();
    Serial.println("VERDICT GUIDE:");
    Serial.println("  own encoder NEGATIVE on FWD  -> set MOTORn_ENCODER_INV true");
    Serial.println("  a DIFFERENT encoder moved    -> encoder pins are mis-mapped");
    Serial.println("  own encoder ~0 on both       -> dead encoder or dead motor");
    Serial.println();
    delay(4000);
}
