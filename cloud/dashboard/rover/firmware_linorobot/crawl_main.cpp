/* SIMPLE FORWARD CRAWL. Nothing else.
 *
 * All four motors, forward, slowly, for 3 seconds. Then stop, forever.
 *
 * NO encoders. NO PID. NO odometry. NO ROS. NO micro-ROS.
 * Open-loop duty straight to the H-bridges -- this deliberately bypasses the
 * velocity loop, which is where every runaway and spin has come from.
 *
 * The whole sequence lives in setup() and loop() is EMPTY, so it runs exactly
 * once and never repeats. The previous test firmware put motor commands in
 * loop() and kept driving the rover with the host shut down. Never again.
 */
#include <Arduino.h>

// board_pins.h: sign-magnitude drive, two PWM pins per motor.
const int IN_A[4] = {4, 15, 9, 13};   // board M1, M2, M3, M4
const int IN_B[4] = {5, 16, 10, 14};

const int DUTY = 62;      // 8-bit (0-255). Slower again, as asked.
const int RUN_MS = 6000;  // long enough to watch properly

/* OPERATOR-OBSERVED 2026-08-04: with all four driven on IN_A at equal duty, the
 * LEFT side ran forward and the RIGHT side ran backward -- the rover spun. No
 * encoders and no PID were involved, so this is a wiring fact: the right-side
 * motors are connected with opposite polarity to the left.
 *
 * Board M1 and M2 are the RIGHT side (SESSION_HANDOFF 2026-08-01 rewire);
 * M3 and M4 are the LEFT. So the right pair must be driven on IN_B to go
 * forward. Index order below is board M1, M2, M3, M4.
 *
 * This also invalidates the earlier encoder conclusion: that test called IN_A
 * "forward" for every motor, so on the right side it was actually measuring
 * reverse. Their negative counts were CORRECT, not inverted. */
const bool DRIVE_ON_IN_B[4] = {true, true, false, false};

void allStop() {
    for (int i = 0; i < 4; i++) {
        analogWrite(IN_A[i], 0);
        analogWrite(IN_B[i], 0);
    }
}

void setup() {
    for (int i = 0; i < 4; i++) {
        pinMode(IN_A[i], OUTPUT);
        pinMode(IN_B[i], OUTPUT);
    }
    allStop();
    delay(6000);            // time to get set and watch before anything moves

    // All four the same PHYSICAL direction: the right pair drives on IN_B
    // because it is wired opposite to the left.
    for (int i = 0; i < 4; i++) {
        if (DRIVE_ON_IN_B[i]) {
            analogWrite(IN_A[i], 0);
            analogWrite(IN_B[i], DUTY);
        } else {
            analogWrite(IN_B[i], 0);
            analogWrite(IN_A[i], DUTY);
        }
    }
    delay(RUN_MS);

    allStop();
}

void loop() {
    // Intentionally empty. The rover moves once and stops.
    allStop();
    delay(1000);
}
