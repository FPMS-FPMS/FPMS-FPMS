/* FORWARD 800, BACK 800, TURN +90, TURN -90.
 *
 * Open-loop duty throughout. NO velocity PID -- that loop is what ran away.
 * Straight legs terminate on ENCODER counts; turns terminate on the GYRO,
 * which is how the golden phase6 driver got +/-1-4 deg turns.
 *
 * OVERSHOOT COMPENSATION (measured, not assumed). Commanding 11840 counts
 * (800mm) produced 13111 forward and 13799 reverse, i.e. ~1270 and ~1960 counts
 * of kick-start plus coast. Those are subtracted from the target so the rover
 * ends up where it was asked to. Reverse coasts further, so the two directions
 * carry separate numbers.
 *
 * loop() is EMPTY: runs once and stops.
 */
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>

const int IN_A[4] = {4, 15, 9, 13};    // board M1, M2, M3, M4
const int IN_B[4] = {5, 16, 10, 14};
const int ENC_A[4] = {6, 47, 11, 1};
const int ENC_B[4] = {7, 48, 12, 2};
const bool ON_IN_B[4] = {true, true, false, false};   // right pair wired opposite
const bool IS_RIGHT[4] = {true, true, false, false};  // M1,M2 right; M3,M4 left
const char *WHEEL[4] = {"M1 front-right", "M2 rear-right ",
                        "M3 front-left ", "M4 rear-left  "};

const int BASE_DUTY = 70;
const int KICK_DUTY = 140;
const int KICK_MS = 220;
const float TRIM[4]     = {0.820f, 0.839f, 0.914f, 1.000f};
/* Reverse needs its OWN trim. Forward trims were calibrated driving forward,
   and the imbalance is not the same going backwards: with the forward set, a
   reverse leg measured M1 753.6, M2 857.3, M3 770.8, M4 785.3 mm -- M2 rear-right
   overruns by ~14%. Each value is the forward trim rescaled to the slowest
   wheel of that reverse run. */
const float TRIM_REV[4] = {0.820f, 0.737f, 0.894f, 0.960f};

const float COUNTS_PER_MM = 14.8f;
const float LEG_MM = 800.0f;
const long COAST_FWD = 1270;    // measured overshoot, counts
const long COAST_REV = 1448;   // re-measured with the reverse trims: cutting at 9880 gave 11328 counts, so 1448 of coast, not the 1960 seen at the old (faster) reverse trims

const int TURN_DUTY = 95;       // turning scrubs all four tyres; needs more than a crawl
/* Coast is a fixed angle, not a fraction: with 36deg for both, LEFT overshot to
   +95.8 and RIGHT undershot to -82.4. The two directions genuinely differ, so
   they get their own values. */
const float TURN_COAST_L_DEG = 38.0f;   // 36 -> 95.8deg, 42 -> 78.0deg; interpolates to 38 for 90
const float TURN_COAST_R_DEG = 27.0f;   // 28 -> 87.1deg, needs a touch less coast

const unsigned long FAILSAFE_MS = 5000;
const long FAILSAFE_MIN_COUNTS = 200;
const unsigned long HARD_MAX_MS = 30000;
const float SLIP_ABORT_FRAC = 0.55f;
const long SLIP_CHECK_AFTER = 7000;

/* ICM42670-P, I2C0 SCL 39 / SDA 40. Register map from this project's own
 * ESP-IDF driver. Gyro data lives at its OWN address (0x11) -- it is NOT
 * contiguous with the accel block, which is a trap that cost a session. */
#define ICM_ADDR 0x68
#define ICM_GYRO_DATA_X1 0x11
#define ICM_PWR_MGMT0 0x1F
#define ICM_GYRO_CONFIG0 0x20
#define ICM_ACCEL_CONFIG0 0x21
#define ICM_WHO_AM_I 0x75

/* Battery sense: board_pins.h PIN_BAT_ADC = 3, 12-bit ADC, 3.3V ref,
 * 33k+10k divider -> volts = raw * 3.3/4096 * 4.3 */
#define BAT_PIN 3
float batteryV() {
    long s = 0;
    for (int i = 0; i < 16; i++) { s += analogRead(BAT_PIN); delay(2); }
    return (s / 16.0f) * (3.3f / 4096.0f) * 4.3f;
}

ESP32Encoder enc[4];
int64_t leg_start[4];
uint8_t imu_addr = ICM_ADDR;
bool imu_ok = false;
float gyro_bias = 0.0f;

void allStop() {
    for (int i = 0; i < 4; i++) { analogWrite(IN_A[i], 0); analogWrite(IN_B[i], 0); }
}

bool regWrite(uint8_t r, uint8_t v) {
    Wire.beginTransmission(imu_addr); Wire.write(r); Wire.write(v);
    return Wire.endTransmission() == 0;
}
bool regRead(uint8_t a, uint8_t r, uint8_t *b, size_t n) {
    Wire.beginTransmission(a); Wire.write(r);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)a, (int)n) != (int)n) return false;
    for (size_t i = 0; i < n; i++) b[i] = Wire.read();
    return true;
}

bool imuInit() {
    for (uint8_t a = 0x68; a <= 0x69; a++) {
        uint8_t who = 0;
        if (regRead(a, ICM_WHO_AM_I, &who, 1) && who == 0x67) { imu_addr = a; goto found; }
    }
    return false;
found:
    regWrite(ICM_GYRO_CONFIG0, 0x06);
    regWrite(ICM_ACCEL_CONFIG0, 0x06);
    regWrite(ICM_PWR_MGMT0, 0x0F);
    delay(200);
    return true;
}

/* degrees/sec about Z, bias removed.
 * Returning 0 on a failed I2C read silently UNDER-COUNTS rotation, and a
 * dropped sample mid-turn is invisible. That is almost certainly why the left
 * turn scattered 95.8 / 78.0 / 81.9 deg at coast values that should have
 * converged. Retry, hold the last good value rather than inventing a zero, and
 * count the drops so they show up in the result. */
volatile long gyro_drops = 0;
float gyro_last = 0.0f;
float gyroZ() {
    uint8_t b[6];
    for (int a = 0; a < 3; a++) {
        if (regRead(imu_addr, ICM_GYRO_DATA_X1, b, 6)) {
            int16_t z = (int16_t)((b[4] << 8) | b[5]);
            gyro_last = z * (2000.0f / 32768.0f) - gyro_bias;
            return gyro_last;
        }
    }
    gyro_drops++;
    return gyro_last;   // hold last good, do not fabricate a zero
}

/* A bias measured while the rover is moving poisons every turn in the run: one
   attempt calibrated at 18.081 dps against a normal 0.361, i.e. 50x off. At rest
   this part reads well under 1 dps, so anything above 3 is a moving rover, not a
   sensor offset. Retry rather than proceed on a number that is known bad. */
bool calibrateGyro() {
    for (int attempt = 0; attempt < 3; attempt++) {
        gyro_bias = 0.0f;
        float s = 0.0f;
        for (int i = 0; i < 120; i++) { s += gyroZ(); delay(8); }
        float b = s / 120.0f;
        if (fabsf(b) <= 3.0f) { gyro_bias = b; return true; }
        Serial.printf("  bias %.3f dps is too large - ROVER MUST BE STILL. retrying...\n", b);
        delay(2000);
    }
    return false;
}

void driveStraight(int duty, bool fwd) {
    for (int i = 0; i < 4; i++) {
        int d = (int)(duty * (fwd ? TRIM[i] : TRIM_REV[i]) + 0.5f);
        bool useB = ON_IN_B[i] == fwd;
        if (useB) { analogWrite(IN_A[i], 0); analogWrite(IN_B[i], d); }
        else      { analogWrite(IN_B[i], 0); analogWrite(IN_A[i], d); }
    }
}

/* left_fwd=true spins counter-clockwise (left side back, right side forward is
 * the OTHER way round -- here left runs forward and right runs back gives CW).
 * ccw=true: left side backward, right side forward. */
void driveTurn(int duty, bool ccw) {
    for (int i = 0; i < 4; i++) {
        bool wheelFwd = IS_RIGHT[i] ? ccw : !ccw;
        bool useB = ON_IN_B[i] == wheelFwd;
        if (useB) { analogWrite(IN_A[i], 0); analogWrite(IN_B[i], duty); }
        else      { analogWrite(IN_B[i], 0); analogWrite(IN_A[i], duty); }
    }
}

long avgAbs() {
    long long s = 0;
    for (int i = 0; i < 4; i++) {
        long long d = (long long)(enc[i].getCount() - leg_start[i]);
        s += (d < 0 ? -d : d);
    }
    return (long)(s / 4);
}
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

int runStraight(const char *name, bool fwd) {
    long target = (long)(LEG_MM * COUNTS_PER_MM) - (fwd ? COAST_FWD : COAST_REV);
    for (int i = 0; i < 4; i++) leg_start[i] = enc[i].getCount();
    unsigned long t0 = millis();
    int result = 2;
    float trip = 0.0f;

    driveStraight(KICK_DUTY, fwd);
    delay(KICK_MS);
    driveStraight(BASE_DUTY, fwd);
    while (true) {
        long got = avgAbs();
        unsigned long el = millis() - t0;
        if (got >= target) { result = 0; break; }
        if (el > FAILSAFE_MS && got < FAILSAFE_MIN_COUNTS) { result = 1; break; }
        if (got > SLIP_CHECK_AFTER) {
            float sf = slipFrac();
            if (sf > SLIP_ABORT_FRAC) { trip = sf; result = 3; break; }
        }
        if (el > HARD_MAX_MS) { result = 2; break; }
        delay(4);
    }
    allStop();
    delay(900);

    Serial.printf("\n--- %s (cut at %ld counts, coast-compensated) ---\n", name, target);
    for (int i = 0; i < 4; i++) {
        long long d = (long long)(enc[i].getCount() - leg_start[i]);
        long long m = d < 0 ? -d : d;
        Serial.printf("  %s %+8lld  %6.1f mm\n", WHEEL[i], d, (double)m / COUNTS_PER_MM);
    }
    long got = avgAbs();
    Serial.printf("  TRAVELLED %.1f mm (asked %.0f)  spread %.0f%%  ",
                  got / COUNTS_PER_MM, LEG_MM, slipFrac() * 100.0f);
    if (result == 0)      Serial.println("[ok]");
    else if (result == 1) Serial.println("[FAILSAFE - not moving]");
    else if (result == 3) Serial.printf("[SLIP %.0f%% at trip]\n", trip * 100.0f);
    else                  Serial.println("[timeout]");
    return result;
}

void runTurn(const char *name, float deg) {
    bool ccw = deg > 0;
    float target = fabsf(deg) - (ccw ? TURN_COAST_L_DEG : TURN_COAST_R_DEG);
    if (target < 8.0f) target = 8.0f;   // never cut so early there is no motion
    float acc = 0.0f;
    unsigned long t0 = millis();
    unsigned long last = micros();

    driveTurn(TURN_DUTY, ccw);
    while (acc < target) {
        unsigned long now = micros();
        float dt = (now - last) / 1000000.0f;
        last = now;
        if (dt > 0 && dt < 0.5f) acc += fabsf(gyroZ()) * dt;
        if (millis() - t0 > HARD_MAX_MS) break;
        delay(2);
    }
    allStop();
    // keep integrating through the coast-down, then report the TRUTH
    t0 = millis(); last = micros();
    while (millis() - t0 < 700) {
        unsigned long now = micros();
        float dt = (now - last) / 1000000.0f;
        last = now;
        if (dt > 0 && dt < 0.5f) acc += fabsf(gyroZ()) * dt;
        delay(2);
    }
    Serial.printf("\n--- %s ---\n  asked %+.0f deg, gyro measured %+.1f deg  (err %+.1f)  i2c drops %ld\n",
                  name, deg, ccw ? acc : -acc, (ccw ? acc : -acc) - deg, gyro_drops);
}

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < 4; i++) { pinMode(IN_A[i], OUTPUT); pinMode(IN_B[i], OUTPUT); }
    allStop();
    Wire.begin(40, 39);
    Wire.setClock(400000);
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    for (int i = 0; i < 4; i++) { enc[i].attachFullQuad(ENC_A[i], ENC_B[i]); enc[i].clearCount(); }

    delay(2500);
    imu_ok = imuInit();
    Serial.println();
    Serial.printf("=== MOVES === IMU %s   BATTERY %.2f V\n",
                  imu_ok ? "OK" : "NOT FOUND (turns blind)", batteryV());
    if (imu_ok) {
        Serial.println("calibrating gyro - HOLD STILL...");
        if (!calibrateGyro()) {
            Serial.println("*** GYRO BIAS UNRELIABLE - turns would be meaningless. Aborting. ***");
            allStop();
            return;
        }
        Serial.printf("gyro bias %.3f dps [good]\n", gyro_bias);
    }
    delay(2500);

    if (runStraight("FORWARD 800mm", true) == 1) { allStop(); return; }
    delay(1500);
    runStraight("BACK 800mm", false);
    delay(1500);
    if (imu_ok) {
        runTurn("TURN 90 LEFT", 90.0f);
        delay(1500);
        runTurn("TURN 90 RIGHT", -90.0f);
    }
    allStop();
    Serial.println("\n=== END - motors off and staying off ===");
}

void loop() { allStop(); delay(1000); }
