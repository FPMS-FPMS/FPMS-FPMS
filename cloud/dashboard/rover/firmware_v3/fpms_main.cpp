/* ===========================================================================
 * FPMS wildfire-monitoring rover -- ESP32-S3 firmware V3 ("duty-first")
 * Yahboom MicroROS Board V2.0 | 4WD skid steer | micro-ROS -> ROS 2 Humble
 *
 * Built on linorobot2_hardware (hippo5329 fork). This file REPLACES upstream's
 * firmware/src/firmware.cpp; it does not modify it. Everything under
 * firmware/lib/ (Motor, Encoder, Kinematics, Odometry, IMU, pwm, led) is used
 * unchanged, because that code already works on this exact board.
 *
 * ---------------------------------------------------------------------------
 * WHAT THIS BOARD IS
 * ---------------------------------------------------------------------------
 * A DUMB DUTY AMPLIFIER with honest sensors. The Pi decides how fast to go and
 * when to stop; the board applies the percentage it is told, and reports what
 * the encoders actually did. All the intelligence -- planning, the LiDAR
 * obstacle guard, heading trim -- lives on the Pi, because that is where the
 * LiDAR is. A control loop on the board cannot see an obstacle.
 *
 * This mirrors the golden B8B/phase6 driver, which achieved 0.3% distance
 * error on this hardware with exactly this model: constant-duty burst, stop,
 * measure the encoders AT REST, correct from the measurement, repeat.
 *
 * ---------------------------------------------------------------------------
 * TOPIC TABLE -- this is the complete interface. Nothing else is listened to.
 * ---------------------------------------------------------------------------
 * SUBSCRIBE (Pi -> board)
 *   /cmd_duty         std_msgs/Int32MultiArray  data[4] = -100..100 PERCENT
 *                     PRIMARY ACTUATION PATH. Order: [front-left, front-right,
 *                     rear-left, rear-right]. Bypasses kinematics and PID
 *                     entirely. Clamped to +/-FPMS_DUTY_LIMIT_PCT. Integers, so
 *                     NaN/inf cannot exist on this path by construction.
 *                     Must be re-sent at >3.3 Hz or the deadman zeroes it.
 *   /cmd_vel          geometry_msgs/Twist       velocity + PID path.
 *                     DISARMED AT BOOT. Ignored until /cmd_enable true.
 *   /cmd_enable       std_msgs/Bool             true = arm the /cmd_vel path.
 *                     Never latches: re-disarms on boot, on link loss, and on
 *                     estop. Does NOT gate /cmd_duty.
 *   /estop            std_msgs/Bool             true = latch all motors to
 *                     zero. Stays latched until an explicit false. Overrides
 *                     everything, on every path.
 *   /reset_encoders   std_msgs/Bool             any message zeroes the four
 *                     published tick counters (and /odom_raw pose).
 *   /servo_s1         std_msgs/Int32            0..180 deg, -1 = release.
 *   /servo_s2         std_msgs/Int32            (see SERVO note -- OFF by
 *                     default, LEDC channel budget)
 *   /beep             std_msgs/Int32            ms to sound, 0 = off, capped.
 *                     (compiled only if FPMS_BUZZER_PIN is defined)
 *
 * PUBLISH (board -> Pi)
 *   /wheel_ticks      std_msgs/Int32MultiArray  data[4] RAW cumulative counts,
 *                     25 Hz. THE measurement B8B's method depends on. Lets the
 *                     Pi re-derive counts/mm WITHOUT a reflash -- which matters
 *                     because that constant is still contested.
 *   /wheel_duty       std_msgs/Int32MultiArray  data[4] duty ACTUALLY APPLIED,
 *                     25 Hz. Post-clamp, post-deadman, post-estop. If you
 *                     command 26 and this reads 0, the deadman is firing.
 *   /odom_raw         nav_msgs/Odometry         10 Hz, integrated on board,
 *                     CORRECT SIGN (see report: unset ODOM_POSE_SIGN on Pi).
 *   /imu              sensor_msgs/Imu           20 Hz, real ICM42670P gyro.
 *   /battery          sensor_msgs/BatteryState  1 Hz, .voltage in VOLTS.
 *   /fpms_health      std_msgs/Int32MultiArray  2 Hz, 12 fields, layout in
 *                     publishHealth() below.
 *
 * ---------------------------------------------------------------------------
 * THE SAFETY RULES THIS FILE ENFORCES (all of them cost real hardware once)
 * ---------------------------------------------------------------------------
 *  1. applyMotors() is the ONLY function in this file that touches a motor.
 *     Nothing else calls spin(). One writer, no exceptions.
 *  2. applyMotors() is called from loop() on EVERY iteration -- not only from
 *     the micro-ROS timer. If the agent dies, the timer stops firing; a deadman
 *     that lived only in the timer would never fire either, and the last duty
 *     would stay latched on the pins forever. This is the bug that let an
 *     earlier build keep driving after the host was shut down.
 *  3. Power-on, reset, reconnect and estop are all ZERO duty.
 *  4. No motion sequence is ever started by the firmware. There is no
 *     self-test that spins wheels on its own: a bounded sequence the operator
 *     did not initiate is exactly the "motion that outlives attention" that is
 *     forbidden. The on-blocks verification is driven from the Pi with
 *     /cmd_duty, which is safer and tests the real path.
 *  5. loop() contains the link state machine and applyMotors(). Nothing else.
 *     No delays, no blocking, no motion decisions.
 * =========================================================================== */

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <i2cdetect.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/type_utilities.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

/* Same include list as upstream firmware.cpp, deliberately. That file is known
 * to compile in this tree; deviating from its includes is how you discover a
 * missing transitive dependency at 2am. The wifi/ota/lidar helpers compile to
 * no-ops when no WIFI macros are defined, which is our case (serial transport). */
#include "config.h"
#include "syslog.h"
#include "led.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "wifis.h"
#include "ota.h"
#include "pwm.h"

/* ===========================================================================
 * SERVOS -- READ BEFORE ENABLING.
 *
 * The ESP32-S3 has EIGHT LEDC channels (SOC_LEDC_CHANNEL_NUM = 8, low-speed
 * group only). This drivetrain is dual-PWM: 4 motors x 2 pins = EIGHT PWM
 * channels. The motors already consume every channel on the chip.
 *
 * So a servo cannot simply be attached: the allocation either fails silently
 * or -- far worse -- reuses a channel that is currently driving a wheel. That
 * is a wheel that stops responding, or one that runs at the servo's duty. On a
 * rover that has already run into a wall, that is not an acceptable surprise.
 *
 * The code below is complete and correct; it is switched OFF because the
 * channel budget says it cannot work as-is. To actually get servos, one of:
 *   (a) drive them from the RMT peripheral instead (ESP32-S3 has 4 TX
 *       channels, unused here) -- the clean fix;
 *   (b) put the payload servo on a separate driver board;
 *   (c) if the payload only needs ON/OFF (a spray pump relay, say), use a
 *       plain digitalWrite GPIO and no PWM channel at all -- usually the right
 *       answer for this mission.
 * Do not just flip this to 1 and hope. Verify with a build + an on-blocks
 * test that all four wheels still respond afterwards.
 * =========================================================================== */
#define FPMS_ENABLE_SERVOS 0

#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif

#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }
#endif

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

#define HZ_TO_MS(hz) (1000 / (hz))

/* ---------------------------------------------------------------------------
 * micro-ROS entities
 * ------------------------------------------------------------------------- */
rcl_publisher_t   odom_publisher, imu_publisher, battery_publisher;
rcl_publisher_t   ticks_publisher, duty_echo_publisher, health_publisher;
rcl_subscription_t cmd_duty_subscriber, estop_subscriber, enable_subscriber;
rcl_subscription_t reset_enc_subscriber;
#if FPMS_ENABLE_CMD_VEL
rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;
#endif
#if FPMS_ENABLE_SERVOS
rcl_subscription_t servo1_subscriber, servo2_subscriber;
std_msgs__msg__Int32 servo1_msg, servo2_msg;
#endif
#ifdef FPMS_BUZZER_PIN
rcl_subscription_t beep_subscriber;
std_msgs__msg__Int32 beep_msg;
#endif

nav_msgs__msg__Odometry          odom_msg;
sensor_msgs__msg__Imu            imu_msg;
sensor_msgs__msg__BatteryState   battery_msg;
std_msgs__msg__Int32MultiArray   cmd_duty_msg;      /* incoming  */
std_msgs__msg__Int32MultiArray   ticks_msg;         /* outgoing  */
std_msgs__msg__Int32MultiArray   duty_echo_msg;     /* outgoing  */
std_msgs__msg__Int32MultiArray   health_msg;        /* outgoing  */
std_msgs__msg__Bool              estop_msg, enable_msg, reset_enc_msg;

/* Publisher payload buffers. These are STATIC and pre-attached to the message
 * structs before any publish, so micro-ROS never has to allocate at runtime.
 * Dynamic allocation inside a real-time publish path on a 512 KB part is how
 * you get an allocation failure three minutes into a mission. */
static int32_t ticks_buf[4];
static int32_t duty_echo_buf[4];
static int32_t health_buf[12];

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     control_timer;

unsigned long long time_offset = 0;

enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

/* ---------------------------------------------------------------------------
 * Hardware objects. Constructed exactly as upstream constructs them.
 * ------------------------------------------------------------------------- */
Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

#if FPMS_ENABLE_CMD_VEL
PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
#endif

Kinematics kinematics(Kinematics::LINO_BASE, MOTOR_MAX_RPM, MAX_RPM_RATIO,
                      MOTOR_OPERATING_VOLTAGE, MOTOR_POWER_MAX_VOLTAGE,
                      WHEEL_DIAMETER, LR_WHEELS_DISTANCE);
Odometry odometry;
IMU imu;

/* ---------------------------------------------------------------------------
 * SAFETY / COMMAND STATE.
 * `volatile` because these are written from micro-ROS callbacks and read by
 * applyMotors() from loop().
 * ------------------------------------------------------------------------- */
enum ActivePath { PATH_STOPPED = 0, PATH_DUTY = 1, PATH_VELOCITY = 2 };

static volatile int32_t  g_duty_cmd[4]      = {0, 0, 0, 0};   /* percent      */
static volatile unsigned long g_duty_cmd_ms = 0;
static volatile bool     g_have_duty_cmd    = false;

static volatile unsigned long g_vel_cmd_ms  = 0;
static volatile bool     g_have_vel_cmd     = false;

/* ESTOP IS LATCHING. Once true it stays true until an explicit `false`
 * arrives. A momentary estop that self-clears is not an estop. */
static volatile bool     g_estop_latched    = false;

/* The velocity path is armed only by an explicit /cmd_enable true, and is
 * disarmed on boot, on every reconnect, and by estop. It NEVER survives a link
 * loss -- that is deliberate: after a dropout the Pi's intent is unknown. */
static volatile bool     g_vel_armed        = (FPMS_VEL_ARMED_AT_BOOT != 0);

static int32_t  g_duty_applied[4] = {0, 0, 0, 0};
static uint8_t  g_active_path     = PATH_STOPPED;
static bool     g_deadman_firing  = true;   /* true until a first command     */

/* Tick offsets for /reset_encoders. We subtract an offset rather than calling
 * Encoder::write(0): the Encoder object also feeds getRPM() and the velocity
 * PID, and yanking its internal position mid-flight would inject a spurious
 * velocity spike. Offsetting only the PUBLISHED number is side-effect free. */
static int32_t  g_tick_offset[4] = {0, 0, 0, 0};

static unsigned long g_prev_odom_update = 0;
static bool     g_imu_ok = false;

/* Rate meters. Counted over a window and RECOMPUTED at publish time, never
 * cached. If nothing happened in the window the answer is 0.0 Hz, not the last
 * good value. This project has been fooled before by a frozen "11 Hz" that was
 * a stale number from a link that had been dead for minutes. A health field
 * that lies is worse than no health field. */
static uint32_t g_duty_rx_count = 0, g_control_count = 0;
static unsigned long g_rate_window_start = 0;
static int32_t  g_duty_hz_x10 = 0, g_control_hz_x10 = 0;

/* Encoder liveness: ticks compared against a snapshot ~2 s old. */
static int32_t  g_enc_snapshot[4] = {0, 0, 0, 0};
static unsigned long g_enc_snapshot_ms = 0;
static uint8_t  g_enc_alive_mask = 0;

void flashLED(int n_times)
{
    for (int i = 0; i < n_times; i++) { setLed(HIGH); delay(150); setLed(LOW); delay(150); }
    delay(1000);
}

/* ===========================================================================
 * THE MOTOR PATH -- the only code in this file allowed to move the rover.
 * =========================================================================== */

/* Percent (-100..100) -> raw PWM counts, with the clamp applied FIRST so a
 * hostile or buggy value cannot overflow the multiply. There is deliberately
 * NO minimum/dead-zone term: the vendor firmware's 200-of-400 dead zone made
 * 50% duty the smallest output that existed and ~227 mm the smallest possible
 * move. Duty passes through untouched. Never "help" the motors here. */
static inline int dutyPctToPwm(int32_t pct)
{
    if (pct >  FPMS_DUTY_LIMIT_PCT) pct =  FPMS_DUTY_LIMIT_PCT;
    if (pct < -FPMS_DUTY_LIMIT_PCT) pct = -FPMS_DUTY_LIMIT_PCT;
    /* PWM_MAX is a double expression (pow(2,PWM_BITS)-1). Round rather than
     * truncate so +1% and -1% are symmetric. */
    return (int)lround((double)pct * ((double)PWM_MAX) / 100.0);
}

/* THE SINGLE WRITER.
 *
 * Called from loop() every iteration AND from the control timer. Everything
 * that could zero the motors is evaluated here, in priority order, so there is
 * exactly one place to read to know what the wheels can possibly be doing.
 *
 * Order matters: estop beats deadman beats arming beats the command itself. */
static void applyMotors()
{
    const unsigned long now = millis();
    /* Work in RAW PWM COUNTS, not percent. The duty path converts percent ->
     * counts once; the velocity path produces counts directly from the PID.
     * An earlier draft round-tripped the PID output through percent so both
     * paths shared a unit -- which quantised the PID to 1% (~10 counts) steps
     * and would have made the crawl the whole firmware exists to enable
     * jerkier on the velocity path than on the duty path. Percent is a
     * REPORTING unit only. */
    int32_t pwm_out[4] = {0, 0, 0, 0};
    int32_t pct_out[4] = {0, 0, 0, 0};
    uint8_t path = PATH_STOPPED;

    /* 1. ESTOP -- latching, beats everything. */
    if (g_estop_latched)
    {
        g_deadman_firing = true;
    }
    else
    {
        /* 2. DEADMAN on the duty path. Note the `g_have_duty_cmd` guard: at
         *    boot millis() is small and (now - 0) would be < 300, which would
         *    read as "a fresh command" for the first 300 ms. The command is
         *    zero at boot so nothing would move, but relying on that is the
         *    kind of accident that survives until it doesn't. Explicit flag. */
        const bool duty_fresh = g_have_duty_cmd &&
                                (now - g_duty_cmd_ms) < FPMS_CMD_TIMEOUT_MS;
#if FPMS_ENABLE_CMD_VEL
        const bool vel_fresh  = g_vel_armed && g_have_vel_cmd &&
                                (now - g_vel_cmd_ms) < FPMS_CMD_TIMEOUT_MS;
#else
        const bool vel_fresh  = false;
#endif

        if (duty_fresh)
        {
            /* Duty wins if both are fresh. It is the primary path and the one
             * the mission executor uses; a stray teleop must never be able to
             * override an active mission command. */
            path = PATH_DUTY;
            for (int i = 0; i < 4; i++)
            {
                pct_out[i] = g_duty_cmd[i];          /* already clamped on rx */
                pwm_out[i] = dutyPctToPwm(pct_out[i]);
            }
        }
#if FPMS_ENABLE_CMD_VEL
        else if (vel_fresh)
        {
            path = PATH_VELOCITY;
            /* The runaway path, kept because the operator asked for it, armed
             * only by explicit request. It is a genuine feedback loop: if
             * MOTORn_ENCODER_INV is wrong for any wheel, the error GROWS as it
             * corrects and that wheel saturates at full duty -- on a command
             * AND on a hand push. Verify polarity with the on-blocks test
             * before ever sending /cmd_enable true. */
            Kinematics::rpm req = kinematics.getRPM(twist_msg.linear.x,
                                                   twist_msg.linear.y,
                                                   twist_msg.angular.z);
            pwm_out[0] = motor1_pid.compute(req.motor1, motor1_encoder.getRPM());
            pwm_out[1] = motor2_pid.compute(req.motor2, motor2_encoder.getRPM());
            pwm_out[2] = motor3_pid.compute(req.motor3, motor3_encoder.getRPM());
            pwm_out[3] = motor4_pid.compute(req.motor4, motor4_encoder.getRPM());
            /* Percent is derived only for /wheel_duty, so the operator reads
             * one unit on both paths. The PID keeps full counts resolution. */
            for (int i = 0; i < 4; i++)
                pct_out[i] = (int32_t)lround(pwm_out[i] * 100.0 / ((double)PWM_MAX));
        }
#endif
        g_deadman_firing = (path == PATH_STOPPED);
    }

    /* 3. Drive. spin() applies MOTORn_INV internally and routes 0 to brake(),
     *    which on this dual-PWM topology writes both pins low = COAST. Coast
     *    (not active brake) is correct: every constant the golden driver tuned,
     *    above all its 0.93 turn-coast factor, is calibrated against coast. */
    const int32_t pwm_ceiling = (int32_t)lround(
        (double)FPMS_DUTY_LIMIT_PCT * ((double)PWM_MAX) / 100.0);
    for (int i = 0; i < 4; i++)
    {
        /* Final saturation, applied to whatever produced the value. The PID in
         * particular can and does hand back its own clamp limits. Saturate --
         * never let an out-of-range value wrap into the opposite direction. */
        if (pwm_out[i] >  pwm_ceiling) pwm_out[i] =  pwm_ceiling;
        if (pwm_out[i] < -pwm_ceiling) pwm_out[i] = -pwm_ceiling;
        if (pct_out[i] >  FPMS_DUTY_LIMIT_PCT) pct_out[i] =  FPMS_DUTY_LIMIT_PCT;
        if (pct_out[i] < -FPMS_DUTY_LIMIT_PCT) pct_out[i] = -FPMS_DUTY_LIMIT_PCT;
        g_duty_applied[i] = pct_out[i];
    }
    g_active_path = path;

    motor1_controller.spin((int)pwm_out[0]);
    motor2_controller.spin((int)pwm_out[1]);
    motor3_controller.spin((int)pwm_out[2]);
    motor4_controller.spin((int)pwm_out[3]);

    /* LED: solid when stopped, blinking while actuating. A wheels-off-the-
     * ground operator can see at a glance whether the board thinks it is
     * driving, without looking at a laptop. */
    setLed(path == PATH_STOPPED ? HIGH : !getLed());
}

/* Hard stop used on disconnect, on entity teardown, and at boot. Clears the
 * command state as well as the pins -- otherwise a stale "fresh" timestamp
 * could re-apply the old duty on the very next applyMotors(). */
static void fullStop()
{
    for (int i = 0; i < 4; i++) g_duty_cmd[i] = 0;
    g_have_duty_cmd = false;
    g_have_vel_cmd  = false;
    g_vel_armed     = false;      /* arming NEVER survives a link loss */
    g_duty_applied[0] = g_duty_applied[1] = g_duty_applied[2] = g_duty_applied[3] = 0;
    g_active_path   = PATH_STOPPED;
    g_deadman_firing = true;
    motor1_controller.spin(0);
    motor2_controller.spin(0);
    motor3_controller.spin(0);
    motor4_controller.spin(0);
    setLed(HIGH);
}

/* ===========================================================================
 * ODOMETRY / SENSORS
 * =========================================================================== */

static inline int32_t wheelTicks(int i)
{
    switch (i) {
        case 0: return motor1_encoder.read() - g_tick_offset[0];
        case 1: return motor2_encoder.read() - g_tick_offset[1];
        case 2: return motor3_encoder.read() - g_tick_offset[2];
        default:return motor4_encoder.read() - g_tick_offset[3];
    }
}

/* Integrated on-board from wheel RPM, exactly as upstream does.
 *
 * SIGN: with MOTORn_ENCODER_INV set as measured, a wheel that is physically
 * moving the rover FORWARD produces a POSITIVE rpm, so linear_x is positive
 * and x integrates POSITIVE. That is the opposite of the vendor firmware,
 * which integrated backwards (confirmed 5/5 against operator observation) and
 * which the Pi compensates for with FPMS_MISSION_ODOM_POSE_SIGN = -1.
 * >>> THAT COMPENSATION MUST BE SET TO +1 WHEN THIS FIRMWARE IS FLASHED. <<<
 * If it is left at -1 the rover will drive away from every target. */
static void updateOdometry()
{
    Kinematics::velocities vel = kinematics.getVelocities(
        motor1_encoder.getRPM(), motor2_encoder.getRPM(),
        motor3_encoder.getRPM(), motor4_encoder.getRPM());

    const unsigned long now = millis();
    float dt = (now - g_prev_odom_update) / 1000.0f;
    g_prev_odom_update = now;
    if (dt <= 0.0f || dt > 1.0f) return;   /* first call / stall: don't integrate garbage */

    odometry.update(dt, vel.linear_x, vel.linear_y, vel.angular_z);
}

static bool syncTime()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true;
    if (rmw_uros_sync_session(timeout_ms) != RMW_RET_OK) return false;
    if (rmw_uros_epoch_synchronized()) {
        time_offset = rmw_uros_epoch_millis() - millis();
        return true;
    }
    return false;
}

static struct timespec getTime()
{
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec  = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

/* Provisional: the divider ratio on BATTERY_PIN is not confirmed. Calibrate
 * BATTERY_ADJUST against a multimeter before trusting the low-voltage
 * interlock. Reported in VOLTS via sensor_msgs/BatteryState (see report -- the
 * Pi currently expects std_msgs/UInt16 DECIVOLTS and must be changed). */
static float readBatteryVolts()
{
    /* Average 8 samples: the ADC sits next to four switching motor drivers. */
    uint32_t acc = 0;
    for (int i = 0; i < 8; i++) acc += analogRead(BATTERY_PIN);
    return (float)BATTERY_ADJUST(acc / 8.0);
}

/* ===========================================================================
 * SUBSCRIPTION CALLBACKS.
 * Callbacks NEVER touch the motor pins. They only record intent; applyMotors()
 * decides. That is what keeps "one writer" true even as the command surface
 * grows.
 * =========================================================================== */

void cmdDutyCallback(const void *msgin)
{
    const std_msgs__msg__Int32MultiArray *m = (const std_msgs__msg__Int32MultiArray *)msgin;

    /* A short array is a malformed command, not a partial one. Reject it
     * whole rather than applying two wheels and leaving two stale. */
    if (m == NULL || m->data.data == NULL || m->data.size < 4) return;

    for (int i = 0; i < 4; i++)
    {
        int32_t v = m->data.data[i];
        /* Saturate, never wrap. Integers, so there is no NaN/inf to reject on
         * this path -- that whole class of bug is designed out by the choice
         * of message type. */
        if (v >  FPMS_DUTY_LIMIT_PCT) v =  FPMS_DUTY_LIMIT_PCT;
        if (v < -FPMS_DUTY_LIMIT_PCT) v = -FPMS_DUTY_LIMIT_PCT;
        g_duty_cmd[i] = v;
    }
    g_duty_cmd_ms   = millis();
    g_have_duty_cmd = true;
    g_duty_rx_count++;
}

#if FPMS_ENABLE_CMD_VEL
void twistCallback(const void *msgin)
{
    (void)msgin;   /* twist_msg is filled in place by the executor */

    /* Reject non-finite values here, because unlike the duty path this one
     * carries float64 straight into the PID. A NaN setpoint propagates through
     * the integrator and never recovers. */
    if (!isfinite(twist_msg.linear.x) || !isfinite(twist_msg.linear.y) ||
        !isfinite(twist_msg.angular.z))
    {
        twist_msg.linear.x = twist_msg.linear.y = twist_msg.angular.z = 0.0;
        return;               /* deliberately do NOT refresh the timestamp */
    }
    g_vel_cmd_ms   = millis();
    g_have_vel_cmd = true;
}
#endif

void enableCallback(const void *msgin)
{
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    if (m == NULL) return;
    /* Arming is refused while estop is latched. Clearing estop is a separate,
     * deliberate act; it must not be possible to arm your way out of it. */
    g_vel_armed = m->data && !g_estop_latched;
}

void estopCallback(const void *msgin)
{
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    if (m == NULL) return;
    if (m->data)
    {
        g_estop_latched = true;
        /* Do not wait for the next applyMotors(): zero the pins now. This is
         * the one callback allowed to be impatient, and it only ever writes
         * ZERO, so it cannot start motion. */
        fullStop();
    }
    else
    {
        /* Clearing estop does NOT resume motion: g_have_duty_cmd was cleared
         * by fullStop(), so the rover stays stopped until the Pi sends a fresh
         * command. Recovery is always an explicit new command. */
        g_estop_latched = false;
    }
}

void resetEncodersCallback(const void *msgin)
{
    (void)msgin;
    g_tick_offset[0] = motor1_encoder.read();
    g_tick_offset[1] = motor2_encoder.read();
    g_tick_offset[2] = motor3_encoder.read();
    g_tick_offset[3] = motor4_encoder.read();
    for (int i = 0; i < 4; i++) { g_enc_snapshot[i] = 0; }
    /* NOTE: this zeroes the published TICKS only. /odom_raw keeps integrating
     * from wherever it was. That is deliberate and it matches how the Pi
     * already works -- teleop and missions ANCHOR on /odom_raw (they record a
     * reference pose and measure displacement from it) rather than expecting
     * the board's pose to be zero. Resetting the board's pose underneath a Pi
     * that is holding an old anchor is precisely the /odom-vs-/odom_raw frame
     * mismatch that has offset this rover's pose by metres before -- stably
     * and silently, which is the worst way to be wrong. Ticks are a raw
     * measurement and safe to re-zero; an integrated pose is not. */
}

#if FPMS_ENABLE_SERVOS
/* 50 Hz frame, 16-bit resolution: one period = 20 ms = 65536 counts, so
 * 1.0 ms = 3277 counts and 2.0 ms = 6554 counts. -1 releases the servo by
 * emitting no pulse at all (many hobby servos then go limp, which is the
 * correct resting state for a payload arm). */
static inline void writeServo(int pin, int32_t angle_deg)
{
    if (angle_deg < 0) { setPwm(pin, 0); return; }
    if (angle_deg > 180) angle_deg = 180;
    setPwm(pin, 3277 + (int)((6554 - 3277) * (angle_deg / 180.0)));
}
void servo1Callback(const void *msgin) { (void)msgin; writeServo(FPMS_SERVO1_PIN, servo1_msg.data); }
void servo2Callback(const void *msgin) { (void)msgin; writeServo(FPMS_SERVO2_PIN, servo2_msg.data); }
#endif

#ifdef FPMS_BUZZER_PIN
/* Bounded by construction: the buzzer turns itself off after at most 3 s even
 * if nothing else ever runs. A latching "on" is a thing that outlives the
 * operator's attention, and this file does not have those. */
static unsigned long g_beep_until = 0;
void beepCallback(const void *msgin)
{
    (void)msgin;
    int32_t ms = beep_msg.data;
    if (ms <= 0)      { g_beep_until = 0; digitalWrite(FPMS_BUZZER_PIN, LOW); return; }
    if (ms > 3000)    ms = 3000;
    g_beep_until = millis() + ms;
    digitalWrite(FPMS_BUZZER_PIN, HIGH);
}
static inline void serviceBuzzer()
{
    if (g_beep_until && millis() >= g_beep_until)
    { g_beep_until = 0; digitalWrite(FPMS_BUZZER_PIN, LOW); }
}
#else
static inline void serviceBuzzer() {}
#endif

/* ===========================================================================
 * PUBLISHERS
 * =========================================================================== */

static void publishTicks(const struct timespec *ts)
{
    (void)ts;
    for (int i = 0; i < 4; i++) ticks_buf[i] = wheelTicks(i);
    ticks_msg.data.size = 4;
    RCSOFTCHECK(rcl_publish(&ticks_publisher, &ticks_msg, NULL));
}

static void publishDutyEcho()
{
    for (int i = 0; i < 4; i++) duty_echo_buf[i] = g_duty_applied[i];
    duty_echo_msg.data.size = 4;
    RCSOFTCHECK(rcl_publish(&duty_echo_publisher, &duty_echo_msg, NULL));
}

/* Recompute rate meters over the elapsed window. Called just before health is
 * published. Rates read ZERO when nothing arrived -- never a cached value. */
static void refreshRates()
{
    const unsigned long now = millis();
    unsigned long elapsed = now - g_rate_window_start;
    if (elapsed < 250) return;              /* too short to be meaningful */
    g_duty_hz_x10    = (int32_t)((g_duty_rx_count * 10000UL) / elapsed);
    g_control_hz_x10 = (int32_t)((g_control_count * 10000UL) / elapsed);
    g_duty_rx_count = g_control_count = 0;
    g_rate_window_start = now;

    /* Encoder liveness: a wheel counts as alive if its tick count CHANGED
     * since the last snapshot. Note this reads "not alive" for a stationary
     * rover, which is correct and honest -- it means "this encoder has been
     * observed to move recently", not "this encoder is plugged in". */
    if (now - g_enc_snapshot_ms >= 2000)
    {
        g_enc_alive_mask = 0;
        for (int i = 0; i < 4; i++)
        {
            int32_t t = wheelTicks(i);
            if (t != g_enc_snapshot[i]) g_enc_alive_mask |= (1 << i);
            g_enc_snapshot[i] = t;
        }
        g_enc_snapshot_ms = now;
    }
}

/* /fpms_health -- 12 int32 fields. Layout (KEEP THIS TABLE IN SYNC):
 *   [0]  firmware version           (FPMS_FW_VERSION, e.g. 30001 = 3.00.01)
 *   [1]  flags bitfield             bit0 estop_latched
 *                                   bit1 velocity path ARMED
 *                                   bit2 deadman currently zeroing the motors
 *                                   bit3 IMU init OK
 *                                   bit4 agent connected
 *                                   bit5 /cmd_vel compiled in
 *                                   bit6 servos compiled in
 *   [2]  active path                0 stopped, 1 duty, 2 velocity
 *   [3]  ms since last /cmd_duty    capped 60000; 60000 means "never/stale"
 *   [4]  ms since last /cmd_vel     capped 60000
 *   [5]  /cmd_duty receive rate     Hz x10, freshly computed, 0 when stale
 *   [6]  control loop rate          Hz x10, freshly computed, 0 when stale
 *   [7]  encoder-moved bitmask      bit per wheel, changed in last ~2 s
 *   [8]  battery millivolts
 *   [9]  IMU WHO_AM_I diagnostic    (0x67 | addr<<8 | readok<<16)
 *   [10] free heap, KB
 *   [11] uptime, seconds
 */
static void publishHealth()
{
    refreshRates();
    const unsigned long now = millis();

    int32_t flags = 0;
    if (g_estop_latched)   flags |= (1 << 0);
    if (g_vel_armed)       flags |= (1 << 1);
    if (g_deadman_firing)  flags |= (1 << 2);
    if (g_imu_ok)          flags |= (1 << 3);
    if (state == AGENT_CONNECTED) flags |= (1 << 4);
#if FPMS_ENABLE_CMD_VEL
    flags |= (1 << 5);
#endif
#if FPMS_ENABLE_SERVOS
    flags |= (1 << 6);
#endif

    unsigned long since_duty = g_have_duty_cmd ? (now - g_duty_cmd_ms) : 60000UL;
    unsigned long since_vel  = g_have_vel_cmd  ? (now - g_vel_cmd_ms)  : 60000UL;
    if (since_duty > 60000UL) since_duty = 60000UL;
    if (since_vel  > 60000UL) since_vel  = 60000UL;

    health_buf[0]  = FPMS_FW_VERSION;
    health_buf[1]  = flags;
    health_buf[2]  = g_active_path;
    health_buf[3]  = (int32_t)since_duty;
    health_buf[4]  = (int32_t)since_vel;
    health_buf[5]  = g_duty_hz_x10;
    health_buf[6]  = g_control_hz_x10;
    health_buf[7]  = g_enc_alive_mask;
    health_buf[8]  = (int32_t)(readBatteryVolts() * 1000.0f);
    health_buf[9]  = (int32_t)fpms_diag_who;
    health_buf[10] = (int32_t)(ESP.getFreeHeap() / 1024);
    health_buf[11] = (int32_t)(now / 1000);
    health_msg.data.size = 12;
    RCSOFTCHECK(rcl_publish(&health_publisher, &health_msg, NULL));
}

/* The micro-ROS control timer. Actuation happens here AND in loop(); telemetry
 * happens only here, because telemetry is pointless without an agent. */
void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;

    g_control_count++;
    applyMotors();
    updateOdometry();

    struct timespec ts = getTime();

    EXECUTE_EVERY_N_MS(HZ_TO_MS(FPMS_TICKS_HZ),     publishTicks(&ts); );
    EXECUTE_EVERY_N_MS(HZ_TO_MS(FPMS_DUTY_ECHO_HZ), publishDutyEcho(); );

    EXECUTE_EVERY_N_MS(HZ_TO_MS(FPMS_ODOM_HZ), {
        odom_msg = odometry.getData();
        odom_msg.header.stamp.sec     = ts.tv_sec;
        odom_msg.header.stamp.nanosec = ts.tv_nsec;
        RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    });

    EXECUTE_EVERY_N_MS(HZ_TO_MS(FPMS_IMU_HZ), {
        imu_msg = imu.getData();
        imu_msg.header.stamp.sec     = ts.tv_sec;
        imu_msg.header.stamp.nanosec = ts.tv_nsec;
        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    });

    EXECUTE_EVERY_N_MS(HZ_TO_MS(FPMS_BATTERY_HZ), {
        battery_msg.voltage = readBatteryVolts();
        battery_msg.present = true;
        battery_msg.header.stamp.sec     = ts.tv_sec;
        battery_msg.header.stamp.nanosec = ts.tv_nsec;
        RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
    });

    EXECUTE_EVERY_N_MS(HZ_TO_MS(FPMS_HEALTH_HZ), publishHealth(); );
}

/* ===========================================================================
 * ENTITY LIFECYCLE (autoconnect)
 * =========================================================================== */

static void attachStaticArray(std_msgs__msg__Int32MultiArray *m, int32_t *buf, size_t cap)
{
    m->data.data     = buf;
    m->data.size     = 0;
    m->data.capacity = cap;
    /* Empty layout. rclpy's Int32MultiArray() default has an empty dim list,
     * so nothing ever needs to deserialise into it. */
    m->layout.dim.data     = NULL;
    m->layout.dim.size     = 0;
    m->layout.dim.capacity = 0;
    m->layout.data_offset  = 0;
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    /* DOMAIN ID must be set on the CLIENT. See fpms_config.h -- without this
     * the board joins domain 0, the agent logs a flawless session, and every
     * topic reads "Publisher count: 0". This is the single most confusing
     * failure mode in the whole stack, so it is done first and explicitly. */
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, FPMS_ROS_DOMAIN_ID));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    /* rcl_init() COPIES the options into the context, and rclc_support_fini()
     * does not free this local copy. Without this fini we would leak a little
     * heap on every reconnect -- and this board is expected to reconnect
     * repeatedly through a competition day, so "a little, every time" is the
     * shape of an out-of-memory failure late in the afternoon. */
    RCSOFTCHECK(rcl_init_options_fini(&init_options));

    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

    /* -- publishers -- */
    RCCHECK(rclc_publisher_init_default(&odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), TOPIC_PREFIX "odom_raw"));
    RCCHECK(rclc_publisher_init_default(&imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), TOPIC_PREFIX "imu"));
    RCCHECK(rclc_publisher_init_default(&battery_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), TOPIC_PREFIX "battery"));
    RCCHECK(rclc_publisher_init_default(&ticks_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), TOPIC_PREFIX "wheel_ticks"));
    RCCHECK(rclc_publisher_init_default(&duty_echo_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), TOPIC_PREFIX "wheel_duty"));
    RCCHECK(rclc_publisher_init_default(&health_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), TOPIC_PREFIX "fpms_health"));

    /* -- subscribers -- */
    RCCHECK(rclc_subscription_init_default(&cmd_duty_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), TOPIC_PREFIX "cmd_duty"));
    RCCHECK(rclc_subscription_init_default(&estop_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_PREFIX "estop"));
    RCCHECK(rclc_subscription_init_default(&enable_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_PREFIX "cmd_enable"));
    RCCHECK(rclc_subscription_init_default(&reset_enc_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), TOPIC_PREFIX "reset_encoders"));
#if FPMS_ENABLE_CMD_VEL
    RCCHECK(rclc_subscription_init_default(&twist_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), TOPIC_PREFIX "cmd_vel"));
#endif
#if FPMS_ENABLE_SERVOS
    RCCHECK(rclc_subscription_init_default(&servo1_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), TOPIC_PREFIX "servo_s1"));
    RCCHECK(rclc_subscription_init_default(&servo2_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), TOPIC_PREFIX "servo_s2"));
#endif
#ifdef FPMS_BUZZER_PIN
    RCCHECK(rclc_subscription_init_default(&beep_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), TOPIC_PREFIX "beep"));
#endif

    const unsigned int control_timeout = HZ_TO_MS(FPMS_CONTROL_HZ);
#if defined(MICRO_ROS_DISTRO_HUMBLE) || defined(MICRO_ROS_DISTRO_FOXY)
    RCCHECK(rclc_timer_init_default(&control_timer, &support,
                                    RCL_MS_TO_NS(control_timeout), controlCallback));
#else
    RCCHECK(rclc_timer_init_default2(&control_timer, &support,
                                     RCL_MS_TO_NS(control_timeout), controlCallback, true));
#endif

    /* Executor handle count = subscriptions + 1 timer. Getting this number too
     * small makes rclc_executor_add_* fail at RUNTIME, not compile time, and
     * the symptom is a topic that silently never delivers. Counted explicitly. */
    size_t n_handles = 4 /* duty, estop, enable, reset */ + 1 /* timer */;
#if FPMS_ENABLE_CMD_VEL
    n_handles += 1;
#endif
#if FPMS_ENABLE_SERVOS
    n_handles += 2;
#endif
#ifdef FPMS_BUZZER_PIN
    n_handles += 1;
#endif
    RCCHECK(rclc_executor_init(&executor, &support.context, n_handles, &allocator));

    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_duty_subscriber,
            &cmd_duty_msg, &cmdDutyCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &estop_subscriber,
            &estop_msg, &estopCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &enable_subscriber,
            &enable_msg, &enableCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &reset_enc_subscriber,
            &reset_enc_msg, &resetEncodersCallback, ON_NEW_DATA));
#if FPMS_ENABLE_CMD_VEL
    RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber,
            &twist_msg, &twistCallback, ON_NEW_DATA));
#endif
#if FPMS_ENABLE_SERVOS
    RCCHECK(rclc_executor_add_subscription(&executor, &servo1_subscriber,
            &servo1_msg, &servo1Callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &servo2_subscriber,
            &servo2_msg, &servo2Callback, ON_NEW_DATA));
#endif
#ifdef FPMS_BUZZER_PIN
    RCCHECK(rclc_executor_add_subscription(&executor, &beep_subscriber,
            &beep_msg, &beepCallback, ON_NEW_DATA));
#endif
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    syncTime();

    /* A fresh session must not inherit the previous session's intent. The
     * velocity path re-disarms and any stale command is dropped. */
    fullStop();
    g_rate_window_start = millis();
    setLed(HIGH);
    return true;
}

bool destroyEntities()
{
    /* Stop the wheels BEFORE tearing anything down. If a fini call hangs, the
     * rover must already be stationary. */
    fullStop();

    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&battery_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&ticks_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&duty_echo_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&health_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&cmd_duty_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&estop_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&enable_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&reset_enc_subscriber, &node));
#if FPMS_ENABLE_CMD_VEL
    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
#endif
#if FPMS_ENABLE_SERVOS
    RCSOFTCHECK(rcl_subscription_fini(&servo1_subscriber, &node));
    RCSOFTCHECK(rcl_subscription_fini(&servo2_subscriber, &node));
#endif
#ifdef FPMS_BUZZER_PIN
    RCSOFTCHECK(rcl_subscription_fini(&beep_subscriber, &node));
#endif
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
    setLed(HIGH);
    return true;
}

/* ===========================================================================
 * setup / loop
 * =========================================================================== */

void setup()
{
    Serial.setRxBufferSize(1024);
    Serial.begin(BAUDRATE);
    initLed();
    BOARD_INIT                    /* Wire.begin(SDA_PIN, SCL_PIN) */

    initWifis();
    initOta();
    /* Upstream calls i2cdetect() here. We do NOT: it prints a scan table to
     * Serial, and on this board Serial IS the micro-ROS transport. On a warm
     * restart with the agent already running, that table is injected straight
     * into the XRCE stream. The IMU is diagnosed properly instead, through
     * /fpms_health and the imu_msg.orientation register readback. */
    initPwm();

    /* Motors are brought up and IMMEDIATELY zeroed. Power-on state is zero
     * duty, before anything else can possibly run. */
    motor1_controller.begin();
    motor2_controller.begin();
    motor3_controller.begin();
    motor4_controller.begin();
    fullStop();

#if FPMS_ENABLE_SERVOS
    setupPwm(FPMS_SERVO1_PIN, 50, 16);
    setupPwm(FPMS_SERVO2_PIN, 50, 16);
    setPwm(FPMS_SERVO1_PIN, 0);   /* no pulse = released, not centred */
    setPwm(FPMS_SERVO2_PIN, 0);
#endif
#ifdef FPMS_BUZZER_PIN
    pinMode(FPMS_BUZZER_PIN, OUTPUT);
    digitalWrite(FPMS_BUZZER_PIN, LOW);
#endif

    /* IMU failure is NOT fatal here, and that is a deliberate change from
     * upstream (which spins forever flashing an LED). The gyro is needed for
     * accurate TURNS, but the rover can still be driven, stopped and measured
     * without it -- and a board that refuses to boot cannot even tell the
     * operator why. The failure is reported on /fpms_health bit3 instead. */
    g_imu_ok = imu.init();

    analogReadResolution(12);
    battery_msg.voltage = readBatteryVolts();
    battery_msg.present = true;

    /* Static payload buffers attached once, before any publish. */
    attachStaticArray(&ticks_msg,     ticks_buf,     4);
    attachStaticArray(&duty_echo_msg, duty_echo_buf, 4);
    attachStaticArray(&health_msg,    health_buf,    12);

    /* The INCOMING array is the one that must survive arbitrary publisher
     * behaviour (a wrong-length array, an unexpected layout), so it gets
     * properly allocated storage via the micro-ROS utility rather than a bare
     * static buffer. Publishers we control; subscribers we do not. */
    static micro_ros_utilities_memory_conf_t conf = {0};
    conf.max_string_capacity = 16;
    conf.max_ros2_type_sequence_capacity = 4;
    conf.max_basic_type_sequence_capacity = 8;
    micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        &cmd_duty_msg, conf);

    g_prev_odom_update  = millis();
    g_rate_window_start = millis();
    g_enc_snapshot_ms   = millis();

    set_microros_serial_transports(Serial);
}

void loop()
{
    /* ---- SAFETY FIRST, UNCONDITIONALLY, IN EVERY LINK STATE. ----
     * This call is the reason the deadman actually works. The control timer
     * only fires while an agent is connected and the executor is spinning; if
     * the agent dies mid-burst the timer stops and, without this line, the
     * last duty would remain latched on the pins indefinitely. An earlier
     * build had exactly that bug and the rover kept driving after the host was
     * shut down. Cheap, and it cannot be skipped by any state transition. */
    applyMotors();
    serviceBuzzer();

    /* ---- AUTOCONNECT ----
     * Standard micro-ROS state machine with ping-based detection and clean
     * destroy/recreate. This is what makes an agent restart, a cable blip or a
     * host reboot recover WITHOUT a physical button press -- the Pi's
     * sequenced external reset (needed because the ESP32-S3 boots on the EN
     * RISING EDGE and a static level is not a reset) becomes unnecessary. */
    switch (state)
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500,
                state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;

        case AGENT_AVAILABLE:
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) destroyEntities();
            break;

        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200,
                state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED)
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
            break;

        case AGENT_DISCONNECTED:
            fullStop();               /* wheels stop the instant the link does */
            destroyEntities();
            state = WAITING_AGENT;
            break;

        default:
            break;
    }

    runWifis();
    runOta();
}
