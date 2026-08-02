/*
 * uros_node.c -- the ROS 2 interface.
 *
 * DELIBERATELY BACKWARD COMPATIBLE. The host stack (fpms_teleop.py,
 * fpms_odom_tf.py, fpms_missions.py, deadband_sweep.py) already speaks
 * this exact interface, and a firmware change is risky enough without
 * also renaming things:
 *
 *   node name   /YB_Car_Node
 *   domain      20
 *   transport   serial, UART0, 921600, framed
 *   QoS         RELIABLE / VOLATILE / KEEP_LAST -- the rclc "default"
 *               initialisers, matching the host's QoSProfile exactly
 *
 *   SUBSCRIBE   /cmd_vel     geometry_msgs/Twist
 *               /beep        std_msgs/UInt16    (0 off, 1 on, >=10 = ms)
 *               /servo_s1    std_msgs/Int32     (degrees, 0..180)
 *               /servo_s2    std_msgs/Int32
 *
 *   PUBLISH     /odom_raw    nav_msgs/Odometry  10 Hz
 *               /imu         sensor_msgs/Imu    25 Hz
 *               /battery     std_msgs/UInt16    1 Hz, DECIVOLTS
 *
 * TWO INTENTIONAL DIFFERENCES FROM THE FIRMWARE THIS REPLACES:
 *
 *   1. /odom_raw twist.linear.x is now CORRECT rather than sign-inverted.
 *      See odometry.h. The host's ODOM_TWIST_SIGN = -1 must become +1.
 *
 *   2. /scan is NOT published. The old firmware published it with every
 *      range set to 0.0 -- it was measured dead, the LiDAR on this rover
 *      is wired to the host SBC, and fpms_teleop.py deliberately does not
 *      subscribe to it. Publishing a topic that is always empty is worse
 *      than not publishing it, because something eventually waits on it.
 */
#include "uros_node.h"

#include <math.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int16.h>

#include "aux_io.h"
#include "battery.h"
#include "car_motion.h"
#include "imu_icm42670.h"
#include "odometry.h"
#include "rover_config.h"
#include "uart_transport.h"
#include "uros_params.h"

#define UROS_DOMAIN_ID        20
#define UROS_NODE_NAME        "YB_Car_Node"
#define UROS_NODE_NAMESPACE   ""

#define ODOM_PERIOD_MS        100    /* 10 Hz  */
#define IMU_PERIOD_MS         40     /* 25 Hz  */
#define SLOW_PERIOD_MS        1000   /* 1 Hz   */

#define FRAME_ODOM            "odom"
#define FRAME_BASE            "base_footprint"
#define FRAME_IMU             "imu_frame"

/* 4 subscriptions + 3 timers + the parameter server's own handles. */
#define EXECUTOR_HANDLES      (4 + 3 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES)

/* ------------------------------------------------------------------ */

static rcl_allocator_t   s_allocator;
static rclc_support_t    s_support;
static rcl_node_t        s_node;
static rclc_executor_t   s_executor;

static rcl_publisher_t   s_pub_odom;
static rcl_publisher_t   s_pub_imu;
static rcl_publisher_t   s_pub_batt;

static rcl_subscription_t s_sub_cmdvel;
static rcl_subscription_t s_sub_beep;
static rcl_subscription_t s_sub_servo1;
static rcl_subscription_t s_sub_servo2;

static rcl_timer_t       s_timer_odom;
static rcl_timer_t       s_timer_imu;
static rcl_timer_t       s_timer_slow;

static rclc_parameter_server_t s_param_server;
static bool                    s_param_server_ok;

static nav_msgs__msg__Odometry   s_msg_odom;
static sensor_msgs__msg__Imu     s_msg_imu;
static std_msgs__msg__UInt16     s_msg_batt;
static geometry_msgs__msg__Twist s_msg_cmdvel;
static std_msgs__msg__UInt16     s_msg_beep;
static std_msgs__msg__Int32      s_msg_servo1;
static std_msgs__msg__Int32      s_msg_servo2;

static bool s_time_synced;

/* ------------------------------------------------------------------ *
 * helpers
 * ------------------------------------------------------------------ */

/* Point a rosidl string at a string literal. The message is never
 * finalised and the field is never written through, so no allocation and
 * no copy is needed -- the standard micro-ROS idiom for constant frame
 * ids. */
static void bind_literal(rosidl_runtime_c__String *s, const char *lit)
{
    s->data     = (char *)lit;
    s->size     = strlen(lit);
    s->capacity = s->size + 1;
}

static void stamp_now(builtin_interfaces__msg__Time *t)
{
    int64_t ns;
    if (s_time_synced) {
        ns = rmw_uros_epoch_nanos();
    } else {
        /* Not yet synced with the agent: fall back to time since boot.
         * Monotonic and correctly spaced, just not wall clock. Better
         * than a zero stamp, which TF treats as "latest" and silently
         * mis-associates. */
        ns = (int64_t)esp_timer_get_time() * 1000LL;
    }
    t->sec     = (int32_t)(ns / 1000000000LL);
    t->nanosec = (uint32_t)(ns % 1000000000LL);
}

/* ------------------------------------------------------------------ *
 * subscription callbacks
 * ------------------------------------------------------------------ */

static void cb_cmdvel(const void *msgin)
{
    const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
    /* linear.y is ignored: this is a skid-steer chassis and cannot
     * translate sideways. Silently ignoring it matches the previous
     * firmware and the host never sets it. */
    car_motion_set_cmd((float)m->linear.x, (float)m->angular.z);
}

static void cb_beep(const void *msgin)
{
    const std_msgs__msg__UInt16 *m = (const std_msgs__msg__UInt16 *)msgin;
    aux_beep_command(m->data);
}

static void cb_servo1(const void *msgin)
{
    const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;
    aux_servo_set_deg(1, m->data);
}

static void cb_servo2(const void *msgin)
{
    const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;
    aux_servo_set_deg(2, m->data);
}

/* ------------------------------------------------------------------ *
 * timer callbacks
 * ------------------------------------------------------------------ */

static void cb_timer_odom(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    odom_state_t st;
    odometry_get(&st);

    stamp_now(&s_msg_odom.header.stamp);

    s_msg_odom.pose.pose.position.x = st.x;
    s_msg_odom.pose.pose.position.y = st.y;
    s_msg_odom.pose.pose.position.z = 0.0;

    const double half = 0.5 * (double)st.theta;
    s_msg_odom.pose.pose.orientation.x = 0.0;
    s_msg_odom.pose.pose.orientation.y = 0.0;
    s_msg_odom.pose.pose.orientation.z = sin(half);
    s_msg_odom.pose.pose.orientation.w = cos(half);

    /* ===== requirement 4: the twist sign is correct here =====
     * st.vx and st.x come from the same per-step wheel displacement (see
     * odometry.c), so a forward-advancing pose ALWAYS reports a positive
     * twist.linear.x. No negation anywhere in this file. */
    s_msg_odom.twist.twist.linear.x  = st.vx;
    s_msg_odom.twist.twist.linear.y  = 0.0;
    s_msg_odom.twist.twist.linear.z  = 0.0;
    s_msg_odom.twist.twist.angular.x = 0.0;
    s_msg_odom.twist.twist.angular.y = 0.0;
    s_msg_odom.twist.twist.angular.z = st.wz;

    (void)rcl_publish(&s_pub_odom, &s_msg_odom, NULL);
}

static void cb_timer_imu(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    imu_sample_t s;
    const bool ok = icm42670_read(&s);

    stamp_now(&s_msg_imu.header.stamp);

    s_msg_imu.linear_acceleration.x = ok ? s.ax : 0.0;
    s_msg_imu.linear_acceleration.y = ok ? s.ay : 0.0;
    s_msg_imu.linear_acceleration.z = ok ? s.az : 0.0;
    s_msg_imu.angular_velocity.x    = ok ? s.gx : 0.0;
    s_msg_imu.angular_velocity.y    = ok ? s.gy : 0.0;
    s_msg_imu.angular_velocity.z    = ok ? s.gz : 0.0;

    /* Orientation is not estimated. Identity quaternion plus a leading
     * covariance of -1, which is the sensor_msgs convention for "this
     * field has no data". The host integrates angular_velocity.z itself
     * and has never used the quaternion -- keep it that way. */
    s_msg_imu.orientation.x = 0.0;
    s_msg_imu.orientation.y = 0.0;
    s_msg_imu.orientation.z = 0.0;
    s_msg_imu.orientation.w = 1.0;

    /* A dead IMU must be visible as a dead IMU, not as a rover that is
     * perfectly still. Set both ways, so a part that recovers stops
     * being reported as broken. */
    s_msg_imu.angular_velocity_covariance[0]    = ok ? 0.001 : -1.0;
    s_msg_imu.linear_acceleration_covariance[0] = ok ? 0.01  : -1.0;

    (void)rcl_publish(&s_pub_imu, &s_msg_imu, NULL);
}

static void cb_timer_slow(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    battery_sample();

    /* DECIVOLTS. The host does `volts = msg.data / 10.0`; this encoding is
     * inherited, not chosen. */
    float v = battery_volts();
    if (v < 0.0f)   v = 0.0f;
    if (v > 655.0f) v = 655.0f;
    s_msg_batt.data = (uint16_t)(v * 10.0f + 0.5f);
    (void)rcl_publish(&s_pub_batt, &s_msg_batt, NULL);

    if (s_param_server_ok) {
        uros_params_export_encoders(&s_param_server);
    }
}

/* ------------------------------------------------------------------ *
 * entity lifecycle
 * ------------------------------------------------------------------ */

static void init_messages(void)
{
    memset(&s_msg_odom,  0, sizeof(s_msg_odom));
    memset(&s_msg_imu,   0, sizeof(s_msg_imu));
    memset(&s_msg_batt,  0, sizeof(s_msg_batt));

    bind_literal(&s_msg_odom.header.frame_id, FRAME_ODOM);
    bind_literal(&s_msg_odom.child_frame_id,  FRAME_BASE);
    bind_literal(&s_msg_imu.header.frame_id,  FRAME_IMU);

    /* Diagonal covariances. Wheel odometry on a skid-steer chassis slips,
     * so these are deliberately loose; they are a hint to any downstream
     * filter, not a measurement. Yaw is the worst of them because the
     * effective track is itself a fitted constant. */
    for (int i = 0; i < 36; ++i) {
        s_msg_odom.pose.covariance[i]  = 0.0;
        s_msg_odom.twist.covariance[i] = 0.0;
    }
    s_msg_odom.pose.covariance[0]   = 0.05;   /* x     */
    s_msg_odom.pose.covariance[7]   = 0.05;   /* y     */
    s_msg_odom.pose.covariance[35]  = 0.20;   /* yaw   */
    s_msg_odom.twist.covariance[0]  = 0.05;   /* vx    */
    s_msg_odom.twist.covariance[35] = 0.20;   /* wz    */

    for (int i = 0; i < 9; ++i) {
        s_msg_imu.orientation_covariance[i]         = 0.0;
        s_msg_imu.angular_velocity_covariance[i]    = 0.0;
        s_msg_imu.linear_acceleration_covariance[i] = 0.0;
    }
    /* No orientation estimate at all. */
    s_msg_imu.orientation_covariance[0] = -1.0;
    s_msg_imu.angular_velocity_covariance[0]    = 0.001;
    s_msg_imu.angular_velocity_covariance[4]    = 0.001;
    s_msg_imu.angular_velocity_covariance[8]    = 0.001;
    s_msg_imu.linear_acceleration_covariance[0] = 0.01;
    s_msg_imu.linear_acceleration_covariance[4] = 0.01;
    s_msg_imu.linear_acceleration_covariance[8] = 0.01;
}

static bool create_entities(void)
{
    /* Zero every handle first. create_entities() can fail part way through
     * and destroy_entities() then runs over all of them; rcl's fini calls
     * are safe on a zero-initialised handle and are NOT safe on a stale one
     * left over from a previous session. */
    s_node       = rcl_get_zero_initialized_node();
    s_pub_odom   = rcl_get_zero_initialized_publisher();
    s_pub_imu    = rcl_get_zero_initialized_publisher();
    s_pub_batt   = rcl_get_zero_initialized_publisher();
    s_sub_cmdvel = rcl_get_zero_initialized_subscription();
    s_sub_beep   = rcl_get_zero_initialized_subscription();
    s_sub_servo1 = rcl_get_zero_initialized_subscription();
    s_sub_servo2 = rcl_get_zero_initialized_subscription();
    s_timer_odom = rcl_get_zero_initialized_timer();
    s_timer_imu  = rcl_get_zero_initialized_timer();
    s_timer_slow = rcl_get_zero_initialized_timer();
    s_executor   = rclc_executor_get_zero_initialized_executor();

    s_allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_options, s_allocator) != RCL_RET_OK) {
        return false;
    }
    if (rcl_init_options_set_domain_id(&init_options, UROS_DOMAIN_ID) != RCL_RET_OK) {
        (void)rcl_init_options_fini(&init_options);
        return false;
    }
    const rcl_ret_t support_rc =
        rclc_support_init_with_options(&s_support, 0, NULL, &init_options, &s_allocator);
    /* rclc copies what it needs out of init_options, so release it either
     * way rather than leaking it once per reconnection attempt. */
    (void)rcl_init_options_fini(&init_options);
    if (support_rc != RCL_RET_OK) {
        return false;
    }
    if (rclc_node_init_default(&s_node, UROS_NODE_NAME, UROS_NODE_NAMESPACE, &s_support) != RCL_RET_OK) {
        return false;
    }

    if (rclc_publisher_init_default(&s_pub_odom, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom_raw") != RCL_RET_OK) {
        return false;
    }
    if (rclc_publisher_init_default(&s_pub_imu, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu") != RCL_RET_OK) {
        return false;
    }
    if (rclc_publisher_init_default(&s_pub_batt, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), "battery") != RCL_RET_OK) {
        return false;
    }

    if (rclc_subscription_init_default(&s_sub_cmdvel, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) {
        return false;
    }
    if (rclc_subscription_init_default(&s_sub_beep, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16), "beep") != RCL_RET_OK) {
        return false;
    }
    if (rclc_subscription_init_default(&s_sub_servo1, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "servo_s1") != RCL_RET_OK) {
        return false;
    }
    if (rclc_subscription_init_default(&s_sub_servo2, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "servo_s2") != RCL_RET_OK) {
        return false;
    }

    if (rclc_timer_init_default(&s_timer_odom, &s_support,
            RCL_MS_TO_NS(ODOM_PERIOD_MS), cb_timer_odom) != RCL_RET_OK) {
        return false;
    }
    if (rclc_timer_init_default(&s_timer_imu, &s_support,
            RCL_MS_TO_NS(IMU_PERIOD_MS), cb_timer_imu) != RCL_RET_OK) {
        return false;
    }
    if (rclc_timer_init_default(&s_timer_slow, &s_support,
            RCL_MS_TO_NS(SLOW_PERIOD_MS), cb_timer_slow) != RCL_RET_OK) {
        return false;
    }

    if (rclc_executor_init(&s_executor, &s_support.context,
                           EXECUTOR_HANDLES, &s_allocator) != RCL_RET_OK) {
        return false;
    }

    if (rclc_executor_add_subscription(&s_executor, &s_sub_cmdvel, &s_msg_cmdvel,
                                       cb_cmdvel, ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }
    if (rclc_executor_add_subscription(&s_executor, &s_sub_beep, &s_msg_beep,
                                       cb_beep, ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }
    if (rclc_executor_add_subscription(&s_executor, &s_sub_servo1, &s_msg_servo1,
                                       cb_servo1, ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }
    if (rclc_executor_add_subscription(&s_executor, &s_sub_servo2, &s_msg_servo2,
                                       cb_servo2, ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }
    if (rclc_executor_add_timer(&s_executor, &s_timer_odom) != RCL_RET_OK) {
        return false;
    }
    if (rclc_executor_add_timer(&s_executor, &s_timer_imu) != RCL_RET_OK) {
        return false;
    }
    if (rclc_executor_add_timer(&s_executor, &s_timer_slow) != RCL_RET_OK) {
        return false;
    }

    /* The parameter server is a nice-to-have, not a precondition for
     * driving. If it fails to come up -- most likely because the rmw
     * service limits in app-colcon.meta were reduced -- the rover still
     * works, it just cannot be tuned without a reflash. Do not let this
     * take the node down with it. */
    s_param_server_ok = (uros_params_init(&s_param_server, &s_node, &s_executor) == RCL_RET_OK);

    /* Wall-clock sync so /odom_raw and /imu stamps line up with the rest
     * of the ROS graph. One second is generous over a 921600 baud link. */
    s_time_synced = (rmw_uros_sync_session(1000) == RMW_RET_OK);

    return true;
}

static void destroy_entities(void)
{
    if (!rcl_context_is_valid(&s_support.context)) {
        /* Nothing was ever built (or it has already been torn down).
         * Reaching into an invalid context below would fault. */
        s_param_server_ok = false;
        s_time_synced     = false;
        return;
    }

    /* Do not wait for the agent to acknowledge the teardown: by the time
     * we are here it is usually the agent that has gone away, and a
     * blocking destroy would stall the reconnection loop. */
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&s_support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

    if (s_param_server_ok) {
        uros_params_fini(&s_param_server, &s_node);
        s_param_server_ok = false;
    }

    (void)rcl_publisher_fini(&s_pub_odom, &s_node);
    (void)rcl_publisher_fini(&s_pub_imu, &s_node);
    (void)rcl_publisher_fini(&s_pub_batt, &s_node);
    (void)rcl_subscription_fini(&s_sub_cmdvel, &s_node);
    (void)rcl_subscription_fini(&s_sub_beep, &s_node);
    (void)rcl_subscription_fini(&s_sub_servo1, &s_node);
    (void)rcl_subscription_fini(&s_sub_servo2, &s_node);
    (void)rcl_timer_fini(&s_timer_odom);
    (void)rcl_timer_fini(&s_timer_imu);
    (void)rcl_timer_fini(&s_timer_slow);
    (void)rclc_executor_fini(&s_executor);
    (void)rcl_node_fini(&s_node);
    (void)rclc_support_fini(&s_support);

    s_time_synced = false;
}

/* ------------------------------------------------------------------ *
 * task
 * ------------------------------------------------------------------ */

typedef enum {
    AGENT_WAITING,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED,
} agent_state_t;

static void uros_task(void *arg)
{
    (void)arg;

    /* Custom transport rather than the component's built-in UART option,
     * because the baud rate must be 921600 to match the host agent and
     * the built-in option does not expose it. */
    rmw_uros_set_custom_transport(
        true,                 /* framing: this is a byte stream, not packets */
        NULL,
        uros_uart_open,
        uros_uart_close,
        uros_uart_write,
        uros_uart_read);

    init_messages();

    agent_state_t state = AGENT_WAITING;
    int64_t       last_ping_ms = 0;
    int64_t       last_sync_ms = 0;

    for (;;) {
        const int64_t now_ms = esp_timer_get_time() / 1000;

        switch (state) {
        case AGENT_WAITING:
            if (now_ms - last_ping_ms >= 500) {
                last_ping_ms = now_ms;
                state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                      ? AGENT_AVAILABLE : AGENT_WAITING;
            }
            break;

        case AGENT_AVAILABLE:
            state = create_entities() ? AGENT_CONNECTED : AGENT_WAITING;
            if (state == AGENT_WAITING) {
                destroy_entities();
            }
            break;

        case AGENT_CONNECTED:
            if (now_ms - last_ping_ms >= 1000) {
                last_ping_ms = now_ms;
                if (rmw_uros_ping_agent(200, 2) != RMW_RET_OK) {
                    state = AGENT_DISCONNECTED;
                    break;
                }
            }
            /* Re-sync occasionally: the ESP32's clock drifts against the
             * host's and /odom_raw stamps feed a TF buffer. */
            if (now_ms - last_sync_ms >= 60000) {
                last_sync_ms = now_ms;
                if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
                    s_time_synced = true;
                }
            }
            (void)rclc_executor_spin_some(&s_executor, RCL_MS_TO_NS(10));
            break;

        case AGENT_DISCONNECTED:
            /* The control task's watchdog has already stopped the motors
             * by now -- it does not need the agent, and that is the point.
             * Stop again explicitly so the intent is in one obvious place. */
            car_motion_stop();
            destroy_entities();
            state = AGENT_WAITING;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void uros_node_start(void)
{
    /* Pinned to core 0, away from the control loop on core 1. The
     * middleware does bursty, unpredictable work; the wheel loop must not
     * share a core with it. */
    xTaskCreatePinnedToCore(uros_task, "uros", 16384, NULL, 5, NULL, 0);
}
