/*
 * uros_params.h -- ROS 2 parameter server exposing every calibration
 * constant, so that tuning this rover NEVER needs a reflash.
 *
 * This is the single biggest quality-of-life change in the firmware.
 * The defect being fixed was a compile-time #define; finding its correct
 * replacement takes tens of iterations of "set a value, watch the wheel".
 * Tens of reflashes is a day's work. Tens of `ros2 param set` calls is
 * ten minutes, and can be scripted into an automatic sweep.
 *
 *   ros2 param list  /YB_Car_Node
 *   ros2 param get   /YB_Car_Node min_pwm_percent
 *   ros2 param set   /YB_Car_Node min_pwm_percent 8.0
 *   ros2 param set   /YB_Car_Node save_to_nvs true     # persist across reboot
 *
 * Values are validated and clamped in rover_config.c before they reach
 * anything that can move a wheel. Treat a parameter write as untrusted
 * remote input, because that is exactly what it is.
 */
#ifndef UROS_PARAMS_H
#define UROS_PARAMS_H

#include <stdbool.h>

#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc_parameter/rclc_parameter.h"

/* Create the server and seed it from the live rover_config. Returns the
 * rcl return code; a failure is logged by the caller and the firmware
 * carries on without runtime tuning rather than refusing to drive. */
rcl_ret_t uros_params_init(rclc_parameter_server_t *server,
                           rcl_node_t *node,
                           rclc_executor_t *executor);

void uros_params_fini(rclc_parameter_server_t *server, rcl_node_t *node);

/* Push the cumulative encoder counts out as parameters enc1..enc4.
 * Call at about 1 Hz. This is how wheel geometry gets calibrated: push
 * the rover a measured distance in a straight line and read the tick
 * delta (README, "Calibrating wheel geometry"). */
void uros_params_export_encoders(rclc_parameter_server_t *server);

#endif /* UROS_PARAMS_H */
