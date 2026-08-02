/*
 * odometry.h -- dead reckoning from the wheel encoders.
 *
 * ==================================================================
 *  THE TWIST SIGN IS FIXED HERE. READ THIS BEFORE CHANGING ANYTHING.
 * ==================================================================
 *
 * The firmware this replaces published an /odom_raw whose
 * twist.linear.x was SIGN-INVERTED with respect to its own
 * pose.position. Measured on this rover, wheels off the ground, with
 * nothing else publishing /cmd_vel:
 *
 *      commanded linear.x   twist.linear.x     pose displacement
 *            +0.012            mean -0.842        +1.395  (forward)
 *            +0.100            mean -1.225        +3.505  (forward)
 *            -0.012            mean +0.574        -1.506  (backward)
 *
 * The whole host stack works around it (ODOM_TWIST_SIGN = -1 appears in
 * fpms_teleop.py, fpms_odom_tf.py and deadband_sweep.py) and NAV2_BRIEF.md
 * codifies the workaround as "TRUST POSE. NEVER TRUST TWIST."
 *
 * Here the two cannot disagree, because they are not computed separately:
 * one per-step wheel displacement produces BOTH the pose increment and the
 * reported velocity. If x advances, d_centre was positive, so twist.linear.x
 * is positive. There is no second code path to get out of step.
 *
 * THIS IS A BREAKING CHANGE FOR THE HOST STACK. Every ODOM_TWIST_SIGN
 * must become +1 in the same deployment that flashes this firmware.
 * See README, "Breaking changes".
 */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;        /* metres, odom frame          */
    float y;        /* metres                      */
    float theta;    /* radians, CCW positive       */
    float vx;       /* m/s, body frame, FORWARD POSITIVE, consistent with x */
    float wz;       /* rad/s, CCW positive         */
} odom_state_t;

void odometry_init(void);

/* One integration step. Call from the control task immediately after
 * car_motion_step(), which is what refreshes the wheel deltas. */
void odometry_step(void);

/* Consistent snapshot for the publisher task. */
void odometry_get(odom_state_t *out);

/* Zero the pose (not the encoders). */
void odometry_reset(void);

#ifdef __cplusplus
}
#endif
#endif /* ODOMETRY_H */
